#include <Arduino.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <DHT.h>

// --- Định nghĩa chân phần cứng theo diagram.json ---
#define DHTPIN        4
#define DHTTYPE       DHT22

#define PIN_PUMP      18
#define PIN_FAN       19
#define PIN_LED_PUMP  22
#define PIN_LED_FAN   23
#define PIN_FOGGER    25  // LED3 mô phỏng Máy phun sương

// Cấu hình PWM cho Máy phun sương
#define PWM_FREQ      5000
#define PWM_RES       8   // 0-255
#define PWM_CHANNEL   0   // Kênh LEDC (chỉ dùng cho core 2.x)

// --- Ngưỡng điều khiển tự động (có độ trễ để tránh bật/tắt liên tục) ---
// ĐÃ ĐỔI VAI: QUẠT chạy khi độ ẩm THẤP, BƠM chạy khi độ ẩm CAO
const float FAN_ON_BELOW   = 40.0;   // Độ ẩm < 40%  -> BẬT quạt
const float FAN_OFF_ABOVE  = 45.0;   // Độ ẩm >= 45% -> TẮT quạt
const float PUMP_ON_ABOVE  = 60.0;   // Độ ẩm > 60%  -> BẬT bơm
const float PUMP_OFF_BELOW = 55.0;   // Độ ẩm <= 55% -> TẮT bơm

// --- Cấu hình WiFi & MQTT Broker (HiveMQ) ---
const char* ssid        = "Wokwi-GUEST";
const char* password    = "";
const int   wifiChannel = 6;   // Wokwi-GUEST dùng kênh 6 -> kết nối nhanh hơn
const char* mqtt_server = "broker.hivemq.com";
const int   mqtt_port   = 1883;

// Broker HiveMQ là PUBLIC: ai dùng cùng tiền tố "farm" sẽ gửi/nhận lẫn dữ liệu với bạn.
// -> ĐỔI "farm" thành tên riêng (vd "farm_tenban_2026") và đổi PREFIX trong index.html cho GIỐNG.
#define TOPIC_PREFIX "farm"
const char* topic_sensor = TOPIC_PREFIX "/sensors/data";
const char* topic_pump   = TOPIC_PREFIX "/control/pump";    // payload: ON | OFF | AUTO
const char* topic_fan    = TOPIC_PREFIX "/control/fan";     // payload: ON | OFF | AUTO
const char* topic_status = TOPIC_PREFIX "/status/devices";  // ESP32 báo trạng thái về web

// --- Cấu trúc dữ liệu Hàng đợi (Queue) ---
struct SensorData {
  float temp;
  float hum;
};

QueueHandle_t dataQueue;   // Dữ liệu gửi MQTT (nhiều gói)
QueueHandle_t dataAlert;   // Độ ẩm mới nhất cho tác vụ điều khiển (chỉ giữ 1 giá trị)

// --- Thiết bị có 2 chế độ: AUTO / MANUAL ---
enum ControlMode : uint8_t { MODE_AUTO, MODE_MANUAL };

struct Actuator {
  const char* name;
  uint8_t     pin;
  uint8_t     ledPin;
  ControlMode mode;
  bool        on;
};

Actuator pump = {"Máy bơm", PIN_PUMP, PIN_LED_PUMP, MODE_AUTO, false};
Actuator fan  = {"Quạt",    PIN_FAN,  PIN_LED_FAN,  MODE_AUTO, false};

// --- Trạng thái máy phun sương (để báo công suất + thời gian về web) ---
struct FoggerInfo {
  volatile bool on;
  uint8_t       powerPct;     // Công suất 0-100 (%)
  uint32_t      durationMs;   // Tổng thời gian phun đã đặt
  uint32_t      endMs;        // Thời điểm millis() sẽ hết giờ phun
};
FoggerInfo fogger = {false, 0, 0, 0};

SemaphoreHandle_t devMutex;          // Bảo vệ pump/fan/fogger vì được truy cập từ 2 core
float lastHumidity = NAN;            // Độ ẩm gần nhất (dùng khi chuyển lại sang AUTO)
volatile bool statusDirty = false;   // Có thay đổi -> Task MQTT sẽ publish trạng thái

DHT dht(DHTPIN, DHTTYPE);
WiFiClient espClient;
PubSubClient client(espClient);

// --- Khai báo các Task FreeRTOS ---
void TaskMQTTTx(void *pvParameters);
void TaskReadDHT(void *pvParameters);
void TaskAlertControl(void *pvParameters);

// =================================================================
// ĐIỀU KHIỂN BƠM / QUẠT
// =================================================================

// Ghi trạng thái ra chân + LED. Chỉ gọi khi đang giữ devMutex.
void setActuator(Actuator &a, bool on) {
  a.on = on;
  digitalWrite(a.pin, on ? HIGH : LOW);
  digitalWrite(a.ledPin, on ? HIGH : LOW);
  statusDirty = true;
}

// Logic tự động theo độ ẩm. Chỉ gọi khi đang giữ devMutex.
// Thiết bị đang MANUAL thì bị bỏ qua hoàn toàn -> lệnh từ web luôn được ưu tiên.
// ĐÃ ĐỔI VAI: bơm chạy khi độ ẩm CAO, quạt chạy khi độ ẩm THẤP.
void applyAutoLogic(float h) {
  if (pump.mode == MODE_AUTO) {
    if (h > PUMP_ON_ABOVE && !pump.on) {
      setActuator(pump, true);
      Serial.printf("[Auto] Độ ẩm cao (%.1f%% > %.0f%%): TỰ ĐỘNG BẬT BƠM & LED 1\n", h, PUMP_ON_ABOVE);
    } else if (h <= PUMP_OFF_BELOW && pump.on) {
      setActuator(pump, false);
      Serial.printf("[Auto] Độ ẩm an toàn (%.1f%%): TỰ ĐỘNG TẮT BƠM & LED 1\n", h);
    }
  }

  if (fan.mode == MODE_AUTO) {
    if (h < FAN_ON_BELOW && !fan.on) {
      setActuator(fan, true);
      Serial.printf("[Auto] Độ ẩm thấp (%.1f%% < %.0f%%): TỰ ĐỘNG BẬT QUẠT & LED 2\n", h, FAN_ON_BELOW);
    } else if (h >= FAN_OFF_ABOVE && fan.on) {
      setActuator(fan, false);
      Serial.printf("[Auto] Độ ẩm ổn định (%.1f%%): TỰ ĐỘNG TẮT QUẠT & LED 2\n", h);
    }
  }
}

// Gọi từ TaskAlertControl mỗi khi có độ ẩm mới
void autoControlDevices(float h) {
  xSemaphoreTake(devMutex, portMAX_DELAY);
  lastHumidity = h;
  applyAutoLogic(h);
  xSemaphoreGive(devMutex);
}

// =================================================================
// PWM TƯƠNG THÍCH CẢ 2 PHIÊN BẢN ARDUINO-ESP32
//   core 3.x (Wokwi web, pioarduino): ledcAttach(chân, tần số, độ phân giải)
//   core 2.x (PlatformIO mặc định)  : ledcSetup + ledcAttachPin theo kênh
// =================================================================
void fogPwmBegin() {
#if defined(ESP_ARDUINO_VERSION_MAJOR) && ESP_ARDUINO_VERSION_MAJOR >= 3
  ledcAttach(PIN_FOGGER, PWM_FREQ, PWM_RES);
#else
  ledcSetup(PWM_CHANNEL, PWM_FREQ, PWM_RES);
  ledcAttachPin(PIN_FOGGER, PWM_CHANNEL);
#endif
}

void fogPwmWrite(uint32_t duty) {
#if defined(ESP_ARDUINO_VERSION_MAJOR) && ESP_ARDUINO_VERSION_MAJOR >= 3
  ledcWrite(PIN_FOGGER, duty);
#else
  ledcWrite(PWM_CHANNEL, duty);
#endif
}

// =================================================================
// ĐIỀU KHIỂN MÁY PHUN SƯƠNG (ghi PWM + lưu công suất / thời gian)
// =================================================================
// powerPct = 0 -> TẮT. powerPct > 0 -> BẬT với công suất đó trong durationMs.
void setFogger(uint8_t powerPct, uint32_t durationMs) {
  xSemaphoreTake(devMutex, portMAX_DELAY);
  fogger.on         = (powerPct > 0);
  fogger.powerPct   = powerPct;
  fogger.durationMs = durationMs;
  fogger.endMs      = millis() + durationMs;
  fogPwmWrite((uint32_t)powerPct * 255 / 100);   // 25% -> 63, 50% -> 127, 100% -> 255
  statusDirty = true;                                      // báo web ngay
  xSemaphoreGive(devMutex);
}

// Xử lý lệnh từ web: "ON" / "OFF" -> chuyển sang MANUAL; "AUTO" -> trả về tự động
void handleCommand(Actuator &a, const String &cmd) {
  xSemaphoreTake(devMutex, portMAX_DELAY);

  if (cmd == "AUTO") {
    a.mode = MODE_AUTO;
    statusDirty = true;
    Serial.printf("[Web] %s -> chế độ TỰ ĐỘNG\n", a.name);
    // Áp dụng ngay theo độ ẩm gần nhất, không cần chờ lần đọc kế tiếp
    if (!isnan(lastHumidity)) applyAutoLogic(lastHumidity);
  }
  else if (cmd == "ON" || cmd == "OFF") {
    a.mode = MODE_MANUAL;
    setActuator(a, cmd == "ON");
    Serial.printf("[Web] %s -> THỦ CÔNG: %s\n", a.name, cmd.c_str());
  }
  else {
    Serial.printf("[Web] Lệnh không hợp lệ cho %s: '%s'\n", a.name, cmd.c_str());
  }

  xSemaphoreGive(devMutex);
}

// Số giây phun còn lại theo đồng hồ của ESP32 (làm tròn lên); 0 nếu đang tắt hoặc đã hết giờ
uint32_t fogRemainingSec() {
  if (!fogger.on) return 0;
  int32_t leftMs = (int32_t)(fogger.endMs - millis());
  return leftMs > 0 ? (uint32_t)(leftMs + 999) / 1000 : 0;
}
uint32_t lastFogSecSent = 0xFFFFFFFF;   // Số giây còn lại đã gửi lần gần nhất (để gửi lại mỗi khi đổi)

// Chỉ được gọi từ TaskMQTTTx (PubSubClient không an toàn khi dùng từ nhiều task)
// log = true -> in ra Serial (dùng khi trạng thái thật sự thay đổi, không in khi chỉ cập nhật đếm ngược)
void publishStatus(bool log) {
  char buf[256];

  xSemaphoreTake(devMutex, portMAX_DELAY);

  // Thời gian phun còn lại (giây), làm tròn lên
  uint32_t remainSec = fogRemainingSec();
  lastFogSecSent = remainSec;

  snprintf(buf, sizeof(buf),
           "{\"pump\":\"%s\",\"pump_mode\":\"%s\",\"fan\":\"%s\",\"fan_mode\":\"%s\","
           "\"fogger\":\"%s\",\"fog_power\":%u,\"fog_duration\":%lu,\"fog_remaining\":%lu}",
           pump.on ? "ON" : "OFF", pump.mode == MODE_AUTO ? "AUTO" : "MANUAL",
           fan.on  ? "ON" : "OFF", fan.mode  == MODE_AUTO ? "AUTO" : "MANUAL",
           fogger.on ? "ON" : "OFF",
           (unsigned)fogger.powerPct,
           (unsigned long)(fogger.durationMs / 1000),
           (unsigned long)remainSec);
  statusDirty = false;
  xSemaphoreGive(devMutex);

  client.publish(topic_status, buf, true);   // retained: web mở sau vẫn thấy trạng thái
  if (log) Serial.printf("[Core 0 MQTT Status] -> %s\n", buf);
}

void callback(char* topic, byte* payload, unsigned int length) {
  String message = "";
  for (unsigned int i = 0; i < length; i++) {
    message += (char)payload[i];
  }
  message.trim();         // bỏ khoảng trắng / xuống dòng thừa
  message.toUpperCase();  // "on", "On", "auto" đều hợp lệ

  if (String(topic) == topic_pump) {
    handleCommand(pump, message);
  }
  else if (String(topic) == topic_fan) {
    handleCommand(fan, message);
  }
}

void setup() {
  Serial.begin(115200);
  dht.begin();

  pinMode(PIN_PUMP, OUTPUT);
  pinMode(PIN_FAN, OUTPUT);
  pinMode(PIN_LED_PUMP, OUTPUT);
  pinMode(PIN_LED_FAN, OUTPUT);

  digitalWrite(PIN_PUMP, LOW);
  digitalWrite(PIN_FAN, LOW);
  digitalWrite(PIN_LED_PUMP, LOW);
  digitalWrite(PIN_LED_FAN, LOW);

  // PWM máy phun sương: tự chọn đúng API theo phiên bản core (xem fogPwmBegin)
  fogPwmBegin();
  fogPwmWrite(0);

  devMutex  = xSemaphoreCreateMutex();
  // In lý do WiFi không kết nối được (201 = không thấy mạng, 202 = sai xác thực, 15 = hết thời gian bắt tay...)
  WiFi.onEvent([](arduino_event_id_t event, arduino_event_info_t info) {
    if (event == ARDUINO_EVENT_WIFI_STA_DISCONNECTED) {
      Serial.printf("[WiFi] Chưa vào được mạng, lý do=%d\n", info.wifi_sta_disconnected.reason);
    } else if (event == ARDUINO_EVENT_WIFI_STA_GOT_IP) {
      Serial.printf("[WiFi] Đã có IP: %s\n", WiFi.localIP().toString().c_str());
    }
  });

  dataQueue = xQueueCreate(5, sizeof(SensorData));
  dataAlert = xQueueCreate(1, sizeof(float));   // độ dài 1 để dùng xQueueOverwrite

  if (dataQueue != NULL && dataAlert != NULL && devMutex != NULL) {
    // Task 1: Truyền Data (WiFi + MQTT) -> Core 0
    xTaskCreatePinnedToCore(TaskMQTTTx, "TaskMQTTTx", 8192, NULL, 1, NULL, 0);

    // Task 2: Đọc dữ liệu cảm biến DHT22 -> Core 1
    xTaskCreatePinnedToCore(TaskReadDHT, "TaskReadDHT", 4096, NULL, 2, NULL, 1);

    // Task 3: Điều khiển tự động (bơm, quạt) & Phun sương PWM -> Core 1
    xTaskCreatePinnedToCore(TaskAlertControl, "TaskAlertControl", 4096, NULL, 2, NULL, 1);
  } else {
    Serial.println("Lỗi: không tạo được Queue/Mutex!");
  }
}

void loop() {
  vTaskDelay(pdMS_TO_TICKS(1000));
}

// =================================================================
// TASK 1: TRUYỀN DATA (CORE 0)
// =================================================================
// Tra DNS một lần rồi nhớ IP, để mỗi lần kết nối lại không phải chờ DNS (DNS trong Wokwi có lúc rất chậm)
IPAddress brokerIP;
bool brokerResolved = false;

bool resolveBroker() {
  Serial.printf("[Core 0] Đang tra DNS %s ...\n", mqtt_server);
  uint32_t t0 = millis();
  if (WiFi.hostByName(mqtt_server, brokerIP)) {
    brokerResolved = true;
    client.setServer(brokerIP, mqtt_port);
    Serial.printf("[Core 0] Broker IP: %s (mất %lu ms)\n", brokerIP.toString().c_str(), (unsigned long)(millis() - t0));
  } else {
    Serial.printf("[Core 0] Tra DNS thất bại sau %lu ms\n", (unsigned long)(millis() - t0));
  }
  return brokerResolved;
}

// Đảm bảo WiFi luôn kết nối; nếu chưa được thì thử lại sau mỗi 20 giây
// status: 0=chờ, 1=không thấy mạng, 4=kết nối thất bại, 5=mất kết nối, 6=chưa kết nối
bool ensureWiFi() {
  if (WiFi.status() == WL_CONNECTED) return true;

  static bool firstTry = true;
  static uint32_t lastTry = 0;
  if (firstTry || millis() - lastTry >= 20000) {
    // Không gọi WiFi.disconnect() trước begin(): tránh tự hủy lần kết nối đang chạy dở
    firstTry = false;
    lastTry = millis();
    Serial.printf("[Core 0] Đang kết nối WiFi \"%s\"... (status=%d)\n", ssid, WiFi.status());
    WiFi.mode(WIFI_STA);
    WiFi.begin(ssid, password, wifiChannel);
  }
  return false;
}

void TaskMQTTTx(void *pvParameters) {
  // Địa chỉ broker được gán trong resolveBroker() sau khi tra DNS
  client.setCallback(callback);
  client.setSocketTimeout(5);   // không để connect() chặn quá lâu khi mạng chập chờn
  client.setBufferSize(512);    // JSON trạng thái giờ dài hơn (thêm thông tin máy phun sương)

  SensorData rxData;
  const uint32_t RECONNECT_MS = 5000;
  uint32_t lastAttempt = millis() - RECONNECT_MS;  // cho phép thử kết nối ngay lần đầu
  bool wifiUp = false;
  bool mqttUp = false;
  int mqttFails = 0;

  for (;;) {
    // Chưa có WiFi (hoặc bị rớt giữa chừng) -> chờ và tự thử lại, không cần reset
    if (!ensureWiFi()) {
      wifiUp = false;
      vTaskDelay(pdMS_TO_TICKS(500));
      continue;
    }
    if (!wifiUp) {
      wifiUp = true;
      Serial.println("[Core 0] WiFi đã kết nối thành công!");
    }

    if (!client.connected()) {
      // Chỉ thử lại mỗi 5 giây, tránh spam broker
      if (millis() - lastAttempt >= RECONNECT_MS) {
        lastAttempt = millis();
        if (!brokerResolved && !resolveBroker()) continue;   // chưa tra được DNS, 5 giây sau thử lại
        Serial.println("[Core 0] Đang kết nối MQTT broker...");
        uint32_t t0 = millis();
        String clientId = "farm-esp32-" + String((uint32_t)esp_random(), HEX) + String((uint32_t)micros(), HEX);
        if (client.connect(clientId.c_str())) {
          client.subscribe(topic_pump);
          client.subscribe(topic_fan);
          statusDirty = true;   // báo trạng thái hiện tại ngay khi kết nối
          mqttFails = 0;
          Serial.printf("[Core 0] Đã kết nối HiveMQ Broker! (mất %lu ms)\n", (unsigned long)(millis() - t0));
        } else {
          Serial.printf("[Core 0] MQTT lỗi, rc=%d sau %lu ms. Thử lại sau 5s\n", client.state(), (unsigned long)(millis() - t0));
          if (++mqttFails >= 3) {   // IP có thể đã cũ -> tra DNS lại ở lần sau
            brokerResolved = false;
            mqttFails = 0;
          }
        }
      }
    }
    client.loop();

    // Ghi log khi mất kết nối để biết lúc nào ESP32 ngừng gửi
    if (client.connected()) {
      mqttUp = true;
    } else if (mqttUp) {
      mqttUp = false;
      Serial.printf("[Core 0] Mất kết nối MQTT, state=%d\n", client.state());
    }

    if (client.connected()) {
      // Khi máy phun sương đang chạy: gửi lại mỗi khi số giây còn lại đổi (đúng nhịp đồng hồ của ESP32)
      // để web hiển thị đúng số giây của ESP32, không phải tự đếm riêng
      bool fogRefresh = fogger.on && (fogRemainingSec() != lastFogSecSent);
      if (statusDirty || fogRefresh) {
        publishStatus(statusDirty);   // chỉ in Serial khi có thay đổi thật sự
      }
    }

    if (xQueueReceive(dataQueue, &rxData, pdMS_TO_TICKS(100)) == pdTRUE) {
      if (client.connected()) {
        char jsonBuffer[128];
        snprintf(jsonBuffer, sizeof(jsonBuffer), "{\"temp\":%.1f,\"hum\":%.1f}", rxData.temp, rxData.hum);
        bool ok = client.publish(topic_sensor, jsonBuffer);
        Serial.printf("[Core 0 MQTT Publish] %s -> %s\n", ok ? "OK" : "LỖI", jsonBuffer);
        if (!ok) client.disconnect();   // ép kết nối lại ở vòng lặp sau
      }
    }
    vTaskDelay(pdMS_TO_TICKS(50));
  }
}

// =================================================================
// TASK 2: ĐỌC SENSOR (CORE 1)
// =================================================================
void TaskReadDHT(void *pvParameters) {
  SensorData currentData;

  for (;;) {
    float h = dht.readHumidity();
    float t = dht.readTemperature();

    if (!isnan(h) && !isnan(t)) {
      currentData.temp = t;
      currentData.hum  = h;

      Serial.printf("[Core 1 Sensor] Temp: %.1f°C | Humidity: %.1f%%\n", t, h);

      // KHÔNG dùng portMAX_DELAY: nếu queue đầy thì bỏ gói cũ nhất, không để task bị treo
      if (xQueueSend(dataQueue, &currentData, 0) != pdTRUE) {
        SensorData dropped;
        xQueueReceive(dataQueue, &dropped, 0);
        xQueueSend(dataQueue, &currentData, 0);
      }

      // Queue độ dài 1: luôn ghi đè bằng giá trị độ ẩm mới nhất, không bao giờ block
      xQueueOverwrite(dataAlert, &h);
    } else {
      Serial.println("[Core 1 Sensor] Lỗi: Không đọc được dữ liệu từ DHT22!");
    }

    vTaskDelay(pdMS_TO_TICKS(2000));
  }
}

// =================================================================
// TASK 3: ĐIỀU KHIỂN TỰ ĐỘNG BƠM/QUẠT & PHUN SƯƠNG (CORE 1)
// =================================================================
// Phun sương theo độ ẩm, MỖI LẦN VÀO MỘT MỨC CHỈ PHUN 1 ĐỢT:
//   mức 1: 35% <= h < 40% -> công suất  25%, phun 10 giây
//   mức 2: 30% <= h < 35% -> công suất  50%, phun 20 giây
//   mức 3:         h < 30% -> công suất 100%, phun 30 giây
// - Vào một mức thì phun 1 đợt theo mức đó, hết giờ thì dừng, không phun lặp lại.
// - Độ ẩm tăng hoặc giảm sang MỨC KHÁC (vẫn < 40%) thì có đợt phun mới theo mức mới,
//   thay luôn đợt đang chạy (nếu có), công suất và thời gian tính lại từ đầu.
// - Độ ẩm >= 40% thì tắt phun sương và làm mới, lần thiếu ẩm sau phun lại từ đầu.
void TaskAlertControl(void *pvParameters) {
  float humidity = 0.0;
  uint8_t lastTier = 0;   // mức đã phun gần nhất (0 = độ ẩm >= 40%, chưa phun)

  const uint8_t  TIER_POWER[4]    = {0, 25, 50, 100};
  const uint32_t TIER_DURATION[4] = {0, 10000, 20000, 30000};

  for (;;) {
    // Chờ tối đa 100ms để vẫn kiểm tra được thời điểm hết giờ phun (không chặn cứng)
    if (xQueueReceive(dataAlert, &humidity, pdMS_TO_TICKS(100)) == pdTRUE) {

      // --- Bơm & Quạt: tự động theo độ ẩm (bỏ qua thiết bị đang MANUAL) ---
      autoControlDevices(humidity);

      // --- Máy phun sương ---
      uint8_t tier = 0;                         // 0 = độ ẩm >= 40%, không cần phun
      if (humidity < 30.0)      tier = 3;
      else if (humidity < 35.0) tier = 2;
      else if (humidity < 40.0) tier = 1;

      if (tier == 0) {
        // Độ ẩm đã đạt ngưỡng
        if (fogger.on) {
          setFogger(0, 0);   // đang phun dở thì tắt sớm
          Serial.printf("[Core 1 Alert] Độ ẩm %.1f%% >= 40%% -> Tắt phun sương sớm\n", humidity);
        }
        if (lastTier != 0) {
          lastTier = 0;      // làm mới: lần thiếu ẩm tiếp theo được phun lại
          Serial.println("[Core 1 Alert] Độ ẩm đã ổn định -> máy phun sương sẵn sàng cho đợt sau");
        }
      }
      else if (tier != lastTier) {
        // Độ ẩm chuyển sang mức khác (tăng hoặc giảm, vẫn < 40%)
        // -> phun đợt mới theo mức này, thay đợt đang chạy (nếu có)
        lastTier = tier;
        setFogger(TIER_POWER[tier], TIER_DURATION[tier]);
        Serial.printf("[Core 1 Alert] Độ ẩm %.1f%% (mức %u) -> BẬT phun sương 1 đợt: công suất %u%% (PWM %u/255) trong %lu giây\n",
                      humidity, (unsigned)tier, (unsigned)TIER_POWER[tier],
                      (unsigned)((uint32_t)TIER_POWER[tier] * 255 / 100),
                      (unsigned long)(TIER_DURATION[tier] / 1000));
      }
      else if (fogger.on) {
        // Đang phun: hiển thị công suất và thời gian còn lại
        int32_t leftMs = (int32_t)(fogger.endMs - millis());
        if (leftMs < 0) leftMs = 0;
        Serial.printf("[Core 1 Alert] Đang phun sương: công suất %u%% | còn %lu/%lu giây\n",
                      (unsigned)fogger.powerPct,
                      (unsigned long)((leftMs + 999) / 1000),
                      (unsigned long)(fogger.durationMs / 1000));
      }
      // else: vẫn ở mức này và đã phun xong 1 đợt -> không phun lại
    }

    // Hết thời gian phun -> tắt và KHÔNG phun lại ở mức này (so sánh kiểu này an toàn khi millis() bị tràn số)
    if (fogger.on && (int32_t)(millis() - fogger.endMs) >= 0) {
      Serial.printf("[Core 1 Alert] Đã phun xong 1 đợt (%lu giây, công suất %u%%) -> Tắt. Chỉ phun lại khi độ ẩm chuyển sang mức khác\n",
                    (unsigned long)(fogger.durationMs / 1000), (unsigned)fogger.powerPct);
      setFogger(0, 0);
    }
  }
}

#include "secrets.h"

#include <WiFi.h>

extern "C"
{
#include "freertos/FreeRTOS.h"
#include "freertos/timers.h"
}
#include <AsyncMQTT_ESP32.h>
#include <FastLED.h>

#define MQTT_HOST         "192.168.1.112"        // Broker address
#define MQTT_PORT         1883

const char *PubTopic  = "solar_assistant/total/battery_state_of_charge/state";               // Topic to publish

class MQTT_data {
  char *name;
  char *topic;
  int max_value;
  CRGB color;
  int current_value;
  public:
  MQTT_data(char *top, char *n, CRGB c, int mv, int cv = 0) {
    topic = top;
    name = n;
    color = c;
    max_value = mv;
    current_value = cv;
  };

  char* get_topic() {
    return topic;
  }
  char* get_name() {
    return name;
  }
  int get_value() {
    return current_value;
  }
  void set_value(int v) {
    current_value = v;
  }
  int get_max_value() {
    return max_value;
  }
  int get_percentage() {
    return current_value * 100 / max_value;
  }
  CRGB get_color() {
    return color;
  }
};

int NUMBER_OF_STATISTICS = 4;
MQTT_data data[4] = {
  MQTT_data("solar_assistant/total/battery_state_of_charge/state", 100, "Battery %", CRGB::Green),
  MQTT_data("solar_assistant/total/pv_power/state", 6000, "Solar %", CRGB::Yellow),
  MQTT_data("solar_assistant/total/load_percentage/state", 100, "Load %", CRGB::Blue),
  MQTT_data("solar_assistant/total/grid_power/state", 2000, "Grid %", CRGB::Red)
};


AsyncMqttClient mqttClient;
TimerHandle_t mqttReconnectTimer;
TimerHandle_t wifiReconnectTimer;

int payloadValue;
int lastMillis, currentMillis;

FASTLED_USING_NAMESPACE

#define DATA_PIN    2
#define LED_TYPE    WS2812B
#define COLOR_ORDER GRB
#define NUM_LEDS    11
CRGB leds[NUM_LEDS];

#define BRIGHTNESS          2
#define FRAMES_PER_SECOND  30
void connectToWifi()
{
  Serial.println("Connecting to Wi-Fi...");
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
}

void connectToMqtt()
{
  Serial.println("Connecting to MQTT...");
  mqttClient.connect();
}

void WiFiEvent(WiFiEvent_t event)
{
  switch (event)
  {
#if USING_CORE_ESP32_CORE_V200_PLUS

    case ARDUINO_EVENT_WIFI_READY:
      Serial.println("WiFi ready");
      break;

    case ARDUINO_EVENT_WIFI_STA_START:
      Serial.println("WiFi STA starting");
      break;

    case ARDUINO_EVENT_WIFI_STA_CONNECTED:
      Serial.println("WiFi STA connected");
      break;

    case ARDUINO_EVENT_WIFI_STA_GOT_IP6:
    case ARDUINO_EVENT_WIFI_STA_GOT_IP:
      Serial.println("WiFi connected");
      Serial.print("IP address: ");
      Serial.println(WiFi.localIP());
      connectToMqtt();
      break;

    case ARDUINO_EVENT_WIFI_STA_LOST_IP:
      Serial.println("WiFi lost IP");
      break;

    case ARDUINO_EVENT_WIFI_STA_DISCONNECTED:
      Serial.println("WiFi lost connection");
      xTimerStop(mqttReconnectTimer, 0); // ensure we don't reconnect to MQTT while reconnecting to Wi-Fi
      xTimerStart(wifiReconnectTimer, 0);
      break;
#else

    case SYSTEM_EVENT_STA_GOT_IP:
      Serial.println("WiFi connected");
      Serial.println("IP address: ");
      Serial.println(WiFi.localIP());
      connectToMqtt();
      break;

    case SYSTEM_EVENT_STA_DISCONNECTED:
      Serial.println("WiFi lost connection");
      xTimerStop(mqttReconnectTimer, 0); // ensure we don't reconnect to MQTT while reconnecting to Wi-Fi
      xTimerStart(wifiReconnectTimer, 0);
      break;
#endif

    default:
      break;
  }
}

void printSeparationLine()
{
  Serial.println("************************************************");
}

void onMqttConnect(bool sessionPresent)
{
  Serial.print("Connected to MQTT broker: ");
  Serial.print(MQTT_HOST);
  Serial.print(", port: ");
  Serial.println(MQTT_PORT);
  printSeparationLine();
  Serial.print("Session present: ");
  Serial.println(sessionPresent);

  // uint16_t packetIdSub = mqttClient.subscribe(PubTopic, 2);
  for (int i = 0; i < NUMBER_OF_STATISTICS; i++) {
    Serial.print("PubTopic: ");
    Serial.println(PubTopic);
    uint16_t packetIdSub = mqttClient.subscribe(data[i].get_topic(), 2);
    Serial.print("Subscribing at QoS 2, packetId: ");
    Serial.println(packetIdSub);
    mqttClient.publish(PubTopic, 0, true, "ESP32 Test");
    Serial.println("Publishing at QoS 0");
    printSeparationLine();
  }
}

void onMqttDisconnect(AsyncMqttClientDisconnectReason reason)
{
  (void) reason;

  Serial.println("Disconnected from MQTT.");

  if (WiFi.isConnected())
  {
    xTimerStart(mqttReconnectTimer, 0);
  }
}

void onMqttSubscribe(const uint16_t& packetId, const uint8_t& qos)
{
  Serial.println("Subscribe acknowledged.");
  Serial.print("  packetId: ");
  Serial.println(packetId);
  Serial.print("  qos: ");
  Serial.println(qos);
}

void onMqttUnsubscribe(const uint16_t& packetId)
{
  Serial.println("Unsubscribe acknowledged.");
  Serial.print("  packetId: ");
  Serial.println(packetId);
}

void onMqttMessage(char* topic, char* payload, const AsyncMqttClientMessageProperties& properties,
                   const size_t& len, const size_t& index, const size_t& total)
{
  (void) payload;
  int id;
  for (int i = 0; i < NUMBER_OF_STATISTICS; i++) {
    if (topic == data[i].get_topic()) id = i;
  }
  payloadValue = 0;
  Serial.println("Publish received.");
  Serial.print("  topic: ");
  Serial.println(topic);
  Serial.print("  len: ");
  Serial.println(len);
  Serial.print("  index: ");
  Serial.println(index);
  Serial.print("  total: ");
  Serial.println(total);
  for (int i = 0; i < len; ++i) {
    payloadValue += payload[i]-48;
    // Serial.println("Payload value [" + String(i) + "]= " + String(payloadValue));
    if(i < len - 1) payloadValue *= 10;
  }
  Serial.print("Payload: ");
  Serial.print(payloadValue);
  Serial.print(" / ");
  Serial.println(data[id].get_max_value());
  data[id].set_value(payloadValue);
  
}

void onMqttPublish(const uint16_t& packetId)
{
  Serial.println("Publish acknowledged.");
  Serial.print("  packetId: ");
  Serial.println(packetId);
}

void setup()
{
  Serial.begin(115200);

  while (!Serial && millis() < 5000);

  delay(500);

  Serial.print("\nStarting FullyFeature_ESP32 on ");
  Serial.println(ARDUINO_BOARD);
  Serial.println(ASYNC_MQTT_ESP32_VERSION);

  mqttReconnectTimer = xTimerCreate("mqttTimer", pdMS_TO_TICKS(2000), pdFALSE, (void*)0,
                                    reinterpret_cast<TimerCallbackFunction_t>(connectToMqtt));
  wifiReconnectTimer = xTimerCreate("wifiTimer", pdMS_TO_TICKS(2000), pdFALSE, (void*)0,
                                    reinterpret_cast<TimerCallbackFunction_t>(connectToWifi));

  WiFi.onEvent(WiFiEvent);

  mqttClient.onConnect(onMqttConnect);
  mqttClient.onDisconnect(onMqttDisconnect);
  mqttClient.onSubscribe(onMqttSubscribe);
  mqttClient.onUnsubscribe(onMqttUnsubscribe);
  mqttClient.onMessage(onMqttMessage);
  mqttClient.onPublish(onMqttPublish);

  mqttClient.setServer(MQTT_HOST, MQTT_PORT);

  connectToWifi();

  FastLED.addLeds<LED_TYPE,DATA_PIN,COLOR_ORDER>(leds, NUM_LEDS).setCorrection(TypicalLEDStrip);
  FastLED.setBrightness(BRIGHTNESS);
  leds[0] = CRGB::Red;
  leds[1] = CRGB::Green;
  leds[2] = CRGB::Blue;
  leds[3] = CRGB::Red;
  FastLED.show();  
  Serial.println("\nSetup end");
}
  
void loop() {
  currentMillis = millis();
  if (currentMillis - lastMillis > 5000){ 
    lastMillis = currentMillis;
    // reset LEDs by setting them to black (off)
    for (int i = 0; i < NUM_LEDS; i++) {
      leds[i] = CRGB::Black;
    }
    for (int iter = 0; iter < NUMBER_OF_STATISTICS; iter++) {
      for (int i = 0; i < NUM_LEDS; i++) {
        if (data[iter].get_percentage() / 10 / NUM_LEDS > i) {
          leds[i] = data[iter].get_color();
          Serial.print("1");
        }
        Serial.print("0");
      }
      Serial.println("------");
      Serial.print(data[iter].get_name());
      Serial.println(data[iter].get_percentage());
      Serial.println("------");
    }
    FastLED.show(); 
  }
}
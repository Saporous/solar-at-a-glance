#define _ETHERNET_WEBSERVER_LOGLEVEL_ 1
#define _ASYNC_MQTT_LOGLEVEL_ 1
#include <WebServer_WT32_ETH01.h>

extern "C"
{
#include "freertos/FreeRTOS.h"
#include "freertos/timers.h"
}

#include <AsyncMQTT_ESP32.h>

//#define MQTT_HOST         IPAddress(192, 168, 2, 110)
#define MQTT_HOST         "192.168.1.112"        // Broker address
#define MQTT_PORT         1883

const char *PubTopic  = "solar_assistant/total/battery_state_of_charge/state";               // Topic to publish

AsyncMqttClient mqttClient;
TimerHandle_t mqttReconnectTimer;

bool messageReceived;
int payloadValue;

void connectToMqtt()
{
  Serial.println("Connecting to MQTT...");
  mqttClient.connect();
}

void ETH_event(WiFiEvent_t event)
{
  switch (event)
  {
#if USING_CORE_ESP32_CORE_V200_PLUS

    case ARDUINO_EVENT_ETH_START:
      Serial.println("ETH starting");
      break;

    case ARDUINO_EVENT_ETH_CONNECTED:
      Serial.println("ETH connected");
      break;

    case ARDUINO_EVENT_ETH_GOT_IP:
      Serial.println("ETH got IP");
      Serial.print("IP address: ");
      Serial.println(ETH.localIP());
      connectToMqtt();
      break;

    case ARDUINO_EVENT_ETH_DISCONNECTED:
      Serial.println("ETH lost connection");

      // ensure we don't reconnect to MQTT when no ETH
      xTimerStop(mqttReconnectTimer, 0);

      break;

    case ARDUINO_EVENT_ETH_STOP:
      Serial.println("ETH stops");

      // ensure we don't reconnect to MQTT when no ETH
      xTimerStop(mqttReconnectTimer, 0);

      break;
#else

    case SYSTEM_EVENT_ETH_CONNECTED:
      erial.println(F("ETH Connected"));
      break;

    case SYSTEM_EVENT_ETH_GOT_IP:
      Serial.println("ETH connected");
      Serial.println("IP address: ");
      Serial.println(ETH.localIP());
      connectToMqtt();
      break;

    case SYSTEM_EVENT_ETH_DISCONNECTED:
    case SYSTEM_EVENT_ETH_STOP:
      Serial.println("ETH lost connection");

      // ensure we don't reconnect to MQTT when no ETH
      xTimerStop(mqttReconnectTimer, 0);

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
  Serial.print("PubTopic: ");
  Serial.println(PubTopic);

  printSeparationLine();
  Serial.print("Session present: ");
  Serial.println(sessionPresent);

  uint16_t packetIdSub = mqttClient.subscribe(PubTopic, 2);
  Serial.print("Subscribing at QoS 2, packetId: ");
  Serial.println(packetIdSub);

  printSeparationLine();
}

void onMqttDisconnect(AsyncMqttClientDisconnectReason reason)
{
  (void) reason;

  Serial.println("Disconnected from MQTT.");

  if (WT32_ETH01_isConnected())
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
  payloadValue = 0;
  messageReceived = 1;
  Serial.println("Publish received.");
  Serial.print("  topic: ");
  Serial.println(topic);
  Serial.print("  qos: ");
  Serial.println(properties.qos);
  Serial.print("  dup: ");
  Serial.println(properties.dup);
  Serial.print("  retain: ");
  Serial.println(properties.retain);
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
  Serial.println(payloadValue);
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
  Serial.println("Setup begin");

  while (!Serial && millis() < 5000);

  Serial.print("\nStarting FullyFeature_WT32_ETH01 on ");
  Serial.print(ARDUINO_BOARD);
  Serial.println(" with " + String(SHIELD_TYPE));
  Serial.println(WEBSERVER_WT32_ETH01_VERSION);
  Serial.println(ASYNC_MQTT_ESP32_VERSION);

  mqttReconnectTimer = xTimerCreate("mqttTimer", pdMS_TO_TICKS(2000), pdFALSE, (void*)0,
                                    reinterpret_cast<TimerCallbackFunction_t>(connectToMqtt));

  mqttClient.onConnect(onMqttConnect);
  mqttClient.onDisconnect(onMqttDisconnect);
  mqttClient.onSubscribe(onMqttSubscribe);
  mqttClient.onUnsubscribe(onMqttUnsubscribe);
  mqttClient.onMessage(onMqttMessage);
  mqttClient.onPublish(onMqttPublish);

  mqttClient.setServer(MQTT_HOST, MQTT_PORT);

  //////////////////////////////////////////////

  // To be called before ETH.begin()
  WiFi.onEvent(ETH_event);

  //bool begin(uint8_t phy_addr=ETH_PHY_ADDR, int power=ETH_PHY_POWER, int mdc=ETH_PHY_MDC, int mdio=ETH_PHY_MDIO,
  //           eth_phy_type_t type=ETH_PHY_TYPE, eth_clock_mode_t clk_mode=ETH_CLK_MODE);
  //ETH.begin(ETH_PHY_ADDR, ETH_PHY_POWER, ETH_PHY_MDC, ETH_PHY_MDIO, ETH_PHY_TYPE, ETH_CLK_MODE);
  ETH.begin(ETH_PHY_ADDR, ETH_PHY_POWER);

  // Static IP, leave without this line to get IP via DHCP
  //bool config(IPAddress local_ip, IPAddress gateway, IPAddress subnet, IPAddress dns1 = 0, IPAddress dns2 = 0);
  //ETH.config(myIP, myGW, mySN, myDNS);

  WT32_ETH01_waitForConnect();

  //////////////////////////////////////////////
  Serial.println("\nSetup end");
}

void loop()
{
}

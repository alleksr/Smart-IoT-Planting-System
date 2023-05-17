#include <Wire.h>
#include <Adafruit_SHT4x.h>
#include <SPI.h>
#include <RadioLib.h>
#include <WiFi.h>
#include <PubSubClient.h>

// Установите параметры LoRa модуля SX1262
#define LORA_SS      18
#define LORA_RST     14
#define LORA_DI0     26
#define LORA_BAND    868E6

// Установите параметры Wi-Fi сети
const char* ssid = "YourWiFiSSID";
const char* password = "YourWiFiPassword";

// Установите параметры MQTT брокера
const char* mqttServer = "your_mqtt_broker_ip";
const int mqttPort = 1883;
const char* mqttUsername = "your_mqtt_username";
const char* mqttPassword = "your_mqtt_password";
const char* mqttClientId = "your_mqtt_client_id";
const char* mqttTopic = "your_mqtt_topic";

Adafruit_SHT4x sht40 = Adafruit_SHT4x();
float temperature, humidity;

SX1262 radio = new Module(LORA_SS, LORA_RST, LORA_DI0);

WiFiClient wifiClient;
PubSubClient mqttClient(wifiClient);

void setup() {
  Serial.begin(9600);

  // Подключение к Wi-Fi сети
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.println("Connecting to WiFi...");
  }
  Serial.println("Connected to WiFi");

  // Подключение к MQTT брокеру
  mqttClient.setServer(mqttServer, mqttPort);
  if (!mqttClient.connected()) {
    connectToMqtt();
  }

  // Настройка LoRa модуля
  SPI.begin(5, 19, 27, 18);
  radio.setFrequency(LORA_BAND);

  if (!radio.beginFSK()) {
    Serial.println("LoRa initialization failed. Check your wiring.");
    while (1);
  }
  Serial.println("LoRa initialized");

  // Установка параметров MESH сети
  radio.meshInit(1, 20, 5);  // Установите идентификатор сети, мощность передатчика и количество повторов

  // Установка адреса устройства в MESH сети
  radio.meshSetNodeAddress(1);  // Установите адрес устройства в сети

  // Включение прослушивания MESH сообщений
  radio.meshStartListening();

  // Инициализация датчика SHT40
  if (!sht40.begin()) {
    Serial.println("SHT40 sensor initialization failed. Check your wiring.");
    while (1);
  }
  Serial.println("SHT40 sensor initialized");
}

void loop() {
  // Проверка наличия MESH сообщений
  if (radio.meshAvailable()) {
    uint8_t sender;
    uint8_t recipient;
    String payload;

    // Чтение MESH сообщения
    radio.meshRead(sender, recipient, payload);

    // Обработка MESH сообщения
    // ...

    // Отправка ответного MESH сообщения (если требуется)
    // ...
  }

  // Измерение температуры и влажности
  temperature = sht40.readTemperature();
  humidity = sht40.readHumidity();

  // Отправка данных на MQTT брокер
  String payload = "Temperature: " + String(temperature) + "°C, Humidity: " + String(humidity) + "%";
  mqttClient.publish(mqttTopic, payload.c_str());

  // Отправка MESH сообщения
  uint8_t recipient = 2;  // Адрес получателя в MESH сети
  payload = "Hello, recipient!";
  radio.meshSend(1, recipient, payload);

  delay(5000);  // Задержка перед повторным выполнением цикла
}

void connectToMqtt() {
  while (!mqttClient.connected()) {
    Serial.println("Connecting to MQTT...");

    if (mqttClient.connect(mqttClientId, mqttUsername, mqttPassword)) {
      Serial.println("Connected to MQTT");
    } else {
      Serial.print("MQTT connection failed, rc=");
      Serial.print(mqttClient.state());
      Serial.println(" Retrying in 5 seconds...");

      delay(5000);
    }
  }
}

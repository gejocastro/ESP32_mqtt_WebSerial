#include <WebSerial.h>
#include <ESPAsyncWebServer.h>
#include <AsyncTCP.h>


/*********
  Modificado de Rui Santos
  Complete project details at https://randomnerdtutorials.com
*********/

#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>

#include <PubSubClient.h>
#include <WiFi.h>
#include <Wire.h>
#include "constantes.hpp" //archivo con los datos de conexiòn a wifi y de IP y puerto de servidor MQTT, credenciales, etc.
//#include "funciones.hpp";

//pins para BME280 sensor
#define I2C_SDA 33
#define I2C_SCL 32
#define LED_GPIO 2

AsyncWebServer server(80);

void message(uint8_t *data, size_t len){
  WebSerial.println("Data Received!");
  String Data = "";
  for(int i=0; i < len; i++){
    Data += char(data[i]);
  }
}

// Configuracion del BME
TwoWire I2CBME = TwoWire(0);
Adafruit_BME280 bme;

WiFiClient espClient;
PubSubClient client(espClient);

byte led_control; // Nivel de iluminacion del led amarillo
bool status;
char humString[8];
char periodString[8];
char presString[8];
char tempString[8];
float temperature = 0;
float humidity = 0;
float pressure = 0;
int analog_value; // Valor de la entrada analogica
int period = 15000 ; // initial miliseconds between readings sending
int retrieve = 1; // Flag para publicar los datos de la variable analogica
long lastMsg = 0; // Inicializamos contador de millis para tarea temporizada
// char msg[50];     // no usada?
String str_led_control;  // String recibido por MQTT para control del led
String str_refresh_time = "15";

void setup_wifi() {
  delay(10);
  // We start by connecting to a WiFi network
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
}

void callback(char* topic, byte* message, unsigned int length) {
  Serial.print("Message arrived on topic: ");
  Serial.print(topic);
  Serial.println(". Message: ");
  String messageTemp;

  WebSerial.print("Message arrived on topic: ");
  WebSerial.print(topic);
  WebSerial.print(". Message: ");

  for (int i = 0; i < length; i++) {
    Serial.print((char)message[i]);
  //  WebSerial.print((char)(message[i]));
    messageTemp += (char)message[i];
  }
  WebSerial.print(String (messageTemp));
  Serial.println();
  WebSerial.println();
  // Feel free to add more if statements to control more GPIOs with MQTT
  // If a message is received on the topic esp32/output, you check if the message is either "on" or "off".
  // Changes the output state according to the message
  if (String(topic) == "esp32/output") {
    Serial.print("Setting output to ");
    WebSerial.print("Setting output to ");
    if (messageTemp == "on") {
      Serial.println("on");
      WebSerial.println("on");
      digitalWrite(ledPin, HIGH);
    }
    else if (messageTemp == "off") {
      Serial.println("off");
      WebSerial.println("off");
      digitalWrite(ledPin, LOW);
    }
  }
  // If a message is received on topic esp32/retrieve, sets retrieve to 1 (will be used to send analog data message)
  if (String(topic) == "esp32/retrieve") {
    Serial.print("Sending analog data ");
    WebSerial.print("Sending analog data ");
    retrieve = 1;
  }
  // If received on esp32/led_control, saves message in variable
  if (String(topic) == "esp32/led_control") {
      Serial.print("Setting PWM to  ");
      Serial.println(messageTemp);    
      WebSerial.print("Setting PWM to  ");
      WebSerial.println(messageTemp);

    str_led_control = messageTemp;
  }

  // If received on esp32/refresh_time, saves message in variable
  if (String(topic) == "esp32/refresh_time") {
    Serial.print("Setting refresh time to ");
    Serial.println(messageTemp);
    WebSerial.print("Setting refresh time to ");
    WebSerial.println(messageTemp);
    
    str_refresh_time = messageTemp;
  }
} //End callback

void reconnect() {
  // Loop until we're reconnected
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    // Attempt to connect
    if (client.connect("ESP32Client", mqtt_server_user, mqtt_server_password)) {
      Serial.println("connected");

      // Subscribe
      topic_subscribe();
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
} //End reconnect

// topic_subscribe(), suscribe el cliente a los topics necesarios.
void topic_subscribe() {
  client.subscribe("esp32/output");
  client.subscribe("esp32/retrieve");
  client.subscribe("esp32/led_control");
  client.subscribe("esp32/refresh_time");
}

void send_analog_value () {
  analog_value = analogRead (AnalogPin);
  char anaString [8];
  dtostrf(analog_value, 1, 2, anaString);
  Serial.print("Analog: ");
  Serial.println(anaString);
  WebSerial.print("Analog: ");
  WebSerial.println(anaString);
  client.publish("esp32/analog", anaString);
}

void send_readings () {
    // Genera variables aleatorias para simular lectura de temperatura y %humedad // Sustituir por obtencion de datos del sensor llegado el caso.
    temperature = bme.readTemperature();    //random(20, 35);
    humidity = bme.readHumidity();//random (70, 100);
    pressure = bme.readPressure();

    // Convert the value to a char array
    dtostrf(temperature, 1, 2, tempString);
    Serial.print("Temperature: ");
    Serial.println(tempString);
    WebSerial.print("Temperature: "); //pending funcion to publish to both terminals (ifpresent)
    WebSerial.println(tempString);
    // Publishes message
    client.publish("esp32/temperature", tempString);

    // Converts the value to a char array
    dtostrf(humidity, 1, 2, humString);
    Serial.print("Humidity: ");
    Serial.println(humString);
    WebSerial.print("Humidity: ");
    WebSerial.println(humString);
    // Publishes
    client.publish("esp32/humidity", humString);

    // Converts the value to a char array
    dtostrf(pressure / 100.0F, 1, 2, presString);
    Serial.print("Pressure: ");
    Serial.print(presString);
    Serial.println(" hPa");
    WebSerial.print("Pressure: ");
    WebSerial.print(presString);
    WebSerial.println(" hPa");
    // Publishes
    client.publish("esp32/pressure", presString);
    
    // Converts value to char array
    dtostrf(period, 2, 0, periodString);
    // and publishes 
    client.publish("esp32/period", periodString);
    
    // for debugging purposes
    Serial.print("period = ");
    Serial.println(period);
    WebSerial.print("period = ");
    WebSerial.println(period);
}

void setup() {
  pinMode(ledPin, OUTPUT);
  pinMode (ledPwmPin, OUTPUT);
  
  ledcSetup(ledChannel, ledChannelFreq, ledChannelResolution);
  ledcAttachPin(22, 0);

  Serial.begin(115200);

  setup_wifi();

  client.setServer(mqtt_server, mqtt_port);
  client.setCallback(callback);

  I2CBME.begin(I2C_SDA, I2C_SCL, 100000); // inicializo sensor
  // default settings
  // (you can also pass in a Wire library object like &Wire2)
  status = bme.begin(0x76, &I2CBME);
  if (!status) {
    Serial.println("Could not find a valid BME280 sensor, check wiring!");
    while (1);
  }
  else
    Serial.println ("BME280 connected successfully");

// Web Serial startup
  WebSerial.begin(&server);
  WebSerial.msgCallback(message);
  server.begin();

} // End setup

void loop() {
  if (!client.connected()) {
    reconnect();
  }
  client.loop();

  // Timer for sending data messages through MQTT
  long now = millis();
  if (now - lastMsg > period) {
    lastMsg = now;
    send_readings(); 
    send_analog_value();
  }

  if (retrieve == 1) {
    send_analog_value();
    send_readings();
    retrieve = 0;
  }

  led_control = str_led_control.toInt();
  ledcWrite(0, led_control * 2.55 );

  period = str_refresh_time.toInt() * 1000;

} //end loop

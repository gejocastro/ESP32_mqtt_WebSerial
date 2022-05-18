// Replace the next variables with your SSID/Password combination
const char* ssid = "TP-LINK_34C808";
const char* password = "n163k25s";

// Add your MQTT Broker IP address, example:
const char* mqtt_server = "3.225.83.240";
const int mqtt_port = 1883;

// Add MQTT server credentials
const char* mqtt_server_user = "user"; 
const char* mqtt_server_password = "MQTT_12$";//"password";


// 
int ledChannelFreq = 5000;
int ledChannel = 0;
int ledChannelResolution = 8;

// Pin para la lectura de la entrada analògica del esp32
const int AnalogPin = A0;

// Pins para el control del LEDs
const int ledPin = 21;
const int ledPwmPin = 22;

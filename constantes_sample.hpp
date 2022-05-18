
// Replace the next variables with your SSID/Password combination
const char* ssid = "WIFI-NETWORK-SSID";
const char* password = "wifi_network_password";

// Add your MQTT Broker IP address and port, example:
const char* mqtt_server = "3.225.83.240";
const int mqtt_port = 1883;

// Add MQTT server credentials
const char* mqtt_server_user = "mqtt_server_user"; 
const char* mqtt_server_password = "mqtt_server_user_password";


// 
int ledChannelFreq = 5000;
int ledChannel = 0;
int ledChannelResolution = 8;

// Pin para la lectura de la entrada analògica del esp32
const int AnalogPin = A0;

// Pins para el control del LEDs
const int ledPin = 21;
const int ledPwmPin = 22;

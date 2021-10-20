/*
  Basic ESP8266 MQTT example

  This sketch demonstrates the capabltes of the pubsub library in combination
  with the ESP8266 board/library.

  it connects to an MQTT server then:

  it wll reconnect to the server if the connecton s lost usng a blocking
  reconnect functon. See the 'mqtt_reconnect_nonblockng' example for how to
  acheve the same result without blockng the man loop.

  To install the ESP8266 board, (usng Arduno 1.6.4+):
  - Add the followng 3rd party board manager under "File -> Preferences -> Addtonal Boards Manager URLs":
       http://arduino.esp8266.com/stable/package_esp8266com_index.json
  - Open the "Tools -> Board -> Board Manager" and click install for the ESP8266"
  - Select your ESP8266 in "Tools -> Board"

*/
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
#include <ESP8266WiFi.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include "credentials.h"
#include "DHT.h"                     // include DHT lbrary
#include <time.h>
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//ADC_MODE(ADC_VCC); //vcc read-mode
#define VCC_ADJ 1.096
#define BATT_WARNNG_VOLTAGE 2.4 // Voltage for Low-Bat warnng
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
#define SERIAL_DEBUG 0
#ifdef SERIAL_DEBUG

#endif
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
#define RTCaddr_toggleFlag 64 //one int is 1-bucket addr. (4 - Bytes)
#define RTCaddr_unix_time 65
#define RTCaddr_index 66
#define RTCaddr_num_data_saved 67
#define RTCaddr_sensor_status 68

#define RTCMEMORYSTART 69
#define RTCMEMORYLEN 127
#define max_save_data 14 //(127-69)/4 //

extern "C" {
#include "user_interface.h" // this is for the RTC memory read/write functions
}

typedef struct {
  int battery;
  int Temp;
  int RH;
  int unix_time;
} rtcStore;

rtcStore rtcMem;

//int i;
int buckets;
int toggleFlag;   // if false than it is the first run of the program
int msg_index = 0;    // mqtt msg counter
int sensor_status = 0;
// 0 - OK
// 1 - have data in the RTC
// 2 - err read DTH sensor
int num_data_saved = 0;    // data saved n RTC
int batt = 0;
int now_int = 0;

bool WiFi_connected = 0;
bool mqtt_connected = 0;
bool ntp_connected = 0;
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

#define DHTPN D7            // DHT11 data pn s connected to ESP8266 pn GPO2
const int analognPin  = A0;    // Analog nput pn
#define led_pin 3
#define DHTTYPE DHT22       // DHT11 sensor s used
DHT dht(DHTPN, DHTTYPE);   // Confgure DHT lbrary

//float temperature = 19.0;
//float readHumidity  = 35;

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
const char* NTP_SERVER = "il.pool.ntp.org";
const char* TZ_INFO    = "IST-2IDT,M3.4.4/26,M10.5.0";  // enter your time zone (https://remotemonitoringsystems.ca/time-zone-abbreviations.php)
tm timeinfo;
time_t now;
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
WiFiClient espClient;
PubSubClient client(espClient);

#define BAUD 115200
#define MAX_MSG_SIZE 254

char msg[MAX_MSG_SIZE + 1];
byte msg_pos;
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void setup_wifi() {
  delay(10);
  // We start by connecting to a WiFi network
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);

  WiFi.persistent(false);
  WiFi.setAutoReconnect(true);
  WiFi.setSleepMode(WIFI_MODEM_SLEEP); // Default is WIFI_NONE_SLEEP
  WiFi.setOutputPower(10); // 0~20.5dBm, Default max

  //WiFi.mode(WIFI_STA);
  //WiFi.config(ip, gateway, subnet); // For Static IP
  WiFi.begin(ssid, password);

  int counter = 0;
  while ((WiFi.status() != WL_CONNECTED) && (counter < 100) ) {
    WiFi_connected = 0;
    delay(500);
    Serial.print(".");
    //if (++counter > 100) ESP.restart();
    counter = counter + 1;
  }

  if (WiFi.status() == WL_CONNECTED)  {
    WiFi_connected = 1;
    Serial.println("");
    Serial.println("WiFi connected");
    Serial.println("IP address: ");
    Serial.println(WiFi.localIP());
  }
  else {
    WiFi_connected = 0;
    Serial.println("");
    Serial.println("WiFi not connected");
  }
}

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void callback(char* topic, byte* payload, unsigned int length) {
  Serial.print("Message arrived [");
  Serial.print(topic);
  Serial.print("] ");
  for (int i = 0; i < length; i++) {
    Serial.write(payload[i]);
  }
  Serial.write('\n');
  Serial.print("payload :  ");
  Serial.println((char *)payload);
  Serial.println();

  DynamicJsonDocument doc((MAX_MSG_SIZE + 1));
  deserializeJson(doc, payload);
  double led    = doc["data"][0];
  if (led == 1) {
    digitalWrite(led_pin, HIGH);
  }
  else {
    digitalWrite(led_pin, LOW);
  }

}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

void reconnect() {
  // Loop until we're reconnected
  int mqtt_reconnect_try = 0;
  while (!client.connected()) {
    mqtt_connected = 0;
    Serial.print("Attempting MQTT connection...");
    if (client.connect(MQTT_ID, MQTT_USER, MQTT_PSWD)) {
      mqtt_reconnect_try = 0;
      Serial.println("connected");
      // Wait 5 seconds before retrying
      for (byte i = 0; i < 2; i++) delay(500); // delay(1000) may cause hang
      // Once connected, publish an announcement...
      // client.publish(MQTT_TOPIC_OUT, "hello world");
      // ... and resubscribe
      client.subscribe(MQTT_TOPIC_IN);
      mqtt_connected = 1;
    }
    else {
      mqtt_reconnect_try++;
      Serial.print("failed, rc=");
      Serial.print(client.state());
      if (mqtt_reconnect_try > 5) {
        Serial.println(" ");
        Serial.println("failed 5 times in a row");
        mqtt_connected = 0;
        break;
      }
      else {
        Serial.println(" try again in 5 seconds");
        // Wait 5 seconds before retrying
        for (byte i = 0; i < 10; i++) delay(500); // delay(5000) may cause hang
      }
    }
  }
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// setup
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void setup() {
  WiFi_connected = 0;
  mqtt_connected = 0;
  ntp_connected = 0;
  Serial.begin(BAUD, SERIAL_8N1, SERIAL_FULL);
  setup_wifi();
  client.setServer(MQTT_SERVER, 1883);
  client.setCallback(callback);
  dht.begin();                       // Initialize DHT sensor
  pinMode(led_pin, OUTPUT);

  // Initialize a NTPclient to get time
  Serial.println("Initialize a NTP");
  configTime(0, 0, NTP_SERVER);
  // See https://github.com/nayarsystems/posix_tz_db/blob/master/zones.csv for Timezone codes for your region
  setenv("TZ", TZ_INFO, 1);
  delay(100);
  if (getNTPtime(10)) {  // wait up to 10sec to sync
    ntp_connected = 1;
  } else {
    Serial.println("Time not set");
    ntp_connected = 0;
    //    delay(100);
    //    // ESP.restart();
    //    Serial.println("going to sleep to 1 minute");
    //    ESP.deepSleep(1 * 60000000, WAKE_RFCAL); //
    //    delay(100);
  }
  Serial.println("setup End");
}



//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// Loop
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
unsigned long lastMsg = 0;
#define MSG_BUFFER_SIZE  (MAX_MSG_SIZE + 1)
//int msg_index = 0;
float HCal = 17;
float TCal = 0.5;

void loop() {
  //~~~~~~~~~~~ RTC   ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
  buckets = (sizeof(rtcMem) / 4);
  if (buckets == 0) buckets = 1;
  Serial.print("Buckets ");
  Serial.println(buckets);
  //~~~~~~~~~~~ ck if it the fierst time   ~~~~~~~~~~~
  system_rtc_mem_read(RTCaddr_toggleFlag, &toggleFlag, 4);   // the toggleFlag is (int) therfore it size is 4 bytes
  Serial.print("toggle Flag ");
  Serial.println(toggleFlag);
  if (toggleFlag != 1) { //if false than it is the fierst run
    Serial.print("it is the firest run ");
    if (ntp_connected == 1) { // all good
      toggleFlag = true;
      system_rtc_mem_write(RTCaddr_toggleFlag, &toggleFlag, 4);
      msg_index = 0;
      system_rtc_mem_write(RTCaddr_index, &msg_index, 4);
      sensor_status = 0; //status s OK
      system_rtc_mem_write(RTCaddr_sensor_status, &sensor_status, 4);
      num_data_saved = 0;
      system_rtc_mem_write(RTCaddr_num_data_saved, &num_data_saved, 4);
      now_int = (int)now;
      system_rtc_mem_write(RTCaddr_unix_time , &now_int, 4);
    }
    else { // firest time and no ntp
      Serial.println("the sensor started and no NTP, going to sleep to 5 minute");
      ESP.deepSleep(5 * 60000000, WAKE_RFCAL);
    }
  }
  else { // not the first time
    system_rtc_mem_read(RTCaddr_index, &msg_index, 4);
    system_rtc_mem_read(RTCaddr_sensor_status, &sensor_status, 4);
    system_rtc_mem_read(RTCaddr_num_data_saved, &num_data_saved, 4);
    if (ntp_connected == 0) { // read the last know time from RTC
      system_rtc_mem_read(RTCaddr_unix_time , &now_int, 4);
      now = (time_t)now_int;
    }
    else {
      now_int = (int)now;
    }
  }
  //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
  if (!client.connected()) {
    reconnect();
  }
  client.loop();
  //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
  Serial.print("Sensor Status :");
  Serial.println(sensor_status);
  Serial.print("num of data saved :");
  Serial.println(num_data_saved);

  //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
  if ((sensor_status == 1) && (mqtt_connected == 1)) { //ck if there is stored data
    for (int  ii = 1;  ii <= num_data_saved; ++ii)
    {
      int rtcPos = RTCMEMORYSTART +  (ii - 1) * buckets;
      system_rtc_mem_read(rtcPos, &rtcMem, sizeof(rtcMem));
      Serial.println(rtcPos);

      make_Json(rtcMem);  // -> msg
      //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
      Serial.print("Publsh message: ");
      Serial.println(msg);
      client.publish(MQTT_TOPIC_OUT, msg);
      msg_index = msg_index + 1;
      delay(250);
    }
    num_data_saved = 0;
    sensor_status = 0;
    Serial.println("All data saved are sent");
  }
  //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
  get_sensor_data();  //-> rtcMem
  Serial.print(" mqtt connected: ");
  Serial.println(mqtt_connected);
  
  if (mqtt_connected == 1) { // all is ok
    
    make_Json(rtcMem);  // ->msg

    Serial.print("Publsh message: ");
    Serial.println(msg);
    client.publish(MQTT_TOPIC_OUT, msg);
    msg_index = msg_index + 1;
    delay(250);

    //~~~~~~
    system_rtc_mem_write(RTCaddr_index, &msg_index, 4);
    system_rtc_mem_write(RTCaddr_sensor_status, &sensor_status, 4);
    system_rtc_mem_write(RTCaddr_num_data_saved, &num_data_saved, 4);
    now_int = now_int + 5 * 60;
    system_rtc_mem_write(RTCaddr_unix_time , &now_int, 4);
    delay(10);
    Serial.println("before 5 min sleep");
    ESP.deepSleep(5 * 60000000, WAKE_RFCAL); //
    delay(100);
  }
  else {    //no mqtt therefor stor the data int RTC
    Serial.println("no mqtt service therefore going to sleep for 10 min");//half-hour");
    Serial.print("Save Data at RTC addr :");
    num_data_saved = num_data_saved + 1;

    sensor_status = 1; //data stored
    system_rtc_mem_write(RTCaddr_sensor_status, &sensor_status, 4);
    system_rtc_mem_write(RTCaddr_num_data_saved, &num_data_saved, 4);
    //system_rtc_mem_write(RTCaddr_index, &msg_index, 4);
    now_int = now_int + 10 * 60;
    system_rtc_mem_write(RTCaddr_unix_time , &now_int, 4);

    if (num_data_saved <= max_save_data) {
      int rtcPos = RTCMEMORYSTART + (num_data_saved - 1) * buckets;
      system_rtc_mem_write(rtcPos, &rtcMem, sizeof(rtcMem));
      Serial.println(rtcPos);
    }

    ESP.deepSleep(10 * 60000000, WAKE_RFCAL); //
    delay(100);
  }

  //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
}

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//bat_percentage = mapfloat(voltage, 2.8, 4.2, 0, 100); //2.8V as Battery Cut off Voltage & 4.2V as Maximum Voltage
float mapfloat(float x, float in_min, float in_max, float out_min, float out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}


//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
bool getNTPtime(int sec) {
  uint32_t start = millis();
  do {
    time(&now); // The time() function only calls the NTP server every hour. So you can always use getNTPtime()
    localtime_r(&now, &timeinfo);
    Serial.print(".");
    delay(100);
  } while (((millis() - start) <= (1000 * sec)) && (timeinfo.tm_year < (2016 - 1900)));
  if (timeinfo.tm_year <= (2016 - 1900)) {
    return false;  // the NTP call was not successful
  }
  else {
    Serial.println(" ");
    Serial.print("now ");
    Serial.println(now);
    /*char time_output[30];
      strftime(time_output, 30, "%a  %d-%m-%y %T", localtime(&now));
      Serial.println(time_output);
      Serial.println();*/
    return true;
  }
}

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void make_Json(rtcStore rtcData) {
  StaticJsonDocument < MAX_MSG_SIZE + 1 > doc; //256 is the RAM allocated to ths document.
  doc["DTH"] = 3;
  doc["Msg#"] = msg_index;
  doc["status"] = sensor_status;
  doc["time"] = rtcData.unix_time;

  // Check f any reads faled and ext early (to try agan)
  JsonArray data = doc.createNestedArray("data");
  data.add((float)rtcData.Temp / 10);
  data.add((float)rtcData.RH / 10);

  //float Vcc = (float)ESP.getVcc() * VCC_ADJ;
  //float Vcc_ = (int)(Vcc * 100 + 0.5);
  //data.add( (float)Vcc_ / 100 );
  data.add(rtcData.battery);

  int b = serializeJson(doc, msg); // Generate the mnfed JSON and send it to the msg char array
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

void get_sensor_data() {
  Serial.println("Read sensor data");
  float h = dht.readHumidity();
  float RH = h - HCal; //I noticed the readings needed calibrating so I have an offset that I can changed, the result being the value I send for use

  //String RH_str;
  float t = dht.readTemperature(); //take a Temperature reading and place in the float variable 't'
  float Temp = t - TCal; //again I noticed the readings needed calibrating so I have an offset that I can change, the result being the value I send for use

  //    if (isnan(h) || isnan(t)) {
  //      sensor_status = 3;
  //    }
  //    else {
  //      sensor_status = 0;
  //    }
  // doc["status"] = sensor_status;

  rtcMem.Temp = (int)(Temp * 10);
  rtcMem.RH = (int)(RH * 10);
  batt = analogRead(analognPin);
  rtcMem.battery = batt;
  rtcMem.unix_time = now_int;
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// getSupplyVoltage
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
float getSupplyVoltage()
{
  //int rawLevel = analogRead(A0);

  //float realVoltage = (float)rawLevel / 1000 / (10000. / (47000 + 10000));
  //float realVoltage = (float)rawLevel / 1023 * 3.3;

  int sensorValue = analogRead(analognPin);
  delay(50);
  float realVoltage = (float)(sensorValue * 3.3) / 1024 * 32 / 22; //multiply by two as voltage divider network is 10K & 22K Resistor
  float Voltage = (int)(realVoltage * 100 + 0.5);
  return (float)Voltage / 100;
}

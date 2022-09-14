/*
-------------------------------------FATİH FURKAN ERSOY-------------------------------------
---------------------------------------SCOOTER V9-------------------------------------
-----------------------------------------MİCROZERR-----------------------------------------
*/

#define TINY_GSM_MODEM_SIM800 // SIM MODULE

// All LIBRARY
#include <TinyGPS++.h>      //GPS
#include <ESP32Servo.h>     // Servo
#include <HardwareSerial.h> //TX RX
#include <TinyGsmClient.h>  //SIM800L
#include <PubSubClient.h>   //MQTT
#include <ArduinoJson.h>    //JSON
#include <Wire.h>           //I2C
#include <EEPROM.h>

// For OTA
#include <WiFi.h>
#include <WiFiClient.h>
#include <WebServer.h>
#include <ESPmDNS.h>
#include <Update.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>

// OTA ALL DEFINE
//--------------------------------------------------------------------------------------------
const char *host = "RACOONSCOOTER";
const char *ssid = "RACOON";
const char *password = "9UDW&yckLrR5Jc4^";
WebServer server(80);

//--------------------------------------------------------------------------------------------
// SERVER AND LOGIN PAGE
//--------------------------------------------------------------------------------------------
/*
 * Login page
 */
const char *loginIndex =
    "<form name='loginForm'>"
    "<table width='20%' bgcolor='A09F9F' align='center'>"
    "<tr>"
    "<td colspan=2>"
    "<center><font size=4><b>RACOON UPDATE VERSION Login Page</b></font></center>"
    "<br>"
    "</td>"
    "<br>"
    "<br>"
    "</tr>"
    "<td>Username:</td>"
    "<td><input type='text' size=25 name='userid'><br></td>"
    "</tr>"
    "<br>"
    "<br>"
    "<tr>"
    "<td>Password:</td>"
    "<td><input type='Password' size=25 name='pwd'><br></td>"
    "<br>"
    "<br>"
    "</tr>"
    "<tr>"
    "<td><input type='submit' onclick='check(this.form)' value='Login'></td>"
    "</tr>"
    "</table>"
    "</form>"
    "<script>"
    "function check(form)"
    "{"
    "if(form.userid.value=='microzerr' && form.pwd.value=='3AVzQV3vuxfkske%mQ5"
    "{"
    "window.open('/serverIndex')"
    "}"
    "else"
    "{"
    " alert('Error Password or Username')/*displays error message*/"
    "}"
    "}"
    "</script>";

/*
 * Server Index Page
 */

const char *serverIndex =
    "<script src='https://ajax.googleapis.com/ajax/libs/jquery/3.2.1/jquery.min.js'></script>"
    "<form method='POST' action='#' enctype='multipart/form-data' id='upload_form'>"
    "<input type='file' name='update'>"
    "<input type='submit' value='Update'>"
    "</form>"
    "<div id='prg'>progress: 0%</div>"
    "<script>"
    "$('form').submit(function(e){"
    "e.preventDefault();"
    "var form = $('#upload_form')[0];"
    "var data = new FormData(form);"
    " $.ajax({"
    "url: '/update',"
    "type: 'POST',"
    "data: data,"
    "contentType: false,"
    "processData:false,"
    "xhr: function() {"
    "var xhr = new window.XMLHttpRequest();"
    "xhr.upload.addEventListener('progress', function(evt) {"
    "if (evt.lengthComputable) {"
    "var per = evt.loaded / evt.total;"
    "$('#prg').html('progress: ' + Math.round(per*100) + '%');"
    "}"
    "}, false);"
    "return xhr;"
    "},"
    "success:function(d, s) {"
    "console.log('success!')"
    "},"
    "error: function (a, b, c) {"
    "}"
    "});"
    "});"
    "</script>";
// END
//--------------------------------------------------------------------------------------------
//--------------------------------------------------------------------------------------------

//--------------------------------------------------------------------------------------------
HardwareSerial SerialGPS(2); // TX RX pins for GPS

//--------------------------------------------------------------------------------------------
Servo myservo; // Servo KILIT
int pos = 0;

//--------------------------------------------------------------------------------------------
TinyGPSPlus gps;

//--------------------------------------------------------------------------------------------
#define SerialMon Serial // UART 1 PC - Device -- Monitor
#define SerialAT Serial1 // UART 2 SIM800L - Device -- GSM

#define GSM_PIN ""             // GSM pin
const char apn[] = "internet"; // GSM apn (operatore gore degisir)
const char gprsUser[] = "";
const char gprsPass[] = "";

//--------------------------------------------------------------------------------------------
// I2C for SIM800 (to keep it running when powered from battery)
TwoWire I2CPower = TwoWire(0);
#define IP5306_ADDR 0x75
#define IP5306_REG_SYS_CTL0 0x00

//--------------------------------------------------------------------------------------------
// MQTT ALL CONFIG
const char *broker = "raccoonscooter.com";     // MQTT hostname
int MQTT_PORT = 1883;                          // MQTT Port
const char *mqttUsername = "raccoon";          // MQTT username
const char *mqttPassword = "Ze255Wer29tete/-"; // MQTT password

String IMEImqttID = "IMEI ERRORRR"; // MQTTID icin
char MQTTID[40];

//--------------------------------------------------------------------------------------------
// Debug
#ifdef DUMP_AT_COMMANDS
#include <StreamDebugger.h>
StreamDebugger debugger(SerialAT, SerialMon);
TinyGsm modem(debugger);
#else
TinyGsm modem(SerialAT);
#endif

//--------------------------------------------------------------------------------------------
// GSM start and integration mqtt
TinyGsmClient client(modem);
PubSubClient mqtt(client);

//--------------------------------------------------------------------------------------------
// Sim modemi kurulumu
#define MODEM_RST 5
#define MODEM_PWKEY 4
#define MODEM_POWER_ON 23
#define MODEM_TX 27
#define MODEM_RX 26
#define I2C_SDA 21
#define I2C_SCL 22

//--------------------------------------------------------------------------------------------
// send msg time
uint32_t lastReconnectAttempt = 0;
long lastMsg = 0;

//--------------------------------------------------------------------------------------------
// Scooter Data
int lock_data;                      // Start - Stop ( Scooter and Lock)
String IMEISENDDATA = "IMEI ERROR"; // IMEI'yi cihazdan almak ve topic'e ekleme yapmak icin
char datasendtopic[50];             // Verilein gonderildiği topic IMEI 'nın char'a donusturulmesi PUBLISH
String IMEI = "IMEI ERROR";         // IMEI'yi cihazdan almak icin
char datataketopic[30];             // Verilein alindigi topic IMEI 'nın char'a donusturulmesi SUBSCRIBE
// BATTERY DATA READ ADC AND CONVERT
float total = 0;
float average = 0;
float okunandeger[100];
float deger = 0;
float yuzde = -1;

int servo_status = 0;
int servo_otuz_status = 0;

int period = 30000;
unsigned long time_now = 0;
unsigned long buton_one_time_now = 0;
int buton_one_period = 29000;

int BUZZER_period = 5000;
unsigned long BUZZER_time_now = 0;
int buzzer_status = 0;

int mqttconnecttryjustone = 0;
//--------------------------------------------------------------------------------------------
// MICROZERR PCB POWER PIN CONFIG
#define RELAY_SCOOTER_STATUS 33
#define BUTTON_LOCK 14
#define BUZZER 25

//--------------------------------------------------------------------------------------------
// watchdog
#include <esp_task_wdt.h>
// 3 seconds WDT
#define WDT_TIMEOUT 800

//--------------------------------------------------------------------------------------------
// eeprom
#define EEPROM_SIZE 12
#define EE_ADRESS 0
int SCOOTER_STATE;
//--------------------------------------------------------------------------------------------
//--------------------------------------------------------------------------------------------
// SETUP
//--------------------------------------------------------------------------------------------
//--------------------------------------------------------------------------------------------
void setup()
{
  //--------------------------------------------------------------------------------------------
  EEPROM.begin(EEPROM_SIZE);
  SerialMon.begin(115200);              // SERIAL UART 1 PC - Device -- Monitor
  esp_task_wdt_init(WDT_TIMEOUT, true); // enable panic so ESP32 restarts
  esp_task_wdt_add(NULL);               // add current thread to WDT watc
  // Serial.println("OTA DENEME");
  //--------------------------------------------------------------------------------------------
  // WIFI CONNECT FOR OTA
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  for (int i = 0; i <= 10000; i++)
  { // döngü başladı
    if (WiFi.waitForConnectResult() != WL_CONNECTED)
    {
      Serial.print(".");
    }
    else
    {
      Serial.println("---------------------------");
      Serial.println("WIFI CONNECTED");
    }
  }
  Serial.println();
  Serial.println("");
  Serial.print("Connected to ");
  Serial.println(ssid);
  Serial.print("Connected with IP: ");
  Serial.println(WiFi.localIP());
  Serial.println();

  //--------------------------------------------------------------------------------------------
  //--------------------------------------------------------------------------------------------
  // FOR WEB ARDUİNO OTA
  //--------------------------------------------------------------------------------------------
  //--------------------------------------------------------------------------------------------
  /*use mdns for host name resolution*/
  if (!MDNS.begin(host))
  { // http://esp32.local
    Serial.println("Error setting up MDNS responder!");
  }
  Serial.println("mDNS responder started");
  /*return index page which is stored in serverIndex */
  server.on("/", HTTP_GET, []()
            {
    server.sendHeader("Connection", "close");
    server.send(200, "text/html", loginIndex); });
  server.on("/serverIndex", HTTP_GET, []()
            {
    server.sendHeader("Connection", "close");
    server.send(200, "text/html", serverIndex); });
  /*handling uploading firmware file */
  server.on(
      "/update", HTTP_POST, []()
      {
    server.sendHeader("Connection", "close");
    server.send(200, "text/plain", (Update.hasError()) ? "FAIL" : "OK");
    ESP.restart(); },
      []()
      {
        HTTPUpload &upload = server.upload();
        if (upload.status == UPLOAD_FILE_START)
        {
          Serial.printf("Update: %s\n", upload.filename.c_str());
          if (!Update.begin(UPDATE_SIZE_UNKNOWN))
          { // start with max available size
            Update.printError(Serial);
          }
        }
        else if (upload.status == UPLOAD_FILE_WRITE)
        {
          /* flashing firmware to ESP*/
          if (Update.write(upload.buf, upload.currentSize) != upload.currentSize)
          {
            Update.printError(Serial);
          }
        }
        else if (upload.status == UPLOAD_FILE_END)
        {
          if (Update.end(true))
          { // true to set the size to the current progress
            Serial.printf("Update Success: %u\nRebooting...\n", upload.totalSize);
          }
          else
          {
            Update.printError(Serial);
          }
        }
      });
  server.begin();
  //------------------------------------
  // FOR IDE ARDUİNO OTA
  // Port defaults to 3232
  // ArduinoOTA.setPort(3232);

  // Hostname defaults to esp3232-[MAC]
  // ArduinoOTA.setHostname("myesp32");

  // No authentication by default
  ArduinoOTA.setPassword("3AVzQV3vuxfkske%mQ5");

  ArduinoOTA
      .onStart([]()
               {
      String type;
      if (ArduinoOTA.getCommand() == U_FLASH)
        type = "sketch";
      else // U_SPIFFS
        type = "filesystem";
      // NOTE: if updating SPIFFS this would be the place to unmount SPIFFS using SPIFFS.end()
      Serial.println("Start updating " + type); })
      .onEnd([]()
             { Serial.println("\nEnd"); })
      .onProgress([](unsigned int progress, unsigned int total)
                  { Serial.printf("Progress: %u%%\r", (progress / (total / 100))); })
      .onError([](ota_error_t error)
               {
      Serial.printf("Error[%u]: ", error);
      if (error == OTA_AUTH_ERROR) Serial.println("Auth Failed");
      else if (error == OTA_BEGIN_ERROR) Serial.println("Begin Failed");
      else if (error == OTA_CONNECT_ERROR) Serial.println("Connect Failed");
      else if (error == OTA_RECEIVE_ERROR) Serial.println("Receive Failed");
      else if (error == OTA_END_ERROR) Serial.println("End Failed"); });
  ArduinoOTA.begin();

  //--------------------------------------------------------------------------------------------
  //--------------------------------------------------------------------------------------------
  // APP
  //--------------------------------------------------------------------------------------------
  //--------------------------------------------------------------------------------------------
  myservo.attach(13); // SERVO FOR LOCK

  //--------------------------------------------------------------------------------------------
  I2CPower.begin(I2C_SDA, I2C_SCL, 400000); // TTGO DEVICE SMD CHARGER

  //--------------------------------------------------------------------------------------------
  // GSM MODULE PIN CONFIGURE SIM800L
  pinMode(MODEM_PWKEY, OUTPUT);
  pinMode(MODEM_RST, OUTPUT);
  pinMode(MODEM_POWER_ON, OUTPUT);
  digitalWrite(MODEM_PWKEY, LOW);
  digitalWrite(MODEM_RST, HIGH);
  digitalWrite(MODEM_POWER_ON, HIGH);
  SerialAT.begin(9600, SERIAL_8N1, MODEM_RX, MODEM_TX); // GSM UART CONNECT START

  //--------------------------------------------------------------------------------------------
  // SCOOTER PINOUT
  pinMode(RELAY_SCOOTER_STATUS, OUTPUT);
  // EEPROM ILE ROLE DURUMU KONTROL
  Check_SC_Relay_Status();
  // digitalWrite(RELAY_SCOOTER_STATUS, SCOOTER_STATE);
  pinMode(BUTTON_LOCK, INPUT_PULLUP);
  pinMode(BUZZER, OUTPUT);
  digitalWrite(BUZZER, LOW);

  //--------------------------------------------------------------------------------------------
  modem.restart();                         // GSM START
  String modemInfo = modem.getModemInfo(); // GSM VERSION
  Serial.print("MODEM INFO : ");
  Serial.println(modemInfo);

  //--------------------------------------------------------------------------------------------
  if (GSM_PIN && modem.getSimStatus() != 3)
  {
    modem.simUnlock(GSM_PIN);
  }

  //--------------------------------------------------------------------------------------------
  // PUBLISH AND SUBSCRIBE TOPIC STRING TO CHAR_ARRAY
  IMEISENDDATA = "data/KW/scootor/" + modem.getIMEI(); // PUBLISH TOPIC
  IMEISENDDATA.toCharArray(datasendtopic, 50);
  Serial.print("DATA SEND TOPIC (PUBLISH) : ");
  Serial.println(datasendtopic);

  IMEI = modem.getIMEI(); // SUBSCRIBE TOPIC
  IMEI.toCharArray(datataketopic, 50);
  Serial.print("DATA TAKE TOPIC (SUBSCRIBE) : ");
  Serial.println(datataketopic);

  IMEImqttID = modem.getIMEI();
  IMEImqttID.toCharArray(MQTTID, 50);
  //--------------------------------------------------------------------------------------------
  // GSM GPRS CONNECT
  if (!modem.gprsConnect(apn, gprsUser, gprsPass))
  {
    ESP.restart();
  }
  if (modem.isGprsConnected())
  {
    Serial.println("MODEM CONNECTED");
  }

  SerialGPS.begin(9600, SERIAL_8N1, 18, 19);

  //--------------------------------------------------------------------------------------------
  // MQTT SET CONFIGURE
  mqtt.setServer(broker, MQTT_PORT);
  mqtt.setCallback(mqttCallback); // MQTT'DE SUBSRIBE OLUNAN TOPIC'TEN VERILERI ALMAK ICIN
  mqttconnecttryjustone = 1;
  //--------------------------------------------------------------------------------------------
  Serial.println("....................................................");
  Serial.println("....................SETUP FINISH....................");
  Serial.println("......................STARTING......................");
}

//--------------------------------------------------------------------------------------------
//--------------------------------------------------------------------------------------------
// LOOP
//--------------------------------------------------------------------------------------------
//--------------------------------------------------------------------------------------------
void loop()
{
  esp_task_wdt_reset();
  // OTA
  server.handleClient();
  ArduinoOTA.handle();
  //--------------------------------------------------------------------------------------------
  // GPS DATA READ
  while (SerialGPS.available() > 0)
  {
    gps.encode(SerialGPS.read());
  }
  //--------------------------------------------------------------------------------------------
  // MQTT CONNECT
  if (mqttconnecttryjustone == 1)
  {
    mqttConnect();
    ReadBattery();
    PublishMqttData();
    mqttconnecttryjustone = 0;
  }
  if (!mqtt.connected())
  {
    uint32_t t = millis();
    if (t - lastReconnectAttempt > 60000)
    {
      lastReconnectAttempt = t;
      if (mqttConnect())
      {
        lastReconnectAttempt = 0;
      }
    }
  }
  //--------------------------------------------------------------------------------------------

  //--------------------------------------------------------------------------------------------
  // PUBLISH DATA
  //--------------------------------------------------------------------------------------------
  long now = millis();
  if (now - lastMsg > 300000)
  {
    lastMsg = now;

    //--------------------------------------------------------------------------------------------
    // MQTT DATA PREPARE
    ReadBattery();
    PublishMqttData();
  }
  //--------------------------------------------------------------------------------------------
  // BUZZER SUSTURMA

  if (buzzer_status == 1)
  {
    if (millis() > BUZZER_time_now + BUZZER_period)
    {
      digitalWrite(BUZZER, LOW);
      buzzer_status = 0;
    }
  }
  //--------------------------------------------------------------------------------------------
  // BUTTON - LOCK CONTROL
  if (digitalRead(BUTTON_LOCK) == 0)
  {

    if (servo_status == 1)
    {
      if (millis() > time_now + period)
      {
        servo_status = 0;
      }
    }
    else if (servo_status == 0)
    {
      myservo.write(20);
    }
  }

  if (digitalRead(BUTTON_LOCK) == 1)
  {
    if (servo_otuz_status == 1)
    {
      if (millis() > buton_one_time_now + buton_one_period)
      {
        myservo.write(40);
        servo_otuz_status = 0;
      }
    }
  }
  //--------------------------------------------------------------------------------------------

  mqtt.loop();
  //--------------------------------------------------------------------------------------------
}

//--------------------------------------------------------------------------------------------
// Subscribe MQTT and read data
//--------------------------------------------------------------------------------------------
void mqttCallback(char *topic, byte *payload, unsigned int length)
{
  StaticJsonDocument<512> doc;
  char json[length + 1];
  strncpy(json, (char *)payload, length);
  json[length] = '\0';
  DeserializationError error = deserializeJson(doc, json);
  if (error)
  {
    Serial.print(F("deserializeJson() failed: "));
    Serial.println(error.f_str());
    return;
  }

  //--------------------------------------------------------------------------------------------
  // DOC ISMIYLE PARSE EDILEN DATA'NIN ICINDEKI CMD DEGERININ DEGISKENE ATILMASI
  if (String(topic) == datataketopic)
  {
    String gelendeger = doc["cmd"];

    if (gelendeger == "start")
    {
      digitalWrite(RELAY_SCOOTER_STATUS, HIGH);
      lock_data = 1;
      myservo.write(60);
      PublishMqttData();
      servo_status = 1;
      servo_otuz_status = 1;
      EEPROM.write(EE_ADRESS, lock_data);
      EEPROM.commit();
      time_now = millis();
      buton_one_time_now = millis();
    }
    else if (gelendeger == "stop")
    {
      lock_data = 0;
      myservo.write(20);
      digitalWrite(RELAY_SCOOTER_STATUS, LOW);
      EEPROM.write(EE_ADRESS, lock_data);
      EEPROM.commit();
      PublishMqttData();
    }
    else if (gelendeger == "buzon")
    {
      buzzer_status = 1;
      BUZZER_time_now = millis();
      digitalWrite(BUZZER, HIGH);
    }
    else if (gelendeger == "restart")
    {
      Serial.println("RESTART");
      ESP.restart();
    }

    else if (gelendeger == "unlock")
    {
      myservo.write(60);
      servo_status = 1;
      servo_otuz_status = 1;
      time_now = millis();
      buton_one_time_now = millis();
    }
    else
    {
      Serial.println("Unknown data");
    }
  }
}

//--------------------------------------------------------------------------------------------
// MQTT CONNECT
//--------------------------------------------------------------------------------------------
boolean mqttConnect()
{
  boolean status = mqtt.connect(MQTTID, mqttUsername, mqttPassword);
  if (status == false)
  {
    ESP.restart();
    return false;
  }
  mqtt.subscribe(datataketopic);
  return mqtt.connected();
}

// 230711
//--------------------------------------------------------------------------------------------
//  SIM800L TTGO IP5306 Charger
//--------------------------------------------------------------------------------------------
bool setPowerBoostKeepOn(int en)
{
  I2CPower.beginTransmission(IP5306_ADDR);
  I2CPower.write(IP5306_REG_SYS_CTL0);
  if (en)
  {
    I2CPower.write(0x37); // Set bit1: 1 enable 0 disable boost keep on
  }
  else
  {
    I2CPower.write(0x35); // 0x37 is default reg value
  }
  return I2CPower.endTransmission() == 0;
}

//--------------------------------------------------------------------------------------------
// DATA
//--------------------------------------------------------------------------------------------
void PublishMqttData()
{
  StaticJsonDocument<512> Message;
  Message["la"] = gps.location.lat();
  Message["lo"] = gps.location.lng();
  Message["sb"] = yuzde; // yuzde -- battery
  Message["ib"] = 100;
  Message["sf"] = lock_data;
  char JSONmessageBuffer[200];
  serializeJsonPretty(Message, JSONmessageBuffer);
  // MQTT PUBLISH
  mqtt.publish(datasendtopic, JSONmessageBuffer);
}

//--------------------------------------------------------------------------------------------
// BATARYA ICIN ADC'NIN ORTALAMASINI ALMA
void ReadBattery()
{
  for (int i = 0; i < 100; i++)
  {
    deger = analogRead(34);
    okunandeger[i] = deger;
    total = total + okunandeger[i];
    average = total / 100;
  }
  // ORTALAMA ANALOG DEGER ILE BATARYA YUZDESINI BELIRLEME
  total = 0;
  // ORTALAMA ANALOG DEGERIN YUZDEYE CEVRILMESI
  yuzde = map(average, 3120, 4050, 0.0, 100.0);
  if (yuzde < 6)
  {
    yuzde = 0.01;
  }
  if (yuzde > 100)
  {
    yuzde = 100;
  }
}

// EEPROM

void Check_SC_Relay_Status()
{

  lock_data = EEPROM.read(EE_ADRESS);
  Serial.println(lock_data);

  if (lock_data == 1)
  {
    digitalWrite(RELAY_SCOOTER_STATUS, HIGH);
  }
  if (lock_data == 0)
  {
    digitalWrite(RELAY_SCOOTER_STATUS, LOW);
  }
  EEPROM.commit();
}

// *******************************************************************************************
// IRSendReceive
//
// A program for an ESP8266 board with following functions
// Version 1.0.1: initial version
//   - http web interface for sending & receiving IR codes, getting status information
//     and for system configuration
//   - network configuration via WiFi manager
//   - receiving IR codes from IR remote controls via an IR receiver (e.g. TSOP3123) at GPIO 14
//   - sending IR codes using IR transmitter diodes connected to GPIO 4
//   - forwarding of received IR codes to Fhem sevrer via http to set a configurable dummy variable
//   - over the air update (OTA)
//
// Version 1.1.0: status led added (using ticker)
//   - additional status LED connected to GPIO 0. The LED
//        blinks with 0.5 Hz during connection to WiFi network
//        is on for 2 seconds when an IR command is sent to Fhem
//        is on for 1 second whem a Temperature/Humidity value is sent to Fhem (Version >= 1.2.0)
//
// Version 1.2.0: DHT22 temperature/humidity sensor added
//   - DHT22 temperature/humidity sensor connected to GPIO 2 with configurable cyclic readout
//   - readout values are sent via http to the Fhem server to set a configurable Fhem dummy variable
//
// Version 1.2.1: bugfix & Status to Fhem added
//   - update of forceSend was missing -> T/H always sent to Fhem
//   - ststus information about received/sent and transferred IR commands is sent to Fhem dummy variable for IR commands
//
// http commands
//   http://xxx.xxx.xxx.xxx<cmd>
//   <cmd>
//   /          Homepage with status information
//   /setup     configure IRSendReceive
//              FhemIP=xxx.xxx.xxx.xxx  IP adress of Fhem server  (default 192.168.2.12)
//              FhemPort=xxxx           Port number of Fhem server (default 8083)
//              FhemMsg=[0|1]           Send IR command messages to Fhem (default 1)
//                                      0=no messages; 1=send messages
//              FhemFmt=[0|"fhem"]      transfer format of received IR commands (default 1)
//                                      0 -> json output; "fhem" -> fhem output (default "fhem")
//              FhemVarIR               Fhem variable to be set with IR command data (default "d_IR")
//              FhemVarTH               Fhem variable to be set with T/H values (default "d_Temp1")
//              DHTcycle                cycle time for T/H measurements in ms (default 60000)
//   /msg       send IR command 
//   /json      send IR command in json format
//   /received  retrieve received IR command in json format
//              Arguments:  
//              &id=n n=[1..5]   retrieve details of lan n-th received code
//   /update    start OTY firmware update
// 
// Huge parts of this program code are taken from the project ESP8266-HTTP-IR-Blaster
// (https://github.com/mdhiggins/ESP8266-HTTP-IR-Blaster) from Michael Higgins.
// Thanks for the cool work!
//
// *******************************************************************************************

// IR support
#include <IRremoteESP8266.h>
#include <IRsend.h>
#include <IRrecv.h>
#include <IRutils.h>

// DHT22 support
#include <Adafruit_Sensor.h>
#include <DHT.h>
#include <DHT_U.h>

// WiFi support
#include <ESP8266WiFi.h>
#include <WiFiManager.h>                // https://github.com/tzapu/WiFiManager WiFi Configuration Magic
#include <ESP8266mDNS.h>                // Useful to access to ESP by hostname.local
#include <Ticker.h>

#include <ArduinoJson.h>                // json support
#include <ESP8266WebServer.h>           // needed for web server
#include <ESP8266HTTPClient.h>          // needed for client functions
#include <ESP8266HTTPUpdateServer.h>    // needed for OTA updates

#include <NTPClient.h>                  // needed for time information

String Version="1.2.1 " + String(__DATE__) + " " + String(__TIME__);

int port = 80;
char host_name[40] = "ESP8266IR";
char port_str[20] = "80";

DynamicJsonBuffer jsonBuffer;

JsonObject& last_code = jsonBuffer.createObject();            // Stores last code
JsonObject& last_code_2 = jsonBuffer.createObject();          // Stores 2nd to last code
JsonObject& last_code_3 = jsonBuffer.createObject();          // Stores 3rd to last code
JsonObject& last_code_4 = jsonBuffer.createObject();          // Stores 4th to last code
JsonObject& last_code_5 = jsonBuffer.createObject();          // Stores 5th to last code

JsonObject& last_send = jsonBuffer.createObject();            // Stores last sent
JsonObject& last_send_2 = jsonBuffer.createObject();          // Stores 2nd last sent
JsonObject& last_send_3 = jsonBuffer.createObject();          // Stores 3rd last sent
JsonObject& last_send_4 = jsonBuffer.createObject();          // Stores 4th last sent
JsonObject& last_send_5 = jsonBuffer.createObject();          // Stores 5th last sent

JsonObject& deviceState = jsonBuffer.createObject();

ESP8266WebServer HTTPServer(port);
ESP8266HTTPUpdateServer httpUpdater;

bool holdReceive = false;                                     // Flag to prevent IR receiving while transmitting

// configuration of GPIO pins
int pinr = 14;                                                // Receiving pin (GPIO14 -> D5)
int pins = 4;                                                 // Transmitting pin (GPIO4 -> D2)
int ledpin = 0;                                               // status LED pin (GPIO0 -> D3)
int dhtpin = 2;                                               // DHT22 sensor pin (GPIO2 -> D4)

IRrecv irrecv(pinr);
IRsend irsend(pins);

// DHT related variables
DHT_Unified dht(dhtpin, DHT22);
static float lastTemp=0;
static float lastHum=0;
static unsigned long lastSent=0;
uint32_t delayMS;

const unsigned long resetfrequency = 259200000;                // 72 hours in milliseconds
int timeOffset = 7200;                                         // Timezone offset in seconds (MESZ)
const char* poolServerName = "time.nist.gov";

const bool getTime = true;                                     // Set to false to disable querying for the time
WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP, poolServerName, timeOffset, (int)resetfrequency);
WiFiManager wifiManager;

String FhemIP   = "192.168.2.12";                              // ip address of fhem system
String FhemPort = "8083";                                      // port of fhem web server
String FhemFmt  = "fhem";                                      // output format for fhem message
String FhemVarIR = "d_IR";                                     // name of fhem dummy variable to set
String FhemVarTH = "d_Temp1";                                  // name of fhem dummy variable to set for temp & humidity
int FhemMsg = 1;                                               // send recieved IR commands to fhem
int DHTcycle = 60000;                                          // time between DHT measures in ms
int STATUScycle = 600000;                                      // time between status messages in ms

// OTA related settings
const char* update_path = "/update";
const char* update_username = "admin";
const char* update_password = "cman";

int CountIRreceived=0;
int CountIRsent=0;
int CountIRtransferred=0;

Ticker LEDticker;

// ------------------------------------------------------------------------------------------
// Status LED blinking
// ------------------------------------------------------------------------------------------
void LEDblink()
{
  int state = digitalRead(ledpin);        // get the current state
  digitalWrite(ledpin, !state);           // toggle state
}

void LEDoff()
{
  digitalWrite(ledpin, LOW);           // toggle state
  LEDticker.detach();
}

// ------------------------------------------------------------------------------------------
// DHT initialization
// ------------------------------------------------------------------------------------------
void DHTinit() {
  dht.begin();
  
  // Print temperature sensor details.
  sensor_t sensor;
  dht.temperature().getSensor(&sensor);
  Serial.println("");
  Serial.print  ("[INIT ] Temperature Sensor: "); Serial.println(sensor.name);
  Serial.print  ("[INIT ] Driver Version:     "); Serial.println(sensor.version);
  Serial.print  ("[INIT ] Unique ID:          "); Serial.println(sensor.sensor_id);
  Serial.print  ("[INIT ] Max Value:          "); Serial.print(sensor.max_value); Serial.println(" *C");
  Serial.print  ("[INIT ] Min Value:          "); Serial.print(sensor.min_value); Serial.println(" *C");
  Serial.print  ("[INIT ] Resolution:         "); Serial.print(sensor.resolution); Serial.println(" *C");  
  Serial.println("");

  // Print humidity sensor details.
  dht.humidity().getSensor(&sensor);
  Serial.print  ("[INIT ] Humidity Sensor:    "); Serial.println(sensor.name);
  Serial.print  ("[INIT ] Driver Version:     "); Serial.println(sensor.version);
  Serial.print  ("[INIT ] Unique ID:          "); Serial.println(sensor.sensor_id);
  Serial.print  ("[INIT ] Max Value:          "); Serial.print(sensor.max_value); Serial.println("%");
  Serial.print  ("[INIT ] Min Value:          "); Serial.print(sensor.min_value); Serial.println("%");
  Serial.print  ("[INIT ] Resolution:         "); Serial.print(sensor.resolution); Serial.println("%");  
  Serial.println("");
  
  // Set delay between sensor readings based on sensor details.
  delayMS = sensor.min_delay / 1000;
}

// ------------------------------------------------------------------------------------------
// Reenable IR receiving
// ------------------------------------------------------------------------------------------
void resetReceive() {
  if (holdReceive) {
    Serial.println("Reenabling receiving");
    irrecv.resume();
    holdReceive = false;
  }
}

// ------------------------------------------------------------------------------------------
// IP Address to String
// ------------------------------------------------------------------------------------------
String ipToString(IPAddress ip)
{
  String s = String(ip[0]);
  for (int i = 1; i < 4; i++)
    s += "." + String(ip[i]);
  return s;
}

// ------------------------------------------------------------------------------------------
// Setup web server and IR receiver/blaster
// ------------------------------------------------------------------------------------------
void setup() {

  // Initialize serial
  Serial.begin(115200);
  delay(1000);
  Serial.println("");
  Serial.printf("ESP8266 IR Controller (Version %s)\n", Version.c_str());
  delay(1000);
  
  // set GPIO0 as output (LED)
  pinMode(ledpin, OUTPUT);
  digitalWrite(ledpin, LOW);
  LEDticker.attach(0.5, LEDblink);
    
  // establish wlan connection
  wifiManager.autoConnect("ESP8266 IR Controller");

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.print("\n");
  LEDoff();

  wifi_set_sleep_type(LIGHT_SLEEP_T);

  Serial.print("[INIT ] Local IP: ");
  Serial.println(ipToString(WiFi.localIP()));

  if (getTime) timeClient.begin(); // Get the time

  // Configure the server
  HTTPServer.on("/json", handle_json);
  HTTPServer.on("/msg", handle_msg);
  HTTPServer.on("/received", handleReceived);
  HTTPServer.on("/setup", handle_setup);
  HTTPServer.on("/reset", []() {
    Serial.println("[HTTP ] Connection received: /reset");
    ESP.reset();
  });
  HTTPServer.on("/", []() {
    Serial.println("[HTTP ] Connection received: /");
    sendHomePage(); // 200
  });

  MDNS.begin(host_name);

  httpUpdater.setup(&HTTPServer, update_path, update_username, update_password);
  HTTPServer.begin();
  Serial.println("[INIT ] HTTP Server started on port " + String(port));
  MDNS.addService("http", "tcp", 80);

  irsend.begin();
  irrecv.enableIRIn();
  Serial.println("[INIT ] Ready to send and receive IR signals");

  // init DHT sensor 
  DHTinit();
}

// ------------------------------------------------------------------------------------------
// handle msg
// ------------------------------------------------------------------------------------------
void handle_msg() {
  Serial.println("[HTTP ] Connection received: /msg");
  int simple = 0;
  if (HTTPServer.hasArg("simple")) simple = HTTPServer.arg("simple").toInt();
  String type = HTTPServer.arg("type");
  String data = HTTPServer.arg("data");

  int len = HTTPServer.arg("length").toInt();
  long address = (HTTPServer.hasArg("address")) ? HTTPServer.arg("address").toInt() : 0;
  int rdelay = (HTTPServer.hasArg("rdelay")) ? HTTPServer.arg("rdelay").toInt() : 1000;
  int pulse = (HTTPServer.hasArg("pulse")) ? HTTPServer.arg("pulse").toInt() : 1;
  int pdelay = (HTTPServer.hasArg("pdelay")) ? HTTPServer.arg("pdelay").toInt() : 100;
  int repeat = (HTTPServer.hasArg("repeat")) ? HTTPServer.arg("repeat").toInt() : 1;

  if (HTTPServer.hasArg("code")) {
    String code = HTTPServer.arg("code");
    char separator = ':';
    data = getValue(code, separator, 0);
    type = getValue(code, separator, 1);
    len = getValue(code, separator, 2).toInt();
  }

  if (simple) {
    HTTPServer.send(200, "text/html", "Success, code sent");
  }

  irblast(type, data, len, rdelay, pulse, pdelay, repeat, address, irsend);

  if (!simple) {
    sendHomePage("Code Sent", "Success", 1); // 200
  }
}

// ------------------------------------------------------------------------------------------
// handle setup
// ------------------------------------------------------------------------------------------
void handle_setup() {
  Serial.println("[HTTP ] Connection received: /setup");
  FhemIP    = (HTTPServer.hasArg("FhemIp"))    ? HTTPServer.arg("FhemIp")           : FhemIP;
  FhemPort  = (HTTPServer.hasArg("FhemPort"))  ? HTTPServer.arg("FhemPort")         : FhemPort;
  FhemMsg   = (HTTPServer.hasArg("FhemMsg"))   ? HTTPServer.arg("FhemMsg").toInt()  : FhemMsg;
  FhemFmt   = (HTTPServer.hasArg("FhemFmt"))   ? HTTPServer.arg("FhemFmt")          : FhemFmt;
  FhemVarIR = (HTTPServer.hasArg("FhemVarIR")) ? HTTPServer.arg("FhemVarIR")        : FhemVarIR;
  FhemVarTH = (HTTPServer.hasArg("FhemVarTH")) ? HTTPServer.arg("FhemVarTH")        : FhemVarTH;
  DHTcycle  = (HTTPServer.hasArg("DHTcycle"))  ? HTTPServer.arg("DHTcycle").toInt() : DHTcycle;
  sendHomePage(); // 200
}

// ------------------------------------------------------------------------------------------
// handle json
// ------------------------------------------------------------------------------------------
void handle_json() {
  Serial.println("[HTTP ] Connection received: /json");
  DynamicJsonBuffer jsonBuffer;
  JsonArray& root = jsonBuffer.parseArray(HTTPServer.arg("plain"));

  int simple = 0;
  if (HTTPServer.hasArg("simple")) simple = HTTPServer.arg("simple").toInt();

  if (!root.success()) {
    Serial.println("[HTTP ] JSON parsing failed");
    if (simple) {
      HTTPServer.send(400, "text/plain", "JSON parsing failed");
    } else {
      sendHomePage("JSON parsing failed", "Error", 3, 400); // 400
    }
  } else {
    if (simple) {
      HTTPServer.send(200, "text/html", "Success, code sent");
    }

    for (unsigned int x = 0; x < root.size(); x++) {
      String type = root[x]["type"];
      String ip = root[x]["ip"];
      int rdelay = root[x]["rdelay"];
      int pulse = root[x]["pulse"];
      int pdelay = root[x]["pdelay"];
      int repeat = root[x]["repeat"];
      //int out = root[x]["out"];
      if (pulse <= 0) pulse = 1; // Make sure pulse isn't 0
      if (repeat <= 0) repeat = 1; // Make sure repeat isn't 0
      if (pdelay <= 0) pdelay = 100; // Default pdelay
      if (rdelay <= 0) rdelay = 1000; // Default rdelay

      if (type == "delay") {
        delay(rdelay);
      } else if (type == "raw") {
        JsonArray &raw = root[x]["data"]; // Array of unsigned int values for the raw signal
        int khz = root[x]["khz"];
        if (khz <= 0) khz = 38; // Default to 38khz if not set
        rawblast(raw, khz, rdelay, pulse, pdelay, repeat, irsend);
      } else {
        String data = root[x]["data"];
        long address = root[x]["address"];
        int len = root[x]["length"];
        irblast(type, data, len, rdelay, pulse, pdelay, repeat, address, irsend);
      }
    }

    if (!simple) {
      Serial.println("[HTTP ] Sending home page");
      sendHomePage("Code sent", "Success", 1); // 200
    }
  }
}

// ------------------------------------------------------------------------------------------
// handle received
// ------------------------------------------------------------------------------------------
void handleReceived() {
  Serial.println("[HTTP ] Connection received: /received");
  int id = HTTPServer.arg("id").toInt();

  if (id == 1 && last_code.containsKey("time"))        sendCodePage(last_code);
  else if (id == 2 && last_code_2.containsKey("time")) sendCodePage(last_code_2);
  else if (id == 3 && last_code_3.containsKey("time")) sendCodePage(last_code_3);
  else if (id == 4 && last_code_4.containsKey("time")) sendCodePage(last_code_4);
  else if (id == 5 && last_code_5.containsKey("time")) sendCodePage(last_code_5);
  else sendHomePage("Code does not exist", "Alert", 2, 404); // 404
}

// ------------------------------------------------------------------------------------------
// Split string by character
// ------------------------------------------------------------------------------------------
String getValue(String data, char separator, int index)
{
  int found = 0;
  int strIndex[] = {0, -1};
  int maxIndex = data.length() - 1;

  for (int i = 0; i <= maxIndex && found <= index; i++) {
    if (data.charAt(i) == separator || i == maxIndex) {
      found++;
      strIndex[0] = strIndex[1] + 1;
      strIndex[1] = (i == maxIndex) ? i + 1 : i;
    }
  }

  return found > index ? data.substring(strIndex[0], strIndex[1]) : "";
}

// ------------------------------------------------------------------------------------------
// Display encoding type
// ------------------------------------------------------------------------------------------
String encoding(decode_results *results) {
  String output;
  switch (results->decode_type) {
    default:
    case UNKNOWN:      output = "UNKNOWN";            break;
    case NEC:          output = "NEC";                break;
    case SONY:         output = "SONY";               break;
    case RC5:          output = "RC5";                break;
    case RC6:          output = "RC6";                break;
    case DISH:         output = "DISH";               break;
    case SHARP:        output = "SHARP";              break;
    case JVC:          output = "JVC";                break;
    case SANYO:        output = "SANYO";              break;
    case SANYO_LC7461: output = "SANYO_LC7461";       break;
    case MITSUBISHI:   output = "MITSUBISHI";         break;
    case SAMSUNG:      output = "SAMSUNG";            break;
    case LG:           output = "LG";                 break;
    case WHYNTER:      output = "WHYNTER";            break;
    case AIWA_RC_T501: output = "AIWA_RC_T501";       break;
    case PANASONIC:    output = "PANASONIC";          break;
    case DENON:        output = "DENON";              break;
    case COOLIX:       output = "COOLIX";             break;
  }
  return output;
  if (results->repeat) Serial.print("[IR   ] (Repeat)");
}

// ------------------------------------------------------------------------------------------
// Uint64 to String
// ------------------------------------------------------------------------------------------
String Uint64toString(uint64_t input, uint8_t base) {
  char buf[8 * sizeof(input) + 1];  // Assumes 8-bit chars plus zero byte.
  char *str = &buf[sizeof(buf) - 1];

  *str = '\0';

  // prevent crash if called with base == 1
  if (base < 2) base = 10;

  do {
    char c = input % base;
    input /= base;

    *--str = c < 10 ? c + '0' : c + 'A' - 10;
  } while (input);

  std::string s(str);
  return s.c_str();
}

// ------------------------------------------------------------------------------------------
// Code to string
// ------------------------------------------------------------------------------------------
void fullCode (decode_results *results)
{
  Serial.print("[IR   ] One line: ");
  serialPrintUint64(results->value, 16);
  Serial.print(":");
  Serial.print(encoding(results));
  Serial.print(":");
  Serial.print(results->bits, DEC);
  if (results->overflow)
    Serial.println("[IR   ] WARNING: IR code too long."
                   "Edit IRrecv.h and increase RAWBUF");
  Serial.println("");
}

// ------------------------------------------------------------------------------------------
// Send header HTML
// ------------------------------------------------------------------------------------------
void sendHeader() {
  sendHeader(200);
}

void sendHeader(int httpcode) {
  HTTPServer.setContentLength(CONTENT_LENGTH_UNKNOWN);
  HTTPServer.send(httpcode, "text/html; charset=utf-8", "");
  HTTPServer.sendContent("<!DOCTYPE html PUBLIC '-//W3C//DTD XHTML 1.0 Strict//EN' 'http://www.w3.org/TR/xhtml1/DTD/xhtml1-strict.dtd'>\n");
  HTTPServer.sendContent("<html xmlns='http://www.w3.org/1999/xhtml' xml:lang='en'>\n");
  HTTPServer.sendContent("  <head>\n");
  HTTPServer.sendContent("    <meta name='viewport' content='width=device-width, initial-scale=.75' />\n");
  HTTPServer.sendContent("    <link rel='stylesheet' href='https://maxcdn.bootstrapcdn.com/bootstrap/3.3.7/css/bootstrap.min.css' />\n");
  HTTPServer.sendContent("    <style>@media (max-width: 991px) {.nav-pills>li {float: none; margin-left: 0; margin-top: 5px; text-align: center;}}</style>\n");
  HTTPServer.sendContent("    <title>ESP8266 IR Controller (" + String(host_name) + ")</title>\n");
  HTTPServer.sendContent("  </head>\n");
  HTTPServer.sendContent("  <body>\n");
  HTTPServer.sendContent("    <div class='container'>\n");
  HTTPServer.sendContent("      <h1><a href='https://github.com/dr-cman/IRSendReceive'>ESP8266 IR Controller</a></h1>\n");
  HTTPServer.sendContent("      <div class='row'>\n");
  HTTPServer.sendContent("        <div class='col-md-12'>\n");
  HTTPServer.sendContent("          <ul class='nav nav-pills'>\n");
  HTTPServer.sendContent("            <li class='active'>\n");
  HTTPServer.sendContent("              <a href='http://" + ipToString(WiFi.localIP()) + ":" + String(port) + "'>Local <span class='badge'>" + ipToString(WiFi.localIP()) + ":" + String(port) + "</span></a></li>\n");
  HTTPServer.sendContent("            <li class='active'>\n");
  HTTPServer.sendContent("              <a href='#'>MAC <span class='badge'>" + String(WiFi.macAddress()) + "</span></a></li>\n");
  HTTPServer.sendContent("          </ul>\n");
  HTTPServer.sendContent("        </div>\n");
  HTTPServer.sendContent("      </div><hr />\n");
}

// ------------------------------------------------------------------------------------------
// Send footer HTML
// ------------------------------------------------------------------------------------------
void sendFooter() {
  HTTPServer.sendContent("      <div class='row'><div class='col-md-12'><em>" + String(millis()) + "ms uptime</em></div></div>\n");
  HTTPServer.sendContent("      <div class='row'><div class='col-md-12'><em> Firmware Version " + Version + "</em></div></div>\n");
  HTTPServer.sendContent("    </div>\n");
  HTTPServer.sendContent("  </body>\n");
  HTTPServer.sendContent("</html>\n");
  HTTPServer.client().stop();
}

// ------------------------------------------------------------------------------------------
// Stream home page HTML
// ------------------------------------------------------------------------------------------
void sendHomePage() {
  sendHomePage("", "");
}

void sendHomePage(String message, String header) {
  sendHomePage(message, header, 0);
}

void sendHomePage(String message, String header, int type) {
  sendHomePage(message, header, type, 200);
}

void sendHomePage(String message, String header, int type, int httpcode) {
  sendHeader(httpcode);
  if (type == 1)
    HTTPServer.sendContent("      <div class='row'><div class='col-md-12'><div class='alert alert-success'><strong>" + header + "!</strong> " + message + "</div></div></div>\n");
  if (type == 2)
    HTTPServer.sendContent("      <div class='row'><div class='col-md-12'><div class='alert alert-warning'><strong>" + header + "!</strong> " + message + "</div></div></div>\n");
  if (type == 3)
    HTTPServer.sendContent("      <div class='row'><div class='col-md-12'><div class='alert alert-danger'><strong>" + header + "!</strong> " + message + "</div></div></div>\n");

  HTTPServer.sendContent("      <div class='row'>\n");
  HTTPServer.sendContent("        <div class='col-md-12'>\n");
  HTTPServer.sendContent("          <h3>Codes Transmitted</h3>\n");
  HTTPServer.sendContent("          <table class='table table-striped' style='table-layout: fixed;'>\n");
  HTTPServer.sendContent("            <thead><tr><th>Sent</th><th>Command</th><th>Type</th><th>Length</th><th>Address</th></tr></thead>\n"); //Title
  HTTPServer.sendContent("            <tbody>\n");
  if (last_send.containsKey("time"))
    HTTPServer.sendContent("              <tr class='text-uppercase'><td>" + last_send["time"].as<String>() + "</td><td><code>" + last_send["data"].as<String>() + "</code></td><td><code>" + last_send["type"].as<String>() + "</code></td><td><code>" + last_send["len"].as<String>() + "</code></td><td><code>" + last_send["address"].as<String>() + "</code></td></tr>\n");
  if (last_send_2.containsKey("time"))
    HTTPServer.sendContent("              <tr class='text-uppercase'><td>" + last_send_2["time"].as<String>() + "</td><td><code>" + last_send_2["data"].as<String>() + "</code></td><td><code>" + last_send_2["type"].as<String>() + "</code></td><td><code>" + last_send_2["len"].as<String>() + "</code></td><td><code>" + last_send_2["address"].as<String>() + "</code></td></tr>\n");
  if (last_send_3.containsKey("time"))
    HTTPServer.sendContent("              <tr class='text-uppercase'><td>" + last_send_3["time"].as<String>() + "</td><td><code>" + last_send_3["data"].as<String>() + "</code></td><td><code>" + last_send_3["type"].as<String>() + "</code></td><td><code>" + last_send_3["len"].as<String>() + "</code></td><td><code>" + last_send_3["address"].as<String>() + "</code></td></tr>\n");
  if (last_send_4.containsKey("time"))
    HTTPServer.sendContent("              <tr class='text-uppercase'><td>" + last_send_4["time"].as<String>() + "</td><td><code>" + last_send_4["data"].as<String>() + "</code></td><td><code>" + last_send_4["type"].as<String>() + "</code></td><td><code>" + last_send_4["len"].as<String>() + "</code></td><td><code>" + last_send_4["address"].as<String>() + "</code></td></tr>\n");
  if (last_send_5.containsKey("time"))
    HTTPServer.sendContent("              <tr class='text-uppercase'><td>" + last_send_5["time"].as<String>() + "</td><td><code>" + last_send_5["data"].as<String>() + "</code></td><td><code>" + last_send_5["type"].as<String>() + "</code></td><td><code>" + last_send_5["len"].as<String>() + "</code></td><td><code>" + last_send_5["address"].as<String>() + "</code></td></tr>\n");
  if (!last_send.containsKey("time") && !last_send_2.containsKey("time") && !last_send_3.containsKey("time") && !last_send_4.containsKey("time") && !last_send_5.containsKey("time"))
    HTTPServer.sendContent("              <tr><td colspan='5' class='text-center'><em>No codes sent</em></td></tr>");
  HTTPServer.sendContent("            </tbody></table>\n");
  HTTPServer.sendContent("          </div></div>\n");
  HTTPServer.sendContent("      <div class='row'>\n");
  HTTPServer.sendContent("        <div class='col-md-12'>\n");
  HTTPServer.sendContent("          <h3>Codes Received</h3>\n");
  HTTPServer.sendContent("          <table class='table table-striped' style='table-layout: fixed;'>\n");
  HTTPServer.sendContent("            <thead><tr><th>Time Sent</th><th>Command</th><th>Type</th><th>Length</th><th>Address</th></tr></thead>\n"); //Title
  HTTPServer.sendContent("            <tbody>\n");
  if (last_code.containsKey("time"))
    HTTPServer.sendContent("              <tr class='text-uppercase'><td><a href='/received?id=1'>" + last_code["time"].as<String>() + "</a></td><td><code>" + last_code["data"].as<String>() + "</code></td><td><code>" + last_code["encoding"].as<String>() + "</code></td><td><code>" + last_code["bits"].as<String>() + "</code></td><td><code>" + last_code["address"].as<String>() + "</code></td></tr>\n");
  if (last_code_2.containsKey("time"))
    HTTPServer.sendContent("              <tr class='text-uppercase'><td><a href='/received?id=2'>" + last_code_2["time"].as<String>() + "</a></td><td><code>" + last_code_2["data"].as<String>() + "</code></td><td><code>" + last_code_2["encoding"].as<String>() + "</code></td><td><code>" + last_code_2["bits"].as<String>() + "</code></td><td><code>" + last_code_2["address"].as<String>() + "</code></td></tr>\n");
  if (last_code_3.containsKey("time"))
    HTTPServer.sendContent("              <tr class='text-uppercase'><td><a href='/received?id=3'>" + last_code_3["time"].as<String>() + "</a></td><td><code>" + last_code_3["data"].as<String>() + "</code></td><td><code>" + last_code_3["encoding"].as<String>() + "</code></td><td><code>" + last_code_3["bits"].as<String>() + "</code></td><td><code>" + last_code_3["address"].as<String>() + "</code></td></tr>\n");
  if (last_code_4.containsKey("time"))
    HTTPServer.sendContent("              <tr class='text-uppercase'><td><a href='/received?id=4'>" + last_code_4["time"].as<String>() + "</a></td><td><code>" + last_code_4["data"].as<String>() + "</code></td><td><code>" + last_code_4["encoding"].as<String>() + "</code></td><td><code>" + last_code_4["bits"].as<String>() + "</code></td><td><code>" + last_code_4["address"].as<String>() + "</code></td></tr>\n");
  if (last_code_5.containsKey("time"))
    HTTPServer.sendContent("              <tr class='text-uppercase'><td><a href='/received?id=5'>" + last_code_5["time"].as<String>() + "</a></td><td><code>" + last_code_5["data"].as<String>() + "</code></td><td><code>" + last_code_5["encoding"].as<String>() + "</code></td><td><code>" + last_code_5["bits"].as<String>() + "</code></td><td><code>" + last_code_5["address"].as<String>() + "</code></td></tr>\n");
  if (!last_code.containsKey("time") && !last_code_2.containsKey("time") && !last_code_3.containsKey("time") && !last_code_4.containsKey("time") && !last_code_5.containsKey("time"))
    HTTPServer.sendContent("              <tr><td colspan='5' class='text-center'><em>No codes received</em></td></tr>");
  HTTPServer.sendContent("            </tbody></table>\n");
  HTTPServer.sendContent("          </div></div>\n");
  HTTPServer.sendContent("      <div class='row'>\n");
  HTTPServer.sendContent("        <div class='col-md-12'>\n");
  HTTPServer.sendContent("          <ul class='list-unstyled'>\n");
  HTTPServer.sendContent("            <li> Receiving GPIO <span class='badge'>" + String(pinr) + "</span></li>\n");
  HTTPServer.sendContent("            <li> Transmitting GPIO <span class='badge'>" + String(pins) + "</span></li>\n");
  HTTPServer.sendContent("            <li> FhemMsg <span class='badge'>" + String(FhemMsg) + "</span></li>\n");
  HTTPServer.sendContent("            <li> FhemIP:FhemPort <span class='badge'>" + String(FhemIP) + ":" + String(FhemPort) + "</span></li>\n");
  HTTPServer.sendContent("            <li> FhemVarIR <span class='badge'>" + String(FhemVarIR) + "</span></li>\n");
  HTTPServer.sendContent("            <li> FhemVarTH <span class='badge'>" + String(FhemVarTH) + "</span></li>\n");
  HTTPServer.sendContent("            <li> FhemFmt <span class='badge'>" + String(FhemFmt) + "</span></li>\n");
  HTTPServer.sendContent("            <li> DHTcycle <span class='badge'>" + String(DHTcycle) + "</span></li>\n");
  HTTPServer.sendContent("        </div>\n");
  HTTPServer.sendContent("      </div>\n");
  sendFooter();
}

// ------------------------------------------------------------------------------------------
// Stream code page HTML
// ------------------------------------------------------------------------------------------
void sendCodePage(JsonObject& selCode) {
  sendCodePage(selCode, 200);
}

void sendCodePage(JsonObject& selCode, int httpcode) {
  sendHeader(httpcode);
  HTTPServer.sendContent("      <div class='row'>\n");
  HTTPServer.sendContent("        <div class='col-md-12'>\n");
  HTTPServer.sendContent("          <h2><span class='label label-success'>" + selCode["data"].as<String>() + ":" + selCode["encoding"].as<String>() + ":" + selCode["bits"].as<String>() + "</span></h2><br/>\n");
  HTTPServer.sendContent("          <dl class='dl-horizontal'>\n");
  HTTPServer.sendContent("            <dt>Data</dt>\n");
  HTTPServer.sendContent("            <dd><code>" + selCode["data"].as<String>()  + "</code></dd></dl>\n");
  HTTPServer.sendContent("          <dl class='dl-horizontal'>\n");
  HTTPServer.sendContent("            <dt>Type</dt>\n");
  HTTPServer.sendContent("            <dd><code>" + selCode["encoding"].as<String>()  + "</code></dd></dl>\n");
  HTTPServer.sendContent("          <dl class='dl-horizontal'>\n");
  HTTPServer.sendContent("            <dt>Length</dt>\n");
  HTTPServer.sendContent("            <dd><code>" + selCode["bits"].as<String>()  + "</code></dd></dl>\n");
  HTTPServer.sendContent("          <dl class='dl-horizontal'>\n");
  HTTPServer.sendContent("            <dt>Address</dt>\n");
  HTTPServer.sendContent("            <dd><code>" + selCode["address"].as<String>()  + "</code></dd></dl>\n");
  HTTPServer.sendContent("          <dl class='dl-horizontal'>\n");
  HTTPServer.sendContent("            <dt>Raw</dt>\n");
  HTTPServer.sendContent("            <dd><code>" + selCode["uint16_t"].as<String>()  + "</code></dd></dl>\n");
  HTTPServer.sendContent("        </div></div>\n");
  HTTPServer.sendContent("      <div class='row'>\n");
  if (selCode["encoding"] == "UNKNOWN") {
    HTTPServer.sendContent("      <div class='row'>\n");
    HTTPServer.sendContent("        <div class='col-md-12'>\n");
    HTTPServer.sendContent("          <ul class='list-unstyled'>\n");
    HTTPServer.sendContent("            <li>Local IP <span class='label label-default'>JSON</span></li>\n");
    HTTPServer.sendContent("            <li><pre>http://" + ipToString(WiFi.localIP()) + ":" + String(port) + "/json?plain=[{'data':[" + selCode["uint16_t"].as<String>() + "], 'type':'raw', 'khz':38}]</pre></li>\n");
  } else {
    HTTPServer.sendContent("      <div class='row'>\n");
    HTTPServer.sendContent("        <div class='col-md-12'>\n");
    HTTPServer.sendContent("          <ul class='list-unstyled'>\n");
    HTTPServer.sendContent("            <li>Local IP <span class='label label-default'>MSG</span></li>\n");
    HTTPServer.sendContent("            <li><pre>http://" + ipToString(WiFi.localIP()) + ":" + String(port) + "/msg?code=" + selCode["data"].as<String>() + ":" + selCode["encoding"].as<String>() + ":" + selCode["bits"].as<String>() + "</pre></li>\n");
    HTTPServer.sendContent("          <ul class='list-unstyled'>\n");
    HTTPServer.sendContent("            <li>Local IP <span class='label label-default'>JSON</span></li>\n");
    HTTPServer.sendContent("            <li><pre>http://" + ipToString(WiFi.localIP()) + ":" + String(port) + "/json?plain=[{'data':'" + selCode["data"].as<String>() + "', 'type':'" + selCode["encoding"].as<String>() + "', 'length':" + selCode["bits"].as<String>() + "}]</pre></li>\n");
  }
  HTTPServer.sendContent("        </div>\n");
  HTTPServer.sendContent("     </div>\n");
  sendFooter();
}

// ------------------------------------------------------------------------------------------
// Code to JsonObject
// ------------------------------------------------------------------------------------------
void codeJson(JsonObject &codeData, decode_results *results)
{
  codeData["data"] = Uint64toString(results->value, 16);
  codeData["encoding"] = encoding(results);
  codeData["bits"] = results->bits;
  codeData["rawlen"] = results->rawlen - 1;
  String r = "";
  for (uint16_t i = 1; i < results->rawlen; i++) {
    r += results->rawbuf[i] * RAWTICK;
    if (i < results->rawlen - 1)
      r += ",";                           // ',' not needed on last one
    //if (!(i & 1)) r += " ";
  }
  codeData["uint16_t"] = r;

  if (results->decode_type != UNKNOWN) {
    codeData["address"] = "0x" + String(results->address, HEX);
    codeData["command"] = "0x" + String(results->command, HEX);
  } else {
    codeData["address"] = "0x";
    codeData["command"] = "0x";
  }
}

// ------------------------------------------------------------------------------------------
// Dump out the decode_results structure.
// ------------------------------------------------------------------------------------------
void dumpInfo(decode_results *results) {
  if (results->overflow)
    Serial.println("[IR   ] WARNING: IR code too long."
                   "Edit IRrecv.h and increase RAWBUF");

  // Show Encoding standard
  Serial.print("[IR   ] Encoding  : ");
  Serial.print(encoding(results));
  Serial.println("");

  // Show Code & length
  Serial.print("[IR   ] Code      : ");
  serialPrintUint64(results->value, 16);
  Serial.print(" (");
  Serial.print(results->bits, DEC);
  Serial.println(" bits)");
}

// ------------------------------------------------------------------------------------------
// Dump out the decode_results structure.
// ------------------------------------------------------------------------------------------
void dumpRaw(decode_results *results) {
  // Print Raw data
  Serial.print("[IR   ] Timing[");
  Serial.print(results->rawlen - 1, DEC);
  Serial.println("]: ");

  for (uint16_t i = 1;  i < results->rawlen;  i++) {
    if (i % 100 == 0)
      yield();  // Preemptive yield every 100th entry to feed the WDT.
    uint32_t x = results->rawbuf[i] * RAWTICK;
    if (!(i & 1)) {  // even
      Serial.print("-");
      if (x < 1000) Serial.print(" ");
      if (x < 100) Serial.print(" ");
      Serial.print(x, DEC);
    } else {  // odd
      Serial.print("     ");
      Serial.print("+");
      if (x < 1000) Serial.print(" ");
      if (x < 100) Serial.print(" ");
      Serial.print(x, DEC);
      if (i < results->rawlen - 1)
        Serial.print(", ");  // ',' not needed for last one
    }
    if (!(i % 8)) Serial.println("");
  }
  Serial.println("");  // Newline
}

// ------------------------------------------------------------------------------------------
// Dump out the decode_results structure.
// ------------------------------------------------------------------------------------------
void dumpCode(decode_results *results) {
  // Start declaration
  Serial.print("[IR   ] uint16_t  ");              // variable type
  Serial.print("rawData[");                // array name
  Serial.print(results->rawlen - 1, DEC);  // array size
  Serial.print("] = {");                   // Start declaration

  // Dump data
  for (uint16_t i = 1; i < results->rawlen; i++) {
    Serial.print(results->rawbuf[i] * RAWTICK, DEC);
    if (i < results->rawlen - 1)
      Serial.print(",");  // ',' not needed on last one
    if (!(i & 1)) Serial.print(" ");
  }

  // End declaration
  Serial.print("};");  //

  // Comment
  Serial.print("  // ");
  Serial.print(encoding(results));
  Serial.print(" ");
  serialPrintUint64(results->value, 16);

  // Newline
  Serial.println("");

  // Now dump "known" codes
  if (results->decode_type != UNKNOWN) {
    // Some protocols have an address &/or command.
    // NOTE: It will ignore the atypical case when a message has been decoded
    // but the address & the command are both 0.
    if (results->address > 0 || results->command > 0) {
      Serial.print("[IR   ] uint32_t  address = 0x");
      Serial.print(results->address, HEX);
      Serial.println(";");
      Serial.print("[IR   ] uint32_t  command = 0x");
      Serial.print(results->command, HEX);
      Serial.println(";");
    }

    // All protocols have data
    Serial.print("[IR   ] uint64_t  data = 0x");
    serialPrintUint64(results->value, 16);
    Serial.println(";");
  }
}

// ------------------------------------------------------------------------------------------
// Convert string to hex, borrowed from ESPBasic
// ------------------------------------------------------------------------------------------
unsigned long HexToLongInt(String h)
{
  // this function replace the strtol as this function is not able to handle hex numbers greather than 7fffffff
  // I'll take char by char converting from hex to char then shifting 4 bits at the time
  int i;
  unsigned long tmp = 0;
  unsigned char c;
  int s = 0;
  h.toUpperCase();
  for (i = h.length() - 1; i >= 0 ; i--)
  {
    // take the char starting from the right
    c = h[i];
    // convert from hex to int
    c = c - '0';
    if (c > 9)
      c = c - 7;
    // add and shift of 4 bits per each char
    tmp += c << s;
    s += 4;
  }
  return tmp;
}

// ------------------------------------------------------------------------------------------
// Send IR codes to variety of sources
// ------------------------------------------------------------------------------------------
void irblast(String type, String dataStr, unsigned int len, int rdelay, int pulse,
             int pdelay, int repeat, long address, IRsend irsend) {
  Serial.println("[IR   ] Blasting off");
  type.toLowerCase();
  unsigned long data = HexToLongInt(dataStr);
  holdReceive = true;
  Serial.println("[IR   ] Blocking incoming IR signals");
  // Repeat Loop
  for (int r = 0; r < repeat; r++) {
    // Pulse Loop
    for (int p = 0; p < pulse; p++) {
      Serial.print(data, HEX);
      Serial.print(":");
      Serial.print(type);
      Serial.print(":");
      Serial.println(len);
      if (type == "nec")              irsend.sendNEC(data, len);
      else if (type == "sony")        irsend.sendSony(data, len);
      else if (type == "coolix")      irsend.sendCOOLIX(data, len);
      else if (type == "whynter")     irsend.sendWhynter(data, len);
      else if (type == "panasonic") { irsend.sendPanasonic(address, data); Serial.println(address); }
      else if (type == "jvc")         irsend.sendJVC(data, len, 0);
      else if (type == "samsung")     irsend.sendSAMSUNG(data, len);
      else if (type == "sharpRaw")    irsend.sendSharpRaw(data, len);
      else if (type == "dish")        irsend.sendDISH(data, len);
      else if (type == "rc5")         irsend.sendRC5(data, len);
      else if (type == "rc6")         irsend.sendRC6(data, len);
      else if (type == "denon")       irsend.sendDenon(data, len);
      else if (type == "lg")          irsend.sendLG(data, len);
      else if (type == "sharp")       irsend.sendSharpRaw(data, len);
      else if (type == "rcmm")        irsend.sendRCMM(data, len);

      if (p + 1 < pdelay) delay(pdelay);
    }
    if (r + 1 < rdelay) delay(rdelay);
  }

  CountIRsent++;
  Serial.println("[IR   ] Transmission complete");

  copyJsonSend(last_send_4, last_send_5);
  copyJsonSend(last_send_3, last_send_4);
  copyJsonSend(last_send_2, last_send_3);
  copyJsonSend(last_send, last_send_2);

  last_send["data"] = dataStr;
  last_send["len"] = len;
  last_send["type"] = type;
  last_send["address"] = address;
  last_send["time"] = String(timeClient.getFormattedTime());

  resetReceive();
}

// ------------------------------------------------------------------------------------------
// send IR raw code  
// ------------------------------------------------------------------------------------------
void rawblast(JsonArray &raw, int khz, int rdelay, int pulse, int pdelay, int repeat, IRsend irsend) {
  Serial.println("[IR   ] Raw transmit");
  holdReceive = true;
  Serial.println("[IR   ] Blocking incoming IR signals");
  // Repeat Loop
  for (int r = 0; r < repeat; r++) {
    // Pulse Loop
    for (int p = 0; p < pulse; p++) {
      Serial.println("[IR   ] Sending code");
      irsend.enableIROut(khz);
      for (unsigned int i = 0; i < raw.size(); i++) {
        int val = raw[i];
        if (i & 1) irsend.space(std::max(0, val));
        else       irsend.mark(val);
      }
      irsend.space(0);
      if (p + 1 < pdelay) delay(pdelay);
    }
    if (r + 1 < rdelay) delay(rdelay);
  }

  CountIRsent++;
  Serial.println("[IR   ] Transmission complete");

  copyJsonSend(last_send_4, last_send_5);
  copyJsonSend(last_send_3, last_send_4);
  copyJsonSend(last_send_2, last_send_3);
  copyJsonSend(last_send, last_send_2);

  last_send["data"] = NULL;
  last_send["len"] = raw.size();
  last_send["type"] = "RAW";
  last_send["address"] = 0;
  last_send["time"] = String(timeClient.getFormattedTime());

  resetReceive();
}

// ------------------------------------------------------------------------------------------
// copy received IR code info from one Jsaon object to another
// ------------------------------------------------------------------------------------------
void copyJson(JsonObject& j1, JsonObject& j2) {
  if (j1.containsKey("data"))     j2["data"]     = j1["data"];     else j2["data"]     = NULL;
  if (j1.containsKey("encoding")) j2["encoding"] = j1["encoding"]; else j2["encoding"] = "";
  if (j1.containsKey("bits"))     j2["bits"]     = j1["bits"];     else j2["bits"]     = 0;
  if (j1.containsKey("address"))  j2["address"]  = j1["address"];  else j2["address"]  = 0;
  if (j1.containsKey("command"))  j2["command"]  = j1["command"];  else j2["command"]  = 0;
  if (j1.containsKey("rawlen"))   j2["rawlen"]   = j1["rawlen"];   else j2["rawlen"]   = 0;
  if (j1.containsKey("time"))     j2["time"]     = j1["time"];
  if (j1.containsKey("uint16_t")) j2["uint16_t"] = j1["uint16_t"]; else j2["uint16_t"] = NULL;
}

// ------------------------------------------------------------------------------------------
// copy sent IR code info from one Jsaon object to another
// ------------------------------------------------------------------------------------------
void copyJsonSend(JsonObject& j1, JsonObject& j2) {
  if (j1.containsKey("data"))     j2["data"]     = j1["data"];     else j2["data"]     = NULL;
  if (j1.containsKey("type"))     j2["type"]     = j1["type"];     else j2["type"]     = "";
  if (j1.containsKey("len"))      j2["len"]      = j1["len"];      else j2["len"]      = 0;
  if (j1.containsKey("address"))  j2["address"]  = j1["address"];  else j2["address"]  = 0;
  if (j1.containsKey("rawlen"))   j2["rawlen"]   = j1["rawlen"];   else j2["rawlen"]   = 0;
  if (j1.containsKey("time"))     j2["time"]     = j1["time"];
}

// ------------------------------------------------------------------------------------------
// send IR code info via http to a fhem web server
// ------------------------------------------------------------------------------------------
void IRtoFhem() {
  String httpCmd;

  digitalWrite(ledpin, HIGH);         // switch on Status LED for two seconds
  LEDticker.attach(2, LEDoff);
  
  httpCmd = "http://" + FhemIP + ":" + FhemPort + "/fhem?cmd.dummy=set%20" + FhemVarIR + "%20";

  if(FhemFmt!="fhem") {
    httpCmd += "[{";
    if (last_code["encoding"] != "UNKNOWN") {
      httpCmd += "'type':'"    + last_code["encoding"].as<String>() + "',%20";
      httpCmd += "'length':"   + last_code["bits"].as<String>()     + "',%20";
      httpCmd += "'address':'" + last_code["address"].as<String>()  + "',%20";
      httpCmd += "'command':'" + last_code["command"].as<String>()  + "',%20";
      httpCmd += "'data':'"    + last_code["data"].as<String>()     + "'}]&XHR=1";
    } else {
      httpCmd += "'type':'RAW',%20";
      httpCmd += "'data':'"    + last_code["data"].as<String>()     + "',%20";
      httpCmd += "'rawlenh':"   + last_code["rawlen"].as<String>()   + "',%20";
      httpCmd += "'rawbuf':'"    + last_code["uint16_t"].as<String>() + "'}]&XHR=1";
    }
  } else {
    // send in a format suited for fhem
    if (last_code["encoding"] != "UNKNOWN") {
      httpCmd += "type:%20"    + last_code["encoding"].as<String>() + "%20";
      httpCmd += "length:%20"  + last_code["bits"].as<String>()     + "%20";
      httpCmd += "address:%20" + last_code["address"].as<String>()  + "%20";
      httpCmd += "command:%20" + last_code["command"].as<String>()  + "%20";
      httpCmd += "data:%20"    + last_code["data"].as<String>()     + "&XHR=1";
    } else {
      httpCmd += "type:%20RAW%20";
      httpCmd += "data:%20"    + last_code["data"].as<String>()     + "%20";
      httpCmd += "rawlen:%20"  + last_code["rawlen"].as<String>()   + "%20";
      httpCmd += "rawbuf:%20"    + last_code["uint16_t"].as<String>() + "&XHR=1";
    }
  }

  Serial.printf("[HTTP ] IRtoFhem: %s\n", httpCmd.c_str());
  SendHttpCmd(httpCmd);
}

// ------------------------------------------------------------------------------------------
// send temperature & humidity to a fhem web server
// ------------------------------------------------------------------------------------------
void DHTtoFhem() {
  bool error=false;
  bool forceSend=false;
  float Temperature=0.0;
  float Humidity=0.0;
  char sHum[10];
  char sTemp[10];
  String httpCmd;
  unsigned long Now=millis();

  // init result values
  sprintf(sHum, "-");
  sprintf(sTemp, "-");

  // Get temperature event and save its value.
  sensors_event_t event;  
  dht.temperature().getEvent(&event);
  if (isnan(event.temperature)) {
    Serial.println("[DHT22] Error reading temperature!");
    error=true;
  }
  else {
    Serial.print("[DHT22] Temperature: ");
    Serial.print(event.temperature);
    Serial.println("*C");
    Temperature=(float)event.temperature;
    dtostrf(Temperature, 5, 2, sTemp);
  }
  
  // Get humidity event and save its value.
  dht.humidity().getEvent(&event);
  if (isnan(event.relative_humidity)) {
    Serial.println("[DHT22] Error reading humidity!");
    error=true;
  }
  else {
    Serial.print("[DHT22] Humidity   : ");
    Serial.print(event.relative_humidity);
    Serial.println("%");
    Humidity=(float)event.relative_humidity;
    dtostrf(Humidity, 5, 2, sHum);
  }

  if(Now-lastSent>(unsigned long)600000)
    // 10 minutes no change in values --> force send
    forceSend=true;
  
  if(error || forceSend || abs(lastHum-Humidity)>0.5 || abs(lastTemp-Temperature)>0.2) { 
    if(!error) {
      lastHum=Humidity;
      lastTemp=Temperature;
    }
      
    digitalWrite(ledpin, HIGH);         // switch on Status LED for one second
    LEDticker.attach(1, LEDoff);

    httpCmd = "http://" + FhemIP + ":" + FhemPort + "/fhem?cmd.dummy=set%20" + FhemVarTH + "%20";
    httpCmd += "T:%20" + String(sTemp) + "%20H:%20" + String(sHum) + "&XHR=1";
    
    Serial.printf("[HTTP ] DHTtoFhem: %s\n", httpCmd.c_str());
    SendHttpCmd(httpCmd);
    lastSent=Now;
  } 
  else 
    Serial.printf("[DHT22] no change in values --> nothing to transmit\n");  
}

// ------------------------------------------------------------------------------------------
// send status info to fhem web server
// ------------------------------------------------------------------------------------------
void STATUStoFhem() {
  String httpCmd;

  httpCmd = "http://" + FhemIP + ":" + FhemPort + "/fhem?cmd.dummy=set%20" + FhemVarIR + "%20";
  httpCmd += "IRsent:%20" + String(CountIRsent) + "%20IRreceived:%20" + String(CountIRreceived) + "%20";
  httpCmd += "IRtoFhem:%20" + String(CountIRtransferred) + "&XHR=1";
  Serial.printf("[HTTP ] STATUStoFhem: %s\n", httpCmd.c_str());
  SendHttpCmd(httpCmd);
}

// ------------------------------------------------------------------------------------------
// send HTTP command
// ------------------------------------------------------------------------------------------
void SendHttpCmd(String httpCmd) {
  // configure traged server and url
  HTTPClient http;

  http.begin(httpCmd); //HTTP
  int httpCode = http.GET();

  // httpCode will be negative on error
  if (httpCode > 0) {
    // HTTP header has been send and Server response header has been handled
    Serial.printf("[HTTP ] GET... code: %d\n", httpCode);
  } else {
    Serial.printf("[HTTP ] GET... failed, error: %s\n", http.errorToString(httpCode).c_str());
  }
  http.end();
}

// ------------------------------------------------------------------------------------------
// main loop
// ------------------------------------------------------------------------------------------
void loop() {
  String tmp;
  static unsigned long DHTlastReadout=0;
  static unsigned long STATUSlastSent=0;
  HTTPServer.handleClient();
  decode_results  results;                                        // Somewhere to store the results
  if (getTime) timeClient.update();                               // Update the time

  if (irrecv.decode(&results) && !holdReceive) {                  // Grab an IR code
    CountIRreceived++;
    Serial.println("[IR   ] Signal received:");
    fullCode(&results);                                           // Print the singleline value
    dumpCode(&results);                                           // Output the results as source code
    copyJson(last_code_4, last_code_5);                           // Pass
    copyJson(last_code_3, last_code_4);                           // Pass
    copyJson(last_code_2, last_code_3);                           // Pass
    copyJson(last_code, last_code_2);                             // Pass
    codeJson(last_code, &results);                                // Store the results
    last_code["time"] = String(timeClient.getFormattedTime());    // Set the new update time
    irrecv.resume();                                              // Prepare for the next value

    if (FhemMsg) {                                                // if Fhem Messaging enabled
      if(last_code["encoding"].as<String>()!="UNKNOWN") {         // reasonable IR code
        CountIRtransferred++;
        IRtoFhem();                                               // --> send to Fhem
      }
      else                                                        // skip otherwise
        Serial.println("[IR   ] UNKNOWN code --> no message to Fhem");
    }
  }

  unsigned long Now=millis();       
  if(Now-DHTlastReadout>(unsigned long)DHTcycle || DHTlastReadout==0) {                
    // readout DHT sensor if cycle time is over (or never sent before)
    DHTtoFhem();
    DHTlastReadout=Now;
  }

  if(Now-STATUSlastSent>(unsigned long)STATUScycle || STATUSlastSent==0) {
    // send STATUS info if cycle time is over (or never sent before)
    STATUStoFhem();
    STATUSlastSent=Now;
  }
}

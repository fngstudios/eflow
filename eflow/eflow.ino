#include <Arduino.h>


/***************************************************
  This is an example for the Adafruit Thermocouple Sensor w/MAX31855K

  Designed specifically to work with the Adafruit Thermocouple Sensor
  ----> https://www.adafruit.com/products/269

  These displays use SPI to communicate, 3 pins are required to
  interface
  Adafruit invests time and resources providing this open source code,
  please support Adafruit and open-source hardware by purchasing
  products from Adafruit!

  Written by Limor Fried/Ladyada for Adafruit Industries.
  BSD license, all text above must be included in any redistribution
 ****************************************************/

#include <SPI.h>
#include <PID_v1.h>
#include "Jm_MAX31855.h"

#include <ESP8266WiFi.h>
//#include <DNSServer.h>
#include <WiFiClient.h>
#include <ESP8266WebServer.h>
//#include <ESP8266mDNS.h>
#include <ESP8266SSDP.h>
#include <EEPROM.h>
#include "EEPROMAnything.h"
#include <Ticker.h>

// Configuration Start

// These buttons are exposed on the nodemcu dev board
const uint8_t key_user = 16; // What can we do with this button?
const uint8_t key_flash = 16; // If pressed within 5 seconds of power on, enter admin mode

const uint8_t ledHTTP = 4;     // Toggled on HTTP Status
const uint8_t ledCONNECTED = 4; // Toggled on when AP connected

const uint8_t SSR_OUTPUT = 5; // This is where the SSR is connected

// End Pin Assignment


//---------------------------------fng
char SerialBuffer[100];
unsigned char SerialBufferPointer = 0;
char SerialCommand = 0;
unsigned char ZXTicks = 0;
#define ZXCYCLE 100

// This structure should not grow larger than 1024 bytes.
struct settings_t
{
  uint8_t initialized;       // If not "1", then we have not yet initialized with defaults
  char ssid[33];         // One more byte than required; String needs to be null terminated
  char ssidPassword[65]; // One more byte than required; String needs to be null terminated
  uint8_t ipMode; // 0 = Dynamic, 1 = Static
  uint8_t ipAddress[4]; // 255.255.255.255
  uint8_t ipGateway[4]; // 255.255.255.255
  uint8_t ipSubnet[4];  // 255.255.255.255
} settings;

int requestTTL = 120;

const byte DNS_PORT = 53;
ESP8266WebServer server ( 80 );
//DNSServer dnsServer;
Ticker ZXtick;
boolean deviceAdmin = 0;

// Define the LED state for ledHTTP
//   This is used for blinking the LED with a non-blocking method
boolean ledHTTPState = LOW;
unsigned long    ledHTTPStateMills = 0;
long    ledHTTPStateInterval = 250; // How fast to blink the LED

unsigned long secretRandNumber; // We will generate a new secret on startup.



// Pins for thermocouples
#define DO         12
#define CS_A       2
#define CS_B       15
#define CLK        14

//Jm_MAX31855 thermocouple_A(CLK, CS_A, DO);
//Jm_MAX31855 thermocouple_B(CLK, CS_B, DO);
Jm_MAX31855 thermocouple_A(CS_A);
Jm_MAX31855 thermocouple_B(CS_B);



const int numReadings = 5;
int readIndex_A = 0;                // the index of the current reading
int readIndex_B = 0;                // the index of the current reading

//Input: The variable we're trying to control (double)
//Output: The variable that will be adjusted by the pid (double)
//Setpoint: The value we want to Input to maintain (double)
double Setpoint, Input, Output;

//Specify the links and initial tuning parameters
//double Kp = 300, Ki = 0.05, Kd = 20;
//double Kp = 100, Ki = 0.01, Kd = 0;
//double Kp = 30.0, Ki = 0.1, Kd = 14; // This works. There's some overshoot, but it works.
double Kp = 8, Ki = 0.003, Kd = 20;

double Kp_agressive = 40, Ki_agressive = 0.003, Kd_agressive = 10;
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);

uint16_t startup_sec = 120; // Default 90
uint16_t startup_temp = 80;



int WindowSize = 1000; //
unsigned long windowStartTime;


float readings_A[numReadings];      // the readings from the analog input
float readings_B[numReadings];      // the readings from the analog input

float sensorA = 0;
float sensorB = 0;

float sensorTemperature = 0;

unsigned long previousMillis1000 = 0;
unsigned long previousMillis100 = 0;

bool heaterDuty[10] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
int8_t heaterDutyIndex = -1;

/*
 * 0 = Nothing going on
 * 1 = Init
 * 2 = In progress
 */
uint8_t processEnable = 0;

uint16_t safeTemperature = 50; // Don't allow oven to be enabled unless it first cools to this temperature

struct reflowStatsProfile_t
{
  uint16_t sensorA;  // Reserved
  uint16_t sensorB;   // Reserved
  uint16_t Setpoint;   // Reserved
  uint16_t time;   // Reserved
  uint16_t reflowTime;   // Time since start of reflow process
};

const uint16_t reflowStatsProfileLength = 500;
struct reflowStats_t
{
  uint8_t run; // 0 = Unexecuted, 1 = Completed, 2, In Progress, 3 = Aborted, 4 = Abnormal Error
  uint8_t reflowProfilePrevious; // What reflow profile was selected?
  uint8_t reflowProfileNext; // What reflow profile is selected?
  reflowStatsProfile_t profile[reflowStatsProfileLength]; // 900 positions to save up to 1200 seconds (15 minutes). uInt for each temerature sensor, Input and Setpoint.
} ;

reflowStats_t reflowStats;

const uint8_t reflowProfileNameLength = 50;

struct reflowProfile_t
{
  char name[reflowProfileNameLength];                 // Name of profile
  uint16_t sort;                 // Sort index
  uint16_t profileRamp[2];       // Time / Temperature
  uint16_t profilePreheat[2];    // Time / Temperature
  uint16_t profileRampToPeak[2]; // Time / Temperature
  uint16_t profileReflow[2];     // Time / Temperature
  uint16_t profileCooling[2];    // Time / Temperature
  uint16_t profileFinishing[2];  // Time / Temperature -- Ramp down to temperature that would be safe to touch the board.
};

reflowProfile_t reflowProfile[4];

String systemMessage = "";

void setup() {

  Serial.begin(115200);
    SSDP.setSchemaURL("description.xml");
    SSDP.setHTTPPort(80);
    SSDP.setName("fng Reflow Oven");
    SSDP.setSerialNumber("001788102201");
    SSDP.setURL("/");
    SSDP.setModelName("fng Reflow Oven");
    SSDP.setModelNumber("929000226503");
    SSDP.setModelURL("mew.ezver.com.ar");
    SSDP.setManufacturer("fngStudios");
    SSDP.setManufacturerURL("fng.ezver.com.ar");
    SSDP.setDeviceType("upnp:rootdevice");
    SSDP.begin();
  EEPROM.begin(1024); // 512 bytes should be more than enough (famous last words)
  loadSettings();

  pinMode( SSR_OUTPUT, OUTPUT);
  pinMode( key_flash, INPUT_PULLDOWN_16 );

  //-- Start PID Setup
  windowStartTime = millis();

  //initialize the variables we're linked to
  Setpoint = 1;

  //tell the PID to range between 0 and the full window size
  myPID.SetOutputLimits(0, 100);
  //-- END PID Setup

  //turn the PID on
  myPID.SetMode(MANUAL);


//  delay(5000);
  // Set deviceAdmin to one if key_flash is depressed. Otherwise, use defaults.
  if (digitalRead(key_flash)) {
    deviceAdmin = 1;
 //   pinMode( key_flash, OUTPUT );
  } else {
//    pinMode( key_flash, OUTPUT );
  }

  
  if (deviceAdmin) {
    WiFi.mode(WIFI_AP);
    WiFi.softAP("eflow_admin", "eflow_admin");
    //WiFi.mode(WIFI_AP);
    WiFi.config ( IPAddress(192, 168, 4, 1), IPAddress(192, 168, 4, 1), IPAddress(255, 255, 255, 0)) ;
    delay(10);

    // if DNSServer is started with "*" for domain name, it will reply with
    // provided IP to all DNS request
 //   dnsServer.start(DNS_PORT, "*", IPAddress(192, 168, 4, 1));

    //WiFi.printDiag(Serial);
    Serial.println ( "Entering admin mode." );

    Serial.print ( "IP address: " );
    Serial.println ( WiFi.softAPIP() );
    printAPMacAddress();

 //   if ( MDNS.begin ( "eflow" ) ) {
 //     Serial.println ( "MDNS responder started" );
 //   }

    // We are using the amount of time required to connect to the AP as the seed to a random number generator.
    //   We should look for other ways to improve the seed. This should be "good enough" for now.

    server.on ( "/", handleAdminFrameset );
    server.on ( "/leftnav", handleAdminNav );
    server.on ( "/conf/wifi", handleAdminConfWifi );
    server.on ( "/conf/network", handleAdminConfNetwork );
    server.on ( "/conf/accounts", handleAdminConfAccounts );
    server.on ( "/conf/sensors", handleAdminConfSensors );
    server.on ( "/system/defaults", handleAdminDefaults );
    server.on ( "/system/settings", handleAdminSettings );
    server.on ( "/system/restart", handleAdminRestart);
    server.on ( "/system/apply", handleAdminApply);
    server.on ( "/eflow.css", handleCSS);
    server.on("/description.xml", HTTP_GET, [](){
      SSDP.schema(server.client());
    });

    server.onNotFound ( handleNotFound );
    server.begin();
    Serial.println ( "HTTP server started" );

  } else {

    Serial.print("Connecting to SSID : ");
    Serial.println (settings.ssid);
    
    WiFi.begin ( settings.ssid, settings.ssidPassword );
    WiFi.mode ( WIFI_STA );



    // Documentation says this is supposed to come before WiFi.begin, but when it is there -- it doesn't work. WHY?!?!?!
    if (settings.ipMode == 1) { // 0 = Dynamic, 1 = Static
      WiFi.config ( settings.ipAddress, settings.ipGateway, settings.ipSubnet) ;
    }

    //Serial.println ( "" );

    //EEPROM_readAnything(0, settings);

    // Wait for connection
    while ( WiFi.status() != WL_CONNECTED ) {
      delay ( 500 );
      Serial.print ( "." );
    }

    digitalWrite ( ledCONNECTED, 1 );

    WiFi.printDiag(Serial);

    Serial.print ( "IP address: " );
    Serial.println ( WiFi.localIP() );
    printMacAddress();

 //   if ( MDNS.begin ( "eflow" ) ) {
 //     Serial.println ( "MDNS responder started" );
 //   }

    // We are using the amount of time required to connect to the AP as the seed to a random number generator.
    //   We should look for other ways to improve the seed. This should be "good enough" for now.
    randomSeed(micros());
    secretRandNumber = random(2147483646); // Full range of long 2147483647
    Serial.println("Secret: " + String(secretRandNumber));

    //server.on ( "/", handleRoot );
    server.on ( "/", handleReflowFrameset );
    server.on ( "/topnav", handleReflowNav );
    server.on ( "/process/start", handleProcessStart );
    server.on ( "/process/stop", handleProcessStop );
    server.on ( "/process/conf", handleProcessConfigure );
    server.on ( "/process/conf/save/global", handleProcessConfigureSaveGlobal );
    server.on ( "/process/chart", handleReflowChart );
    server.on ( "/process/data.csv", handleProcessData );
    server.on ( "/restart", handleSystemRestart );
    server.on ( "/oven", handleOven );
    server.on ( "/externalScript.js", handleExternalScriptJS );
    server.on ( "/json/sensors", handleJSONSensors );
    server.on ( "/json/oven", handleJSONOven );	
    server.on ( "/eflow.css", handleCSS);
    server.on ( "/blank.html", handleBlank);
    server.on("/description.xml", HTTP_GET, [](){
      SSDP.schema(server.client());
    });
    server.onNotFound ( handleNotFound );
    server.begin();
    Serial.println ( "HTTP server started" );
  }
  ZXtick.attach(0.01, ISR_ZX);
}

void handleRoot2() {
  server.send(200, "text/plain", "hello from esp8266!");
}

void loop() {

  updateSensors();
  yield();
  // Call the timer dispatchers
  dispatchers();
  yield();
  // Start Pid Control
  Input = (sensorA + sensorB) / 2;
  yield();
  myPID.Compute();
  yield();
  // Handle TCP Server
  server.handleClient();
//  dnsServer.processNextRequest();
  yield();
  doSerial();
/*
  if (deviceAdmin) {
    unsigned long ledHTTPCurrentMills = millis();

    if (ledHTTPCurrentMills - ledHTTPStateMills > ledHTTPStateInterval) {
      ledHTTPStateMills = ledHTTPCurrentMills;

      if (ledHTTPState) {
        ledHTTPState = 0;
      } else {
        ledHTTPState = 1;
      }
      digitalWrite( ledCONNECTED, ledHTTPState );
      //Serial.println ( WiFi.softAPIP() );
    }

    // If we've been in admin mode for 30 minutes, reboot ESP to get out of
    //   admin mode.
    if (millis() > 1800000) {
      ESP.reset();
    }
  }
*/


}

void dispatchers ( void ) {
  // Call dispatchSecond once a second
  unsigned long currentMillis1000 = millis();
  if (currentMillis1000 - previousMillis1000 >= 1000) {
    previousMillis1000 = currentMillis1000;

//    dispatchSecond();
    dispatchProcessPerSecond();
  }

/*  // Call dispatch100ms every 100ms (1/10 sec)
  unsigned long currentMillis100 = millis();
  if (currentMillis100 - previousMillis100 >= 100) {
    previousMillis100 = currentMillis100;

    dispatch100ms();
  }*/
}



void dispatchSecond( void ) {
  // We may use this for debug output

}

void doSerial(){
  if (Serial.available()){
   char entrada = Serial.read();
   if (entrada == 'a'){
     Output -=10;
     Serial.println(Output);
   }
   if (entrada == 's'){
     Output +=10;
     Serial.println(Output);
   }
}
}

/*    Paquete Serial:

un paquete de comunicaciones con el display/teclado tiene la siguiente trama:

Inicio(char)/Comando(char)/Data0(int)/Data1(int)/Data2(int)/chksum(char)/Fin(char)
0x04                                                                     0x05     

comando          Data0          Data1          Data2
0xD0             processEnable  x              x
0xD1             Setpoint       Temperatura    Tiempo
*/
/* void wSendData(unsigned char lTarget, unsigned char lSource, unsigned char lComando,unsigned char lData1,unsigned char lData2,unsigned char lData3,unsigned char lData4){
  unsigned char lCHKSUM = lTarget + lSource + lComando + lData1+ lData2+ lData3+ lData4;
    mySerial.write(0x05);    //Inicio
    mySerial.write(lTarget);  //Target
    mySerial.write(lSource);  
    mySerial.write(lComando);
    mySerial.write(lData1);
    mySerial.write(lData2);
    mySerial.write(lData3);
    mySerial.write(lData4);
    mySerial.write(lCHKSUM);
    mySerial.write(0x04);    //Fin
   packet_Sent = 1;
 
  }*/

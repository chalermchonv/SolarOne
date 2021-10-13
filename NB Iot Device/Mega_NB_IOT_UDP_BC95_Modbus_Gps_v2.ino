
#include <avr/wdt.h>
 
// reconnect Udp Server in ? min
#define secMills  1000UL  // in  s * 1000
#define minMills 60*1000UL  // in  s * 1000
#define hrMills  60 * minMills
long reconnectUdpMillis = 5 * hrMills;  // if Lost connection From Udp Server for ? min and then Reboot 
long resetEveryHours = 12*hrMills; // reset time for every
long interval = 30*secMills;  // in  s * 1000


// https://github.com/sparkfun/GPS_Shield/tree/V_2.0/Firmware/Examples/TinyGPSPlus_GPS_Shield
#include <TinyGPS++.h> // Include the TinyGPS++ library
#define GPS_BAUD 9600 // GPS module baud rate. GP3906 defaults to 9600.
#define gpsPort Serial2 // Mega rx 16 Tx17   // GPS TX, Arduino RX pin   // GPS RX, Arduino TX pin
 
TinyGPSPlus tinyGPS; // Create a TinyGPSPlus object

#define modbusRate      9600
#define MAX485_DE      20
#define MAX485_RE_NEG  21
#define maxloop 15

#include <ModbusMaster.h>
#define modbusPort Serial1 // Mega rx 19 Tx18
// instantiate ModbusMaster object
ModbusMaster node1;
ModbusMaster node2;

void preTransmission()
{
  digitalWrite(MAX485_RE_NEG, 1);
  digitalWrite(MAX485_DE, 1);
}

void postTransmission()
{
  digitalWrite(MAX485_RE_NEG, 0);
  digitalWrite(MAX485_DE, 0);
}
  
// NB IOT +++++++++++++++++++++++++
/* 
    " Plug NB-IoT Shield on Arduino Mega "
    1. Jump Pin 8 (RX) NB to Pin 48 Mega 
    2. Jump Pin 9 (TX) NB to Pin 46 Mega
    3 Serail1  Tx 18  Rx 19  lisening from Uno Modusmater_TH_v1      
*/

#include <ArduinoJson.h>   
#include "AIS_BC95_API.h"

String address = "167.71.218.13";    // Your Server IP
String desport = "9999";    // Your Server Port
String inAddress ="167.71.218.13";

String payload = "Hello World";

String data_return;
String receiveJson;
String runTimeStr;
unsigned long previousMillis = 0;
long lastResponseMillis; // lastResponseMillis from UDP Server

byte isReceive = 1;
long cnt=0;
AIS_BC95_API nb;
String imsi;
String icccid;
String imei;
// NB IOT ---------------------


// macros from DateTime.h 
/* Useful Constants */
#define SECS_PER_MIN  (60UL)
#define SECS_PER_HOUR (3600UL)
#define SECS_PER_DAY  (SECS_PER_HOUR * 24L)
 
/* Useful Macros for getting elapsed time */
#define numberOfSeconds(_time_) (_time_ % SECS_PER_MIN)  
#define numberOfMinutes(_time_) ((_time_ / SECS_PER_MIN) % SECS_PER_MIN) 
#define numberOfHours(_time_) (( _time_ % SECS_PER_DAY) / SECS_PER_HOUR)
#define numberOfDays(_time_) ( (_time_ / SECS_PER_DAY)%1  )
#define elapsedDays(_time_) ( _time_ / SECS_PER_DAY) 

String runTime(long val){    
 // int days = elapsedDays(val);  // Maxuim Days
 long days = numberOfDays(val); 
 int hours = numberOfHours(val);
 int minutes = numberOfMinutes(val);
 int seconds = numberOfSeconds(val);    
 return (String(days) + " d " + String(hours) + ':' + String(minutes) + ':' + String(seconds) ) ;  
}
/*
void sendCmdToGetData() {
      String cmd ;
      unsigned long allSeconds=millis()/1000;
      runTimeStr = runTime(allSeconds);
      cmd = "read-" + runTimeStr;
      Serial.println(cmd);   
      Serial1.println(cmd);   
}
*/

float lat;
float lng;
// ===================================================================
// ===================================================================
// ===================================================================
void setup() {
       
     Serial.begin(9600);
                
     init_AIS_NbIot();
     
     Serial.println("start init GPS and Modbus Devices");
      
     delay(1000);

     init_GpsDevice();
      
     init_ModbusDevice();   

      delay(1000);
        
      Serial.print("interval time sec:");
      Serial.println(interval/secMills);
      Serial.print("reconnect Udp Server time min :");
      Serial.println(reconnectUdpMillis/minMills);

      readModbusDateAndSendOut();    
      previousMillis = millis();
      lastResponseMillis = previousMillis;
      //  wdt_reset(); // wdt reset  
      wdt_enable(WDTO_8S);// working morethan 8 s (maximuim) -->> Must Reset befor 8 s  
}
 
// ===================================================================
// ===================================================================
// ===================================================================
void loop() {
  
   // ==========================================
    data_return=""; 
    unsigned long currentMillis = millis();       
    if (currentMillis - previousMillis >= interval)    
    {          
      Serial.println("interval Action");
       cnt++;   
       readModbusDateAndSendOut(); 
       
       // ==========================================
       long lostConnect = (currentMillis - lastResponseMillis);
       Serial.write("#..Reset If lost Connect time (ms) "); Serial.print(lostConnect); Serial.print(" > "); Serial.println(reconnectUdpMillis);
       // Reconnect to UDP Server After Lose connection for 
       if(lostConnect >= reconnectUdpMillis ) { software_Reboot();  }
     
        previousMillis = currentMillis;  
         
    }
    // ==========================================
      nb.waitResponse(data_return,inAddress);    
      if(data_return!=""){         
         wdt_reset(); // wdt reset  
         lastResponseMillis = millis();    // Update 
         data_return = hexToASCII(data_return);
         Serial.println();
         Serial.println("#<<-- UDP From " + inAddress + " : " + data_return );   
         //Serial.println("#<<-- UDP From " + inAddress + " : " + data_return);   
         Serial1.println("From UDP " + inAddress + " : " + data_return); 
           
          if (data_return.startsWith("-read")) readModbusDateAndSendOut();       
          else if (data_return.startsWith("-interval10s")) interval = 10*secMills;       
          else if (data_return.startsWith("-interval20s")) interval = 20*secMills;       
          else if (data_return.startsWith("-interval1m"))  interval = minMills;                     
          else if (data_return.startsWith("-interval5m")) interval = 5*minMills;
                               
      }
  
   // if (millis()>= resetMillis) software_Reboot(); // Reset Every Hours

   
    wdt_reset(); // wdt reset  
}
// ===================================================================
// ===================================================================

void software_Reboot() {
        Serial.println("");
       Serial.println("software_Reboot .....");  
       wdt_enable(WDTO_1S);
      while(1)  {  Serial.print("#"); delay(100); }         
}
 
// NB Iot ++++++++++++++++++++++
void init_AIS_NbIot() {
  nb.begin(address,desport);
      icccid = nb.getICCID();
      imsi = nb.getIMSI();
      imei = nb.getIMEI();
      Serial.print("Device IP : ");    Serial.println(nb.getDeviceIP());
      Serial.print(F(">>IMSI   : "));  Serial.println(imsi);
      Serial.print(F(">>ICCID  : "));  Serial.println(icccid);
      Serial.print(F(">>IMEI   : "));  Serial.println(imei);
      Serial.print(F(">>Signal : "));  Serial.print(nb.getSignal()); 
      Serial.println(F(" dBm")); 
      nb.pingIP(address);  
       
     nb.sendMsgSTR(address,desport,"Start UDP_BC95 " + String(previousMillis));  
}
// NB Iot  --------------------

// GPS +++++++++++++++++++++++
void init_GpsDevice() {

  gpsPort.begin(GPS_BAUD);
  
  Serial.print("Search Gps for 10 times ");
  byte c = 0 ; 
     do {
        c ++;
        GpsSmartDelay(1000);
         Serial.print("#");
        lat = tinyGPS.location.lat();
        lng = tinyGPS.location.lng();
        if (lat > 0) break;      
     } while (c < 10);  
      
      Serial.println("");
      if (lat > 0)  printGPSInfo();         
      else Serial.println("Can not connect GPS");
       
}

void printGPSInfo()
{
  // Print latitude, longitude, altitude in feet, course, speed, date, time,
  // and the number of visible satellites.
  Serial.print("");
  Serial.print("Lat: "); Serial.println(tinyGPS.location.lat(), 6);
  Serial.print("Long: "); Serial.println(tinyGPS.location.lng(), 6);
  Serial.print("Alt: "); Serial.println(tinyGPS.altitude.feet());
  Serial.print("Course: "); Serial.println(tinyGPS.course.deg());
  Serial.print("Speed: "); Serial.println(tinyGPS.speed.mph());
  Serial.print("Date: "); printDate();
  Serial.print("Time: "); printTime();
  Serial.print("Sats: "); Serial.println(tinyGPS.satellites.value());
  Serial.println();
}

// This custom version of delay() ensures that the tinyGPS object
// is being "fed". From the TinyGPS++ examples.
static void GpsSmartDelay(unsigned long ms)
{
  unsigned long start = millis();
  do
  {
    // If data has come in from the GPS module
    while (gpsPort.available())
      tinyGPS.encode(gpsPort.read()); // Send it to the encode function
  // tinyGPS.encode(char) continues to "load" the tinGPS object with new
  // data coming in from the GPS module. As full NMEA strings begin to come in
  // the tinyGPS library will be able to start parsing them for pertinent info
  } while (millis() - start < ms);
}

// printDate() formats the date into dd/mm/yy.
void printDate()
{
  Serial.print(tinyGPS.date.day());
  Serial.print("/");
  Serial.print(tinyGPS.date.month());
  Serial.print("/");
  Serial.println(tinyGPS.date.year());
}

// printTime() formats the time into "hh:mm:ss", and prints leading 0's
// where they're called for.
void printTime()
{
  Serial.print(tinyGPS.time.hour()+7);
  Serial.print(":");
  if (tinyGPS.time.minute() < 10) Serial.print('0');
  Serial.print(tinyGPS.time.minute());
  Serial.print(":");
  if (tinyGPS.time.second() < 10) Serial.print('0');
  Serial.println(tinyGPS.time.second());
}
// GPS ------------------------------


// Modbus +++++++++++++++++++++++++
void init_ModbusDevice() {
   // ===================
    modbusPort.begin(modbusRate);  
    pinMode(MAX485_RE_NEG, OUTPUT);
    pinMode(MAX485_DE, OUTPUT);
    // Init in receive mode
    digitalWrite(MAX485_RE_NEG, 0);
    digitalWrite(MAX485_DE, 0);
        
    // communicate with Modbus slave ID 2 over Serial (port 0)    
      node1.begin(1, Serial1);  
      node1.preTransmission(preTransmission);
      node1.postTransmission(postTransmission);

      node2.begin(2, Serial1);  
      node2.preTransmission(preTransmission);
      node2.postTransmission(postTransmission);
   // ===================
 
}
void readModbusDateAndSendOut() {
        
        byte x = 0;
        byte v = 0;
        StaticJsonDocument<200> doc;        
        doc["runtime"] = runTime(millis()/1000);   
        doc["name"] = "wind01";
        doc["lat"] = lat;            
        doc["lon"] = lng;            
        
        wdt_reset(); // wdt reset                 
        Serial.print("");
        Serial.print("1# HR-0-2 : ");
        do {
                    x++;                    
                    uint8_t result;                           
                    Serial.print(x);
                    
                    delay(50);
                    result = node1.readHoldingRegisters(0, 2);                          
                    if (result == node1.ku8MBSuccess) {                          
                        // Create the JSON document                                              
                        doc["direction"] = node1.getResponseBuffer(0);
                        doc["angle"] = node1.getResponseBuffer(1);                       
                        // Send the JSON document over the "link" serial port                                                                                                      
                          x = maxloop; v ++;
                     }  
                     delay(50);
                     wdt_reset(); // wdt reset  
       } while (x < maxloop  );
       
       x = 0;
       Serial.println("");
       Serial.print("2# HR-0-2 : ");
       do {
                    x++;                    
                    uint8_t result;                           
                    Serial.print(x);
                    
                    delay(50);
                    result = node2.readHoldingRegisters(0, 2);                          
                    if (result == node2.ku8MBSuccess) {                          
                        // Create the JSON document                                              
                        doc["speed"] = node2.getResponseBuffer(0);                        
                        // Send the JSON document over the "link" serial port                                                                                                       
                        x = maxloop;   v ++;
                     }  
                     
                      wdt_reset(); // wdt reset  
       } while (x < maxloop  );
       Serial.println("");
       
       if (v==2) {
              // Read All Modbus Device Completed  and Send data to MQTT Server   
              //doc["imsi"] = imsi ;                      
              payload = "";     
              serializeJson(doc, payload);          
               nb.sendMsgSTR(address,desport,payload);
                                
            
              Serial.println("#-->> MQTT "+ payload);                          
       }       
}

// Modbus ------------------------


// hexToASCII ++++++++++++++++++++++++++
String hexToASCII(String hex)
{    
     int size = hex.length();
     char input[size+2];
     hex.toCharArray(input,size+2);
     char temp[3];    char c;    int index;    int i;    int val;
    
    // initialize the ASCII code string as empty.
    String ascii = "";
   
    for (i = 0; i < size ; i += 2) {
        temp[0] = input[i];  temp[1] = input[i + 1];
        val = ASCIIHexToInt(temp[0]) * 16;      // First Hex digit
        val += ASCIIHexToInt(temp[1]);          // Second hex digit
        c = toascii(val);        
        ascii += c;
    }      
    return ascii;
}

int ASCIIHexToInt(char c)
{
  int ret = 0;

  if ((c >= '0') && (c <= '9'))
    ret = (ret << 4) + c - '0';
  else
    ret = (ret << 4) + toupper(c) - 'A' + 10;

  return ret;
}
// hexToASCII --------------------------

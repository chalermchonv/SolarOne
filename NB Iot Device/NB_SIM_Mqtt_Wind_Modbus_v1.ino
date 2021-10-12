/*
 Supported DEVIO NB-DEVKIT I Board 
    |  Do not use PIN   |
    |      16 TX        |
    |      17 RX        |
    |      4 EINT       |
    |   26 power key    |
    |     27 reset      |

    If you have any questions, please see more details at https://www.facebook.com/AISDEVIO
*/

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

#include <esp_task_wdt.h>
#include <ArduinoJson.h>
#include <SoftwareSerial.h> 
#include <ModbusMaster.h>
#define WDT_TIMEOUT 6
SoftwareSerial modSerial(21,22); // RX, TX  8  9
#define modbusRate      9600
#define MAX485_DE      32
#define MAX485_RE_NEG  33
#define maxloop 15

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




#include "AIS_SIM7020E_API.h"
String address    = "159.138.241.21";               //Your IPaddress or mqtt server url
String serverPort = "1883";               //Your server port
String clientID   = "1940014320554";               //Your client id < 120 characters
String topic      = "aisnbiot/20554";               //Your topic     < 128 characters
String payload    = "Start-aisnbiot/20554!";    //Your payload   < 500 characters
String username   = "";               //username for mqtt server, username <= 100 characters
String password   = "";               //password for mqtt server, password <= 100 characters 
unsigned int subQoS       = 0;
unsigned int pubQoS       = 0;
unsigned int pubRetained  = 0;
unsigned int pubDuplicate = 0;


const long interval = 30000;           //time in millisecond 
unsigned long previousMillis = 0;
const long reconnectUdpMillis = 2*60000;  // in min  
long lastResponseMillis; 

int cnt = 0;

AIS_SIM7020E_API nb;

SoftwareSerial readSerial(21,22); // RX, TX
String receiveJson;
String runTimeStr;
String imsi;
void setup() {
  Serial.begin(9600);
  nb.begin();
  imsi = nb.getIMSI();
   
  readSerial.begin(9600);
  setupMQTT();
  nb.setCallback(callback);   

   // ===================
    pinMode(MAX485_RE_NEG, OUTPUT);
    pinMode(MAX485_DE, OUTPUT);
    // Init in receive mode
    digitalWrite(MAX485_RE_NEG, 0);
    digitalWrite(MAX485_DE, 0);
    
    modSerial.begin(modbusRate);
    
    // communicate with Modbus slave ID 2 over Serial (port 0)    
      node1.begin(1, modSerial);  
      node1.preTransmission(preTransmission);
      node1.postTransmission(postTransmission);

      node2.begin(2, modSerial);  
      node2.preTransmission(preTransmission);
      node2.postTransmission(postTransmission);
   // ===================
   
  Serial.println("Configuring WDT...");
  esp_task_wdt_init(WDT_TIMEOUT, true); //enable panic so ESP32 restarts
  esp_task_wdt_add(NULL); //add current thread to WDT watch
  previousMillis = millis();   
  lastResponseMillis = previousMillis;      
  readModbusDateAndSendOut();   
}
void loop() {   
  nb.MQTTresponse();
  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis >= interval) {
        cnt++;
        connectStatus();
       // nb.publish(topic,payload+String(cnt));      
       // nb.publish(topic, payload, pubQoS, pubRetained, pubDuplicate);      
       // QoS = 0, 1, or 2, retained = 0 or 1, dup = 0 or 1
        previousMillis = currentMillis;  
        readModbusDateAndSendOut();

        // ++++++++++++++++
            long lostConnect = (millis() - lastResponseMillis);
            Serial.write("lost Connect time ");
            Serial.print(lostConnect);      
            Serial.print(" >= ");
            Serial.println(reconnectUdpMillis);
            // Reconnect to UDP Server After Lose connection for 
            if(lostConnect >= reconnectUdpMillis ) ESP.restart();  
        // ++++++++++++++++               
  }

     
   esp_task_wdt_reset(); // wdt reset  
}

//=========== MQTT Function ================
void setupMQTT(){
  if(!nb.connectMQTT(address,serverPort,clientID,username,password)){ 
     Serial.println("\nconnectMQTT");
  }
    nb.subscribe(topic,subQoS);
//  nb.unsubscribe(topic); 
}
void connectStatus(){
    if(!nb.MQTTstatus()){
        if(!nb.NBstatus()){
           Serial.println("reconnectNB ");
           nb.begin();
        }
       Serial.println("reconnectMQ ");
       setupMQTT();
    }   
}
void callback(String &topic,String &payload, String &QoS,String &retained){
  Serial.println("-------------------------------");
  Serial.println("# Message from Topic \""+topic+"\" : "+nb.toString(payload));
  Serial.println("# QoS = "+QoS);
  if(retained.indexOf(F("1"))!=-1){
    Serial.println("# Retained = "+retained);
  }
  // Update lastResponseMillis
  lastResponseMillis = millis();
}

// **************************************************

String runTime(long val){  
  
 // int days = elapsedDays(val);  // Maxuim Days
 
 long days = numberOfDays(val); 
 int hours = numberOfHours(val);
 int minutes = numberOfMinutes(val);
 int seconds = numberOfSeconds(val);
   
 return (String(days) + " d " + String(hours) + ':' + String(minutes) + ':' + String(seconds) ) ;        
  
}

// Not used
void sendCmdToGetData() {
      String cmd ;
      unsigned long allSeconds=millis()/1000;
      runTimeStr = runTime(allSeconds);
      cmd = "read-" + runTimeStr;
      Serial.println("");
      Serial.println("#-->> " + cmd);   
      readSerial.println(cmd);   
}

void readModbusDateAndSendOut() {
        
        byte x = 0;
        byte v = 0;
        StaticJsonDocument<200> doc;
        Serial.print("");
        Serial.print("1# HR-0-2 : ");
        do {
                    x++;                    
                    uint8_t result;                           
                    Serial.print(x);
                    modSerial.listen();
                    delay(50);
                    result = node1.readHoldingRegisters(0, 2);                          
                    if (result == node1.ku8MBSuccess) {                          
                        // Create the JSON document                      
                        doc["timestamp"] = millis();
                        doc["device"] = "wind01";
                        doc["direction"] = node1.getResponseBuffer(0);
                        doc["angle"] = node1.getResponseBuffer(1);                       
                        // Send the JSON document over the "link" serial port                                                                                                      
                          x = maxloop; v ++;
                     }  
                     delay(50);
                     // esp_task_wdt_reset(); // wdt reset  
       } while (x < maxloop  );
       
       x = 0;
       Serial.println("");
       Serial.print("2# HR-0-2 : ");
       do {
                    x++;                    
                    uint8_t result;                           
                    Serial.print(x);
                    modSerial.listen();
                    delay(50);
                    result = node2.readHoldingRegisters(0, 2);                          
                    if (result == node2.ku8MBSuccess) {                          
                        // Create the JSON document                                              
                        doc["speed"] = node2.getResponseBuffer(0);                        
                        // Send the JSON document over the "link" serial port                                                                                                       
                        x = maxloop;   v ++;
                     }  
                     
                     
       } while (x < maxloop  );
       Serial.println("");
       
       if (v==2) {
              // Read All Modbus Device Completed  and Send data to MQTT Server   
              doc["imsi"] = imsi ;
              doc["runtime"] = runTime(millis()/1000);            
              payload = "";     
              serializeJson(doc, payload);          
              connectStatus();
              nb.publish(topic,payload);                    
              //nb.sendMsgSTR(address,desport,payload);
              Serial.println("#-->> MQTT "+ payload);                          
       }       
}

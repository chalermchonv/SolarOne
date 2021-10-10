#include <avr/wdt.h>
#include <ArduinoJson.h>
#include <SoftwareSerial.h>   
#include <ModbusMaster.h>

#define slaveId  1   // Wind Direction  40001
#define slaveId  2   // Wind Speed

SoftwareSerial outSerial(8,9); // RX, TX  8  9
SoftwareSerial modSerial(2,3); // RX, TX  8  9

#define MAX485_DE      5
#define MAX485_RE_NEG  5


ModbusMaster node1;
ModbusMaster node2;
 
const long startMillis = millis();
const byte hourReset = 12;
const long resetMillis = hourReset*60*60*1000; // reset time for every
const long reconnectUdpMillis = 2*60000;  // in min  
long lastResponseMillis; // lastResponseMillis from NB IOT
void preTransmission()
{
 // digitalWrite(MAX485_RE_NEG, 1);
  digitalWrite(MAX485_DE, 1);
}

void postTransmission()
{
 // digitalWrite(MAX485_RE_NEG, 0);
  digitalWrite(MAX485_DE, 0);
}

unsigned long previousMillis = 0;
const long interval = 10000;  //millisecond
byte maxloop = 10 ;

byte isReadData = 0;
String tmpstr2 ;
String strCmd; 

//************************************
void setup()
{
      wdt_enable(WDTO_8S);// working morethan 8 s -->> Reset  
      
     // pinMode(MAX485_RE_NEG, OUTPUT);
      pinMode(MAX485_DE, OUTPUT);
      // Init in receive mode
     // digitalWrite(MAX485_RE_NEG, 0);
      digitalWrite(MAX485_DE, 0);
      Serial.println("start init serial 0");
      Serial.begin(9600);
      
      while (!Serial) {
        Serial.println("loop for init serial 0"); 
      }
      Serial.println("start init software serial");
      
      outSerial.begin(9600);
      modSerial.begin(4800);
       
      node1.begin(1, modSerial);  
      node1.preTransmission(preTransmission);
      node1.postTransmission(postTransmission);

      node2.begin(2, modSerial);  
      node2.preTransmission(preTransmission);
      node2.postTransmission(postTransmission);

         
     previousMillis = millis();
     lastResponseMillis = previousMillis;
}
uint16_t data[2];


//*************************************************
//*************************************************
//*************************************************
void loop()
{
   
    unsigned long currentMillis = millis();
    outSerial.listen();
    if ( currentMillis - previousMillis >= interval ) {

        long lostConnect = (millis() - lastResponseMillis);
        Serial.write("lost Connect time "); Serial.print(lostConnect);      
        Serial.print(" >= ");        Serial.println(reconnectUdpMillis);
  
        // Reconnect to UDP Server After Lose connection for 
        if(lostConnect >= reconnectUdpMillis )  software_Reboot();
        
        previousMillis = currentMillis;
    }
        
    if (outSerial.available() > 0)
    {
       // Reset  lastResponseMillis
       lastResponseMillis = currentMillis;
       strCmd = "";
       strCmd = outSerial.readString();
       Serial.print("<<-- " + strCmd);
       if (strCmd.startsWith("read-")) readModbusDateAndSendOut();                  
    }

    // For testing
    if (Serial.available() )
    {
        strCmd = "";
        strCmd = Serial.readString();
        //  Serial.println("-->> " + strCmd);
        if (strCmd.startsWith("read-")) readModbusDateAndSendOut();   
    }
            
       wdt_reset(); // wdt reset 

       // if (millis() - startMillis >= resetMillis) {  software_Reboot(); }     
}


// ===============================
void software_Reboot() {
       Serial.println("Reboot");  wdt_enable(WDTO_1S);
       while(1)  {  Serial.print("#"); delay(100); }  
}
void readModbusDateAndSendOut() {
        byte x = 0;
        byte v = 0;
        StaticJsonDocument<200> doc;
        do {
                    x++;                    
                    uint8_t result;                           
                    Serial.println("1#get data " + String(x)  );
                    modSerial.listen();
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
                     wdt_reset(); // wdt reset  
       } while (x < maxloop  );
       
           x = 0;
           do {
                    x++;                    
                    uint8_t result;                           
                    Serial.println("2#get data " + String(x)  );
                    modSerial.listen();
                    result = node2.readHoldingRegisters(0, 2);                          
                    if (result == node2.ku8MBSuccess) {                          
                        // Create the JSON document                                              
                        doc["speed"] = node2.getResponseBuffer(0);                        
                        // Send the JSON document over the "link" serial port                                                                                                       
                        x = maxloop;   v ++;
                     }  
                     delay(50);
                     wdt_reset(); // wdt reset  
       } while (x < maxloop  );

       if (v==2) {
            serializeJson(doc, outSerial);                       
            serializeJson(doc, Serial);   
            Serial.println("");   
       }       
}
 

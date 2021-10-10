/* 
         " Plug NB-IoT Shield on Arduino Mega "
    1. Jump Pin 8 (RX) NB to Pin 48 Mega 
    2. Jump Pin 9 (TX) NB to Pin 46 Mega
    3 Serail1  Tx 18  Rx 19  lisening from Uno Modusmater_TH_v1
      
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



#include <avr/wdt.h>
#include <ArduinoJson.h>   
#include "AIS_BC95_API.h"

const long startMillis = millis();
const byte hourReset = 12;
const long resetMillis = hourReset*60*60*1000; // reset time for every

// reconnect Udp Server in ? min
const long interval = 30*1000;  // in  s * 1000
const long reconnectUdpMillis = 2*60000;  // in min  



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

String runTime(long val){  
  
 // int days = elapsedDays(val);  // Maxuim Days
 
 long days = numberOfDays(val); 
 int hours = numberOfHours(val);
 int minutes = numberOfMinutes(val);
 int seconds = numberOfSeconds(val);
  
 String strtime = String(hours) + ':' + String(minutes) + ':' + String(seconds) ;
        strtime = strtime + " d " + String(days);
 return strtime;
  
}

void sendCmdToGetData() {
      String cmd ;
      unsigned long allSeconds=millis()/1000;
      runTimeStr = runTime(allSeconds);
      cmd = "read-" + runTimeStr;
      Serial.println(cmd);   
      Serial1.println(cmd);   
}

// ===================================================================
// ===================================================================
// ===================================================================
void setup() {
      
      
      Serial.begin(9600);
      Serial1.begin(9600);
      
      nb.begin(address,desport);
      Serial.print("Device IP : ");    Serial.println(nb.getDeviceIP());
      Serial.print(F(">>IMSI   : "));  Serial.println(nb.getIMSI());
      Serial.print(F(">>ICCID  : "));  Serial.println(nb.getICCID());
      Serial.print(F(">>IMEI   : "));  Serial.println(nb.getIMEI());
      Serial.print(F(">>Signal : "));  Serial.print(nb.getSignal()); 
      Serial.println(F(" dBm")); 
      nb.pingIP(address);  
      
   
      nb.sendMsgSTR(address,desport,"Start UDP_BC95 " + String(previousMillis));
    
      Serial.println("start init software serial");
      
      
      Serial1.begin(9600);      
      Serial1.print("read");
      
      previousMillis = millis();
      lastResponseMillis = previousMillis;
     //  wdt_reset(); // wdt reset  
     
   wdt_enable(WDTO_8S);// working morethan 8 s (maximuim) -->> Must Reset befor 8 s

       sendCmdToGetData();    

       
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
      
      // ==========================================
      long lostConnect = (millis() - lastResponseMillis);
      Serial.write("lost Connect time ");
      Serial.print(lostConnect);      
      Serial.print(" >= ");
      Serial.println(reconnectUdpMillis);

      // Reconnect to UDP Server After Lose connection for 
      if(lostConnect >= reconnectUdpMillis ) software_Reboot();  
     
      // ========
      cnt++;     
      // send strCmd to Read  with run time in min
      sendCmdToGetData();              
      previousMillis = currentMillis;  
         
      }
    // ==========================================
      nb.waitResponse(data_return,inAddress);    
      if(data_return!=""){
         
         lastResponseMillis = millis();    // Update 
         data_return = hexToASCII(data_return);
         Serial.println();
         Serial.println("#<<-- UDP From " + inAddress + " : " + data_return );   
         //Serial.println("#<<-- UDP From " + inAddress + " : " + data_return);   
         Serial1.println("From UDP " + inAddress + " : " + data_return);   
         
      }
 
    // ==========================================
     if (Serial1.available()) 
      {
          wdt_reset(); // wdt reset  
            // Allocate the JSON document
            // This one must be bigger than for the sender because it must store the strings
            StaticJsonDocument<300> doc;
        
            // Read the JSON document from the "link" serial port
            DeserializationError err = deserializeJson(doc, Serial1);
            
            serializeJson(doc, receiveJson);
            delay(100); Serial1.println("NB<<:"+receiveJson);
            
            if (err == DeserializationError::Ok) 
            {         
              payload = "";
    
               doc["runtime"] = runTimeStr;
              serializeJson(doc, payload);          
              nb.sendMsgSTR(address,desport,payload);
              Serial.println("#-->> UDP "+ payload);
              Serial1.println("-->> "+ payload);
              delay(100); Serial1.println("NB>>:"+ payload);
              
            } 
            else 
            {
              //# Print error to the "debug" serial port
              //  Serial.print("deserializeJson() returned ");
              //  Serial.println(err.c_str());
          
              //# Flush all bytes in the "link" serial port buffer
              //  while (Serial1.available() > 0)Serial1.read();
             
            }
    
       }
        

   // if (millis() - startMillis >= resetMillis) {  software_Reboot(); }
    wdt_reset(); // wdt reset  
}
// ===================================================================
// ===================================================================
// ===================================================================

void software_Reboot() {
       Serial.println("Reboot");  wdt_enable(WDTO_1S);
       while(1)  {  Serial.print("#"); delay(100); }         
}

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

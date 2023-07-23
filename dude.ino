#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <ESPAsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <AsyncElegantOTA.h>
#include <BlynkSimpleEsp8266.h>
#include <Wire.h>
#include "time.h"
#include "DHTesp.h"

#include <OneWire.h>
#include <DallasTemperature.h>

// GPIO where the DS18B20 is connected to
const int oneWireBus = 4;     
const int oneWireBus2 = 12;   
int Ra=25;
int R1= 80 + Ra;
int ECPin= A0;
int ECGround=9;
int ECPower =10;
float PPMconversion=0.5; 
float TemperatureCoef = 0.0187;
float K=2.88;
float EC=0;
float EC25 =0;
int ppm =0;
float raw= 0;
float Vin= 5;
float Vdrop= 0;
float Rc= 0;
float buffer=0;
 

float tempoffset = -1.0;

// Number of temperature devices found
int numberOfDevices;

// We'll use this variable to store a found device address
DeviceAddress tempDeviceAddress; 
DeviceAddress sensor1 = { 0x28, 0x82, 0x20, 0x07, 0xD6, 0x01, 0x3C, 0xF5 };
DeviceAddress sensor2 = { 0x28, 0xC1, 0x59, 0x35, 0x0C, 0x32, 0x20, 0x3D };

// Setup a oneWire instance to communicate with any OneWire devices
OneWire oneWire(oneWireBus);
OneWire oneWire2(oneWireBus2);

// Pass our oneWire reference to Dallas Temperature sensor 
DallasTemperature sensors(&oneWire);
DallasTemperature sensors2(&oneWire2);

#define DHTTYPE DHT11
uint8_t DHTpin = 2;
DHTesp dht;

int LED2pin = 0;
int LED1pin = 5;

const char* ssid = "mikesnet";
const char* password = "springchicken";
char auth[] = "Eg3J3WA0zM3MA7HGJjT_P6uUh73wQ2ed"; //BLYNK

const char* ntpServer = "pool.ntp.org";
const long  gmtOffset_sec = -14400;   //Replace with your GMT offset (seconds)
const int   daylightOffset_sec = 0;  //Replace with your daylight offset (seconds)
float temperatureC;
float tempprobe;

unsigned long millisBlynk = 0;
float humDHT, tempDHT, abshum;
WidgetTerminal terminal(V10);

AsyncWebServer server(80);

BLYNK_WRITE(V10)
{
    if (String("help") == param.asStr()) 
    {
    terminal.println("==List of available commands:==");
    terminal.println("wifi");
    terminal.println("sensors");
    terminal.println("blink");
    terminal.println("tds");
     terminal.println("==End of list.==");
    }
        if (String("wifi") == param.asStr()) 
    {
        terminal.print("Connected to: ");
        terminal.println(ssid);
        terminal.print("IP address:");
        terminal.println(WiFi.localIP());
        terminal.print("Signal strength: ");
        terminal.println(WiFi.RSSI());
    }
    if (String("sensors") == param.asStr()) {
      pollsensors();
    }
    if (String("blink") == param.asStr()) {
    redtoyellow();
    redtoyellow();
    redtoyellow();
     blinkgreen();
    }
     if (String("tds") == param.asStr()) {
            GetEC();         
      PrintReadings();
     }
    
    terminal.flush();

}

void setup(void) {
        pinMode(LED1pin, OUTPUT);
    pinMode(LED2pin, OUTPUT);
    Serial.begin(115200);
    WiFi.mode(WIFI_STA);
    WiFi.setPhyMode(WIFI_PHY_MODE_11B);
    WiFi.begin(ssid, password);
    Serial.println("");

    // Wait for connection
    while (WiFi.status() != WL_CONNECTED) 
    {
      redtoyellow();
      //delay(500);
      Serial.print(".");
    }
    blinkgreen();

      pinMode(ECPin,INPUT);
 pinMode(ECPower,OUTPUT);//Setting pin for sourcing current
  //pinMode(ECGround,OUTPUT);//setting pin for sinking current
  //digitalWrite(ECGround,LOW);//We can leave the ground connected permanantly
  
    sensors.begin();
    sensors2.begin();
    sensors.requestTemperatures(); 
    sensors2.requestTemperatures(); 
    configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);
    
    Serial.print("Connected to ");
    Serial.println(ssid);
    Serial.print("IP address: ");
    Serial.println(WiFi.localIP());
    Blynk.config(auth, IPAddress(192, 168, 50, 197), 8080);
    Blynk.connect();
    terminal.println("**********DUDE***********");
    printLocalTime();
    terminal.print("Connected to ");
    terminal.println(ssid);
    terminal.print("IP address: ");
    terminal.println(WiFi.localIP());
      // Grab a count of devices on the wire
  /*numberOfDevices = sensors.getDeviceCount();
  
  // locate devices on the bus
  terminal.print("Locating devices...");
  terminal.print("Found ");
  terminal.print(numberOfDevices, DEC);
  terminal.println(" devices.");
    // Loop through each device, print out address
  for(int i=0;i<numberOfDevices; i++){
    // Search the wire for address
    if(sensors.getAddress(tempDeviceAddress, i)){
      terminal.print("Found device ");
      terminal.print(i, DEC);
      terminal.print(" with address: ");
      printAddress(tempDeviceAddress);
      terminal.println();
    } else {
      terminal.print("Found ghost device at ");
      terminal.print(i, DEC);
      terminal.print(" but could not detect address. Check power and cabling");
    }
  }*/

    terminal.flush();
    
    server.on("/", HTTP_GET, [](AsyncWebServerRequest *request) {
      request->send(200, "text/plain", "D - Hey sup dude");
    });
    dht.setup(DHTpin, DHTesp::DHT11);
    AsyncElegantOTA.begin(&server);    // Start ElegantOTA
    server.begin();
    Serial.println("HTTP server started");
}

void loop(void) {
    Blynk.run();
    if  (millis() - millisBlynk >= 30000)  //if it's been 30 seconds OR we just booted up, skip the 30 second wait
    {
        millisBlynk = millis();
        pollsensors();
        GetEC();         
      PrintReadings();
        if (humDHT > 0) {Blynk.virtualWrite(V1, humDHT);}
        if (tempDHT > 0) {Blynk.virtualWrite(V2, tempDHT);}
        if (temperatureC > 0) {Blynk.virtualWrite(V3, temperatureC);}
        if (abshum > 0) {Blynk.virtualWrite(V4, abshum);}
        if (tempprobe > 0) {Blynk.virtualWrite(V5, tempprobe);}
    }
}

void printLocalTime()
{
  time_t rawtime;
  struct tm * timeinfo;
  time (&rawtime);
  timeinfo = localtime (&rawtime);
  terminal.print("-");
  terminal.print(asctime(timeinfo));
  terminal.print(" - ");
}

void redtoyellow()
{
     for (int val = 255; val > 0; val--)  {
      digitalWrite(LED1pin, HIGH);
      analogWrite(LED2pin, 255-val);
      delay(2);
   }
   digitalWrite(LED1pin, LOW);
  digitalWrite(LED2pin, LOW);
}

void blinkgreen()
{
  digitalWrite(LED2pin, HIGH);
  delay(500);
    digitalWrite(LED2pin, LOW);
  delay(500);
    digitalWrite(LED2pin, HIGH);
  delay(500);
    digitalWrite(LED2pin, LOW);
  delay(500);
    digitalWrite(LED2pin, HIGH);
  delay(500);
    digitalWrite(LED2pin, LOW);
}

void printAddress(DeviceAddress deviceAddress) {
  for (uint8_t i = 0; i < 8; i++){
    if (deviceAddress[i] < 16) terminal.print("0");
      terminal.print(deviceAddress[i], HEX);
  }
}

void pollsensors()
{
          humDHT = dht.getHumidity();
        tempDHT = dht.getTemperature() + tempoffset;
        printLocalTime();
        sensors.requestTemperatures();
        delay(1000); 
        sensors.requestTemperatures();    
        sensors2.requestTemperatures();
        delay(1000); 
        sensors2.requestTemperatures(); 
        temperatureC = sensors.getTempCByIndex(0);
        tempprobe = sensors2.getTempCByIndex(0);
        abshum = (6.112 * pow(2.71828, ((17.67 * temperatureC)/(temperatureC + 243.5))) * humDHT * 2.1674)/(273.15 + temperatureC);
        terminal.print("T1[V1]: ");
        terminal.print(tempDHT);
        terminal.print(", H1[V2]: ");
        terminal.print(humDHT);
        terminal.print(", T2[V3]: ");
        terminal.print(temperatureC);
        terminal.print(", AH[V4]: ");
        terminal.println(abshum);
        terminal.print(", T3[V5]: ");
        terminal.println(tempprobe);
         terminal.flush();
}

void GetEC(){
 
        sensors2.requestTemperatures();
                sensors2.requestTemperatures();
                tempprobe = sensors2.getTempCByIndex(0);
//*********Reading Temperature Of Solution *******************//


 
 
//************Estimates Resistance of Liquid ****************//
digitalWrite(ECPower,HIGH);
raw= analogRead(ECPin);
raw= analogRead(ECPin);// This is not a mistake, First reading will be low beause if charged a capacitor
digitalWrite(ECPower,LOW);
 
 
 
 
//***************** Converts to EC **************************//
Vdrop= (Vin*raw)/1024.0;
Rc=(Vdrop*R1)/(Vin-Vdrop);
Rc=Rc-Ra; //acounting for Digital Pin Resitance
EC = 1000/(Rc*K);
 
 
//*************Compensating For Temperaure********************//
EC25  =  EC/ (1+ TemperatureCoef*(tempprobe-25.0));
ppm=(EC25)*(PPMconversion*1000);
 
 
;}
//************************** End OF EC Function ***************************//
 
 
 
 
//***This Loop Is called From Main Loop- Prints to serial usefull info ***//
void PrintReadings(){
terminal.print("Rc: ");
terminal.print(Rc);
terminal.print(" EC: ");
terminal.print(EC25);
terminal.print(" Simens  ");
terminal.print(ppm);
terminal.print(" ppm  ");
terminal.print(tempprobe);
terminal.println(" *C ");
 terminal.flush();
 Blynk.virtualWrite(V6, Rc);
 Blynk.virtualWrite(V7, EC25);
 Blynk.virtualWrite(V8, ppm);
 
};

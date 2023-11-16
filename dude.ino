#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <ESPAsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <AsyncElegantOTA.h>
#include <BlynkSimpleEsp8266.h>
#include <Wire.h>
#include "time.h"
#include "Adafruit_SHT31.h"

Adafruit_SHT31 sht31 = Adafruit_SHT31();

#include <Average.h>
#include <OneWire.h>
#include <DallasTemperature.h>

int updateSeconds = 60;
int sampleSeconds = 5;





float tempoffset = 0;





int LED2pin = 0;
int LED1pin = 14;

const char* ssid = "mikesnet";
const char* password = "springchicken";
char auth[] = "Eg3J3WA0zM3MA7HGJjT_P6uUh73wQ2ed"; //BLYNK

const char* ntpServer = "pool.ntp.org";
const long  gmtOffset_sec = -14400;   //Replace with your GMT offset (seconds)
const int   daylightOffset_sec = 0;  //Replace with your daylight offset (seconds)
float temperatureC;
float tempprobe;

unsigned long millisBlynk = 0;
unsigned long millisAvg = 0;
float humDHT, tempDHT, abshum, abshum2, dewpoint, humidex, tempSHT, humSHT;
WidgetTerminal terminal(V10);

AsyncWebServer server(80);

BLYNK_WRITE(V10)
{
    if (String("help") == param.asStr()) 
    {
    terminal.println("==List of available commands:==");
    terminal.println("wifi");

    terminal.println("blink");
    terminal.println("temps");
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


    if (String("blink") == param.asStr()) {
      terminal.println("Blinking...");
    redtoyellow();
    redtoyellow();
    redtoyellow();
     blinkgreen();
    }


        if (String("temps") == param.asStr()) {
               tempSHT = sht31.readTemperature();
       humSHT = sht31.readHumidity();
          terminal.print("TempSHT: ");
          terminal.print(tempSHT);
          terminal.print(", HumSHT: ");
          terminal.println(humSHT);
    }

    terminal.flush();

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


void setup(void) {
        pinMode(LED1pin, OUTPUT);
    pinMode(LED2pin, OUTPUT);
    Serial.begin(115200);
    WiFi.mode(WIFI_STA);
    WiFi.setPhyMode(WIFI_PHY_MODE_11B);
    WiFi.begin(ssid, password);
    Serial.println("");
    sht31.begin(0x44);
    // Wait for connection
    while (WiFi.status() != WL_CONNECTED) 
    {
      redtoyellow();
      //delay(500);
      Serial.print(".");
    }
    blinkgreen();


    configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);
    
    Serial.print("Connected to ");
    Serial.println(ssid);
    Serial.print("IP address: ");
    Serial.println(WiFi.localIP());
    Blynk.config(auth, IPAddress(192, 168, 50, 197), 8080);
    Blynk.connect();
    terminal.println("**********DUDE v0.5***********");
    printLocalTime();
    terminal.print("Connected to ");
    terminal.println(ssid);
    terminal.print("IP address: ");
    terminal.println(WiFi.localIP());

 
    
    server.on("/", HTTP_GET, [](AsyncWebServerRequest *request) {
      request->send(200, "text/plain", "D - Hey sup dude");
    });


    AsyncElegantOTA.begin(&server);    // Start ElegantOTA
    server.begin();
    terminal.println("HTTP server started");
       terminal.flush();
       tempSHT = sht31.readTemperature();
       humSHT = sht31.readHumidity();
}

void loop(void) {
    Blynk.run();
    if  (millis() - millisBlynk >= (updateSeconds * 1000))  //if it's been 30 seconds
    {
        millisBlynk = millis();
        tempSHT = sht31.readTemperature();
       humSHT = sht31.readHumidity();
        abshum = (6.112 * pow(2.71828, ((17.67 * tempSHT)/(tempSHT + 243.5))) * humSHT * 2.1674)/(273.15 + tempSHT);
        dewpoint = tempSHT - ((100 - humSHT)/5);
        humidex = tempSHT + 0.5555 * (6.11 * pow(2.71828, 5417.7530*( (1/273.16) - (1/(273.15 + dewpoint)) ) ) - 10);
        if (abshum > 0) {Blynk.virtualWrite(V4, abshum);}
        if (dewpoint > 0) {Blynk.virtualWrite(V5, dewpoint);}
        if (humidex > 0) {Blynk.virtualWrite(V6, humidex);}
        //Blynk.virtualWrite(V7, wifiAvg.mean());
        Blynk.virtualWrite(V8, tempSHT);
        Blynk.virtualWrite(V9, humSHT);
    }
}

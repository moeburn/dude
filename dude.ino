#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <ESPAsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <ESPAsyncWiFiManager.h> 
#include <AsyncElegantOTA.h>
#include <BlynkSimpleEsp8266.h>
#include <Wire.h>
#include "time.h"
   

#include <Average.h>
#if defined(ARDUINO_ARCH_ESP32) || (ARDUINO_ARCH_ESP8266)
#include <EEPROM.h>
#define USE_EEPROM
#endif
#include <bsec2.h>
#include "config/bme680_iaq_33v_3s_28d/bsec_iaq.h"


#include <SoftwareSerial.h>
char output[256];

SoftwareSerial SoftSerial1(14, 12);
#include <NOxGasIndexAlgorithm.h>
#include <SensirionI2CSgp41.h>
#include <SensirionI2cSht4x.h>
#include <VOCGasIndexAlgorithm.h>

SensirionI2cSht4x sht4x;
SensirionI2CSgp41 sgp41;

VOCGasIndexAlgorithm voc_algorithm;
NOxGasIndexAlgorithm nox_algorithm;

// time in seconds needed for NOx conditioning
uint16_t conditioning_s = 10;

char errorMessage[256];

//#include "Adafruit_SHT4x.h"
#include <Adafruit_SCD30.h>
Adafruit_SCD30  scd30;
#define SCD_OFFSET 210

#define SD_CS 5
String dataMessage;

#define every(interval) \
    static uint32_t __every__##interval = millis(); \
    if (millis() - __every__##interval >= interval && (__every__##interval = millis()))


//Adafruit_SHT4x sht4 = Adafruit_SHT4x();
 // sensors_event_t humidity, temp;

#define ONE_WIRE_BUS 14                          //PIN of the Maxim DS18B20 temperature sensor
#define TIME_INTERVAL 15000

/*OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature dallasTemp(&oneWire);
NonBlockingDallas sensorDs18b20(&dallasTemp); */

/* Uncomment according to your sensortype. */
#define DHT_SENSOR_TYPE DHT_TYPE_11
//#define DHT_SENSOR_TYPE DHT_TYPE_21
//#define DHT_SENSOR_TYPE DHT_TYPE_22

static const int DHT_SENSOR_PIN = 2;
//DHT_nonblocking dht_sensor( DHT_SENSOR_PIN, DHT_SENSOR_TYPE );

int updateSeconds = 60;
int sampleSeconds = 5;

float tempSCD, humSCD, co2SCD, gasBME, presBME, bmeiaq, bmeiaqAccuracy, bmestaticIaq, bmeco2Equivalent, bmebreathVocEquivalent, bmestabStatus, bmerunInStatus, bmegasPercentage, dallastemp, tempSHT, humSHT;

#define STATE_SAVE_PERIOD UINT32_C(720 * 60 * 1000) /* 360 minutes - 4 times a day */
#define PANIC_LED 2
#define ERROR_DUR 1000
#define tempoffset 1.4

/* Helper functions declarations */
/**
 * @brief : This function toggles the led continuously with one second delay
 */
void errLeds(void);

/**
 * @brief : This function checks the BSEC status, prints the respective error code. Halts in case of error
 * @param[in] bsec  : Bsec2 class object
 */
void checkBsecStatus(Bsec2 bsec);

/**
 * @brief : This function updates/saves BSEC state
 * @param[in] bsec  : Bsec2 class object
 */
void updateBsecState(Bsec2 bsec);

/**
 * @brief : This function is called by the BSEC library when a new output is available
 * @param[in] input     : BME68X sensor data before processing
 * @param[in] outputs   : Processed BSEC BSEC output data
 * @param[in] bsec      : Instance of BSEC2 calling the callback
 */
void newDataCallback(const bme68xData data, const bsecOutputs outputs, Bsec2 bsec);

/**
 * @brief : This function retrieves the existing state
 * @param : Bsec2 class object
 */
bool loadState(Bsec2 bsec);

/**
 * @brief : This function writes the state into EEPROM
 * @param : Bsec2 class object
 */
bool saveState(Bsec2 bsec);


Bsec2 envSensor;
#ifdef USE_EEPROM
static uint8_t bsecState[BSEC_MAX_STATE_BLOB_SIZE];
#endif
/* Gas estimate names will be according to the configuration classes used */
const String gasName[] = { "Field Air", "Hand sanitizer", "Undefined 3", "Undefined 4"};






int LED2pin = 0;
int LED1pin = 14;

const char* ssid = "mikesnet";
const char* password = "springchicken";
char auth[] = "Eg3J3WA0zM3MA7HGJjT_P6uUh73wQ2ed"; //BLYNK
char remoteAuth2[] = "pO--Yj8ksH2fjJLMW6yW9trkHBhd9-wc"; //indiana clock auth
char remoteAuth3[] = "qS5PQ8pvrbYzXdiA4I6uLEWYfeQrOcM4"; //indiana clock auth
char remoteAuth4[] = "eT_7FL7IUpqonthsAr-58uTK_-su_GYy"; //wsbedroom auth

const char* ntpServer = "pool.ntp.org";
const long  gmtOffset_sec = -18000;   //Replace with your GMT offset (seconds)
const int   daylightOffset_sec = 0;  //Replace with your daylight offset (seconds)
float temperatureC;
float tempprobe;

unsigned long millisBlynk = 0;
unsigned long millisAvg = 0;
float humDHT, tempDHT, abshum, abshum2, dewpoint, humidex, tempBME, humBME;
WidgetTerminal terminal(V10);

AsyncWebServer server(80);
DNSServer dns;
/*void handleTemperatureChange(float temperature, bool valid, int deviceIndex){
dallastemp = temperature;
}*/

/*static bool measure_environment( float *temperature, float *humidity )
{
  static unsigned long measurement_timestamp = millis( );


  if( millis( ) - measurement_timestamp > 4000ul )
  {
    if( dht_sensor.measure( temperature, humidity ) == true )
    {
      measurement_timestamp = millis( );
      return( true );
    }
  }

  return( false );
}*/


WidgetBridge bridge2(V60);
WidgetBridge bridge3(V70);
WidgetBridge bridge4(V100);

BLYNK_CONNECTED() {
  bridge2.setAuthToken (remoteAuth2);
  bridge3.setAuthToken (remoteAuth3);
  bridge4.setAuthToken (remoteAuth4);
}
unsigned int up3, up5, up10, up25, up50, up100;

BLYNK_WRITE(V10)
{
    if (String("help") == param.asStr()) 
    {
    terminal.println("==List of available commands:==");
    terminal.println("wifi");
    terminal.println("bsec");
    terminal.println("temps");
    terminal.println("erase");
    terminal.println("recal");
    terminal.println("scd");
    terminal.println("particles");
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



        if (String("temps") == param.asStr()) {
          terminal.print("TempBME: ");
          terminal.print(tempBME);
          terminal.print(", HumBME: ");
          terminal.print(humBME);
          terminal.print(", TempDHT: ");
          terminal.print(tempDHT);
          terminal.print(", HumDHT: ");
          terminal.print(humDHT);
          terminal.print(", Dasllastemp: ");
          terminal.print(dallastemp);
          //sht4.getEvent(&humidity, &temp);
         // tempSHT = //temp.temperature;
          //humSHT = //humidity.relative_humidity;
          terminal.print(", TempSHT: ");
          terminal.print(tempSHT);
          terminal.print(", HumSHT: ");
          terminal.print(humSHT);
        terminal.print(", tempSCD =");
        terminal.print(tempSCD);
        terminal.print(", humSCD =");
        terminal.print(humSCD);
        terminal.print(", CO2 =");
        terminal.println(co2SCD);
    }
    if (String("bsec") == param.asStr()) {
        terminal.print("bmeiaq[v23],bmeiaqAccuracy[v24],bmestaticIaq[v25],bmeco2Equivalent[v26],bmebreathVocEquivalent[v27],bmestabStatus[v28],bmerunInStatus[v29],bmegasPercentage[v30]:");
        terminal.print(bmeiaq);
        terminal.print(",,,");
        terminal.print(bmeiaqAccuracy);
        terminal.print(",,,");
        terminal.print(bmestaticIaq);
        terminal.print(",,,");
        terminal.print(bmeco2Equivalent);
        terminal.print(",,,");
        terminal.print(bmebreathVocEquivalent);
        terminal.print(",,,");
        terminal.print(bmestabStatus);
        terminal.print(",,,");
        terminal.print(bmerunInStatus);
        terminal.print(",,,");
        terminal.println(bmegasPercentage);
    }
        if (String("erase") == param.asStr()) {
      terminal.println("Erasing EEPROM");

      for (uint8_t i = 0; i <= BSEC_MAX_STATE_BLOB_SIZE; i++)
      EEPROM.write(i, 0);

      EEPROM.commit();
      terminal.println("Erase complete.");
      terminal.flush();
    }
        if (String("recal") == param.asStr()) {
      if (!scd30.forceRecalibrationWithReference(420)){
        terminal.println("Failed to force recalibration with reference");
      }
      else {terminal.println("> Recalibrated CO2 sensor.");}
      terminal.flush();
    }
    if (String("scd") == param.asStr()) {
      if (!scd30.startContinuousMeasurement(int(presBME))){
        terminal.println("Failed to set ambient pressure offset");
        terminal.flush();
      }
      terminal.print("Measurement interval: ");
      terminal.print(scd30.getMeasurementInterval());
      terminal.println(" seconds");
      terminal.print("Ambient pressure offset: ");
      terminal.print(scd30.getAmbientPressureOffset());
      terminal.println(" mBar");
      terminal.print("Temperature offset: ");
      terminal.print((float)scd30.getTemperatureOffset()/100.0);
      terminal.println(" degrees C");
      terminal.print("Forced Recalibration reference: ");
      terminal.print(scd30.getForcedCalibrationReference());
      terminal.println(" ppm");
      terminal.flush();
    }

    terminal.flush();

}



void printLocalTime()
{
  time_t rawtime;
  struct tm * timeinfo;
  time (&rawtime);
  timeinfo = localtime (&rawtime);
  terminal.print(asctime(timeinfo));
  terminal.flush();
}


void errLeds(void)
{
    //while(1)
    //{
        digitalWrite(2, HIGH);
        delay(ERROR_DUR);
        digitalWrite(2, LOW);
        delay(ERROR_DUR);
    //}
}

void updateBsecState(Bsec2 bsec)
{
    static uint16_t stateUpdateCounter = 0;
    bool update = false;

    if (!stateUpdateCounter || (stateUpdateCounter * STATE_SAVE_PERIOD) < millis())
    {
        /* Update every STATE_SAVE_PERIOD minutes */
        update = true;
        stateUpdateCounter++;
    }

    if (update && !saveState(bsec))
        checkBsecStatus(bsec);
}

void newDataCallback(const bme68xData data, const bsecOutputs outputs, Bsec2 bsec)
{
    if (!outputs.nOutputs)
        return;

    Serial.println("BSEC outputs:\n\ttimestamp = " + String((int) (outputs.output[0].time_stamp / INT64_C(1000000))));
    for (uint8_t i = 0; i < outputs.nOutputs; i++)
    {
        const bsecData output  = outputs.output[i];
        switch (output.sensor_id)
        {
            case BSEC_OUTPUT_RAW_GAS:
                gasBME = (1 / (output.signal / 1000.0)) * 10;
                break;
            case BSEC_OUTPUT_SENSOR_HEAT_COMPENSATED_TEMPERATURE:
                tempBME = output.signal;
                break;
            case BSEC_OUTPUT_SENSOR_HEAT_COMPENSATED_HUMIDITY:
                humBME = output.signal;
                break;
            case BSEC_OUTPUT_IAQ:
            bmeiaq = output.signal;
            bmeiaqAccuracy = output.accuracy;
                break;
            case BSEC_OUTPUT_STATIC_IAQ:
            bmestaticIaq = output.signal;
                break;
            case BSEC_OUTPUT_CO2_EQUIVALENT:
            bmeco2Equivalent = output.signal;
                break;
            case BSEC_OUTPUT_BREATH_VOC_EQUIVALENT:
            bmebreathVocEquivalent = output.signal;
                break;
            case BSEC_OUTPUT_RAW_PRESSURE:
            presBME = (output.signal / 100.0);
                break;
            case BSEC_OUTPUT_STABILIZATION_STATUS:
            bmestabStatus = output.signal;
                break;
            case BSEC_OUTPUT_RUN_IN_STATUS:
            bmerunInStatus = output.signal;
                break;
            case BSEC_OUTPUT_GAS_PERCENTAGE:
            bmegasPercentage = output.signal;
                break;

            default:
                break;
        }
    }

    updateBsecState(envSensor);
}

void checkBsecStatus(Bsec2 bsec)
{
    if (bsec.status < BSEC_OK)
    {
        Serial.println("BSEC error code : " + String(bsec.status));
        errLeds(); /* Halt in case of failure */
    } else if (bsec.status > BSEC_OK)
    {
        Serial.println("BSEC warning code : " + String(bsec.status));
    }

    if (bsec.sensor.status < BME68X_OK)
    {
        Serial.println("BME68X error code : " + String(bsec.sensor.status));
        errLeds(); /* Halt in case of failure */
    } else if (bsec.sensor.status > BME68X_OK)
    {
        Serial.println("BME68X warning code : " + String(bsec.sensor.status));
    }
}


bool loadState(Bsec2 bsec)
{
#ifdef USE_EEPROM
    

    if (EEPROM.read(0) == BSEC_MAX_STATE_BLOB_SIZE)
    {
        /* Existing state in EEPROM */
        Serial.println("Reading state from EEPROM");
        Serial.print("State file: ");
        for (uint8_t i = 0; i < BSEC_MAX_STATE_BLOB_SIZE; i++)
        {
            bsecState[i] = EEPROM.read(i + 1);
            Serial.print(String(bsecState[i], HEX) + ", ");
        }
        Serial.println();

        if (!bsec.setState(bsecState))
            return false;
    } else
    {
        /* Erase the EEPROM with zeroes */
        Serial.println("Erasing EEPROM");

        for (uint8_t i = 0; i <= BSEC_MAX_STATE_BLOB_SIZE; i++)
            EEPROM.write(i, 0);

        EEPROM.commit();
    }
#endif
    return true;
}

bool saveState(Bsec2 bsec)
{
#ifdef USE_EEPROM
    if (!bsec.getState(bsecState))
        return false;

    Serial.println("Writing state to EEPROM");
    Serial.print("State file: ");

    for (uint8_t i = 0; i < BSEC_MAX_STATE_BLOB_SIZE; i++)
    {
        EEPROM.write(i + 1, bsecState[i]);
        Serial.print(String(bsecState[i], HEX) + ", ");
    }
    Serial.println();

    EEPROM.write(0, BSEC_MAX_STATE_BLOB_SIZE);
    EEPROM.commit();
#endif
    return true;
}




    uint16_t error;
    float humidity = 0;     // %RH
    float temperature = 0;  // degreeC
    uint16_t srawVoc = 0;
    uint16_t srawNox = 0;
    uint16_t defaultCompenstaionRh = 0x8000;  // in ticks as defined by SGP41
    uint16_t defaultCompenstaionT = 0x6666;   // in ticks as defined by SGP41
    uint16_t compensationRh = 0;              // in ticks as defined by SGP41
    uint16_t compensationT = 0;               // in ticks as defined by SGP41
            int32_t voc_index;
        int32_t nox_index;

void setup(void) {
  pinMode(1, INPUT_PULLUP);
  pinMode(3, INPUT_PULLUP);
  pinMode(9, INPUT_PULLUP);
  pinMode(10, INPUT_PULLUP);
    for(int i=12; i<=16; i++) {
    pinMode(i, INPUT_PULLUP);
  }
  Serial.begin(9600);
  //SoftSerial1.begin(9600);


    WiFi.mode(WIFI_STA);
    WiFi.setPhyMode(WIFI_PHY_MODE_11B);
    AsyncWiFiManager wifiManager(&server,&dns);
    wifiManager.autoConnect("Dude Setup");
    //WiFi.begin(ssid, password);
    Serial.println("");
    // Wait for connection
    while (WiFi.status() != WL_CONNECTED) 
    {
      delay(250);
      Serial.print(".");
    }

  //sensorDs18b20.begin(NonBlockingDallas::resolution_12, NonBlockingDallas::unit_C, TIME_INTERVAL);

  //sensorDs18b20.onTemperatureChange(handleTemperatureChange);
    configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);
    
    Serial.print("Connected to ");
    Serial.println(ssid);
    Serial.print("IP address: ");
    Serial.println(WiFi.localIP());
    Blynk.config(auth, IPAddress(192, 168, 50, 197), 8080);
    Blynk.connect();
    Serial.println("Blynk connected");
  /*terminal.println("Adafruit SHT4x test");
  if (! sht4.begin()) {
    terminal.println("Couldn't find SHT4x");
    //while (1) delay(1);
  }
  Serial.println("SHT connected");
  terminal.println("Found SHT4x sensor");
  terminal.print("terminal number 0x");
  terminal.println(sht4.readSerial(), HEX);
  sht4.setPrecision(SHT4X_HIGH_PRECISION);
  sht4.setHeater(SHT4X_NO_HEATER);*/
      Wire.begin();

    sht4x.begin(Wire, SHT40_I2C_ADDR_44);
    sgp41.begin(Wire);
 
    
    server.on("/", HTTP_GET, [](AsyncWebServerRequest *request) {
      request->send(200, "text/plain", "D - Hey sup dude");
    });


    AsyncElegantOTA.begin(&server);    // Start ElegantOTA
    server.begin();
    terminal.println("HTTP server started");
    Serial.println("HTTP connected");
       terminal.flush();

    bsecSensor sensorList[13] = {
    BSEC_OUTPUT_IAQ,
    BSEC_OUTPUT_STATIC_IAQ,
    BSEC_OUTPUT_CO2_EQUIVALENT,
    BSEC_OUTPUT_BREATH_VOC_EQUIVALENT,
    BSEC_OUTPUT_RAW_TEMPERATURE,
    BSEC_OUTPUT_RAW_PRESSURE,
    BSEC_OUTPUT_RAW_HUMIDITY,
    BSEC_OUTPUT_RAW_GAS,
    BSEC_OUTPUT_STABILIZATION_STATUS,
    BSEC_OUTPUT_RUN_IN_STATUS,
    BSEC_OUTPUT_SENSOR_HEAT_COMPENSATED_TEMPERATURE,
    BSEC_OUTPUT_SENSOR_HEAT_COMPENSATED_HUMIDITY,
    BSEC_OUTPUT_GAS_PERCENTAGE
    };
  
    #ifdef USE_EEPROM
    EEPROM.begin(BSEC_MAX_STATE_BLOB_SIZE + 1);
  #endif
   // Wire.begin();
    pinMode(PANIC_LED, OUTPUT);

    /* Valid for boards with USB-COM. Wait until the port is open */
    while (!Serial) delay(10);
   
    /* Initialize the library and interfaces */
    if (!envSensor.begin(BME68X_I2C_ADDR_HIGH, Wire))
    {
        checkBsecStatus(envSensor);
    }

    /* Load the configuration string that stores information on how to classify the detected gas */
    if (!envSensor.setConfig(bsec_config_iaq))
    {
        checkBsecStatus (envSensor);
    }

    /* Copy state from the EEPROM to the algorithm */
    if (!loadState(envSensor))
    {
        checkBsecStatus (envSensor);
    }

    /* Subscribe for the desired BSEC2 outputs */
    if (!envSensor.updateSubscription(sensorList, 13, BSEC_SAMPLE_RATE_LP))
    {
        checkBsecStatus (envSensor);
    }
    envSensor.setTemperatureOffset(tempoffset);
    /* Whenever new data is available call the newDataCallback function */
    envSensor.attachCallback(newDataCallback);

  String output = "\nBSEC library version " + String(envSensor.version.major) + "." + String(envSensor.version.minor) + "." + String(envSensor.version.major_bugfix) + "." + String(envSensor.version.minor_bugfix);
                //terminal.clear();
    terminal.println("**********DUDE v0.8***********");
    terminal.println(output);
    printLocalTime();
    terminal.print("Connected to ");
    terminal.println(ssid);
    terminal.print("IP address: ");
    terminal.println(WiFi.localIP());
  if (!scd30.begin()) {
    terminal.println("Failed to find SCD30 chip");
    terminal.flush();
  }
  else
  {
    scd30.setTemperatureOffset(SCD_OFFSET); // subtract 2 degrees
    terminal.print("Measurement interval: ");
    terminal.print(scd30.getMeasurementInterval());
    terminal.println(" seconds");
    terminal.print("Ambient pressure offset: ");
    terminal.print(scd30.getAmbientPressureOffset());
    terminal.println(" mBar");
    terminal.print("Temperature offset: ");
    terminal.print((float)scd30.getTemperatureOffset()/100.0);
    terminal.println(" degrees C");
    terminal.print("Forced Recalibration reference: ");
    terminal.print(scd30.getForcedCalibrationReference());
    terminal.println(" ppm");
    terminal.flush();
  }
  Serial.println("Setup complete");
}

void loop(void) {
  //sensorDs18b20.update();
      if (!envSensor.run()) {
        checkBsecStatus (envSensor);
    }
  if (scd30.dataReady()) {
        if (!scd30.read()){ 
      terminal.println("Error reading CO2 sensor data"); 
      return; 
    }
    tempSCD = scd30.temperature;
    humSCD = scd30.relative_humidity;
    co2SCD = scd30.CO2;
  }

  every(1000){


    // 3. Measure SGP4x signals
    if (conditioning_s > 0) {
        // During NOx conditioning (10s) SRAW NOx will remain 0
        error =
            sgp41.executeConditioning(compensationRh, compensationT, srawVoc);
        conditioning_s--;
    } else {
        error = sgp41.measureRawSignals(compensationRh, compensationT, srawVoc,
                                        srawNox);
    }

    // 4. Process raw signals by Gas Index Algorithm to get the VOC and NOx
    // index
    //    values
    if (error) {
        Serial.print("SGP41 - Error trying to execute measureRawSignals(): ");
        errorToString(error, errorMessage, 256);
        Serial.println(errorMessage);
    } else {
         voc_index = voc_algorithm.process(srawVoc);
         nox_index = nox_algorithm.process(srawNox);
        Serial.print("VOC Index: ");
        Serial.print(voc_index);
        Serial.print("\t");
        Serial.print("NOx Index: ");
        Serial.println(nox_index);
    }
  }

  every(15000){
            error = sht4x.measureHighPrecision(temperature, humidity);
    if (error) {
        terminal.print(
            "SHT4x - Error trying to execute measureHighPrecision(): ");
        errorToString(error, errorMessage, 256);
        terminal.println(errorMessage);
        terminal.println("Fallback to use default values for humidity and "
                       "temperature compensation for SGP41");
        compensationRh = defaultCompenstaionRh;
        compensationT = defaultCompenstaionT;
        terminal.flush();
    } else {
        Serial.print("T:");
        Serial.print(temperature);
        Serial.print("\t");
        Serial.print("RH:");
        Serial.println(humidity);

        // convert temperature and humidity to ticks as defined by SGP41
        // interface
        // NOTE: in case you read RH and T raw signals check out the
        // ticks specification in the datasheet, as they can be different for
        // different sensors
        tempSHT = temperature;
        humSHT = humidity;
        compensationT = static_cast<uint16_t>((temperature + 45) * 65535 / 175);
        compensationRh = static_cast<uint16_t>(humidity * 65535 / 100);
    }
  }

  every (120000)
  {
       if (!scd30.startContinuousMeasurement(int(presBME))){
     terminal.println("Failed to set ambient pressure offset");
     terminal.flush();

   }
  }

  /* Measure temperature and humidity.  If the functions returns
     true, then a measurement is available. */
  /*if( measure_environment( &dtemperature, &dhumidity ) == true )
  {
    tempDHT = dtemperature;
    humDHT = dhumidity;
  }*/
  if (WiFi.status() == WL_CONNECTED) {Blynk.run();} 

    if  (millis() - millisBlynk >= (updateSeconds * 1000))  //if it's been 30 seconds
    {
        millisBlynk = millis();
          //sht4.getEvent(&humidity, &temp);
         // tempSHT = temp.temperature;
         // humSHT = humidity.relative_humidity;
        abshum = (6.112 * pow(2.71828, ((17.67 * tempSHT)/(tempSHT + 243.5))) * humSHT * 2.1674)/(273.15 + tempSHT);
        //abshum2 = (6.112 * pow(2.71828, ((17.67 * dallastemp)/(dallastemp + 243.5))) * humDHT * 2.1674)/(273.15 + dallastemp);
        dewpoint = tempSHT - ((100 - humSHT)/5);
        humidex = tempSHT + 0.5555 * (6.11 * pow(2.71828, 5417.7530*( (1/273.16) - (1/(273.15 + dewpoint)) ) ) - 10);
        if (abshum > 0) {Blynk.virtualWrite(V4, abshum);}
        if (dewpoint > 0) {Blynk.virtualWrite(V5, dewpoint);}
        if (humidex > 0) {Blynk.virtualWrite(V6, humidex);}
        //Blynk.virtualWrite(V7, wifiAvg.mean());
                if ((tempBME <= 0) && (humBME <= 0)) {}
        else {
        Blynk.virtualWrite(V8, tempBME);
        Blynk.virtualWrite(V9, humBME);
        Blynk.virtualWrite(V11, presBME);

        Blynk.virtualWrite(V12, up3);
        Blynk.virtualWrite(V13, up5);
        Blynk.virtualWrite(V14, up10);
        Blynk.virtualWrite(V15, up25);
        Blynk.virtualWrite(V16, up50);
        Blynk.virtualWrite(V17, up100);

        Blynk.virtualWrite(V23, bmeiaq);
        Blynk.virtualWrite(V24, bmeiaqAccuracy);
        Blynk.virtualWrite(V25, bmestaticIaq);
        Blynk.virtualWrite(V26, bmeco2Equivalent);
        Blynk.virtualWrite(V27, bmebreathVocEquivalent);
        Blynk.virtualWrite(V28, bmestabStatus);
        Blynk.virtualWrite(V29, bmerunInStatus);
        Blynk.virtualWrite(V30, bmegasPercentage);
        Blynk.virtualWrite(V33, gasBME);
        //if (abshum2 > 0) {Blynk.virtualWrite(V34, abshum2);}
        //if (tempDHT > 0) {Blynk.virtualWrite(V35, tempDHT);}
        //if (humDHT > 0) {Blynk.virtualWrite(V36, humDHT);}
        if (tempSHT > 0) {Blynk.virtualWrite(V37, tempSHT);}
        if (humSHT > 0) {Blynk.virtualWrite(V38, humSHT);}
        bridge3.virtualWrite(V91, tempSHT);
        bridge3.virtualWrite(V92, humSHT);
        bridge3.virtualWrite(V93, co2SCD);
        bridge3.virtualWrite(V94, presBME);
        bridge2.virtualWrite(V80, presBME);
        bridge4.virtualWrite(V80, presBME);
        Blynk.virtualWrite(V42, tempSCD);
        Blynk.virtualWrite(V43, humSCD);
        Blynk.virtualWrite(V45, co2SCD);
        Blynk.virtualWrite(V101, srawVoc);
        Blynk.virtualWrite(V102, srawNox);
        Blynk.virtualWrite(V103, voc_index);
        Blynk.virtualWrite(V104, nox_index);
        }
    }
}

#include <WiFi.h>
#include <ArduinoJson.h>
#include <Wire.h>
#include <Adafruit_ADS1X15.h>
Adafruit_ADS1115 ads1;
Adafruit_ADS1115 ads2;
#include <PubSubClient.h>

const char* ssid = "WifiSSID";
const char* password =  "WifiPWd";
const char* mqttServer = "IpAddressMqttTinyAutomator";
const int mqttPort = 1883;
#define MQTT_PUB_TEMP "test-topic"
unsigned long last_time = 0;
unsigned long timer_delay = 1000;
///
WiFiClient espClient;
PubSubClient client(espClient);




///Accelerometer def
float Voltage = 0.0;
int decimalPrecision = 2;
int xvalue;
int yvalue;
int zvalue;
///

/// Temperature mesurement
float voltageDividerR1 = 5100;         // Resistor value in R1 for voltage devider method
float BValue = 3977;                    // The B Value of the thermistor for the temperature measuring range
float R1 = 10000;                        // Thermistor resistor rating at based temperature (25 degree celcius)
float t1 = 298.15;                      /* Base temperature t1 in Kelvin (default should be at 25 degree)*/
float R21, R22, R23 ;                            /* Resistance of Thermistor (in ohm) at Measuring Temperature*/
float t21, t22, t23 ;
float NTC_Temperature_1 = 0;/* Measurement temperature t2 in Kelvin */
float NTC_Temperature_2 = 0;
float NTC_Temperature_3 = 0;

float a ;                               /* Use for calculation in Temperature*/
float b1, b2, b3 ;                             /* Use for calculation in Temperature*/
float c1, c2, c3 ;                             /* Use for calculation in Temperature*/
float d1, d2, d3 ;                             /* Use for calculation in Temperature*/
float e = 2.718281828 ;                 /* the value of e use for calculation in Temperature*/

float tempSampleRead1  = 0;               /* to read the value of a sample including currentOffset1 value*/
float tempSampleRead2  = 0;
float tempSampleRead3  = 0;
float tempSampleSum1   = 0;
float tempSampleSum2   = 0;
float tempSampleSum3   = 0;
float tempLastSample  = 0;
float tempSampleCount = 0;
float tempMean1 ;
float tempMean2 ;
float tempMean3 ;
//////////

///smoothing adc reads
float NormAverage = 0;
float averagex = 0;// the average
float averagey = 0;
float averagez = 0;
float averagex100 = 0;// the average
float averagey100 = 0;
float averagez100 = 0;
//smoothing temp reads
float NTCtemperature = 0;
float averageTemperature1 = 0;
float averageTemperature2 = 0;
float averageTemperature3 = 0;
///
float volts0, tm1, tm2, tm3;
int16_t ts1, ts2, ts3;


void setup() {
  ///ADC initialization
  Serial.begin(115200);
  ads1.setGain(GAIN_ONE);
  ads1.begin(0x48, &Wire);
  ads2.setGain(GAIN_ONE);
  ads2.begin(0x49, &Wire);

  
  char buffer1[256];
  StaticJsonDocument<300> doc;
  //JsonObject& doc = doc.createObject();

  doc["device"] = "ESP32";
  doc["sensorType"] = "Temperature";
  doc["value"] = 24;

  char JSONmessageBuffer[100];
  serializeJson(doc, Serial);
  serializeJson(doc, buffer1);
  Serial.println("Sending message to MQTT topic..");
  Serial.println(JSONmessageBuffer);

  if (client.publish(MQTT_PUB_TEMP, buffer1) == true) {
    Serial.println("Success sending message");
  } else {
    Serial.println("Error sending message");
  }

  client.loop();
  Serial.println("-------------");

  delay(10000);
  Wire.begin(16, 17);

  ///WiFi connect&MQTT setup
  WiFi.begin(ssid, password);
  Serial.println("Connecting to WIFI…");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("");
  Serial.print("IP Address: ");
  Serial.println(WiFi.localIP());
  Serial.println("After 10 seconds the first reading will be displayed");
  client.setServer(mqttServer, mqttPort);
  //
  while (!client.connected()) {
    Serial.println("Connecting to MQTT...");

    if (client.connect("ESP32Client"/*, mqttUser, mqttPassword */)) {

      Serial.println("connected");

    } else {

      Serial.print("failed with state ");
      Serial.print(client.state());
      delay(2000);

    }
  }

}

void loop() {
  int16_t adc0, adc1, adc2, adc3;
  if ((millis() - last_time) > timer_delay) {
    if (WiFi.status() == WL_CONNECTED) {
      ////


      for (int i = 0; i < 100; i++) {
        tm1 = ads1.readADC_SingleEnded(3);
        ts1 = map(tm1, 0, 16000, 0, 1023);
        tm2 = ads2.readADC_SingleEnded(0);
        ts2 = map(tm2, 0, 16000, 0, 1023);
        tm3 = ads2.readADC_SingleEnded(1);
        ts3 = map(tm3, 0, 16000, 0, 1023);
        //Serial.print(tm);


        // Serial.println(ts);
        tempSampleRead1 = ts1;
        tempSampleRead2 = ts2;
        tempSampleRead3 = ts3;

        tempSampleSum1 = tempSampleSum1 + tempSampleRead1;
        tempSampleSum2 = tempSampleSum2 + tempSampleRead2;
        tempSampleSum3 = tempSampleSum3 + tempSampleRead3;

        tempSampleCount = tempSampleCount + 1;                                                      /* keep counting the sample quantity*/
        tempLastSample = millis();                                                                  /* reset the time in order to repeat the loop again*/


        if (tempSampleCount == 100)                                                                        /* after 1000 sample readings taken*/
        {
          tempMean1 = tempSampleSum1 / tempSampleCount;
          tempMean2 = tempSampleSum2 / tempSampleCount;
          tempMean3 = tempSampleSum3 / tempSampleCount; /* find the average analog value from those data*/

          R21 = (voltageDividerR1 * tempMean1) / (1023 - tempMean1);
          R22 = (voltageDividerR1 * tempMean2) / (1023 - tempMean2);
          R23 = (voltageDividerR1 * tempMean3) / (1023 - tempMean3);

          a = 1 / t1;
          b1 = log10(R1 / R21);
          c1 = b1 / log10(e);
          d1 = c1 / BValue ;
          b2 = log10(R1 / R22);
          c2 = b2 / log10(e);
          d2 = c2 / BValue ;
          b3 = log10(R1 / R23);
          c3 = b3 / log10(e);
          d3 = c3 / BValue ;


          t21 = 1 / (a - d1);
          t22 = 1 / (a - d2);
          t23 = 1 / (a - d3);

          Serial.print(t21 - 298.15, decimalPrecision); Serial.print("  ");
          Serial.print(t22 - 298.15, decimalPrecision); Serial.print("  ");
          Serial.print(t23 - 298.15, decimalPrecision); Serial.print("  ");
          Serial.println(" °C");


          tempSampleSum1 = 0;
          tempSampleSum2 = 0;
          tempSampleSum3 = 0;                                                                          /* reset all the total analog value back to 0 for the next count */
          tempSampleCount = 0;                                                                        /* reset the total number of samples taken back to 0 for the next count*/
        }
        NTC_Temperature_1 = NTC_Temperature_1 + (t21 - 298.15);
        NTC_Temperature_2 = NTC_Temperature_1 + (t22 - 298.15);
        NTC_Temperature_3 = NTC_Temperature_1 + (t23 - 298.15);
        //// End of Read temperature and store in NTC_Temperature_1

        //// Read accelerometer values
        yvalue = ads1.readADC_SingleEnded(0);                              //reads values from x-pin & measures acceleration in X direction
        int y = map(xvalue, 598, 898, -100, 100);               //maps the extreme ends analog values from -100 to 100 for our understanding

        float yg = float(y) / (-100.00);                        //converts the mapped value into acceleration in terms of "g"
        Serial.print(yg);                                       //prints value of acceleration in X direction
        Serial.print("g   ");                                   //prints "g"

        xvalue = ads1.readADC_SingleEnded(1);
        int x = map(yvalue, 602, 900, -100, 100);
        float xg = float(x) / (-100.00);
        Serial.print("\t");
        Serial.print(xg);
        Serial.print("g   ");

        zvalue = ads1.readADC_SingleEnded(2);
        int z = map(zvalue, 622, 932, -100, 100);
        float zg = float(z) / (100.00);
        Serial.print("\t");
        Serial.print(zg);
        Serial.println("g   ");
        //// End of Read accelerometer values
        averagex100 = averagex100 + xg;
        averagey100 = averagey100 + yg;
        averagez100 = averagez100 + zg;
        delay(18000); //9000=15 min
      }
      averagex = averagex100 / 100;
      averagey = averagey100 / 100;
      averagez = averagez100 / 100;
      averageTemperature1 = NTC_Temperature_1 / 100;
      averageTemperature2 = NTC_Temperature_2 / 100;
      averageTemperature3 = NTC_Temperature_3 / 100;
      NormAverage = sqrt(averagex * averagex + averagey * averagey + averagez * averagez);

      ////////////////////////////////////Sending Json Via MQTT Post Req//////////////////////////////////
      char buffer1[256];
      StaticJsonDocument<300> doc;
      //JsonObject& doc = doc.createObject();

      doc["device"] = "ESP32_NorviIIot";
      doc[" Temperature_1"] = averageTemperature1;
      doc[" Temperature_2"] = averageTemperature2;
      doc[" Temperature_3"] = averageTemperature3;
      doc[" AverageAccelerationX"] = averagex;
      doc[" AverageAccelerationY"] = averagey;
      doc[" AverageAccelerationZ"] = averagez;
      doc[" Vibration"] = NormAverage;
      averagex100 = 0;
      averagey100 = 0;
      averagez100 = 0;
      NTC_Temperature_1 = 0;
      NTC_Temperature_2 = 0;
      NTC_Temperature_3 = 0;
      char JSONmessageBuffer[100];
      serializeJson(doc, Serial);
      serializeJson(doc, buffer1);
      Serial.println("Sending message to MQTT topic..");
      Serial.println(JSONmessageBuffer);

      if (client.publish(MQTT_PUB_TEMP, buffer1) == true) {
        Serial.println("Success sending message");
      } else {
        Serial.println("Error sending message");
      }

      client.loop();
      Serial.println("-------------");
      delay(10000);
    }
    else {
      Serial.println("WiFi is Disconnected!");
    }
    last_time = millis();
  }
}

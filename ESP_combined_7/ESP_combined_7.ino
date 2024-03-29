// Code for ESP32 Based DHT11 and MQ-2 Sensor Pulse Oxymeter Sensor
// Integration with Firebase Done
// Added Push Buttons 2 Mode System
// Added ECG Sensor

/*
  It is best to attach the sensor to your finger using a rubber band or other tightening 
  device. Humans are generally bad at applying constant pressure to a thing. When you 
  press your finger against the sensor it varies enough to cause the blood in your 
  finger to flow differently which causes the sensor readings to go wonky.
*/

// Including Libraries
#include <dht.h>
#include <Wire.h>
#include "MAX30105.h"
#include "spo2_algorithm.h"
#include "heartRate.h"
#include <LiquidCrystal_I2C.h>

#if defined(ESP32)
#include <WiFi.h>
#include <FirebaseESP32.h>
#elif defined(ESP8266)
#include <ESP8266WiFi.h>
#include <FirebaseESP8266.h>
#endif
//Provide the token generation process info.
#include <addons/TokenHelper.h>
//Provide the RTDB payload printing info and other helper functions.
#include <addons/RTDBHelper.h>


#define WIFI_SSID "Omi"
#define WIFI_PASSWORD "sudo_21101272"
#define API_KEY "cllTYvDbOrUOys8ewHxJMCLefw8aHYgyYjFkJxza"
#define DATABASE_URL "https://health-monitoring-device-e9146-default-rtdb.firebaseio.com/"


MAX30105 particleSensor;

//#define led 2
#define dht_apin 16
#define MQ2pin 34
#define MQ2_Led 27

#define Threshold 1400  // For MQ-2
#define MAX_BRIGHTNESS 255 // For Oxymeter

// Defining Pins for ECG Sensor
//#define lp 25
//#define lm 26
#define out 36

// ---------------------------- Defining Pins for Push Button
#define push1 35
#define push2 39

#if defined(__AVR_ATmega328P__) || defined(__AVR_ATmega168__)
//Arduino Uno doesn't have enough SRAM to store 100 samples of IR led data and red led data in 32-bit format
//To solve this problem, 16-bit MSB of the sampled data will be truncated. Samples become 16-bit data.
uint16_t irBuffer[100]; //infrared LED sensor data
uint16_t redBuffer[100];  //red LED sensor data
#else
uint32_t irBuffer[100]; //infrared LED sensor data
uint32_t redBuffer[100];  //red LED sensor data
#endif

int32_t bufferLength; //data length
int32_t spo2; //SPO2 value
int8_t validSPO2; //indicator to show if the SPO2 calculation is valid
int32_t heartRate; //heart rate value
int8_t validHeartRate; //indicator to show if the heart rate calculation is valid

byte pulseLED = 15; //Must be on PWM pin
byte readLED = 12; //Blinks with each data read
float sensorValue;  //variable to store sensor value

// Calling the library
dht DHT;

//Define Firebase Data object
FirebaseData fbdo;
FirebaseAuth auth;
FirebaseConfig config;


// Defining variables for firebase.
int bpm, oxy;
float hmid, b_temp, r_temp, smoke_d;

// Setting LCD Display
LiquidCrystal_I2C lcd(0x27,16,2);  // set the LCD address to 0x3F for a 16 chars and 2 line display

const byte RATE_SIZE = 4; //Increase this for more averaging. 4 is good.
byte rates[RATE_SIZE]; //Array of heart rates
byte rateSpot = 0; // For using as a Index of the rates array
long lastBeat = 0; //Time at which the last beat occurred

float beatsPerMinute;
int beatAvg;


void setup()
{
  Serial.begin(115200); // initialize serial communication at 115200 bits per second:
  delay(2000);

  // --------------------------------------- Firebase -------------------------------------------------------
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  Serial.print("Connecting to Wi-Fi");
  
  while (WiFi.status() != WL_CONNECTED)
  {
    Serial.print(".");
    delay(300);
  }
  Serial.println();
  Serial.print("Connected with IP: ");
  Serial.println(WiFi.localIP());
  Serial.println();
  Serial.printf("Firebase Client v%s\n\n", FIREBASE_CLIENT_VERSION);

  config.api_key = API_KEY;
  config.database_url = DATABASE_URL;

  Firebase.begin(DATABASE_URL, API_KEY);
  //Comment or pass false value when WiFi reconnection will control by your code or third party library
 // Firebase.reconnectWiFi(true);
  Firebase.setDoubleDigits(5);
  // --------------------------------------- Firebase -------------------------------------------------------
  
  Serial.println("DHT11 Humidity & temperature Sensor");
  Serial.println("MQ2 warming up!");

  delay(1000);//Wait before accessing Sensor
//  pinMode(led, OUTPUT);
  pinMode(MQ2_Led, OUTPUT);  
  pinMode(pulseLED, OUTPUT);
  pinMode(readLED, OUTPUT);

  // For ECG Sensor
//  pinMode(lp, INPUT); // Setup for leads off detection LO +
//  pinMode(lm, INPUT); // Setup for leads off detection LO -

//  For Push Button Mode 
  pinMode(push1, INPUT);
  pinMode(push2, INPUT);

  pinMode(MQ2pin, INPUT);

  lcd.init();
  lcd.clear();         
  lcd.backlight();      // Make sure backlight is on

  // Print a message on both lines of the LCD.
  lcd.setCursor(3,0);   //Set cursor to character 2 on line 0
  lcd.print("Health");
  
  lcd.setCursor(0,1);   //Move cursor to character 2 on line 1
  lcd.print("MonitoringDevice");

  // Initialize sensor
  if (!particleSensor.begin(Wire, I2C_SPEED_FAST)) //Use default I2C port, 400kHz speed
  {
    Serial.println(F("MAX30105 was not found. Please check wiring/power."));
    while (1);
  }

  

  Serial.println(F("Attach sensor to finger with rubber band. Press any key to start conversion"));
//  while (Serial.available() == 0) ; //wait until user presses a key
//  Serial.read();

  byte ledBrightness = 60; //Options: 0=Off to 255=50mA
  byte sampleAverage = 4; //Options: 1, 2, 4, 8, 16, 32
  byte ledMode = 2; //Options: 1 = Red only, 2 = Red + IR, 3 = Red + IR + Green
  byte sampleRate = 100; //Options: 50, 100, 200, 400, 800, 1000, 1600, 3200
  int pulseWidth = 411; //Options: 69, 118, 215, 411
  int adcRange = 4096; //Options: 2048, 4096, 8192, 16384

  particleSensor.setup(ledBrightness, sampleAverage, ledMode, sampleRate, pulseWidth, adcRange); //Configure sensor with these settings
  particleSensor.enableDIETEMPRDY(); //Enable the temp ready interrupt. This is required.

}

void loop()
{
  if (digitalRead(push1) == 1){
    Serial.println("ECG Part");
    
    while (1 == 1){
// ========================================================== ECG Sensor Part ===============================
//      Serial.println("ECG Part");
      Serial.println(analogRead(out));
      delay(10);
      }
    }
  if (digitalRead(push2) == 1){
    Serial.println("Other Sensors Working");
    while (1 == 1){      
//     ====================================================== Other sensor part ==============================
      bufferLength = 100; //buffer length of 100 stores 4 seconds of samples running at 25sps
      //read the first 100 samples, and determine the signal range
      for (byte i = 0 ; i < bufferLength ; i++)
      {
        while (particleSensor.available() == false) //do we have new data?
          particleSensor.check(); //Check the sensor for new data
    
        redBuffer[i] = particleSensor.getRed();
        irBuffer[i] = particleSensor.getIR();
        particleSensor.nextSample(); //We're finished with this sample so move to next sample
    
        Serial.print(F("red="));
        Serial.print(redBuffer[i], DEC);
        Serial.print(F(", ir="));
        Serial.println(irBuffer[i], DEC);
      }
    
      //calculate heart rate and SpO2 after first 100 samples (first 4 seconds of samples)
      maxim_heart_rate_and_oxygen_saturation(irBuffer, bufferLength, redBuffer, &spo2, &validSPO2, &heartRate, &validHeartRate);
    
      //Continuously taking samples from MAX30102.  Heart rate and SpO2 are calculated every 1 second
      while (1)
      {
        //dumping the first 25 sets of samples in the memory and shift the last 75 sets of samples to the top
        for (byte i = 25; i < 100; i++)
        {
          redBuffer[i - 25] = redBuffer[i];
          irBuffer[i - 25] = irBuffer[i];
        }
    
        Serial.println("Pulse-Oxymeter Sensor Working.......");
    
        //take 25 sets of samples before calculating the heart rate.

        int x = 0;
        for (byte i = 75; i < 100; i++){
          while (particleSensor.available() == false) //do we have new data?
            particleSensor.check(); //Check the sensor for new data
    
          digitalWrite(readLED, !digitalRead(readLED)); //Blink onboard LED with every data read
    
          redBuffer[i] = particleSensor.getRed();
          irBuffer[i] = particleSensor.getIR();
          particleSensor.nextSample(); //We're finished with this sample so move to next sample

//          =====================================
          for (int i = 0; i < 8; i++){
              long irValue = particleSensor.getIR();
              if (checkForBeat(irValue) == true) {
//                digitalWrite(led, HIGH);
                //We sensed a beat!
                long delta = millis() - lastBeat;
                lastBeat = millis();
            
                beatsPerMinute = 60 / (delta / 1000.0);
            
                if (beatsPerMinute < 255 && beatsPerMinute > 20) {
                  rates[rateSpot++] = (byte)beatsPerMinute; //Store this reading in the array
                  rateSpot %= RATE_SIZE; //Wrap variable
            
                  //Take average of readings
                  beatAvg = 0;
                  for (byte x = 0 ; x < RATE_SIZE ; x++)
                    beatAvg += rates[x];
                  beatAvg /= RATE_SIZE;
                }
              }
            }
            i = 0;

          Serial.print("IR=");
          Serial.print(irValue);
          Serial.print(", BPM=");
          Serial.print(beatsPerMinute);
          Serial.print(", Avg BPM=");
          Serial.print(beatAvg);
            
          //send samples and calculation result to terminal program through UART
          Serial.print(F("red="));
          Serial.print(redBuffer[i], DEC);
          Serial.print(F(", ir="));
          Serial.println(irBuffer[i], DEC);
    
          Serial.print(F("HR="));
          Serial.print(heartRate, DEC);
    
          Serial.print(F(", HRvalid="));
          Serial.print(validHeartRate, DEC);
    
          Serial.print(F(", SPO2="));
          Serial.print(spo2, DEC);
    
          Serial.print(F(", SPO2Valid="));
          Serial.print(validSPO2, DEC);
    
          // Printing Heart Rate and SP02 In LCD Display
          lcd.clear(); 
          lcd.setCursor(0,0);   //Move cursor to character 0 on line 1
          lcd.print("Heart Rate ");
          lcd.print(heartRate);
          lcd.print("BPM");
    
          lcd.setCursor(0, 1);
          lcd.print("SP02: ");
          lcd.print(spo2);
    
    //   Reading Body Temperature
         float temperature = particleSensor.readTemperature();
         temperature = temperature + 5;
      
         Serial.print(F(" Body TemperatureC="));
         Serial.print(temperature, 4);
         Serial.println(" ");
    
    //  DHT11 - Sensor
        DHT.read11(dht_apin);
      
        Serial.print(F("Current humidity = "));
        Serial.print(DHT.humidity);
        Serial.print("%  ");
        Serial.print(F("Room Temperature = "));
        Serial.print(DHT.temperature);
        Serial.print("C  ");
      
        //Fastest should be once every two seconds
      
        // MQ-2 Smoke Sensor
        sensorValue = analogRead(MQ2pin); // read analog input pin 17
        
        Serial.print("Smoke: ");
        Serial.print(sensorValue);
      
        if(sensorValue > Threshold)
        {
          Serial.print(" | Smoke detected!");
          digitalWrite(MQ2_Led, HIGH);
        }
        else{
          digitalWrite(MQ2_Led, LOW);
          }
        Serial.println("");
        
        // ================================== Firebase ==================================
          if (x == 100){
            if (Firebase.ready()){
            Firebase.setInt(fbdo, "/test/Pulse_Rate: ", heartRate);
            Firebase.setInt(fbdo, "/test/SP02: ", spo2);
            Firebase.setFloat(fbdo, "/test/Humidity: ", DHT.humidity);
            Firebase.setFloat(fbdo, "/test/Room_Temperature: ", DHT.temperature);
            Firebase.setFloat(fbdo, "/test/Body_Temperature: ", temperature);
            Firebase.setFloat(fbdo, "/test/Smoke_Level: ", sensorValue);
            Serial.println("Firebase Update Done.......");
//            x = 0;
            }
          }
       // ================================== Firebase ==================================
    
    
//        x = x + 1;;
        delay(1); // wait 2s for next reading
        Serial.println("=======================");
        }
    
        //After gathering 25 new samples recalculate HR and SP02
        maxim_heart_rate_and_oxygen_saturation(irBuffer, bufferLength, redBuffer, &spo2, &validSPO2, &heartRate, &validHeartRate);
      }
      }
    }
  else{
    Serial.println("Please Press one Push Button to Start");
    }
  delay(1000);
  
}

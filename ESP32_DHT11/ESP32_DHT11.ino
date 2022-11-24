// Code for ESP32 Based DHT11 and MQ-2 Sensor

// Including Libraries
#include <dht.h>
//#include <OneWire.h>
//#include <DallasTemperature.h>

// Defining the pins
#define led 2
#define dht_apin 16
#define MQ2pin 34
#define MQ2_Led 0

#define Threshold 400


// Calling the library
dht DHT;


float sensorValue;  //variable to store sensor value

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  delay(500);//Delay to let system boot
  Serial.println("DHT11 Humidity & temperature Sensor\n\n");
  Serial.println("MQ2 warming up!");

  
  delay(1000);//Wait before accessing Sensor
  pinMode(led, OUTPUT);
  pinMode(MQ2_Led, OUTPUT);


}

void loop() {
  // put your main code here, to run repeatedly:

  // Just Blinking a Led
  digitalWrite(led, HIGH);
  delay(500);
  digitalWrite(led, LOW);
  delay(500);

  // DHT11 - Sensor

  DHT.read11(dht_apin);

  Serial.print("Current humidity = ");
  Serial.print(DHT.humidity);
  Serial.print("%  ");
  Serial.print("temperature = ");
  Serial.print(DHT.temperature);
  Serial.println("C  ");

//  delay(2000);//Wait 2 seconds before accessing sensor again.

  //Fastest should be once every two seconds

  // MQ-2 Smoke Sensor
  sensorValue = analogRead(MQ2pin); // read analog input pin 17
  
  Serial.print("Sensor Value: ");
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
  delay(500); // wait 2s for next reading
  Serial.println("=======================");

}

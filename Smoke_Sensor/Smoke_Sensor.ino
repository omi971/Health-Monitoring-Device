/* Change the threshold value with your own reading */
#define Threshold 400

#define MQ2pin 34

//int MQ2pin = A0;
float sensorValue;  //variable to store sensor value

void setup() {
  Serial.begin(115200); // sets the serial port to 9600
  Serial.println("MQ2 warming up!");
  delay(2000); // allow the MQ2 to warm up
//  pinMode(MQ2pin, INPUT);
}

void loop() {
  sensorValue = analogRead(MQ2pin); // read analog input pin 0
  
  Serial.print("Sensor Value: ");
  Serial.print(sensorValue);

  if(sensorValue > Threshold)
  {
    Serial.print(" | Smoke detected!");
  }
  
  Serial.println("");
  delay(500); // wait 2s for next reading
}

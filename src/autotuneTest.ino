// #include <Arduino.h>
// /*
//    AutoPID BasicTempControl Example Sketch

//    This program reads a dallas temperature probe as input, potentiometer as setpoint, drives an analog output.
//    It lights an LED when the temperature has reached the setpoint.
// */
// #include <AutoPID.h>
// #include <DallasTemperature.h>
// #include <OneWire.h>

// //pins
// // #define POT_PIN A0
// // #define OUTPUT_PIN A1
// // #define LED_PIN 6
// #define ONE_WIRE_BUS 2
// #define RELAY_PIN 3

// #define TEMP_READ_DELAY 800 //can only read digital temp sensor every ~750ms

// //pid settings and gains
// #define OUTPUT_MIN 0
// #define OUTPUT_MAX 5000
// #define KP .12
// #define KI .0003
// #define KD 0

// double temperature, setPoint, outputVal;
// double OutputPercentage;


// int WindowSize = 5000;
// unsigned long windowStartTime;

// OneWire oneWire(ONE_WIRE_BUS);
// DallasTemperature temperatureSensors(&oneWire);

// //input/output variables passed by reference, so they are updated automatically
// AutoPID myPID(&temperature, &setPoint, &outputVal, OUTPUT_MIN, OUTPUT_MAX, KP, KI, KD);

// unsigned long lastTempUpdate; //tracks clock time of last temp update

// //call repeatedly in loop, only updates after a certain time interval
// //returns true if update happened
// bool updateTemperature() {
//   if ((millis() - lastTempUpdate) > TEMP_READ_DELAY) {
//     temperature = temperatureSensors.getTempCByIndex(0); //get temp reading
//     lastTempUpdate = millis();
//     temperatureSensors.requestTemperatures(); //request reading for next time
//     return true;
//   }
//   return false;
// }//void updateTemperature


// void setup() {
//   // pinMode(POT_PIN, INPUT);
//   // pinMode(OUTPUT_PIN, OUTPUT);
//   pinMode(RELAY_PIN, OUTPUT);

//   windowStartTime = millis();

//   // pinMode(LED_PIN, OUTPUT);

//   setPoint = 40;

//   temperatureSensors.begin();
//   temperatureSensors.requestTemperatures();
//   while (!updateTemperature()) {} //wait until temp sensor updated

//   //if temperature is more than 4 degrees below or above setpoint, OUTPUT will be set to min or max respectively
//   myPID.setBangBang(4);
//   //set PID update interval to 4000ms
//   myPID.setTimeStep(4000);

//   Serial.begin(9600);
// }//void setup


// void loop() {
//   updateTemperature();
//   // setPoint = analogRead(POT_PIN);
//   myPID.run(); //call every loop, updates automatically at certain time interval
//   // analogWrite(OUTPUT_PIN, outputVal);
//   // digitalWrite(LED_PIN, myPID.atSetPoint(1)); //light up LED when we're at setpoint +-1 degree

//   Serial.print("CurrentTemp:");
//   Serial.print(temperature);
//   Serial.print(",");
//   Serial.print("SetPoint:");
//   Serial.print(setPoint);
//   Serial.print(",");
//   Serial.print("OutputPercentage:");
//   Serial.println(outputVal);
  

//   /************************************************
//      turn the output pin on/off based on pid output
//    ************************************************/
//   unsigned long now = millis();
//   if (now - windowStartTime > WindowSize)
//   { //time to shift the Relay Window
//     windowStartTime += WindowSize;
//   }
//   if (outputVal > now - windowStartTime) digitalWrite(RELAY_PIN, LOW);
//   else digitalWrite(RELAY_PIN, HIGH);

// }//void loop

// // void loop()
// // {
// //   // Send the command to get temperatures
// //   sensors.requestTemperatures(); 

// //   Input = sensors.getTempCByIndex(0);
// //   myPID.Compute();

// //   // Calculate output as percentage of window size
// //   OutputPercentage = (Output / WindowSize) * 100;

// //   Serial.print("CurrentTemp:");
// //   Serial.print(Input);
// //   Serial.print(",");
// //   Serial.print("SetPoint:");
// //   Serial.print(Setpoint);
// //   Serial.print(",");
// //   Serial.print("OutputPercentage:");
// //   Serial.println(OutputPercentage);
  

// //   /************************************************
// //      turn the output pin on/off based on pid output
// //    ************************************************/
// //   unsigned long now = millis();
// //   if (now - windowStartTime > WindowSize)
// //   { //time to shift the Relay Window
// //     windowStartTime += WindowSize;
// //   }
// //   if (Output > now - windowStartTime) digitalWrite(RELAY_PIN, LOW);
// //   else digitalWrite(RELAY_PIN, HIGH);
// // }
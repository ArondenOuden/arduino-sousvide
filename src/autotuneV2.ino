// #include <OneWire.h>
// #include <DallasTemperature.h>
// #include <pidautotuner.h>

// #define ONE_WIRE_BUS 2
// #define RelayPin 3

// double targetInputValue = 50;
// double loopInterval = 1000;

// int WindowSize = 5000;
// unsigned long windowStartTime;

// double OutputPercentage;

// // Setup a oneWire instance to communicate with any OneWire device
// OneWire oneWire(ONE_WIRE_BUS);	

// // Pass oneWire reference to DallasTemperature library
// DallasTemperature sensors(&oneWire);

// void setup() {
//     Serial.begin(9600);

//     pinMode(RelayPin, OUTPUT);

//     windowStartTime = millis();

//     PIDAutotuner tuner = PIDAutotuner();

//     // Set the target value to tune to
//     // This will depend on what you are tuning. This should be set to a value within
//     // the usual range of the setpoint. For low-inertia systems, values at the lower
//     // end of this range usually give better results. For anything else, start with a
//     // value at the middle of the range.
//     tuner.setTargetInputValue(targetInputValue);

//     // Set the loop interval in microseconds
//     // This must be the same as the interval the PID control loop will run at
//     tuner.setLoopInterval(loopInterval);

//     // Set the output range
//     // These are the minimum and maximum possible output values of whatever you are
//     // using to control the system (Arduino analogWrite, for example, is 0-255)
//     tuner.setOutputRange(0, 5000);

//     // Set the Ziegler-Nichols tuning mode
//     // Set it to either PIDAutotuner::ZNModeBasicPID, PIDAutotuner::ZNModeLessOvershoot,
//     // or PIDAutotuner::ZNModeNoOvershoot. Defaults to ZNModeNoOvershoot as it is the
//     // safest option.
//     tuner.setZNMode(PIDAutotuner::znModeNoOvershoot);

//     // This must be called immediately before the tuning loop
//     // Must be called with the current time in microseconds
//     // tuner.startTuningLoop(micros());
//     tuner.startTuningLoop();

//     // Run a loop until tuner.isFinished() returns true
//     long microseconds;
//     while (!tuner.isFinished()) {

//         // This loop must run at the same speed as the PID control loop being tuned
//         long prevMicroseconds = microseconds;
//         microseconds = micros();

//         // Send the command to get temperatures
//         sensors.requestTemperatures(); 

//         // Get input value here (temperature, encoder position, velocity, etc)
//         double input = sensors.getTempCByIndex(0);

//         // Call tunePID() with the input value and current time in microseconds
//         double output = tuner.tunePID(input);

//         Serial.print("CurrentTemp:");
//         Serial.print(input);
//         Serial.print(",");
//         Serial.print("SetPoint:");
//         Serial.print(targetInputValue);
//         Serial.print(",");
//         Serial.print("OutputPercentage:");
//         Serial.println(output);

//         // Set the output - tunePid() will return values within the range configured
//         // by setOutputRange(). Don't change the value or the tuning results will be
//         // incorrect.
//         unsigned long now = millis();
//         if (now - windowStartTime > WindowSize)
//         { //time to shift the Relay Window
//             windowStartTime += WindowSize;
//         }
//         if (output > now - windowStartTime) digitalWrite(RelayPin, LOW);
//         else digitalWrite(RelayPin, HIGH);

//         // This loop must run at the same speed as the PID control loop being tuned
//         while (micros() - microseconds < loopInterval) delayMicroseconds(1);
//     }

//     // Turn the output off here.
//     digitalWrite(RelayPin, HIGH);

//     // Get PID gains - set your PID controller's gains to these
//     Serial.println("Tuning finished!");
//     Serial.println("Gathering tune values");

//     double kp = tuner.getKp();
//     double ki = tuner.getKi();
//     double kd = tuner.getKd();
//     Serial.print("kp: ");
//     Serial.print(kp);
//     Serial.print(", ");

//     Serial.print("ki: ");
//     Serial.print(ki);
//     Serial.print(", ");

//     Serial.print("kd");
//     Serial.println(kd);
// }

// void loop() {

//     // ...
// }
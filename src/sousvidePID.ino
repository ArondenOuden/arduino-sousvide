#include <PID_v1.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <math.h>
#include <movingAvgFloat.h>
#include <ArduinoJson.h>

#define RelayPin 3
#define ONE_WIRE_BUS 2

//Define Variables we'll be connecting to
double Setpoint, Input, Output;
double OutputPercentage;

unsigned long startMillis;
unsigned long currentMillis;
const unsigned long period = 5000;

// Temperature threshold. Arduino light turns on when temp is within threshold 
double lightThreshold = 0.5;

double kp = 2;
double ki = 1;
double kd = 3;

//Specify the links and initial tuning parameters
// PID pidController(&Input, &Output, &Setpoint, kp, ki, kd, DIRECT);
PID pidController(&Input, &Output, &Setpoint, kp, ki, kd, P_ON_M, DIRECT);
movingAvgFloat tempMovingAvg(5);

int WindowSize = 5000;
unsigned long windowStartTime;

// Temperature reading //
// Data wire is plugged into digital pin 2 on the Arduino

// Setup a oneWire instance to communicate with any OneWire device
OneWire oneWire(ONE_WIRE_BUS);	

// Pass oneWire reference to DallasTemperature library
DallasTemperature sensors(&oneWire);

void setup()
{
    startMillis = millis();
    pinMode(RelayPin, OUTPUT);
    pinMode(LED_BUILTIN, OUTPUT);

    windowStartTime = millis();

    tempMovingAvg.begin();

    //initialize the variables we're linked to
    Setpoint = 30;

    //tell the PID to range between 0 and the full window size
    pidController.SetOutputLimits(0, WindowSize);

    //turn the PID on
    pidController.SetMode(AUTOMATIC);

    Serial.begin(9600);
}

void loop()
{
    // Send the command to get temperatures
    sensors.requestTemperatures();
    
    // Gets current temperature from sensor
    float currentTempReading = sensors.getTempCByIndex(0);
    
    // add new current data point to the moving average and save the moving average as the new PID Input
    Input = tempMovingAvg.reading(currentTempReading);

    // Compute PID values
    pidController.Compute();

    // Turn light on Arduino on when temperature is within a threshold of the setpoint
    double tempDifference = fabs(Input - Setpoint);
    if (tempDifference <= lightThreshold) {
        digitalWrite(LED_BUILTIN, HIGH);
    }
    else {
        digitalWrite(LED_BUILTIN, LOW);
    }

    // Calculate output as percentage of window size
    OutputPercentage = (Output / WindowSize) * 100;

    // Serial.print("CurrentTemp:");
    // Serial.print(currentTempReading);
    // Serial.print(",");
    // Serial.print("Input (moving average):");
    // Serial.print(Input);
    // Serial.print(",");
    // Serial.print("SetPoint:");
    // Serial.print(Setpoint);
    // Serial.print(",");
    // Serial.print("TempDiff:");
    // Serial.print(tempDifference);
    // Serial.print(",");
    // Serial.print("OutputPercentage:");
    // Serial.println(OutputPercentage);

    currentMillis = millis();
    if (currentMillis - startMillis >= period)
    {
        JsonDocument jsonOutput;
        jsonOutput["currentTemp"] = currentTempReading;
        jsonOutput["currentInputAvg"] = Input;
        jsonOutput["SetPoint"] = Setpoint;
        jsonOutput["tempDifference"] = tempDifference;
        jsonOutput["OutputPercentage"] = OutputPercentage;
        serializeJson(jsonOutput, Serial);
        Serial.println("");
        startMillis = currentMillis;
    }

    /************************************************
     turn the output pin on/off based on pid output
    ************************************************/
    unsigned long now = millis();
    if (now - windowStartTime > WindowSize)
    { //time to shift the Relay Window
        windowStartTime += WindowSize;
    }
    if (Output > now - windowStartTime) digitalWrite(RelayPin, LOW);
    else digitalWrite(RelayPin, HIGH);
}
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
unsigned long elapsedMillis = 0;

const unsigned long serialPrintInterval = 5000;

double tempApproveThreshold = 0.5; // Arduino light turns on when temp is within threshold of Setpoint
double safetyTempMax = 90.0; // Maximum safe temperature 
double lowerThreshold = -2.0; // Bang-bang lower threshold (offset from setpoint) 
double upperThreshold = 1.5; // Bang-bang upper threshold (offset from setpoint)

double kp = 2;
double ki = 1;
double kd = 3;

//Specify the links and initial tuning parameters
PID pidController(&Input, &Output, &Setpoint, kp, ki, kd, P_ON_E, DIRECT);
movingAvgFloat tempMovingAvg(5);

int WindowSize = 5000;
unsigned long windowStartTime;

// Temperature reading
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);

void setup()
{
    startMillis = millis();
    pinMode(RelayPin, OUTPUT);
    pinMode(LED_BUILTIN, OUTPUT);

    windowStartTime = millis();

    tempMovingAvg.begin();

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

    // Safety: Stop heating if the sensor is malfunctioning
    if (currentTempReading == -127.0) {
        digitalWrite(RelayPin, HIGH); // Turn off heating
        Serial.println("Error: Temperature sensor failure!");
        return;
    }

    // Safety: Stop heating if temperature exceeds maximum safety limit
    if (currentTempReading > safetyTempMax) {
        digitalWrite(RelayPin, HIGH); // Turn off heating
        Serial.println("Error: Max temperature exceeded!");
        return;
    }
    
    // Add new current data point to the moving average
    // And save the moving average as the new PID Input
    Input = tempMovingAvg.reading(currentTempReading);

    // Bang-bang or PID control
    if (Input < (Setpoint + lowerThreshold)) {
        digitalWrite(RelayPin, LOW); // Turn heating on
        OutputPercentage = 100;
    } else if (Input > (Setpoint + upperThreshold)) {
        digitalWrite(RelayPin, HIGH); // Turn heating off
        OutputPercentage = 0;
    } else {
        // PID control for fine-tuning within threshold
        pidController.Compute();
        unsigned long now = millis();
        if (now - windowStartTime > WindowSize) {
            windowStartTime += WindowSize;
        }
        if (Output > now - windowStartTime) digitalWrite(RelayPin, LOW);
        else digitalWrite(RelayPin, HIGH);

        // Calculate output as percentage of window size
        OutputPercentage = (Output / WindowSize) * 100;
    }


    // Turn light on Arduino on when temperature is within a threshold of the setpoint
    double tempDifference = fabs(Input - Setpoint);
    if (tempDifference <= tempApproveThreshold) {
        digitalWrite(LED_BUILTIN, HIGH);
    }
    else {
        digitalWrite(LED_BUILTIN, LOW);
    }

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
    if (currentMillis - startMillis >= serialPrintInterval)
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
}
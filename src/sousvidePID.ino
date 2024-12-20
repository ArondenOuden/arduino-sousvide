#include <PID_v1.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <math.h>
#include <movingAvgFloat.h>
#include <ArduinoJson.h>

#define RelayPin 3
#define ONE_WIRE_BUS 2

double Setpoint, Input, Output; // PID class variables
double OutputPercentage; // Status variables
float currentTempReading;
double tempDifference;

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

int pidWindowSize = 5000;
unsigned long pidWindowStartTime;

// Temperature reading
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);

void setup()
{
    startMillis = millis();
    pinMode(RelayPin, OUTPUT);
    pinMode(LED_BUILTIN, OUTPUT);

    pidWindowStartTime = millis();

    tempMovingAvg.begin();

    // Initialize PID
    Setpoint = 30;
    pidController.SetOutputLimits(0, pidWindowSize);
    pidController.SetMode(AUTOMATIC); // turn the PID on

    Serial.begin(9600);
}

void loop()
{
    readTemp();
    if (!isSafe()) return;
    temperatureControl();
    tempApproveLight();
    // sendSerialPlotterStatus():
    sendJsonStatus();
}

void readTemp() {
    // Send the command to get temperatures
    sensors.requestTemperatures();
    
    // Gets current temperature from sensor
    currentTempReading = sensors.getTempCByIndex(0);
}

bool isSafe() {
    // Safety: Stop heating if the sensor is malfunctioning
    if (currentTempReading == -127.0) {
        digitalWrite(RelayPin, HIGH); // Turn off heating
        Serial.println("Error: Temperature sensor failure!");
        return true;
    }

    // Safety: Stop heating if temperature exceeds maximum safety limit
    if (currentTempReading > safetyTempMax) {
        digitalWrite(RelayPin, HIGH); // Turn off heating
        Serial.println("Error: Max temperature exceeded!");
        return true;
    }
}

void temperatureControl() {
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
        currentMillis = millis();
        if (currentMillis - pidWindowStartTime > pidWindowSize) {
            pidWindowStartTime += pidWindowSize;
        }
        if (Output > currentMillis - pidWindowStartTime) digitalWrite(RelayPin, LOW);
        else digitalWrite(RelayPin, HIGH);

        // Calculate output as percentage of window size
        OutputPercentage = (Output / pidWindowSize) * 100;
    }
}

void tempApproveLight() {
    // Turn light on Arduino on when temperature is within a threshold of the setpoint
    tempDifference = fabs(Input - Setpoint);
    digitalWrite(LED_BUILTIN, (tempDifference <= tempApproveThreshold) ? HIGH : LOW);
}

void sendSerialPlotterStatus() {
    Serial.print("CurrentTemp:");
    Serial.print(currentTempReading);
    Serial.print(",");
    Serial.print("Input (moving average):");
    Serial.print(Input);
    Serial.print(",");
    Serial.print("SetPoint:");
    Serial.print(Setpoint);
    Serial.print(",");
    Serial.print("TempDiff:");
    Serial.print(tempDifference);
    Serial.print(",");
    Serial.print("OutputPercentage:");
    Serial.println(OutputPercentage);
}

void sendJsonStatus() {
    currentMillis = millis();
    elapsedMillis = currentMillis - startMillis;
    if (currentMillis - startMillis >= serialPrintInterval)
    {
        JsonDocument jsonOutput;
        jsonOutput["currentTemp"] = currentTempReading; 
        jsonOutput["currentInputAvg"] = Input;
        jsonOutput["SetPoint"] = Setpoint;
        jsonOutput["tempDifference"] = tempDifference;
        jsonOutput["OutputPercentage"] = OutputPercentage;
        jsonOutput["elapsedTime"] = elapsedMillis / (60000); // Time elapsed in minutes
        serializeJson(jsonOutput, Serial);
        Serial.println("");
        startMillis = currentMillis;
    }
}
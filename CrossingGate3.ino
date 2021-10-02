//8/9/21
//Railroad crossing gates operated with light dependent resistors (LDR) and stepper motor. Version 2
//Sensors are the Arduino LDRs mounted between the rails pointing up, activated by the shadow of the train (I hope)
//This version has two sensors so trains can run in either direction, gate can only open if neither
//detects a train, so no need for long train delay.  Gate will only open after down side sensor is clear.
//8/10/21 Trap for low light condition.
//8/11/21 Warning lights added.
//Warning lights enabled, delays removed.
//Added a calibration LDR to handle changes in ambient light.  Note that the LDRs can have significantly
//different levels, might need to apply an offset.
//8/16/21 moved to the Accelstepper driver
//Made warninglights a subroutine
//8/18/21Added a Hall sensor to reset the gate position at power-up. Runs in a sub-routine.
//Warning LEDs replaced with external circuit. This avoids the problem of making the warninglights flash while
//the stepper is running!
//8/22/21 Tidied up, removed old flasher code.
//8/28/21 Final tweaking of the closed / open values to match mechanics
//9/5/21 Attempting to handle short trains! TODO: figure out how to open the gates!
//9/23/21 Added a third sensor positioned at the crossing.  Code now handles short trains and long trains, also warning light
//behaviour is more realistic.
//9/29/21 Added an adjustment to sens3 for correct operation of gate-open function.
//10/2/21 Added a smoothing routine to the #3 sensor.  This sensor is sensitive to the gaps between the cars, causing the the gate to open prematurly


#include <AccelStepper.h>

#define motorPin1  8      // IN1 on the ULN2003 driver
#define motorPin2  9      // IN2 on the ULN2003 driver
#define motorPin3  10     // IN3 on the ULN2003 driver
#define motorPin4  11     // IN4 on the ULN2003 driver  // Motor pin definitions:
#define MotorInterfaceType 8 // Define the AccelStepper interface type; 4 wire motor in half step mode:
// Initialize with pin sequence IN1-IN3-IN2-IN4 for using the AccelStepper library with 28BYJ-48 stepper motor:
AccelStepper stepper = AccelStepper(MotorInterfaceType, motorPin1, motorPin3, motorPin2, motorPin4);

int dt = 100;
int detect;
int ambient;
int dark = 625;
int lightVal1;
int lightPin1 = A1;
int lightVal2;
int lightPin2 = A2;
int lightPin3 = A3; //Sensor at the crossing
int ambientPin = A0;
int gateOpen = 0;             //Rotates the stepper to the open posn.
int gateClose = 3600;         //Rotates the stepper to the closed (down) posn.
int gateState = 0;
int sens1 = 0;
int sens2 = 0;
int sens3 = 0;
int sens3Old = 0;
int sens3New = 0;
const int numReadings =  30; //amount of smoothing for #3 sensor
int readings[numReadings];   // the readings from the analog input for #3 sensor
int readIndex = 0;           // the index of the current reading
float lightVal3total = 0;    // the running total of readings for #3 sensor
int lightVal3;               // the smoothed value of the #3 sensor

int warningLeds = 4;         //Pin attached to the warning lights flasher circuit.
int hallPin = 3;             //Pin attached to Hall sensor, normally high.
int dtt = 10000;             // delay to allow the train to clear the down sensor after opening the gate


void homefunction() {        //Resets the gate position to the open position, in case something went wrong.
  stepper.setMaxSpeed(1000);
  stepper.setSpeed(500);
  pinMode(hallPin, INPUT);
  while (digitalRead(hallPin) == 1) { //Low unless the gate is displaced from the open position.
    stepper.runSpeed();
  }
  stepper.setCurrentPosition(0);  //Sets the motor position to 0 (the open position)
}

void setup() {
  homefunction();
  stepper.setMaxSpeed(1000); // Set the maximum steps per second:
  stepper.setSpeed(500);
  stepper.setAcceleration(400);  // Set the maximum acceleration in steps per second^2:
  pinMode (lightPin1, INPUT);
  pinMode (lightPin2, INPUT);
  pinMode (lightPin3, INPUT);
  pinMode (ambientPin, INPUT);
  pinMode (warningLeds, OUTPUT);
  Serial.begin(9600);
  for (int thisReading = 0; thisReading < numReadings; thisReading++) {
    readings[thisReading] = 0;                             //Initialize all smoothing readings for Sensor#3 to zero
  }
}

void loop() {

  ambient = analogRead(ambientPin);                        //Read ambient light from ambient LDR.
  Serial.print(" ambient:");
  Serial.print(ambient);
  detect = ((ambient) + 150);                              //Set the detect value based on the ambient light
  Serial.print(" detect:");
  Serial.print(detect);
  lightVal1 = analogRead(lightPin1);                       //Read the #1 LDR (no train is <detect train is >detect
  Serial.print(" lightVal1:");
  Serial.print(lightVal1);
  lightVal2 = analogRead(lightPin2);                       //Read the #2 LDR (no train is <detect train is >detect
  Serial.print(" lightVal2:");
  Serial.print(lightVal2);

  //smoothing routine applied to sensor#3 to avoid gates opening prematurly due to gaps between cars.
  lightVal3total = lightVal3total - readings[readIndex];  //Subtract the last reading
  readings[readIndex] = analogRead(lightPin3);            //Read the #3 LDR
  lightVal3total = lightVal3total + readings[readIndex];  //Add the reading to the total
  readIndex = readIndex + 1;                              //Advance to the next position in the array
  if (readIndex >= numReadings) {                         //If we are at the end of the array...
    readIndex = 0;                                        //...Wrap around to the beginning.
  }
  lightVal3 = lightVal3total / numReadings;              //Take the average to get smoothed sensor #3 value
  lightVal3 = lightVal3 - 40;                            //small correction to sensor #3

  //lightVal3 = analogRead(lightPin3)+(adj3sens);
  Serial.print(" lightVal3:");
  Serial.println(lightVal3);
  if (ambient < dark) {                                  //Check if ambient is too dark, skip to the end

    //**************************************************************************************
    if (lightVal1 > detect) {              //If sensor 1 detects a train (distant sensor one direction)
      sens1 = 1;
    }
    if (lightVal1 < detect) {
      sens1 = 0;
    }
    //Serial.print(" sens 1:");
    //Serial.print(sens1);
    if (lightVal2 > detect) {             //If sensor 2 detects a train  (distant sensor other direction)
      sens2 = 1;
    }
    if (lightVal2 < detect) {
      sens2 = 0;
    }
    //Serial.print(" sens 2:");
    //Serial.print(sens2);
    if (lightVal3 > detect) {             //If sensor 3 detects a train (sensor 3 is at the crossing)
      sens3 = 1;
    }
    if (lightVal3 < detect) {
      sens3 = 0;
    }
    //Serial.print(" sens 3:");
    //Serial.println(sens3);
    //delay(500);
    if (sens1 == 1 && gateState == 0) {
      digitalWrite (warningLeds, HIGH);// Warning lights on gate closing
      stepper.moveTo(gateClose);
      stepper.runToPosition();            //Close the gate to traffic.
      gateState = 1;                      //Set gate State=1 (closed)
    }
    if (sens2 == 1 && gateState == 0) {
      digitalWrite (warningLeds, HIGH);// Warning lights on gate closing
      stepper.moveTo(gateClose);
      stepper.runToPosition();            //Close the gate to traffic.
      gateState = 1;                      //Set gate State=1 (closed)
    }
    sens3New = sens3;
    if (sens3Old == 1 && sens3New == 0) {
      if (gateState == 1) {                //If the gate is closed, wait until the last car has cleared the crossing and...
        stepper.moveTo(gateOpen);
        stepper.runToPosition();           //Open the gate to road traffic.
        gateState = 0;                    //Set gate State=0 (open)
        digitalWrite (warningLeds, LOW); //Warning lights off, gate is open
        delay(dtt);                      //delay to allow the train to clear the downstream sensor
      }
    }
    sens3Old = sens3New;
  }
  stepper.disableOutputs();               //Remove power from the stepper to avoid overheating
}

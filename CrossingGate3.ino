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
//8/18/21Added a Hall sensor to reset the gate position at power-up runs in a sub-routine.


#include <AccelStepper.h>

#define motorPin1  8      // IN1 on the ULN2003 driver
#define motorPin2  9      // IN2 on the ULN2003 driver
#define motorPin3  10     // IN3 on the ULN2003 driver
#define motorPin4  11     // IN4 on the ULN2003 driver  // Motor pin definitions:
#define MotorInterfaceType 8 // Define the AccelStepper interface type; 4 wire motor in half step mode:
// Initialize with pin sequence IN1-IN3-IN2-IN4 for using the AccelStepper library with 28BYJ-48 stepper motor:
AccelStepper stepper = AccelStepper(MotorInterfaceType, motorPin1, motorPin3, motorPin2, motorPin4);

int dt = 250;
int detect;
int ambient;
int dark = 625;
int lightVal;
int lightPin = A1;
int lightVal2;
int lightPin2 = A2;
int ambientPin = A0;
int gateOpen = 0; //   Rotates the stepper 90 anticlock. Could change depending on how the gates are connected to the stepper.
int gateClose = 1024; // Rotates the stepper 90 clockwise. Could change depending on how the gates are connected to the stepper.
int gateState;
int led1Pin = 2;
int led2Pin = 4;
int delayTleds = 500;
int hallPin = 3; //Pin attached to Hall sensor, normally high.

void warninglights() {
  digitalWrite (led1Pin, HIGH);
  digitalWrite (led2Pin, LOW);
  delay (delayTleds);
  digitalWrite (led1Pin, LOW);
  digitalWrite (led2Pin, HIGH);
  delay (delayTleds);
}
void homefunction() { //Resets the gate position to the closed position.
  stepper.setMaxSpeed(1000);
  stepper.setSpeed(500);
  pinMode(hallPin, FALLING); //Trigger on falling edge as magnet approaches the sensor
  while (digitalRead(hallPin) == 1) { //Normally this will be low unless the gate is displaced from the open position.
    stepper.runSpeed();
  }
  stepper.setCurrentPosition(0);  //Sets the motor position to 0
}


void setup() {
  homefunction();
  stepper.setMaxSpeed(1000); // Set the maximum steps per second:
  stepper.setSpeed(500);
  stepper.setAcceleration(400);  // Set the maximum acceleration in steps per second^2:
  pinMode (lightPin, INPUT);
  pinMode (lightPin2, INPUT);
  pinMode (ambientPin, INPUT);
  pinMode (led1Pin, OUTPUT);
  pinMode (led2Pin, OUTPUT);
  Serial.begin(9600);
}

void loop() {

  ambient = analogRead(ambientPin); //Read ambient light from third photoresistor.
  //Serial.println(ambient);
  detect = ((ambient) + 150); //Set the detect value based on the ambient light
  //Serial.println(detect);
  lightVal = analogRead(lightPin); //           Read the #1 light detector (no train is <detect train is >detect
  //Serial.println(lightVal);
  lightVal2 = analogRead(lightPin2); //         Read the #2 light detector (no train is <detect train is >detect
  //Serial.println(lightVal2);
  if (ambient < dark) { //      Check if ambient is too dark, skip to the end
    if (lightVal > detect || lightVal2 > detect) { // Warning lights on if either sensor detects a train
      warninglights();
    }
    if (lightVal < detect && lightVal2 < detect) { //Warning lights off if neither sensor detects a train
      digitalWrite (led1Pin, LOW);
      digitalWrite (led2Pin, LOW);
    }
    if (lightVal > detect && gateState == 0) { //   If there is a train and the gate is open (gateState=0)
      stepper.moveTo(gateClose);
      stepper.runToPosition();//             Close the gate to traffic.
      gateState = 1; //                           Set gate State=1 (closed)
    }
    if (lightVal < detect && gateState == 1 && lightVal2 < detect) { //If neither sensor detects a train and the gate is closed, open it.
      stepper.moveTo(gateOpen);
      stepper.runToPosition();//   Open the gate to traffic
      gateState = 0; //                          Set gate State=0 (open)
    }
    if (lightVal2 > detect && gateState == 0) { // If there is a train (other direction) and the gate is open (gateState=0)
      stepper.moveTo(gateClose);
      stepper.runToPosition();//             Close the gate to traffic.
      gateState = 1; //                           Set gate State=1 (closed)
    }
    if (lightVal2 < detect && gateState == 1 && lightVal < detect) { //If neither sensor detects a train and the gate is closed, open it.
      stepper.moveTo(gateOpen);
      stepper.runToPosition();//   Open the gate to traffic
      gateState = 0; //                          Set gate State=0 (open)
      delay(1000);
    }
  }
  stepper.disableOutputs();//Remove power from the stepper to avoid overheating
}

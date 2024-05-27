#include <Arduino.h>
#include "BasicStepperDriver.h"
#include "A4988.h"
#include "Servo.h"

// Receiver Inputs
// #define CH1 7
// #define CH2 8
#define CH3 10
#define CH4 11
#define SWA_CH5 12
#define SWB_CH6 13
#define TimeOut 50000

// // Defining motor driver pins
// #define MotorPin1 2
// #define MotorPin2 4

// All the wires needed for full functionality
#define DIR_Roll 2
#define STEP_Roll 3
#define DIR_Grip 4
#define STEP_Grip 5
//Uncomment line to use enable/disable functionality
//#define SLEEP 13

// // Relay pins for link 1 & 2
// #define RelayIn_1A 2
// #define RelayIn_1B 4
// #define RelayIn_2A 2
// #define RelayIn_2B 4
// #define En 3

#define ServoPin 6

// Motor steps per revolution. Most steppers are 200 steps or 1.8 degrees/STEP_Roll
#define SPR 200  //Steps per Revolution
#define MICROSTEPS 1
#define RPM_Roll 50
#define RPM_Grip 200

// int Rotate = 0;
// int RelayOut_1 = 0;
// int RelayOut_2 = 0;
int Pitch = 90;
int Roll = 0;
int Grip = 0;
int RollPrevPos;
int RollCurrPos;
int RollCurrPos_F;
int GripPrevPos;
int GripCurrPos;
int GripCurrPos_F;
int pos = 0;

Servo ServoPitch;

// 2-wire basic config, microstepping is hardwired on the driver
A4988 StepperRoll(SPR, DIR_Roll, STEP_Roll);
A4988 StepperGrip(SPR, DIR_Grip, STEP_Grip);

//Uncomment line to use enable/disable functionality
//A4988 stepper(SPR, DIR_Roll, STEP_Roll, SLEEP);

void setup() {
  Serial.begin(9600);
  StepperRoll.begin(RPM_Roll, MICROSTEPS);
  StepperGrip.begin(RPM_Grip, MICROSTEPS);
  // if using enable/disable on ENABLE pin (active LOW) instead of SLEEP uncomment next line
  // stepper.setEnableActiveState(LOW);
  // pinMode(MotorPin1, OUTPUT);
  // pinMode(MotorPin2, OUTPUT);
  // pinMode(RelayIn_1A, OUTPUT);
  // pinMode(RelayIn_1B, OUTPUT);
  // pinMode(En, OUTPUT);
  ServoPitch.attach(ServoPin);

  RollPrevPos = 0;
  GripPrevPos = 0;
}

void loop() {
  if (pulseIn(SWA_CH5, HIGH, TimeOut) > 1250 && pulseIn(SWB_CH6, HIGH, TimeOut) > 1250) {
    // MoveBase();
    // MoveLink1();
    // MoveLink2();
    MoveGripper();
  } else if (pulseIn(SWA_CH5, HIGH, TimeOut) > 1250 && pulseIn(SWB_CH6, HIGH, TimeOut) < 1250) {
    // MoveLink1();
    // MoveLink2();
    MoveClawPitch();
    MoveClawRoll();
  } else {
    // Default();
  }
  // PrintData();
  delay(5);
}

// void MoveBase() {
//   Rotate = map(pulseIn(CH4, HIGH, TimeOut), 1020, 1800, -100, 100);
//   Rotate = constrain(Rotate, -100, 100);
//   if (pulseIn(CH4, HIGH, TimeOut) == 0) {
//     Rotate = 0;
//   }
//   if (Rotate < -10) {
//     digitalWrite(MotorPin1, LOW);
//     digitalWrite(MotorPin2, HIGH);
//   } else if (Rotate > 10) {
//     digitalWrite(MotorPin1, HIGH);
//     digitalWrite(MotorPin2, LOW);
//   } else {
//     digitalWrite(MotorPin1, LOW);
//     digitalWrite(MotorPin2, LOW);
//   }
// }

// void MoveLink1() {
//   RelayOut_1 = map(pulseIn(CH1, HIGH, TimeOut), 1020, 1800, -100, 100);
//   RelayOut_1 = constrain(RelayOut_1, -100, 100);
//   if (pulseIn(CH1, HIGH, TimeOut) == 0) {
//     RelayOut_1 = 0;
//   }
//   if (RelayOut_1 < -10) {
//     digitalWrite(RelayIn_1A, LOW);
//     digitalWrite(RelayIn_1B, HIGH);
//   } else if (RelayOut_1 > 10) {
//     digitalWrite(RelayIn_1A, HIGH);
//     digitalWrite(RelayIn_1B, LOW);
//   } else {
//     digitalWrite(RelayIn_1A, LOW);
//     digitalWrite(RelayIn_1B, LOW);
//   }
// }

// void MoveLink2() {
//   RelayOut_2 = map(pulseIn(CH2, HIGH, TimeOut), 1080, 1730, -100, 100);
//   RelayOut_2 = constrain(RelayOut_2, -100, 100);
//   if (pulseIn(CH2, HIGH, TimeOut) == 0) {
//     RelayOut_2 = 0;
//   }
//   if (RelayOut_2 < -10) {
//     digitalWrite(RelayIn_2A, LOW);
//     digitalWrite(RelayIn_2B, HIGH);
//   } else if (RelayOut_2 > 10) {
//     digitalWrite(RelayIn_2A, HIGH);
//     digitalWrite(RelayIn_2B, LOW);
//   } else {
//     digitalWrite(RelayIn_2A, LOW);
//     digitalWrite(RelayIn_2B, LOW);
//   }
// }

void MoveClawPitch() {
  Pitch = map(pulseIn(CH3, HIGH, TimeOut), 1070, 1740, 0, 180);
  Pitch = constrain(Pitch, 0, 180);
  if (pulseIn(CH3, HIGH, TimeOut) == 0) {
    Pitch = 90;
  }
  ServoPitch.write(Pitch);
}

void MoveClawRoll() {
  Roll = map(pulseIn(CH4, HIGH, TimeOut), 1020, 1800, -180, 180);
  Roll = constrain(Roll, -180, 180);
  if (pulseIn(CH4, HIGH, TimeOut) == 0) {
    Roll = 0;
  }
  RollCurrPos = Roll;
  RollCurrPos_F = 0.9 * RollCurrPos_F + 0.1 * RollCurrPos;
  if (RollPrevPos != RollCurrPos) {
    StepperRoll.move(RollCurrPos_F - RollPrevPos);
    RollPrevPos = RollCurrPos_F;
  }
}

void MoveGripper() {
  Grip = map(pulseIn(CH3, HIGH, TimeOut), 1070, 1740, -180, 180);
  Grip = constrain(Grip, -180, 180);
  if (pulseIn(CH3, HIGH, TimeOut) == 0) {
    Grip = 0;
  }
  // GripCurrPos = Grip;
  // GripCurrPos_F = 0.9 * GripCurrPos_F + 0.1 * GripCurrPos;
  if (Grip < -100) {
    StepperGrip.move(-SPR);
  }else if (Grip > 100) {
    StepperGrip.move(SPR);
  }
}

void Default() {
  // Rotate = 0;
  // RelayOut_1 = 0;
  // RelayOut_2 = 0;
  // Pitch = 0;
  // Roll = 0;
  // Grip = 0;
}

void PrintData() {
  // Serial.print("Rotate:");
  // Serial.print(Rotate);
  // Serial.print("  RelayOut_1:");
  // Serial.print(RelayOut_1);
  // Serial.print("  RelayOut_2:");
  // Serial.print(RelayOut_2);
  Serial.print("  Pitch:");
  Serial.print(Pitch);
  Serial.print("  Roll:");
  Serial.print(Roll);
  Serial.print("  Grip:");
  Serial.print(Grip);

  Serial.println();
}

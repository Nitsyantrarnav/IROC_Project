// #include <Arduino.h>
// #include "BasicStepperDriver.h"
// #include "A4988.h"
// #include "Servo.h"

// Receiver Inputs
#define CH1 8
#define CH2 9
// #define CH3 10
#define CH4 11
#define SWA_CH5 12
#define SWB_CH6 13
#define TimeOut 50000

// Defining motor driver pins
#define MotorPin1 2
#define MotorPin2 3
#define En 10

// // All the wires needed for full functionality
// #define DIR 4
// #define STEP 5
// //Uncomment line to use enable/disable functionality
// //#define SLEEP 13

// Relay pins for link 1 & 2
#define RelayIn_1A 4
#define RelayIn_1B 5
#define RelayIn_2A 6
#define RelayIn_2B 7

// #define ServoPin 3

// // Motor steps per revolution. Most steppers are 200 steps or 1.8 degrees/step
// #define SPR 200  //Steps per Revolution
// #define MICROSTEPS 1
// #define RPM 50

int Rotate = 0;
int RelayOut_1 = 0;
int RelayOut_2 = 0;
// int Pitch = 0;
// int Roll = 0;
// int Grip = 0;
// int RollPrevPos;
// int RollCurrPos;
// int RollCurrPos_F;
// int GripPrevPos;
// int GripCurrPos;
// int GripCurrPos_F;
// int pos = 0;

// Servo ServoPitch;

// // 2-wire basic config, microstepping is hardwired on the driver
// A4988 StepperRoll(SPR, DIR, STEP);
// A4988 StepperGrip(SPR, DIR, STEP);

//Uncomment line to use enable/disable functionality
//A4988 stepper(SPR, DIR, STEP, SLEEP);

void setup() {
  Serial.begin(9600);
  // StepperRoll.begin(RPM, MICROSTEPS);
  // StepperGrip.begin(RPM, MICROSTEPS);
  // if using enable/disable on ENABLE pin (active LOW) instead of SLEEP uncomment next line
  // stepper.setEnableActiveState(LOW);
  pinMode(MotorPin1, OUTPUT);
  pinMode(MotorPin2, OUTPUT);
  pinMode(RelayIn_1A, OUTPUT);
  pinMode(RelayIn_1B, OUTPUT);
  pinMode(En, OUTPUT);
  // ServoPitch.attach(ServoPin);

  // RollPrevPos = 0;
  // GripPrevPos = 0;
}

void loop() {
  if (pulseIn(SWA_CH5, HIGH, TimeOut) > 1250 && pulseIn(SWB_CH6, HIGH, TimeOut) > 1250) {
    MoveBase();
    MoveLink1();
    MoveLink2();
    // MoveGripper();
  } else if (pulseIn(SWA_CH5, HIGH, TimeOut) > 1250 && pulseIn(SWB_CH6, HIGH, TimeOut) < 1250) {
    MoveLink1();
    MoveLink2();
    // MoveClawPitch();
    // MoveClawRoll();
  } else {
    Default();
  }
  // PrintData();
  delay(5);
}

void MoveBase() {
  Rotate = map(pulseIn(CH4, HIGH, TimeOut), 1020, 1800, -100, 100);
  Rotate = constrain(Rotate, -100, 100);
  if (pulseIn(CH4, HIGH, TimeOut) == 0) {
    Rotate = 0;
  }
  if (Rotate < -10) {
    digitalWrite(MotorPin1, LOW);
    digitalWrite(MotorPin2, HIGH);
    analogWrite(En, 50);
  } else if (Rotate > 10) {
    digitalWrite(MotorPin1, HIGH);
    digitalWrite(MotorPin2, LOW);
    analogWrite(En, 50);
  } else {
    digitalWrite(MotorPin1, LOW);
    digitalWrite(MotorPin2, LOW);
    analogWrite(En, LOW);
  }
}

void MoveLink1() {
  RelayOut_1 = map(pulseIn(CH1, HIGH, TimeOut), 1020, 1800, -100, 100);
  RelayOut_1 = constrain(RelayOut_1, -100, 100);
  if (pulseIn(CH1, HIGH, TimeOut) == 0) {
    RelayOut_1 = 0;
  }
  if (RelayOut_1 < -10) {
    digitalWrite(RelayIn_1A, LOW);
    digitalWrite(RelayIn_1B, HIGH);
  } else if (RelayOut_1 > 10) {
    digitalWrite(RelayIn_1A, HIGH);
    digitalWrite(RelayIn_1B, LOW);
  } else {
    digitalWrite(RelayIn_1A, LOW);
    digitalWrite(RelayIn_1B, LOW);
  }
}

void MoveLink2() {
  RelayOut_2 = map(pulseIn(CH2, HIGH, TimeOut), 1080, 1730, -100, 100);
  RelayOut_2 = constrain(RelayOut_2, -100, 100);
  if (pulseIn(CH2, HIGH, TimeOut) == 0) {
    RelayOut_2 = 0;
  }
  if (RelayOut_2 < -10) {
    digitalWrite(RelayIn_2A, LOW);
    digitalWrite(RelayIn_2B, HIGH);
  } else if (RelayOut_2 > 10) {
    digitalWrite(RelayIn_2A, HIGH);
    digitalWrite(RelayIn_2B, LOW);
  } else {
    digitalWrite(RelayIn_2A, LOW);
    digitalWrite(RelayIn_2B, LOW);
  }
}

// void MoveClawPitch() {
//   Pitch = map(pulseIn(CH3, HIGH, TimeOut), 1070, 1740, 0, 180);
//   Pitch = constrain(Pitch, 0, 180);
//   if (pulseIn(CH3, HIGH, TimeOut) == 0) {
//     Pitch = 90;
//   }
//   ServoPitch.write(Pitch);
// }

// void MoveClawRoll() {
//   Roll = map(pulseIn(CH4, HIGH, TimeOut), 1020, 1800, -180, 180);
//   Roll = constrain(Roll, -180, 180);
//   if (pulseIn(CH4, HIGH, TimeOut) == 0) {
//     Roll = 0;
//   }
//   RollCurrPos = Roll;
//   RollCurrPos_F = 0.9 * RollCurrPos_F + 0.1 * RollCurrPos;
//   if (RollPrevPos != RollCurrPos) {
//     StepperRoll.move(RollCurrPos_F - RollPrevPos);
//     RollPrevPos = RollCurrPos_F;
//   }
// }

// void MoveGripper() {
//   Grip = map(pulseIn(CH3, HIGH, TimeOut), 1070, 1740, -180, 180);
//   Grip = constrain(Grip, -180, 180);
//   if (pulseIn(CH3, HIGH, TimeOut) == 0) {
//     Grip = 0;
//   }
//   GripCurrPos = Roll;
//   GripCurrPos_F = 0.9 * GripCurrPos_F + 0.1 * GripCurrPos;
//   if (GripPrevPos != GripCurrPos) {
//     StepperGrip.move((GripCurrPos_F - GripPrevPos)*50);
//     GripPrevPos = GripCurrPos_F;
//   }
// }

void Default() {
  Rotate = 0;
  RelayOut_1 = 0;
  RelayOut_2 = 0;
  // Pitch = 0;
  // Roll = 0;
  // Grip = 0;
}

void PrintData() {
  Serial.print("Rotate:");
  Serial.print(Rotate);
  Serial.print("  RelayOut_1:");
  Serial.print(RelayOut_1);
  Serial.print("  RelayOut_2:");
  Serial.print(RelayOut_2);
  // Serial.print("  Pitch:");
  // Serial.print(Pitch);
  // Serial.print("  Roll:");
  // Serial.print(Roll);
  // Serial.print("  Grip:");
  // Serial.print(Grip);

  Serial.println();
}

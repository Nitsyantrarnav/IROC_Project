#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

Adafruit_PWMServoDriver myServo = Adafruit_PWMServoDriver(0x40);

// Motor Driver Outputs
#define dirL 2
#define dirR 4
#define signal_LT 3
#define signal_RT 5
#define signal_LM 6
#define signal_RM 9

// Receiver Inputs
#define SteerIn_CH1 7
#define DirIn_CH2 8
#define RpmIn_CH3 10
#define RotateIn_CH4 11
#define SWA_CH5 12
#define SWB_CH6 13
#define RcMin 1300
#define RcMax 1700
#define TimeOut 50000

// ServoPWM Range
#define SERVOMIN 100
#define SERVOMAX 520
#define SERVOMID 310

// Motor Driver Parameters (in metres)
const double x1 = 0.21;
const double x2 = 0.328;
const double y = 0.37;

volatile double rS = 10000;
volatile double rM = 10000;
volatile double rLF;
volatile double rRF;
volatile double rLM;
volatile double rRM;
volatile double rLB;
volatile double rRB;

volatile double vLF;
volatile double vRF;
volatile double vLM;
volatile double vRM;
volatile double vLB;
volatile double vRB;
volatile double vLT;
volatile double vRT;

volatile double w;
int Polarity = 0;
int Rotate = 0;
int rpm = 0;

// Servo Parameters
uint8_t servoLF = 0;
uint8_t servoRF = 1;
uint8_t servoLB = 2;
uint8_t servoRB = 3;

volatile double thLF = 90;
volatile double thLB = 90;
volatile double thRF = 90;
volatile double thRB = 90;
int ServoPWM = 310;
int ServoLF = 310;
int ServoLB = 310;
int ServoRF = 310;
int ServoRB = 310;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);

  // Set Reciever Input pins
  pinMode(DirIn_CH2, INPUT);
  pinMode(RpmIn_CH3, INPUT);
  pinMode(SteerIn_CH1, INPUT);
  pinMode(RotateIn_CH4, INPUT);
  pinMode(SWA_CH5, INPUT);
  pinMode(SWB_CH6, INPUT);

  // Set Motor Driver pins
  pinMode(dirL, OUTPUT);
  pinMode(dirR, OUTPUT);
  pinMode(signal_LT, OUTPUT);
  pinMode(signal_RT, OUTPUT);
  pinMode(signal_LM, OUTPUT);
  pinMode(signal_RM, OUTPUT);

  myServo.begin();
  myServo.setPWMFreq(50);
}

void loop() {
  // put your main code here, to run repeatedly:
  if (pulseIn(SWB_CH6, HIGH, TimeOut) > 1250 && pulseIn(SWA_CH5, HIGH, TimeOut) < 1250) {
    DriveMotor();
    Steer();
    delay(5);
  } else {
    DefaultState();
  }
  // PrintData();
}

void ServoParam() {
  if (((ServoPWM < 10 && ServoPWM > -10) && (Rotate > -50 && Rotate < 50)) || pulseIn(SteerIn_CH1, HIGH, TimeOut) == 0) {
    rLF = rRF = rLM = rRM = rLB = rRB = rS = rS = 10000;
  } else if (Rotate > -50 && Rotate < 50) {
    if (ServoPWM < -10) {
      rS = -(11 + rS);
    } else if (ServoPWM > 10) {
      rS = 11 - rS;
    }
  }

  thLB = 180 / PI * (PI / 2 + atan2(-y * (rS / abs(rS)), (abs(rS) + (x1 * (rS / abs(rS))))));
  thRB = 180 / PI * (PI / 2 + atan2(-y * (rS / abs(rS)), (abs(rS) - (x1 * (rS / abs(rS))))));
  thLF = 180 / PI * (PI / 2 + atan2(y * (rS / abs(rS)), (abs(rS) + (x1 * (rS / abs(rS))))));
  thRF = 180 / PI * (PI / 2 + atan2(y * (rS / abs(rS)), (abs(rS) - (x1 * (rS / abs(rS))))));

  // thL = constrain(thL, (180 / PI * (atan2(y, (x1)))), (180 / PI * (atan2(y, (-x1)))));
  // thR = constrain(thR, (180 / PI * (atan2(y, (x1)))), (180 / PI * (atan2(y, (-x1)))));
  if (Rotate < -50 || Rotate > 50) {
    rS = 0.0001;
    thLF = 180 / PI * (PI / 2 + atan2(y * (rS / abs(rS)), (abs(rS) + (x1 * (rS / abs(rS))))));
    thLB = 180 / PI * (PI / 2 + atan2(-y * (rS / abs(rS)), (abs(rS) + (x1 * (rS / abs(rS))))));
    rS = -rS;
    thRF = 180 / PI * (PI / 2 + atan2(y * (rS / abs(rS)), (abs(rS) - (x1 * (rS / abs(rS))))));
    thRB = 180 / PI * (PI / 2 + atan2(-y * (rS / abs(rS)), (abs(rS) - (x1 * (rS / abs(rS))))));
  }

  ServoLF = (7 * thLF / 3) + 100 - 35;
  ServoRF = (7 * thRF / 3) + 100;
  ServoLB = (7 * thLB / 3) + 100 - 11.67;
  ServoRB = (7 * thRB / 3) + 100;
}

void MotorParam() {
  if (((ServoPWM < 10 && ServoPWM > -10) && (Rotate > -50 && Rotate < 50) || pulseIn(SteerIn_CH1, HIGH, TimeOut) == 0)) {
    rLF = rRF = rLM = rRM = rLB = rRB = rM = 10000;
    w = rpm;
    vLF = vRF = vLM = vRM = vLB = vRB = w;
  } else if (Rotate < -50 || Rotate > 50) {
    rM = 0;
    rLF = -sqrt(sq(rM + x1) + sq(y));
    rRF = sqrt(sq(rM - x1) + sq(y));
    rLM = -(rM + x2);
    rRM = -(rM - x2);
    rLB = rLF;
    rRB = rRF;

    w = rpm;
    vLF = w * abs(rLF);
    vRF = w * abs(rRF);
    vLM = w * abs(rLM);
    vRM = w * abs(rRM);
    vLB = vLT = vLF;
    vRB = vRT = vRF;

  } else if (Rotate > -50 && Rotate < 50) {
    if (ServoPWM < -10) {
      rM = -(11 + rM);
    } else if (ServoPWM > 10) {
      rM = 11 - rM;
    }
    rLF = rM / abs(rM) * sqrt(sq(rM + x1) + sq(y));
    rRF = rM / abs(rM) * sqrt(sq(rM - x1) + sq(y));
    rLM = rM + x2;
    rRM = rM - x2;
    rLB = rLF;
    rRB = rRF;

    // w = abs(map((rpm / (map(ServoPWM, -100, 100, -10, 10))), 0, 10, 0, 17) / 8);
    w = rpm / (max(abs(rLM), abs(rRM)));
    vLF = w * abs(rLF);
    vRF = w * abs(rRF);
    vLM = w * abs(rLM);
    vRM = w * abs(rRM);
    vLB = vLT = vLF;
    vRB = vRT = vRF;
  }
}

void Steer() {
  if (pulseIn(SteerIn_CH1, HIGH, TimeOut) != 0) {
    ServoPWM = map(pulseIn(SteerIn_CH1, HIGH, TimeOut), 1050, 1800, -100, 100);
    ServoPWM = constrain(ServoPWM, -100, 100);
  } else {
    ServoPWM = 0;
  }

  rS = map(ServoPWM, -100, 100, -10, 10);
  rS = constrain(rS, -10, 10);
  rM = map(ServoPWM, -100, 100, -10, 10);
  rM = constrain(rM, -10, 10);
  // int fServo = (0.1 * ServoPWM) + (0.9 * fServo);

  ServoParam();
  myServo.setPWM(servoLF, 0, ServoLF);
  myServo.setPWM(servoRF, 0, ServoRF);
  myServo.setPWM(servoLB, 0, ServoLB);
  myServo.setPWM(servoRB, 0, ServoRB);
}

void DriveMotor() {
  // Reading Reciever Input from CH2, CH3 and CH4
  if (pulseIn(DirIn_CH2, HIGH, TimeOut) != 0) {
    Polarity = map(pulseIn(DirIn_CH2, HIGH, TimeOut), 1080, 1730, -100, 100);
    Polarity = constrain(Polarity, -100, 100);
  } else {
    Polarity = 0;
  }
  if (pulseIn(RpmIn_CH3, HIGH, TimeOut) != 0) {
    rpm = map(pulseIn(RpmIn_CH3, HIGH, TimeOut), 1070, 1740, 0, 250);
    rpm = constrain(rpm, 0, 250);
  } else {
    rpm = 0;
  }
  if (pulseIn(RotateIn_CH4, HIGH, TimeOut) != 0) {
    Rotate = map(pulseIn(RotateIn_CH4, HIGH, TimeOut), 1020, 1800, -100, 100);
    Rotate = constrain(Rotate, -100, 100);
  } else {
    Rotate = 0;
  }
  MotorParam();
  if (Rotate > -50 && Rotate < 50) {
    if (Polarity > 50) {
      digitalWrite(dirL, LOW);
      digitalWrite(dirR, LOW);
      analogWrite(signal_LT, vLT);
      analogWrite(signal_RT, vRT);
      analogWrite(signal_LM, vLM);
      analogWrite(signal_RM, vRM);
    }
    if (Polarity < -50) {
      digitalWrite(dirL, HIGH);
      digitalWrite(dirR, HIGH);
      analogWrite(signal_LT, vLT);
      analogWrite(signal_RT, vRT);
      analogWrite(signal_LM, vLM);
      analogWrite(signal_RM, vRM);

    } else if (Polarity > -50 && Polarity < 50) {
      digitalWrite(dirL, LOW);
      digitalWrite(dirR, LOW);
      analogWrite(signal_LT, LOW);
      analogWrite(signal_RT, LOW);
      analogWrite(signal_LM, LOW);
      analogWrite(signal_RM, LOW);
    }
  } else if (Rotate < -50) {
    delay(100);
    digitalWrite(dirL, HIGH);
    digitalWrite(dirR, LOW);
    analogWrite(signal_LT, vLT);
    analogWrite(signal_RT, vRT);
    analogWrite(signal_LM, vLM);
    analogWrite(signal_RM, vRM);
  } else if (Rotate > 50) {
    delay(100);
    digitalWrite(dirL, LOW);
    digitalWrite(dirR, HIGH);
    analogWrite(signal_LT, vLT);
    analogWrite(signal_RT, vRT);
    analogWrite(signal_LM, vLM);
    analogWrite(signal_RM, vRM);
  }
}

void DefaultState() {
  myServo.setPWM(servoLF, 0, SERVOMID);
  myServo.setPWM(servoRF, 0, SERVOMID);
  myServo.setPWM(servoLB, 0, SERVOMID);
  myServo.setPWM(servoRB, 0, SERVOMID);

  digitalWrite(dirL, LOW);
  digitalWrite(dirR, LOW);
  analogWrite(signal_LT, LOW);
  analogWrite(signal_RT, LOW);
  analogWrite(signal_LM, LOW);
  analogWrite(signal_RM, LOW);
}

void PrintData() {
  Serial.print("SteerIn_CH1:");
  Serial.print(pulseIn(SteerIn_CH1, HIGH, TimeOut));
  Serial.print("  DirIn_CH2:");
  Serial.print(pulseIn(DirIn_CH2, HIGH, TimeOut));
  Serial.print("  RpmIn_CH3:");
  Serial.print(pulseIn(RpmIn_CH3, HIGH, TimeOut));
  Serial.print("  RotateIn_CH4:");
  Serial.print(pulseIn(RotateIn_CH4, HIGH, TimeOut));
  Serial.print("  SWB_CH6:");
  Serial.print(pulseIn(SWB_CH6, HIGH, TimeOut));
  Serial.print("  SWA_CH5:");
  Serial.print(pulseIn(SWA_CH5, HIGH, TimeOut));

  // Serial.print("  ServoPWM:");
  // Serial.print(ServoPWM);
  // Serial.print("  Polarity:");
  // Serial.print(Polarity);
  // Serial.print("  rpm:");
  // Serial.print(rpm);
  // Serial.print("  Rotate:");
  // Serial.print(Rotate);

  // Serial.print("  rS:");
  // Serial.print(rS);
  // Serial.print("  rM:");
  // Serial.print(rM);
  // Serial.print("  rLF:");
  // Serial.print(rLF);
  // Serial.print("  rRF:");
  // Serial.print(rRF);
  // Serial.print("  rLM:");
  // Serial.print(rLM);
  // Serial.print("  rRM:");
  // Serial.print(rRM);

  // Serial.print("  w:");
  // Serial.print(w);
  // Serial.print("  vLF:");
  // Serial.print(vLF);
  // Serial.print("  vRF:");
  // Serial.print(vRF);
  // Serial.print("  vLM:");
  // Serial.print(vLM);
  // Serial.print("  vRM:");
  // Serial.print(vRM);

  // Serial.print("  thLF:");
  // Serial.print(thLF);
  // Serial.print("  thLB:");
  // Serial.print(thLB);
  // Serial.print("  thRF:");
  // Serial.print(thRF);
  // Serial.print("  thRB:");
  // Serial.print(thRB);
  // Serial.print("  ServoLF:");
  // Serial.print(ServoLF);
  // Serial.print("  ServoLB:");
  // Serial.print(ServoLB);
  // Serial.print("  ServoRF:");
  // Serial.print(ServoRF);
  // Serial.print("  ServoRB:");
  // Serial.print(ServoRB);

  Serial.println();
}
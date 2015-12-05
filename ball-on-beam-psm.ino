#include <NewPing.h>

//  Pendefinisian Pin
#define encoderPinA     2
#define encoderPinB     4
#define motorEnPin      10
#define motorIn1Pin     9
#define motorIn2Pin     8
#define pingPin         12
#define pingMaxDistance 37

float error[3] = {0, 0, 0};
float output[2] = {0, 0};

//  Inisialisasi Parameter Encoder
byte encoderPinALast;

float duration;
boolean Direction;

//  Inisialisasi Parameter Kontrol Posisi
float Kp = 0.009;
float Ki = 0.00005;
float Kd = 0.15;

float errPos = 0;
float errDPos = 0;
float errIPos = 0;
float lasterrPos = 0;

float actPos;
float desPos = 22;
float lastPos = 0;

float actAngle = 0;
float maxAngle = 12;
float minAngle = -12;

int cycleTime = 30;
int lastTime = 0;
int startTime = 0;
int endTime = 0;

bool motorDir;

//  Inisialisasi Parameter PWM
float pwmVal = 0;
float pidVal = 0;
float maxPwmVal = 52; //hanya untuk minmax angle +-15
float minPwmVal = 45;

//  Inisialisasi Parameter Sensor PING)))
NewPing sonar(pingPin, pingPin, pingMaxDistance);

unsigned int pingSpeed = 30;
unsigned long pingTimer;

void setup() {
  Serial.begin(9600);
  encoderInit();
  motorInit();
  pingTimer = millis();

}

void loop() {
  startTime = pingTimer;
  if (millis() >= pingTimer) {
    pingTimer += pingSpeed;
    sonar.ping_timer(echoCheck);
  }
  pidPos();
  limitAngle();
}

//  Prosedur inisialisasi encoder
void encoderInit() {
  Direction = true;
  duration = 0;
  pinMode(encoderPinB, INPUT);
  attachInterrupt(0, wheelSpeed, CHANGE);
  Serial.println("Encoder Initialization Complete!");
}

//  Prosedur inisialisasi motor
void motorInit() {
  pinMode(motorEnPin, OUTPUT);
  pinMode(motorIn1Pin, OUTPUT);
  pinMode(motorIn2Pin, OUTPUT);
  Serial.println("Motor Initialization Complete!");
}

//  Prosedur perhitungan posisi encoder
void wheelSpeed() {
  int Lstate = digitalRead(encoderPinA);
  if ((encoderPinALast == LOW) && Lstate == HIGH)
  {
    int val = digitalRead(encoderPinB);
    if (val == LOW && Direction)
    {
      Direction = false; //Reverse
    }
    else if (val == HIGH && !Direction)
    {
      Direction = true;  //Forward
    }
  }
  encoderPinALast = Lstate;

  if (!Direction)  duration++;
  else  duration--;

  actAngle = duration * 360 / 4192;
}

//  Prosedur kontrol putaran motor
void turnMotorCCW() {
  digitalWrite(motorIn1Pin, LOW);
  digitalWrite(motorIn2Pin, HIGH);
  analogWrite(motorEnPin, pwmVal);
  motorDir = false;
}

void turnMotorCW() {
  digitalWrite(motorIn1Pin, HIGH);
  digitalWrite(motorIn2Pin, LOW);
  analogWrite(motorEnPin, pwmVal);
  motorDir = true;
}

void brakeMotor() {
  digitalWrite(motorIn1Pin, HIGH);
  digitalWrite(motorIn2Pin, HIGH);
  analogWrite(motorEnPin, pwmVal);
}

//  Prosedur perhitungan posisi bola oleh sensor
void echoCheck() {
  // Don't do anything here!
  if (sonar.check_timer()) {
    // Here's where you can add code.
    actPos = sonar.ping_result / US_ROUNDTRIP_CM;
  }
  // Don't do anything here!
}

// Batas sudut
void limitAngle() {
  if ((actAngle > maxAngle) && (output[0] < 0)) {
    pwmVal = 60;
    turnMotorCCW();
  }
  else if ((actAngle < minAngle) && (output[0] > 0)) {
    pwmVal = 60;
    turnMotorCW();
  }
}


//  Fungsi perhitungan PID
void pidPos() {
  error[2] = error[1];
  error[1] = error[0];
  error[0] = desPos - actPos;

  output[1] = output[0];
  output[0] = ((Kp + Ki + Kd) * error[0] - (Kp + 2 * Kd) * error[1] + Kd * error[2] + output[1]);

  if (abs(error[1] - error[2]) < 2 && abs(actAngle) < 1) {
    brakeMotor();
  }
  else {
    if (output[0] > 0.1) {
      pwmVal = map(abs(output[0]) * 10, 0, 20, 60, 100); //50 57
      turnMotorCCW();
    }
    else if (output[0] < -0.1) {
      pwmVal = map(abs(output[0]) * 10, 0, 20, 55, 95); //50 57
      turnMotorCW();
    }
  }
}


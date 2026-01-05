#include <util/atomic.h> // For the ATOMIC_BLOCK macro
#include "Wire.h"
#include "I2Cdev.h"
#include "MPU6050.h"

MPU6050 mpu;

int16_t ax, ay, az;
int16_t gx, gy, gz;

struct MyData {
  byte X;
  byte Y;
  byte Z;
};

MyData data;

// Encoder pins
#define ENCA 2 // YELLOW
#define ENCB 3 // WHITE

// Motor driver pins
#define PWM 5
#define IN2 6
#define IN1 7

volatile int posi = 0;
long prevT = 0;
float eprev = 0;
float eintegral = 0;
int target = 0;

void setup() {
  Serial.begin(9600);

  pinMode(ENCA, INPUT);
  pinMode(ENCB, INPUT);
  attachInterrupt(digitalPinToInterrupt(ENCA), readEncoder, RISING);

  pinMode(PWM, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);

  Wire.begin();
  mpu.initialize();
  delay(10);

  int targz[10];
  float sgnt = 1;

  for (int i = 0; i < 10; i++) {
    mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

    data.X = map(ax, -17000, 17000, 0, 255);
    data.Y = map(ay, -17000, 17000, 0, 255);
    data.Z = map(az, -17000, 17000, 0, 255);

    if (127 - data.X <= 0) sgnt = -1;
    targz[i] = data.Z;
  }

  for (int i = 0; i < 10; i++) {
    for (int j = 1; j < 10; j++) {
      if (targz[j - 1] > targz[j]) {
        int temp = targz[j];
        targz[j] = targz[j - 1];
        targz[j - 1] = temp;
      }
    }
  }

  int zval = targz[5];
  target = 600 * sgnt * (255 - (float)zval) / 255;
}

void loop() {
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

  data.X = map(ax, -17000, 17000, 0, 255);
  data.Y = map(ay, -17000, 17000, 0, 255);
  data.Z = map(az, -17000, 17000, 0, 255);

  float kp = 3;
  float kd = 0.065;
  float ki = 0;

  long currT = micros();
  float deltaT = ((float)(currT - prevT)) / 1.0e6;
  prevT = currT;

  int pos = 0;
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    pos = posi;
  }

  int e = target - pos;
  float dedt = (e - eprev) / deltaT;
  eintegral = eintegral + e * deltaT;

  float u = kp * e + kd * dedt + ki * eintegral;
  float pwr = fabs(u);
  if (pwr > 255) pwr = 255;

  int dir = (u < 0) ? -1 : 1;
  setMotor(dir, pwr, PWM, IN1, IN2);

  eprev = e;
}

void setMotor(int dir, int pwmVal, int pwm, int in1, int in2) {
  analogWrite(pwm, pwmVal);
  if (dir == 1) {
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
  } else {
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
  }
}

void readEncoder() {
  int b = digitalRead(ENCB);
  if (b > 0) posi++;
  else posi--;
}

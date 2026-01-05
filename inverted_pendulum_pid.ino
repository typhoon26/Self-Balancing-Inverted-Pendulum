// We have declared the required modules that we are going to use like mpu
#include <util/atomic.h>   // For the ATOMIC_BLOCK macro
#include "Wire.h"
#include "I2Cdev.h"
#include "MPU6050.h"

// Creating MPU object
MPU6050 mpu;

// Variables to store accelerometer and gyroscope values
int16_t ax, ay, az;
int16_t gx, gy, gz;

// Structure for storing mapped IMU data
struct MyData {
  byte X;
  byte Y;
  byte Z;
};

MyData data;

// defining my port from encoded motor to the arduino both yellow and white one
#define ENCA 2 // YELLOW
#define ENCB 3 // WHITE

// defining motor driver to the arduino
// one for anticlockwise rotation, other for clockwise rotation
// PWM is like a driver for speed control
#define PWM 5
#define IN2 6
#define IN1 7

// Defining global variable that we are going to use in the code
volatile int posi = 0; // specify posi as volatile
// https://www.arduino.cc/reference/en/language/variables/variable-scope-qualifiers/volatile/

long prevT = 0;
float eprev = 0;
float eintegral = 0;

int target = 0;

void setup() {

  Serial.begin(9600);

  // Setting up encoder pins
  pinMode(ENCA, INPUT);
  pinMode(ENCB, INPUT);
  attachInterrupt(digitalPinToInterrupt(ENCA), readEncoder, RISING);

  // Setting up the pinmode for motor driver
  pinMode(PWM, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);

  // Setting up the MPU for getting accurate position for moving
  Wire.begin();
  mpu.initialize();
  delay(10);

  int targz[10];     // storage for IMU Z-axis data
  float sgnt = 1;    // +ve implies x is east of z axis

  // We find the initial position using the IMU
  // A sample of 10 is used to improve robustness to the IMU data
  for (int i = 0; i < 10; i++) {

    mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

    data.X = map(ax, -17000, 17000, 0, 255); // X axis data
    data.Y = map(ay, -17000, 17000, 0, 255); // Y axis data
    data.Z = map(az, -17000, 17000, 0, 255); // Z axis data

    // Determining sign convention
    if (127 - data.X <= 0) sgnt = -1;

    targz[i] = data.Z; // determining initial position
  }

  // Bubble sort for median filtering
  for (int i = 0; i < 10; i++) {
    for (int j = 1; j < 10; j++) {
      if (targz[j - 1] > targz[j]) {
        int temp = targz[j];
        targz[j] = targz[j - 1];
        targz[j - 1] = temp;
      }
    }
  }

  int zval = targz[5]; // median value

  // 180 degree for encoder is 600
  // 255 is the value of x/y/z when axis is faced up
  // 0 is the value when axis is faced down
  // Sign convention determined by IMU attachment
  target = 600 * sgnt * (255 - (float)zval) / 255;
}

void loop() {

  // Setting up MPU
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

  // Mapping IMU values
  data.X = map(ax, -17000, 17000, 0, 255);
  data.Y = map(ay, -17000, 17000, 0, 255);
  data.Z = map(az, -17000, 17000, 0, 255);

  // Definition of the PID constants
  float kp = 3;
  float kd = 0.065;
  float ki = 0;

  // Calculate the time difference used in u(t)
  long currT = micros();
  float deltaT = ((float)(currT - prevT)) / 1.0e6;
  prevT = currT;

  // Read the position in an atomic block to avoid misread
  // https://www.arduino.cc/reference/en/language/variables/variable-scope-qualifiers/volatile/
  int pos = 0;
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    pos = posi;
  }

  // Error calculation
  int e = target - pos;

  // Derivative
  float dedt = (e - eprev) / deltaT;

  // Integral
  eintegral = eintegral + e * deltaT;

  // Control signal tells motor direction and speed
  float u = kp * e + kd * dedt + ki * eintegral;

  // Motor power
  float pwr = fabs(u);
  if (pwr > 255) {
    pwr = 255;
  }

  // Motor direction
  int dir = 1;
  if (u < 0) {
    dir = -1;
  }

  // Setting motor direction and speed
  setMotor(dir, pwr, PWM, IN1, IN2);

  // Store previous error
  eprev = e;
}

// A helper function used for setting the motor direction
void setMotor(int dir, int pwmVal, int pwm, int in1, int in2) {

  analogWrite(pwm, pwmVal);

  if (dir == 1) {
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
  }
  else if (dir == -1) {
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
  }
  else {
    digitalWrite(in1, LOW);
    digitalWrite(in2, LOW);
  }
}

// Read encoder helps in determining the position of the bob
void readEncoder() {

  int b = digitalRead(ENCB);

  if (b > 0) {
    posi++;
  }
  else {
    posi--;
  }
}

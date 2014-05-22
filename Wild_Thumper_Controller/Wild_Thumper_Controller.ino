#include "IOpins.h"
#include "Constants.h"
#include <Wire.h>
#include <LSM303.h>

// Globals
unsigned int batteryVoltage;
unsigned int leftAmps;
unsigned int rightAmps;
unsigned long lastCommandTime;
unsigned long reportTimer;
unsigned long leftoverload;
unsigned long rightoverload;
int highVolts;
int startVolts;
int leftSpeed = 0;
int rightSpeed = 0;
int Speed;
int Steer;

// Whether the battery is dead
unsigned int batteryDead = 0;

// The communication mode
int commMode = MODE_RC;

// 0 = reverse, 1 = brake, 2 = forward
int leftMode = 1;

// 0 = reverse, 1 = brake, 2 = forward
int rightMode = 1;

// PWM value for left  motor speed / brake
int leftPWM;

// PWM value for right motor speed / brake
int rightPWM;

// Ping distance
long leftDist, centerDist, rightDist;

// Compass and heading
LSM303 compass;
float heading;

// Serial data
int data;

void setup() {
  // Initialize I/O pins

  // Change charger pin to output
  // Disable current regulator
  pinMode(PIN_CHARGER, OUTPUT);
  digitalWrite(PIN_CHARGER, 1);

  // Enable pullups to put A4 and A5 into I2C mode
  digitalWrite(D18, 1);
  digitalWrite(D19, 1);

  // Start the compass
  Wire.begin();
  compass.init();
  compass.enableDefault();

  // Compass calibration - in box, wire tied
  compass.m_min = (LSM303::vector<int16_t>){+717, -8921, -1185};
  compass.m_max = (LSM303::vector<int16_t>){+6771, -2171, +4772};

  // Enable serial
  Serial.begin(BAUDRATE);
  Serial.flush();

  // Print a newline so the next command isn't garbled
  Serial.println("");

  // Say hi
  Serial.println("{\"type\":\"ready\"}");
  Serial.println("{\"type\":\"log\",\"message\":\"Hello from dilBot!\"}");
}


void loop() {
  // Check battery voltage and current draw of motors

  // Read the battery voltage
  batteryVoltage = analogRead(PIN_BATTERY);

  // Read left motor current draw
  leftAmps = analogRead(PIN_MOTOR_LEFT_CURRENT);

  // Read right motor current draw
  rightAmps = analogRead(PIN_MOTOR_RIGHT_CURRENT);

  if (leftAmps > LEFTMAXAMPS) { // is motor current draw exceeding safe limit
    // Turn off motors
    analogWrite(PIN_MOTOR_LEFT_A, 0);
    analogWrite(PIN_MOTOR_LEFT_B, 0);

    // Record time of overload
    leftoverload = millis();
  }

  if (rightAmps > RIGHTMAXAMPS) { // is motor current draw exceeding safe limit
    // Turn off motors
    analogWrite(PIN_MOTOR_RIGHT_A, 0);
    analogWrite(PIN_MOTOR_RIGHT_B, 0);

    // Record time of overload
    rightoverload = millis();
  }

  // Check if battery is dead
  // This is a safety feature to prevent malfunction at low voltages
  if (batteryVoltage < LOWVOLT) {
    batteryDead = 1;
    turnOffMotors();
    reportDeadBattery();
  }

  // Always read from serial
  SCmode();

  if (!batteryDead) {
    // Only power motors if battery voltage is good
    // Drive dual "H" bridges
    if ((millis() - leftoverload) > OVERLOADTIME) {
      // if left motor has not overloaded recently
      switch (leftMode) {
        // left motor forward
        case FORWARD:
          analogWrite(PIN_MOTOR_LEFT_A, leftPWM);
          analogWrite(PIN_MOTOR_LEFT_B, 0);
          break;

        // left motor brake
        case BRAKE:
          analogWrite(PIN_MOTOR_LEFT_A, leftPWM);
          analogWrite(PIN_MOTOR_LEFT_B, leftPWM);
          break;

        // left motor reverse
        case REVERSE:
          analogWrite(PIN_MOTOR_LEFT_A, 0);
          analogWrite(PIN_MOTOR_LEFT_B, leftPWM);
          break;
      }
    }

    if ((millis() - rightoverload) > OVERLOADTIME) {
      // if right motor has not overloaded recently
      switch (rightMode) {
        // right motor forward
        case FORWARD:
          analogWrite(PIN_MOTOR_RIGHT_A, rightPWM);
          analogWrite(PIN_MOTOR_RIGHT_B, 0);
          break;

        // right motor brake
        case BRAKE:
          analogWrite(PIN_MOTOR_RIGHT_A, rightPWM);
          analogWrite(PIN_MOTOR_RIGHT_B, rightPWM);
          break;

        // right motor reverse
        case REVERSE:
          analogWrite(PIN_MOTOR_RIGHT_A, 0);
          analogWrite(PIN_MOTOR_RIGHT_B, rightPWM);
          break;
      }
    }

    // Read distances
    leftDist = doPing(PING_LEFT);
    centerDist = doPing(PING_CENTER);
    rightDist = doPing(PING_RIGHT);

    // Get the current heading
    compass.read();
    heading = compass.heading();
  }

  // Send report
  reportState();
}


//
// Code for RC inputs
//
void RCmode() {
  // read throttle/left stick
  Speed = pulseIn(RC_LEFT, HIGH, 25000);

  // read steering/right stick
  Steer = pulseIn(RC_RIGHT, HIGH, 25000);

  // if pulseIn times out (25mS) then set speed to stop
  if (Speed == 0) Speed = RC_CENTER;

  // if pulseIn times out (25mS) then set steer to centre
  if (Steer == 0) Steer = RC_CENTER;

  // if Speed input is within deadband set to 1500 (1500uS = center position for most servos)
  if (abs(Speed - RC_CENTER) < RC_DEADBAND) Speed = RC_CENTER;

  // if Steer input is within deadband set to 1500 (1500uS = center position for most servos)
  if (abs(Steer - RC_CENTER) < RC_DEADBAND) Steer = RC_CENTER;

  // Store last command time
  lastCommandTime = millis();

  if (RC_MIX == 1) {
    // Mixes speed and steering signals
    Steer = Steer - RC_CENTER;
    leftSpeed = Speed + Steer;
    rightSpeed = Speed - Steer;
  }
  else {
    // Individual stick control
    leftSpeed = Speed;
    rightSpeed = Steer;
  }

  if (
    rightSpeed > (RC_CENTER - RC_DEADBAND) && rightSpeed < (RC_CENTER + RC_DEADBAND) &&
    leftSpeed > (RC_CENTER - RC_DEADBAND) && leftSpeed < (RC_CENTER + RC_DEADBAND)
  ) {
    // Stick is in deadband, robot should stop
    stop();
  }
  else {
    if (leftSpeed > (RC_CENTER + RC_DEADBAND)) {
      // Stick is forward
      leftMode = FORWARD;
    }
    else {
      // Stick is reversed
      leftMode = REVERSE;
    }

    if (rightSpeed > (RC_CENTER + RC_DEADBAND)) {
      // Stick is forward
      rightMode = FORWARD;
    }
    else {
      // Stick is reversed
      rightMode = REVERSE;
    }

    // scale 1000-2000uS to 0-255
    leftPWM = abs(leftSpeed - RC_CENTER) * 10 / RC_SCALE;

    // set maximum limit 255
    leftPWM = min(leftPWM, 255);

    // scale 1000-2000uS to 0-255
    rightPWM = abs(rightSpeed - RC_CENTER) * 10 / RC_SCALE;

    // set maximum limit 255
    rightPWM = min(rightPWM, 255);
  }
}

//
// Code for Serial Communications
//
// ST = Stop
// MO = Set control mode
// HB = "H" bridge data - next 4 bytes will be:
//      left  motor mode 0-2
//      left  motor PWM  0-255
//      right motor mode 0-2
//      right motor PWM  0-255
void SCmode() {
  if (Serial.available() > 1) {
    int A = Serial.read();
    int B = Serial.read();
    int command = A * 256 + B;
    switch (command) {
      // Stop
      case ST:
        stop();
        break;

      // Change mode
      case MO:
        serialRead();
        commMode = data;

        break;

       // Set mode and PWM data for left and right motors
      case HB:
        serialRead();
        leftMode = data;
        serialRead();
        leftPWM = data;
        serialRead();
        rightMode = data;
        serialRead();
        rightPWM = data;

        lastCommandTime = millis();
        break;

       // invalid command
       default:
         Serial.flush();
    }
  }

  if (commMode == MODE_RC) {
    // Run RC input detection
    RCmode();
  }
  else if (commMode == MODE_SERIAL && (leftMode != BRAKE || rightMode != BRAKE) && (millis() - lastCommandTime > COMMANDTIMEOUT)) {
    // Prevent runaway when connection drops
    stop();
  }
}

//
// Read serial port until data has been received
//
void serialRead() {
  do {
    data = Serial.read();
  } while (data < 0);
}

// Stop by applying brakes
void stop() {
  leftMode = BRAKE;
  leftPWM = 255;
  rightMode = BRAKE;
  rightPWM = 255;
}

void reportState() {
  Serial.print("{\"type\":\"state\"");
  Serial.print(",\"heading\":");
  Serial.print(heading);
  Serial.print(",\"leftDist\":");
  Serial.print(leftDist);
  Serial.print(",\"centerDist\":");
  Serial.print(centerDist);
  Serial.print(",\"rightDist\":");
  Serial.print(rightDist);
  Serial.print(",\"commMode\": ");
  Serial.print(commMode);
  Serial.print(",\"battery\": ");
  Serial.print(batteryVoltage);
  Serial.print(",\"batteryDead\": ");
  Serial.print(batteryDead);
  Serial.print(",\"leftMode\": ");
  Serial.print(leftMode);
  Serial.print(",\"leftPWM\": ");
  Serial.print(leftPWM);
  Serial.print(",\"rightMode\": ");
  Serial.print(rightMode);
  Serial.print(",\"rightPWM\": ");
  Serial.print(rightPWM);
  Serial.println("}");
}

void reportDeadBattery() {
  Serial.print("{\"type\":\"batteryDead\"");
  Serial.print(",\"battery\": ");
  Serial.print(batteryVoltage);
  Serial.println("}");
}

long doPing(int pin) {
  long duration;

  // The PING))) is triggered by a HIGH pulse of 2 or more microseconds.
  // Give a short LOW pulse beforehand to ensure a clean HIGH pulse:
  pinMode(pin, OUTPUT);
  digitalWrite(pin, LOW);
  delayMicroseconds(2);
  digitalWrite(pin, HIGH);
  delayMicroseconds(5);
  digitalWrite(pin, LOW);

  // The same pin is used to read the signal from the PING))): a HIGH
  // pulse whose duration is the time (in microseconds) from the sending
  // of the ping to the reception of its echo off of an object.
  pinMode(pin, INPUT);
  duration = pulseIn(pin, HIGH);

  // convert the time into centimeters
  return microsecondsToCentimeters(duration);
}

// The speed of sound is 340 m/s or 29 microseconds per centimeter.
// The ping travels out and back, so to find the distance of the
// object we take half of the distance travelled.
// See: http://www.parallax.com/dl/docs/prod/acc/28015-PING-v1.3.pdf
long microsecondsToCentimeters(long microseconds) {
  return microseconds / 29.0 / 2.0;
}

void turnOffMotors() {
  // Disable motors and reset modes for when we resume
  leftPWM = 0;
  leftMode = FORWARD;
  rightPWM = 0;
  rightMode = FORWARD;

  // Turn off motors
  analogWrite(PIN_MOTOR_LEFT_A, 0);
  analogWrite(PIN_MOTOR_LEFT_B, 0);
  analogWrite(PIN_MOTOR_RIGHT_A, 0);
  analogWrite(PIN_MOTOR_RIGHT_B, 0);
}

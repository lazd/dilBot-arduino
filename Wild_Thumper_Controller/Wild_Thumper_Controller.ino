#include "IOpins.h"
#include "Constants.h"

//
// define global variables
//
unsigned int batteryVoltage;
unsigned int LeftAmps;
unsigned int RightAmps;
unsigned long lastCommandTime;
unsigned long chargeStart;
unsigned long chargeTimer;
unsigned long reportTimer;
unsigned long leftoverload;
unsigned long rightoverload;
int highVolts;
int startVolts;
int leftSpeed = 0;
int rightSpeed = 0;
int Speed;
int Steer;

// 0 = Flat battery  1 = Charged battery
byte isCharged = 1;

// 0 = reverse, 1 = brake, 2 = forward
int leftMode = 1;

// 0 = reverse, 1 = brake, 2 = forward
int rightMode = 1;

// PWM value for left  motor speed / brake
int leftPWM;

// PWM value for right motor speed / brake
int rightPWM;

int data;

void setup() {
  //
  // Initialize I/O pins
  //

  // Change charger pin to output
  pinMode(Charger, OUTPUT);

  // Disable current regulator to charge battery
  digitalWrite(Charger, 1);

  if (Cmode == 1) {
    // enable serial communications if Cmode = 1
    Serial.begin(Brate);

  }
  else {
    // Enable regardless for logging
    Serial.begin(Brate);
  }

  Serial.flush();

  // Say hi
  Serial.println("{\"type\":\"hello\"}");
}


void loop() {
  //
  // Check battery voltage and current draw of motors
  //

  // read the battery voltage
  batteryVoltage = analogRead(Battery);

  // read left motor current draw
  LeftAmps = analogRead(LmotorC);

  // read right motor current draw
  RightAmps = analogRead(RmotorC);

  if (LeftAmps>Leftmaxamps) { // is motor current draw exceeding safe limit
    // Turn off motors
    analogWrite(LmotorA, 0);
    analogWrite(LmotorB, 0);

    // Record time of overload
    leftoverload = millis();
  }

  if (RightAmps>Rightmaxamps) { // is motor current draw exceeding safe limit
    // Turn off motors
    analogWrite(RmotorA, 0);
    analogWrite(RmotorB, 0);

    // Record time of overload
    rightoverload = millis();
  }

  // check condition of the battery
  if ((batteryVoltage < LOWVOLT) && (isCharged == 1)) {
    // change battery status from charged to flat
    //
    // FLAT BATTERY speed controller shuts down until battery is recharged
    //
    //
    // This is a safety feature to prevent malfunction at low voltages!!
    //

    // Battery is flat
    isCharged = 0;

    // Record the voltage
    highVolts = batteryVoltage;
    startVolts = batteryVoltage;

    // Record the time
    chargeTimer = millis();

    // Record start time
    chargeStart = millis();

    // Disable motors and reset modes for when we resume
    leftPWM = 0;
    leftMode = BRAKE;
    rightPWM = 0;
    rightMode = BRAKE;

    // Enable current regulator to charge battery
    digitalWrite(Charger,0);

    reportDeadBattery();
  }

  if ((isCharged == 0) && (batteryVoltage - startVolts > UNITSPERVOLT)) {
    //
    // CHARGE BATTERY
    //
    // if battery is flat and charger has been connected (voltage has increased by at least 1V)

    // has battery voltage increased?
    if (batteryVoltage > highVolts) {
      // record the highest voltage. Used to detect peak charging.
      highVolts = batteryVoltage;

      // when voltage increases record the time
      chargeTimer = millis();
    }

    // battery voltage must be higher than this before peak charging can occur.
    if (batteryVoltage > BATVOLT) {
      // has voltage begun to drop or levelled out?
      if ((highVolts - batteryVoltage) > 5 || (millis() - chargeTimer) > CHARGETIMEOUT) {
        // battery voltage has peaked
        isCharged = 1;

        // turn off current regulator
        digitalWrite(Charger, 1);

        reportChargeComplete(millis() - chargeStart);
      }
    }
  }

  else {
    //
    // GOOD BATTERY speed controller operates normally
    //
    // @todo does this need to go into the isCharged block?
    switch(Cmode) {
      // RC mode via D0 and D1
      case MODE_RC:
        RCmode();
        break;

      // Serial mode via D0(RX) and D1(TX)
      case MODE_SERIAL:
        SCmode();
        break;
    }

    //
    // Code to drive dual "H" bridges
    //
    if (isCharged == 1) {
      // Only power motors if battery voltage is good
      if ((millis() - leftoverload) > overloadtime) {
        // if left motor has not overloaded recently
        switch (leftMode) {
          // left motor forward
          case FORWARD:
            analogWrite(LmotorA, 0);
            analogWrite(LmotorB, leftPWM);
            break;

          // left motor brake
          case BRAKE:
            analogWrite(LmotorA, leftPWM);
            analogWrite(LmotorB, leftPWM);
            break;

          // left motor reverse
          case REVERSE:
            analogWrite(LmotorA, leftPWM);
            analogWrite(LmotorB, 0);
            break;
        }
      }

      if ((millis() - rightoverload) > overloadtime) {
        // if right motor has not overloaded recently
        switch (rightMode) {
          // right motor forward
          case FORWARD:
            analogWrite(RmotorA, 0);
            analogWrite(RmotorB, rightPWM);
            break;

          // right motor brake
          case BRAKE:
            analogWrite(RmotorA, rightPWM);
            analogWrite(RmotorB, rightPWM);
            break;

          // right motor reverse
          case REVERSE:
            analogWrite(RmotorA, rightPWM);
            analogWrite(RmotorB, 0);
            break;
        }
      }
    }
    else {
      // Battery is flat
      // Turn off motors
      analogWrite(LmotorA, 0);
      analogWrite(LmotorB, 0);
      analogWrite(RmotorA, 0);
      analogWrite(RmotorB, 0);
    }
  }

  // Send report
  reportState();
}


//
// Code for RC inputs
//
void RCmode() {
  // read throttle/left stick
  Speed = pulseIn(RCleft, HIGH, 25000);

  // read steering/right stick
  Steer = pulseIn(RCright, HIGH, 25000);

  // if pulseIn times out (25mS) then set speed to stop
  if (Speed == 0) Speed = 1500;

  // if pulseIn times out (25mS) then set steer to centre
  if (Steer == 0) Steer = 1500;

  // if Speed input is within deadband set to 1500 (1500uS = center position for most servos)
  if (abs(Speed-1500) < RCDEADBAND) Speed = 1500;

  // if Steer input is within deadband set to 1500 (1500uS = center position for most servos)
  if (abs(Steer-1500) < RCDEADBAND) Steer = 1500;

  if (RCMIX == 1) {
    // Mixes speed and steering signals
    Steer = Steer - 1500;
    leftSpeed = Speed + Steer;
    rightSpeed = Speed - Steer;
  }
  else {
    // Individual stick control
    leftSpeed = Speed;
    rightSpeed = Steer;
  }

  leftMode = REVERSE;
  rightMode = REVERSE;

  // if left input is forward then set left mode to forward
  if (leftSpeed > (RCLEFTCENTER + RCDEADBAND)) {
    leftMode = FORWARD;
  }

  // if right input is forward then set right mode to forward
  if (rightSpeed > (RCRIGHTCENTER + RCDEADBAND)) {
    rightMode = FORWARD;
  }

  // scale 1000-2000uS to 0-255
  leftPWM = abs(leftSpeed - RCLEFTCENTER) * 10 / RCSCALE;

  // set maximum limit 255
  leftPWM = min(leftPWM, 255);

  // scale 1000-2000uS to 0-255
  rightPWM = abs(rightSpeed - RCRIGHTCENTER) * 10 / RCSCALE;

  // set maximum limit 255
  rightPWM = min(rightPWM, 255);
}

//
// Code for Serial Communications
//
// FL = flush serial buffer
// AN = report Analog inputs 1-5
// SV = next 7 integers will be position information for servos 0-6
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

      // Flush buffer
      case FL:
        Serial.flush();
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

  if ((leftMode != BRAKE || rightMode != BRAKE) && (millis() - lastCommandTime > COMMANDTIMEOUT)) {
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
  Serial.print(",\"isCharged\": ");
  Serial.print(isCharged);
  Serial.print(",\"battery\": ");
  Serial.print(batteryVoltage);
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

void reportChargeComplete(int chargeTime) {
  Serial.print("{\"type\":\"batteryCharged\"");
  Serial.print(",\"time\": ");
  Serial.print(chargeTime);
  Serial.print(",\"battery\": ");
  Serial.print(batteryVoltage);
  Serial.println("}");
}

void reportDeadBattery() {
  Serial.print("{\"type\":\"batteryDead\"");
  Serial.print(",\"battery\": ");
  Serial.print(batteryVoltage);
  Serial.println("}");
}

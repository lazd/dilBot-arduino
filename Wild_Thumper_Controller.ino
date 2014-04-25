#include <Servo.h>
#include "IOpins.h"
#include "Constants.h"

//
// define global variables
//
unsigned int Volts;
unsigned int LeftAmps;
unsigned int RightAmps;
unsigned long chargeStart;
unsigned long chargeTimer;
unsigned long reportTimer;
unsigned long leftoverload;
unsigned long rightoverload;
int highVolts;
int startVolts;
int Leftspeed = 0;
int Rightspeed = 0;
int Speed;
int Steer;

// 0 = Flat battery  1 = Charged battery
byte Charged = 1;

// 0 = reverse, 1 = brake, 2 = forward
int Leftmode = 1;

// 0 = reverse, 1 = brake, 2 = forward
int Rightmode = 1;

// PWM value for left  motor speed / brake
int LeftPWM;

// PWM value for right motor speed / brake
int RightPWM;

int data;
int servo[7];

//
// define servos
//
Servo Servo0;
Servo Servo1;
Servo Servo2;
Servo Servo3;
Servo Servo4;
Servo Servo5;
Servo Servo6;

void setup()
{
  //
  // Attach servos to I/O pins
  //
  Servo0.attach(S0);
  Servo1.attach(S1);
  Servo2.attach(S2);
  Servo3.attach(S3);
  Servo4.attach(S4);
  Servo5.attach(S5);
  Servo6.attach(S6);

  //
  // Set servos to default position
  //
  Servo0.writeMicroseconds(DServo0);
  Servo1.writeMicroseconds(DServo1);
  Servo2.writeMicroseconds(DServo2);
  Servo3.writeMicroseconds(DServo3);
  Servo4.writeMicroseconds(DServo4);
  Servo5.writeMicroseconds(DServo5);
  Servo6.writeMicroseconds(DServo6);

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

    // flush buffer
    Serial.flush();
  }
  else {
    // Enable regardless for logging
    Serial.begin(Brate);
  }

  Serial.print("Wild Thumper started in ");

  switch(Cmode) {
    case 0:
      Serial.print("RC");
      break;
    case 1:
      Serial.print("Serial");
      break;
    case 2:
      Serial.print("I2C");
      break;
  }

  Serial.println(" mode");
  logBatteryVoltage();
}


void loop() {
  //
  // Check battery voltage and current draw of motors
  //

  // read the battery voltage
  Volts = analogRead(Battery);

  // read left motor current draw
  LeftAmps = analogRead(LmotorC);

  // read right motor current draw
  RightAmps = analogRead(RmotorC);

  //Serial.print(LeftAmps);
  //Serial.print("    ");
  //Serial.println(RightAmps);

  if (LeftAmps>Leftmaxamps) { // is motor current draw exceeding safe limit
    // Turn off motors
    analogWrite (LmotorA,0);
    analogWrite (LmotorB,0);

    // Record time of overload
    leftoverload = millis();
  }

  if (RightAmps>Rightmaxamps) { // is motor current draw exceeding safe limit
    // Turn off motors
    analogWrite (RmotorA,0);
    analogWrite (RmotorB,0);

    // Record time of overload
    rightoverload = millis();
  }

  if ((Volts<lowvolt) && (Charged == 1)) { // check condition of the battery                                                           // change battery status from charged to flat
    //
    // FLAT BATTERY speed controller shuts down until battery is recharged
    //
    //
    // This is a safety feature to prevent malfunction at low voltages!!
    //

    // Battery is flat
    Charged = 0;

    // Record the voltage
    highVolts = Volts;
    startVolts = Volts;

    // Record the time
    chargeTimer = millis();

    // Record start time
    chargeStart = millis();

    Serial.print("Battery dead at ");
    Serial.print(Volts, DEC);
    Serial.println(", enabling charger...");

    // Enable current regulator to charge battery
    digitalWrite(Charger,0);
  }

  if ((Charged == 0) && (Volts-startVolts>67)) {
    //
    // CHARGE BATTERY
    //
    // if battery is flat and charger has been connected (voltage has increased by at least 1V)

    // has battery voltage increased?
    if (Volts>highVolts) {
      // record the highest voltage. Used to detect peak charging.
      highVolts = Volts;

      // when voltage increases record the time
      chargeTimer = millis();
    }

    if ((millis()-reportTimer) > reportInterval) {
      logBatteryVoltage();
      reportTimer = millis();
    }

    // battery voltage must be higher than this before peak charging can occur.
    if (Volts>batvolt) {
      // has voltage begun to drop or levelled out?
      if ((highVolts-Volts)>5 || (millis()-chargeTimer)>chargetimeout) {
        // battery voltage has peaked
        Charged = 1;

        // turn off current regulator
        digitalWrite(Charger,1);

        Serial.print("Charge complete after ");
        Serial.print((millis()-chargeStart)*1000 / 60);
        Serial.print(":");
        Serial.print((millis()-chargeStart)*1000 % 60);
        Serial.println("!");

        logBatteryVoltage();
      }
    }
  }

  else {
    //
    // GOOD BATTERY speed controller opperates normally
    //
    switch(Cmode) {
      // RC mode via D0 and D1
      case 0:
        RCmode();
        break;

      // Serial mode via D0(RX) and D1(TX)
      case 1:
        SCmode();
        break;

      // I2C mode via A4(SDA) and A5(SCL)
      case 2:
        I2Cmode();
        break;
    }

    //
    // Code to drive dual "H" bridges
    //

    if (Charged == 1) { // Only power motors if battery voltage is good
      if ((millis()-leftoverload)>overloadtime) {
        switch (Leftmode) { // if left motor has not overloaded recently
          case 2:                                               // left motor forward
            analogWrite(LmotorA,0);
            analogWrite(LmotorB,LeftPWM);
            break;

          case 1:                                               // left motor brake
            analogWrite(LmotorA,LeftPWM);
            analogWrite(LmotorB,LeftPWM);
            break;

          case 0:                                               // left motor reverse
            analogWrite(LmotorA,LeftPWM);
            analogWrite(LmotorB,0);
            break;
        }
      }

      if ((millis()-rightoverload)>overloadtime) {
        switch (Rightmode) { // if right motor has not overloaded recently
          case 2:                                               // right motor forward
            analogWrite(RmotorA,0);
            analogWrite(RmotorB,RightPWM);
            break;

          case 1:                                               // right motor brake
            analogWrite(RmotorA,RightPWM);
            analogWrite(RmotorB,RightPWM);
            break;

          case 0:                                               // right motor reverse
            analogWrite(RmotorA,RightPWM);
            analogWrite(RmotorB,0);
            break;
        }
      }
    }
    else {
      // Battery is flat
      // Turn off motors
      analogWrite(LmotorA,0);
      analogWrite(LmotorB,0);
      analogWrite(RmotorA,0);
      analogWrite(RmotorB,0);
    }
  }
}


//
// Code for RC inputs
//
void RCmode() {
  // read throttle/left stick
  Speed = pulseIn(RCleft,HIGH,25000);

  // read steering/right stick
  Steer = pulseIn(RCright,HIGH,25000);

  // if pulseIn times out (25mS) then set speed to stop
  if (Speed == 0) Speed = 1500;

  // if pulseIn times out (25mS) then set steer to centre
  if (Steer == 0) Steer = 1500;

  // if Speed input is within deadband set to 1500 (1500uS = center position for most servos)
  if (abs(Speed-1500)<RCdeadband) Speed = 1500;

  // if Steer input is within deadband set to 1500 (1500uS = center position for most servos)
  if (abs(Steer-1500)<RCdeadband) Steer = 1500;

  if (Mix == 1) {
    // Mixes speed and steering signals
    Steer = Steer-1500;
    Leftspeed = Speed-Steer;
    Rightspeed = Speed+Steer;
  }
  else {
    // Individual stick control
    Leftspeed = Speed;
    Rightspeed = Steer;
  }

  /*
  Serial.print("Left:");
  Serial.println(Leftspeed);
  Serial.print("Right:");
  Serial.println(Rightspeed);
  */

  Leftmode = 2;
  Rightmode = 2;

  // if left input is forward then set left mode to forward
  if (Leftspeed>(Leftcenter+RCdeadband)) {
    Leftmode = 0;
  }

  // if right input is forward then set right mode to forward
  if (Rightspeed>(Rightcenter+RCdeadband)) {
    Rightmode = 0;
  }

  // scale 1000-2000uS to 0-255
  LeftPWM = abs(Leftspeed-Leftcenter)*10/scale;

  // set maximum limit 255
  LeftPWM = min(LeftPWM,255);

  // scale 1000-2000uS to 0-255
  RightPWM = abs(Rightspeed-Rightcenter)*10/scale;

  // set maximum limit 255
  RightPWM = min(RightPWM,255);
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
  if (Serial.available()>1) {
    int A = Serial.read();
    int B = Serial.read();
    int command = A*256+B;
    switch (command) {
      // Stop
      case ST:
         Leftmode = 1;
         LeftPWM = 0;
         Rightmode = 1;
         RightPWM = 0;
         break;

      // Check battery level
      case BT:
        logBatteryVoltage();
        break;

      // Flush buffer
      case FL:
        Serial.flush();
        break;

      // AN - return values of analog inputs 1-5
      case AN:
        Serial.println('Reporting analog input values');
        // index analog inputs 1-5
        for (int i = 1;i<6;i++) {
          // read 10bit analog input
          data = analogRead(i);

          // transmit high byte
          Serial.write(highByte(data));

          // transmit low byte
          Serial.write(lowByte(data));
        }
        break;

        // SV - receive postion information for servos 0-6
       case SV:
         // read 14 bytes of data
         for (int i = 0;i<15;i++) {
           Serialread();
           servo[i] = data;
         }
         // set servo positions
         Servo0.writeMicroseconds(servo[0]*256+servo[1]);
         Servo1.writeMicroseconds(servo[2]*256+servo[3]);
         Servo2.writeMicroseconds(servo[4]*256+servo[5]);
         Servo3.writeMicroseconds(servo[6]*256+servo[7]);
         Servo4.writeMicroseconds(servo[8]*256+servo[9]);
         Servo5.writeMicroseconds(servo[10]*256+servo[11]);
         Servo6.writeMicroseconds(servo[12]*256+servo[13]);
         break;

       // Set mode and PWM data for left and right motors
       case HB:
         Serialread();
         Leftmode = data;
         Serialread();
         LeftPWM = data;
         Serialread();
         Rightmode = data;
         Serialread();
         RightPWM = data;

         /*
         Serial.print("Leftmode = ");
         Serial.print(Leftmode);
         Serial.print(" at ");
         Serial.println(LeftPWM);
         Serial.print("Rightmode = ");
         Serial.print(Rightmode);
         Serial.print(" at ");
         Serial.println(RightPWM);
         */

         break;

       // invalid command
       default:
         Serial.flush();
    }
  }
}

//
// Read serial port until data has been received
//
void Serialread() {
  do {
    data = Serial.read();
  } while (data<0);
}

void I2Cmode() {
  // Your code goes here
}

void logBatteryVoltage() {
  int units = analogRead(Battery);
  float volts = units/65.0;
  Serial.print("Battery at ");
  Serial.print(volts, DEC);
  Serial.print("v (");
  Serial.print(units);
  Serial.println(" units)");
}

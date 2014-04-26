// MODE OF COMMUNICATION
#define Cmode                1     // Sets communication mode: 0=RC    1=Serial    2=I2C
#define Brate           115200     // Baud rate for serial communications
#define reportInterval     100     // How often to report state

// CONTROL CODES
#define FL               17996     // Flush buffer
#define AN               16718     // Get analog input vlaues
#define SV               21334     // Get servo positions
#define HB               18498     // Set H-bridge values
#define BT               16980     // Check battery level
#define ST               21332     // Stop

// SERIAL
#define COMMANDTIMEOUT    250     // Stop after this long if no commands received

// MOTION
#define FORWARD             2
#define BRAKE               1
#define REVERSE             0

// RC MODE OPTIONS
#define RCMIX                1     // Set to 1 if L/R and F/R signals from RC need to be mixed
#define RCLEFTCENTER      1500     // when RC inputs are centered then input should be 1.5mS
#define RCRIGHTCENTER     1500     // when RC inputs are centered then input should be 1.5mS
#define RCDEADBAND          35     // inputs do not have to be perfectly centered to stop motors
#define RCSCALE             12     // scale factor for RC signal to PWM

// BATTERY CHARGER SETTINGS
#define UNITSPERVOLT        67     // Units per volt
#define BATVOLT            487     // This is the nominal battery voltage reading. Peak charge can only occur above this voltage.
#define LOWVOLT            400     // This is the voltage at which the speed controller goes into recharge mode.
#define CHARGETIMEOUT   300000     // If the battery voltage does not change in this number of milliseconds then stop charging.

// H BRIDGE SETTINGS
#define Leftmaxamps        800     // set overload current for left motor
#define Rightmaxamps       800     // set overload current for right motor
#define overloadtime       100     // time in mS before motor is re-enabled after overload occurs

// SERVO SETTINGS
#define DServo0           1500     // default position for servo0 on "power up" - 1500uS is center position on most servos
#define DServo1           1500     // default position for servo1 on "power up" - 1500uS is center position on most servos
#define DServo2           1500     // default position for servo2 on "power up" - 1500uS is center position on most servos
#define DServo3           1500     // default position for servo3 on "power up" - 1500uS is center position on most servos
#define DServo4           1500     // default position for servo4 on "power up" - 1500uS is center position on most servos
#define DServo5           1500     // default position for servo5 on "power up" - 1500uS is center position on most servos
#define DServo6           1500     // default position for servo6 on "power up" - 1500uS is center position on most servos

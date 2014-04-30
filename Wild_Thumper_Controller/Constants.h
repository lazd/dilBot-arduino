// COMMUNICATION
#define Brate           115200     // Baud rate for serial communications
#define MODE_RC              0
#define MODE_SERIAL          1

// CONTROL CODES
#define FL               17996     // Flush buffer
#define HB               18498     // Set H-bridge values
#define ST               21332     // Stop
#define MO               19791     // Change control mode
#define CH               17224     // Enter charge mode

// SENSORS
#define PING_LEFT           D0
#define PING_RIGHT          D1
#define RC_LEFT             D3
#define RC_RIGHT            D2

// SERIAL
#define COMMANDTIMEOUT     250     // Stop after this long if no commands received

// MOTION
#define FORWARD              2
#define BRAKE                1
#define REVERSE              0

// RC MODE OPTIONS
#define RC_MIX               1     // Set to 1 if L/R and F/R signals from RC need to be mixed
#define RC_CENTER         1800     // when RC inputs are centered then input should be 1.5mS
#define RC_DEADBAND         35     // inputs do not have to be perfectly centered to stop motors
#define RC_SCALE            12     // scale factor for RC signal to PWM

// BATTERY CHARGER SETTINGS
#define UNITSPERVOLT        67     // Units per volt
#define BATVOLT            487     // This is the nominal battery voltage reading. Peak charge can only occur above this voltage.
#define LOWVOLT            400     // This is the voltage at which the speed controller goes into recharge mode.
#define CHARGETIMEOUT   300000     // If the battery voltage does not change in this number of milliseconds then stop charging.

// H BRIDGE SETTINGS
#define Leftmaxamps        800     // set overload current for left motor
#define Rightmaxamps       800     // set overload current for right motor
#define overloadtime       100     // time in mS before motor is re-enabled after overload occurs

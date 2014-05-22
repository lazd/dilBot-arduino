// COMMUNICATION
#define BAUDRATE        115200     // Baud rate for serial communications
#define MODE_RC              0
#define MODE_SERIAL          1

// CONTROL CODES
#define FL               17996     // Flush buffer
#define HB               18498     // Set H-bridge values
#define ST               21332     // Stop
#define MO               19791     // Change control mode
#define CH               17224     // Enter charge mode

// SENSORS
#define PING_LEFT           D2
#define PING_CENTER         D4
#define PING_RIGHT          D7
#define RC_LEFT             D_A2
#define RC_RIGHT            D_A1

// SERIAL
#define COMMANDTIMEOUT     250     // Stop after this long if no commands received

// MOTION
#define FORWARD              2
#define BRAKE                1
#define REVERSE              0

// RC MODE OPTIONS
#define RC_MIX               1     // Set to 1 if L/R and F/R signals from RC need to be mixed
#define RC_CENTER         1500     // when RC inputs are centered then input should be 1.5mS
#define RC_DEADBAND         75     // inputs do not have to be perfectly centered to stop motors
#define RC_SCALE            12     // scale factor for RC signal to PWM

// BATTERY SETTINGS
#define UNITSPERVOLT        67     // Units per volt
#define LOWVOLT            400     // This is the voltage at which the speed controller shuts down

// H BRIDGE SETTINGS
#define LEFTMAXAMPS        800     // set overload current for left motor
#define RIGHTMAXAMPS       800     // set overload current for right motor
#define OVERLOADTIME       100     // time in mS before motor is re-enabled after overload occurs

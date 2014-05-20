#define PIN_MOTOR_LEFT_A    3  // Left  motor H bridge, input A
#define PIN_MOTOR_LEFT_B    11 // Left  motor H bridge, input B
#define PIN_MOTOR_RIGHT_A   5  // Right motor H bridge, input A
#define PIN_MOTOR_RIGHT_B   6  // Right motor H bridge, input B

// Digital pins 0 and 1 interfere with USB, don't use them
// Digital pins 2-12 are safe
#define D2                 2   // Digital output 00
#define D4                 4   // Digital output 01
#define D7                 7   // Digital output 02
#define D8                 8   // Digital output 03
#define D9                 9   // Digital output 04
#define D10                10  // Digital output 05
#define D12                12  // Digital output 06

#define D18                18  // D18 for enabling pullup resistors
#define D19                19  // D19 for enabling pullup resistors

#define PIN_BATTERY                 0  // Analog input 00
#define PIN_MOTOR_RIGHT_CURRENT     6  // Analog input 06
#define PIN_MOTOR_LEFT_CURRENT      7  // Analog input 07
#define PIN_CHARGER                 13 // Low=ON High=OFF

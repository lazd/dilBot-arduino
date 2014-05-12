
#define LmotorA             3  // Left  motor H bridge, input A
#define LmotorB            11  // Left  motor H bridge, input B
#define RmotorA             5  // Right motor H bridge, input A
#define RmotorB             6  // Right motor H bridge, input B

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

#define Battery             0  // Analog input 00
#define RmotorC             6  // Analog input 06
#define LmotorC             7  // Analog input 07
#define Charger            13  // Low=ON High=OFF

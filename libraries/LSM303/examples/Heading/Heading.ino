#include <Wire.h>
#include <LSM303.h>

LSM303 compass;

void setup() {
  Serial.begin(9600);

  // Enable pullups to put A4 and A5 into I2C mode
  digitalWrite(18, 1);
  digitalWrite(19, 1);

  Wire.begin();
  compass.init();
  compass.enableDefault();

  // Compass calibration
  /*
  Calibration values; the default values of +/-32767 for each axis
  lead to an assumed magnetometer bias of 0. Use the Calibrate example
  program to determine appropriate values for your particular unit.
  */
  // Compass calibration - In front near motors
  // compass.m_min = (LSM303::vector<int16_t>){+3436, -7299, +1491};
  // compass.m_max = (LSM303::vector<int16_t>){+9920, -744, +7532};

  /* Calibration - Above driver, try 1
  0: -6
  90: +41
  180: +10
  270: -9
  */
  // compass.m_min = (LSM303::vector<int16_t>){-7939, -4124, +585};
  // compass.m_max = (LSM303::vector<int16_t>){-880, +1423, +6156};

  /* Calibration - Above driver, try 2
  0: 0
  90: +30
  180: +13
  270: -8
  */
  // compass.m_min = (LSM303::vector<int16_t>){-7885, -3803, +483};
  // compass.m_max = (LSM303::vector<int16_t>){-1750, +1139, +6101};

  /* Calibration - Above driver, try 3
  0: -2
  90: +28
  180: +13
  270: +5
  */
  // compass.m_min = (LSM303::vector<int16_t>){-7864, -4085, +660};
  // compass.m_max = (LSM303::vector<int16_t>){-1828, +1448, +6044};

  /* Calibration - Above driver, try 4
  0: -10
  90: +26
  180: +11
  270: -15
  */
  // compass.m_min = (LSM303::vector<int16_t>){-7441, -3583, +1041};
  // compass.m_max = (LSM303::vector<int16_t>){-1796, +1084, +5934};

  /* Calibration - Above driver, try 5
  0: -10
  90: +17
  180: +12
  270: +1
  */
  compass.m_min = (LSM303::vector<int16_t>){-7865, -4027, +803};
  compass.m_max = (LSM303::vector<int16_t>){-1676, +1536, +6193};
}

void loop() {
  compass.read();
  
  /*
  When given no arguments, the heading() function returns the angular
  difference in the horizontal plane between a default vector and
  north, in degrees.
  
  The default vector is chosen by the library to point along the
  surface of the PCB, in the direction of the top of the text on the
  silkscreen. This is the +X axis on the Pololu LSM303D carrier and
  the -Y axis on the Pololu LSM303DLHC, LSM303DLM, and LSM303DLH
  carriers.
  
  To use a different vector as a reference, use the version of heading()
  that takes a vector argument; for example, use
  
    compass.heading((LSM303::vector<int>){0, 0, 1});
  
  to use the +Z axis as a reference.
  */
  float heading = compass.heading();
  
  Serial.println(heading);
  delay(100);
}
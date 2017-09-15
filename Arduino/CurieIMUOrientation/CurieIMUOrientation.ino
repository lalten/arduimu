// Send IMU values over serial
// 2017-04 Laurenz Altenmueller <bsh@laure.nz>
// if serial output doesn't work, try to "stty -F /dev/serial/by-id/usb-Intel_ARDUINO_101_AE6642SQ541001F-if00 raw"
// REP-145 configuration is North-West-Up!


#include <CurieIMU.h>

unsigned long microsPerReading, microsNext;
float accelScale, gyroScale;


void setup() {

  Serial.begin(115200);
  while (!Serial);

  // start the IMU and filter
  CurieIMU.begin();
  CurieIMU.setGyroRate(100);
  CurieIMU.setAccelerometerRate(100);

  // Set the accelerometer range to 2G
  CurieIMU.setAccelerometerRange(2);
  // Set the gyroscope range to 250 degrees/second
  CurieIMU.setGyroRange(250);

  // initialize variables to pace updates to correct rate
  microsPerReading = 1000000 / 100;
  microsNext = micros();

  // do a autocalibration on startup
  delay(50); // wait so we don't get any movement from pressing the reset button
  CurieIMU.autoCalibrateGyroOffset();
  CurieIMU.autoCalibrateAccelerometerOffset(X_AXIS, 0);
  CurieIMU.autoCalibrateAccelerometerOffset(Y_AXIS, 0);
  CurieIMU.autoCalibrateAccelerometerOffset(Z_AXIS, 1);

}

void loop() {

    int aix, aiy, aiz;
    int gix, giy, giz;
    float ax, ay, az;
    float gx, gy, gz;

    // read raw data from CurieIMU
    CurieIMU.readMotionSensor(aix, aiy, aiz, gix, giy, giz);

    // convert from raw data to m/s^2 and degrees/second units
    ax = convertRawAcceleration(aix);
    ay = convertRawAcceleration(aiy);
    az = convertRawAcceleration(aiz);
    gx = convertRawGyro(gix);
    gy = convertRawGyro(giy);
    gz = convertRawGyro(giz);

    // assemble value string
    char buf[255];
    sprintf(buf, "%f,%f,%f,%f,%f,%f", ax,ay,az, gx,gy,gz);
    
    // put the checksum at the end of the string
    char chk = 0;
    for(unsigned int i=0; i<strlen(buf); i++)
    {
      chk ^= buf[i];
    }
    sprintf(buf, "%s,%d", buf, chk);

    // send it out
    Serial.println(buf);
}

float convertRawAcceleration(int aRaw) {
  // since we are using 2G range
  // -2g maps to a raw value of -32768
  // +2g maps to a raw value of 32767

  // 1 g = 9.80665 m/s^2
  float a = (aRaw * 2.0 * 9.80665) / 32768.0;
  return a;
}

float convertRawGyro(int gRaw) {
  // since we are using 250 degrees/seconds range
  // -250 maps to a raw value of -32768
  // +250 maps to a raw value of 32767

  // 1 deg/s = 0.0174533 rad/s
  float g = (gRaw * 250.0 * 0.0174533) / 32768.0;
  return g;
}


#include <ros.h>
#include <ArduinoHardware.h>

#include <sensor_msgs/Imu.h>

#include <CurieIMU.h>
#include <MadgwickAHRS.h>


ros::NodeHandle nh;
sensor_msgs::Imu imu_msg;
ros::Publisher pub("imu", &imu_msg);


Madgwick filter;
unsigned long microsPerReading, microsNext;
float accelScale, gyroScale;


void setup() {

  while (!Serial);
  Serial.begin(1000000);
  nh.getHardware()->setBaud(1000000);

  nh.initNode();
  nh.advertise(pub);

  //wait until you are actually connected
  while (!nh.connected() ){
      nh.spinOnce();
  }
  
  int rate = 40; // the max seems to be about 44 or 45 on arduino 101
  nh.getParam("~rate", &rate);
  
  imu_msg.header.frame_id = "arduimu";
  
  // start the IMU and filter
  CurieIMU.begin();
  CurieIMU.setGyroRate(25);
  CurieIMU.setAccelerometerRate(25);
  filter.begin(25);

  // Set the accelerometer range to 2G
  CurieIMU.setAccelerometerRange(2);
  // Set the gyroscope range to 250 degrees/second
  CurieIMU.setGyroRange(250);

  // initialize variables to pace updates to correct rate
  microsPerReading = 1000000 / rate;
  microsNext = micros();

  char buf[100];
  sprintf(buf, "Starting IMU publisher with rate %d / period %lu us\n", rate, microsPerReading);
  nh.loginfo(buf);
}

void loop() {

  // Re-initialize on connection loss
//  if(!nh.connected()) setup();
  
  int aix, aiy, aiz;
  int gix, giy, giz;
  float ax, ay, az;
  float gx, gy, gz;
  float roll, pitch, yaw;

  // read raw data from CurieIMU
  CurieIMU.readMotionSensor(aix, aiy, aiz, gix, giy, giz);

  // convert from raw data to gravity and degrees/second units
  ax = convertRawAcceleration(aix);
  ay = convertRawAcceleration(aiy);
  az = convertRawAcceleration(aiz);
  gx = convertRawGyro(gix);
  gy = convertRawGyro(giy);
  gz = convertRawGyro(giz);

  // update the filter, which computes orientation
  filter.updateIMU(gx, gy, gz, ax, ay, az);

  // check if it's time to read data and update the filter
  if (micros() >= microsNext) {
    microsNext = microsNext + microsPerReading;
    
    roll = filter.getRoll();
    pitch = filter.getPitch();
    yaw = filter.getYaw();

    // Calculate quaternion 
    float cosYaw = cos(yaw/2.0);
    float sinYaw = sin(yaw/2.0);
    float cosPitch = cos(pitch/2.0);
    float sinPitch = sin(pitch/2.0);
    float cosRoll = cos(roll/2.0);
    float sinRoll = sin(roll/2.0);
    float qx = sinRoll * cosPitch * cosYaw - cosRoll * sinPitch * sinYaw;
    float qy = cosRoll * sinPitch * cosYaw + sinRoll * cosPitch * sinYaw;
    float qz = cosRoll * cosPitch * sinYaw - sinRoll * sinPitch * cosYaw;
    float qw = cosRoll * cosPitch * cosYaw + sinRoll * sinPitch * sinYaw;

    // update ros message
    imu_msg.header.seq++;
    imu_msg.header.stamp = nh.now(); // slow?
    imu_msg.orientation.x = qx;
    imu_msg.orientation.y = qy;
    imu_msg.orientation.z = qz;
    imu_msg.orientation.w = qw;
    imu_msg.angular_velocity.x = gx;
    imu_msg.angular_velocity.y = gy;
    imu_msg.angular_velocity.z = gz;
    imu_msg.linear_acceleration.x = ax;
    imu_msg.linear_acceleration.y = ay;
    imu_msg.linear_acceleration.z = az;

    // send out ros message
    pub.publish(&imu_msg);
    nh.spinOnce();

    // print the yaw, pitch and roll
//    Serial.print("Orientation: ");
//    Serial.print(yaw);
//    Serial.print(" ");
//    Serial.print(pitch);
//    Serial.print(" ");
//    Serial.println(roll);

    }
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


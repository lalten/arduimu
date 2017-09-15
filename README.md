# arduimu
Use an Arduino 101's Intel Curie as IMU with ROS

## Requirements
 * [ROS](http://www.ros.org/)
 * wjwwood's [Serial Communication Library](https://github.com/wjwwood/serial)
 * Arduino [MadgwickAHRS](https://github.com/arduino-libraries/MadgwickAHRS) library
 * Arduino [CurieIMU](https://github.com/01org/corelibs-arduino101/tree/master/libraries/CurieIMU) library

## Files
  * `Arduino/CurieIMUOrientation/` is the code that you want to flash on your Arduino 101.
  * `ROS/arduimu` ROS catkin package. Contains the code that reads the Arduino's serial data and publishes it as `sensor_msgs::Imu` messages
  * `Arduino/CurieIMUOrientationROS` is an alternative version that publishes a `sensor_msgs::Imu` rostopic via [rosserial](http://wiki.ros.org/rosserial)

## Run
 * Flash your Arduino 101 with the CurieIMUOrientation code
 * `roslaunch arduimu arduimu.launch`
 * If the arduino has issues and hangs, you can use the `ROS/arduimu/scripts/reset_arduino.py` script, which opens the Arduino at a magic 1200 baud, which resets it. Alternatively, `usb_modeswitch -R` may work.

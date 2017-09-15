// Receives IMU data from an Arduino 101 via serial and publishes ROS IMU messages
// 2017-04 Laurenz Altenmueller <bsh@laure.nz>

#include <ros/ros.h>
#include <sensor_msgs/Imu.h>

#include <serial/serial.h>

#include <string>
#include <iostream>
#include <thread>
#include <cstdio>
#include <limits>

bool reset_arduino(serial::Serial &s)
{
    int baud = s.getBaudrate();
    s.close();
    
    ROS_INFO_STREAM("Resetting arduino via 1200 baud connection");
    s.setBaudrate(1200);
    s.open();
    s.close();
    
    ROS_INFO_STREAM("Re-opening arduino via "<<baud<<" baud connection");
    s.setBaudrate(baud);
    ros::Time timeout = ros::Time::now() + ros::Duration(6); // timeout 6seconds (restart should take around 5)
    while(ros::Time::now() < timeout)
    {
        try
        {
            s.open();
            std::string buf;
            size_t r = s.readline(buf);
            
            if(r > 0)
            {
                ROS_INFO_STREAM("Arduino connection re-established");
                return true;
            }
            else
            {
                ROS_INFO_STREAM("got \""<<buf<<"\"");
            }
            s.close();
        }
        catch (serial::SerialException ex)
        {
            ROS_INFO_STREAM(timeout - ros::Time::now() << "s: " << ex.what());
            std::this_thread::sleep_for(std::chrono::milliseconds(500));
        }
    }
    
    return false;
}

int main(int argc, char **argv)
{
    // Set up ROS.
    ros::init(argc, argv, "imu_serial");
    ros::NodeHandle nh("~");
    
    std::string port;
    nh.param<std::string>("port", port, std::string("/dev/ttyACM0"));
    int baud;
    nh.param<int>("baud", baud, int(9600));

    // Create a publisher and name the topic.
    ros::Publisher pub = nh.advertise<sensor_msgs::Imu>("imu", 10);
    
    std::string frame;
    nh.param<std::string>("frame", frame, std::string("arduimu"));
    sensor_msgs::Imu imu_msg;
    imu_msg.header.frame_id = frame;

    imu_msg.orientation_covariance.assign(std::numeric_limits<double>::infinity());
    imu_msg.angular_velocity_covariance =    {1e-3, 0, 0,    0, 1e-3, 0,    0, 0, 1e-3};
    imu_msg.linear_acceleration_covariance = {1e-1, 0, 0,    0, 1e-1, 0,    0, 0, 1e-1};
    
    try
    {
    
        ROS_INFO_STREAM("Connecting to port "<< port);
        serial::Serial my_serial(port, baud, serial::Timeout::simpleTimeout(1000));
    
        // wait for serial to be available        
        if(!my_serial.isOpen())
        {
            while(!my_serial.isOpen())
            {
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
            }
            my_serial.readline(); // throw away first line
        }
        

        // Main loop.
        while (nh.ok())
        {
            std::string buf;
            size_t s;
            
            // read one line
            try {
                s = my_serial.readline(buf);
            } catch (serial::SerialException ex) {
                if(ros::ok()) ROS_ERROR_STREAM_THROTTLE(5, ex.what());
                else std::cerr << ex.what() << std::endl;
                break;
            }
            
            ROS_DEBUG_STREAM("Received \"" << buf << "\" (" << (int)s <<")");
            
            if(s <= 0)
            {
                ROS_WARN_STREAM_THROTTLE(5, "IMU Serial has no data, resetting");
                return 1; // just kill the node and have roslaunch restart it for now
//                 if(! reset_arduino(my_serial))
//                 {
//                     ROS_ERROR_STREAM("Could not recover arduino connection.");
//                     break;
//                 }
                    
            }
            
            if(s > 0)
            {
                
                float ax,ay,az,gx,gy,gz,q0,q1,q2,q3;
                int chk, r, chk_calc=0;
                r = sscanf(buf.c_str(), "%f,%f,%f,%f,%f,%f,%d\n", &ax, &ay, &az, &gx, &gy, &gz, &chk);
                
                if(r==7)
                {
                    int num_chars_chk = (chk < 10 ? 1 : (chk < 100 ? 2 : 3));
                    int str_len = buf.rfind(",");
                    for(int i=0; i<str_len; i++)
                    {
                        chk_calc ^= buf.c_str()[i];
                    }
                    ROS_DEBUG_STREAM("Checksum of \"" << buf.substr(0, str_len) << "\" is " << std::to_string(chk_calc));
                }
                if(r != 7 || chk_calc != chk)
                {
                    ROS_WARN_STREAM("Received garbled IMU serial data");
                    continue;
                }
                
                ROS_INFO_STREAM_ONCE("Receiving IMU data");

                // Publish the message.
                imu_msg.header.stamp = ros::Time::now();
                imu_msg.linear_acceleration.x = ax;
                imu_msg.linear_acceleration.y = ay;
                imu_msg.linear_acceleration.z = az;
                imu_msg.angular_velocity.x = gx;
                imu_msg.angular_velocity.y = gy;
                imu_msg.angular_velocity.z = gz;
                
                pub.publish(imu_msg);
            }

            ros::spinOnce();
        }
        
    } catch (serial::IOException ex) {
        if(ros::ok()) ROS_ERROR_STREAM(ex.what());
        else std::cerr << ex.what() << std::endl;
    }
    
    return 0;
}

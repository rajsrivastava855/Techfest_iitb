#include "ros/ros.h"
#include "std_msgs/String.h"
#include <serial/serial.h> // Include the serial library for communication

int main(int argc, char **argv)
{
  
    ros::init(argc, argv, "depth_publisher");
    ros::NodeHandle nh;

    ros::Publisher sensor_pub = nh.advertise<std_msgs::String>("depth_data", 10);

 
    ros::Rate loop_rate(10);

    serial::Serial ser;
    try
    {
        ser.setPort("/dev/tty*");  // Change according to your device port
        ser.setBaudrate(9600);      // Match baud rate with your microcontroller
        serial::Timeout timeout = serial::Timeout::simpleTimeout(1000);
        ser.setTimeout(timeout);
        ser.open();
    }
    catch (serial::IOException& e)
    {
        ROS_ERROR_STREAM("Unable to open port.");
        return -1;
    }

   
    if (ser.isOpen())
    {
        ROS_INFO_STREAM("Serial port initialized.");
    }
    else
    {
        ROS_ERROR_STREAM("Failed to open serial port.");
        return -1;
    }

    while (ros::ok())
    {
        if (ser.available())
        {
          
            std_msgs::String msg;
            msg.data = ser.readline(); 

           
            sensor_pub.publish(msg);
            ROS_INFO_STREAM("Data published: " << msg.data);
        }

        ros::spinOnce();
        loop_rate.sleep();
    }
}

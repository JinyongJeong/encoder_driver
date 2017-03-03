#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <ros/ros.h>
#include <serial/serial.h>
#include <std_msgs/String.h>
#include <std_msgs/Int64MultiArray.h>
#include <poll.h>
#include <termios.h>

using namespace std;

serial::Serial ser;

int 
_serial_set_canonical (int fd)
{
    struct termios tio;

    if ( tcgetattr (fd, &tio) ) {
        return -1;
    }

    tio.c_lflag |= ICANON;

    if ( tcsetattr (fd, TCSANOW, &tio) ) {
        return -1;
    }

    return 0;
}

int main (int argc, char** argv){
    ros::init(argc, argv, "encoder_driver_node");
    ros::NodeHandle nh;

    ros::Publisher encoder_pub = nh.advertise<std_msgs::Int64MultiArray>("encoder_count", 10);

    std::string device;
    int baudrate;

    //char ttydev[] = "/dev/ttyUSB-encoder";
    //int baudrate = 115200; 
    
    nh.param("device", device, std::string("/dev/ttyUSB-encoder"));
    nh.param("baudrate", baudrate, 115200);
    
    try
    {
        ser.setPort(device);
        ser.setBaudrate(baudrate);
        serial::Timeout to = serial::Timeout::simpleTimeout(10000);
        ser.setTimeout(to);
        ser.open();
    }
    catch (serial::IOException& e)
    {
        ROS_ERROR_STREAM("Unable to open port ");
        return -1;
    }

    if(ser.isOpen()){
        ROS_INFO_STREAM("Serial Port initialized");
    }else{
        return -1;
    }

    ros::Rate loop_rate(100);
    
    string str_buf;

    long left_count;
    long right_count;

    std_msgs::Int64MultiArray count;
    
    while(ros::ok()){

    ros::spinOnce();

        if(ser.available()){        
            ROS_INFO_STREAM("Reading from serial port");

            str_buf = ser.read(ser.available());
            char comma;

            sscanf(str_buf.c_str()," %011ld%c%011ld",&left_count, &comma,&right_count);
            printf("LEFT: %ld \t RIGHT: %ld\n", left_count, right_count);
                
            count.data.clear();
            count.data.push_back(left_count);
            count.data.push_back(right_count);
            ROS_INFO_STREAM("Read: " << str_buf.c_str());
            encoder_pub.publish(count);

        }
        loop_rate.sleep();
    }
}

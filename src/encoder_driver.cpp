#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Int64MultiArray.h>


#include <encoder_driver/encoder.h>
#include <basic_serial.h>

using namespace std;

int 
main (int argc, char *argv[])
{

    ros::init(argc, argv, "encoder_driver_node");
    ros::NodeHandle nh;
    ros::Publisher encoder_pub = nh.advertise<encoder_driver::encoder>("encoder_count", 10);

    std::string ttydev;
    int brate = 115200;
    ros::param::param<std::string>("~device", ttydev, "/dev/ttyUSB-encoder");
    ros::param::param<int>("~baudrate", brate, 115200);
    int fd = serial_open (ttydev.c_str(), brate, 0);

    char channel[] = "encoder_counter";

    serial_set_canonical (fd);

    char buf [50];
    int len;
    memset (&buf, '\0', 50);
    
    struct pollfd poll_events;

    int poll_state;

    long left_count;
    long right_count;

    std_msgs::Int64MultiArray count;

    encoder_driver::encoder encoder_data;
    
    poll_events.fd = fd;
    poll_events.events = POLLIN|POLLERR;
    poll_events.revents = 0;

    while(ros::ok()) {
        ros::spinOnce();
        poll_state = poll((struct pollfd*)&poll_events, 1, 1000);

        if(0 < poll_state) {
            if (poll_events.revents & POLLIN) {
                ROS_INFO_STREAM("Reading from serial port");
                memset (&buf, '\0', 50);
                
                len = read (fd, buf, 50);

                char comma;

                sscanf(buf," %011ld%c%011ld", &left_count, &comma, &right_count);

                encoder_data.header.stamp  = ros::Time::now();
                encoder_data.header.frame_id = "encoder";
                
                encoder_data.left_count = left_count;
                encoder_data.right_count = right_count;

                encoder_pub.publish(encoder_data);


            }

            if (poll_events.revents & POLLERR) {
                printf( "receiver error!" );
                break;
            }
        }
    }

    close(fd);

    return 0;
}



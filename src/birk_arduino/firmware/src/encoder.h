#ifndef _ENCODER_H_
#define _ENCODER_H_

#include <Arduino.h>
#include <ros.h>
#include <std_msgs/Int32.h>

#define ENCODER_LEFT_PIN 3
#define ENCODER_RIGHT_PIN 2

class encoder
{
    private:
        std_msgs::Int32 setps_right_msg;
        ros::Publisher setps_right_publisher;
        std_msgs::Int32 setps_left_msg;
        ros::Publisher setps_left_publisher;
        ros::NodeHandle* node_handle;



    public:
        encoder();
        void set_node_handle(ros::NodeHandle &nh);
        void process();

};

#endif

#ifndef _IR_DETECTOR_H_
#define _IR_DETECTOR_H_

#include <Arduino.h>
#include <ros.h>
#include <std_msgs/String.h>


class ir_detector
{
    private:
        std_msgs::String ir_state_msg;
        ros::Publisher ir_state_publisher;
        ros::NodeHandle* node_handle;

    public:
        ir_detector()
            :   ir_state_publisher("infrared_sensor", &ir_state_msg)
        {
        };

        void set_node_handle(ros::NodeHandle &nh)
        {
            node_handle = &nh;
            nh.advertise(ir_state_publisher);
        };

        void process() {
            ir_state_msg.data = "nothing";
            ir_state_publisher.publish( &ir_state_msg );
        };
};

#endif

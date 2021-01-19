#include "encoder.h"


static uint32_t left_encoder_count;
static uint32_t right_encoder_count;

static void left_interrupt() {
    left_encoder_count++;
}

static void right_interrupt() {
    right_encoder_count++;
}

encoder::encoder()
    :   setps_right_publisher("encoder/setps_right", &setps_right_msg),
        setps_left_publisher("encoder/setps_left", &setps_left_msg)
{
    left_encoder_count = 0;
    right_encoder_count = 0;
    attachInterrupt(
        digitalPinToInterrupt(ENCODER_LEFT_PIN), 
        left_interrupt, 
        RISING);
    attachInterrupt(
        digitalPinToInterrupt(ENCODER_RIGHT_PIN), 
        right_interrupt, 
        RISING);
};

void encoder::set_node_handle(ros::NodeHandle &nh)
{
    node_handle = &nh;
    nh.advertise(setps_right_publisher);
    nh.advertise(setps_left_publisher);
};

void encoder::process() {
    setps_right_msg.data = right_encoder_count;
    setps_left_msg.data = left_encoder_count;
    setps_right_publisher.publish( &setps_right_msg );
    setps_left_publisher.publish( &setps_left_msg );
};

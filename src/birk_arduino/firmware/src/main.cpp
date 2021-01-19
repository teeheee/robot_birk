#include <ros.h>
#include "battery_managment.h"
#include "debug_leds.h"
#include "ir_detector.h"
#include "encoder.h"

ros::NodeHandle nh;
debug_leds leds;
battery_managment bat_manage(leds);
ir_detector ir;
encoder enc;

void setup()
{
  nh.initNode();
  bat_manage.set_node_handle(nh);
  ir.set_node_handle(nh);
  enc.set_node_handle(nh);
}

void loop()
{
  bat_manage.process();
  leds.process();
  ir.process();
  enc.process();
  nh.spinOnce();
  delay(20);
}

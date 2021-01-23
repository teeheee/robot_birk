#ifndef _BATTERY_MANAGMENT_H_
#define _BATTERY_MANAGMENT_H_

#include <Arduino.h>
#include <ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Float32MultiArray.h>

#include "debug_leds.h"

#define CELL_EMPTY_VOLTAGE 3.4
#define CELL_HALF_VOLTAGE 3.6
#define CELL_FULL_VOLTAGE 4.1
#define CELL_MAX_DIFFERENCE 0.2

typedef enum  {
        BAT_CHARGING,
        BAT_EMTPY,
        BAT_FULL,
        BAT_UNBALANCED,
        BAT_NC,
        BAT_ALMOST_EMPTY,
        BAT_STATE_LENGTH
} battery_state_t;

class battery_managment
{
    private:
        std_msgs::Float32MultiArray voltages_msg;
        std_msgs::String charge_state_msg;
        ros::Publisher voltage_publisher;
        ros::Publisher charge_state_publisher;
        ros::NodeHandle* node_handle;

        const char* charge_state_messages[BAT_STATE_LENGTH] = {
            "BAT_CHARGING",
            "BAT_EMTPY",
            "BAT_FULL",
            "BAT_UNBALANCED",
            "BAT_NC",
            "BAT_ALMOST_EMPTY"
        };

        debug_leds* leds;

        float voltages[3];

        battery_state_t bat_state;

        const int adc_cell_pins[3] = {
            A2,A1,A0
        };

        /**
         * @brief 1023 = 3,3 V
         * 1: (98+180)/98 = 2.837
         * 2: (100+467)/100 = 5.67
         * 3: (99+751)/99 = 8.586
         *  3.3*2.837/1023
         *  3.3*5.67/1023
         *  3.3*8.586/1023
         */
        const double adc_cell_scale[3] = {
            3.3*2.837/1023,
            3.3*5.67/1023,
            3.3*8.586/1023
        };

    public:
        battery_managment(debug_leds &aleds)
            :   voltage_publisher("battery/voltage", &voltages_msg),
                charge_state_publisher("battery/charge_state", &charge_state_msg)
        {
            voltages_msg.data = voltages;
            voltages_msg.data_length = 3;
            leds = &aleds;
        };

        void set_node_handle(ros::NodeHandle &nh)
        {
            node_handle = &nh;
            nh.advertise(voltage_publisher);
            nh.advertise(charge_state_publisher);
        };

        void process() {
            measure_voltage();
            calculate_battery_state();
            voltage_publisher.publish( &voltages_msg );
            charge_state_msg.data = charge_state_messages[bat_state];
            charge_state_publisher.publish( &charge_state_msg );
        };

        void measure_voltage() {
            double voltage_sum = 0;
            for(int pin = 0; pin < 3; pin++)
            {
                double voltage = analogRead(adc_cell_pins[pin]);
                voltage *= adc_cell_scale[pin];
                voltage -= voltage_sum;
                voltages[pin] = voltage;
                voltage_sum += voltage;
            }
        };

        void calculate_battery_state()
        {
            for(int pin = 0; pin < 3; pin++)
            {
                if(abs(voltages[pin]-voltages[(pin+1)%3]) > CELL_MAX_DIFFERENCE)
                {
                    bat_state = BAT_UNBALANCED;
                    leds->set_led_state(LED_RED, LED_BLINK_FAST);
                    leds->set_led_state(LED_YELLOW, LED_OFF);
                    leds->set_led_state(LED_GREEN, LED_OFF);
                    return;
                }
            }
            for(int pin = 0; pin < 3; pin++)
            {
                if(voltages[pin] < CELL_EMPTY_VOLTAGE)
                {
                    bat_state = BAT_EMTPY;
                    leds->set_led_state(LED_RED, LED_BLINK_SLOW);
                    leds->set_led_state(LED_YELLOW, LED_OFF);
                    leds->set_led_state(LED_GREEN, LED_OFF);
                    return;
                }
            }
            for(int pin = 0; pin < 3; pin++)
            {
                if(voltages[pin] < CELL_HALF_VOLTAGE)
                {
                    bat_state = BAT_ALMOST_EMPTY;
                    leds->set_led_state(LED_YELLOW, LED_BLINK_SLOW);
                    leds->set_led_state(LED_RED, LED_OFF);
                    leds->set_led_state(LED_GREEN, LED_OFF);
                    return;
                }
            }
            bat_state = BAT_FULL;
            leds->set_led_state(LED_YELLOW, LED_OFF);
            leds->set_led_state(LED_RED, LED_OFF);
            leds->set_led_state(LED_GREEN, LED_ON);
        };
};

#endif

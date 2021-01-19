#ifndef _DEBUG_LES_H_
#define _DEBUG_LES_H_

#include <Arduino.h>

#define SLOW_BLINK_MS 1000
#define FAST_BLINK_MS 200

typedef enum  {
        LED_OFF,
        LED_ON,
        LED_BLINK_SLOW,
        LED_BLINK_FAST,
        LED_FLASH
} led_state_t;

typedef enum  {
        LED_RED,
        LED_YELLOW,
        LED_GREEN,
        NUM_OF_LEDS
} led_type_t;

class debug_leds{
    private:
        led_state_t led_states[NUM_OF_LEDS];
        uint8_t led_pin_state[NUM_OF_LEDS];
        uint32_t time_stamps[NUM_OF_LEDS];

        const int led_pins[NUM_OF_LEDS] = {8,6,4};

    public:
        debug_leds(){
            for(int led_id = 0; led_id < NUM_OF_LEDS; led_id++)
            {
                led_states[led_id] = LED_OFF;
                time_stamps[led_id] = 0;
                led_pin_state[led_id] = 0;
                pinMode(led_pins[led_id], OUTPUT);
            }
        };

        void set_led_state(led_type_t led_type, led_state_t led_state)
        {
            led_states[led_type] = led_state;
            if(led_state != LED_OFF)
            {
                time_stamps[led_type] = millis();
                led_pin_state[led_type] = 1;
                digitalWrite(led_pins[led_type], led_pin_state[led_type]);
            }
        };

        void process()
        {
            for(int led_id = 0; led_id < NUM_OF_LEDS; led_id++)
            {
                switch(led_states[led_id])
                {
                    case LED_OFF:
                        led_pin_state[led_id] = 0;
                        break;
                    case LED_ON:
                        led_pin_state[led_id] = 1;
                        break;
                    case LED_BLINK_SLOW:
                        if(millis() - time_stamps[led_id] > SLOW_BLINK_MS)
                        {
                            led_pin_state[led_id] ^= 1;
                            time_stamps[led_id] = millis();
                        }
                        break;
                    case LED_BLINK_FAST:
                        if(millis() - time_stamps[led_id] > FAST_BLINK_MS)
                        {
                            led_pin_state[led_id] ^= 1;
                            time_stamps[led_id] = millis();
                        }
                        break;
                    case LED_FLASH:
                        if(millis() - time_stamps[led_id] > FAST_BLINK_MS)
                        {
                            led_pin_state[led_id] = 0;
                            led_states[led_id] = LED_OFF;
                        }
                        else
                        {
                            led_pin_state[led_id] = 1;
                        }
                        break;
                }
                digitalWrite(led_pins[led_id], led_pin_state[led_id]);
            }
        };
};

#endif

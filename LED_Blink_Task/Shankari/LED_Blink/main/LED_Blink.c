#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "sdkconfig.h"

#define LED1 12
#define LED2 33
#define LED3 25
#define LED4 26
#define LED5 27
#define LED6 14
#define LED7 13
#define LED8 23

void set_leds_from_data(uint8_t data,  const int *pin_out)
{
 for (int i = 0; i < 8; i++)
{
        // bitwise-and bitmask with 0x80, so with this we can extract the value of leftmost bit.
        // Yxxxxxxx & 10000000 == 10000000, if Y = 1 and Yxxxxxxx & 10000000 == 00000000, if Y = 0
        // if it is 1, means the pin is usable, so we get into the if loop.
        // if ((mode_bitmask & 0x80) == 0x80)
        // {
            // We do the same process to extract the leftmost bit of data
            // If it is 1, then set the pin to 1, else set it to 0
            if ((data & 0x80) == 0x80)
            {
                // set gpio value to 1
                gpio_set_level((gpio_num_t)pin_out[i], 1);
            }
            else
            {
                // set gpio value to 0
                gpio_set_level((gpio_num_t)pin_out[i], 0);
            }
        //}
        // left-shift bitmask and data once, we do this 8 times to read all the bits
        // 00110011 << 1 = 01100110
      // mode_bitmask = mode_bitmask << 1;
        data = data << 1;
    }
}

void app_main()
{
// Just an array of pins used by bar graph led
    static const int pin_out[8] = {LED1, LED2, LED3, LED4, LED5, LED6, LED7, LED8};
    uint8_t data;
    //static int enabled_bar_graph_flag = 0;
   // int8_t mode_bitmask = bitmask[enabled_bar_graph_flag];
    static const char *TAG_BAR_GRAPH = "bar_graph";
    uint64_t bit_mask = 0;
    bit_mask = (1ULL << LED1) | (1ULL << LED2) | (1ULL << LED3) | (1ULL << LED4) |
               (1ULL << LED5) | (1ULL << LED6) | (1ULL << LED7) | (1ULL << LED8);//bit masking

    gpio_config_t io_conf;
    // bit mask for the pins, each bit maps to a GPIO
    io_conf.pin_bit_mask = bit_mask;
    // set gpio mode to input
    io_conf.mode = GPIO_MODE_OUTPUT;
    // enable pull up resistors
    io_conf.pull_up_en = 0;
    // disable pull down resistors
    io_conf.pull_down_en = 1;
    // disable gpio interrupts
    io_conf.intr_type = GPIO_INTR_DISABLE;

    esp_err_t err = gpio_config(&io_conf);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG_BAR_GRAPH, "error: %s", esp_err_to_name(err));
    return;
    }

    while(1)
    {
        data = 0xFF;
        //calling the function when data value is set to 0xFF
        set_leds_from_data(data, pin_out);
        vTaskDelay(1000/ portTICK_PERIOD_MS); //delay of 1s
        //calling the function when data value is 0x00 so leds will switch off for 1s
        set_leds_from_data(0x00, pin_out);
       vTaskDelay(1000/ portTICK_PERIOD_MS);
       //while loop continues
    }
}
    
   

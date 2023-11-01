#include <stdio.h>
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

// Define GPIO pin numbers for the LEDs
#define LED1 12
#define LED2 33
#define LED3 25
#define LED4 26
#define LED5 27
#define LED6 14
#define LED7 13
#define LED8 23

void app_main(void)
{
    // Define an array of GPIO pin numbers for the LEDs.
    int LED_PIN[] = {LED1, LED2, LED3, LED4, LED5, LED6, LED7, LED8};

    // Configure each GPIO pin in the array as an output.
    for (int i = 0; i < 8; i++)
    {
        gpio_set_direction(LED_PIN[i], GPIO_MODE_OUTPUT);
    }

    int on = 0;
    while (1)
    {
        // Toggle the 'on' variable to switch the LED state.
        on = !on;

        // Loop through the LED pins and set their state to 'on' (0 or 1).
        for (int i = 0; i < 8; i++)
        {
            gpio_set_level(LED_PIN[i], on);
        }

        // Introduce a 1-second delay using FreeRTOS.
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}
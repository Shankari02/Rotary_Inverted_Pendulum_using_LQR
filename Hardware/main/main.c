#include "driver/ledc.h"
#include "esp_err.h"
#include "esp_log.h"
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <math.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "rotary_encoder.h"

#define PI 3.14159
#define LEDC_TIMER LEDC_TIMER_0
#define LEDC_MODE LEDC_HIGH_SPEED_MODE
#define LEDC_CHANNEL LEDC_CHANNEL_0
#define LEDC_DUTY_RES LEDC_TIMER_10_BIT

// STEP and DIRECTION output pins for stepper motor driver.
static const gpio_num_t step_pin = GPIO_NUM_14;
static const gpio_num_t direction_pin = GPIO_NUM_27;

//TAG for ESP_LOGI function
static const char *TAG = "JMD";

//K is a constant for lqr controller calculated from MATLAB
float k[4] = {-0.0003, -0.1798, -6.2407, -8.4460};

//Setting the stable position
float state_setpoint[4] = {PI, 0, 0, 0};

//defining the variables required
float u = 0; // the input required to be provided (u = -kx)

// state variables [theta_s  theta_s_dot  theta_e  theta_e_dot]

float theta_s_curr = 0;
float theta_s_prev = 0;
float theta_s_dot = 0;
float theta_e_curr = 0;
float theta_e_prev = 0;
float theta_e_dot = 0;
float frequency = 2;
float frequency_change = 0;

//variable for encoder_readings
int encoder_value;

//error definitions for PID control
float prev_error = 0;
float difference_error = 0, cumulative_error = 0;
float kp = 0.5, kd = 0, ki = 0;

void set_ledc()
{
    // apply the LEDC PWM timer configuration
    ledc_timer_config_t ledc_timer = {
        .speed_mode = LEDC_MODE,
        .timer_num = LEDC_TIMER,
        .duty_resolution = LEDC_DUTY_RES,
        .freq_hz = frequency,
        .clk_cfg = LEDC_AUTO_CLK};
    ledc_timer_config(&ledc_timer);

    // apply the LEDC PWM channel configuration
    ledc_channel_config_t ledc_channel = {
        .speed_mode = LEDC_MODE,
        .channel = LEDC_CHANNEL,
        .timer_sel = LEDC_TIMER,
        .intr_type = LEDC_INTR_DISABLE,
        .gpio_num = step_pin,
        .duty = 0, // Set duty to 0%
        .hpoint = 0};
    ledc_channel_config(&ledc_channel);

    gpio_config_t io_conf = {
        .mode = GPIO_MODE_OUTPUT,
        .intr_type = GPIO_INTR_DISABLE,
        .pull_down_en = 0,
        .pull_up_en = 0,
        .pin_bit_mask = (1ULL << direction_pin),
    };
    gpio_config(&io_conf);

    ledc_set_duty(ledc_channel.speed_mode, ledc_channel.channel, 50);
    ledc_update_duty(ledc_channel.speed_mode, ledc_channel.channel);
}

float calc_error()
{
    // error = u
    difference_error = (u - prev_error) * 1000 / (20); // Calculating difference error
    cumulative_error = cumulative_error + ((u * (20)) / 1000); // Calculatong cumulative error

    frequency_change = kp * (u) + ki * (cumulative_error) + kd * (difference_error); // Frequency change
    prev_error = u;
    return frequency_change;
}

float set_freq()
{
    frequency = frequency + ((frequency_change * (20)) / (1000));
    ledc_set_freq(LEDC_MODE, LEDC_TIMER, abs(frequency));
    return frequency;
}

void direction()
{   
    // anticlockwise means -ve freq
    // clockwise means +ve freq
    if (frequency < 0)
    {
        gpio_set_level(direction_pin, 1); // Setting the direction pin high
    }
    else
    {
        gpio_set_level(direction_pin, 0); // Setting the direction pin low
    }
}


//functions to calculate variables

float calculate_theta_s()
{
    theta_s_curr = theta_s_curr + (0.036) * (frequency);
    return theta_s_curr;
}

float calculate_theta_s_dot()
{
    theta_s_dot = ((theta_s_curr - theta_s_prev) * (1000)) / (20);
    theta_s_prev = theta_s_curr;
    return theta_s_dot;
}

float calculate_theta_e()
{
    theta_e_curr = theta_e_curr + ((float)encoder_value);
    return theta_e_curr;
}

float calculate_theta_e_dot()
{
    theta_e_dot = ((theta_e_curr - theta_e_prev) * (1000))/ (20);
    theta_e_prev = theta_e_curr;
    return theta_e_dot;
}

float calculate_u()
{
    u = -(k[0] * (theta_s_curr - state_setpoint[0]) + k[1] * (theta_s_dot - state_setpoint[1]) + k[2] * (theta_e_curr - state_setpoint[2]) + k[3] * (theta_e_dot - state_setpoint[3]));
    return u;
}


void run_motor()
{
    // Rotary encoder underlying device is represented by a PCNT unit in this example
    uint32_t pcnt_unit = 0;

    // Create rotary encoder instance
    rotary_encoder_config_t config = ROTARY_ENCODER_DEFAULT_CONFIG((rotary_encoder_dev_t)pcnt_unit, 15, 17);
    rotary_encoder_t *encoder = NULL;
    ESP_ERROR_CHECK(rotary_encoder_new_ec11(&config, &encoder));

    // Filter out glitch (1us)
    ESP_ERROR_CHECK(encoder->set_glitch_filter(encoder, 1));

    // Start encoder
    ESP_ERROR_CHECK(encoder->start(encoder));
    // Report counter value

    while (1)
    {   
        //Printing the Encoder values
        encoder_value = encoder->get_counter_value(encoder);
        ESP_LOGI(TAG, "Encoder value: %d", encoder->get_counter_value(encoder));

        // Calling the respective functions to calculate the variables

        theta_e_curr = calculate_theta_e();
        theta_e_dot = calculate_theta_e_dot();

        theta_s_curr = calculate_theta_s();
        theta_s_dot = calculate_theta_s_dot();

        u = calculate_u();
        frequency_change = calc_error();

        frequency = set_freq();
        direction();

        ESP_LOGI(TAG, "[Theta_s: %f, Theta_s_dot: %f, Theta_e: %f, Theta_e_dot: %f]", theta_s_curr, theta_s_dot, theta_e_curr, theta_e_dot);
        ESP_LOGI(TAG, "U = %f", u);
    }

    vTaskDelay(pdMS_TO_TICKS(0.2)); //Delay
}

void app_main()
{

    set_ledc(); // Setting the ledc configuration

    // Creating task
    xTaskCreatePinnedToCore(&run_motor, "MOTOR RUNNING", 2048, NULL, 1, NULL, 1);
}

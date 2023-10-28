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
#define LEDC_DUTY_RES LEDC_TIMER_10_BIT // Set duty resolution to 10 bits
#define LEDC_DUTY (3150.77)                // Set duty to 50%. (2 ** 10) * 50% = 4096
#define LEDC_FREQUENCY (5000)           // Frequency in Hertz. Set frequency at 5 kHz

// STEP and DIRECTION output pins for stepper motor driver.
static const gpio_num_t step_pin = GPIO_NUM_14;
static const gpio_num_t direction_pin = GPIO_NUM_27;

static const uint32_t time_period = 0.002; // Update period set to 20 ms
//Defining a constant for number of steps of the stepper motor in 20 ms
float step_const = (20 / 1000) * (360 / 200); 

//TAG for ESP_LOGI functiom
static const char *TAG = "JMD";

//K is a constant for lqr controller calculated from MATLAB
float k[4] = {-0.0003, -0.1798, -6.2407, -8.4460};

//Setting the stable position
float state_setpoint[4] = {PI, 0, 0, 0};

//defining the variables required
float u; // the input required to be provided (u = -kx)

// state variables [theta_s  theta_s_dot  theta_e  theta_e_dot]

float theta_s_prev = 0; 
float theta_s_curr = 0;
float theta_e_prev = 0;
float theta_e_curr = 0;
float theta_s_dot = 0;
float theta_e_dot = 0;


float frequency = 0;
float frequency_change = 0;

//error definitions for PID control
float prev_error=0, difference, cumulative_error;
float kp = 1, ki = 0, kd = 0;

//variable for encoder_readings
uint32_t encoder_value;


//functions to calculate variables
float calculate_u(float u, float theta_s_curr, float theta_e_curr, float theta_s_dot, float theta_e_dot, float k[4], float state_setpoint[4])
{
    u = -(k[0] * (theta_s_curr - state_setpoint[0]) + k[1] * (theta_s_dot - state_setpoint[1]) + k[2] * (theta_e_curr - state_setpoint[2]) + k[3] * (theta_e_dot - state_setpoint[3]));
    return u;
}

float calculate_theta_s(float theta_s_curr, float step_const, float frequency_change)
{
    theta_s_curr = frequency*step_const;
    return theta_s_curr;
}

float calculate_theta_s_dot(float theta_s_dot, float theta_s_curr, float theta_s_prev, float step_const, float frequency_change, uint32_t time_period)
{
    theta_s_dot = (theta_s_curr - theta_s_prev) / (time_period);
    return theta_s_dot;
}

float calculate_theta_e(float theta_e_curr, uint32_t encoder_value)
{
    theta_e_curr = ((float)encoder_value - 180);
    return theta_e_curr;
}

float calculate_theta_e_dot(float theta_e_dot, float theta_e_curr, float theta_e_prev, uint32_t time_period)
{    
    theta_e_dot = ((float)encoder_value - 180 - theta_e_prev) / (time_period);
    return theta_e_dot;
}

void direction(float frequency)
{
    // anticlockwise means -ve freq
    // clockwise means +ve freq
    if (frequency <= 0)
    {
        if(frequency == 0){
            frequency = 1;
        }
        gpio_set_level(direction_pin, 1); // Setting the direction pin high
    }
    else
    {
        gpio_set_level(direction_pin, 0); // Setting the direction pin low
    }
}

void set_ledc()
{
    //Applying the LEDC PWM timer configuration
    ledc_timer_config_t ledc_timer = {
        .speed_mode = LEDC_MODE,
        .timer_num = LEDC_TIMER,
        .duty_resolution = LEDC_DUTY_RES,
        .freq_hz = LEDC_FREQUENCY, // Set output frequency at 5 kHz
        .clk_cfg = LEDC_AUTO_CLK};
    ledc_timer_config(&ledc_timer);

    //Applying LEDC PWM channel configuration
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

void find_angle()
{
    // Rotary encoder underlying device is represented by a PCNT unit in this example
    uint32_t pcnt_unit = 0;

    // Create rotary encoder instance
    rotary_encoder_config_t config = ROTARY_ENCODER_DEFAULT_CONFIG((rotary_encoder_dev_t)pcnt_unit, 15, 17);
    rotary_encoder_t *encoder = NULL;
    ESP_ERROR_CHECK(rotary_encoder_new_ec11(&config, &encoder));

    // Filter out glitch
    ESP_ERROR_CHECK(encoder->set_glitch_filter(encoder, 1));

    // Start encoder
    ESP_ERROR_CHECK(encoder->start(encoder));

    while (1)
    {
        encoder_value = encoder->get_counter_value(encoder);
        //Printing the Encoder values
        ESP_LOGI(TAG, "Encoder value : %d", encoder->get_counter_value(encoder));
        vTaskDelay(pdMS_TO_TICKS(50)); //Delay
    }
}

float calc_error(float u, float difference, float cumulative_error, float prev_error, float kp, float ki, float kd, float frequency_change)
{   

    //error = u;
    difference = u - prev_error; // Calculating difference error
    cumulative_error += u; // Calculatong cumulative error

    frequency_change = kp*u + ki*cumulative_error + kd*difference; // Frequency change
    return frequency_change;
}

float set_freq(float frequency, float frequency_change, uint32_t time_period)
{
    frequency = frequency + (frequency_change*time_period);
   
    ledc_set_freq(LEDC_MODE, LEDC_TIMER, abs(frequency));
    return frequency;
}

void run_motor()
{
    
    while (1)
    {   
        // Calling the respective functions to calculate the variables
        theta_e_curr = calculate_theta_e(theta_e_curr, encoder_value);
        theta_e_dot = calculate_theta_e_dot(theta_e_dot, theta_e_curr, theta_e_prev, time_period);
        theta_e_prev = theta_e_curr;       
        theta_s_curr = calculate_theta_s(theta_s_curr, step_const, frequency_change);
        theta_s_dot = calculate_theta_s_dot(theta_s_dot, theta_s_curr, theta_s_prev, step_const, frequency_change, time_period);
        theta_s_prev = theta_s_curr; 
        u = calculate_u(u, theta_s_curr, theta_s_curr, theta_e_dot, theta_e_dot, k, state_setpoint);        
        frequency_change = calc_error(u, difference, cumulative_error, prev_error, kp, ki, kd, frequency_change);
        prev_error = u;
        frequency = set_freq(frequency, frequency_change);
        direction(frequency);

        ESP_LOGI(TAG, " [theta_s: %f theta_s_dot: %f theta_e: %f theta_e_dot: %f ]", theta_s_curr, theta_s_dot, theta_e_curr, theta_e_dot);
        ESP_LOGI(TAG, "u = %f", u);

        vTaskDelay(pdMS_TO_TICKS(50)); //Delay
    }
}

void app_main()
{
   
    set_ledc(); // Setting the ledc configuration

    /* Creating two tasks :
        One for finding the angle from the encoder
        Other for running the motor acc to the specific frequency
    */

    xTaskCreate(&find_angle, "ENCODER RUNNING", 2048, NULL, 1, NULL);
    xTaskCreate(&run_motor, "MOTOR RUNNING", 2048, NULL, 5, NULL);
}

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
#define LEDC_OUTPUT_IO (5) // Define the output GPIO
#define LEDC_CHANNEL LEDC_CHANNEL_0
#define LEDC_DUTY_RES LEDC_TIMER_13_BIT // Set duty resolution to 13 bits
#define LEDC_DUTY (4096)                // Set duty to 50%. (2 ** 13) * 50% = 4096
#define LEDC_FREQUENCY (5000)           // Frequency in Hertz. Set frequency at 5 kHz

// STEP and DIRECTION output pins for stepper motor driver.
// static const gpio_num_t step_pin = GPIO_NUM_16;
static const gpio_num_t direction_pin = GPIO_NUM_17;

static const uint32_t time_period = 0.002; 
float step_const = (20 / 1000) * (360 / 200);
static const char *TAG = "RIP";

xQueueHandle encoder_queue;
//int txbuff[25];

float k[4] = {-0.0003, -0.1798, -6.2407, -8.4460};
float state_setpoint[4] = {PI, 0, 0, 0};
float u;
float theta_s_prev = 0;
float theta_s_curr = 0;
float theta_e_prev = 0;
float theta_e_curr = 0;
float theta_s_dot = 0;
float theta_e_dot = 0;
float frequency = 0;
float frequency_change = 0;
float prev_error=0, difference, cumulative_error;
float kp = 1, ki = 0, kd = 0;

float calculate_u(float u, float theta_s_curr, float theta_e_curr, float theta_s_dot, float theta_e_dot, float k[4], float state_setpoint[4])
{
    u = -(k[0] * (theta_s_curr - state_setpoint[0]) + k[1] * (theta_s_dot - state_setpoint[1]) + k[2] * (theta_e_curr - state_setpoint[2]) + k[3] * (theta_e_dot - state_setpoint[3]));
    ESP_LOGI(TAG, "u= %f", u);
    return u;
}

float calculate_theta_s(float theta_s_curr, float step_const, float frequency_change)
{
    theta_s_curr = frequency_change * (step_const);
    ESP_LOGI(TAG, "thetas_curr= %f", theta_s_curr);
    return theta_s_curr;
}

float calculate_theta_s_dot(float theta_s_dot, float theta_s_curr, float theta_s_prev, float step_const, float frequency_change, uint32_t time_period)
{
    theta_s_dot = (theta_s_curr - theta_s_prev) / (0.002);
    ESP_LOGI(TAG, "thetasdot_curr= %f", theta_s_dot);
    return theta_s_dot;
}

float calculate_theta_e(float theta_e_curr, uint32_t encoder_value)
{
    ESP_LOGI(TAG, "encoder value: %d", encoder_value);
    theta_e_curr = (float)encoder_value - 180;
    ESP_LOGI(TAG, "thetas_e_curr= %f", theta_e_curr);
    return theta_e_curr;
}

float calculate_theta_e_dot(float theta_e_dot, float theta_e_prev, float theta_e_curr, uint32_t time_period)
{    
    ESP_LOGI(TAG, "thetas_e_curr before dot calc= %f", theta_e_curr);
    theta_e_dot = (theta_e_curr - theta_e_prev) / (0.002);
    ESP_LOGI(TAG, "theta_e_dot_curr= %f", theta_e_dot);
    return theta_e_dot;
}

void direction(float frequency)
{
    // acw means -ve theta2
    // cw means +ve theta2

    if (frequency > 0)
    {
        gpio_set_level(direction_pin, 1);
    }
    else
    {
        gpio_set_level(direction_pin, 0);
    }
}

void set_ledc()
{
    // Prepare and then apply the LEDC PWM timer configuration
    ledc_timer_config_t ledc_timer = {
        .speed_mode = LEDC_MODE,
        .timer_num = LEDC_TIMER,
        .duty_resolution = LEDC_DUTY_RES,
        .freq_hz = LEDC_FREQUENCY, // Set output frequency at 5 kHz
        .clk_cfg = LEDC_AUTO_CLK};
    ledc_timer_config(&ledc_timer);

    // Prepare and then apply the LEDC PWM channel configuration
    ledc_channel_config_t ledc_channel = {
        .speed_mode = LEDC_MODE,
        .channel = LEDC_CHANNEL,
        .timer_sel = LEDC_TIMER,
        .intr_type = LEDC_INTR_DISABLE,
        .gpio_num = LEDC_OUTPUT_IO,
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
    rotary_encoder_config_t config = ROTARY_ENCODER_DEFAULT_CONFIG((rotary_encoder_dev_t)pcnt_unit, 14, 15);
    rotary_encoder_t *encoder = NULL;
    ESP_ERROR_CHECK(rotary_encoder_new_ec11(&config, &encoder));

    // Filter out glitch (1us)
    ESP_ERROR_CHECK(encoder->set_glitch_filter(encoder, 1));

    // Start encoder
    ESP_ERROR_CHECK(encoder->start(encoder));
    //ESP_LOGI(TAG, "inside find angle");
    
    // Report counter value
    while (1)
    {
        //ESP_LOGI(TAG, "inside fine angle while loops");
        uint32_t encoder_value = encoder->get_counter_value(encoder);
        xQueueSend(encoder_queue, &encoder_value , ((TickType_t)0));

        ESP_LOGI(TAG, "Encoder value: %d", encoder_value);
        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

float calc_error(float u, float difference, float cumulative_error, float prev_error, float kp, float ki, float kd, float frequency_change)
{   
    //ESP_LOGI(TAG, "inside calc error");
    //error = u;
    difference = u - prev_error;
    cumulative_error += u;

    frequency_change = kp*u + ki*cumulative_error + kd*difference;
    ESP_LOGI(TAG, "ERROR kp %f\n%f", u, kp);
    ESP_LOGI(TAG, "freq change %f", frequency_change);

    return frequency_change;
}

float set_freq(float frequency, float frequency_change)
{
    frequency += frequency_change;
    //ESP_LOGI(TAG, "inside set_freq");
    if(frequency>0)
    {
        ledc_set_freq(LEDC_MODE, LEDC_TIMER, frequency);
    }
    else{
        //frequency = abs(frequency);
        //ledc_set_freq(LEDC_MODE, LEDC_TIMER, frequency);
        ESP_LOGI(TAG, "Invalid freq");
    }
   // ESP_LOGI(TAG, "func done");
    ESP_LOGI(TAG, "Freq= %f", frequency);
    return frequency;
}

void run_motor()
{
    //ESP_LOGI(TAG, "inside run motor");
     ESP_LOGI(TAG, "thetas_e_curr= %f", encoder_value);
    while (1)
    {   
        //ESP_LOGI(TAG, "inside run motor while loop");
        uint32_t encoder_value;

        if (encoder_queue != 0)
        {
            // Receive a message on the created queue.  Block for 10 ticks if a
            // message is not immediately available.
            if (xQueueReceive(encoder_queue, &encoder_value, (TickType_t)10))
            {
                printf("Received encoder value %n", &encoder_value);
        
                // pcRxedMessage now points to the struct AMessage variable posted
                // by vATask.
            }
        }

         ESP_LOGI(TAG, "Encoder value before theta_e: %d", encoder_value);
        calculate_theta_e(theta_e_curr, encoder_value);
        //ESP_LOGI(TAG, "thetae complete");
        calculate_theta_e_dot(theta_e_dot, theta_e_prev, theta_e_curr, time_period);
        //ESP_LOGI(TAG, "thetaedot complete");
        
        calculate_theta_s(theta_s_curr, step_const, frequency_change);
        //ESP_LOGI(TAG, "thetas complete");
        calculate_theta_s_dot(theta_s_dot, theta_s_curr, theta_s_prev, step_const, frequency_change, time_period);
        //ESP_LOGI(TAG, "thetasdot complete");
        theta_s_prev = theta_s_curr;
        ESP_LOGI(TAG, "theta_s_prev aft value input: %f", theta_s_prev);
        theta_e_prev = theta_e_curr;
        ESP_LOGI(TAG, "theta_e_prev aft value input: %f", theta_e_prev);
        calculate_u(u, theta_s_curr, theta_s_curr, theta_e_dot, theta_e_dot, k, state_setpoint);
        ESP_LOGI(TAG, "u complete");
        
        calc_error(u, difference, cumulative_error, prev_error, kp, ki, kd, frequency_change);
        prev_error = u;
        ESP_LOGI(TAG, "freq done");
        direction(frequency);
        set_freq(frequency, frequency_change);
        ESP_LOGI(TAG, "freq set");
        

        vTaskDelay(pdMS_TO_TICKS(50));
        ESP_LOGI(TAG, "motor done");
    }
}

void app_main()
{
    // float m = ;
    // float M = ;
    // float L = ;

   
   

    set_ledc();

    ESP_LOGI(TAG, "set ledc");
    encoder_queue = xQueueCreate( 10, sizeof( uint32_t ) );

    xTaskCreate(&find_angle, "ENCODER RUNNING", 2048, NULL, 1, NULL);
    xTaskCreate(&run_motor, "MOTOR RUNNING", 2048, NULL, 2, NULL);
}

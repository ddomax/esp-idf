#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2s.h"
#include "driver/adc.h"
#include "driver/ledc.h"
#include "dig_i2s_adc.h"

#define I2S_ADC_UNIT            (ADC_UNIT_1)

/*
This example set up I2S ADC mode, The ADC scans 4 channels (ADC unit0: ch3, ch0, ch7 and ch6) in sequence, and the sampling frequency is 2M. 
each sample takes 2 bytes, low 12bit is the result of AD conversion and the high 4bit is the channel num.

        |-500K-|
         __     __     __     __     __     __     __     __     __  
WS    __|  |___|  |___|  |___|  |___|  |___|  |___|  |___|  |___|  |__

       CH3   CH0   CH7   CH6   CH3   CH0   CH7   CH6  ...

receive buffer: [ CH0 ][ CH3 ][ CH6 ][ CH7 ]...

*/

//Out put WS signal from gpio18(only for debug mode)
void i2s_adc_check_clk(void)
{
    PIN_FUNC_SELECT(GPIO_PIN_MUX_REG[18], PIN_FUNC_GPIO);
    gpio_set_direction(18, GPIO_MODE_DEF_OUTPUT);
    gpio_matrix_out(18, I2S0I_WS_OUT_IDX, 0, 0);
}

esp_err_t i2s_adc_setup(void)
{
    adc_channel_t channel[] = {
        ADC1_CHANNEL_3,
        ADC1_CHANNEL_0,
        ADC1_CHANNEL_7,
        ADC1_CHANNEL_6,
    };
    if (i2s_adc_init(I2S_NUM_0) != ESP_OK) {
        printf("i2s adc init fail\n");
        return ESP_FAIL;
    }
    //Configuring scan channels
    adc_i2s_scale_mode_init(I2S_ADC_UNIT, channel, 4);
    //rate = 2M
    //ADC sampling rate = 4000,000 / clkm_num (4,000,000 should be divisible by clkm_num)
    i2s_adc_set_clk(I2S_NUM_0, 2);
    if (i2s_adc_driver_install(I2S_NUM_0, NULL, NULL) != ESP_OK){
        printf("driver install fail\n");
        return ESP_FAIL;
    }
    //Uncomment this line only in debug mode.
    //i2s_adc_check_clk();
    return ESP_OK;
}

void app_main()
{
    uint16_t *buf = malloc(sizeof(uint16_t) * 100);
    if (!buf) {
        printf("buffer malloc fail\n");
        goto error;
    }
    if (i2s_adc_setup() != ESP_OK) {
        printf("i2s adc setup fail\n");
        goto error;
    }
    while(1) {
        if (i2s_adc_start(I2S_NUM_0, buf, 100*2) != ESP_OK) {
            goto error;
        }
        i2s_wait_adc_done(I2S_NUM_0, portMAX_DELAY);
        for (int i = 0; i < 100; i++) {
            printf("ch %d  %fv\n", (buf[i] >> 12), 3.3 - (3.14* (buf[i] & 0xfff)  / 4095) );
        }
        printf("------------------------\n");
        memset(buf, 0, 200);
        vTaskDelay(100/portTICK_PERIOD_MS);
    }
error:
    while(1) vTaskDelay(1000/portTICK_PERIOD_MS);
}
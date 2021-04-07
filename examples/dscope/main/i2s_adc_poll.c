#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_spi_flash.h"
#include "esp_err.h"
#include "esp_log.h"
#include "driver/i2s.h"
#include "driver/adc.h"
#include "esp_adc_cal.h"
#include "soc/syscon_periph.h"

static const char* TAG = "ad/da";

/*---------------------------------------------------------------
                            EXAMPLE CONFIG
---------------------------------------------------------------*/
//i2s number
#define EXAMPLE_I2S_NUM           (0)
//i2s sample rate
#define EXAMPLE_I2S_SAMPLE_RATE   (100000)
//i2s data bits
#define EXAMPLE_I2S_SAMPLE_BITS   (16)
//I2S read buffer length
// #define EXAMPLE_I2S_READ_LEN      (16 * 1024)
//I2S data format
#define EXAMPLE_I2S_FORMAT        (I2S_CHANNEL_FMT_ONLY_RIGHT)
//I2S channel number
#define EXAMPLE_I2S_CHANNEL_NUM   ((EXAMPLE_I2S_FORMAT < I2S_CHANNEL_FMT_ONLY_RIGHT) ? (2) : (1))
//I2S built-in ADC unit
#define I2S_ADC_UNIT              ADC_UNIT_1
//I2S built-in ADC channel
#define I2S_ADC_CHANNEL           ADC1_CHANNEL_3

//Out put WS signal from gpio18(only for debug mode)
void i2s_adc_check_clk(void)
{
    PIN_FUNC_SELECT(GPIO_PIN_MUX_REG[18], PIN_FUNC_GPIO);
    gpio_set_direction(18, GPIO_MODE_DEF_OUTPUT);
    gpio_matrix_out(18, I2S0I_WS_OUT_IDX, 0, 0);
}

/**
 * @brief I2S ADC/DAC mode init.
 */
void adc_i2s_init()
{
	 int i2s_num = EXAMPLE_I2S_NUM;
	 i2s_config_t i2s_config = {
        .mode = I2S_MODE_MASTER | I2S_MODE_RX | I2S_MODE_ADC_BUILT_IN,
        .sample_rate =  EXAMPLE_I2S_SAMPLE_RATE,
        .bits_per_sample = EXAMPLE_I2S_SAMPLE_BITS,
	    .communication_format = I2S_COMM_FORMAT_I2S_MSB,
	    .channel_format = EXAMPLE_I2S_FORMAT,
	    .intr_alloc_flags = 0,
	    .dma_buf_count = 32,
	    .dma_buf_len = 1024,
	    .use_apll = 0,
	 };
	 //install and start i2s driver
	 i2s_driver_install(i2s_num, &i2s_config, 0, NULL);
	 //init ADC pad
	 i2s_set_adc_mode(I2S_ADC_UNIT, I2S_ADC_CHANNEL);
     // workaround
    //  vTaskDelay(10 / portTICK_PERIOD_MS);

     // enable continuous adc sampling
     SYSCON.saradc_ctrl2.meas_num_limit = 0;
}

void adc_i2s_read(uint16_t *i2s_read_buff, size_t i2s_read_len, size_t *bytes_read)
{
    //read data from I2S bus, in this case, from ADC.
    i2s_read(EXAMPLE_I2S_NUM, (void*) i2s_read_buff, i2s_read_len, bytes_read, portMAX_DELAY);
}
#ifndef __I2S_ADC_H__
#define __I2S_ADC_H__

#include "esp_err.h"
#include <esp_types.h>
#include "soc/soc.h"
#include "esp_attr.h"
#include "driver/periph_ctrl.h"
#include "driver/adc.h"
#include "driver/i2s.h"

#ifdef __cplusplus
extern "C" {
#endif

void i2s_adc_check_clk(void);
void adc_i2s_init();
void adc_i2s_read(uint16_t *i2s_read_buff, size_t i2s_read_len, size_t *bytes_read);

#ifdef __cplusplus
}
#endif


#endif
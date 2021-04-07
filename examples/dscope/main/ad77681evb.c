/***************************************************************************//**
* @file ad77681evb.c
* @brief Implementation of Main Function.
* @author SPopa (stefan.popa@analog.com)
********************************************************************************
* Copyright 2017(c) Analog Devices, Inc.
*
* All rights reserved.
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:
* - Redistributions of source code must retain the above copyright
* notice, this list of conditions and the following disclaimer.
* - Redistributions in binary form must reproduce the above copyright
* notice, this list of conditions and the following disclaimer in
* the documentation and/or other materials provided with the
* distribution.
* - Neither the name of Analog Devices, Inc. nor the names of its
* contributors may be used to endorse or promote products derived
* from this software without specific prior written permission.
* - The use of this software may or may not infringe the patent rights
* of one or more patent holders. This license does not release you
* from the requirement that you obtain separate licenses from these
* patent holders to use this software.
* - Use of the software either in source or binary form, must be run
* on or directly connected to an Analog Devices Inc. component.
*
* THIS SOFTWARE IS PROVIDED BY ANALOG DEVICES "AS IS" AND ANY EXPRESS OR
* IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, NON-INFRINGEMENT,
* MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
* IN NO EVENT SHALL ANALOG DEVICES BE LIABLE FOR ANY DIRECT, INDIRECT,
* INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
* LIMITED TO, INTELLECTUAL PROPERTY RIGHTS, PROCUREMENT OF SUBSTITUTE GOODS OR
* SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
* CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
* OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
* OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*******************************************************************************/

/******************************************************************************/
/***************************** Include Files **********************************/
/******************************************************************************/
#include <stdio.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include "ad77681.h"
#include "spi_engine.h"
#include "parameters.h"
#include "delay.h"
#include "error.h"
// #include "driver/spi_master.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

struct spi_engine_init_param spi_eng_init_param  = {
	// .type = SPI_ENGINE,
	// .spi_engine_baseaddr = AD77681_SPI1_ENGINE_BASEADDR,
	.cs_delay = 0,
	.data_width = 32,
	// .ref_clk_hz = AD77681_SPI_ENG_REF_CLK_FREQ_HZ,
};

/**
 * @struct spi_platform_ops
 * @brief Structure holding SPI function pointers that point to the platform
 * specific function
 */
struct spi_platform_ops spi_eng_platform_ops = {
	/** SPI initialization function pointer */
	.spi_ops_init = spi_engine_init,
	/** SPI write/read function pointer */
	.spi_ops_write_and_read = spi_engine_write_and_read,
	/** SPI remove function pointer */
	.spi_ops_remove = spi_engine_remove
};

struct ad77681_init_param ADC_default_init_param = {
	/* SPI */
	{
		.chip_select = AD77681_SPI_CS,
		.max_speed_hz = 1000000,
		.mode = SPI_MODE_3,
		.platform_ops = &spi_eng_platform_ops,
		.extra = (void*)&spi_eng_init_param,
	},
	/* Configuration */
	AD77681_ECO,				// power_mode
	AD77681_MCLK_DIV_16,			// mclk_div
	AD77681_CLOCK_SEL_CRYSTAL,	// clock_sel
	AD77681_CONV_ONE_SHOT,	// conv_mode
	AD77681_POSITIVE_FS,		// diag_mux_sel
	false,						// conv_diag_sel
	AD77681_CONV_16BIT,			// conv_len
	AD77681_CRC, 				// crc_sel
	0, 							// status_bit
	AD77681_VCM_HALF_VCC,		/* VCM setup*/
	AD77681_AINn_ENABLED,		/* AIN- precharge buffer*/
	AD77681_AINp_ENABLED,		/* AIN+ precharge buffer*/
	AD77681_BUFn_ENABLED,		/* REF- buffer*/
	AD77681_BUFp_ENABLED,		/* REF+ buffer*/
	AD77681_FIR,			/* FIR Filter*/
	AD77681_SINC5_FIR_DECx64,	/* Decimate by 64*/
	0,				/* OS ratio of SINC3*/
	3000,				/* Reference voltage*/
	32768,				/* MCLK in kHz*/
};

void ad77681_evb_task(void *arg)
{
	struct ad77681_dev	*adc_dev;
	struct ad77681_status_registers *adc_status;
	uint8_t			adc_data[5];
	uint8_t 		*data;
	uint32_t 		i = 0;
	int32_t ret;

	ad77681_setup(&adc_dev, ADC_default_init_param, &adc_status);
	ESP_LOGI("ad77681_evb_task", "ADC Sample rate: %d Hz", adc_dev->sample_rate);

	while(1) {
		ret = ad77681_spi_read_adc_data(adc_dev, adc_data, AD77681_REGISTER_DATA_READ);
		printf("[ADC DATA]: 0x");
		for(i = 0; i < sizeof(adc_data) / sizeof(uint8_t); i++) {
			printf("%x", adc_data[i]);
		}
		printf("\r\n");
		ESP_LOGI("ad77681_evb_task", "%x", adc_data[i]);
        vTaskDelay(1000 / portTICK_RATE_MS);
	}
}

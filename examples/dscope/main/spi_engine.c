/*******************************************************************************
 *   @file   spi_engine.c
 *   @brief  Core implementation of the SPI Engine Driver.
 *   @author Sergiu Cuciurean (sergiu.cuciurean@analog.com)
********************************************************************************
 * Copyright 2019(c) Analog Devices, Inc.
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *  - Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *  - Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 *  - Neither the name of Analog Devices, Inc. nor the names of its
 *    contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *  - The use of this software may or may not infringe the patent rights
 *    of one or more patent holders.  This license does not release you
 *    from the requirement that you obtain separate licenses from these
 *    patent holders to use this software.
 *  - Use of the software either in source or binary form, must be run
 *    on or directly connected to an Analog Devices Inc. component.
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
 ******************************************************************************/

/******************************************************************************/
/***************************** Include Files **********************************/
/******************************************************************************/
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "esp_log.h"

/* In debug mode the printf function used in displaying the messages is causing
significant delays */
//#define DEBUG_LEVEL 2

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <inttypes.h>
#include "error.h"
#include "spi_engine.h"

#define PIN_NUM_MISO 12
#define PIN_NUM_MOSI 13
#define PIN_NUM_CLK  14
#define PIN_NUM_CS   15

//To speed up transfers, every SPI transfer sends a bunch of lines. This define specifies how many. More means more memory use,
//but less overhead for setting up / finishing transfers. Make sure 240 is dividable by this.
#define PARALLEL_LINES 16

/******************************************************************************/
/***************************** Static variables *******************************/
/******************************************************************************/
static spi_device_handle_t spi;

/******************************************************************************/
/************************** Functions Implementation **************************/
/******************************************************************************/
/**
 * @brief Write/read on the spi interface
 *
 * @param desc Decriptor containing SPI interface parameters
 * @param data Pointer to data buffer
 * @param bytes_number Number of bytes to transfer
 * @return int32_t - SUCCESS if the transfer finished
 *		   - FAILURE if the memory allocation or transfer failed
 */
int32_t spi_engine_write_and_read(struct spi_desc *desc,
				  uint8_t *data,
				  uint16_t bytes_number)
{
    spi_transaction_t t;
    memset(&t, 0, sizeof(t));
    ESP_LOGI("SPI Engine", "bytes_number: %d", bytes_number);
    t.length = bytes_number * 8; // transaction length in bits!
    t.flags = SPI_TRANS_USE_RXDATA | SPI_TRANS_USE_TXDATA;
    t.user = (void*)1;
    memcpy(t.tx_data, data, bytes_number);

    spi_device_handle_t spi = *(spi_device_handle_t*)(desc->extra);
    esp_err_t ret = spi_device_polling_transmit(spi, &t);
    assert( ret == ESP_OK );

    memcpy(data, t.rx_data, bytes_number);

	int32_t ret_spi = SUCCESS;
	return ret_spi;
}

int32_t spi_engine_init(struct spi_desc **desc,
			const struct spi_init_param *param)
{
	*desc = malloc(sizeof(**desc));
	if(! *desc) {
		free(*desc);
		return FAILURE;
	}

    esp_err_t ret;

	(*desc)->max_speed_hz = param->max_speed_hz;
	(*desc)->chip_select = param->chip_select;
	(*desc)->mode = param->mode;

    spi_bus_config_t buscfg={
        .miso_io_num=PIN_NUM_MISO,
        .mosi_io_num=PIN_NUM_MOSI,
        .sclk_io_num=PIN_NUM_CLK,
        .quadwp_io_num=-1,
        .quadhd_io_num=-1,
        .max_transfer_sz=PARALLEL_LINES*320*2+8
    };
    spi_device_interface_config_t devcfg={
#ifdef CONFIG_LCD_OVERCLOCK
        .clock_speed_hz=26*1000*1000,           //Clock out at 26 MHz
#else
        .clock_speed_hz=100*1000,           //Clock out at 100 kHz
#endif
        .mode=3,                                //SPI mode 0
        .spics_io_num=PIN_NUM_CS,               //CS pin
        .queue_size=7,                          //We want to be able to queue 7 transactions at a time
        // .pre_cb=lcd_spi_pre_transfer_callback,  //Specify pre-transfer callback to handle D/C line
    };
    //Initialize the SPI bus
    ret=spi_bus_initialize(HSPI_HOST, &buscfg, 1);
    ESP_ERROR_CHECK(ret);
    //Attach the LCD to the SPI bus
    ret=spi_bus_add_device(HSPI_HOST, &devcfg, &spi);
    ESP_ERROR_CHECK(ret);

    (*desc)->extra = &spi;

    int32_t ret_spi = SUCCESS;
	return ret_spi;
}

// /* Write SPI Engine's axi registers */
int32_t spi_engine_write(struct spi_engine_desc *desc,
			 uint32_t reg_addr,
			 uint32_t reg_data)
{
    int32_t ret = FAILURE;
	return ret;
}

// /* Read SPI Engine's axi registers */
int32_t spi_engine_read(struct spi_engine_desc *desc,
			uint32_t reg_addr,
			uint32_t *reg_data)
{
    int32_t ret = FAILURE;
	return ret;
}

// /* Free the resources used by the SPI engine device */
int32_t spi_engine_remove(struct spi_desc *desc)
{
    int32_t ret = FAILURE;
	return ret;
}
#include "freertos/FreeRTOS.h"

#define NOP() asm volatile ("nop")

static inline unsigned long micros()
{
    return (unsigned long) (esp_timer_get_time());
}

static inline void udelay(uint32_t us)
{
    uint32_t m = micros();
    if(us){
        uint32_t e = (m + us);
        if(m > e){ //overflow
            while(micros() > e){
                NOP();
            }
        }
        while(micros() < e){
            NOP();
        }
    }
}

static inline void mdelay(uint32_t ms)
{
    uint32_t us = 1000 * ms;
    uint32_t m = micros();
    if(us){
        uint32_t e = (m + us);
        if(m > e){ //overflow
            while(micros() > e){
                NOP();
            }
        }
        while(micros() < e){
            NOP();
        }
    }
}
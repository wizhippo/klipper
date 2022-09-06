/* Code to setup clocks and gpio on STM32MP1
 *
 * Copyright (C) 2022 lodge 'Connor <shilong.native@gmail.com>
 *
 * This file may be distributed under the terms of the GNU GPLv3 license.
 */

// USB and I2C is not supported, SPI is untested!

#include "autoconf.h"         // CONFIG_CLOCK_REF_FREQ
#include "board/armcm_boot.h" // VectorTable
#include "board/irq.h"        // irq_disable
#include "command.h"          // DECL_CONSTANT_STR
#include "internal.h"         // enable_pclock
#include "sched.h"            // sched_main
#include "board/gpio.h"       // gpio_out_setup

//#include "sys.h"

#define FREQ_PERIPH (CONFIG_CLOCK_FREQ / 4)

// Enable a peripheral clock
void enable_pclock(uint32_t periph_base)
{
    // periph_base determines in which bitfield at wich position to set a bit
    // E.g. D2_AHB1PERIPH_BASE is the adress offset of the given bitfield
    if (periph_base < MCU_APB2_PERIPH_BASE)
    {
        if (periph_base <= LPTIM1_BASE)
        {
            uint32_t pos = (periph_base - TIM2_BASE) / 0x1000;
            RCC->MC_APB1ENSETR |= (1 << pos); // we assume it is not in APB1HENR
            RCC->MC_APB1ENSETR;
        }
        else if (periph_base == WWDG1_BASE)
        {
            RCC->MC_APB1ENSETR |= (1 << 0x1C); // we assume it is not in APB1HENR
            RCC->MC_APB1ENSETR;
        }
        else if (periph_base <= SPI3_BASE)
        {
            uint32_t pos = (periph_base - TIM2_BASE) / 0x1000;
            RCC->MC_APB1ENSETR |= (1 << pos); // we assume it is not in APB1HENR
            RCC->MC_APB1ENSETR;
        }
        else if (periph_base == SPDIFRX_BASE)
        {
            RCC->MC_APB1ENSETR |= (1 << 0x1A); // we assume it is not in APB1HENR
            RCC->MC_APB1ENSETR;
        }
        else if (periph_base <= UART5_BASE)
        {
            uint32_t pos = (periph_base - TIM2_BASE) / 0x1000;
            RCC->MC_APB1ENSETR |= (1 << pos); // we assume it is not in APB1HENR
            RCC->MC_APB1ENSETR;
        }
        else if (periph_base <= I2C5_BASE)
        {
            uint32_t pos = ((periph_base + 0x3000) - TIM2_BASE) / 0x1000;
            RCC->MC_APB1ENSETR |= (1 << pos); // we assume it is not in APB1HENR
            RCC->MC_APB1ENSETR;
        }
        else if (periph_base == CEC_BASE)
        {
            RCC->MC_APB1ENSETR |= (1 << 0x1B); // we assume it is not in APB1HENR
            RCC->MC_APB1ENSETR;
        }
        else if (periph_base == DAC1_BASE)
        {
            RCC->MC_APB1ENSETR |= (1 << 0x1D); // we assume it is not in APB1HENR
            RCC->MC_APB1ENSETR;
        }
        else if (periph_base == UART7_BASE)
        {
            RCC->MC_APB1ENSETR |= (1 << 0x12); // we assume it is not in APB1HENR
            RCC->MC_APB1ENSETR;
        }
        else if (periph_base == UART8_BASE)
        {
            RCC->MC_APB1ENSETR |= (1 << 0x13); // we assume it is not in APB1HENR
            RCC->MC_APB1ENSETR;
        }
    }
    else if (periph_base < MCU_AHB2_PERIPH_BASE)
    {
        if (periph_base <= TIM8_BASE)
        {
            uint32_t pos = (periph_base - TIM1_BASE) / 0x1000;
            RCC->MC_APB2ENSETR |= (1 << pos); // we assume it is not in APB1HENR
            RCC->MC_APB2ENSETR;
        }
        if (periph_base == USART6_BASE)
        {
            RCC->MC_APB2ENSETR |= (1 << 0x0D); // we assume it is not in APB1HENR
            RCC->MC_APB2ENSETR;
        }
        if (periph_base <= SPI4_BASE)
        {
            uint32_t pos = ((periph_base + 0x4000) - TIM1_BASE) / 0x1000;
            RCC->MC_APB2ENSETR |= (1 << pos); // we assume it is not in APB1HENR
            RCC->MC_APB2ENSETR;
        }
        if (periph_base <= TIM17_BASE)
        {
            uint32_t pos = ((periph_base - 0x4000) - TIM1_BASE) / 0x1000;
            RCC->MC_APB2ENSETR |= (1 << pos); // we assume it is not in APB1HENR
            RCC->MC_APB2ENSETR;
        }
        if (periph_base == SPI5_BASE)
        {
            RCC->MC_APB2ENSETR |= (1 << 0x0A); // we assume it is not in APB1HENR
            RCC->MC_APB2ENSETR;
        }
    }
    else if (periph_base < MCU_AHB3_PERIPH_BASE)
    {
        if (periph_base <= DMAMUX1_BASE)
        {
            uint32_t pos = (periph_base - DMA1_BASE) / 0x1000;
            RCC->MC_AHB2ENSETR |= (1 << pos); // we assume it is not in APB1HENR
            RCC->MC_AHB2ENSETR;
        }
        if (periph_base <= ADC12_COMMON_BASE)
        {
            RCC->MC_AHB2ENSETR |= RCC_MC_AHB2ENSETR_ADC12EN;
            RCC->MC_AHB2ENSETR;
        }
    }
    else if (periph_base < MCU_AHB4_PERIPH_BASE)
    {
        uint32_t pos = (periph_base - MCU_AHB3_PERIPH_BASE) / 0x1000;
        RCC->MC_AHB3ENSETR |= (1 << pos);
        RCC->MC_AHB3ENSETR;
    }
    else if (periph_base < MCU_APB3_PERIPH_BASE)
    {
        uint32_t pos = ((periph_base - 0x2000) - MCU_AHB4_PERIPH_BASE) / 0x1000;
        RCC->MC_AHB4ENSETR |= (1 << pos);
        RCC->MC_AHB4ENSETR;
    }
    else if (periph_base < APB_DEBUG_PERIPH_BASE)
    {
        uint32_t pos = (periph_base - MCU_APB3_PERIPH_BASE) / 0x1000;
        RCC->MC_APB3ENSETR |= (1 << pos);
        RCC->MC_APB3ENSETR;
    }
    else if (periph_base < GPV_PERIPH_BASE)
    {
        if (periph_base == GPIOZ_BASE)
        {
            RCC->MC_AHB5ENSETR |= (1 << 0x00); // we assume it is not in APB1HENR
            RCC->MC_AHB5ENSETR;
        }
    }
    else if (periph_base < MPU_APB4_PERIPH_BASE)
    {
        uint32_t pos = (periph_base - MPU_AHB6_PERIPH_BASE) / 0x1000;
        RCC->MC_AHB6ENSETR |= (1 << pos);
        RCC->MC_AHB6ENSETR;
    }
    else if (periph_base < MPU_APB5_PERIPH_BASE)
    {
        uint32_t pos = (periph_base - MPU_APB4_PERIPH_BASE) / 0x1000;
        RCC->MC_APB4ENSETR |= (1 << pos);
        RCC->MC_APB4ENSETR;
    }
    else
    {
        uint32_t pos = (periph_base - MPU_APB5_PERIPH_BASE) / 0x1000;
        RCC->MC_APB5ENSETR |= (1 << pos);
        RCC->MC_APB5ENSETR;
    }
}

// Check if a peripheral clock has been enabled
int is_enabled_pclock(uint32_t periph_base)
{
    if (periph_base < MCU_APB2_PERIPH_BASE)
    {
        uint32_t pos = (periph_base - MCU_APB1_PERIPH_BASE) / 0x1000;
        return RCC->MC_APB1ENSETR & (1 << pos); // we assume it is not in APB1HENR
    }
    else if (periph_base < MCU_AHB2_PERIPH_BASE)
    {
        uint32_t pos = (periph_base - MCU_APB2_PERIPH_BASE) / 0x1000;
        return RCC->MC_APB2ENSETR & (1 << pos);
    }
    else if (periph_base < MCU_AHB3_PERIPH_BASE)
    {
        uint32_t pos = (periph_base - MCU_AHB2_PERIPH_BASE) / 0x1000;
        return RCC->MC_AHB2ENSETR & (1 << pos);
    }
    else if (periph_base < MCU_AHB4_PERIPH_BASE)
    {
        uint32_t pos = (periph_base - MCU_AHB3_PERIPH_BASE) / 0x1000;
        return RCC->MC_AHB3ENSETR & (1 << pos);
    }
    else if (periph_base < MCU_APB3_PERIPH_BASE)
    {
        uint32_t pos = (periph_base - MCU_AHB4_PERIPH_BASE) / 0x1000;
        return RCC->MC_AHB4ENSETR & (1 << pos);
    }
    else if (periph_base < APB_DEBUG_PERIPH_BASE)
    {
        uint32_t pos = (periph_base - MCU_APB3_PERIPH_BASE) / 0x1000;
        return RCC->MC_APB3ENSETR & (1 << pos);
    }
    else if (periph_base < GPV_PERIPH_BASE)
    {
        uint32_t pos = (periph_base - MPU_AHB5_PERIPH_BASE) / 0x1000;
        return RCC->MC_AHB5ENSETR & (1 << pos);
    }
    else if (periph_base < MPU_APB4_PERIPH_BASE)
    {
        uint32_t pos = (periph_base - MPU_AHB6_PERIPH_BASE) / 0x1000;
        return RCC->MC_AHB6ENSETR & (1 << pos);
    }
    else if (periph_base < MPU_APB5_PERIPH_BASE)
    {
        uint32_t pos = (periph_base - MPU_APB4_PERIPH_BASE) / 0x1000;
        return RCC->MC_APB4ENSETR & (1 << pos);
    }
    else
    {
        uint32_t pos = (periph_base - MPU_APB5_PERIPH_BASE) / 0x1000;
        return RCC->MC_APB5ENSETR & (1 << pos);
    }
}

// Return the frequency of the given peripheral clock
uint32_t get_pclock_frequency(uint32_t periph_base)
{
    return FREQ_PERIPH;
}

// Enable a GPIO peripheral clock
void gpio_clock_enable(GPIO_TypeDef *regs)
{
    enable_pclock((uint32_t)regs);
}

// Main entry point - called from armcm_boot.c:ResetHandler()
void armcm_main(void)
{
    SystemInit(); // Run SystemInit() and then restore VTOR
    // SCB->VTOR = (uint32_t)(0x00000000);

    //-----------------------------------------------------
    gpio_out_setup(GPIO('A', 11), 1); // GPIOA_11设置为输出, 高电平; 开启tf-card-led

    sched_main();
}

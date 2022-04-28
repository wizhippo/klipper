// Code to setup clocks and gpio on stm32h7
//
// Copyright (C) 2020 Konstantin Vogel <konstantin.vogel@gmx.net>
//
// This file may be distributed under the terms of the GNU GPLv3 license.


// USB and I2C is not supported, SPI is untested!

#include "autoconf.h" // CONFIG_CLOCK_REF_FREQ
#include "board/armcm_boot.h" // VectorTable
#include "board/irq.h" // irq_disable
#include "command.h" // DECL_CONSTANT_STR
#include "internal.h" // enable_pclock
#include "sched.h" // sched_main
#include "board/gpio.h" // gpio_out_setup

//#include "sys.h"

#define FREQ_PERIPH (CONFIG_CLOCK_FREQ / 4)

// Enable a peripheral clock
void
enable_pclock(uint32_t periph_base)
{
    // periph_base determines in which bitfield at wich position to set a bit
    // E.g. D2_AHB1PERIPH_BASE is the adress offset of the given bitfield
    if (periph_base < MCU_APB2_PERIPH_BASE) {
        if(periph_base <= LPTIM1_BASE) {
            uint32_t pos = (periph_base - TIM2_BASE) / 0x1000;
            RCC->MC_APB1ENSETR |= (1<<pos); // we assume it is not in APB1HENR
            RCC->MC_APB1ENSETR;
        } else if (periph_base == WWDG1_BASE) {
            RCC->MC_APB1ENSETR |= (1<<0x1C); // we assume it is not in APB1HENR
            RCC->MC_APB1ENSETR;
        } else if (periph_base <= SPI3_BASE) {
            uint32_t pos = (periph_base - TIM2_BASE) / 0x1000;
            RCC->MC_APB1ENSETR |= (1<<pos); // we assume it is not in APB1HENR
            RCC->MC_APB1ENSETR;
        } else if (periph_base == SPDIFRX_BASE) {
            RCC->MC_APB1ENSETR |= (1<<0x1A); // we assume it is not in APB1HENR
            RCC->MC_APB1ENSETR;
        } else if (periph_base <= UART5_BASE) {
            uint32_t pos = (periph_base - TIM2_BASE) / 0x1000;
            RCC->MC_APB1ENSETR |= (1<<pos); // we assume it is not in APB1HENR
            RCC->MC_APB1ENSETR;
        } else if (periph_base <= I2C5_BASE) {
            uint32_t pos = ((periph_base + 0x3000) - TIM2_BASE) / 0x1000;
            RCC->MC_APB1ENSETR |= (1<<pos); // we assume it is not in APB1HENR
            RCC->MC_APB1ENSETR;
        } else if (periph_base == CEC_BASE) {
            RCC->MC_APB1ENSETR |= (1<<0x1B); // we assume it is not in APB1HENR
            RCC->MC_APB1ENSETR;
        } else if (periph_base == DAC1_BASE) {
            RCC->MC_APB1ENSETR |= (1<<0x1D); // we assume it is not in APB1HENR
            RCC->MC_APB1ENSETR;
        } else if (periph_base == UART7_BASE) {
            RCC->MC_APB1ENSETR |= (1<<0x12); // we assume it is not in APB1HENR
            RCC->MC_APB1ENSETR;
        } else if (periph_base == UART8_BASE) {
            RCC->MC_APB1ENSETR |= (1<<0x13); // we assume it is not in APB1HENR
            RCC->MC_APB1ENSETR;
        }
    } else if (periph_base < MCU_AHB2_PERIPH_BASE) {
        if(periph_base <= TIM8_BASE) {
            uint32_t pos = (periph_base - TIM1_BASE) / 0x1000;
            RCC->MC_APB2ENSETR |= (1<<pos); // we assume it is not in APB1HENR
            RCC->MC_APB2ENSETR;
        } if(periph_base == USART6_BASE) {
            RCC->MC_APB2ENSETR |= (1<<0x0D); // we assume it is not in APB1HENR
            RCC->MC_APB2ENSETR;
        } if(periph_base <= SPI4_BASE) {
            uint32_t pos = ((periph_base + 0x4000) - TIM1_BASE) / 0x1000;
            RCC->MC_APB2ENSETR |= (1<<pos); // we assume it is not in APB1HENR
            RCC->MC_APB2ENSETR;
        } if(periph_base <= TIM17_BASE) {
            uint32_t pos = ((periph_base - 0x4000) - TIM1_BASE) / 0x1000;
            RCC->MC_APB2ENSETR |= (1<<pos); // we assume it is not in APB1HENR
            RCC->MC_APB2ENSETR;
        } if(periph_base == SPI5_BASE) {
            RCC->MC_APB2ENSETR |= (1<<0x0A); // we assume it is not in APB1HENR
            RCC->MC_APB2ENSETR;
        }
    } else if (periph_base < MCU_AHB3_PERIPH_BASE) {
        if(periph_base <= DMAMUX1_BASE) {
            uint32_t pos = (periph_base - DMA1_BASE) / 0x1000;
            RCC->MC_AHB2ENSETR |= (1<<pos); // we assume it is not in APB1HENR
            RCC->MC_AHB2ENSETR;
        } if(periph_base <= ADC12_COMMON_BASE) {
            RCC->MC_AHB2ENSETR |= RCC_MC_AHB2ENSETR_ADC12EN;
            RCC->MC_AHB2ENSETR;
        }
    } else if (periph_base < MCU_AHB4_PERIPH_BASE) {
        uint32_t pos = (periph_base - MCU_AHB3_PERIPH_BASE) / 0x1000;
        RCC->MC_AHB3ENSETR |= (1<<pos);
        RCC->MC_AHB3ENSETR;
    } else if (periph_base < MCU_APB3_PERIPH_BASE) {
        uint32_t pos = ((periph_base - 0x2000) - MCU_AHB4_PERIPH_BASE) / 0x1000;
        RCC->MC_AHB4ENSETR |= (1<<pos);
        RCC->MC_AHB4ENSETR;
    } else if (periph_base < APB_DEBUG_PERIPH_BASE) {
        uint32_t pos = (periph_base - MCU_APB3_PERIPH_BASE) / 0x1000;
        RCC->MC_APB3ENSETR |= (1<<pos);
        RCC->MC_APB3ENSETR;
    } else if (periph_base < GPV_PERIPH_BASE) {
        if(periph_base == GPIOZ_BASE) {
            RCC->MC_AHB5ENSETR |= (1<<0x00); // we assume it is not in APB1HENR
            RCC->MC_AHB5ENSETR;
        }
    } else if (periph_base < MPU_APB4_PERIPH_BASE) {
        uint32_t pos = (periph_base - MPU_AHB6_PERIPH_BASE) / 0x1000;
        RCC->MC_AHB6ENSETR |= (1<<pos);
        RCC->MC_AHB6ENSETR;
    } else if (periph_base < MPU_APB5_PERIPH_BASE) {
        uint32_t pos = (periph_base - MPU_APB4_PERIPH_BASE) / 0x1000;
        RCC->MC_APB4ENSETR |= (1<<pos);
        RCC->MC_APB4ENSETR;
    } else {
        uint32_t pos = (periph_base - MPU_APB5_PERIPH_BASE) / 0x1000;
        RCC->MC_APB5ENSETR |= (1<<pos);
        RCC->MC_APB5ENSETR;
    }
}

// Check if a peripheral clock has been enabled
int
is_enabled_pclock(uint32_t periph_base)
{
    if (periph_base < MCU_APB2_PERIPH_BASE) {
        uint32_t pos = (periph_base - MCU_APB1_PERIPH_BASE) / 0x1000;
        return RCC->MC_APB1ENSETR & (1<<pos); // we assume it is not in APB1HENR
    } else if (periph_base < MCU_AHB2_PERIPH_BASE) {
        uint32_t pos = (periph_base - MCU_APB2_PERIPH_BASE) / 0x1000;
        return RCC->MC_APB2ENSETR & (1<<pos);
    } else if (periph_base < MCU_AHB3_PERIPH_BASE) {
        uint32_t pos = (periph_base - MCU_AHB2_PERIPH_BASE) / 0x1000;
        return RCC->MC_AHB2ENSETR & (1<<pos);
    } else if (periph_base < MCU_AHB4_PERIPH_BASE) {
        uint32_t pos = (periph_base - MCU_AHB3_PERIPH_BASE) / 0x1000;
        return RCC->MC_AHB3ENSETR & (1<<pos);
    } else if (periph_base < MCU_APB3_PERIPH_BASE) {
        uint32_t pos = (periph_base - MCU_AHB4_PERIPH_BASE) / 0x1000;
        return RCC->MC_AHB4ENSETR & (1<<pos);
    } else if (periph_base < APB_DEBUG_PERIPH_BASE) {
        uint32_t pos = (periph_base - MCU_APB3_PERIPH_BASE) / 0x1000;
        return RCC->MC_APB3ENSETR & (1<<pos);
    } else if (periph_base < GPV_PERIPH_BASE) {
        uint32_t pos = (periph_base - MPU_AHB5_PERIPH_BASE) / 0x1000;
        return RCC->MC_AHB5ENSETR & (1<<pos);
    } else if (periph_base < MPU_APB4_PERIPH_BASE) {
        uint32_t pos = (periph_base - MPU_AHB6_PERIPH_BASE) / 0x1000;
        return RCC->MC_AHB6ENSETR & (1<<pos);
    } else if (periph_base < MPU_APB5_PERIPH_BASE) {
        uint32_t pos = (periph_base - MPU_APB4_PERIPH_BASE) / 0x1000;
        return RCC->MC_APB4ENSETR & (1<<pos);
    } else {
        uint32_t pos = (periph_base - MPU_APB5_PERIPH_BASE) / 0x1000;
        return RCC->MC_APB5ENSETR & (1<<pos);
    }
}

// Return the frequency of the given peripheral clock
uint32_t
get_pclock_frequency(uint32_t periph_base)
{
    return FREQ_PERIPH;
}

// Enable a GPIO peripheral clock
void
gpio_clock_enable(GPIO_TypeDef *regs)
{
    enable_pclock((uint32_t)regs);
}

#if !CONFIG_STM32_CLOCK_REF_INTERNAL
DECL_CONSTANT_STR("RESERVE_PINS_crystal", "PH0,PH1");
#endif

void clock_init(uint32_t plln, uint32_t pllm, uint32_t pllp, uint32_t pllq, uint32_t pllfracv)
{
    SET_BIT(PWR->CR1, PWR_CR1_DBP);    /* 使能访问backup区域     */
    MODIFY_REG(RCC->BDCR, RCC_BDCR_LSEDRV, (uint32_t)(RCC_BDCR_LSEDRV_2));    /* 允许修改LSE    */
    
    WRITE_REG(RCC->OCENCLRR, RCC_OCENCLRR_HSEON);
    while( (RCC->OCRDYR & RCC_OCRDYR_HSERDY) == RCC_OCRDYR_HSERDY );    /* Wait till HSE is disabled */

    WRITE_REG(RCC->OCENCLRR, (RCC_OCENCLRR_DIGBYP | RCC_OCENSETR_HSEBYP));  /* Clear remaining bits */

    /* Enable oscillator */
    SET_BIT(RCC->OCENSETR, RCC_OCENSETR_HSEON);
    while( (RCC->OCRDYR & RCC_OCRDYR_HSERDY) != RCC_OCRDYR_HSERDY );

    SET_BIT(RCC->RDLSICR, RCC_RDLSICR_LSION);
    while((RCC->RDLSICR & RCC_RDLSICR_LSIRDY)  !=  RCC_RDLSICR_LSIRDY );    /* Wait till LSI is ready */
    
//---------- Configure PLL1 -------------------------------------------
    CLEAR_BIT(RCC->PLL1CR, RCC_PLL1CR_DIVPEN | RCC_PLL1CR_DIVQEN | RCC_PLL1CR_DIVREN );
    CLEAR_BIT(RCC->PLL1CR, RCC_PLL1CR_PLLON);       /* Disable the main PLL. */
    while((RCC->PLL1CR & RCC_PLL1CR_PLL1RDY) ==  RCC_PLL1CR_PLL1RDY );  /* Wait till PLL is ready */

    /* Configure PLL1 and PLL2 clock source */
    MODIFY_REG( RCC->RCK12SELR, RCC_RCK12SELR_PLL12SRC, RCC_RCK12SELR_PLL12SRC_1 );
    while( (RCC->RCK12SELR & RCC_RCK12SELR_PLL12SRCRDY) != RCC_RCK12SELR_PLL12SRCRDY ); /* Wait till PLL SOURCE is ready */

    /* Configure the PLL1 multiplication and division factors. */        
    MODIFY_REG( RCC->PLL1CFGR1, (RCC_PLL1CFGR1_DIVN | RCC_PLL1CFGR1_DIVM1) , ( 99U | (2U << 16U )));
    MODIFY_REG( RCC->PLL1CFGR2, (RCC_PLL1CFGR2_DIVP | RCC_PLL1CFGR2_DIVQ | RCC_PLL1CFGR2_DIVR), ( 0U | ( (0U) <<8U ) | ( (0U) <<16U) ));

    /* Configure the Fractional Divider */
    CLEAR_BIT(RCC->PLL1FRACR, RCC_PLL1FRACR_FRACLE);    /*Set FRACLE to 0 */
    MODIFY_REG(RCC->PLL1FRACR, RCC_PLL1FRACR_FRACV, (uint32_t)0 << RCC_PLL1FRACR_FRACV_Pos);    /* Configure PLL  PLL1FRACV  in fractional mode*/
    SET_BIT(RCC->PLL1FRACR, RCC_PLL1FRACR_FRACLE);      /* Set FRACLE to 1 */

    /* Configure the Spread Control */
    CLEAR_BIT(RCC->PLL1CR, RCC_PLL1CR_SSCG_CTRL);
    SET_BIT(RCC->PLL1CR, RCC_PLL1CR_PLLON );    /* Enable the PLL1. */
    while((RCC->PLL1CR & RCC_PLL1CR_PLL1RDY)  !=  RCC_PLL1CR_PLL1RDY );/* Wait till PLL is ready */

    /* Enable post-dividers */
    SET_BIT(RCC->PLL1CR, RCC_PLL1CR_DIVPEN | RCC_PLL1CR_DIVQEN | RCC_PLL1CR_DIVREN );
    
//-------- Configure PLL2 --------------------------------------
    CLEAR_BIT(RCC->PLL2CR, RCC_PLL2CR_DIVPEN | RCC_PLL2CR_DIVQEN | RCC_PLL2CR_DIVREN );     /*Disable the post-dividers*/
    CLEAR_BIT(RCC->PLL2CR, RCC_PLL2CR_PLLON);   /* Disable the main PLL. */
    while((RCC->PLL2CR & RCC_PLL2CR_PLL2RDY) == RCC_PLL2CR_PLL2RDY );  /* Wait till PLL2 is ready */

    /* Configure PLL1 and PLL2 clock source */
    MODIFY_REG( RCC->RCK12SELR, RCC_RCK12SELR_PLL12SRC, RCC_RCK12SELR_PLL12SRC_1 );

    /* Configure the PLL2 multiplication and division factors. */
    MODIFY_REG( RCC->PLL2CFGR1, (RCC_PLL2CFGR1_DIVN | RCC_PLL2CFGR1_DIVM2) , ( (65U) | ( (2U) << 16U) ) );
    MODIFY_REG( RCC->PLL2CFGR2, (RCC_PLL2CFGR2_DIVP | RCC_PLL2CFGR2_DIVQ | RCC_PLL2CFGR2_DIVR), ( (1U) | ( (0U) <<8U ) | ( (0U) <<16U) )); \

    /* Configure the Fractional Divider */
    CLEAR_BIT(RCC->PLL2FRACR, RCC_PLL2FRACR_FRACLE); //Set FRACLE to 0
    /* In integer or clock spreading mode the application shall ensure that a 0 is loaded into the SDM */
    MODIFY_REG(RCC->PLL2FRACR, RCC_PLL2FRACR_FRACV, (uint32_t)(5120) << RCC_PLL2FRACR_FRACV_Pos);   /* Configure PLL  PLL2FRACV  in fractional mode*/
    SET_BIT(RCC->PLL2FRACR, RCC_PLL2FRACR_FRACLE); //Set FRACLE to 1

    CLEAR_BIT(RCC->PLL2CR, RCC_PLL2CR_SSCG_CTRL);

    /* Enable the PLL2. */
    SET_BIT(RCC->PLL2CR, RCC_PLL2CR_PLLON );
    while((RCC->PLL2CR & RCC_PLL2CR_PLL2RDY) !=  RCC_PLL2CR_PLL2RDY );/* Wait till PLL is ready */

    /*Enable the post-dividers*/
    SET_BIT(RCC->PLL2CR, RCC_PLL2CR_DIVPEN | RCC_PLL2CR_DIVQEN | RCC_PLL2CR_DIVREN );
        
//------- Configure PLL3 ---------------------------
    CLEAR_BIT(RCC->PLL3CR, RCC_PLL3CR_DIVPEN | RCC_PLL3CR_DIVQEN | RCC_PLL3CR_DIVREN ); /*Disable the post-dividers*/
    CLEAR_BIT(RCC->PLL3CR, RCC_PLL3CR_PLLON);   /* Disable the main PLL. */
    while((RCC->PLL3CR & RCC_PLL3CR_PLL3RDY) ==  RCC_PLL3CR_PLL3RDY );  /* Wait till PLL is ready */

    /* Configure PLL3 clock source */
    MODIFY_REG( RCC->RCK3SELR, RCC_RCK3SELR_PLL3SRC, RCC_RCK3SELR_PLL3SRC_1 );
    while( (RCC->RCK3SELR & RCC_RCK3SELR_PLL3SRCRDY) != RCC_RCK3SELR_PLL3SRCRDY );  /* Wait till PLL SOURCE is ready */

    /* Select PLL3 input reference frequency range */
    MODIFY_REG(RCC->PLL3CFGR1, RCC_PLL3CFGR1_IFRGE, RCC_PLL3CFGR1_IFRGE_1);

    /* Configure the PLL3 multiplication and division factors. */
    MODIFY_REG( RCC->PLL3CFGR1, (RCC_PLL3CFGR1_DIVN | RCC_PLL3CFGR1_DIVM3) , ( (plln - 1U) | ( (pllm - 1U) << 16U) ) );
    MODIFY_REG( RCC->PLL3CFGR2, (RCC_PLL3CFGR2_DIVP | RCC_PLL3CFGR2_DIVQ | RCC_PLL3CFGR2_DIVR),( (pllp - 1U) | ( (pllq - 1U) <<8U ) | ( (36U) <<16U) ));
                      
    /* Configure the Fractional Divider */
    CLEAR_BIT(RCC->PLL3FRACR, RCC_PLL3FRACR_FRACLE); //Set FRACLE to 0
    MODIFY_REG(RCC->PLL3FRACR, RCC_PLL3FRACR_FRACV,(uint32_t)(pllfracv) << RCC_PLL3FRACR_FRACV_Pos);    /* Configure PLL  PLL3FRACV  in fractional mode*/
    SET_BIT(RCC->PLL3FRACR, RCC_PLL3FRACR_FRACLE); //Set FRACLE to 1

    CLEAR_BIT(RCC->PLL3CR, RCC_PLL3CR_SSCG_CTRL);

    /* Enable the PLL3. */
    SET_BIT(RCC->PLL3CR, RCC_PLL3CR_PLLON );
    while((RCC->PLL3CR & RCC_PLL3CR_PLL3RDY) !=  RCC_PLL3CR_PLL3RDY );  /* Wait till PLL is ready */

    /* Enable the post-dividers */
    SET_BIT(RCC->PLL3CR, RCC_PLL3CR_DIVPEN | RCC_PLL3CR_DIVQEN | RCC_PLL3CR_DIVREN );

//---------- Configure PLL4 -----------------------
    CLEAR_BIT(RCC->PLL4CR, RCC_PLL4CR_DIVPEN | RCC_PLL4CR_DIVQEN | RCC_PLL4CR_DIVREN );/*Disable the post-dividers*/
    CLEAR_BIT(RCC->PLL4CR, RCC_PLL4CR_PLLON);     /* Disable the main PLL. */
    while((RCC->PLL4CR & RCC_PLL4CR_PLL4RDY) ==  RCC_PLL4CR_PLL4RDY );  /* Wait till PLL is ready */

    /* Configure PLL4 and PLL4 clock source */
    MODIFY_REG( RCC->RCK4SELR, RCC_RCK4SELR_PLL4SRC, RCC_RCK4SELR_PLL4SRC_1 );
    while( (RCC->RCK4SELR & RCC_RCK4SELR_PLL4SRCRDY) != RCC_RCK4SELR_PLL4SRCRDY );  /* Wait till PLL SOURCE is ready */

    /* Select PLL4 input reference frequency range */
    MODIFY_REG(RCC->PLL4CFGR1, RCC_PLL4CFGR1_IFRGE, RCC_PLL4CFGR1_IFRGE_0);

    /* Configure the PLL4 multiplication and division factors. */
    MODIFY_REG( RCC->PLL4CFGR1, (RCC_PLL4CFGR1_DIVN | RCC_PLL4CFGR1_DIVM4) , ( 98U | ( 3U << 16U) ) );
    MODIFY_REG( RCC->PLL4CFGR2, (RCC_PLL4CFGR2_DIVP | RCC_PLL4CFGR2_DIVQ | RCC_PLL4CFGR2_DIVR), ( 5U | ( 7U <<8U ) | ( 7U <<16U) ));
                               
    /* Configure the Fractional Divider */
    CLEAR_BIT(RCC->PLL4FRACR, RCC_PLL4FRACR_FRACLE);      //Set FRACLE to 0
    MODIFY_REG(RCC->PLL4FRACR, RCC_PLL4FRACR_FRACV,(uint32_t)(0) << RCC_PLL4FRACR_FRACV_Pos);     /* Do not use the fractional divider */
    SET_BIT(RCC->PLL4FRACR, RCC_PLL4FRACR_FRACLE); //Set FRACLE to 1

    CLEAR_BIT(RCC->PLL4CR, RCC_PLL4CR_SSCG_CTRL);

    /* Enable the PLL4. */
    SET_BIT(RCC->PLL4CR, RCC_PLL4CR_PLLON );    
    while((RCC->PLL4CR & RCC_PLL4CR_PLL4RDY) !=  RCC_PLL4CR_PLL4RDY );      /* Wait till PLL is ready */

    /* Enable PLL4P Clock output. */
    SET_BIT(RCC->PLL4CR, RCC_PLL4CR_DIVPEN | RCC_PLL4CR_DIVQEN | RCC_PLL4CR_DIVREN );

//-----------------------------------------
    /* Set MCU clock source */
    MODIFY_REG( RCC->MSSCKSELR, RCC_MSSCKSELR_MCUSSRC , RCC_MSSCKSELR_MCUSSRC_3 );
    while( (RCC->MSSCKSELR & RCC_MSSCKSELR_MCUSSRCRDY)  != RCC_MSSCKSELR_MCUSSRCRDY );  /* Wait till MCU is ready */

    /* Set MCU division factor */
    MODIFY_REG( RCC->MCUDIVR, RCC_MCUDIVR_MCUDIV , RCC_MCUDIVR_MCUDIV_0 );
    while( (RCC->MCUDIVR & RCC_MCUDIVR_MCUDIVRDY) != RCC_MCUDIVR_MCUDIVRDY );   /* Wait till MCU is ready */

    /* Set APB1 division factor */
    MODIFY_REG( RCC->APB1DIVR, RCC_APB1DIVR_APB1DIV , RCC_APB1DIVR_APB1DIV_1 );
    while((RCC->APB1DIVR & RCC_APB1DIVR_APB1DIVRDY) !=  RCC_APB1DIVR_APB1DIVRDY );  /* Wait till APB1 is ready */

    /* Set APB2 division factor */
    MODIFY_REG( RCC->APB2DIVR, RCC_APB2DIVR_APB2DIV , RCC_APB2DIVR_APB2DIV_1 ); 
    while((RCC->APB2DIVR & RCC_APB2DIVR_APB2DIVRDY) !=  RCC_APB2DIVR_APB2DIVRDY );  /* Wait till APB2 is ready */

    /* Set APB3 division factor */
    MODIFY_REG( RCC->APB3DIVR, RCC_APB3DIVR_APB3DIV , RCC_APB3DIVR_APB3DIV_1 );
    while((RCC->APB3DIVR & RCC_APB3DIVR_APB3DIVRDY) !=  RCC_APB3DIVR_APB3DIVRDY );  /* Wait till APB3 is ready */
}

void delay_short(volatile unsigned int n)
{
    while(n--){}
}


void delay(volatile unsigned int n)
{
    while(n--)
    {
        delay_short(0x7fff);
    }
}

// Main entry point - called from armcm_boot.c:ResetHandler()
void
armcm_main(void)
{
    SystemInit();       // Run SystemInit() and then restore VTOR

    // sys_stm32_clock_init(34, 2, 2, 17, 6826); /* 初始化M4内核时钟 */
    // clock_init(34, 2, 2, 17, 6826);
    // SCB->VTOR = (uint32_t)(0x00000000);
    // clock_setup();

    //-----------------------------------------------------
    gpio_out_setup(GPIO('B', 9),0);     // GPIOB_9设置为输出, 低电平; 关闭热床
    gpio_out_setup(GPIO('A', 11),1);    // GPIOA_11设置为输出, 高电平; 开启tf-card-led

    sched_main();
}

/* ADC functions on STM32MP1
 *
 * Copyright (C) 2022  lodge 'Connor <shilong.native@gmail.com>
 *
 * This file may be distributed under the terms of the GNU GPLv3 license.
 */

#include "board/irq.h"           // irq_save
#include "board/misc.h"          // timer_from_us
#include "command.h"             // shutdown
#include "compiler.h"            // ARRAY_SIZE
#include "generic/armcm_timer.h" // udelay
#include "gpio.h"                // gpio_adc_setup
#include "internal.h"            // GPIO
#include "sched.h"               // sched_shutdown
#include "stm32mp1xx.h"          // stm32mp1xx

// Number of samples is 2^OVERSAMPLES_EXPONENT (exponent can be 0-10)
#define OVERSAMPLES_EXPONENT 3
#define OVERSAMPLES (1 << OVERSAMPLES_EXPONENT)

// LDORDY registers are missing from CMSIS (only available on revision V!)
#define ADC_ISR_LDORDY_Pos (12U)
#define ADC_ISR_LDORDY_Msk (0x1UL << ADC_ISR_LDORDY_Pos)
#define ADC_ISR_LDORDY ADC_ISR_LDORDY_Msk

DECL_CONSTANT("ADC_MAX", 4095);

// GPIOs like A0_C are not covered!
// This always gives the pin connected to the positive channel
static const int8_t adc_pins[] = {
    // ADC1
    -1,            // ANA0                 ADC12_INP0
    -1,            // ANA1                 ADC12_INP1
    GPIO('F', 11), //          ADC1_INP2
    GPIO('A', 6),  //          ADC12_INP3
    GPIO('C', 4),  //          ADC12_INP4
    GPIO('B', 1),  //          ADC12_INP5
    GPIO('F', 12), //          ADC1_INP6
    GPIO('A', 7),  //          ADC12_INP7
    GPIO('C', 5),  //          ADC12_INP8
    GPIO('B', 0),  //          ADC12_INP9
    GPIO('C', 0),  //          ADC12_INP10
    GPIO('C', 1),  //          ADC12_INP11
    GPIO('C', 2),  //          ADC1_INP12
    GPIO('C', 3),  //          ADC1_INP13
    GPIO('A', 2),  //          ADC1_INP14
    GPIO('A', 3),  //          ADC1_INP15
    GPIO('A', 0),  //          ADC1_INP16
    GPIO('A', 1),  //          ADC1_INP17
    GPIO('A', 4),  //          ADC12_INP18
    GPIO('A', 5),  //          ADC12_INP19

    // ADC2
    -1,            // PA0_C                ADC12_INP0
    -1,            // PA1_C                ADC12_INP1
    GPIO('F', 13), //          ADC2_INP2
    GPIO('A', 6),  //          ADC12_INP3
    GPIO('C', 4),  //          ADC12_INP4
    GPIO('B', 1),  //          ADC12_INP5
    GPIO('F', 14), //          ADC2_INP6
    GPIO('A', 7),  //          ADC12_INP7
    GPIO('C', 5),  //          ADC12_INP8
    GPIO('B', 0),  //          ADC12_INP9
    GPIO('C', 0),  //          ADC12_INP10
    GPIO('C', 1),  //           ADC12_INP11
    -1,            //           ADC1_INP12
    -1,            //           ADC1_INP13
    -1,            //           ADC1_INP14
    -1,            //           ADC1_INP15
    -1,            //           ADC2_INP16
    -1,            //           ADC2_INP17
    GPIO('A', 4),  //          ADC12_INP18
    GPIO('A', 5),  //          ADC12_INP19
};

// ADC timing:
// ADC clock=30Mhz, Tconv=6.5, Tsamp=64.5, total=2.3666us*OVERSAMPLES

struct gpio_adc
gpio_adc_setup(uint32_t pin)
{
    // Find pin in adc_pins table
    int chan;
    for (chan = 0;; chan++)
    {
        if (chan >= ARRAY_SIZE(adc_pins))
            shutdown("Not a valid ADC pin");
        if (adc_pins[chan] == pin)
            break;
    }

    // Determine which ADC block to use, enable peripheral clock
    // (SYSCLK 480Mhz) /HPRE(2) /CKMODE divider(4) /additional divider(2)
    // (ADC clock 30Mhz)
    RCC->MC_AHB2ENSETR = RCC_MC_AHB2ENSETR_ADC12EN;
    MODIFY_REG(ADC12_COMMON->CCR, ADC_CCR_CKMODE_Msk,
               0b11 << ADC_CCR_CKMODE_Pos);

    ADC_TypeDef *adc;
    if (chan >= 20)
    {
        adc = ADC2;
    }
    else
    {
        adc = ADC1;
    }
    chan %= 20;

    // Enable the ADC
    if (!(adc->CR & ADC_CR_ADEN))
    {
        // Pwr
        // Exit deep power down
        MODIFY_REG(adc->CR, ADC_CR_DEEPPWD_Msk, 0);
        // Switch on voltage regulator
        adc->CR |= ADC_CR_ADVREGEN;
        while (!(adc->ISR & ADC_ISR_LDORDY))
            ;

        // Set Boost mode for 25Mhz < ADC clock <= 50Mhz
        MODIFY_REG(adc->CR, ADC_CR_BOOST_Msk, 0b1 << ADC_CR_BOOST_Pos);

        // Calibration
        // Set calibration mode to Single ended (not differential)
        MODIFY_REG(adc->CR, ADC_CR_ADCALDIF_Msk, 0);
        // Enable linearity calibration
        MODIFY_REG(adc->CR, ADC_CR_ADCALLIN_Msk, ADC_CR_ADCALLIN);
        // Start the calibration
        MODIFY_REG(adc->CR, ADC_CR_ADCAL_Msk, ADC_CR_ADCAL);
        while (adc->CR & ADC_CR_ADCAL)
            ;

        // Enable ADC
        // "Clear the ADRDY bit in the ADC_ISR register by writing ‘1’"
        adc->ISR |= ADC_ISR_ADRDY;
        adc->CR |= ADC_CR_ADEN;
        while (!(adc->ISR & ADC_ISR_ADRDY))
            ;

        // Set 64.5 ADC clock cycles sample time for every channel
        // (Reference manual pg.940)
        uint32_t aticks = 0b101; // 64.5 ADC clock
        // uint32_t aticks = 0b111;    //810.5 ADC clock
        //  Channel 0-9
        adc->SMPR1 = (aticks | (aticks << 3) | (aticks << 6) | (aticks << 9) | (aticks << 12) | (aticks << 15) | (aticks << 18) | (aticks << 21) | (aticks << 24) | (aticks << 27));
        // Channel 10-19
        adc->SMPR2 = (aticks | (aticks << 3) | (aticks << 6) | (aticks << 9) | (aticks << 12) | (aticks << 15) | (aticks << 18) | (aticks << 21) | (aticks << 24) | (aticks << 27));
        // Disable Continuous Mode
        MODIFY_REG(adc->CFGR, ADC_CFGR_CONT_Msk, 0);
        // Set to 12 bit
        MODIFY_REG(adc->CFGR, ADC_CFGR_RES_Msk, 0b010 << ADC_CFGR_RES_Pos);
        // Set hardware oversampling
        // MODIFY_REG(adc->CFGR2, ADC_CFGR2_ROVSE_Msk, ADC_CFGR2_ROVSE);
        // MODIFY_REG(adc->CFGR2, ADC_CFGR2_OVSR_Msk,
        //     (OVERSAMPLES - 1) << ADC_CFGR2_OVSR_Pos);
        // MODIFY_REG(adc->CFGR2, ADC_CFGR2_OVSS_Msk,
        //     OVERSAMPLES_EXPONENT << ADC_CFGR2_OVSS_Pos);
    }
    gpio_peripheral(pin, GPIO_ANALOG, 0);
    // Preselect (connect) channel
    adc->PCSEL |= (1 << chan);
    return (struct gpio_adc){.adc = adc, .chan = chan};
}

// Try to sample a value. Returns zero if sample ready, otherwise
// returns the number of clock ticks the caller should wait before
// retrying this function.
uint32_t gpio_adc_sample(struct gpio_adc g)
{
    ADC_TypeDef *adc = g.adc;
    // Conversion ready
    // EOC flag is cleared by hardware when reading DR
    // the channel condition only works if this ist the only channel
    // on the sequence and length set to 1 (ADC_SQR1_L = 0000)
    if (adc->ISR & ADC_ISR_EOC && adc->SQR1 == (g.chan << 6))
        return 0;
    // Conversion started but not ready or wrong channel
    if (adc->CR & ADC_CR_ADSTART)
        return timer_from_us(10);
    // Start sample
    adc->SQR1 = (g.chan << 6);
    adc->CR |= ADC_CR_ADSTART;
    // Should take 2.3666us, add 1us for clock synchronisation etc.
    return timer_from_us(1 + 2.3666 * OVERSAMPLES);
}

// Read a value; use only after gpio_adc_sample() returns zero
uint16_t gpio_adc_read(struct gpio_adc g)
{
    ADC_TypeDef *adc = g.adc;
    return adc->DR;
}

// Cancel a sample that may have been started with gpio_adc_sample()
void gpio_adc_cancel_sample(struct gpio_adc g)
{
    ADC_TypeDef *adc = g.adc;
    irqstatus_t flag = irq_save();
    // ADSTART is not as long true as SR_STRT on stm32f4
    if ((adc->CR & ADC_CR_ADSTART || adc->ISR & ADC_ISR_EOC) && adc->SQR1 == (g.chan << 6))
        gpio_adc_read(g);
    irq_restore(flag);
}

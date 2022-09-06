/* STM32MP1 serial
 *
 * Copyright (C) 2022  lodge 'Connor <shilong.native@gmail.com>
 *
 * This file may be distributed under the terms of the GNU GPLv3 license.
 */

#include "autoconf.h"         // CONFIG_SERIAL_BAUD
#include "board/armcm_boot.h" // armcm_enable_irq
#include "board/serial_irq.h" // serial_rx_byte
#include "command.h"          // DECL_CONSTANT_STR
#include "internal.h"         // enable_pclock
#include "sched.h"            // DECL_INIT

// Select the configured serial port
#if CONFIG_STM32MP1_SERIAL_UART8
DECL_CONSTANT_STR("RESERVE_PINS_serial", "PE0,PE1");
#define GPIO_Rx GPIO('E', 0)
#define GPIO_Tx GPIO('E', 1)
#define USARTx UART8
#define USARTx_IRQn UART8_IRQn
#endif

#define CR1_FLAGS (USART_CR1_UE | USART_CR1_RE | USART_CR1_TE | USART_CR1_RXNEIE)

void USARTx_IRQHandler(void)
{
    uint32_t isr = USARTx->ISR;
    if (isr & (USART_ISR_RXNE_RXFNE | USART_ISR_ORE))
        serial_rx_byte(USARTx->RDR);
    // USART_ISR_TXE_TXFNF only works with Fifo mode disabled
    if (isr & USART_ISR_TXE_TXFNF && USARTx->CR1 & USART_CR1_TXEIE)
    {
        uint8_t data;
        int ret = serial_get_tx_byte(&data);
        if (ret)
            USARTx->CR1 = CR1_FLAGS;
        else
            USARTx->TDR = data;
    }
}

void serial_enable_tx_irq(void)
{
    USARTx->CR1 = CR1_FLAGS | USART_CR1_TXEIE;
}

void UART_Send(uint8_t ch)
{
    while ((USARTx->ISR & 0X40) == 0)
        ;
    USARTx->TDR = (uint8_t)ch;
}

void serial_init(void)
{
    enable_pclock((uint32_t)USARTx);

    /* 设置UART4时钟源=PLL4Q=74.25MHz  */
    WRITE_REG(RCC->UART78CKSELR, (0x02));

    USARTx->BRR = (0x22C); // baud:115200
    USARTx->CR1 = (USART_CR1_UE | USART_CR1_RE | USART_CR1_TE | USART_CR1_RXNEIE);

    armcm_enable_irq(USARTx_IRQHandler, USARTx_IRQn, 0);

    gpio_peripheral(GPIO_Rx, GPIO_FUNCTION(8), 1);
    gpio_peripheral(GPIO_Tx, GPIO_FUNCTION(8), 0);
}
DECL_INIT(serial_init);

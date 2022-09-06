/* SPI functions on STM32MP1
 *
 * Copyright (C) 2022  lodge 'Connor <shilong.native@gmail.com>
 *
 * This file may be distributed under the terms of the GNU GPLv3 license.
 */

#include "board/io.h" // readb, writeb
#include "command.h"  // shutdown
#include "gpio.h"     // spi_setup
#include "internal.h" // gpio_peripheral
#include "sched.h"    // sched_shutdown

struct spi_info
{
    SPI_TypeDef *spi;
    uint16_t miso_pin, mosi_pin, sck_pin, function;
};

DECL_ENUMERATION("spi_bus", "spi1", __COUNTER__);
DECL_CONSTANT_STR("BUS_PINS_spi1", "PZ1,PZ2,PZ0");

DECL_ENUMERATION("spi_bus", "spi2", __COUNTER__);
DECL_CONSTANT_STR("BUS_PINS_spi2", "PI2,PI3,PB10");

#ifdef SPI3
DECL_ENUMERATION("spi_bus", "spi3", __COUNTER__);
DECL_CONSTANT_STR("BUS_PINS_spi3", "PB4,PB5,PB3");

DECL_ENUMERATION("spi_bus", "spi3a", __COUNTER__);
DECL_CONSTANT_STR("BUS_PINS_spi3a", "PC11,PC12,PC10");
#endif

#ifdef SPI4
DECL_ENUMERATION("spi_bus", "spi4", __COUNTER__);
DECL_CONSTANT_STR("BUS_PINS_spi4", "PE13,PE14,PE12");
#endif

static const struct spi_info spi_bus[] = {
    {SPI1, GPIO('Z', 1), GPIO('Z', 2), GPIO('Z', 0), GPIO_FUNCTION(5)},
    {SPI2, GPIO('I', 2), GPIO('I', 3), GPIO('B', 10), GPIO_FUNCTION(5)},
#ifdef SPI3
    {SPI3, GPIO('B', 4), GPIO('B', 5), GPIO('B', 3), GPIO_FUNCTION(6)},
    {SPI3, GPIO('C', 11), GPIO('C', 12), GPIO('C', 10), GPIO_FUNCTION(6)},
#endif
#ifdef SPI4
    {SPI4, GPIO('E', 13), GPIO('E', 14), GPIO('E', 12), GPIO_FUNCTION(5)},
#endif
};

struct spi_config
spi_setup(uint32_t bus, uint8_t mode, uint32_t rate)
{
    if (bus >= ARRAY_SIZE(spi_bus))
        shutdown("Invalid spi bus");

    // Enable SPI
    SPI_TypeDef *spi = spi_bus[bus].spi;
    if (!is_enabled_pclock((uint32_t)spi))
    {
        enable_pclock((uint32_t)spi);
        gpio_peripheral(spi_bus[bus].miso_pin, spi_bus[bus].function, 1);
        gpio_peripheral(spi_bus[bus].mosi_pin, spi_bus[bus].function, 0);
        gpio_peripheral(spi_bus[bus].sck_pin, spi_bus[bus].function, 0);
    }

    // Calculate CR1 register
    uint32_t pclk = get_pclock_frequency((uint32_t)spi);
    uint32_t div = 0;
    while ((pclk >> (div + 1)) > rate && div < 7)
        div++;

    uint32_t cr1 = SPI_CR1_SPE;
    spi->CFG1 |= (div << SPI_CFG1_MBR_Pos) | (7 << SPI_CFG1_DSIZE_Pos);
    CLEAR_BIT(spi->CFG1, SPI_CFG1_CRCSIZE);
    spi->CFG2 |= ((mode << SPI_CFG2_CPHA_Pos) | SPI_CFG2_MASTER | SPI_CFG2_SSM | SPI_CFG2_AFCNTR | SPI_CFG2_SSOE);
    spi->CR1 |= SPI_CR1_SSI;

    return (struct spi_config){.spi = spi, .spi_cr1 = cr1};
}

void spi_prepare(struct spi_config config)
{
}

void spi_transfer(struct spi_config config, uint8_t receive_data,
                  uint8_t len, uint8_t *data)
{
    uint8_t rdata = 0;
    SPI_TypeDef *spi = config.spi;

    MODIFY_REG(spi->CR2, SPI_CR2_TSIZE, len);
    // Enable SPI and start transfer, these MUST be set in this sequence
    SET_BIT(spi->CR1, SPI_CR1_SPE);
    SET_BIT(spi->CR1, SPI_CR1_CSTART);

    while (len--)
    {
        writeb((void *)&spi->TXDR, *data);
        while ((spi->SR & (SPI_SR_RXWNE | SPI_SR_RXPLVL)) == 0)
            ;
        rdata = readb((void *)&spi->RXDR);

        if (receive_data)
        {
            *data = rdata;
        }
        data++;
    }

    while ((spi->SR & SPI_SR_EOT) == 0)
        ;

    // Clear flags and disable SPI
    SET_BIT(spi->IFCR, 0xFFFFFFFF);
    CLEAR_BIT(spi->CR1, SPI_CR1_SPE);
}

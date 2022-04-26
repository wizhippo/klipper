// Watchdog handler on STM32
//
// Copyright (C) 2019  Kevin O'Connor <kevin@koconnor.net>
//
// This file may be distributed under the terms of the GNU GPLv3 license.

#include "autoconf.h" // CONFIG_MACH_STM32H7
#include "internal.h" // IWDG
#include "sched.h" // DECL_TASK

#if CONFIG_MACH_STM32H7 // stm32h7 libraries only define IWDG1 and IWDG2
#define IWDG IWDG1

void
watchdog_reset(void)
{
    IWDG->KR = 0xAAAA;
}
DECL_TASK(watchdog_reset);

void
watchdog_init(void)
{
    IWDG->KR = 0x5555;
    IWDG->PR = 0;
    IWDG->RLR = 0x0FFF; // 410-512ms timeout (depending on stm32 chip)
    IWDG->KR = 0xCCCC;
}
DECL_INIT(watchdog_init);

#endif

#if CONFIG_MACH_STM32MP1 

#define IWDG WWDG1

void
watchdog_reset(void)
{
    IWDG->CR = 0xAAAA;
}
DECL_TASK(watchdog_reset);

void
watchdog_init(void)
{
    IWDG->CR = 0x5555;
    IWDG->SR = 0;
    IWDG->CFR = 0x0FFF; // 410-512ms timeout (depending on stm32 chip)
    IWDG->CR = 0xCCCC;
}
DECL_INIT(watchdog_init);

#endif

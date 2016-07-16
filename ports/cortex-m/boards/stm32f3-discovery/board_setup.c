/*
 * Copyright (c) 2015, Tido Klaassen. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. No personal names or organizations' names associated with the
 *    Atomthreads project may be used to endorse or promote products
 *    derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE ATOMTHREADS PROJECT AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED
 * TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE PROJECT OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include <stdbool.h>

#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/usart.h>
#include <libopencm3/cm3/systick.h>
#include <libopencm3/cm3/cortex.h>
#include <libopencm3/cm3/nvic.h>

#include "atomport.h"

#define DBG_USART   USART1

/**
 * Set up USART2.
 * This one is connected via the virtual serial port on the Nucleo Board
 */
static void usart_setup(uint32_t baud)
{
  /*
    rcc_periph_clock_enable(RCC_GPIOC);
    rcc_periph_clock_enable(RCC_USART1);

    usart_disable(DBG_USART);

    gpio_mode_setup(GPIOC, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO5 | GPIO4);
    gpio_set_af(GPIOC, GPIO_AF7, GPIO5);

    usart_set_baudrate(DBG_USART, baud);
    usart_set_databits(DBG_USART, 8);
    usart_set_stopbits(DBG_USART, USART_STOPBITS_1);
    usart_set_parity(DBG_USART, USART_PARITY_NONE);
    usart_set_mode(DBG_USART, USART_MODE_TX_RX);
    usart_set_flow_control(DBG_USART, USART_FLOWCONTROL_NONE);

    usart_enable(DBG_USART);
    */
}

/**
 * initialise and start SysTick counter. This will trigger the
 * sys_tick_handler() periodically once interrupts have been enabled
 * by archFirstThreadRestore()
 */
static void systick_setup(void)
{
    systick_set_frequency(SYSTEM_TICKS_PER_SEC, 8000000);

    systick_interrupt_enable();

    systick_counter_enable();
}

/**
 * Set up the core clock to something other than the internal 16MHz PIOSC.
 * Make sure that you use the same clock frequency in  systick_setup().
 */
static void clock_setup(void)
{
    rcc_osc_on(HSE);
    rcc_wait_for_osc_ready(HSE);
    
    rcc_set_pll_source(RCC_CFGR_PLLSRC_HSE_PREDIV);
    rcc_set_pll_multiplier(9);
    rcc_osc_on(RCC_PLL);
    rcc_wait_for_osc_ready(RCC_PLL);
    
    rcc_set_hpre(1);
    rcc_set_ppre2(1);
    rcc_set_ppre1(2);
    rcc_usb_prescale_1_5();
    rcc_set_sysclk_source(RCC_CFGR_SW_PLL);
    rcc_wait_for_sysclk_status(RCC_PLL);
    
    rcc_ahb_frequency   = 72000000;
    rcc_apb1_frequency  = 36000000;
    rcc_apb2_frequency  = 72000000;
}

/**
 * Set up user LED and provide function for toggling it. This is for
 * use by the test suite programs
 */
static void test_led_setup(void)
{
    /* LED is connected to GPIO5 on port A */
    rcc_periph_clock_enable(RCC_GPIOE);
    gpio_mode_setup(GPIOE, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO14);
    gpio_set(GPIOE, GPIO14);
}

void test_led_toggle(void)
{
    gpio_toggle(GPIOE, GPIO14);
}

/**
 * Callback from your main program to set up the board's hardware before
 * the kernel is started.
 */
int board_setup(void)
{
    /* Disable interrupts. This makes sure that the sys_tick_handler will
     * not be called before the first thread has been started.
     * Interrupts will be enabled by archFirstThreadRestore().
     */
    cm_mask_interrupts(true);

    /* configure system clock, user LED and UART */
    clock_setup();
    test_led_setup();
    usart_setup(115200);

    /* initialise SysTick counter */
    systick_setup();

    /* Set exception priority levels. Make PendSv the lowest priority and
     * SysTick the second to lowest
     */
    nvic_set_priority(NVIC_PENDSV_IRQ, 0xFF);
    nvic_set_priority(NVIC_SYSTICK_IRQ, 0xFE);

    return 0;
}

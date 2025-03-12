/*
 * The MIT License (MIT)
 *
 * Copyright (c) 2025, Ennebi Elettronica (https://ennebielettronica.com)
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 *
 */

#include "bsp/board_api.h"
#include "project.h"

//--------------------------------------------------------------------+
// Board porting API
//--------------------------------------------------------------------+

#if CFG_TUSB_OS  == OPT_OS_NONE
volatile uint32_t system_ticks = 0u;
void SysTick_Handler(void)
{
  system_ticks++;
}

uint32_t board_millis(void)
{
  return system_ticks;
}
#endif

void board_init(void)
{
    CySysTickStart();  /* Configure and start system tick interrupt */
    CySysTickSetCallback(0, SysTick_Handler);

    CyGlobalIntEnable; /* Enable global interrupts. */
}

void board_led_write(bool state)
{
    PIN_LED_Write(state);
}

uint32_t board_button_read(void)
{
  return PIN_BUTTON_Read();
}

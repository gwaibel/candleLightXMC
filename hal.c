/*

The MIT License (MIT)

Copyright (c) 2016 Hubert Denkmair

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE.

*/

#include "xmc_gpio.h"
#include "hal.h"


static uint32_t ms_tick;

// TODO: install timer tick ISR
/**
  * @brief This function is called to increment  a global variable "uwTick"
  *        used as application time base.
  * @note In the default implementation, this variable is incremented each 1ms
  *       in Systick ISR.
  * @retval None
  */
void HAL_IncTick(void)
{
   ms_tick++;
}

/**
  * @brief  Provides a tick value in millisecond.
  * @retval tick value
  */
uint32_t HAL_GetTick(void)
{
  return ms_tick;
}

/**
  * @brief  Provides a tick value in microsecond.
  * @retval tick value
  */
uint64_t HAL_GetTick_us(void)
{
   return (uint64_t)ms_tick * 1000u;
}

/**
  * @brief  Set a GPIO pin value.
  * @param GPIO_Port    GPIO port (XMC_GPIO_PORTx)
  * @param GPIO_Pin     GPIO pin (0...31)
  * @param PinState     Value (GPIO_PIN_(RE)SET)
  */
void HAL_GPIO_WritePin(void* GPIO_Port, uint16_t GPIO_Pin, GPIO_PinState PinState)
{
   XMC_GPIO_SetOutputLevel((XMC_GPIO_PORT_t*)GPIO_Port, GPIO_Pin,
                           ((PinState == GPIO_PIN_RESET) ? 0 : 1));
}

void HAL_GPIO_InitOutPin(void* GPIO_Port, uint16_t GPIO_Pin, GPIO_PinState PinState)
{
   XMC_GPIO_CONFIG_t config;

   config.mode = XMC_GPIO_MODE_OUTPUT_PUSH_PULL;
   config.output_level = (PinState == GPIO_PIN_RESET) ? 0 : 1;
   config.output_strength = XMC_GPIO_OUTPUT_STRENGTH_MEDIUM;
   XMC_GPIO_Init((XMC_GPIO_PORT_t*)GPIO_Port, GPIO_Pin, &config);
}



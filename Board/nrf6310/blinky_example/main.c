/* Copyright (c) 2009 Nordic Semiconductor. All Rights Reserved.
 *
 * The information contained herein is property of Nordic Semiconductor ASA.
 * Terms and conditions of usage are described in detail in NORDIC
 * SEMICONDUCTOR STANDARD SOFTWARE LICENSE AGREEMENT.
 *
 * Licensees are granted free, non-transferable use of the information. NO
 * WARRANTY of ANY KIND is provided. This heading must NOT be removed from
 * the file.
 *
 */

/** @file
 *
 * @defgroup blinky_example_main main.c
 * @{
 * @ingroup blinky_example
 * @brief Blinky Example Application main file.
 *
 */

#include <stdbool.h>
#include <stdint.h>
#include "nrf_delay.h"
#include "nrf_gpio.h"
#include "boards.h"

#define led1 24
#define led2 25
/**
 * @brief Function for application main entry.
 */
int main(void)
{
    // Configure LED-pins as outputs.
    nrf_gpio_cfg_output(led1);
    //nrf_gpio_cfg_output(led2);
    nrf_gpio_pin_set(led1);
    //nrf_gpio_pin_clear(led2);
    // LED 0 and LED 1 blink alternately.
    while (true)
    {
        nrf_gpio_pin_set(led1);
        //nrf_gpio_pin_set(led2);

        nrf_delay_ms(500);

        //nrf_gpio_pin_clear(led1);
       //nrf_gpio_pin_clear(led2);

        //nrf_delay_ms(500);
    }
}
/** @} */

/** @file
* @brief Example GPIOTE project.
* @defgroup nrf_app_gpiote_example Example Template
*
*/

#include <stdbool.h>
#include "SEGGER_RTT.h"
#include "nrf_gpio.h"
#include "app_gpiote.h"
#include "boards.h"

#define BUTTON_1 20

#define APP_GPIOTE_MAX_USERS    1 /** Change this to match amount ot GPIOTE users, app_button uses one */

app_gpiote_user_id_t m_app_gpiote_my_id;

void app_error_handler(uint32_t error_code, uint32_t line_num, const uint8_t * p_file_name)
{
    NVIC_SystemReset();
}

void gpiote_event_handler(uint32_t event_pins_low_to_high, uint32_t event_pins_high_to_low)
{
    if (event_pins_high_to_low & (1 << BUTTON_1))
    {
        SEGGER_RTT_WriteString(0, "nrf_gpio_pin_set.\n\n");//这边可以更据自己的情况进行替换
    }
    if (event_pins_low_to_high & (1 << BUTTON_1))
    {
        SEGGER_RTT_WriteString(0, "nrf_gpio_pin_clear.\n\n");//这边可以更据自己的情况进行替换
    }
}

/**
 * Initialize the button GPIO pins to generate interrupts on push.
 */
static void buttons_init()
{
    uint32_t err_code;
    

    uint32_t   low_to_high_bitmask = (1 << BUTTON_1);
    uint32_t   high_to_low_bitmask = (1 << BUTTON_1);
    
    // Configure BUTTON1 with SENSE enabled
    nrf_gpio_cfg_sense_input(BUTTON_1, NRF_GPIO_PIN_PULLUP, NRF_GPIO_PIN_SENSE_LOW);
    
    APP_GPIOTE_INIT(APP_GPIOTE_MAX_USERS);   
    
    err_code = app_gpiote_user_register(&m_app_gpiote_my_id, 
                                        low_to_high_bitmask, 
                                        high_to_low_bitmask, 
                                        gpiote_event_handler);
    APP_ERROR_CHECK(err_code);
    
    err_code = app_gpiote_user_enable(m_app_gpiote_my_id);
    APP_ERROR_CHECK(err_code);
}



/**
 * @brief Function for application main entry.
 */
int main(void)
{   
    SEGGER_RTT_WriteString(0, "\033[2JBY libra .\n\n");
    buttons_init();
    while(true)
    {
        //power_manage();
    }
}


/** @} */
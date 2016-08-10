/* Copyright (c) 2014 Nordic Semiconductor. All Rights Reserved.
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

/**@file
 * @defgroup spi_master_example_with_slave_main main.c
 * @{
 * @ingroup spi_master_example
 *
 * @brief SPI master example application to be used with the SPI slave example application.
 */

#include <string.h>
#include "app_error.h"
#include "nrf_delay.h"
#include "boards.h"
#include "radio.h"
#include "SEGGER_RTT.h"


#define led0   25
#define led1   26
#define led2   27
#define ledstart led0
#define ledstop  led1
#define RX_TIMEOUT_VALUE                            5000
#define BUFFER_SIZE                                 64 // Define the payload size here 

#define TX_OUTPUT_POWER                             14

#define LORA_BANDWIDTH                              0         // [0: 125 kHz,
                                                              //  1: 250 kHz,
                                                              //  2: 500 kHz,
                                                              //  3: Reserved]
#define LORA_SPREADING_FACTOR                       7         // [SF7..SF12]
#define LORA_CODINGRATE                             1         // [1: 4/5,
                                                              //  2: 4/6,
                                                              //  3: 4/7,
                                                              //  4: 4/8]
#define LORA_PREAMBLE_LENGTH                        8         // Same for Tx and Rx
#define LORA_SYMBOL_TIMEOUT                         5         // Symbols
#define LORA_FIX_LENGTH_PAYLOAD_ON                  false
#define LORA_IQ_INVERSION_ON                        false

#define RF_FREQUENCY                                869800000 // Hz

const uint8_t PingMsg[] = "PING";
const uint8_t PongMsg[] = "PONG";

uint16_t BufferSize = BUFFER_SIZE;
uint8_t Buffer[BUFFER_SIZE];                        // Change Buffer size in radio.h as well

States_t State = LOWPOWER;

int8_t RssiValue = 0;
int8_t SnrValue = 0;

/**@brief Function for error handling, which is called when an error has occurred. 
 *
 * @param[in] error_code  Error code supplied to the handler.
 * @param[in] line_num    Line number where the handler is called.
 * @param[in] p_file_name Pointer to the file name. 
 */
void app_error_handler(uint32_t error_code, uint32_t line_num, const uint8_t * p_file_name)
{
    //Set LED2 high to indicate that error has occurred.
    nrf_gpio_pin_toggle(led2);
    SEGGER_RTT_printf(0,"Died at %d line with error code %d in file %s\n",line_num,error_code,p_file_name);
    for (;;)
    {
        //No implementation needed.
    }
}

/**@brief Function for application main entry. Does not return. */
int main(void)
{
    SEGGER_RTT_WriteString(0, "Ping Pong Start!\n");

    //Configure all LEDs as outputs. 
    nrf_gpio_range_cfg_output(ledstart, ledstop);
        
    bool isMaster = true;
    uint8_t i;

    Radio.Init( );

    Radio.SetChannel( RF_FREQUENCY );

    Radio.SetTxConfig( MODEM_LORA, TX_OUTPUT_POWER, 0, LORA_BANDWIDTH,
                                   LORA_SPREADING_FACTOR, LORA_CODINGRATE,
                                   LORA_PREAMBLE_LENGTH, LORA_FIX_LENGTH_PAYLOAD_ON,
                                   true, 0, 0, LORA_IQ_INVERSION_ON, 3000  );
    
    Radio.SetRxConfig( MODEM_LORA, LORA_BANDWIDTH, LORA_SPREADING_FACTOR,
                                   LORA_CODINGRATE, 0, LORA_PREAMBLE_LENGTH,
                                   LORA_SYMBOL_TIMEOUT, LORA_FIX_LENGTH_PAYLOAD_ON,
                                   0, true, 0, 0, LORA_IQ_INVERSION_ON, true );

    Radio.Rx( RX_TIMEOUT_VALUE );

    SEGGER_RTT_WriteString(0,"Entering Main Loop\n");

    while( 1 )
    {
        Radio.AppScheduler();
        switch( State )
        {
        case RX:
            SEGGER_RTT_WriteString(0,"**********************************\n");
            SEGGER_RTT_WriteString(0,"Receive Done State!!!!!!!!!!!!!!!!\n");
            SEGGER_RTT_WriteString(0,"**********************************\n");
            if( isMaster == true )
            {
                if( BufferSize > 0 )
                {
                    if( strncmp( ( const char* )Buffer, ( const char* )PongMsg, 4 ) == 0 )
                    {
                        // Indicates on a LED that the received frame is a PONG
                        nrf_gpio_pin_toggle(led1);

                        // Send the next PING frame            
                        Buffer[0] = 'P';
                        Buffer[1] = 'I';
                        Buffer[2] = 'N';
                        Buffer[3] = 'G';
                        // We fill the buffer with numbers for the payload 
                        for( i = 4; i < BufferSize; i++ )
                        {
                            Buffer[i] = i - 4;
                        }
                        nrf_delay_ms( 1 ); 
                        Radio.Send( Buffer, BufferSize );
                    }
                    else if( strncmp( ( const char* )Buffer, ( const char* )PingMsg, 4 ) == 0 )
                    { // A master already exists then become a slave
                        isMaster = false;
                        nrf_gpio_pin_toggle(led1);
                        Radio.Rx( RX_TIMEOUT_VALUE );
                    }
                    else // valid reception but neither a PING or a PONG message
                    {    // Set device as master ans start again
                        isMaster = true;
                        Radio.Rx( RX_TIMEOUT_VALUE );
                    }
                }
            }
            else
            {
                if( BufferSize > 0 )
                {
                    if( strncmp( ( const char* )Buffer, ( const char* )PingMsg, 4 ) == 0 )
                    {
                        // Indicates on a LED that the received frame is a PING
                        nrf_gpio_pin_toggle(led1);

                        // Send the reply to the PONG string
                        Buffer[0] = 'P';
                        Buffer[1] = 'O';
                        Buffer[2] = 'N';
                        Buffer[3] = 'G';
                        // We fill the buffer with numbers for the payload 
                        for( i = 4; i < BufferSize; i++ )
                        {
                            Buffer[i] = i - 4;
                        }
                        nrf_delay_ms( 1 );
                        Radio.Send( Buffer, BufferSize );
                    }
                    else // valid reception but not a PING as expected
                    {    // Set device as master and start again
                        isMaster = true;
                        Radio.Rx( RX_TIMEOUT_VALUE );
                    }   
                }
            }
            State = LOWPOWER;
            break;
        case TX:
            SEGGER_RTT_WriteString(0,"**********************************\n");
            SEGGER_RTT_WriteString(0,"TXDONE State!!!!!!!!!!!!!!!!!!!!\n");
            SEGGER_RTT_WriteString(0,"**********************************\n");
            // Indicates on a LED that we have sent a PING [Master]
            // Indicates on a LED that we have sent a PONG [Slave]
            nrf_gpio_pin_toggle(led0);
            Radio.Rx( RX_TIMEOUT_VALUE );
            State = LOWPOWER;
            break;
        case RX_TIMEOUT:
        case RX_ERROR:
            SEGGER_RTT_WriteString(0,"RX_ERROR/RX_TIMEOUT\n");
            if( isMaster == true )
            {
                SEGGER_RTT_WriteString(0,"I am the master!\n");
                // Send the next PING frame
                Buffer[0] = 'P';
                Buffer[1] = 'I';
                Buffer[2] = 'N';
                Buffer[3] = 'G';
                for( i = 4; i < BufferSize; i++ )
                {
                    Buffer[i] = i - 4;
                }
                nrf_delay_ms( 1 ); 
                Radio.Send( Buffer, BufferSize );
            }
            else
            {
                Radio.Rx( RX_TIMEOUT_VALUE );
            }
            State = LOWPOWER;
            break;
        case TX_TIMEOUT:
            SEGGER_RTT_WriteString(0,"TX_TIMEOUT\n");
            Radio.Rx( RX_TIMEOUT_VALUE );
            State = LOWPOWER;
            break;
        case LOWPOWER:
        default:
            // Set low power
            break;
        }
    }
}
/** @} */

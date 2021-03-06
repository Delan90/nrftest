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

#include "app_timer.h"
#include "app_scheduler.h"
#include "sx1276.h"


#include "main.h"
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

States_t State = INIT;

int8_t RssiValue = 0;
int8_t SnrValue = 0;


/* Printing buffers in hex
 *
 * @param[in] p_buf     A pointer to a data buffer.
 * @param[in] len       A length of the data buffer.
 * 
 *
 * @retval true     Data Printed.
 */
static __INLINE bool buf_print(uint8_t * p_buf, uint16_t len)
{
    uint16_t i;
    for (i = 0; i < len; i++)
    {
        SEGGER_RTT_printf(0, "0x%02x ",p_buf[i]);
    }
    return true;
}

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




/** Starting the internal LFCLK RC oscillator.
 */
static void lfclk_config(void)
{
    NRF_CLOCK->LFCLKSRC            = (CLOCK_LFCLKSRC_SRC_RC << CLOCK_LFCLKSRC_SRC_Pos);
    NRF_CLOCK->EVENTS_LFCLKSTARTED = 0;
    NRF_CLOCK->TASKS_LFCLKSTART    = 1;
    while (NRF_CLOCK->EVENTS_LFCLKSTARTED == 0)
    {
        //Do nothing.
    }
    NRF_CLOCK->EVENTS_LFCLKSTARTED = 0;
    SEGGER_RTT_WriteString(0,"Started LFCLK RC oscillator!\n");
}

/*
 * Timer initialization.
 */

static void timers_init(void)
{   
    uint32_t err_code;

    lfclk_config();

    // Initialize timer module.
    APP_TIMER_INIT(APP_TIMER_PRESCALER, APP_TIMER_MAX_TIMERS, APP_TIMER_OP_QUEUE_SIZE, false);

    // Initialize driver timeout timers
    err_code = app_timer_create(&tx_timeout_timer,
                                APP_TIMER_MODE_SINGLE_SHOT,
                                SX1276OnTimeoutIrq);    
    APP_ERROR_CHECK(err_code);    

     err_code = app_timer_create(&rx_timeout_timer,
                                APP_TIMER_MODE_SINGLE_SHOT,
                                SX1276OnTimeoutIrq);
    APP_ERROR_CHECK(err_code); 

    err_code = app_timer_create(&rx_timeout_sync_word,
                                APP_TIMER_MODE_SINGLE_SHOT,
                                SX1276OnTimeoutIrq);
    APP_ERROR_CHECK(err_code); 

    // Initialize scheduler module
    APP_SCHED_INIT(SCHED_MAX_EVENT_DATA_SIZE, SCHED_QUEUE_SIZE);
}



/**@brief Function for application main entry. Does not return. */
int main(void)
{
    timers_init();
    SEGGER_RTT_WriteString(0, "Ping Pong Start!\n");


    //Configure all LEDs as outputs. 
    nrf_gpio_range_cfg_output(ledstart, ledstop);
        
    bool isMaster = true;
    uint8_t i;

    

    SEGGER_RTT_WriteString(0,"Ping Pong Main Entry Start\n");

    while( 1 )
    {
        //Radio.AppScheduler();
        app_sched_execute();
        switch( State )
        {
        case INIT:
            SEGGER_RTT_WriteString(0,"INIT.......\n");
            RadioEvents.TxDone = OnTxDone;
            RadioEvents.RxDone = OnRxDone;
            RadioEvents.TxTimeout = OnTxTimeout;
            RadioEvents.RxTimeout = OnRxTimeout;
            RadioEvents.RxError = OnRxError;

            

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

            State = LOWPOWER;
            SEGGER_RTT_WriteString(0,"INIT--------\n");
            break;

        case RX:
            //SEGGER_RTT_WriteString(0,"Receive Done State!!!!!!!!!!!!!!!!\n");
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
                            Buffer[i] = i-4;
                        }
                        nrf_delay_ms( 1 ); 
                        //SEGGER_RTT_WriteString(0,"master Buffer sending\n");
                        //buf_print(Buffer, BufferSize);
                        SEGGER_RTT_WriteString(0,"PING PING !!!!!! PING PING\n");
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
                            Buffer[i] = i-4;
                        }
                        nrf_delay_ms( 1 );
                        
                        SEGGER_RTT_WriteString(0,"!!!!!PONG PONG PONG!!!!!\n");
                        //buf_print(Buffer, BufferSize);
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
            SEGGER_RTT_WriteString(0,"TXDONE State\n");
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
                //SEGGER_RTT_WriteString(0,"I am the master!\n");
                // Send the next PING frame
                Buffer[0] = 'P';
                Buffer[1] = 'I';
                Buffer[2] = 'N';
                Buffer[3] = 'G';
                for( i = 4; i < BufferSize; i++ )
                {
                    Buffer[i] = i-4;
                }
                nrf_delay_ms( 1 );
                //SEGGER_RTT_WriteString(0,"RX_ERROR/RX_TIMEOUT Buffer sending\n");
                //buf_print(Buffer, BufferSize); 
                //SEGGER_RTT_WriteString(0,"PING............ \n");
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





void OnTxDone( void )
{
    //SEGGER_RTT_WriteString(0,"§§§§§§§§§§OnTxDone\n");
    //Radio.Sleep( );
    State = TX;
}

void OnRxDone( uint8_t *payload, uint16_t size, int16_t rssi, int8_t snr )
{
    //SEGGER_RTT_WriteString(0,"§§§§§§§§OnRxDone\n");
    //Radio.Sleep( );
    BufferSize = size;
    memcpy( Buffer, payload, BufferSize );

   // buf_print(Buffer, BufferSize);
    RssiValue = rssi;
    SnrValue = snr;
    State = RX;

    //SEGGER_RTT_WriteString(0,"RxDone receiveing Buffer-----------\n");
    //buf_print(Buffer, BufferSize); 
}

void OnTxTimeout( void )
{
    //SEGGER_RTT_WriteString(0,"§§§§§§§§§OnTxTimeout\n");
    //Radio.Sleep( );
    State = TX_TIMEOUT;
}

void OnRxTimeout( void )
{
    //SEGGER_RTT_WriteString(0,"§§§§§§§§§OnRxTimeout\n");
   // Radio.Sleep( );
    State = RX_TIMEOUT;
}

void OnRxError( void )
{
    //SEGGER_RTT_WriteString(0,"§§§§§§§§§OnRxError\n");
    //Radio.Sleep( );
    State = RX_ERROR;
}

/** @} */

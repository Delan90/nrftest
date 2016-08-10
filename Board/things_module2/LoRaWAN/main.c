/*
 / _____)             _              | |
( (____  _____ ____ _| |_ _____  ____| |__
 \____ \| ___ |    (_   _) ___ |/ ___)  _ \
 _____) ) ____| | | || |_| ____( (___| | | |
(______/|_____)_|_|_| \__)_____)\____)_| |_|
    (C)2013 Semtech

Description: LoRaMac classA device implementation

License: Revised BSD License, see LICENSE.TXT file include in the project

Maintainer: Miguel Luis and Gregory Cristian
*/
#include <string.h>
#include <math.h>
#include "LoRaMac.h"
#include "radio.h"
#include "sx1276.h"
#include "Comissioning.h"

#include "app_error.h"
#include "app_timer.h"
#include "app_scheduler.h"
#include "nrf_delay.h"
#include "boards.h"
#include "SEGGER_RTT.h"


#include "main.h"
#include "lora_val.h" 

#define led0   25
#define led1   26
#define led2   27
#define ledstart led0
#define ledstop  led1

#define DEBUG_info                                  1 // print debug information via SEGGER_RTT when 1

/*!
 * Join requests trials duty cycle.
 */
#define OVER_THE_AIR_ACTIVATION_DUTYCYCLE           10000  // 10 [s] value in us 10000000

/*!
 * Defines the application data transmission duty cycle. 5/20s, value in [us].
 */
#define APP_TX_DUTYCYCLE                            5000000

/*!
 * Defines a random delay for application data transmission duty cycle. 1s,
 * value in [us].
 */
#define APP_TX_DUTYCYCLE_RND                        1000000

/*!
 * LoRaWAN confirmed messages
 */
#define LORAWAN_CONFIRMED_MSG_ON                    false

/*!
 * LoRaWAN Adaptive Data Rate
 *
 * \remark Please note that when ADR is enabled the end-device should be static
 */
#define LORAWAN_ADR_ON                              1

#if defined( USE_BAND_868 )

#include "LoRaMacTest.h"

/*!
 * LoRaWAN ETSI duty cycle control enable/disable
 *
 * \remark Please note that ETSI mandates duty cycled transmissions. Use only for test purposes
 */
#define LORAWAN_DUTYCYCLE_ON                        true

#define USE_SEMTECH_DEFAULT_CHANNEL_LINEUP          1

#if( USE_SEMTECH_DEFAULT_CHANNEL_LINEUP == 1 ) 

#define LC4                { 867100000, { ( ( DR_5 << 4 ) | DR_0 ) }, 0 }
#define LC5                { 867300000, { ( ( DR_5 << 4 ) | DR_0 ) }, 0 }
#define LC6                { 867500000, { ( ( DR_5 << 4 ) | DR_0 ) }, 0 }
#define LC7                { 867700000, { ( ( DR_5 << 4 ) | DR_0 ) }, 0 }
#define LC8                { 867900000, { ( ( DR_5 << 4 ) | DR_0 ) }, 0 }
#define LC9                { 868800000, { ( ( DR_7 << 4 ) | DR_7 ) }, 2 }

#endif

#endif

/*!
 * LoRaWAN application port
 */
#define LORAWAN_APP_PORT                            2

/*!
 * User application data buffer size
 */
#if defined( USE_BAND_433 ) || defined( USE_BAND_780 ) || defined( USE_BAND_868 )

#define LORAWAN_APP_DATA_SIZE                       16

#elif defined( USE_BAND_915 ) || defined( USE_BAND_915_HYBRID )

#define LORAWAN_APP_DATA_SIZE                       11

#endif

#if( OVER_THE_AIR_ACTIVATION != 0 )

static uint8_t DevEui[] = LORAWAN_DEVICE_EUI;
static uint8_t AppEui[] = LORAWAN_APPLICATION_EUI;
static uint8_t AppKey[] = LORAWAN_APPLICATION_KEY;

#else

static uint8_t NwkSKey[] = LORAWAN_NWKSKEY;
static uint8_t AppSKey[] = LORAWAN_APPSKEY;

/*!
 * Device address
 */
static uint32_t DevAddr;

#endif

/*
  Function that initializes timer 4
 */
void TIM4_Init(void) {
 
}

uint8_t data;
/*!
 * Application port
 */
static uint8_t AppPort = LORAWAN_APP_PORT;

/*!
 * User application data size
 */
static uint8_t AppDataSize = LORAWAN_APP_DATA_SIZE;

/*!
 * User application data buffer size
 */
#define LORAWAN_APP_DATA_MAX_SIZE                           64

/*!
 * User application data
 */
static uint8_t AppData[LORAWAN_APP_DATA_MAX_SIZE];

/*!
 * Indicates if the node is sending confirmed or unconfirmed messages
 */
static uint8_t IsTxConfirmed = LORAWAN_CONFIRMED_MSG_ON;

/*!
 * Defines the application data transmission duty cycle
 */
static uint32_t TxDutyCycleTime;

/*!
 * Timer to handle the application data transmission duty cycle
 */
//static TimerEvent_t TxNextPacketTimer;

/*!
 * Specifies the state of the application LED
 */
static bool AppLedStateOn = false;

/*!
 * Timer to handle the state of LED1
 */
//static TimerEvent_t Led1Timer;

/*!
 * Timer to handle the state of LED2
 */
//static TimerEvent_t Led2Timer;

/*!
 * Timer to handle the state of LED4
 */
//static TimerEvent_t Led4Timer;

/*!
 * Indicates if a new packet can be sent
 */
static bool NextTx = true;

uint8_t counter = 0;
/*!
 * Device states
 */
static enum eDevicState
{
    DEVICE_STATE_INIT,
    DEVICE_STATE_JOIN,
    DEVICE_STATE_SEND,
    DEVICE_STATE_CYCLE,
    DEVICE_STATE_SLEEP
}DeviceState;

/*!
 * LoRaWAN compliance tests support data
 */
struct ComplianceTest_s
{
    bool Running;
    uint8_t State;
    bool IsTxConfirmed;
    uint8_t AppPort;
    uint8_t AppDataSize;
    uint8_t *AppDataBuffer;
    uint16_t DownLinkCounter;
    bool LinkCheck;
    uint8_t DemodMargin;
    uint8_t NbGateways;
}ComplianceTest;


/***********************************************AIRFY*******************************************************************************/
#define RAND_LOCAL_MAX 2147483647L

static uint32_t next = 1;

static int32_t rand1( void )
{
    return ( ( next = next * 1103515245L + 12345L ) % RAND_LOCAL_MAX );
}

static int32_t randr( int32_t min, int32_t max )
{
    return ( int32_t )rand1( ) % ( max - min + 1 ) + min;
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
#if DEBUG_info
    SEGGER_RTT_printf(0,"Died at %d line with error code %d in file %s\n",line_num,error_code,p_file_name);
#endif
    /*for (;;)
    {
        //No implementation needed.
    }*/
}


volatile uint8_t HasLoopedThroughMain = 0;

/**@brief Function for wakeup function, callen when device state is DEVICE_STATE_SLEEP
 *
 * @param[in] none
 */
void TimerWakeUpHandler( void )
{
    if( HasLoopedThroughMain < 5 )
        {
            HasLoopedThroughMain++;
        }
        else
        { 
            nrf_delay_ms(1000);
        }
    //
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
   // SEGGER_RTT_WriteString(0,"Started LFCLK RC oscillator!\n");
}

/*************************************SI7006**********************************************************************************************/
#include "twi_master.h"

char print_buff[10] ;
#define SI7006_ADDRESS             0x80    //SI7006 7 bit base address is 0x40, set LSB to 1 for read, 0 for write
const uint8_t command_read_hum =   0xE5;   //Measure Relative Humidity, Hold Master Mode
const uint8_t command_read_temp =  0xE3;   //Measure Temperature, Hold Master Mode


/**@brief: SI7006 sensor read funtion, reading the SI7006 sensor data base on I2C
*
*  param[i]: data_in_celcius: interger part of the detected sensor data
*  param[i]: data_fraction:   fraction part of the detected sensor data
*  param[i]: command_read:    I2C commond transfered to the device address
*  @retval true     Data Readed successful.
*/
bool SI7006_read(uint8_t * data_in_celcius, uint8_t * data_fraction, uint8_t command_read)
{
    bool transfer_succeeded = false;

    // Write: Begin read sensor command
    if (twi_master_transfer(SI7006_ADDRESS , (uint8_t *)&command_read, 1, TWI_DONT_ISSUE_STOP))
    {   
        uint8_t data_buffer[2];

        // Read: 2 sensor bytes to data_buffer
        if (twi_master_transfer(SI7006_ADDRESS | TWI_READ_BIT , data_buffer, 2, TWI_ISSUE_STOP))
        {
            *data_in_celcius = (int8_t)data_buffer[0];
            *data_fraction   = (int8_t)data_buffer[1];

            transfer_succeeded = true;
        }
    }

    return transfer_succeeded;
}
/******************************************************************************************************************************/


/*!
 * \brief   Prepares the payload of the frame
 */
static void PrepareTxFrame( uint8_t port )
{
    uint8_t si_temp;
    uint8_t si_temp_fraction;
    float si_temperature=0;

    //read temperature if available
    if (SI7006_read(&si_temp, &si_temp_fraction,command_read_temp))
    {
        si_temperature = (float)((((si_temp * 256 + si_temp_fraction) * 175.72) / 65536.0) - 46.85);

#if DEBUG_info
        sprintf(print_buff, "temperature: %f\n\r", si_temperature);
        SEGGER_RTT_WriteString(0,print_buff);
#endif
    }else{
#if DEBUG_info
        SEGGER_RTT_WriteString(0,"rtemperature read error\n");
#endif
    }

    switch( port )
    {
    case 2:
        {
#if defined( USE_BAND_433 ) || defined( USE_BAND_780 ) || defined( USE_BAND_868 )
            uint16_t pressure = 0;
            int16_t altitudeBar = 0;
            int16_t temperature = 0;
            int32_t latitude, longitude = 0;
            uint16_t altitudeGps = 0xFFFF;
            uint8_t batteryLevel = 0;
                        //DHT22_Read();
            pressure = 0;//( uint16_t )( MPL3115ReadPressure( ) / 10 );             // in hPa / 10
            temperature = ( uint16_t )(si_temperature*100);;//( DHT22getTemperature() * 100 );       // in °C * 100
            altitudeBar = 0;//( int16_t )( MPL3115ReadAltitude( ) * 10 );           // in m * 10
            batteryLevel = 0;//BoardGetBatteryLevel( );                             // 1 (very low) to 254 (fully charged)
            //GpsGetLatestGpsPositionBinary( &latitude, &longitude );
            latitude = 0;
            altitudeGps = 0;//GpsGetLatestGpsAltitude( );                           // in m

            AppData[0] = AppLedStateOn;
            AppData[1] = ( pressure >> 8 ) & 0xFF;
            AppData[2] = pressure & 0xFF;
            AppData[3] = ( temperature >> 8 ) & 0xFF;
            AppData[4] = temperature & 0xFF;
            AppData[5] = ( altitudeBar >> 8 ) & 0xFF;
            AppData[6] = altitudeBar & 0xFF;
            AppData[7] = batteryLevel;
            AppData[8] = ( latitude >> 16 ) & 0xFF;
            AppData[9] = ( latitude >> 8 ) & 0xFF;
            AppData[10] = latitude & 0xFF;
            AppData[11] = ( longitude >> 16 ) & 0xFF;
            AppData[12] = ( longitude >> 8 ) & 0xFF;
            AppData[13] = longitude & 0xFF;
            AppData[14] = ( altitudeGps >> 8 ) & 0xFF;
            AppData[15] = altitudeGps & 0xFF;
                        AppData[16] = counter;
                        counter++;
//                      MMA8451Write(0xF4,0x2E);
//                      MMA8451Read(0xF6,&data);
//                      AppData[3] = data & 0xFF;
//                      MMA8451Read(0xF7,&data);
//                      AppData[4] = data & 0xFF;
//                      AppData[15] = 0x10 & 0xFF;
#elif defined( USE_BAND_915 ) || defined( USE_BAND_915_HYBRID )
            int16_t temperature = 0;
            int32_t latitude, longitude = 0;
            uint16_t altitudeGps = 0xFFFF;
            uint8_t batteryLevel = 0;
        
            temperature = ( int16_t )( MPL3115ReadTemperature( ) * 100 );       // in °C * 100
        
            batteryLevel = BoardGetBatteryLevel( );                             // 1 (very low) to 254 (fully charged)
            GpsGetLatestGpsPositionBinary( &latitude, &longitude );
            altitudeGps = GpsGetLatestGpsAltitude( );                           // in m
        
            AppData[0] = AppLedStateOn;
            AppData[1] = temperature;                                           // Signed degrees Celcius in half degree units. So,  +/-63 C
            AppData[2] = batteryLevel;                                          // Per LoRaWAN spec; 0=Charging; 1...254 = level, 255 = N/A
            AppData[3] = ( latitude >> 16 ) & 0xFF;
            AppData[4] = ( latitude >> 8 ) & 0xFF;
            AppData[5] = latitude & 0xFF;
            AppData[6] = ( longitude >> 16 ) & 0xFF;
            AppData[7] = ( longitude >> 8 ) & 0xFF;
            AppData[8] = longitude & 0xFF;
            AppData[9] = ( altitudeGps >> 8 ) & 0xFF;
            AppData[10] = altitudeGps & 0xFF;
#endif
        }
        break;
    case 224:
        if( ComplianceTest.LinkCheck == true )
        {
            ComplianceTest.LinkCheck = false;
            AppDataSize = 3;
            AppData[0] = 5;
            AppData[1] = ComplianceTest.DemodMargin;
            AppData[2] = ComplianceTest.NbGateways;
            ComplianceTest.State = 1;
        }
        else
        {
            switch( ComplianceTest.State )
            {
            case 4:
                ComplianceTest.State = 1;
                break;
            case 1:
                AppDataSize = 2;
                AppData[0] = ComplianceTest.DownLinkCounter >> 8;
                AppData[1] = ComplianceTest.DownLinkCounter;
                break;
            }
        }
        break;
    default:
        break;
    }
}

/*!
 * \brief   Prepares the payload of the frame
 *
 * \retval  [0: frame could be send, 1: error]
 */
static bool SendFrame( void )
{
    //McpsReq_t mcpsReq;
    LoRaMacTxInfo_t txInfo;
    
    if( LoRaMacQueryTxPossible( AppDataSize, &txInfo ) != LORAMAC_STATUS_OK )
    {
        // Send empty frame in order to flush MAC commands
        mcpsRequest.Type = MCPS_UNCONFIRMED;
        mcpsRequest.Req.Unconfirmed.fBuffer = NULL;
        mcpsRequest.Req.Unconfirmed.fBufferSize = 0;
        mcpsRequest.Req.Unconfirmed.Datarate = DR_0;
    }
    else
    {
        if( IsTxConfirmed == false )
        {
            mcpsRequest.Type = MCPS_UNCONFIRMED;
            mcpsRequest.Req.Unconfirmed.fPort = AppPort;
            mcpsRequest.Req.Unconfirmed.fBuffer = AppData;
            mcpsRequest.Req.Unconfirmed.fBufferSize = AppDataSize;
            mcpsRequest.Req.Unconfirmed.Datarate = DR_0;
        }
        else
        {
            mcpsRequest.Type = MCPS_CONFIRMED;
            mcpsRequest.Req.Confirmed.fPort = AppPort;
            mcpsRequest.Req.Confirmed.fBuffer = AppData;
            mcpsRequest.Req.Confirmed.fBufferSize = AppDataSize;
            mcpsRequest.Req.Confirmed.NbTrials = 8;
            mcpsRequest.Req.Confirmed.Datarate = DR_0;
        }
    }
        //nrf_gpio_pin_toggle(led2);
    //define McpsReq_t mcpsRequest as global value to fix the array bug
    if( LoRaMacMcpsRequest() == LORAMAC_STATUS_OK )
    {
        return false;
    }
    return true;
}

/*!
 * \brief Function executed on TxNextPacket Timeout event
 */
static void OnTxNextPacketTimerEvent( void * p_context )
{
    MibRequestConfirm_t mibReq;
    LoRaMacStatus_t status;

    app_timer_stop(TxNextPacketTimer);


    mibReq.Type = MIB_NETWORK_JOINED;
    status = LoRaMacMibGetRequestConfirm( &mibReq );

    if( status == LORAMAC_STATUS_OK )
    {
        if( mibReq.Param.IsNetworkJoined == true )
        {
            DeviceState = DEVICE_STATE_SEND;
            NextTx = true;
        }
        else
        {
            DeviceState = DEVICE_STATE_JOIN;
        }
    }
}


/*
 * Timer initialization.
 */
static void Timer_init(void)
{   
    uint32_t err_code;

    lfclk_config();

    // Initialize timer module.
    APP_TIMER_INIT(APP_TIMER_PRESCALER, APP_TIMER_MAX_TIMERS, APP_TIMER_OP_QUEUE_SIZE, true);

    // Initialize driver timeout timers

    err_code = app_timer_create(&TxNextPacketTimer,
                                APP_TIMER_MODE_SINGLE_SHOT,
                                OnTxNextPacketTimerEvent);    
    APP_ERROR_CHECK(err_code);  

    
    err_code = app_timer_create(&tx_timeout_timer,
                                APP_TIMER_MODE_SINGLE_SHOT,
                                SX1276OnTimeoutIrq);    
    APP_ERROR_CHECK(err_code);    

     err_code = app_timer_create(&rx_timeout_timer,
                                APP_TIMER_MODE_SINGLE_SHOT,
                                SX1276OnTimeoutIrq);
    APP_ERROR_CHECK(err_code); 

    /*err_code = app_timer_create(&rx_timeout_sync_word,
                                APP_TIMER_MODE_SINGLE_SHOT,
                                SX1276OnTimeoutIrq);
    APP_ERROR_CHECK(err_code);*/

    err_code = app_timer_create(&MacStateCheckTimer,
                                APP_TIMER_MODE_SINGLE_SHOT,
                                OnMacStateCheckTimerEvent);    
    APP_ERROR_CHECK(err_code); 

    err_code = app_timer_create(&TxDelayedTimer,
                                APP_TIMER_MODE_SINGLE_SHOT,
                                OnTxDelayedTimerEvent);    
    APP_ERROR_CHECK(err_code);    

    err_code = app_timer_create(&RxWindowTimer1,
                                APP_TIMER_MODE_SINGLE_SHOT,
                                OnRxWindow1TimerEvent);
    APP_ERROR_CHECK(err_code); 

    err_code = app_timer_create(&RxWindowTimer2,
                                APP_TIMER_MODE_SINGLE_SHOT,
                                OnRxWindow2TimerEvent);
    APP_ERROR_CHECK(err_code); 

    err_code = app_timer_create(&AckTimeoutTimer,
                                APP_TIMER_MODE_SINGLE_SHOT,
                                OnAckTimeoutTimerEvent);
    APP_ERROR_CHECK(err_code); 

    // Initialize scheduler module
   APP_SCHED_INIT(SCHED_MAX_EVENT_DATA_SIZE, SCHED_QUEUE_SIZE);
}
/*********************************************************************************************/

/*!
 * \brief   MCPS-Confirm event function
 *
 * \param   [IN] McpsConfirm - Pointer to the confirm structure,
 *               containing confirm attributes.
 */
static void McpsConfirm( McpsConfirm_t *McpsConfirm )
{
    if( McpsConfirm->Status == LORAMAC_EVENT_INFO_STATUS_OK )
    {
        switch( McpsConfirm->McpsRequest )
        {
            case MCPS_UNCONFIRMED:
            {
                // Check Datarate
                // Check TxPower
                break;
            }
            case MCPS_CONFIRMED:
            {
                // Check Datarate
                // Check TxPower
                // Check AckReceived
                // Check NbTrials
                break;
            }
            case MCPS_PROPRIETARY:
            {
                break;
            }
            default:
                break;
        }

        // Switch LED 1 ON
        //nrf_gpio_pin_toggle(led1);
    }
    NextTx = true;
}

/*!
 * \brief   MCPS-Indication event function
 *
 * \param   [IN] McpsIndication - Pointer to the indication structure,
 *               containing indication attributes.
 */
static void McpsIndication( McpsIndication_t *McpsIndication )
{
#if DEBUG_info
    SEGGER_RTT_WriteString(0,"McpsIndication...........\n");
#endif
    if( McpsIndication->Status != LORAMAC_EVENT_INFO_STATUS_OK )
    {
        return;
    }

    switch( McpsIndication->McpsIndication )
    {
        case MCPS_UNCONFIRMED:
        {
            break;
        }
        case MCPS_CONFIRMED:
        {
            break;
        }
        case MCPS_PROPRIETARY:
        {
            break;
        }
        case MCPS_MULTICAST:
        {
            break;
        }
        default:
            break;
    }

    // Check Multicast
    // Check Port
    // Check Datarate
    // Check FramePending
    // Check Buffer
    // Check BufferSize
    // Check Rssi
    // Check Snr
    // Check RxSlot

    if( ComplianceTest.Running == true )
    {
        ComplianceTest.DownLinkCounter++;
    }

    if( McpsIndication->RxData == true )
    {
        switch( McpsIndication->Port )
        {
        case 1: // The application LED can be controlled on port 1 or 2
        case 2:
            if( McpsIndication->BufferSize == 1 )
            {
                AppLedStateOn = McpsIndication->Buffer[0] & 0x01;
            }
            break;
        /*case 10: // Locate: Send GPS Data
            TimerStop( &TxNextPacketTimer );
            TimerSetValue( &TxNextPacketTimer, 100000 );
            TimerStart( &TxNextPacketTimer );
            break;*/
        case 224:
            if( ComplianceTest.Running == false )
            {
                // Check compliance test enable command (i)
                if( ( McpsIndication->BufferSize == 4 ) && 
                    ( McpsIndication->Buffer[0] == 0x01 ) &&
                    ( McpsIndication->Buffer[1] == 0x01 ) &&
                    ( McpsIndication->Buffer[2] == 0x01 ) &&
                    ( McpsIndication->Buffer[3] == 0x01 ) )
                {
                    IsTxConfirmed = false;
                    AppPort = 224;
                    AppDataSize = 2;
                    ComplianceTest.DownLinkCounter = 0;
                    ComplianceTest.LinkCheck = false;
                    ComplianceTest.DemodMargin = 0;
                    ComplianceTest.NbGateways = 0;
                    ComplianceTest.Running = true;
                    ComplianceTest.State = 1;
                    
                    MibRequestConfirm_t mibReq;
                    mibReq.Type = MIB_ADR;
                    mibReq.Param.AdrEnable = true;
                    LoRaMacMibSetRequestConfirm( &mibReq );

#if defined( USE_BAND_868 )
                    LoRaMacTestSetDutyCycleOn( false );
#endif
                }
            }
            else
            {
                ComplianceTest.State = McpsIndication->Buffer[0];
                switch( ComplianceTest.State )
                {
                case 0: // Check compliance test disable command (ii)
                    IsTxConfirmed = LORAWAN_CONFIRMED_MSG_ON;
                    AppPort = LORAWAN_APP_PORT;
                    AppDataSize = LORAWAN_APP_DATA_SIZE;
                    ComplianceTest.DownLinkCounter = 0;
                    ComplianceTest.Running = false;
                    
                    MibRequestConfirm_t mibReq;
                    mibReq.Type = MIB_ADR;
                    mibReq.Param.AdrEnable = LORAWAN_ADR_ON;
                    LoRaMacMibSetRequestConfirm( &mibReq );
#if defined( USE_BAND_868 )
                    LoRaMacTestSetDutyCycleOn( LORAWAN_DUTYCYCLE_ON );
#endif
                    break;
                case 1: // (iii, iv)
                    AppDataSize = 2;
                    break;
                case 2: // Enable confirmed messages (v)
                    IsTxConfirmed = true;
                    ComplianceTest.State = 1;
                    break;
                case 3:  // Disable confirmed messages (vi)
                    IsTxConfirmed = false;
                    ComplianceTest.State = 1;
                    break;
                case 4: // (vii)
                    AppDataSize = McpsIndication->BufferSize;

                    AppData[0] = 4;
                    for( uint8_t i = 1; i < AppDataSize; i++ )
                    {
                        AppData[i] = McpsIndication->Buffer[i] + 1;
                    }
                    break;
                case 5: // (viii)
                    {
                        MlmeReq_t mlmeReq;
                        mlmeReq.Type = MLME_LINK_CHECK;
                        LoRaMacMlmeRequest( &mlmeReq );
                    }
                    break;
                default:
                    break;
                }
            }
            break;
        default:
            break;
        }
    }

    // Switch LED 2 ON for each received downlink
    nrf_gpio_pin_toggle(led2);
}

/*!
 * \brief   MLME-Confirm event function
 *
 * \param   [IN] MlmeConfirm - Pointer to the confirm structure,
 *               containing confirm attributes.
 */
static void MlmeConfirm( MlmeConfirm_t *MlmeConfirm )
{
    if( MlmeConfirm->Status == LORAMAC_EVENT_INFO_STATUS_OK )
    {
        switch( MlmeConfirm->MlmeRequest )
        {
            case MLME_JOIN:
            {
                // Status is OK, node has joined the network
                break;
            }
            case MLME_LINK_CHECK:
            {
                // Check DemodMargin
                // Check NbGateways
                if( ComplianceTest.Running == true )
                {
                    ComplianceTest.LinkCheck = true;
                    ComplianceTest.DemodMargin = MlmeConfirm->DemodMargin;
                    ComplianceTest.NbGateways = MlmeConfirm->NbGateways;
                }
                break;
            }
            default:
                break;
        }
    }
    NextTx = true;
}

/**
 * Main application entry point.
 */
int main( void )
{
    Timer_init();

    nrf_gpio_range_cfg_output(ledstart, ledstop);

    LoRaMacPrimitives_t LoRaMacPrimitives;
    LoRaMacCallback_t LoRaMacCallbacks;
    MibRequestConfirm_t mibReq;

    DeviceState = DEVICE_STATE_INIT;

    nrf_gpio_pin_toggle(led1);
    

    while( 1 )
    {
        nrf_gpio_pin_toggle(led1);
        app_sched_execute();
        switch( DeviceState )
        {
            case DEVICE_STATE_INIT:
            {
#if DEBUG_info
                SEGGER_RTT_WriteString(0,"*****DEVICE_STATE_INIT******\n");
#endif
                LoRaMacPrimitives.MacMcpsConfirm = McpsConfirm;
                LoRaMacPrimitives.MacMcpsIndication = McpsIndication;
                LoRaMacPrimitives.MacMlmeConfirm = MlmeConfirm;
                LoRaMacCallbacks.GetBatteryLevel = 0;//BoardGetBatteryLevel;
                LoRaMacInitialization( &LoRaMacPrimitives, &LoRaMacCallbacks );


                mibReq.Type = MIB_ADR;
                mibReq.Param.AdrEnable = LORAWAN_ADR_ON;
                LoRaMacMibSetRequestConfirm( &mibReq );

                mibReq.Type = MIB_PUBLIC_NETWORK;
                mibReq.Param.EnablePublicNetwork = LORAWAN_PUBLIC_NETWORK;
                LoRaMacMibSetRequestConfirm( &mibReq );

#if defined( USE_BAND_868 )
                LoRaMacTestSetDutyCycleOn( LORAWAN_DUTYCYCLE_ON );

#if( USE_SEMTECH_DEFAULT_CHANNEL_LINEUP == 1 ) 
                LoRaMacChannelAdd( 3, ( ChannelParams_t )LC4 );
                LoRaMacChannelAdd( 4, ( ChannelParams_t )LC5 );
                LoRaMacChannelAdd( 5, ( ChannelParams_t )LC6 );
                LoRaMacChannelAdd( 6, ( ChannelParams_t )LC7 );
                LoRaMacChannelAdd( 7, ( ChannelParams_t )LC8 );
                LoRaMacChannelAdd( 8, ( ChannelParams_t )LC9 );
#endif

#endif
                DeviceState = DEVICE_STATE_JOIN;
                break;
            }
            case DEVICE_STATE_JOIN:
            {
#if DEBUG_info
                SEGGER_RTT_WriteString(0,"*****DEVICE_STATE_JOIN******\n");
#endif
#if( OVER_THE_AIR_ACTIVATION != 0 )
                MlmeReq_t mlmeReq;

                // Initialize LoRaMac device unique ID
//                BoardGetUniqueId( DevEui );

                mlmeReq.Type = MLME_JOIN;

                mlmeReq.Req.Join.DevEui = DevEui;
                mlmeReq.Req.Join.AppEui = AppEui;
                mlmeReq.Req.Join.AppKey = AppKey;

                if( NextTx == true )
                {
                    LoRaMacMlmeRequest( &mlmeReq );
                }

                // Schedule next packet transmission
                TxDutyCycleTime = OVER_THE_AIR_ACTIVATION_DUTYCYCLE;
                DeviceState = DEVICE_STATE_CYCLE;

#else
                // Random seed initialization
//                srand1( BoardGetRandomSeed( ) );

                // Choose a random device address
 //               DevAddr = randr( 0, 0x01FFFFFF );
                DevAddr = LORAWAN_DEVICE_ADDRESS;
                mibReq.Type = MIB_NET_ID;
                mibReq.Param.NetID = LORAWAN_NETWORK_ID;
                LoRaMacMibSetRequestConfirm( &mibReq );

                mibReq.Type = MIB_DEV_ADDR;
                mibReq.Param.DevAddr = DevAddr;
                LoRaMacMibSetRequestConfirm( &mibReq );

                mibReq.Type = MIB_NWK_SKEY;
                mibReq.Param.NwkSKey = NwkSKey;
                LoRaMacMibSetRequestConfirm( &mibReq );

                mibReq.Type = MIB_APP_SKEY;
                mibReq.Param.AppSKey = AppSKey;
                LoRaMacMibSetRequestConfirm( &mibReq );

                mibReq.Type = MIB_NETWORK_JOINED;
                mibReq.Param.IsNetworkJoined = true;
                LoRaMacMibSetRequestConfirm( &mibReq );

                DeviceState = DEVICE_STATE_SEND;
#endif
                break;
            }
            case DEVICE_STATE_SEND:
            {
#if DEBUG_info
                SEGGER_RTT_WriteString(0,"*****DEVICE_STATE_SEND******\n");
#endif
                if( NextTx == true )
                {
                    PrepareTxFrame( AppPort );

                    NextTx = SendFrame( );
                }
                if( ComplianceTest.Running == true )
                {
                    // Schedule next packet transmission as soon as possible
                    TxDutyCycleTime = 300000; // 300 ms
                }
                else
                {
                    // Schedule next packet transmission
                    TxDutyCycleTime = APP_TX_DUTYCYCLE + randr( -APP_TX_DUTYCYCLE_RND, APP_TX_DUTYCYCLE_RND );
                }
                //define TxDutyCycleTime maully to fit for the schedule
                TxDutyCycleTime = 8000;
#if DEBUG_info
                SEGGER_RTT_printf(0, " TxDutyCycleTime %d \r\n", TxDutyCycleTime);
#endif
                DeviceState = DEVICE_STATE_CYCLE;
                break;
            }
            case DEVICE_STATE_CYCLE:
            {
#if DEBUG_info
                SEGGER_RTT_WriteString(0,"*****DEVICE_STATE_CYCLE******\n");
#endif
                DeviceState = DEVICE_STATE_SLEEP;

                // Schedule next packet transmission
                app_timer_start(TxNextPacketTimer,APP_TIMER_TICKS(TxDutyCycleTime,0),NULL);
                break;
            }
            case DEVICE_STATE_SLEEP:
            {
                // Wake up through events
                //TimerWakeUpHandler( );
                break;
            }
            default:
            {
                DeviceState = DEVICE_STATE_INIT;
                break;
            }
        }
        /*if( GpsGetPpsDetectedState( ) == true )
        {
            // Switch LED 4 ON
            GpioWrite( &Led4, 0 );
            TimerStart( &Led4Timer );
        }*/
    }
}

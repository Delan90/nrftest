/*
 / _____)             _              | |
( (____  _____ ____ _| |_ _____  ____| |__
 \____ \| ___ |    (_   _) ___ |/ ___)  _ \
 _____) ) ____| | | || |_| ____( (___| | | |
(______/|_____)_|_|_| \__)_____)\____)_| |_|
    (C)2013 Semtech

Description: Generic SX1276 driver implementation

License: Revised BSD License, see LICENSE.TXT file include in the project

Maintainer: Miguel Luis, Gregory Cristian and Wael Guibene
*/
#include <math.h>
#include <string.h>
#include <stdbool.h>
#include "radio.h"
#include "sx1276.h"
#include "nrf.h"
#include "app_timer.h"
#include "app_scheduler.h"
#include "nrf_delay.h"
#include "nrf_gpio.h"
#include "nrf_gpiote.h"
#include "boards.h"
#include "spi_master.h"
#include "SEGGER_RTT.h"

#include "include.h"
#include "main.h"
//#include "timer_main.h"
 
/*
 * Local types definition
 */

/*!
 * Radio registers definition
 */
typedef struct
{
    RadioModems_t Modem;
    uint8_t       Addr;
    uint8_t       Value;
}RadioRegisters_t;

/*!
 * FSK bandwidth definition
 */
typedef struct
{
    uint32_t bandwidth;
    uint8_t  RegValue;
}FskBandwidth_t;


const struct Radio_s Radio =
{
    SX1276Init,
    SX1276GetStatus,
    SX1276SetModem,
    SX1276SetChannel,
    SX1276IsChannelFree,
    SX1276Random,
    SX1276SetRxConfig,
    SX1276SetTxConfig,
    SX1276CheckRfFrequency,
    SX1276GetTimeOnAir,
    SX1276Send,
    SX1276SetSleep,
    SX1276SetStby, 
    SX1276SetRx,
    SX1276StartCad,
    SX1276ReadRssi,
    SX1276Write,
    SX1276Read,
    SX1276WriteBuffer,
    SX1276ReadBuffer,
    SX1276SetMaxPayloadLength,
    SX1276Scheduler    
};


static volatile bool m_transfer_completed = true; /**< A flag to inform about completed transfer. */

#if defined(SPI_MASTER_0_ENABLE)
    #define SPI_MASTER_HW SPI_MASTER_0
#elif defined(SPI_MASTER_1_ENABLE)
    #define SPI_MASTER_HW SPI_MASTER_1
#endif




#define TX_RX_BUF_LENGTH    100u     /**< SPI transaction buffer length. */
#define DELAY_MS            1000u    /**< Timer delay in milliseconds. */

static uint8_t m_tx_data[TX_RX_BUF_LENGTH] = {0}; /**< A buffer with data to transfer. */
static uint8_t m_rx_data[TX_RX_BUF_LENGTH] = {0}; /**< A buffer for incoming data. */

/*
 * Private functions prototypes
 */

/*!
 * Performs the Rx chain calibration for LF and HF bands
 * \remark Must be called just after the reset so all registers are at their
 *         default values
 */
static void RxChainCalibration( void );

/*!
 * \brief Resets the SX1276
 */
void SX1276Reset( void );

/*!
 * \brief Sets the SX1276 in transmission mode for the given time
 * \param [IN] timeout Transmission timeout [us] [0: continuous, others timeout]
 */
void SX1276SetTx( uint32_t timeout );

/*!
 * \brief Writes the buffer contents to the SX1276 FIFO
 *
 * \param [IN] buffer Buffer containing data to be put on the FIFO.
 * \param [IN] size Number of bytes to be written to the FIFO
 */
void SX1276WriteFifo( uint8_t *buffer, uint8_t size );

/*!
 * \brief Reads the contents of the SX1276 FIFO
 *
 * \param [OUT] buffer Buffer where to copy the FIFO read data.
 * \param [IN] size Number of bytes to be read from the FIFO
 */
void SX1276ReadFifo( uint8_t *buffer, uint8_t size );

/*!
 * \brief Sets the SX1276 operating mode
 *
 * \param [IN] opMode New operating mode
 */
void SX1276SetOpMode( uint8_t opMode );

/*
 * SX1276 DIO IRQ callback functions prototype
 */

/*!
 * \brief DIO 0 IRQ callback
 */
void SX1276OnDio0Irq( void * p_event_data, uint16_t event_size );
//void SX1276OnDio0Irq( void);

/*!
 * \brief DIO 1 IRQ callback
 */
void SX1276OnDio1Irq( void * p_event_data, uint16_t event_size );
//void SX1276OnDio1Irq( void);

/*!
 * \brief DIO 2 IRQ callback
 */
void SX1276OnDio2Irq( void * p_event_data, uint16_t event_size );

/*!
 * \brief DIO 3 IRQ callback
 */
void SX1276OnDio3Irq( void * p_event_data, uint16_t event_size );

/*!
 * \brief DIO 4 IRQ callback
 */
void SX1276OnDio4Irq( void * p_event_data, uint16_t event_size );

/*!
 * \brief DIO 5 IRQ callback
 */
void SX1276OnDio5Irq( void * p_event_data, uint16_t event_size );

/*!
 * \brief Tx & Rx timeout timer callback
 */
void SX1276OnTimeoutIrq( void *p_context);

void SX1276SetAntSwLowPower( bool status );


/*!
 * \brief Initializes the RF Switch I/Os pins interface
 */
void SX1276AntSwInit( void );

/*!
 * \brief De-initializes the RF Switch I/Os pins interface 
 *
 * \remark Needed to decrease the power consumption in MCU lowpower modes
 */
void SX1276AntSwDeInit( void );


void SX1276SetAntSw( uint8_t rxTx );

static void spi_master_event_handler(spi_master_evt_t spi_master_evt);

/*
 * Private global constants
 */

/*!
 * Radio hardware registers initialization
 * 
 * \remark RADIO_INIT_REGISTERS_VALUE is defined in sx1276-board.h file
 */
const RadioRegisters_t RadioRegsInit[] = RADIO_INIT_REGISTERS_VALUE;

/*!
 * Constant values need to compute the RSSI value
 */
#define RSSI_OFFSET_LF                              -164
#define RSSI_OFFSET_HF                              -157

/*!
 * Precomputed FSK bandwidth registers values
 */
const FskBandwidth_t FskBandwidths[] =
{       
    { 2600  , 0x17 },   
    { 3100  , 0x0F },
    { 3900  , 0x07 },
    { 5200  , 0x16 },
    { 6300  , 0x0E },
    { 7800  , 0x06 },
    { 10400 , 0x15 },
    { 12500 , 0x0D },
    { 15600 , 0x05 },
    { 20800 , 0x14 },
    { 25000 , 0x0C },
    { 31300 , 0x04 },
    { 41700 , 0x13 },
    { 50000 , 0x0B },
    { 62500 , 0x03 },
    { 83333 , 0x12 },
    { 100000, 0x0A },
    { 125000, 0x02 },
    { 166700, 0x11 },
    { 200000, 0x09 },
    { 250000, 0x01 },
    { 300000, 0x00 }, // Invalid Badwidth
};

/*
 * Private global variables
 */

/*!
 * Radio callbacks variable
 */
 //static RadioEvents_t *RadioEvents;

/*!
 * Reception buffer
 */
static uint8_t RxTxBuffer[RX_BUFFER_SIZE];

/*
 * Public global variables
 */

/*!
 * Radio hardware and global parameters
 */
SX1276_t SX1276;

/*!
 * Hardware DIO IRQ callback initialization
 */
// DioIrqHandler *DioIrq[] = { SX1276OnDio0Irq, SX1276OnDio1Irq,
                            // SX1276OnDio2Irq, SX1276OnDio3Irq,
                            // SX1276OnDio4Irq, NULL };

uint32_t AntSwitchLf = 28;
uint32_t AntSwitchHf = 29;


static bool RadioIsActive = false;


/*
 * Radio driver functions implementation
 */

/* Enabling GPIO Interruts */

void gpio_interrupts(){
    SEGGER_RTT_WriteString(0,"Enabling Gpio Interrupts\n");
    nrf_gpio_range_cfg_output(AntSwitchLf,AntSwitchHf);
    nrf_gpio_cfg_input(SX1276.DIO0, NRF_GPIO_PIN_PULLUP);
    nrf_gpio_cfg_input(SX1276.DIO1, NRF_GPIO_PIN_PULLUP);

    // Enable interrupt:
    nrf_gpiote_event_config(0, SX1276.DIO0 , NRF_GPIOTE_POLARITY_LOTOHI);
    nrf_gpiote_event_config(1, SX1276.DIO1 , NRF_GPIOTE_POLARITY_LOTOHI);

    NRF_GPIOTE->INTENSET  = GPIOTE_INTENSET_IN0_Set << GPIOTE_INTENSET_IN0_Pos;
    NRF_GPIOTE->INTENSET  = GPIOTE_INTENSET_IN1_Set << GPIOTE_INTENSET_IN1_Pos;
   // NVIC_SetPriority(GPIOTE_IRQn,2);
    NVIC_EnableIRQ(GPIOTE_IRQn);
    SEGGER_RTT_printf(0, "NVIC_GetPriority(GPIOTE_IRQn) :%d \r\n",  NVIC_GetPriority(GPIOTE_IRQn));
}

/*
 * Interrupt handler
 */

void GPIOTE_IRQHandler(void)
{
    // Event causing the interrupt must be cleared.
    if ((NRF_GPIOTE->EVENTS_IN[0] == 1) && 
        (NRF_GPIOTE->INTENSET & GPIOTE_INTENSET_IN0_Msk))
    {
        NRF_GPIOTE->EVENTS_IN[0] = 0;
        SEGGER_RTT_WriteString(0,"DIO0 Interrupt Handler\n");

        app_sched_event_put(NULL, 0, SX1276OnDio0Irq);    /* Scheduler for DIO0 */

        SEGGER_RTT_WriteString(0,"DIO0 Interrupt END\n");

    }else if ((NRF_GPIOTE->EVENTS_IN[1] == 1) && 
        (NRF_GPIOTE->INTENSET & GPIOTE_INTENSET_IN1_Msk))
    {
        NRF_GPIOTE->EVENTS_IN[1] = 0;
        SEGGER_RTT_WriteString(0,"DIO1 Interrupt Handler\n");
        app_sched_event_put(NULL, 0, SX1276OnDio1Irq);   /* Scheduler for DIO1 */
        SEGGER_RTT_WriteString(0,"DIO1 Interrupt END\n");
    }
    nrf_gpio_pin_toggle(LED_0);
}


/* Prepares buffers and starts data transfer.
 *
 * @param[in] p_tx_data     A pointer to a buffer TX.
 * @param[in] p_rx_data     A pointer to a buffer RX.
 * @param[in] len           A length of the data buffers.
 */
static void spi_send_recv(uint8_t * const p_tx_data,
                          uint8_t * const p_rx_data,
                          const uint16_t len)
{
    //Start transfer.
    uint32_t err_code = spi_master_send_recv(SPI_MASTER_HW, p_tx_data, len, p_rx_data, len);
    APP_ERROR_CHECK(err_code);
    
    nrf_gpio_pin_toggle(LED_0);
}

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

/*
 * Initializing a SPI master driver.
 */

static void spi_master_init(void)
{
    uint32_t err_code;

    spi_master_config_t spi_config = SPI_MASTER_SX1276;
    
    #if defined(SPI_MASTER_0_ENABLE)
    SEGGER_RTT_WriteString(0,"SPI0 MASTER INIT!\n");
        spi_config.SPI_Pin_SCK = SPIM0_SCK_PIN;
        spi_config.SPI_Pin_MISO = SPIM0_MISO_PIN;
        spi_config.SPI_Pin_MOSI = SPIM0_MOSI_PIN;
        spi_config.SPI_Pin_SS = SPIM0_SS_PIN;
    #elif defined(SPI_MASTER_1_ENABLE)
    SEGGER_RTT_WriteString(0,"SPI1 MASTER INIT!\n");
        spi_config.SPI_Pin_SCK = SPIM1_SCK_PIN;
        spi_config.SPI_Pin_MISO = SPIM1_MISO_PIN;
        spi_config.SPI_Pin_MOSI = SPIM1_MOSI_PIN;
        spi_config.SPI_Pin_SS = SPIM1_SS_PIN;
    #endif /* SPI_MASTER_ENABLE */
    
    err_code = spi_master_open(SPI_MASTER_HW, &spi_config);
    APP_ERROR_CHECK(err_code);


    spi_master_evt_handler_reg(SPI_MASTER_HW, spi_master_event_handler);

    SEGGER_RTT_printf(0, "NVIC_GetPriority(SPI0_TWI0_IRQn) :%d \r\n",  NVIC_GetPriority(SPI0_TWI0_IRQn));
    SEGGER_RTT_printf(0, "NVIC_GetPriority(SPI1_TWI1_IRQn) :%d \r\n",  NVIC_GetPriority(SPI1_TWI1_IRQn));
}

/* SPI master event callback.
 *
 * Upon receiving an SPI transaction complete event, checks if received data are valid.
 *
 * @param[in] spi_master_evt    SPI master driver event.
 */
static void spi_master_event_handler(spi_master_evt_t spi_master_evt)
{
    switch (spi_master_evt.evt_type)
    {
        SEGGER_RTT_WriteString(0,"SPI_MASTER_EVT_TRANSFER_COMPLETED\n");
        case SPI_MASTER_EVT_TRANSFER_COMPLETED:        
            //Inform application that transfer is completed.
            m_transfer_completed = true;
            break;
        
        default:
            //No implementation needed.
            break;
    }
}

void SX1276SetAntSw( uint8_t rxTx )
{
    if( SX1276.RxTx == rxTx )
    {
        SEGGER_RTT_WriteString(0,"AntSwitchLf AntSwitchHf same as before\n");
        return;
    }

    SX1276.RxTx = rxTx;

    if( rxTx != 0 ) // 1: TX, 0: RX
    {   
        SEGGER_RTT_WriteString(0,"AntSwitchLf 0 AntSwitchHf 1\n");
        nrf_gpio_pin_clear( AntSwitchLf );
        nrf_gpio_pin_set( AntSwitchHf );
    }
    else
    {
        SEGGER_RTT_WriteString(0,"AntSwitchLf 1 AntSwitchHf 0\n");
        nrf_gpio_pin_set( AntSwitchLf );
        nrf_gpio_pin_clear( AntSwitchHf );
    }
}

void SX1276SetAntSwLowPower( bool status )
{
    if( RadioIsActive != status )
    {
        RadioIsActive = status;
    
        if( status == false )
        {
            SX1276AntSwInit( );
        }
        else
        {
            SX1276AntSwDeInit( );
        }
    }
}


void SX1276AntSwInit( void )
{
    nrf_gpio_pin_set( AntSwitchLf );
    nrf_gpio_pin_clear( AntSwitchHf );
}


void SX1276AntSwDeInit( void )
{
    
}


uint8_t SX1276GetPaSelect( uint32_t channel )
{
    return RF_PACONFIG_PASELECT_PABOOST; /* Enble this statement for sending data on PABOOST always */
    if( channel < RF_MID_BAND_THRESH )
    {
        return RF_PACONFIG_PASELECT_PABOOST;
    }
    else
    {
        return RF_PACONFIG_PASELECT_RFO;
    }
}

void SX1276Init( void )
{
    uint8_t i;
    //uint32_t err_code;

    //timers_init();

    SEGGER_RTT_WriteString(0, "INIT SX1276!\n");

    //Initialize SPI master.
    spi_master_init();
    //APP_ERROR_CHECK(err_code);

    //spi_master_evt_handler_reg(SPI_MASTER_HW, spi_master_event_handler);

    /* Initialising pi numbers */
    SX1276.Reset = 30;
    SX1276.DIO0 = 12;
    SX1276.DIO1 = 11;
    // SX1276.DIO2 = 13;
    // SX1276.DIO3 = 12;
    // SX1276.DIO4 = 11;
    // SX1276.DIO5 = 10;
    SX1276.Spi  = 1;
    
    gpio_interrupts();

    SX1276Reset( );

    RxChainCalibration( );
    
    SX1276SetOpMode( RF_OPMODE_SLEEP );        

    for( i = 0; i < sizeof( RadioRegsInit ) / sizeof( RadioRegisters_t ); i++ )
    {
        SX1276SetModem( RadioRegsInit[i].Modem );
        SX1276Write( RadioRegsInit[i].Addr, RadioRegsInit[i].Value );
    }

    SX1276SetModem( MODEM_FSK );
    SX1276.Settings.State = RF_IDLE;

   

    SEGGER_RTT_WriteString(0, "INIT Complete\n");
}

RadioState_t SX1276GetStatus( void )
{
    return SX1276.Settings.State;
}

void SX1276SetChannel( uint32_t freq )
{
    SX1276.Settings.Channel = freq;
    freq = ( uint32_t )( ( double )freq / ( double )FREQ_STEP );
    SX1276Write( REG_FRFMSB, ( uint8_t )( ( freq >> 16 ) & 0xFF ) );
    SX1276Write( REG_FRFMID, ( uint8_t )( ( freq >> 8 ) & 0xFF ) );
    SX1276Write( REG_FRFLSB, ( uint8_t )( freq & 0xFF ) );

    SEGGER_RTT_WriteString(0, "Channel Set!\n");
}

bool SX1276IsChannelFree( RadioModems_t modem, uint32_t freq, int16_t rssiThresh )
{
    int16_t rssi = 0;
    
    //SX1276SetModem( modem );

    SX1276SetChannel( freq );
    
    SX1276SetOpMode( RF_OPMODE_RECEIVER );

    nrf_delay_ms( 1 );
    
    rssi = SX1276ReadRssi( modem );
    
    SX1276SetSleep( );
    
    if( rssi > rssiThresh )
    {
        return false;
    }
    return true;
}

uint32_t SX1276Random( void )
{
    uint8_t i;
    uint32_t rnd = 0;

    /*
     * Radio setup for random number generation 
     */
    // Set LoRa modem ON
   // SX1276SetModem( MODEM_LORA );

    // Disable LoRa modem interrupts
    SX1276Write( REG_LR_IRQFLAGSMASK, RFLR_IRQFLAGS_RXTIMEOUT |
                  RFLR_IRQFLAGS_RXDONE |
                  RFLR_IRQFLAGS_PAYLOADCRCERROR |
                  RFLR_IRQFLAGS_VALIDHEADER |
                  RFLR_IRQFLAGS_TXDONE |
                  RFLR_IRQFLAGS_CADDONE |
                  RFLR_IRQFLAGS_FHSSCHANGEDCHANNEL |
                  RFLR_IRQFLAGS_CADDETECTED );

    // Set radio in continuous reception
    SX1276SetOpMode( RF_OPMODE_RECEIVER );

    for( i = 0; i < 32; i++ )
    {
        nrf_delay_ms( 1 );
        // Unfiltered RSSI value reading. Only takes the LSB value
        rnd |= ( ( uint32_t )SX1276Read( REG_LR_RSSIWIDEBAND ) & 0x01 ) << i;
    }

    SX1276SetSleep( );

    return rnd;
}

/*!
 * Performs the Rx chain calibration for LF and HF bands
 * \remark Must be called just after the reset so all registers are at their
 *         default values
 */
static void RxChainCalibration( void )
{
    uint8_t regPaConfigInitVal;
    uint32_t initialFreq;

    SEGGER_RTT_WriteString(0, "RxChainCalibration!\n");

    // Save context
    regPaConfigInitVal = SX1276Read( REG_PACONFIG );
    initialFreq = ( double )( ( ( uint32_t )SX1276Read( REG_FRFMSB ) << 16 ) |
                              ( ( uint32_t )SX1276Read( REG_FRFMID ) << 8 ) |
                              ( ( uint32_t )SX1276Read( REG_FRFLSB ) ) ) * ( double )FREQ_STEP;

    // Cut the PA just in case, RFO output, power = -1 dBm
    SX1276Write( REG_PACONFIG, 0x00 );

    // Launch Rx chain calibration for LF band
    SX1276Write( REG_IMAGECAL, ( SX1276Read( REG_IMAGECAL ) & RF_IMAGECAL_IMAGECAL_MASK ) | RF_IMAGECAL_IMAGECAL_START );
    while( ( SX1276Read( REG_IMAGECAL ) & RF_IMAGECAL_IMAGECAL_RUNNING ) == RF_IMAGECAL_IMAGECAL_RUNNING )
    {
    }

    // Sets a Frequency in HF band
    SX1276SetChannel( 868000000 );

    // Launch Rx chain calibration for HF band 
    SX1276Write( REG_IMAGECAL, ( SX1276Read( REG_IMAGECAL ) & RF_IMAGECAL_IMAGECAL_MASK ) | RF_IMAGECAL_IMAGECAL_START );
    while( ( SX1276Read( REG_IMAGECAL ) & RF_IMAGECAL_IMAGECAL_RUNNING ) == RF_IMAGECAL_IMAGECAL_RUNNING )
    {
    }

    // Restore context
    SX1276Write( REG_PACONFIG, regPaConfigInitVal );
    SX1276SetChannel( initialFreq );

    SEGGER_RTT_WriteString(0, "Exit RxChainCalibration!\n");
}

/*!
 * Returns the known FSK bandwidth registers value
 *
 * \param [IN] bandwidth Bandwidth value in Hz
 * \retval regValue Bandwidth register value.
 */
static uint8_t GetFskBandwidthRegValue( uint32_t bandwidth )
{
    uint8_t i;

    for( i = 0; i < ( sizeof( FskBandwidths ) / sizeof( FskBandwidth_t ) ) - 1; i++ )
    {
        if( ( bandwidth >= FskBandwidths[i].bandwidth ) && ( bandwidth < FskBandwidths[i + 1].bandwidth ) )
        {
            return FskBandwidths[i].RegValue;
        }
    }
    // ERROR: Value not found
    while( 1 );
}

void SX1276SetRxConfig( RadioModems_t modem, uint32_t bandwidth,
                         uint32_t datarate, uint8_t coderate,
                         uint32_t bandwidthAfc, uint16_t preambleLen,
                         uint16_t symbTimeout, bool fixLen,
                         uint8_t payloadLen,
                         bool crcOn, bool freqHopOn, uint8_t hopPeriod,
                         bool iqInverted, bool rxContinuous )
{
    
    //modem = MODEM_LORA;
    SX1276SetModem(modem);

    SEGGER_RTT_printf(0, " Rx Setting Modem :%d \r\n", modem);
    switch( modem )
    {
    case MODEM_FSK:
        {
            SX1276.Settings.Fsk.Bandwidth = bandwidth;
            SX1276.Settings.Fsk.Datarate = datarate;
            SX1276.Settings.Fsk.BandwidthAfc = bandwidthAfc;
            SX1276.Settings.Fsk.FixLen = fixLen;
            SX1276.Settings.Fsk.PayloadLen = payloadLen;
            SX1276.Settings.Fsk.CrcOn = crcOn;
            SX1276.Settings.Fsk.IqInverted = iqInverted;
            SX1276.Settings.Fsk.RxContinuous = rxContinuous;
            SX1276.Settings.Fsk.PreambleLen = preambleLen;
            
            datarate = ( uint16_t )( ( double )XTAL_FREQ / ( double )datarate );
            SX1276Write( REG_BITRATEMSB, ( uint8_t )( datarate >> 8 ) );
            SX1276Write( REG_BITRATELSB, ( uint8_t )( datarate & 0xFF ) );

            SX1276Write( REG_RXBW, GetFskBandwidthRegValue( bandwidth ) );
            SX1276Write( REG_AFCBW, GetFskBandwidthRegValue( bandwidthAfc ) );

            SX1276Write( REG_PREAMBLEMSB, ( uint8_t )( ( preambleLen >> 8 ) & 0xFF ) );
            SX1276Write( REG_PREAMBLELSB, ( uint8_t )( preambleLen & 0xFF ) );
            
            if( fixLen == 1 )
            {
                SX1276Write( REG_PAYLOADLENGTH, payloadLen );
            }
            else
            {
                SX1276Write( REG_PAYLOADLENGTH, 0xFF ); // Set payload length to the maximum 
            }
            
            SX1276Write( REG_PACKETCONFIG1,
                         ( SX1276Read( REG_PACKETCONFIG1 ) & 
                           RF_PACKETCONFIG1_CRC_MASK &
                           RF_PACKETCONFIG1_PACKETFORMAT_MASK ) |
                           ( ( fixLen == 1 ) ? RF_PACKETCONFIG1_PACKETFORMAT_FIXED : RF_PACKETCONFIG1_PACKETFORMAT_VARIABLE ) |
                           ( crcOn << 4 ) );
        }
        break;
    case MODEM_LORA:
        {
            if( bandwidth > 2 )
            {
                // Fatal error: When using LoRa modem only bandwidths 125, 250 and 500 kHz are supported
                while( 1 );
            }
            bandwidth += 7;
            SX1276.Settings.LoRa.Bandwidth = bandwidth;
            SX1276.Settings.LoRa.Datarate = datarate;
            SX1276.Settings.LoRa.Coderate = coderate;
            SX1276.Settings.LoRa.PreambleLen = preambleLen;
            SX1276.Settings.LoRa.FixLen = fixLen;
            SX1276.Settings.LoRa.PayloadLen = payloadLen;
            SX1276.Settings.LoRa.CrcOn = crcOn;
            SX1276.Settings.LoRa.FreqHopOn = freqHopOn;
            SX1276.Settings.LoRa.HopPeriod = hopPeriod;
            SX1276.Settings.LoRa.IqInverted = iqInverted;
            SX1276.Settings.LoRa.RxContinuous = rxContinuous;

            if( datarate > 12 )
            {
                datarate = 12;
            }
            else if( datarate < 6 )
            {
                datarate = 6;
            }
        
            if( ( ( bandwidth == 7 ) && ( ( datarate == 11 ) || ( datarate == 12 ) ) ) ||
                ( ( bandwidth == 8 ) && ( datarate == 12 ) ) )
            {
                SX1276.Settings.LoRa.LowDatarateOptimize = 0x01;
            }
            else
            {
                SX1276.Settings.LoRa.LowDatarateOptimize = 0x00;
            }

           // SEGGER_RTT_WriteString(0, "REG_LR_MODEMCONFIG1\n");
            SX1276Write( REG_LR_MODEMCONFIG1, 
                         ( SX1276Read( REG_LR_MODEMCONFIG1 ) &
                           RFLR_MODEMCONFIG1_BW_MASK &
                           RFLR_MODEMCONFIG1_CODINGRATE_MASK &
                           RFLR_MODEMCONFIG1_IMPLICITHEADER_MASK ) |
                           ( bandwidth << 4 ) | ( coderate << 1 ) | 
                           fixLen );

            //SEGGER_RTT_WriteString(0, "REG_LR_MODEMCONFIG2\n");            
            SX1276Write( REG_LR_MODEMCONFIG2,
                         ( SX1276Read( REG_LR_MODEMCONFIG2 ) &
                           RFLR_MODEMCONFIG2_SF_MASK &
                           RFLR_MODEMCONFIG2_RXPAYLOADCRC_MASK &
                           RFLR_MODEMCONFIG2_SYMBTIMEOUTMSB_MASK ) |
                           ( datarate << 4 ) | ( crcOn << 2 ) |
                           ( ( symbTimeout >> 8 ) & ~RFLR_MODEMCONFIG2_SYMBTIMEOUTMSB_MASK ) );

            //SEGGER_RTT_WriteString(0, "REG_LR_MODEMCONFIG3\n");
            SX1276Write( REG_LR_MODEMCONFIG3, 
                         ( SX1276Read( REG_LR_MODEMCONFIG3 ) &
                           RFLR_MODEMCONFIG3_LOWDATARATEOPTIMIZE_MASK ) |
                           ( SX1276.Settings.LoRa.LowDatarateOptimize << 3 ) );

            //SEGGER_RTT_WriteString(0, "REG_LR_SYMBTIMEOUTLSB\n");
            SX1276Write( REG_LR_SYMBTIMEOUTLSB, ( uint8_t )( symbTimeout & 0xFF ) );
            
            //SEGGER_RTT_WriteString(0, "REG_LR_PREAMBLEMSB\n");
            SX1276Write( REG_LR_PREAMBLEMSB, ( uint8_t )( ( preambleLen >> 8 ) & 0xFF ) );

            //SEGGER_RTT_WriteString(0, "REG_LR_PREAMBLELSB\n");
            SX1276Write( REG_LR_PREAMBLELSB, ( uint8_t )( preambleLen & 0xFF ) );

            if( fixLen == 1 )
            {
                SX1276Write( REG_LR_PAYLOADLENGTH, payloadLen );
            }

            if( SX1276.Settings.LoRa.FreqHopOn == true )
            {
                SX1276Write( REG_LR_PLLHOP, ( SX1276Read( REG_LR_PLLHOP ) & RFLR_PLLHOP_FASTHOP_MASK ) | RFLR_PLLHOP_FASTHOP_ON );
                SX1276Write( REG_LR_HOPPERIOD, SX1276.Settings.LoRa.HopPeriod );
            }

            if( ( bandwidth == 9 ) && ( RF_MID_BAND_THRESH ) )
            {
                // ERRATA 2.1 - Sensitivity Optimization with a 500 kHz Bandwidth 
                SX1276Write( REG_LR_TEST36, 0x02 );
                SX1276Write( REG_LR_TEST3A, 0x64 );
            }
            else if( bandwidth == 9 )
            {
                // ERRATA 2.1 - Sensitivity Optimization with a 500 kHz Bandwidth
                SX1276Write( REG_LR_TEST36, 0x02 );
                SX1276Write( REG_LR_TEST3A, 0x7F );
            }
            else
            {
                // ERRATA 2.1 - Sensitivity Optimization with a 500 kHz Bandwidth
                SX1276Write( REG_LR_TEST36, 0x03 );
            }
            
            if( datarate == 6 )
            {
                SX1276Write( REG_LR_DETECTOPTIMIZE, 
                             ( SX1276Read( REG_LR_DETECTOPTIMIZE ) &
                               RFLR_DETECTIONOPTIMIZE_MASK ) |
                               RFLR_DETECTIONOPTIMIZE_SF6 );
                SX1276Write( REG_LR_DETECTIONTHRESHOLD, 
                             RFLR_DETECTIONTHRESH_SF6 );
            }
            else
            {
                SX1276Write( REG_LR_DETECTOPTIMIZE,
                             ( SX1276Read( REG_LR_DETECTOPTIMIZE ) &
                             RFLR_DETECTIONOPTIMIZE_MASK ) |
                             RFLR_DETECTIONOPTIMIZE_SF7_TO_SF12 );
                SX1276Write( REG_LR_DETECTIONTHRESHOLD, 
                             RFLR_DETECTIONTHRESH_SF7_TO_SF12 );
            }
        }
        break;
    }
    SEGGER_RTT_WriteString(0, "SX1276 RXCONFIG SET!\n");
}

void SX1276SetTxConfig( RadioModems_t modem, int8_t power, uint32_t fdev, 
                        uint32_t bandwidth, uint32_t datarate,
                        uint8_t coderate, uint16_t preambleLen,
                        bool fixLen, bool crcOn, bool freqHopOn, 
                        uint8_t hopPeriod, bool iqInverted, uint32_t timeout )
{
    //modem = MODEM_LORA;
    uint8_t paConfig = 0;
    uint8_t paDac = 0;
    
    SX1276SetModem( modem );

    SEGGER_RTT_printf(0, " Tx Setting Modem :%d \r\n", modem);
    
    paConfig = SX1276Read( REG_PACONFIG );
    paDac = SX1276Read( REG_PADAC );

    paConfig = ( paConfig & RF_PACONFIG_PASELECT_MASK ) | SX1276GetPaSelect( SX1276.Settings.Channel );
    paConfig = ( paConfig & RF_PACONFIG_MAX_POWER_MASK ) | 0x70;

    if( ( paConfig & RF_PACONFIG_PASELECT_PABOOST ) == RF_PACONFIG_PASELECT_PABOOST )
    {
        if( power > 17 )
        {
            paDac = ( paDac & RF_PADAC_20DBM_MASK ) | RF_PADAC_20DBM_ON;
        }
        else
        {
            paDac = ( paDac & RF_PADAC_20DBM_MASK ) | RF_PADAC_20DBM_OFF;
        }
        if( ( paDac & RF_PADAC_20DBM_ON ) == RF_PADAC_20DBM_ON )
        {
            if( power < 5 )
            {
                power = 5;
            }
            if( power > 20 )
            {
                power = 20;
            }
            paConfig = ( paConfig & RF_PACONFIG_OUTPUTPOWER_MASK ) | ( uint8_t )( ( uint16_t )( power - 5 ) & 0x0F );
        }
        else
        {
            if( power < 2 )
            {
                power = 2;
            }
            if( power > 17 )
            {
                power = 17;
            }
            paConfig = ( paConfig & RF_PACONFIG_OUTPUTPOWER_MASK ) | ( uint8_t )( ( uint16_t )( power - 2 ) & 0x0F );
        }
    }
    else
    {
        if( power < -1 )
        {
            power = -1;
        }
        if( power > 14 )
        {
            power = 14;
        }
        paConfig = ( paConfig & RF_PACONFIG_OUTPUTPOWER_MASK ) | ( uint8_t )( ( uint16_t )( power + 1 ) & 0x0F );
    }
		SX1276Write( REG_PACONFIG, paConfig ); //B0
    SX1276Write( REG_PADAC, paDac ); //87
		
    switch( modem )
    {
    case MODEM_FSK:
        {
            SX1276.Settings.Fsk.Power = power;
            SX1276.Settings.Fsk.Fdev = fdev;
            SX1276.Settings.Fsk.Bandwidth = bandwidth;
            SX1276.Settings.Fsk.Datarate = datarate;
            SX1276.Settings.Fsk.PreambleLen = preambleLen;
            SX1276.Settings.Fsk.FixLen = fixLen;
            SX1276.Settings.Fsk.CrcOn = crcOn;
            SX1276.Settings.Fsk.IqInverted = iqInverted;
            SX1276.Settings.Fsk.TxTimeout = timeout;
            
            fdev = ( uint16_t )( ( double )fdev / ( double )FREQ_STEP );
            SX1276Write( REG_FDEVMSB, ( uint8_t )( fdev >> 8 ) );
            SX1276Write( REG_FDEVLSB, ( uint8_t )( fdev & 0xFF ) );

            datarate = ( uint16_t )( ( double )XTAL_FREQ / ( double )datarate );
            SX1276Write( REG_BITRATEMSB, ( uint8_t )( datarate >> 8 ) );
            SX1276Write( REG_BITRATELSB, ( uint8_t )( datarate & 0xFF ) );

            SX1276Write( REG_PREAMBLEMSB, ( preambleLen >> 8 ) & 0x00FF );
            SX1276Write( REG_PREAMBLELSB, preambleLen & 0xFF );

            SX1276Write( REG_PACKETCONFIG1,
                         ( SX1276Read( REG_PACKETCONFIG1 ) & 
                           RF_PACKETCONFIG1_CRC_MASK &
                           RF_PACKETCONFIG1_PACKETFORMAT_MASK ) |
                           ( ( fixLen == 1 ) ? RF_PACKETCONFIG1_PACKETFORMAT_FIXED : RF_PACKETCONFIG1_PACKETFORMAT_VARIABLE ) |
                           ( crcOn << 4 ) );
        
        }
        break;
    case MODEM_LORA:
        {
            SX1276.Settings.LoRa.Power = power;
            if( bandwidth > 2 )
            {
                // Fatal error: When using LoRa modem only bandwidths 125, 250 and 500 kHz are supported
                while( 1 );
            }
            bandwidth += 7;
            SX1276.Settings.LoRa.Bandwidth = bandwidth;
            SX1276.Settings.LoRa.Datarate = datarate;
            SX1276.Settings.LoRa.Coderate = coderate;
            SX1276.Settings.LoRa.PreambleLen = preambleLen;
            SX1276.Settings.LoRa.FixLen = fixLen;
            SX1276.Settings.LoRa.FreqHopOn = freqHopOn;
            SX1276.Settings.LoRa.HopPeriod = hopPeriod;
            SX1276.Settings.LoRa.CrcOn = crcOn;
            SX1276.Settings.LoRa.IqInverted = iqInverted;
            SX1276.Settings.LoRa.TxTimeout = timeout;

            if( datarate > 12 )
            {
                datarate = 12;
            }
            else if( datarate < 6 )
            {
                datarate = 6;
            }
            if( ( ( bandwidth == 7 ) && ( ( datarate == 11 ) || ( datarate == 12 ) ) ) ||
                ( ( bandwidth == 8 ) && ( datarate == 12 ) ) )
            {
                SX1276.Settings.LoRa.LowDatarateOptimize = 0x01;
            }
            else
            {
                SX1276.Settings.LoRa.LowDatarateOptimize = 0x00;
            }

            if( SX1276.Settings.LoRa.FreqHopOn == true )
            {
                SX1276Write( REG_LR_PLLHOP, ( SX1276Read( REG_LR_PLLHOP ) & RFLR_PLLHOP_FASTHOP_MASK ) | RFLR_PLLHOP_FASTHOP_ON );
                SX1276Write( REG_LR_HOPPERIOD, SX1276.Settings.LoRa.HopPeriod );
            }

            SX1276Write( REG_LR_MODEMCONFIG1, 
                         ( SX1276Read( REG_LR_MODEMCONFIG1 ) &
                           RFLR_MODEMCONFIG1_BW_MASK &
                           RFLR_MODEMCONFIG1_CODINGRATE_MASK &
                           RFLR_MODEMCONFIG1_IMPLICITHEADER_MASK ) |
                           ( bandwidth << 4 ) | ( coderate << 1 ) | 
                           fixLen );

            SX1276Write( REG_LR_MODEMCONFIG2,
                         ( SX1276Read( REG_LR_MODEMCONFIG2 ) &
                           RFLR_MODEMCONFIG2_SF_MASK &
                           RFLR_MODEMCONFIG2_RXPAYLOADCRC_MASK ) |
                           ( datarate << 4 ) | ( crcOn << 2 ) );

            SX1276Write( REG_LR_MODEMCONFIG3, 
                         ( SX1276Read( REG_LR_MODEMCONFIG3 ) &
                           RFLR_MODEMCONFIG3_LOWDATARATEOPTIMIZE_MASK ) |
                           ( SX1276.Settings.LoRa.LowDatarateOptimize << 3 ) );
        
            SX1276Write( REG_LR_PREAMBLEMSB, ( preambleLen >> 8 ) & 0x00FF );
            SX1276Write( REG_LR_PREAMBLELSB, preambleLen & 0xFF );
            
            if( datarate == 6 )
            {
                SX1276Write( REG_LR_DETECTOPTIMIZE, 
                             ( SX1276Read( REG_LR_DETECTOPTIMIZE ) &
                               RFLR_DETECTIONOPTIMIZE_MASK ) |
                               RFLR_DETECTIONOPTIMIZE_SF6 );
                SX1276Write( REG_LR_DETECTIONTHRESHOLD, 
                             RFLR_DETECTIONTHRESH_SF6 );
            }
            else
            {
                SX1276Write( REG_LR_DETECTOPTIMIZE,
                             ( SX1276Read( REG_LR_DETECTOPTIMIZE ) &
                             RFLR_DETECTIONOPTIMIZE_MASK ) |
                             RFLR_DETECTIONOPTIMIZE_SF7_TO_SF12 );
                SX1276Write( REG_LR_DETECTIONTHRESHOLD, 
                             RFLR_DETECTIONTHRESH_SF7_TO_SF12 );
            }
        }
        break;
    }
    SEGGER_RTT_WriteString(0, "SX1276 TXCONFIG SET!\n");
}

uint32_t SX1276GetTimeOnAir( RadioModems_t modem, uint8_t pktLen )
{
    uint32_t airTime = 0;

    switch( modem )
    {
    case MODEM_FSK:
        {
            airTime = round( ( 8 * ( SX1276.Settings.Fsk.PreambleLen +
                                     ( ( SX1276Read( REG_SYNCCONFIG ) & ~RF_SYNCCONFIG_SYNCSIZE_MASK ) + 1 ) +
                                     ( ( SX1276.Settings.Fsk.FixLen == 0x01 ) ? 0.0 : 1.0 ) +
                                     ( ( ( SX1276Read( REG_PACKETCONFIG1 ) & ~RF_PACKETCONFIG1_ADDRSFILTERING_MASK ) != 0x00 ) ? 1.0 : 0 ) +
                                     pktLen +
                                     ( ( SX1276.Settings.Fsk.CrcOn == 0x01 ) ? 2.0 : 0 ) ) /
                                     SX1276.Settings.Fsk.Datarate ) * 1e6 );
        }
        break;
    case MODEM_LORA:
        {
            double bw = 0.0;
            // REMARK: When using LoRa modem only bandwidths 125, 250 and 500 kHz are supported
            switch( SX1276.Settings.LoRa.Bandwidth )
            {
            //case 0: // 7.8 kHz
            //    bw = 78e2;
            //    break;
            //case 1: // 10.4 kHz
            //    bw = 104e2;
            //    break;
            //case 2: // 15.6 kHz
            //    bw = 156e2;
            //    break;
            //case 3: // 20.8 kHz
            //    bw = 208e2;
            //    break;
            //case 4: // 31.2 kHz
            //    bw = 312e2;
            //    break;
            //case 5: // 41.4 kHz
            //    bw = 414e2;
            //    break;
            //case 6: // 62.5 kHz
            //    bw = 625e2;
            //    break;
            case 7: // 125 kHz
                bw = 125e3;
                break;
            case 8: // 250 kHz
                bw = 250e3;
                break;
            case 9: // 500 kHz
                bw = 500e3;
                break;
            }

            // Symbol rate : time for one symbol (secs)
            double rs = bw / ( 1 << SX1276.Settings.LoRa.Datarate );
            double ts = 1 / rs;
            // time of preamble
            double tPreamble = ( SX1276.Settings.LoRa.PreambleLen + 4.25 ) * ts;
            // Symbol length of payload and time
            double tmp = ceil( ( 8 * pktLen - 4 * SX1276.Settings.LoRa.Datarate +
                                 28 + 16 * SX1276.Settings.LoRa.CrcOn -
                                 ( SX1276.Settings.LoRa.FixLen ? 20 : 0 ) ) /
                                 ( double )( 4 * SX1276.Settings.LoRa.Datarate -
                                 ( ( SX1276.Settings.LoRa.LowDatarateOptimize > 0 ) ? 2 : 0 ) ) ) *
                                 ( SX1276.Settings.LoRa.Coderate + 4 );
            double nPayload = 8 + ( ( tmp > 0 ) ? tmp : 0 );
            double tPayload = nPayload * ts;
            // Time on air 
            double tOnAir = tPreamble + tPayload;
            // return us secs
            airTime = floor( tOnAir * 1e6 + 0.999 );
        }
        break;
    }
    return airTime;
}

void SX1276Send( uint8_t *buffer, uint8_t size )
{
    uint32_t txTimeout = 0;

    SEGGER_RTT_WriteString(0,"SX1276 Sending!\n");

    switch( SX1276.Settings.Modem )
    {
    case MODEM_FSK:
        {
            SX1276.Settings.FskPacketHandler.NbBytes = 0;
            SX1276.Settings.FskPacketHandler.Size = size;

            if( SX1276.Settings.Fsk.FixLen == false )
            {
                SX1276WriteFifo( ( uint8_t* )&size, 1 );
            }
            else
            {
                SX1276Write( REG_PAYLOADLENGTH, size );
            }            
            
            if( ( size > 0 ) && ( size <= 64 ) )
            {
                SX1276.Settings.FskPacketHandler.ChunkSize = size;
            }
            else
            {
                SX1276.Settings.FskPacketHandler.ChunkSize = 32;
            }

            // Write payload buffer
            SX1276WriteFifo( buffer, SX1276.Settings.FskPacketHandler.ChunkSize );
            SX1276.Settings.FskPacketHandler.NbBytes += SX1276.Settings.FskPacketHandler.ChunkSize;
            txTimeout = SX1276.Settings.Fsk.TxTimeout;
        }
        break;
    case MODEM_LORA:
        {
            if( SX1276.Settings.LoRa.IqInverted == true )
            {
                SX1276Write( REG_LR_INVERTIQ, ( ( SX1276Read( REG_LR_INVERTIQ ) & RFLR_INVERTIQ_TX_MASK & RFLR_INVERTIQ_RX_MASK ) | RFLR_INVERTIQ_RX_OFF | RFLR_INVERTIQ_TX_ON ) );
                SX1276Write( REG_LR_INVERTIQ2, RFLR_INVERTIQ2_ON );
            }
            else
            {
                SX1276Write( REG_LR_INVERTIQ, ( ( SX1276Read( REG_LR_INVERTIQ ) & RFLR_INVERTIQ_TX_MASK & RFLR_INVERTIQ_RX_MASK ) | RFLR_INVERTIQ_RX_OFF | RFLR_INVERTIQ_TX_OFF ) );
                SX1276Write( REG_LR_INVERTIQ2, RFLR_INVERTIQ2_OFF );
            }      
        
            SX1276.Settings.LoRaPacketHandler.Size = size;

            // Initializes the payload size
            SX1276Write( REG_LR_PAYLOADLENGTH, size );

            // Full buffer used for Tx            
            SX1276Write( REG_LR_FIFOTXBASEADDR, 0 );
            SX1276Write( REG_LR_FIFOADDRPTR, 0 );

            // FIFO operations can not take place in Sleep mode
            if( ( SX1276Read( REG_OPMODE ) & ~RF_OPMODE_MASK ) == RF_OPMODE_SLEEP )
            {
                SX1276SetStby( );
                nrf_delay_ms( 1 );
            }
            // Write payload buffer
            SX1276WriteFifo( buffer, size );
            //SEGGER_RTT_WriteString(0,"Initi Sending buffer..............\n");
            //buf_print(buffer, size); 
            txTimeout = SX1276.Settings.LoRa.TxTimeout;
            SEGGER_RTT_printf(0, " SX1276.Settings.LoRa.TxTimeout %d \r\n", txTimeout);


        }
        break;
    }
    SX1276SetTx( txTimeout );
}

void SX1276SetSleep( void )
{
    SEGGER_RTT_WriteString(0, "SX1276 SET Sleep\n");
    app_timer_stop( rx_timeout_timer );
    app_timer_stop( tx_timeout_timer );

    SX1276SetOpMode( RF_OPMODE_SLEEP );
    SX1276.Settings.State = RF_IDLE;
}

void SX1276SetStby( void )
{
    app_timer_stop( rx_timeout_timer );
    app_timer_stop( tx_timeout_timer );

    SX1276SetOpMode( RF_OPMODE_STANDBY );
    SX1276.Settings.State = RF_IDLE;
}

void SX1276SetRx( uint32_t timeout )
{
    bool rxContinuous = false;
    
    SEGGER_RTT_WriteString(0, "SX1276 SET RX\n");

    switch( SX1276.Settings.Modem )
    {
    case MODEM_FSK:
        {
            rxContinuous = SX1276.Settings.Fsk.RxContinuous;
            
            // DIO0=PayloadReady
            // DIO1=FifoLevel
            // DIO2=SyncAddr
            // DIO3=FifoEmpty
            // DIO4=Preamble
            // DIO5=ModeReady
            SX1276Write( REG_DIOMAPPING1, ( SX1276Read( REG_DIOMAPPING1 ) & RF_DIOMAPPING1_DIO0_MASK &
                                                                            RF_DIOMAPPING1_DIO1_MASK &
                                                                            RF_DIOMAPPING1_DIO2_MASK ) |
                                                                            RF_DIOMAPPING1_DIO0_00 |
                                                                            RF_DIOMAPPING1_DIO1_00 |
                                                                            RF_DIOMAPPING1_DIO2_11 );
            
            SX1276Write( REG_DIOMAPPING2, ( SX1276Read( REG_DIOMAPPING2 ) & RF_DIOMAPPING2_DIO4_MASK &
                                                                            RF_DIOMAPPING2_MAP_MASK ) | 
                                                                            RF_DIOMAPPING2_DIO4_11 |
                                                                            RF_DIOMAPPING2_MAP_PREAMBLEDETECT );
            
            SX1276.Settings.FskPacketHandler.FifoThresh = SX1276Read( REG_FIFOTHRESH ) & 0x3F;
            
            SX1276.Settings.FskPacketHandler.PreambleDetected = false;
            SX1276.Settings.FskPacketHandler.SyncWordDetected = false;
            SX1276.Settings.FskPacketHandler.NbBytes = 0;
            SX1276.Settings.FskPacketHandler.Size = 0;
        }
        break;
    case MODEM_LORA:
        {
            if( SX1276.Settings.LoRa.IqInverted == true )
            {
                SX1276Write( REG_LR_INVERTIQ, ( ( SX1276Read( REG_LR_INVERTIQ ) & RFLR_INVERTIQ_TX_MASK & RFLR_INVERTIQ_RX_MASK ) | RFLR_INVERTIQ_RX_ON | RFLR_INVERTIQ_TX_OFF ) );
                SX1276Write( REG_LR_INVERTIQ2, RFLR_INVERTIQ2_ON );
            }
            else
            {
                SX1276Write( REG_LR_INVERTIQ, ( ( SX1276Read( REG_LR_INVERTIQ ) & RFLR_INVERTIQ_TX_MASK & RFLR_INVERTIQ_RX_MASK ) | RFLR_INVERTIQ_RX_OFF | RFLR_INVERTIQ_TX_OFF ) );
                SX1276Write( REG_LR_INVERTIQ2, RFLR_INVERTIQ2_OFF );
            }         

            // ERRATA 2.3 - Receiver Spurious Reception of a LoRa Signal
            if( SX1276.Settings.LoRa.Bandwidth < 9 )
            {
                SX1276Write( REG_LR_DETECTOPTIMIZE, SX1276Read( REG_LR_DETECTOPTIMIZE ) & 0x7F );
                SX1276Write( REG_LR_TEST30, 0x00 );
                switch( SX1276.Settings.LoRa.Bandwidth )
                {
                case 0: // 7.8 kHz
                    SX1276Write( REG_LR_TEST2F, 0x48 );
                    SX1276SetChannel(SX1276.Settings.Channel + 7.81e3 );
                    break;
                case 1: // 10.4 kHz
                    SX1276Write( REG_LR_TEST2F, 0x44 );
                    SX1276SetChannel(SX1276.Settings.Channel + 10.42e3 );
                    break;
                case 2: // 15.6 kHz
                    SX1276Write( REG_LR_TEST2F, 0x44 );
                    SX1276SetChannel(SX1276.Settings.Channel + 15.62e3 );
                    break;
                case 3: // 20.8 kHz
                    SX1276Write( REG_LR_TEST2F, 0x44 );
                    SX1276SetChannel(SX1276.Settings.Channel + 20.83e3 );
                    break;
                case 4: // 31.2 kHz
                    SX1276Write( REG_LR_TEST2F, 0x44 );
                    SX1276SetChannel(SX1276.Settings.Channel + 31.25e3 );
                    break;
                case 5: // 41.4 kHz
                    SX1276Write( REG_LR_TEST2F, 0x44 );
                    SX1276SetChannel(SX1276.Settings.Channel + 41.67e3 );
                    break;
                case 6: // 62.5 kHz
                    SX1276Write( REG_LR_TEST2F, 0x40 );
                    break;
                case 7: // 125 kHz
                    SX1276Write( REG_LR_TEST2F, 0x40 );
                    break;
                case 8: // 250 kHz
                    SX1276Write( REG_LR_TEST2F, 0x40 );
                    break;
                }
            }
            else
            {
                SX1276Write( REG_LR_DETECTOPTIMIZE, SX1276Read( REG_LR_DETECTOPTIMIZE ) | 0x80 );
            }

            rxContinuous = SX1276.Settings.LoRa.RxContinuous;
            
            if( SX1276.Settings.LoRa.FreqHopOn == true )
            {
                SX1276Write( REG_LR_IRQFLAGSMASK, //RFLR_IRQFLAGS_RXTIMEOUT |
                                                  //RFLR_IRQFLAGS_RXDONE |
                                                  //RFLR_IRQFLAGS_PAYLOADCRCERROR |
                                                  RFLR_IRQFLAGS_VALIDHEADER |
                                                  RFLR_IRQFLAGS_TXDONE |
                                                  RFLR_IRQFLAGS_CADDONE |
                                                  //RFLR_IRQFLAGS_FHSSCHANGEDCHANNEL |
                                                  RFLR_IRQFLAGS_CADDETECTED );
                                              
                // DIO0=RxDone, DIO2=FhssChangeChannel
                SX1276Write( REG_DIOMAPPING1, ( SX1276Read( REG_DIOMAPPING1 ) & RFLR_DIOMAPPING1_DIO0_MASK & RFLR_DIOMAPPING1_DIO2_MASK  ) | RFLR_DIOMAPPING1_DIO0_00 | RFLR_DIOMAPPING1_DIO2_00 );
            }
            else
            {
                SX1276Write( REG_LR_IRQFLAGSMASK, //RFLR_IRQFLAGS_RXTIMEOUT |
                                                  //RFLR_IRQFLAGS_RXDONE |
                                                  //RFLR_IRQFLAGS_PAYLOADCRCERROR |
                                                  RFLR_IRQFLAGS_VALIDHEADER |
                                                  RFLR_IRQFLAGS_TXDONE |
                                                  RFLR_IRQFLAGS_CADDONE |
                                                  RFLR_IRQFLAGS_FHSSCHANGEDCHANNEL |
                                                  RFLR_IRQFLAGS_CADDETECTED );
                                              
                // DIO0=RxDone
                SX1276Write( REG_DIOMAPPING1, ( SX1276Read( REG_DIOMAPPING1 ) & RFLR_DIOMAPPING1_DIO0_MASK ) | RFLR_DIOMAPPING1_DIO0_00 );
            }
            SX1276Write( REG_LR_FIFORXBASEADDR, 0 );
            SX1276Write( REG_LR_FIFOADDRPTR, 0 );
        }
        break;
    }

    memset( RxTxBuffer, 0, ( size_t )RX_BUFFER_SIZE );


    SX1276.Settings.State = RF_RX_RUNNING;
    SEGGER_RTT_WriteString(0, "SX1276 LORA RX\n");
    SEGGER_RTT_printf(0, " SX1276.Settings.State %d \r\n", SX1276.Settings.State);
    //timeout = 10000;
    if( timeout != 0 )
    {
        //SEGGER_RTT_WriteString(0, "rx_timeout_timer\n");
        
        SEGGER_RTT_printf(0, " rx_timeout_timer timeout %d \r\n", timeout);
        
        app_timer_start(rx_timeout_timer,APP_TIMER_TICKS(timeout,0),NULL);
    }

    SEGGER_RTT_printf(0, " SX1276.Settings.Modem %d \r\n", SX1276.Settings.Modem);
    if( SX1276.Settings.Modem == MODEM_FSK )
    {
        SX1276SetOpMode( RF_OPMODE_RECEIVER );
        
        if( rxContinuous == false )
        {
             //app_timer_start(rx_timeout_sync_word, APP_TIMER_TICKS((( 8.0 * ( SX1276.Settings.Fsk.PreambleLen +
               //                                           ( ( SX1276Read( REG_SYNCCONFIG ) &
                 //                                            ~RF_SYNCCONFIG_SYNCSIZE_MASK ) +
                   //                                       1.0 ) + 10.0 ) /
                     //                                    ( double )SX1276.Settings.Fsk.Datarate ) * 1e6), 0), NULL);
        }
    }
    else
    {
        SEGGER_RTT_printf(0, " rxContinuous %d \r\n", rxContinuous);
        //SX1276SetOpMode( RFLR_OPMODE_RECEIVER );
        //rxContinuous = true;
        if( rxContinuous == true )
        {
            SX1276SetOpMode( RFLR_OPMODE_RECEIVER );
        }
        else
        {
            SX1276SetOpMode( RFLR_OPMODE_RECEIVER_SINGLE );
        }
    }
}

void SX1276SetTx( uint32_t timeout )
{
    switch( SX1276.Settings.Modem )
    {
    case MODEM_FSK:
        {
            // DIO0=PacketSent
            // DIO1=FifoEmpty
            // DIO2=FifoFull
            // DIO3=FifoEmpty
            // DIO4=LowBat
            // DIO5=ModeReady
            SX1276Write( REG_DIOMAPPING1, ( SX1276Read( REG_DIOMAPPING1 ) & RF_DIOMAPPING1_DIO0_MASK &
                                                                            RF_DIOMAPPING1_DIO1_MASK &
                                                                            RF_DIOMAPPING1_DIO2_MASK ) |
                                                                            RF_DIOMAPPING1_DIO1_01 );

            SX1276Write( REG_DIOMAPPING2, ( SX1276Read( REG_DIOMAPPING2 ) & RF_DIOMAPPING2_DIO4_MASK &
                                                                            RF_DIOMAPPING2_MAP_MASK ) );
            SX1276.Settings.FskPacketHandler.FifoThresh = SX1276Read( REG_FIFOTHRESH ) & 0x3F;
        }
        break;
    case MODEM_LORA:
        {
            if( SX1276.Settings.LoRa.FreqHopOn == true )
            {
                SX1276Write( REG_LR_IRQFLAGSMASK, RFLR_IRQFLAGS_RXTIMEOUT |
                                                  RFLR_IRQFLAGS_RXDONE |
                                                  RFLR_IRQFLAGS_PAYLOADCRCERROR |
                                                  RFLR_IRQFLAGS_VALIDHEADER |
                                                  //RFLR_IRQFLAGS_TXDONE |
                                                  RFLR_IRQFLAGS_CADDONE |
                                                  //RFLR_IRQFLAGS_FHSSCHANGEDCHANNEL |
                                                  RFLR_IRQFLAGS_CADDETECTED );
                                              
                // DIO0=TxDone, DIO2=FhssChangeChannel
                SX1276Write( REG_DIOMAPPING1, ( SX1276Read( REG_DIOMAPPING1 ) & RFLR_DIOMAPPING1_DIO0_MASK & RFLR_DIOMAPPING1_DIO2_MASK ) | RFLR_DIOMAPPING1_DIO0_01 | RFLR_DIOMAPPING1_DIO2_00 );
            }
            else
            {
                SX1276Write( REG_LR_IRQFLAGSMASK, RFLR_IRQFLAGS_RXTIMEOUT |
                                                  RFLR_IRQFLAGS_RXDONE |
                                                  RFLR_IRQFLAGS_PAYLOADCRCERROR |
                                                  RFLR_IRQFLAGS_VALIDHEADER |
                                                  //RFLR_IRQFLAGS_TXDONE |
                                                  RFLR_IRQFLAGS_CADDONE |
                                                  RFLR_IRQFLAGS_FHSSCHANGEDCHANNEL |
                                                  RFLR_IRQFLAGS_CADDETECTED );

                // DIO0=TxDone
                SX1276Write( REG_DIOMAPPING1, ( SX1276Read( REG_DIOMAPPING1 ) & RFLR_DIOMAPPING1_DIO0_MASK ) | RFLR_DIOMAPPING1_DIO0_01 );
            }
        }
        break;
    }

    SX1276.Settings.State = RF_TX_RUNNING;
    app_timer_stop(tx_timeout_timer);
    //timeout = 10000;
    SEGGER_RTT_printf(0, " tx_timeout_timer %d \r\n", timeout);

    app_timer_start(tx_timeout_timer,APP_TIMER_TICKS(timeout,0),NULL);
    SX1276SetOpMode( RF_OPMODE_TRANSMITTER );
    //SEGGER_RTT_printf(0, " tx_timeout_timer %d \r\n", timeout);
}

void SX1276StartCad( void )
{
    switch( SX1276.Settings.Modem )
    {
    case MODEM_FSK:
        {
           
        }
        break;
    case MODEM_LORA:
        {
            SX1276Write( REG_LR_IRQFLAGSMASK, RFLR_IRQFLAGS_RXTIMEOUT |
                                        RFLR_IRQFLAGS_RXDONE |
                                        RFLR_IRQFLAGS_PAYLOADCRCERROR |
                                        RFLR_IRQFLAGS_VALIDHEADER |
                                        RFLR_IRQFLAGS_TXDONE |
                                        //RFLR_IRQFLAGS_CADDONE |
                                        RFLR_IRQFLAGS_FHSSCHANGEDCHANNEL // |
                                        //RFLR_IRQFLAGS_CADDETECTED 
                                        );
                                          
            // DIO3=CADDone
            SX1276Write( REG_DIOMAPPING1, ( SX1276Read( REG_DIOMAPPING1 ) & RFLR_DIOMAPPING1_DIO0_MASK ) | RFLR_DIOMAPPING1_DIO0_00 );
            
            SX1276.Settings.State = RF_CAD;
            SX1276SetOpMode( RFLR_OPMODE_CAD );
        }
        break;
    default:
        break;
    }
}

int16_t SX1276ReadRssi( RadioModems_t modem )
{
    int16_t rssi = 0;

    switch( modem )
    {
    case MODEM_FSK:
        rssi = -( SX1276Read( REG_RSSIVALUE ) >> 1 );
        break;
    case MODEM_LORA:
        if( SX1276.Settings.Channel > RF_MID_BAND_THRESH )
        {
            rssi = RSSI_OFFSET_HF + SX1276Read( REG_LR_RSSIVALUE );
        }
        else
        {
            rssi = RSSI_OFFSET_LF + SX1276Read( REG_LR_RSSIVALUE );
        }
        break;
    default:
        rssi = -1;
        break;
    }
    return rssi;
}

void SX1276Reset( void )
{
    SEGGER_RTT_WriteString(0, "SX1276 RESET!\n");

    // Set RESET pin to 0
    nrf_gpio_cfg_output( SX1276.Reset );
    nrf_gpio_pin_clear( SX1276.Reset );

    // Wait 1 ms
    nrf_delay_ms( 1 );

    // Configure RESET as input
    nrf_gpio_cfg_input( SX1276.Reset, NRF_GPIO_PIN_NOPULL );
    nrf_gpio_pin_set( SX1276.Reset );

    // Wait 6 ms
    nrf_delay_ms( 6 );
}

void SX1276SetOpMode( uint8_t opMode )
{
    static uint8_t opModePrev = RF_OPMODE_STANDBY;

    if( opMode != opModePrev )
    {
        opModePrev = opMode;
        if( opMode == RF_OPMODE_SLEEP )
        {
            SX1276SetAntSwLowPower( true );
        }
        else
        {
            SX1276SetAntSwLowPower( false );
            if( opMode == RF_OPMODE_TRANSMITTER )
            {
                SEGGER_RTT_WriteString(0, "RF_OPMODE_TRANSMITTER\n");
                 SX1276SetAntSw( 1 );
            }
            else
            { 
                SEGGER_RTT_WriteString(0, "NO RF_OPMODE_TRANSMITTER\n");
                 SX1276SetAntSw( 0 );
            }
        }
        SX1276Write( REG_OPMODE, ( SX1276Read( REG_OPMODE ) & RF_OPMODE_MASK ) | opMode );
    }
}

void SX1276SetModem( RadioModems_t modem )
{
    if( SX1276.Spi == 0 )
    {
        while( 1 );
    }
    if( SX1276.Settings.Modem == modem )
    {
        return;
    }

    SEGGER_RTT_WriteString(0, "Setting Modem!\n");

    SX1276.Settings.Modem = modem;
    switch( SX1276.Settings.Modem )
    {
    default:
    case MODEM_FSK:
        SEGGER_RTT_WriteString(0, "Setting MODEM_FSK!\n");
        SX1276SetOpMode( RF_OPMODE_SLEEP );
        SX1276Write( REG_OPMODE, ( SX1276Read( REG_OPMODE ) & RFLR_OPMODE_LONGRANGEMODE_MASK ) | RFLR_OPMODE_LONGRANGEMODE_OFF );
    
        SX1276Write( REG_DIOMAPPING1, 0x00 );
        SX1276Write( REG_DIOMAPPING2, 0x30 ); // DIO5=ModeReady
        break;
    case MODEM_LORA:
        SEGGER_RTT_WriteString(0, "Setting MODEM_LORA!\n");
        SX1276SetOpMode( RF_OPMODE_SLEEP );
        SX1276Write( REG_OPMODE, ( SX1276Read( REG_OPMODE ) & RFLR_OPMODE_LONGRANGEMODE_MASK ) | RFLR_OPMODE_LONGRANGEMODE_ON );

        SX1276Write( REG_DIOMAPPING1, 0x00 );
        SX1276Write( REG_DIOMAPPING2, 0x00 );
        break;
    }
}

void SX1276Write( uint8_t addr, uint8_t data )
{
    SX1276WriteBuffer( addr, &data, 1 );
}

uint8_t SX1276Read( uint8_t addr )
{
    uint8_t data;
    SX1276ReadBuffer( addr, &data, 1 );
    return data;
}

void SX1276WriteBuffer( uint8_t addr, uint8_t *buffer, uint8_t size )
{
    uint8_t i;

    //SEGGER_RTT_WriteString(0,"SX Writing Sending buffer..............\n");
    //buf_print(buffer, size); 
    //SEGGER_RTT_WriteString(0,"\n------------------------------------\n");

    //NSS = 0;
    nrf_gpio_pin_clear(SPIM0_SS_PIN);

    m_tx_data[0] = (addr | 0x80);

    for (i = 0 ; i < size+1 ; i++){
        m_tx_data[i+1] = buffer[i];
    }

    //SEGGER_RTT_WriteString(0,"SX Writing Sending buffer\n");
    //SEGGER_RTT_WriteString(0,"Writing: \n  Address:");
    //buf_print(m_tx_data,1);
    //SEGGER_RTT_WriteString(0,"\n  Buffer:");
    //buf_print(&m_tx_data[1],size);
    //SEGGER_RTT_WriteString(0,"\n----------\n");

    spi_send_recv(m_tx_data,m_rx_data, size + 1);
    //SEGGER_RTT_WriteString(0,"SX Writing Sending buffer end\n");
    
    //NSS = 1;
    nrf_gpio_pin_set(SPIM0_SS_PIN);
}

void SX1276ReadBuffer( uint8_t addr, uint8_t *buffer, uint8_t size )
{
    uint8_t i;


    //NSS = 0;
    nrf_gpio_pin_clear(SPIM0_SS_PIN);


    m_tx_data[0] = (addr & 0x7F);

    spi_send_recv(m_tx_data,m_rx_data,size + 1);
    
    for (i = 0 ; i < size ; i++){
        buffer[i] = m_rx_data[i+1];
    }

    //SEGGER_RTT_WriteString(0,"Reading: \n  Address:");
    //buf_print(m_tx_data,1);
    //SEGGER_RTT_WriteString(0,"\n  Buffer:");
    //buf_print(buffer,size);
    //SEGGER_RTT_WriteString(0,"\n");

    //NSS = 1;
    nrf_gpio_pin_set(SPIM0_SS_PIN);
}

void SX1276WriteFifo( uint8_t *buffer, uint8_t size )
{
    
    SX1276WriteBuffer( 0, buffer, size );
}

void SX1276ReadFifo( uint8_t *buffer, uint8_t size )
{
    SX1276ReadBuffer( 0, buffer, size );
}

void SX1276SetMaxPayloadLength( RadioModems_t modem, uint8_t max )
{
    //SX1276SetModem( modem );

    switch( modem )
    {
    case MODEM_FSK:
        if( SX1276.Settings.Fsk.FixLen == false )
        {
            SX1276Write( REG_PAYLOADLENGTH, max );
        }
        break;
    case MODEM_LORA:
        SX1276Write( REG_LR_PAYLOADMAXLENGTH, max );
        break;
    }
}

void SX1276OnTimeoutIrq( void * p_context )
{
    SEGGER_RTT_WriteString(0,"SX1276OnTimeoutIrq!\n");
    switch( SX1276.Settings.State )
    {
    case RF_RX_RUNNING:
        SEGGER_RTT_WriteString(0,"RF_RX_RUNNING\n");
        if( SX1276.Settings.Modem == MODEM_FSK )
        {
            SX1276.Settings.FskPacketHandler.PreambleDetected = false;
            SX1276.Settings.FskPacketHandler.SyncWordDetected = false;
            SX1276.Settings.FskPacketHandler.NbBytes = 0;
            SX1276.Settings.FskPacketHandler.Size = 0;

            // Clear Irqs
            SX1276Write( REG_IRQFLAGS1, RF_IRQFLAGS1_RSSI | 
                                        RF_IRQFLAGS1_PREAMBLEDETECT |
                                        RF_IRQFLAGS1_SYNCADDRESSMATCH );
            SX1276Write( REG_IRQFLAGS2, RF_IRQFLAGS2_FIFOOVERRUN );

            if( SX1276.Settings.Fsk.RxContinuous == true )
            {
                // Continuous mode restart Rx chain
                SX1276Write( REG_RXCONFIG, SX1276Read( REG_RXCONFIG ) | RF_RXCONFIG_RESTARTRXWITHOUTPLLLOCK );
            }
            else
            {
                SX1276.Settings.State = RF_IDLE;
                //app_timer_stop( rx_timeout_sync_word );
            }
        }
        //State = RX_TIMEOUT;
        if( RadioEvents.RxTimeout != NULL ) 
        {
            RadioEvents.RxTimeout( );
        }
        break;
    case RF_TX_RUNNING:
        SEGGER_RTT_WriteString(0,"RF_TX_RUNNING\n");
        SX1276.Settings.State = RF_IDLE;
        //State = TX_TIMEOUT;
        if(  RadioEvents.TxTimeout != NULL ) 
        {
            RadioEvents.TxTimeout( );
        }
        break;
    default:
        break;
    }

    SEGGER_RTT_WriteString(0,"Leaving timeout\n");
}

void SX1276OnDio0Irq( void * p_event_data, uint16_t event_size  )
{
    volatile uint8_t irqFlags = 0;
    SEGGER_RTT_WriteString(0,"!!!!!enter Dio0IRQ!!!!!\n"); 
    SEGGER_RTT_printf(0, " state %d \r\n", SX1276.Settings.State);
    switch( SX1276.Settings.State )
    {         
        //SEGGER_RTT_WriteString(0,"***********switch!!!!!*********\n");       
        case RF_RX_RUNNING:
            SEGGER_RTT_WriteString(0,"!!!!!RF_RX_RUNNING!!!!!\n");
            //SEGGER_RTT_WriteString(0,"***********RxIRQ*********\n"); 
            //TimerStop( &RxTimeoutTimer );
            // RxDone interrupt
            switch( SX1276.Settings.Modem )
            {
            case MODEM_FSK:
                if( SX1276.Settings.Fsk.CrcOn == true )
                {
                    irqFlags = SX1276Read( REG_IRQFLAGS2 );
                    if( ( irqFlags & RF_IRQFLAGS2_CRCOK ) != RF_IRQFLAGS2_CRCOK )
                    {
                        // Clear Irqs
                        SX1276Write( REG_IRQFLAGS1, RF_IRQFLAGS1_RSSI | 
                                                    RF_IRQFLAGS1_PREAMBLEDETECT |
                                                    RF_IRQFLAGS1_SYNCADDRESSMATCH );
                        SX1276Write( REG_IRQFLAGS2, RF_IRQFLAGS2_FIFOOVERRUN );

                        if( SX1276.Settings.Fsk.RxContinuous == false )
                        {
                            SX1276.Settings.State = RF_IDLE;
                            // TimerStart( &RxTimeoutSyncWord );
                        }
                        else
                        {
                            // Continuous mode restart Rx chain
                            SX1276Write( REG_RXCONFIG, SX1276Read( REG_RXCONFIG ) | RF_RXCONFIG_RESTARTRXWITHOUTPLLLOCK );
                        }
                        app_timer_stop( rx_timeout_timer );
                        //State = RX_ERROR;
                        
                        if(  RadioEvents.RxError != NULL ) 
                        {
                            RadioEvents.RxError( );
                        }
    
                        SX1276.Settings.FskPacketHandler.PreambleDetected = false;
                        SX1276.Settings.FskPacketHandler.SyncWordDetected = false;
                        SX1276.Settings.FskPacketHandler.NbBytes = 0;
                        SX1276.Settings.FskPacketHandler.Size = 0;
                        break;
                    }
                }
                
                // Read received packet size
                if( ( SX1276.Settings.FskPacketHandler.Size == 0 ) && ( SX1276.Settings.FskPacketHandler.NbBytes == 0 ) )
                {
                    if( SX1276.Settings.Fsk.FixLen == false )
                    {
                        SX1276ReadFifo( ( uint8_t* )&SX1276.Settings.FskPacketHandler.Size, 1 );
                    }
                    else
                    {
                        SX1276.Settings.FskPacketHandler.Size = SX1276Read( REG_PAYLOADLENGTH );
                    }
                    SX1276ReadFifo( RxTxBuffer + SX1276.Settings.FskPacketHandler.NbBytes, SX1276.Settings.FskPacketHandler.Size - SX1276.Settings.FskPacketHandler.NbBytes );
                    SX1276.Settings.FskPacketHandler.NbBytes += ( SX1276.Settings.FskPacketHandler.Size - SX1276.Settings.FskPacketHandler.NbBytes );
                }
                else
                {
                    SX1276ReadFifo( RxTxBuffer + SX1276.Settings.FskPacketHandler.NbBytes, SX1276.Settings.FskPacketHandler.Size - SX1276.Settings.FskPacketHandler.NbBytes );
                    SX1276.Settings.FskPacketHandler.NbBytes += ( SX1276.Settings.FskPacketHandler.Size - SX1276.Settings.FskPacketHandler.NbBytes );
                }

                if( SX1276.Settings.Fsk.RxContinuous == false )
                {
                    SX1276.Settings.State = RF_IDLE;
                    // TimerStart( &RxTimeoutSyncWord );
                }
                else
                {
                    // Continuous mode restart Rx chain
                    SX1276Write( REG_RXCONFIG, SX1276Read( REG_RXCONFIG ) | RF_RXCONFIG_RESTARTRXWITHOUTPLLLOCK );
                }
                app_timer_stop( rx_timeout_timer );

                //State = RX;
                if( RadioEvents.RxDone != NULL ) 
                {
                    RadioEvents.RxDone( RxTxBuffer, SX1276.Settings.FskPacketHandler.Size, SX1276.Settings.FskPacketHandler.RssiValue, 0 );
                }
               
                SX1276.Settings.FskPacketHandler.PreambleDetected = false;
                SX1276.Settings.FskPacketHandler.SyncWordDetected = false;
                SX1276.Settings.FskPacketHandler.NbBytes = 0;
                SX1276.Settings.FskPacketHandler.Size = 0;
                break;
            case MODEM_LORA:
                {
                    int8_t snr = 0;

                    SEGGER_RTT_WriteString(0,"RF_RX_RUNNING SX1276OnDio0Irq\n");
                    // Clear Irq
                    SX1276Write( REG_LR_IRQFLAGS, RFLR_IRQFLAGS_RXDONE );

                    irqFlags = SX1276Read( REG_LR_IRQFLAGS );

                    if( ( irqFlags & RFLR_IRQFLAGS_PAYLOADCRCERROR_MASK ) == RFLR_IRQFLAGS_PAYLOADCRCERROR )
                    {
                        // Clear Irq
                        SX1276Write( REG_LR_IRQFLAGS, RFLR_IRQFLAGS_PAYLOADCRCERROR );

                        if( SX1276.Settings.LoRa.RxContinuous == false )
                        {
                            SX1276.Settings.State = RF_IDLE;
                        }
                        app_timer_stop( rx_timeout_timer );
                        
                        //State = RX_ERROR;
                        if( RadioEvents.RxError != NULL ) 
                        {
                            RadioEvents.RxError( );
                        }

                        break;
                    }

                    SX1276.Settings.LoRaPacketHandler.SnrValue = SX1276Read( REG_LR_PKTSNRVALUE );
                    if( SX1276.Settings.LoRaPacketHandler.SnrValue & 0x80 ) // The SNR sign bit is 1
                    {
                        // Invert and divide by 4
                        snr = ( ( ~SX1276.Settings.LoRaPacketHandler.SnrValue + 1 ) & 0xFF ) >> 2;
                        snr = -snr;
                    }
                    else
                    {
                        // Divide by 4
                        snr = ( SX1276.Settings.LoRaPacketHandler.SnrValue & 0xFF ) >> 2;
                    }

                    int16_t rssi = SX1276Read( REG_LR_PKTRSSIVALUE );
                    if( snr < 0 )
                    {
                        if( SX1276.Settings.Channel > RF_MID_BAND_THRESH )
                        {
                            SX1276.Settings.LoRaPacketHandler.RssiValue = RSSI_OFFSET_HF + rssi + ( rssi >> 4 ) +
                                                                          snr;
                        }
                        else
                        {
                            SX1276.Settings.LoRaPacketHandler.RssiValue = RSSI_OFFSET_LF + rssi + ( rssi >> 4 ) +
                                                                          snr;
                        }
                    }
                    else
                    {    
                        if( SX1276.Settings.Channel > RF_MID_BAND_THRESH )
                        {
                            SX1276.Settings.LoRaPacketHandler.RssiValue = RSSI_OFFSET_HF + rssi + ( rssi >> 4 );
                        }
                        else
                        {
                            SX1276.Settings.LoRaPacketHandler.RssiValue = RSSI_OFFSET_LF + rssi + ( rssi >> 4 );
                        }
                    }

                    //SEGGER_RTT_WriteString(0,"***********Loar Buffer *********\n");
                    SX1276.Settings.LoRaPacketHandler.Size = SX1276Read( REG_LR_RXNBBYTES );
                    SX1276ReadFifo( RxTxBuffer, SX1276.Settings.LoRaPacketHandler.Size );
                    //SEGGER_RTT_WriteString(0,"*********************************\n");
                    
                    SEGGER_RTT_WriteString(0,"*********************************\n");
                    SEGGER_RTT_WriteString(0,"Buffer:");
                    buf_print(RxTxBuffer,10);
                    SEGGER_RTT_WriteString(0,"\n");

                    if( SX1276.Settings.LoRa.RxContinuous == false )
                    {
                        SX1276.Settings.State = RF_IDLE;
                    }
                    app_timer_stop( rx_timeout_timer );

                    /*RssiValue = SX1276.Settings.LoRaPacketHandler.RssiValue;
                    SnrValue = SX1276.Settings.LoRaPacketHandler.SnrValue;
                    BufferSize = SX1276.Settings.LoRaPacketHandler.Size;
                    memcpy( Buffer, RxTxBuffer, BufferSize );

                    State = RX;*/

                    if( RadioEvents.RxDone != NULL ) 
                    {
                        RadioEvents.RxDone( RxTxBuffer, SX1276.Settings.LoRaPacketHandler.Size, SX1276.Settings.LoRaPacketHandler.RssiValue, SX1276.Settings.LoRaPacketHandler.SnrValue );
                    }
                    

                }
                break;
            default:
                break;
            }
            break;
        case RF_TX_RUNNING:
            app_timer_stop( tx_timeout_timer );
            SEGGER_RTT_WriteString(0,"!!!!!RF_TX_RUNNING!!!!!\n");
            //SEGGER_RTT_printf(0, " Tx MODEM %d \r\n", SX1276.Settings.Modem);
            // TxDone interrupt
            switch( SX1276.Settings.Modem )
            {
            case MODEM_LORA:
                // Clear Irq
                //SEGGER_RTT_WriteString(0,"!!!!!MODEM_LORA!!!!!\n");
                //SEGGER_RTT_WriteString(0,"***********Tx Lora*********\n");
                SX1276Write( REG_LR_IRQFLAGS, RFLR_IRQFLAGS_TXDONE );
                // Intentional fall through
            case MODEM_FSK:
                //SEGGER_RTT_WriteString(0,"!!!!!MODEM_FSK!!!!!\n");
                //SEGGER_RTT_WriteString(0,"***********Tx Fsk*********\n");
            default:
                //SEGGER_RTT_WriteString(0,"!!!!!default!!!!!\n");
                //SEGGER_RTT_WriteString(0,"***********Tx defulat*********\n");
                SX1276.Settings.State = RF_IDLE;                
                //State = TX;

                if( RadioEvents.TxDone != NULL )
                {
                    
                    RadioEvents.TxDone( );
                    //SEGGER_RTT_WriteString(0,"***********TxDoneFunction*********\n");
                    SEGGER_RTT_WriteString(0,"!!!!!TxDoneFunction!!!!!\n");
                }
                SEGGER_RTT_printf(0, " NVIC_GetPendingIRQ %d \r\n", NVIC_GetPendingIRQ(GPIOTE_IRQn));
                
                break;
            }
            break;
        default:
            break;
    }
    SEGGER_RTT_WriteString(0,"Exiting DIO0\n");
}

void SX1276OnDio1Irq( void * p_event_data, uint16_t event_size  )
{
    SEGGER_RTT_WriteString(0,"!!!!!SX1276OnDio1Irq!!!!!\n"); 
    //volatile uint8_t irqFlags = 0;
    switch( SX1276.Settings.State )
    {         
              
        case RF_RX_RUNNING:
            SEGGER_RTT_WriteString(0,"!!!!!DIO1 RF_RX_RUNNING!!!!!\n");
            switch( SX1276.Settings.Modem )
            {
            case MODEM_FSK:
                SEGGER_RTT_WriteString(0,"!!!!!DIO1 MODEM_FSK!!!!!\n"); 
                // FifoLevel interrupt
                // Read received packet size
                if( ( SX1276.Settings.FskPacketHandler.Size == 0 ) && ( SX1276.Settings.FskPacketHandler.NbBytes == 0 ) )
                {
                    if( SX1276.Settings.Fsk.FixLen == false )
                    {
                        SX1276ReadFifo( ( uint8_t* )&SX1276.Settings.FskPacketHandler.Size, 1 );
                    }
                    else
                    {
                        SX1276.Settings.FskPacketHandler.Size = SX1276Read( REG_PAYLOADLENGTH );
                    }
                }

                if( ( SX1276.Settings.FskPacketHandler.Size - SX1276.Settings.FskPacketHandler.NbBytes ) > SX1276.Settings.FskPacketHandler.FifoThresh )
                {
                    SX1276ReadFifo( ( RxTxBuffer + SX1276.Settings.FskPacketHandler.NbBytes ), SX1276.Settings.FskPacketHandler.FifoThresh );
                    SX1276.Settings.FskPacketHandler.NbBytes += SX1276.Settings.FskPacketHandler.FifoThresh;
                }
                else
                {
                    SX1276ReadFifo( ( RxTxBuffer + SX1276.Settings.FskPacketHandler.NbBytes ), SX1276.Settings.FskPacketHandler.Size - SX1276.Settings.FskPacketHandler.NbBytes );
                    SX1276.Settings.FskPacketHandler.NbBytes += ( SX1276.Settings.FskPacketHandler.Size - SX1276.Settings.FskPacketHandler.NbBytes );
                }
                break;
            case MODEM_LORA:
                NVIC_SetPriority(SPI0_TWI0_IRQn,3); 
                SEGGER_RTT_WriteString(0,"!!!!!DIO1 MODEM_LORA!!!!!\n");  
                // Sync time out
                app_timer_stop( rx_timeout_timer );
                SX1276.Settings.State = RF_IDLE;
                //State = RX_TIMEOUT;
                if(  RadioEvents.RxTimeout != NULL ) 
                {
                    RadioEvents.RxTimeout( );
                }

                /*{
                    int8_t snr = 0;

                    SEGGER_RTT_WriteString(0,"RF_RX_RUNNING SX1276OnDio0Irq\n");
                    // Clear Irq
                    SX1276Write( REG_LR_IRQFLAGS, RFLR_IRQFLAGS_RXDONE );

                    irqFlags = SX1276Read( REG_LR_IRQFLAGS );

                    if( ( irqFlags & RFLR_IRQFLAGS_PAYLOADCRCERROR_MASK ) == RFLR_IRQFLAGS_PAYLOADCRCERROR )
                    {
                        // Clear Irq
                        SX1276Write( REG_LR_IRQFLAGS, RFLR_IRQFLAGS_PAYLOADCRCERROR );

                        if( SX1276.Settings.LoRa.RxContinuous == false )
                        {
                            SX1276.Settings.State = RF_IDLE;
                        }
                        app_timer_stop( rx_timeout_timer );
                        
                        //State = RX_ERROR;
                        if( RadioEvents.RxError != NULL ) 
                        {
                            RadioEvents.RxError( );
                        }

                        break;
                    }

                    SX1276.Settings.LoRaPacketHandler.SnrValue = SX1276Read( REG_LR_PKTSNRVALUE );
                    SEGGER_RTT_printf(0, " SX1276.Settings.LoRaPacketHandler.SnrValue %d \r\n", SX1276.Settings.LoRaPacketHandler.SnrValue);
                    if( SX1276.Settings.LoRaPacketHandler.SnrValue & 0x80 ) // The SNR sign bit is 1
                    {
                        // Invert and divide by 4
                        snr = ( ( ~SX1276.Settings.LoRaPacketHandler.SnrValue + 1 ) & 0xFF ) >> 2;
                        snr = -snr;
                    }
                    else
                    {
                        // Divide by 4
                        snr = ( SX1276.Settings.LoRaPacketHandler.SnrValue & 0xFF ) >> 2;
                    }

                    int16_t rssi = SX1276Read( REG_LR_PKTRSSIVALUE );
                     SEGGER_RTT_printf(0, " rssi%d \r\n", rssi);
                    if( snr < 0 )
                    {
                        if( SX1276.Settings.Channel > RF_MID_BAND_THRESH )
                        {
                            SX1276.Settings.LoRaPacketHandler.RssiValue = RSSI_OFFSET_HF + rssi + ( rssi >> 4 ) +
                                                                          snr;
                        }
                        else
                        {
                            SX1276.Settings.LoRaPacketHandler.RssiValue = RSSI_OFFSET_LF + rssi + ( rssi >> 4 ) +
                                                                          snr;
                        }
                    }
                    else
                    {    
                        if( SX1276.Settings.Channel > RF_MID_BAND_THRESH )
                        {
                            SX1276.Settings.LoRaPacketHandler.RssiValue = RSSI_OFFSET_HF + rssi + ( rssi >> 4 );
                        }
                        else
                        {
                            SX1276.Settings.LoRaPacketHandler.RssiValue = RSSI_OFFSET_LF + rssi + ( rssi >> 4 );
                        }
                    }

                    //SEGGER_RTT_WriteString(0,"***********Loar Buffer *********\n");
                    SX1276.Settings.LoRaPacketHandler.Size = SX1276Read( REG_LR_RXNBBYTES );
                    SX1276ReadFifo( RxTxBuffer, SX1276.Settings.LoRaPacketHandler.Size );

                    SEGGER_RTT_WriteString(0,"*********************************\n");
                    SEGGER_RTT_WriteString(0,"Buffer:");
                    buf_print(RxTxBuffer,10);
                    SEGGER_RTT_WriteString(0,"\n");

                    if( SX1276.Settings.LoRa.RxContinuous == false )
                    {
                        SX1276.Settings.State = RF_IDLE;
                    }
                    app_timer_stop( rx_timeout_timer );

                    if( RadioEvents.RxDone != NULL ) 
                    {
                        RadioEvents.RxDone( RxTxBuffer, SX1276.Settings.LoRaPacketHandler.Size, SX1276.Settings.LoRaPacketHandler.RssiValue, SX1276.Settings.LoRaPacketHandler.SnrValue );
                    }
                }*/
                break;
            default:
                break;
            }
            break;
        case RF_TX_RUNNING:
            SEGGER_RTT_WriteString(0,"!!!!!DIO1 RF_TX_RUNNING!!!!!\n"); 
            switch( SX1276.Settings.Modem )
            {
            case MODEM_FSK:
                // FifoEmpty interrupt
                if( ( SX1276.Settings.FskPacketHandler.Size - SX1276.Settings.FskPacketHandler.NbBytes ) > SX1276.Settings.FskPacketHandler.ChunkSize )
                {
                    SX1276WriteFifo( ( RxTxBuffer + SX1276.Settings.FskPacketHandler.NbBytes ), SX1276.Settings.FskPacketHandler.ChunkSize );
                    SX1276.Settings.FskPacketHandler.NbBytes += SX1276.Settings.FskPacketHandler.ChunkSize;
                }
                else 
                {
                    // Write the last chunk of data
                    SX1276WriteFifo( RxTxBuffer + SX1276.Settings.FskPacketHandler.NbBytes, SX1276.Settings.FskPacketHandler.Size - SX1276.Settings.FskPacketHandler.NbBytes );
                    SX1276.Settings.FskPacketHandler.NbBytes += SX1276.Settings.FskPacketHandler.Size - SX1276.Settings.FskPacketHandler.NbBytes;
                }
                break;
            case MODEM_LORA:
                SEGGER_RTT_WriteString(0,"!!!!!DIO1 MODEM_LORA!!!!!\n"); 
                break;
            default:
                break;
            }
            break;      
        default:
            break;
    }
}

void SX1276OnDio2Irq( void * p_event_data, uint16_t event_size )
{
    switch( SX1276.Settings.State )
    {                
        case RF_RX_RUNNING:
            switch( SX1276.Settings.Modem )
            {
            case MODEM_FSK:
                if( ( SX1276.Settings.FskPacketHandler.PreambleDetected == true ) && ( SX1276.Settings.FskPacketHandler.SyncWordDetected == false ) )
                {
                    //app_timer_stop( rx_timeout_sync_word );
                    
                    SX1276.Settings.FskPacketHandler.SyncWordDetected = true;
                
                    SX1276.Settings.FskPacketHandler.RssiValue = -( SX1276Read( REG_RSSIVALUE ) >> 1 );

                    SX1276.Settings.FskPacketHandler.AfcValue = ( int32_t )( double )( ( ( uint16_t )SX1276Read( REG_AFCMSB ) << 8 ) |
                                                                           ( uint16_t )SX1276Read( REG_AFCLSB ) ) *
                                                                           ( double )FREQ_STEP;
                    SX1276.Settings.FskPacketHandler.RxGain = ( SX1276Read( REG_LNA ) >> 5 ) & 0x07;
                }
                break;
            case MODEM_LORA:
                if( SX1276.Settings.LoRa.FreqHopOn == true )
                {
                    // Clear Irq
                    SX1276Write( REG_LR_IRQFLAGS, RFLR_IRQFLAGS_FHSSCHANGEDCHANNEL );
                    
                    // if( ( RadioEvents != NULL ) && ( RadioEvents->FhssChangeChannel != NULL ) )
                    // {
                    //     RadioEvents->FhssChangeChannel( ( SX1276Read( REG_LR_HOPCHANNEL ) & RFLR_HOPCHANNEL_CHANNEL_MASK ) );
                    // }
                }
                break;
            default:
                break;
            }
            break;
        case RF_TX_RUNNING:
            switch( SX1276.Settings.Modem )
            {
            case MODEM_FSK:
                break;
            case MODEM_LORA:
                if( SX1276.Settings.LoRa.FreqHopOn == true )
                {
                    // Clear Irq
                    SX1276Write( REG_LR_IRQFLAGS, RFLR_IRQFLAGS_FHSSCHANGEDCHANNEL );
                    
                    // if( ( RadioEvents != NULL ) && ( RadioEvents->FhssChangeChannel != NULL ) )
                    // {
                    //     RadioEvents->FhssChangeChannel( ( SX1276Read( REG_LR_HOPCHANNEL ) & RFLR_HOPCHANNEL_CHANNEL_MASK ) );
                    // }
                }
                break;
            default:
                break;
            }
            break;      
        default:
            break;
    }
}

void SX1276OnDio3Irq( void * p_event_data, uint16_t event_size )
{
    switch( SX1276.Settings.Modem )
    {
    case MODEM_FSK:
        break;
    case MODEM_LORA:
        if( ( SX1276Read( REG_LR_IRQFLAGS ) & RFLR_IRQFLAGS_CADDETECTED ) == RFLR_IRQFLAGS_CADDETECTED )
        {
            // Clear Irq
            SX1276Write( REG_LR_IRQFLAGS, RFLR_IRQFLAGS_CADDETECTED | RFLR_IRQFLAGS_CADDONE );
            // if( ( RadioEvents != NULL ) && ( RadioEvents->CadDone != NULL ) )
            // {
            //     RadioEvents->CadDone( true );
            // }
        }
        else
        {        
            // Clear Irq
            SX1276Write( REG_LR_IRQFLAGS, RFLR_IRQFLAGS_CADDONE );
            // if( ( RadioEvents != NULL ) && ( RadioEvents->CadDone != NULL ) )
            // {
            //     RadioEvents->CadDone( false );
            // }
        }
        break;
    default:
        break;
    }
}

void SX1276OnDio4Irq( void * p_event_data, uint16_t event_size )
{
    switch( SX1276.Settings.Modem )
    {
    case MODEM_FSK:
        {
            if( SX1276.Settings.FskPacketHandler.PreambleDetected == false )
            {
                SX1276.Settings.FskPacketHandler.PreambleDetected = true;
            }    
        }
        break;
    case MODEM_LORA:
        break;
    default:
        break;
    }
}

void SX1276OnDio5Irq( void * p_event_data, uint16_t event_size )
{
    switch( SX1276.Settings.Modem )
    {
    case MODEM_FSK:
        break;
    case MODEM_LORA:
        break;
    default:
        break;
    }
}

void SX1276Scheduler( void ){
    app_sched_execute();
}


bool SX1276CheckRfFrequency( uint32_t frequency )
{
    // Implement check. Currently all frequencies are supported
    return true;
}

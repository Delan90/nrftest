#ifndef __MAIN_H__
#define __MAIN_H__

#define APP_TIMER_PRESCALER             0 /**< Value of the RTC1 PRESCALER register. */
#define APP_TIMER_MAX_TIMERS            12 /**< Maximum number of simultaneously created timers. */
#define APP_TIMER_OP_QUEUE_SIZE         10 /**< Size of timer operation queues. */

// Scheduler settings
#define SCHED_MAX_EVENT_DATA_SIZE       12
#define SCHED_QUEUE_SIZE                10
/*!
 * Tx and Rx timers
 */
app_timer_id_t   tx_timeout_timer;
app_timer_id_t   rx_timeout_timer;
app_timer_id_t   rx_timeout_sync_word;

RadioEvents_t RadioEvents;

extern void OnTxDone( void );

extern void OnRxDone( uint8_t *payload, uint16_t size, int16_t rssi, int8_t snr );

extern void OnTxTimeout( void );

extern void OnRxTimeout( void );

extern void OnRxError( void );


#endif // __MAIN_H__
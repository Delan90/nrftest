#ifndef __LORA_VAL_H__
#define __LORA_VAL_H__



RadioEvents_t RadioEvents;
McpsReq_t mcpsRequest;
MlmeReq_t mlmeRequest;

void * r_context;
void * q_context;

/*!
 * Tx and Rx timers
 */
app_timer_id_t   tx_timeout_timer;
app_timer_id_t   rx_timeout_timer;
app_timer_id_t   rx_timeout_sync_word;


/*!
 * LoRaMac timer used to check the LoRaMacState (runs every second)
 */
//static TimerEvent_t MacStateCheckTimer;
app_timer_id_t MacStateCheckTimer;

/*!
 * LoRaMac duty cycle delayed Tx timer
 */
//static TimerEvent_t TxDelayedTimer;
app_timer_id_t TxDelayedTimer;
/*!
 * LoRaMac reception windows timers
 */
//static TimerEvent_t RxWindowTimer1;
app_timer_id_t RxWindowTimer1;
//static TimerEvent_t RxWindowTimer2;
app_timer_id_t RxWindowTimer2;

/*!
 * Acknowledge timeout timer. Used for packet retransmissions.
 */
//static TimerEvent_t AckTimeoutTimer;
app_timer_id_t AckTimeoutTimer;

/*!
 * Timer to handle the application data transmission duty cycle
 */
//static TimerEvent_t TxNextPacketTimer;
app_timer_id_t   TxNextPacketTimer;


/*!
 * Timer to handle the state of LED1
 */
//static TimerEvent_t Led1Timer;
app_timer_id_t   Led1Timer;

/*!
 * Timer to handle the state of LED2
 */
//static TimerEvent_t Led2Timer;
app_timer_id_t   Led2Timer;

/*!
 * Timer to handle the state of LED4
 */
//static TimerEvent_t Led4Timer;
app_timer_id_t   Led4Timer;

#endif // __LORA_VAL_H__
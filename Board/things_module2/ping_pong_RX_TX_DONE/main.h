#ifndef __MAIN_H__
#define __MAIN_H__


RadioEvents_t RadioEvents;

extern void OnTxDone( void );

extern void OnRxDone( uint8_t *payload, uint16_t size, int16_t rssi, int8_t snr );

extern void OnTxTimeout( void );

extern void OnRxTimeout( void );

extern void OnRxError( void );


#endif // __MAIN_H__
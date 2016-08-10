#ifndef __MAIN_H__
#define __MAIN_H__


//#define USE_BAND_433   
//#define USE_BAND_780   
#define USE_BAND_868   
//#define USE_BAND_915  
//#define USE_BAND_915_HYBRID


#define APP_TIMER_PRESCALER             0 /**< Value of the RTC1 PRESCALER register. */
#define APP_TIMER_MAX_TIMERS            15 /**< Maximum number of simultaneously created timers. */
#define APP_TIMER_OP_QUEUE_SIZE         15 /**< Size of timer operation queues. */

// Scheduler settings
#define SCHED_MAX_EVENT_DATA_SIZE       20
#define SCHED_QUEUE_SIZE                20


#endif // __MAIN_H__
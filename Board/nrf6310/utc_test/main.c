#include "nrf51.h"
#include "nrf_gpio.h"
#include "rtc.h"
#include "app_scheduler.h"
#include "ble_gatts.h"
#include "app_error.h"
#include "nrf_gpio.h"
//#include "power_manger.h"
#include <stdbool.h>
#include "app_timer.h"
#include <stdint.h>
#include <string.h>
#include "nordic_common.h"
#include "nrf.h"
#include "nrf51_bitfields.h"
/*********************************************************************
 * MACROS
 */

#define YearLength(yr)  (IsLeapYear(yr) ? 366 : 365)
//日历变量

UTCTimeStruct Time_iOS = {8,59,15,9,6,2015};
UTCTimeStruct Time_Current;

/*********************************************************************
 * CONSTANTS
 */

// (MAXCALCTICKS * 5) + (max remainder) must be <= (uint16_t max),
// so: (13105 * 5) + 7 <= 65535
#define MAXCALCTICKS  ((uint16_t)(13105))

#define BEGYEAR         1970     // UTC started at 00:00:00 January 1, 2000

#define DAY             86400UL  // 24 hours * 60 minutes * 60 seconds


/*********************************************************************
 * EXTERNAL FUNCTIONS
 */
extern uint16_t ll_McuPrecisionCount(void);

/*********************************************************************
 * LOCAL VARIABLES
 */
//static uint16_t previousLLTimerTick = 0;
//static uint16_t remUsTicks = 0;
//static uint16_t timeMSec = 0;

// number of seconds since   2010.1.1.  00:00:00
UTCTime TimeSeconds = 0;

/*********************************************************************
 * LOCAL FUNCTION PROTOTYPES
 */
static uint8_t monthLength( uint8_t lpyr, uint8_t mon );

/*********************************************************************
 * FUNCTIONS
 */

/*********************************************************************
 * @fn      Set_Clock
 *
 * @brief   Set the new time.  This will only set the seconds portion
 *          of time and doesn't change the factional second counter.
 *
 * @param   newTime - number of seconds since 0 hrs, 0 minutes,
 *                    0 seconds, on the 1st of January 2000 UTC
 *
 * @return  none
 */
void Set_Clock( UTCTime newTime )
{
  TimeSeconds = newTime;
}

/*********************************************************************
 * @fn      Get_Clock
 *
 * @brief   Gets the current time.  This will only return the seconds
 *          portion of time and doesn't include the factional second
 *          counter.
 *
 * @param   none
 *
 * @return  number of seconds since 0 hrs, 0 minutes, 0 seconds,
 *          on the 1st of January 2000 UTC
 */
UTCTime Get_Clock( void )
{
  return ( TimeSeconds );
}

/*********************************************************************
 * @fn      ConvertUTCTime
 *
 * @brief   Converts UTCTime to UTCTimeStruct
 *
 * @param   输出 tm - pointer to breakdown struct
 *
 * @param   输入 secTime - number of seconds since 0 hrs, 0 minutes,
 *          0 seconds, on the 1st of January 2000 UTC
 *
 * @return  none
 */
uint16_t numDays =0;
void ConvertUTCTime( UTCTimeStruct *tm, UTCTime secTime )
{
  // calculate the time less than a day - hours, minutes, seconds
  {
    uint32_t day = secTime % DAY;  // 求出最后一天的剩余秒数，（除去整数的天数之后的剩余秒数）
    tm->seconds = day % 60UL;      // 求出最后一分钟的剩余秒数 （最后一天的剩余秒数对60求余）
    tm->minutes = (day % 3600UL) / 60UL;  //求出 最后一小时剩余的整分钟数
    tm->hour = day / 3600UL;       //求最后一天的整小时数
  }

  // Fill in the calendar - day, month, year
  {
     numDays = secTime / DAY;
    tm->year = BEGYEAR;
    while ( numDays >= YearLength( tm->year ) )
    {
      numDays -= YearLength( tm->year );
      tm->year++;
    }

    tm->month = 0;
    while ( numDays > monthLength( IsLeapYear( tm->year ), tm->month ) )
    {
      numDays -= monthLength( IsLeapYear( tm->year ), tm->month );
      tm->month++;
    }
    tm->month++;
    tm->day = numDays;
        tm->day++;
  }
}

/*********************************************************************
 * @fn      monthLength
 *
 * @param   lpyr - 1 for leap year, 0 if not
 *
 * @param   mon - 0 - 11 (jan - dec)
 *
 * @return  number of days in specified month
 */
static uint8_t monthLength( uint8_t lpyr, uint8_t mon )
{
  uint8_t days = 31;

  if ( mon == 1 ) // feb
  {
    days = ( 28 + lpyr );
  }
  else
  {
    if ( mon > 6 ) // aug-dec
    {
      mon--;
    }

    if ( mon & 1 )
    {
      days = 30;
    }
  }

  return ( days );
}

/*********************************************************************
 * @fn      ConvertUTCSecs
 *
 * @brief   Converts a UTCTimeStruct to UTCTime
 *
 * @param   tm - pointer to provided struct
 *
 * @return  number of seconds since 00:00:00 on 01/01/2000 (UTC)
 */
UTCTime ConvertUTCSecs( UTCTimeStruct *tm )
{
  uint32_t seconds;

  /* Seconds for the partial day */
  seconds = (((tm->hour * 60UL) + tm->minutes) * 60UL) + tm->seconds;

  /* Account for previous complete days */
  {
    /* Start with complete days in current month */
    uint16_t days = tm->day;

    /* Next, complete months in current year */
    {
      int8_t month = tm->month - 1;//
      while ( --month >= 0 )
      {
        days += monthLength( IsLeapYear( tm->year ), month );
      }
    }

    /* Next, complete years before current year */
    {
      uint16_t year = tm->year;
      while ( --year >= BEGYEAR )
      {
        days += YearLength( year );
      }
    }

    /* Add total seconds before partial day */
    seconds += (days * DAY);
  }

  return ( seconds );
}
/*********************************************************************
 * @fn      CaculateWeekday
 *
 * @brief   Converts a UTCTimeStruct to UTCTime
 *
 * @param   tm - pointer to provided struct
 *
 * @return  number of seconds since 00:00:00 on 01/01/2000 (UTC)
 */
uint8_t CaculateWeekday(UTCTimeStruct *ptime)
{ 
      // ¸ù¾ÝÈÕÆÚÊý¼ÆËãÐÇÆÚÊý
        uint16_t year   = ptime->year ;
    
        uint8_t  mounth = ptime->month;
    
        uint8_t  day    = ptime->day + 1;
    
        uint8_t  week;
    
        if(mounth==1 || mounth==2) { mounth += 12;  year--; }  
        week = (day + 2*mounth + 3*(mounth+1)/5 + year + year/4 - year/100 + year/400)%7; 
        if(week == 0)  week = 7;
        return (uint8_t)week;
}


/**********************************************************
注册一个1s的时间用于RTC
********************************************************/
#define APP_TIMER_PRESCALER 0
//wall clock id
app_timer_id_t                              SecondID;        //广播的时候闪烁蓝灯
#define ONESECOND_INTERVAL                   APP_TIMER_TICKS(1000, APP_TIMER_PRESCALER) // 广播时间设置
static void SECONED_ADD_clock(void * p_context)  //采集皮肤数据灯闪烁指示
{
    (void)p_context;
      UTCTimeStruct utc;
     TimeSeconds++ ;
     //一天有86400s   一个小时 3600s
      ConvertUTCTime( &utc, Get_Clock() );  // ??????
    if(utc.minutes == 0 && utc.seconds == 0 && utc.year !=0)
    { 
             //每个小时
           if(utc.hour == 0)
             { 
                  //每一天
                  if(utc.day == 0)
                    {
                        //每一个月
                    }
             }
        }           
        
}
/**************************************************************************
* system clock init
***************************************************************************/
void SECOND_clock_init(void)
{
      uint32_t err_code;
      UTCTimeStruct utc;  
        utc.seconds = 0x00;
        utc.minutes = 0x00;
        utc.hour    = 0x00;
        utc.day     = 0x00; //CC2540 ???? 0 -30
        utc.month   = 0x00; //CC2540 ???? 0 -11  
        utc.year    = 2000;
        Set_Clock( ConvertUTCSecs( &utc ) );
                
    err_code = app_timer_create(&SecondID, APP_TIMER_MODE_REPEATED, SECONED_ADD_clock); //广播事件
    APP_ERROR_CHECK(err_code);
 
    
    err_code = app_timer_start(SecondID, ONESECOND_INTERVAL, NULL);
    APP_ERROR_CHECK(err_code);
    
}


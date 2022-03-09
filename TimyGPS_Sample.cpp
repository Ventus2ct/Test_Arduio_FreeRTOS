#include <TinyGPS++.h>
#include <HardwareSerial.h>
#include "esp_system.h" //This inclusion configures the peripherals in the ESP system.
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/timers.h"
#include "freertos/event_groups.h"

#define GPS_Parse                 ( 1 << 4 ) //10000 
EventGroupHandle_t eg;
HardwareSerial GPSSerial(2);
// Connect to the GPS on the hardware port
// The TinyGPS++ object
TinyGPSPlus GPS;

/* create a hardware timer */
hw_timer_t * timer = NULL;

// timer ISR callback set at 1000X a second
void IRAM_ATTR onTimer()
{
  BaseType_t xHigherPriorityTaskWoken;
  //
  if ( (xSemaphoreTakeFromISR(sema_GPS_Gate, &xHigherPriorityTaskWoken)) == pdTRUE ) // grab semaphore, no wait
  {
    xEventGroupSetBitsFromISR(eg, GPS_Parse, &xHigherPriorityTaskWoken); // trigger every 1mS, if not already processing
  }
} // void IRAM_ATTR onTimer()

void setup()
{
  eg = xEventGroupCreate();
  //
  GPSSerial.begin( GPS_DataBits ); // begin GPS hardwareware serial
  //
  vTaskDelay( pdMS_TO_TICKS( OneK ) ); // delay one second
  timer = timerBegin( TIMER_FOUR, TimerDivider, true );
  timerAttachInterrupt( timer, &onTimer, true );
  timerAlarmWrite(timer, OneK, true);
  timerAlarmEnable(timer);

  xTaskCreatePinnedToCore( 
    fGPS_Parse, 
    "fGPS_Parse", 
    TaskStack10K2, 
    NULL, 
    Priority4, 
    NULL, 
    TaskCore0 ); // assigned to core 0
} //void setup() // runs on core 1
///////////////////////////
void loop() {} // runs on core 1


void fGPS_Parse( void * parameter )
{
  TickType_t xDoDistanceExpireTicks;
  struct stuTime *pxTime;
  struct stuPosit pxPosit;
  struct stuDistance pxDistance;
  struct stuSpeed pxSpeed;
  float Alti;
  float Hdg;
  int Sats;
  float LatNow;
  float LonNow;
  float LatPast;
  float LonPast;
  int DoDistanceTicks = 5000;
  ////
  xDoDistanceExpireTicks = xTaskGetTickCount() + pdMS_TO_TICKS( DoDistanceTicks ); // add DoDistanceTicks mS worth of ticks to current tick count
  for (;;)
  {
    xEventGroupWaitBits (eg, GPS_Parse, pdTRUE, pdTRUE, portMAX_DELAY) ;
    //query GPS: has a new complete chunk of data been received?
    if ( GPSSerial.available() > 1 )
    {
      if ( GPS.encode(GPSSerial.read()) )
      {
      //    
      }
    }
//parse code follows, exceeds the 9K character limit.
  } // for (;;)
  vTaskDelete( NULL );
} // void fGPS_Parse( void *pdata )
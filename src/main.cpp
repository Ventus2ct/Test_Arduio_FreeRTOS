#include <Arduino_FreeRTOS.h>
#include <semphr.h>  // add the FreeRTOS functions for Semaphores (or Flags).
#include <Arduino.h>
#include <TinyGPS++.h>
#include <IndicatiorHandler.h>
#include <MemoryFree.h>

// #ifdef __cplusplus
// extern "C" {
// #endif
// void setup();
// void loop();
// #ifdef __cplusplus
// }
// #endif

// Declare a mutex Semaphore Handle which we will use to manage the Serial Port.
// It will be used to ensure only one Task is accessing this resource at any time.
SemaphoreHandle_t xSerialSemaphore;

// define Tasks (move to a header file)
void TaskDigitalRead( void *pvParameters );
void TaskAnalogRead( void *pvParameters );
void Task_GPS_Serial_Read( void *pvParameters );
void Task_GPS_Debug( void *pvParameters);
void Task_Comfort_Indicator( void *pvParameters);

TinyGPSPlus GPS;
int iRightIndicator=0;
int iLeft_Indicator=0;

int iRightpin=10;
int iLeft_pin=9;

int iIndicator =0; // Left 0, Right 1

double d_15Voltage=0;
double d_30Voltage=0;

// Global init:
IndicatiorHandler indicator;


void setup() {
  // Monitor
  Serial.begin(115200);
  // GPS
  Serial3.begin(9600);

  //indicator.Enable();

  // Serial3.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);
  // Serial3.sendCommand(PMTK_API_SET_FIX_CTL_1HZ);

  // Semaphores are useful to stop a Task proceeding, where it should be paused to wait,
  // because it is sharing a resource, such as the Serial port.
  // Semaphores should only be used whilst the scheduler is running, but we can set it up here.
  if ( xSerialSemaphore == NULL )  // Check to confirm that the Serial Semaphore has not already been created.
  {
    xSerialSemaphore = xSemaphoreCreateMutex();  // Create a mutex semaphore we will use to manage the Serial Port
    if ( ( xSerialSemaphore ) != NULL )
      xSemaphoreGive( ( xSerialSemaphore ) );  // Make the Serial Port available for use, by "Giving" the Semaphore.
  }

  // Now set up two Tasks to run independently.
  xTaskCreate(
    TaskDigitalRead
    ,  "LeftRead"  // A name just for humans
    ,  128  // This stack size can be checked & adjusted by reading the Stack Highwater
    ,  (void*)0 //Parameters for the task
    ,  2  // Priority, with 3 (configMAX_PRIORITIES - 1) being the highest, and 0 being the lowest.
    ,  NULL ); //Task Handle

  // Now set up two Tasks to run independently.
  xTaskCreate(TaskDigitalRead, "RightRead", 128, (void*) 1, 2, NULL ); 
  // read voltage
  xTaskCreate(TaskAnalogRead,  "15Voltage", 128, (void*) A12, 1, NULL ); 
  // read voltage
  xTaskCreate(TaskAnalogRead,  "30Voltage", 128, (void*) A10, 1, NULL ); 
  // Start GPS read task
  xTaskCreate(Task_GPS_Serial_Read, "GpsReadAndParse", 128, NULL, 1, NULL );
  // Start debug task
  xTaskCreate(Task_GPS_Debug, "GpsDebug", 256, NULL, 1, NULL );
  // Start comfort blinkers
  xTaskCreate(Task_Comfort_Indicator, "ComfortBlink", 128, NULL, 1, NULL );

  // Add a new line..
  Serial.println();
  // Now the Task scheduler, which takes over control of scheduling individual Tasks, is automatically started.
}

void loop() {
  // Empty. Things are done in Tasks.
}

/*---------------------- Tasks ---------------------*/
void Task_Comfort_Indicator( void *pvParameters __attribute__((unused)) )
{
  indicator.Enable();
  indicator.SetMaxBlink(4);
  for (;;)
  {
    indicator.IndicatiorUpdate();
  }
}

void Task_GPS_Debug( void *pvParameters __attribute__((unused)) )
{
  char sz[32];  // for print formatting
  char blank[84] = "\r                                                                                 ";
  char buf[12]; // for float or double formatting
  String sBlinkStatus;
  unsigned long l_Last_GPS_Debug = millis(); // The last output, so we can time next
  unsigned long NowTick;                     // Just millins 
  int DebugPrintInterval = 250;              // print interval in [ms]
  for (;;) 
  {
    NowTick = millis();
    if(NowTick > (l_Last_GPS_Debug + DebugPrintInterval) )
    {
      Serial.print(blank);
      // Output a one liner debug info:
      sprintf(sz, "\r%02d/%02d/%02d %02d:%02d:%02d",
            GPS.date.day() , GPS.date.month(), GPS.date.year(), 
            GPS.time.hour(), GPS.time.minute(), GPS.time.second());
      Serial.print(sz);
      if(GPS.location.age() < 9999)
      {
        sprintf(sz, " Sat: %2d Age %4d ms " , int(GPS.satellites.value()), int( GPS.location.age()));
        Serial.print(sz);
      }
      // sprintf(sz, ", Left %1d, Right %1d, +15: ", iLeft_Indicator, iRightIndicator);
      // sprintf(sz, "%s", indicator.Status());
      
      dtostrf(d_15Voltage,5,2,buf);
      Serial.print(buf);
      Serial.print(" V, +30: ");
      dtostrf(d_30Voltage,5,2,buf);
      Serial.print(buf);
      Serial.print(" V");
      
      sBlinkStatus = " CB: " + indicator.Status() + "  ";
      sBlinkStatus.toCharArray(sz, sBlinkStatus.length());
      Serial.print(sz);

      // sprintf(sz, " %d B", freeMemory());
      // Serial.print(freeMemory());
      // Serial.print(" B");

      l_Last_GPS_Debug = NowTick;
    }
    vTaskDelay(1);
  }
}
/*
  SerialEvent occurs whenever a new data comes in the hardware serial RX. 
  Multiple bytes of data may be available.
*/
// void serialEvent3() {
//   while (Serial3.available()) {
//     GPS.encode((char) Serial3.read());
//   }
// }
/**
 * 
 */  
void Task_GPS_Serial_Read( void *pvParameters __attribute__((unused)) )
{
  for (;;) // A Task shall never return or exit.
  {
    while (Serial3.available()) {
      GPS.encode( (char) Serial3.read());
    }
    vTaskDelay(1);  // one tick delay (15ms) in between reads for stability
  }
}

void TaskDigitalRead( void *pvParameters )  // This is a Task.
{
  /*
    Left:0 / right:1 indicator
  */
  int direction = (int) pvParameters;
  uint8_t pushButton = 0;
  // digital pin 2 has a pushbutton attached to it. Give it a name:
  if (direction == 0) // Left
  {
    pushButton = 9;
  }
  else
  {
    pushButton = 10;
  }
  

  // make the pushbutton's pin an input:
  pinMode(pushButton, INPUT);

  for (;;) // A Task shall never return or exit.
  {
    // read the input pin:
    int buttonState = digitalRead(pushButton);

    // See if we can obtain or "Take" the Serial Semaphore.
    // If the semaphore is not available, wait 5 ticks of the Scheduler to see if it becomes free.
    if ( xSemaphoreTake( xSerialSemaphore, ( TickType_t ) 5 ) == pdTRUE )
    {
      // We were able to obtain or "Take" the semaphore and can now access the shared resource.
      // We want to have the Serial Port for us alone, as it takes some time to print,
      // so we don't want it getting stolen during the middle of a conversion.
      // print out the state of the button:
      // Serial.println(buttonState);
      if (direction == 0)
      {
        iLeft_Indicator = buttonState;
      }
      else
      {
        iRightIndicator = buttonState;
      }
      
      xSemaphoreGive( xSerialSemaphore ); // Now free or "Give" the Serial Port for others.
    }
    vTaskDelay(1);  // one tick delay (15ms) in between reads for stability
  }
}

void TaskAnalogRead( void *pvParameters )  // This is a Task.
{
  int iAnalogPin = (int) pvParameters;
  for (;;)
  {
    // read the input on analog pin:
    int sensorValue = analogRead(iAnalogPin);

    // See if we can obtain or "Take" the Serial Semaphore.
    // If the semaphore is not available, wait 5 ticks of the Scheduler to see if it becomes free.
    if ( xSemaphoreTake( xSerialSemaphore, ( TickType_t ) 5 ) == pdTRUE )
    {
      // We were able to obtain or "Take" the semaphore and can now access the shared resource.
      // We want to have the Serial Port for us alone, as it takes some time to print,
      // so we don't want it getting stolen during the middle of a conversion.
      // print out the value you read:
      // Serial.println(sensorValue);
      // iAnalogValue = sensorValue;
      switch (iAnalogPin)
      {
      case A10:
        /* +30 */
        d_30Voltage = (double) sensorValue * 0.0240305936661;
        
        break;
      
      case A12:
        /* +15 */
        d_15Voltage = (double) sensorValue * 0.0240305936661;
        
        break;
      
      default:
        break;
      }
      xSemaphoreGive( xSerialSemaphore ); // Now free or "Give" the Serial Port for others.
    }
    vTaskDelay(1);  // one tick delay (15ms) in between reads for stability
  }
}

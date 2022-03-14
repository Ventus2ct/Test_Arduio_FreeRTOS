#pragma once
#include <Arduino.h>

// Digital IO pins on MEGA 2560 for Indicator signals from DICE
#define PinIndicatorLEFT  9
#define PinIndicatorRIGHT 10

/**
 * Timed indicator
 * counted 40 blinks in 27 seconds
 * 26/40 = 0.650 seconds
 * #define IndicatorBlinkInterval 675 
 */
/**
 * Indicator handle direction
*/
enum class DIRECTION : short int
{
    OFF = 0,
    LEFT = 1,
    RIGHT = 2
};
class IndicatiorHandler
{
public:
    IndicatiorHandler();
    void IndicatiorDetected(DIRECTION directionLeftRight);
    void IndicatiorUpdate();
    void SetBlinkInterval(int interval);
    void SetMaxBlink(int count);
    int  GetMaxBlink();
    void Enable();
    void Disable();
    String Status();                                            // Returns OFF, RIGHT, LEFT
    
private:
    bool IndicatorComfortBlinkEnabled = false;                  // Set by config 
    bool IndicatorSetByHandle = false;                          // If handle is set to one side - no need to start comfort blink
    DIRECTION IndicatorComfortBlinkDirection = DIRECTION::OFF;  // 1: left, 2: right
    bool IndicatorComfortBlinkInAction = false;                 // only true if active
    short int BlinkCounter = 0;                                 // Counter 
    short int maxBlink = 3;                                     // Max blink
    int indicatorInterval = 650;                                // in milliseconds
    int indicatorPulsLength = 150;                              // The digital port to DICE needs to stay high for x ms
    unsigned long indicatiorLastBlink = 0;                      // Set from millis()
    unsigned long indicatorComfortBlinkStart = 0;               // Start of run 
    void SetBlink(DIRECTION directionLeftRight);                // set dig IO HIGH
    void ResetBlink(DIRECTION directionLeftRight);              // set dig IO Low 
    void PollDice();                                            // Polls the dig IO pins
    bool bDebug = false;                                        // Set true to get debug info
};

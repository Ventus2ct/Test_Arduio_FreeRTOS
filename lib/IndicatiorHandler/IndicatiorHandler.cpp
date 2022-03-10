#include "IndicatiorHandler.h"

// IndicatiorHandler::IndicatiorHandler(MCP_CAN *CAN)
IndicatiorHandler::IndicatiorHandler()
{
    // this->CAN = CAN;
    // Indicator pin setup
    pinMode(PinIndicatorLEFT, INPUT);
    pinMode(PinIndicatorRIGHT, INPUT);
}

/**
 * Using POLL in stead of interrupt..
 */
void IndicatiorHandler::PollDice()
{
    if(digitalRead(PinIndicatorLEFT))
    {
        IndicatiorDetected(DIRECTION::LEFT);
        return;
    }
    if(digitalRead(PinIndicatorRIGHT))
    {
        IndicatiorDetected(DIRECTION::RIGHT);
        return;
    }
}
/**
 * Called from loop
 * - This controlles the blink rate and duration
 * 
 * Read digital IO (both) and see if handle is permanet set
*/
void IndicatiorHandler::IndicatiorUpdate()
{
    PollDice();

    /* Check if active, if not break */
    if(!IndicatorComfortBlinkInAction) return;

    // Serial.println("CB time " + String(millis()));

    /* First, check that more than 100 ms has passed after enabling CB*/ 
    unsigned long now = millis();
    if ((now - indicatorComfortBlinkStart) < 251) 
    {
       // Serial.print("\r" +String(millis()) + "Low pass filter..");
        return;
    }
    
    //  Serial.print("\rLeft: " + String(digitalRead(PinIndicatorLEFT)) + ", Right: " + String(digitalRead(PinIndicatorRIGHT)));

    /* Check if handle is permanet set? Set / Unset */
    if(digitalRead(PinIndicatorLEFT) == 1 || digitalRead(PinIndicatorRIGHT) == 1) 
    {
        if(bDebug) Serial.print("\r Permanent handle, disabling CB");
        indicatorComfortBlinkStart = 0;
        IndicatorComfortBlinkInAction = false;
        IndicatorComfortBlinkDirection = DIRECTION::OFF;
        return;
    }
    if(now > indicatorComfortBlinkStart + indicatorInterval)
    {
        // noInterrupts();
        SetBlink(IndicatorComfortBlinkDirection);
        // TODO: Use loop counting in stead of delay..
        delay(indicatorPulsLength);
        ResetBlink(IndicatorComfortBlinkDirection);
        // interrupts();
        BlinkCounter +=1;
        indicatorComfortBlinkStart = millis();
    }
    if(BlinkCounter >= maxBlink)
    {
        IndicatorComfortBlinkDirection = DIRECTION::OFF;
        indicatorComfortBlinkStart = 0;
        IndicatorComfortBlinkInAction = false;
        if(bDebug) Serial.println(String(BlinkCounter));
        BlinkCounter = 0;
    }
}

void IndicatiorHandler::SetBlink(DIRECTION directionLeftRight)
{
    switch(static_cast<DIRECTION>(directionLeftRight))
    {
        // Left
        case DIRECTION::LEFT:
            if(bDebug) Serial.println("Left");
            pinMode(PinIndicatorLEFT, OUTPUT);
            digitalWrite(PinIndicatorLEFT,HIGH);
            break;
        // Right
        case DIRECTION::RIGHT:
            if(bDebug) Serial.println("Right");
            pinMode(PinIndicatorRIGHT, OUTPUT);
            digitalWrite(PinIndicatorRIGHT,HIGH);
            break;
        case DIRECTION::OFF:
            if(bDebug) Serial.println("Off");
            break;
    }
}

void IndicatiorHandler::ResetBlink(DIRECTION directionLeftRight)
{
    switch(static_cast<DIRECTION>(directionLeftRight))
    {
        // Left
        case DIRECTION::LEFT:
            if(bDebug) Serial.println(" reset Left");
            digitalWrite(PinIndicatorLEFT,LOW); // might not be needed, just in case..
            pinMode(PinIndicatorLEFT, INPUT);
            break;
        // Right
        case DIRECTION::RIGHT:
            if(bDebug) Serial.println("reset Right");
            digitalWrite(PinIndicatorRIGHT,LOW); // might not be needed, just in case..
            pinMode(PinIndicatorRIGHT, INPUT);
            break;
        case DIRECTION::OFF:
            if(bDebug) Serial.println("Off");
            break;
    }
}

/**
 * Left 0
 * Right 1
*/ 
void IndicatiorHandler::IndicatiorDetected(DIRECTION directionLeftRight )
{
    


    // If already active, leave (might be used to detect long tap to cancel)
    // Also, need to turn off if tapped in other direction
    if(IndicatorComfortBlinkInAction)
    {
        // Check if driver has tapped in the other direction,
        // if so, turn to the other side?
        if(directionLeftRight == IndicatorComfortBlinkDirection)
        {
            return;
        }
        
    }

    IndicatorComfortBlinkInAction = true;
    IndicatorComfortBlinkDirection = static_cast<DIRECTION>(directionLeftRight);
    indicatorComfortBlinkStart = millis();
    BlinkCounter =1;

}

void IndicatiorHandler::SetBlinkInterval(int interval)
{
    indicatorInterval = interval;
}
void IndicatiorHandler::SetMaxBlink(int count)
{
    maxBlink = count;
}
int IndicatiorHandler::GetMaxBlink()
{
    return maxBlink;
}
void IndicatiorHandler::Enable()
{
    IndicatorComfortBlinkEnabled = true;
}
void IndicatiorHandler::Disable()
{
    IndicatorComfortBlinkEnabled = false;
}
String IndicatiorHandler::Status()
{
    String Status;
    if(IndicatorComfortBlinkInAction)
    {
        switch(static_cast<DIRECTION>(IndicatorComfortBlinkDirection))
        {
            // Left
            case DIRECTION::LEFT:
                // Serial.println("interrupt Left");
                Status = "LEFT";
                break;
            // Right
            case DIRECTION::RIGHT:
                // Serial.println("interrupt Right");
                Status = "RIGHT";
                break;
            case DIRECTION::OFF:
                // Serial.println("Off");
                Status = "OFF";
                break;
            default:
                Status = "OFF";
                break;
        }
    }
    else
    {
         Status = "OFF";
    }
    
    return Status;
}
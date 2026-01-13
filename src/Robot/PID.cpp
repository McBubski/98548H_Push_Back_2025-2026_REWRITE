/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       PID.cpp                                                   */
/*    Author:       Team 98548J (Ace)                                         */
/*    Created:      7-14-2022                                                 */
/*    Description:  Source file for PID class                                 */
/*                                                                            */
/*----------------------------------------------------------------------------*/

#include "Robot/PID.h"
#include "Robot/utility_functions.h"

#include <cmath>

PID::PID(double p, double i, double d, double r, double integral_cap, double speed_cap, double timeout, double settle_time)
{
    P = p;
    I = i;
    D = d;
    R = r;

    IntegralCap = integral_cap;
    SpeedCap = speed_cap;
    Timeout = timeout;
    SettleTime = settle_time;
}

double PID::Update(double error, double dt)
{   
    
    Time += dt;
    PreviousError = Error;
    Error = error;
    Integral += error * dt;
    Derivative = Error - PreviousError;

    if(std::abs(Integral) * I > IntegralCap)
    {
        Integral = (IntegralCap * Sign(Error)) / I;
    }

    PIDSpeed = Error * P + Integral * I + Derivative * D;

    if(!HasRampedUp)
    {
        RampUp += (R * dt) * Sign(Error);
        if(std::abs(RampUp) < std::abs(PIDSpeed))
        {
            SmartSpeed = RampUp;
        }
        else
        {   
            HasRampedUp = true;
            SmartSpeed = PIDSpeed;
        }
    }
    else
    {
        SmartSpeed = PIDSpeed;
    }

    if(std::abs(SmartSpeed) > SpeedCap)
    {
        SmartSpeed = SpeedCap * Sign(SmartSpeed);
    }

    RanOnce = true;
    return SmartSpeed;
}

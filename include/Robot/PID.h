/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       PID.h                                                     */
/*    Author:       Team 98548J (Ace)                                         */
/*    Created:      7-14-2022                                                 */
/*    Description:  Header file for PID class                                 */
/*                                                                            */
/*----------------------------------------------------------------------------*/
#pragma once

class PID
{
private:
  void DataLogCheck();
  void DataLogEnd();
public:

  double P;
  double I;
  double D;
  double R;

  double IntegralCap;
  double SpeedCap;

  double RampUp = 0;
  double Error = 0;
  double PreviousError = 0;

  double PIDSpeed;
  double SmartSpeed;

  double Integral = 0;
  double Derivative = 0;
  bool HasRampedUp = false;

  bool HasReachedEnd = false;
  double TimeReachedEnd = 0;
  double Timeout;
  double SettleTime;
  double Time = 0;

  bool RanOnce = false;

  PID(double p, double i, double d, double r, double integral_cap, double speed_cap, double timeout, double settle_time);
    
  /**
    * @brief Updates the PID and related tasks.
    * @returns A value representing the output speed/power of the PID.
    * @param error Distance left to travel.
    * @param dt Time elapsed since last call (Delta time).
    */ 
  double Update(double error, double dt);

};

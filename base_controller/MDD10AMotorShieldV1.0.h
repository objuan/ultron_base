#ifndef DualMC33926MotorShield_h
#define DualMC33926MotorShield_h

#if (ARDUINO >= 100)
 #include <Arduino.h>
#else
 #include <WProgram.h>
#endif

class MDD10AMotorShield
{
  public:  
    // CONSTRUCTORS
    MDD10AMotorShield(); // Default pin selection.
    MDD10AMotorShield(unsigned char M1DIR, unsigned char M1PWM,
                      unsigned char M2DIR, unsigned char M2PWM); // User-defined pin selection. 
    
    // PUBLIC METHODS
    void init(); // Initialize TIMER 1, set the PWM to 20kHZ. 
    void setM1Speed(int speed); // Set speed for M1.
    void setM2Speed(int speed); // Set speed for M2.
    void setSpeeds(int m1Speed, int m2Speed); // Set speed for both M1 and M2.
   // unsigned int getM1CurrentMilliamps(); // Get current reading for M1. 
  // unsigned int getM2CurrentMilliamps(); // Get current reading for M2.
    unsigned char getFault(); // Get fault reading.
    
  private:

    unsigned char _M1DIR;
    unsigned char _M2DIR;
    unsigned char _M1PWM;
    unsigned char _M2PWM ;

};

#endif

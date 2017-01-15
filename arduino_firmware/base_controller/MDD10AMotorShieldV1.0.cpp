#include "MDD10AMotorShieldV1.0.h"
#include "config.h"

// Constructors ////////////////////////////////////////////////////////////////

MDD10AMotorShield::MDD10AMotorShield()
{
  //Pin map
  _M1PWM = 6;
  _M1DIR = 7;

  _M2PWM = 3;
  _M2DIR = 4;
}

MDD10AMotorShield::MDD10AMotorShield(unsigned char M1DIR, unsigned char M1PWM,
                                     unsigned char M2DIR, unsigned char M2PWM)
{
  //Pin map
  _M1PWM = M1PWM;
  _M2PWM = M2PWM;
  _M1DIR = M1DIR;
  _M2DIR = M2DIR;
}

// Public Methods //////////////////////////////////////////////////////////////
void MDD10AMotorShield::init( ros::NodeHandle *nh)
{
  this->nh=nh;
// Define pinMode for the pins and set the frequency for timer1.

  pinMode(_M1DIR,OUTPUT);
  pinMode(_M1PWM,OUTPUT);
  pinMode(_M2DIR,OUTPUT);
  pinMode(_M2PWM,OUTPUT);

  inputToMotorFactor = float(MAX_PWM) / MOTOR_INPUT_LIMIT; //   // 51 / 80
  
  /*#if defined(__AVR_ATmega168__)|| defined(__AVR_ATmega328P__)
  // Timer 1 configuration
  // prescaler: clockI/O / 1
  // outputs enabled
  // phase-correct PWM
  // top of MOTOR_INPUT_LIMIT
  //
  // PWM frequency calculation
  // 16MHz / 1 (prescaler) / 2 (phase-correct) / 400 (top) = 20kHz
  TCCR1A = 0b10100000;
  TCCR1B = 0b00010001;
  ICR1 = 400;
  #endif
  */
}



// Set speed for motor 1, speed is a number betwenn -400 and 400
void MDD10AMotorShield::setM1Speed(int speed)
{
 //#ifdef DEBUG_MOTOR
  //Serial.print("setM1Speed 1: ");
  //Serial.println(speed);
  //#endif
  
  unsigned char reverse = 0;
  
  if (speed < 0)
  {
    speed = -speed;  // Make speed a positive quantity
    reverse = 1;  // Preserve the direction
  }
  if (speed > MAX_PWM)  // Max PWM dutycycle
    speed = MAX_PWM;
    
 /* #if defined(__AVR_ATmega168__)|| defined(__AVR_ATmega328P__)
  OCR1A = speed;
  #else*/
  #if ROS_LOG_ENABLED
  int debugValue = speed * inputToMotorFactor;
  Serial.println(debugValue);
  sprintf(log_msg, "setM1Speed (%d)", debugValue);
  (*nh).loginfo(log_msg);
  #endif

  
  // #ifdef DEBUG_MOTOR
 // Serial.print("setM1Speed 1: ");
 // Serial.println(speed * inputToMotorFactor);
  //#endif

  
  analogWrite(_M1PWM,speed * inputToMotorFactor); // default to using analogWrite, mapping MOTOR_INPUT_LIMIT to 255
  //#endif
  if (reverse)
    digitalWrite(_M1DIR,HIGH);
  else
    digitalWrite(_M1DIR,LOW);
}

// Set speed for motor 2, speed is a number betwenn -MOTOR_INPUT_LIMIT and MOTOR_INPUT_LIMIT
void MDD10AMotorShield::setM2Speed(int speed)
{
//  #ifdef DEBUG_MOTOR
//  Serial.print("setM1Speed 2: ");
//  Serial.println(speed);
//  #endif
  
  unsigned char reverse = 0;
  
  if (speed < 0)
  {
    speed = -speed;  // Make speed a positive quantity
    reverse = 1;  // Preserve the direction
  }
  if (speed > MAX_PWM)  // Max PWM dutycycle
    speed = MAX_PWM;
 /* #if defined(__AVR_ATmega168__)|| defined(__AVR_ATmega328P__)
  OCR1B = speed;
  #else*/
  analogWrite(_M2PWM,speed * inputToMotorFactor); // default to using analogWrite, mapping 400 to 255
  //#endif
  if (reverse)
    digitalWrite(_M2DIR,HIGH);
  else
    digitalWrite(_M2DIR,LOW);
}

// Set speed for motor 1 and 2
void MDD10AMotorShield::setSpeeds(int m1Speed, int m2Speed)
{
  setM1Speed(m1Speed);
  setM2Speed(m2Speed);
}
/*
// Return motor 1 current value in milliamps.
unsigned int DualMC33926MotorShield::getM1CurrentMilliamps()
{
  // 5V / 1024 ADC counts / 525 mV per A = 9 mA per count
  return analogRead(_M1FB) * 9;
}

// Return motor 2 current value in milliamps.
unsigned int DualMC33926MotorShield::getM2CurrentMilliamps()
{
  // 5V / 1024 ADC counts / 525 mV per A = 9 mA per count
  return analogRead(_M2FB) * 9;
}
*/
// Return error status
unsigned char MDD10AMotorShield::getFault()
{
  return 0;
 //  return !digitalRead(_nSF);
}


/* Functions and type-defs for PID control.

   Taken mostly from Mike Ferguson's ArbotiX code which lives at:
   
   http://vanadium-ros-pkg.googlecode.com/svn/trunk/arbotix/
*/
#include "config.h"

// FROM EXTERNAL
void setMotorSpeeds(int leftFrontSpeed, int leftBackSpeed, int rightFrontSpeed,int rightBackSpeed);
extern float PID_INTERVAL_FLOAT;

// 

/* PID setpoint info For a Motor */
typedef struct {
  double TargetTicksPerFrame;    // target speed in ticks per frame
  long Encoder;                  // encoder count
  long PrevEnc;                  // last encoder count
  long Delta;

  /*
  * Using previous input (PrevInput) instead of PrevError to avoid derivative kick,
  * see http://brettbeauregard.com/blog/2011/04/improving-the-beginner%E2%80%99s-pid-derivative-kick/
  */
  int PrevInput;                // last input
  int PrevErr;                   // last error

  /*
  * Using integrated term (ITerm) instead of integrated error (Ierror),
  * to allow tuning changes,
  * see http://brettbeauregard.com/blog/2011/04/improving-the-beginner%E2%80%99s-pid-tuning-changes/
  */
  int Ierror;
  int ITerm;                    //integrated term

  long output;                    // last motor setting
}

SetPointInfo;

SetPointInfo motorPID[4];

/* PID Parameters */
int Kp = 20;
int Kd = 12;
int Ki = 0;
int Ko = 50;

unsigned char moving = 0; // is the base in motion?

/*
* Initialize PID variables to zero to prevent startup spikes
* when turning PID on to start moving
* In particular, assign both Encoder and PrevEnc the current encoder value
* See http://brettbeauregard.com/blog/2011/04/improving-the-beginner%E2%80%99s-pid-initialization/
* Note that the assumption here is that PID is only turned on
* when going from stop to moving, that's why we can init everything on zero.
*/
void resetPID(){
  for(int i=0;i<4;i++)
  {
   motorPID[i].TargetTicksPerFrame = 0.0;
   motorPID[i].Encoder = readEncoder(i);
   motorPID[i].PrevEnc = motorPID[i].Encoder;
   motorPID[i].output = 0;
   motorPID[i].PrevInput = 0;
   motorPID[i].PrevErr = 0;
   motorPID[i].Ierror = 0;
   motorPID[i].ITerm = 0;

  }
}

/* PID routine to compute the next motor commands */

void doPID(SetPointInfo * p,float frameTimeFactor){
  long Perror;
  long output;

 // p->TargetTicksPerFrame = 200;
  // encoder e' soggetto a frameTimeFactor
  p->Delta = frameTimeFactor * ((p->Encoder)-p->PrevEnc);
  
  Perror = p->TargetTicksPerFrame - ( p->Delta);

 // frameTimeFactor=1;
          
  // Derivative error is the delta Perror
  output = ((Kp*Perror + Kd*(Perror - p->PrevErr) + Ki*p->Ierror)/Ko);
  
  p->PrevErr = Perror;
  p->PrevEnc = p->Encoder;
  
  output += p->output;   
  // Accumulate Integral error *or* Limit output.
  // Stop accumulating when output saturates
  if (output >= MOTOR_INPUT_LIMIT)
    output = MOTOR_INPUT_LIMIT;
  else if (output <= -MOTOR_INPUT_LIMIT)
    output = -MOTOR_INPUT_LIMIT;
  else
    p->Ierror += Perror;

 // output = 100;
  
  p->output = output;
}

void doPID_bo(SetPointInfo * p) {
  long Perror;
  long output;
  int input;

  //Perror = p->TargetTicksPerFrame - (p->Encoder - p->PrevEnc);
  input = p->Encoder - p->PrevEnc;
  Perror = p->TargetTicksPerFrame - input;


  /*
  * Avoid derivative kick and allow tuning changes,
  * see http://brettbeauregard.com/blog/2011/04/improving-the-beginner%E2%80%99s-pid-derivative-kick/
  * see http://brettbeauregard.com/blog/2011/04/improving-the-beginner%E2%80%99s-pid-tuning-changes/
  */
  //output = (Kp * Perror + Kd * (Perror - p->PrevErr) + Ki * p->Ierror) / Ko;
  // p->PrevErr = Perror;
  output = (Kp * Perror - Kd * (input - p->PrevInput) + p->ITerm) / Ko;

//  
  p->PrevEnc = p->Encoder;

  output += p->output;
  // Accumulate Integral error *or* Limit output.
  // Stop accumulating when output saturates
  if (output >= MOTOR_INPUT_LIMIT)
    output = MOTOR_INPUT_LIMIT;
  else if (output <= -MOTOR_INPUT_LIMIT)
    output = -MOTOR_INPUT_LIMIT;
  else
  /*
  * allow turning changes, see http://brettbeauregard.com/blog/2011/04/improving-the-beginner%E2%80%99s-pid-tuning-changes/
  */
    p->ITerm += Ki * Perror;
  
  p->output = output;
  p->PrevInput = input;
}

/* Read the encoder values and call the PID routine */
void updatePID(float deltaTimeMS,ros::NodeHandle *nh) {
  /* Read the encoders */

  motorPID[0].Encoder = readEncoder(0);
  motorPID[1].Encoder = readEncoder(1);
  motorPID[2].Encoder = readEncoder(2);
  motorPID[3].Encoder = readEncoder(3);
  
  //leftPID.Encoder = readEncoder(MOTOR_LEFT);
  //rightPID.Encoder = readEncoder(MOTOR_RIGHT);

  #ifdef DEBUG_ENC
  Serial.print("L:");
   Serial.println(motorPID[0].Encoder);
   Serial.print("R:");
   Serial.println(motorPID[2].Encoder);
   #endif
  
  /* If we're not moving there is nothing more to do */
  if (!moving){
    /*
    * Reset PIDs once, to prevent startup spikes,
    * see http://brettbeauregard.com/blog/2011/04/improving-the-beginner%E2%80%99s-pid-initialization/
    * PrevInput is considered a good proxy to detect
    * whether reset has already happened
    */
    #if 1
        nh->loginfo("resetPID");
#endif
    //if (motorPID[0].PrevInput != 0 || motorPID[2].PrevInput != 0) 
        resetPID();
    return;
  }

  float frameTimeFactor=  PID_INTERVAL_FLOAT / deltaTimeMS;

  //dtostrf(frameTimeFactor,4,3,tmp_msg1);
  ///sprintf(log_msg, "DIFF: frameTimeFactor:%s ", tmp_msg1);
  //(*nh).loginfo(log_msg);

  /* Compute PID update for each motor */
  doPID(&motorPID[0],frameTimeFactor);
  doPID(&motorPID[1],frameTimeFactor);
  doPID(&motorPID[2],frameTimeFactor);
  doPID(&motorPID[3],frameTimeFactor);

#if 0
        dtostrf(motorPID[0].Encoder,4,3,tmp_msg1);
        dtostrf(motorPID[0].output,4,3,tmp_msg2);
  
        sprintf(log_msg, "TIME (%s , %s)", tmp_msg1,tmp_msg2);
        nh->loginfo(log_msg);
#endif

  
  /* Set the motor speeds accordingly */
  setMotorSpeeds(motorPID[0].output, motorPID[1].output,motorPID[2].output,motorPID[3].output);
}


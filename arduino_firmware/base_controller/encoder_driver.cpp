
// This optional setting causes Encoder to use more optimized code,
// It must be defined before Encoder.h is included.
//#define ENCODER_OPTIMIZE_INTERRUPTS
#include <Encoder.h>
#include "config.h"

struct EncInfo
{
  Encoder* encoder;

  long enc_pos_base;
  long enc_pos;
  void init()
  {
    enc_pos_base=enc_pos=0;
  }

};

EncInfo encoders[4];

  void initEncoders(){
    encoders[0].encoder = new Encoder(ENC_FRONT_LEFT_PIN_A,ENC_FRONT_LEFT_PIN_B);
    encoders[1].encoder = new Encoder(ENC_BACK_LEFT_PIN_A,ENC_BACK_LEFT_PIN_B);
    encoders[2].encoder = new Encoder(ENC_FRONT_RIGHT_PIN_A,ENC_FRONT_RIGHT_PIN_B);
    encoders[3].encoder = new Encoder(ENC_BACK_RIGHT_PIN_A,ENC_BACK_RIGHT_PIN_B);
    encoders[0].init();
    encoders[1].init();
    encoders[2].init();
    encoders[3].init();
  }
  
  /* Wrap the encoder reading function */
  long readEncoder(int i) {
   
      encoders[i].enc_pos = encoders[i].encoder->read() -  encoders[i].enc_pos_base;
      return encoders[i].enc_pos;
   
  }

  /* Wrap the encoder reset function */
  void resetEncoder(int i) {

     encoders[i].enc_pos=0L;
     encoders[i].enc_pos_base = encoders[i].encoder->read() ;
     return;
  }

/* Wrap the encoder reset function */
void resetEncoders() {
  resetEncoder(MOTOR_FRONT_LEFT);
  resetEncoder(MOTOR_FRONT_RIGHT);
  resetEncoder(MOTOR_BACK_LEFT);
  resetEncoder(MOTOR_BACK_RIGHT);
}




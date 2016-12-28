
// This optional setting causes Encoder to use more optimized code,
// It must be defined before Encoder.h is included.
//#define ENCODER_OPTIMIZE_INTERRUPTS
#include <Encoder.h>
#include "config.h"

  Encoder left_enc (LEFT_ENC_PIN_A, LEFT_ENC_PIN_B);
  Encoder right_enc (RIGHT_ENC_PIN_A, RIGHT_ENC_PIN_B);

  long left_enc_pos_base=0;
  long left_enc_pos;

  long right_enc_pos_base=0;
  long right_enc_pos;


  /* Wrap the encoder reading function */
  long readEncoder(int i) {
    if (i == MOTOR_LEFT) {
      left_enc_pos = left_enc.read() - left_enc_pos_base;
      return left_enc_pos;
    }
    else
    {
      right_enc_pos = right_enc.read() - right_enc_pos_base;
      return right_enc_pos;
    }
  }

  /* Wrap the encoder reset function */
  void resetEncoder(int i) {
    if (i == MOTOR_LEFT){
      left_enc_pos=0L;
      left_enc_pos_base = left_enc.read();
      return;
    } else { 
      right_enc_pos=0L;
      right_enc_pos_base = right_enc.read();
      return;
    }
  }

/* Wrap the encoder reset function */
void resetEncoders() {
  resetEncoder(MOTOR_LEFT);
  resetEncoder(MOTOR_RIGHT);
}




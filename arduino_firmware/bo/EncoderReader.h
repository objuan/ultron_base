// This optional setting causes Encoder to use more optimized code,
// It must be defined before Encoder.h is included.
//#define ENCODER_OPTIMIZE_INTERRUPTS
#include <Encoder.h>

class EncoderReader
{
  public:  

    Encoder *enc;
    long enc_pos_base=0;
    long enc_pos;
  
    EncoderReader(int ENC_PIN_A,int ENC_PIN_B){
      enc = new Encoder(ENC_PIN_A,ENC_PIN_B);
    }
    ~EncoderReader(){
      delete enc;
    }

    /* Wrap the encoder reading function */
    long readEncoder() {
     
        enc_pos = enc->read() - enc_pos_base;
        return enc_pos;
   
    }
  
    /* Wrap the encoder reset function */
    void resetEncoder() {
      
        enc_pos=0L;
        enc_pos_base = enc->read();
    }

};


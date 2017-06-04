//#include <EnableInterrupt.h>
#include "PinChangeInt.h"

#define RC_NUM_CHANNELS  2

#define RC_CH1  0
#define RC_CH2  1
#define RC_CH3  2
#define RC_CH4  3

#define RC_CH1_INPUT  A0
#define RC_CH2_INPUT  A1
//#define RC_CH3_INPUT  A2
//#define RC_CH4_INPUT  A3

uint16_t rc_values[RC_NUM_CHANNELS];
uint32_t rc_start[RC_NUM_CHANNELS];
volatile uint16_t rc_shared[RC_NUM_CHANNELS];
  
  void calc_input(uint8_t channel, uint8_t input_pin) {
    if (digitalRead(input_pin) == HIGH) {
      rc_start[channel] = micros();
    } else {
      uint16_t rc_compare = (uint16_t)(micros() - rc_start[channel]);
      rc_shared[channel] = rc_compare;
    }
  }
  
  void calc_ch1() { calc_input(RC_CH1, RC_CH1_INPUT); }
  void calc_ch2() { calc_input(RC_CH2, RC_CH2_INPUT); }
  //void calc_ch3() { calc_input(RC_CH3, RC_CH3_INPUT); }
  //void calc_ch4() { calc_input(RC_CH4, RC_CH4_INPUT); }


  // packet structure 
uint8_t remoteControllerPacket[28] = { '$', 0x09, 0,0,0,0 , '\r', '\n' };

class RemoteController
{
public:

  void rc_read_values() {
    noInterrupts();
    memcpy(rc_values, (const void *)rc_shared, sizeof(rc_shared));
    interrupts();
  }

  
  void setup() {
    #ifdef DEBUG_REMOTE_CONT_MODE
    Serial.println("setup remote");
    #endif
   
    pinMode(RC_CH1_INPUT, INPUT);
    pinMode(RC_CH2_INPUT, INPUT);
    //pinMode(RC_CH3_INPUT, INPUT);
    //pinMode(RC_CH4_INPUT, INPUT);
  
    PCintPort::attachInterrupt(RC_CH1_INPUT, calc_ch1, CHANGE);
    PCintPort::attachInterrupt(RC_CH2_INPUT, calc_ch2, CHANGE);
    //enableInterrupt(RC_CH3_INPUT, calc_ch3, CHANGE);
    //enableInterrupt(RC_CH4_INPUT, calc_ch4, CHANGE);
    #ifdef DEBUG_REMOTE_CONT_MODE
    Serial.println("setup end");
    #endif
  }
  
  void loop() {
    rc_read_values();

  #if !defined(DEBUG_REMOTE_CONT_MODE)
        // from 0 to 1024

        int16_t ch1 = constraint(rc_values[RC_CH1],1024,2048) - 1024);
        int16_t ch2 = constraint(rc_values[RC_CH2],1024,2048) - 1024);
        
        remoteControllerPacket[2] = ch1 >> 8;
        remoteControllerPacket[3] = ch1 & 0xFF;

        remoteControllerPacket[4] = ch1 >> 8;
        remoteControllerPacket[5] = ch1 & 0xFF;
        
        Serial.write(remoteControllerPacket, 8);
   #endif
        
 #ifdef DEBUG_REMOTE_CONT_MODE
    Serial.println("------------");
    Serial.print("CH1:"); Serial.print(rc_values[RC_CH1]); Serial.print("\t");
    Serial.print("CH2:"); Serial.print(rc_values[RC_CH2]); Serial.print("\t");
  //  Serial.print("CH3:"); Serial.print(rc_values[RC_CH3]); Serial.print("\t");
  //  Serial.print("CH4:"); Serial.println(rc_values[RC_CH4]);
  
 //   delay(200);
 #endif
  }
};

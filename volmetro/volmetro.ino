#if (ARDUINO >= 100)
 #include <Arduino.h>
#else
 #include <WProgram.h>
#endif
#include <ros.h>
#include <rosserial_arduino/Adc.h>

#define BAUDRATE 115200

ros::NodeHandle nh;
rosserial_arduino::Adc adc_msg;
ros::Publisher sensor_pub("volt_value", &adc_msg);

// can be used for getting rid of noise
int averageAnalog(int pin){
  int v=0;
  for(int i=0; i<4; i++) v+= analogRead(pin);
  return v/4;
}

long adc_timer;

/*
DC Voltmeter
An Arduino DVM based on voltage divider concept
T.K.Hareendran
TVout port code by TCPMeta
Use a 1K resistor on pin 7
Use a 470 ohm resistor on Pin 9
Just follow the typical pinout for the TVout Library found everwhere on the net.
*/


//TVout TV;
int analogInput = A1;
float vout = 0.0;
float vin = 0.0;
float R1 = 20000.0; // resistance of R1 (100K) -see text!
float R2 = 10000.0; // resistance of R2 (10K) – see text!
int value = 0;

void setup(){
  Serial.begin(BAUDRATE);
  Serial.println("GO");
  
  pinMode(analogInput, INPUT);
 // TV.begin(NTSC,120,96);
 // TV.select_font(font6x8);
  //TV.println(“//////////////////”);
  //TV.println(“// DC VOLTMETER //”);
 // TV.println(“//////////////////”);
}
void loop(){
  // read the value at analog input
  value = analogRead(analogInput);

  // Convert the analog reading (which goes from 0 - 1023) to a voltage (0 - 5V):
  vout = (value * 5.0) / 1023.0; // see text
  vin = vout / (R2/(R1+R2));
  if (vin<0.09) {
    vin=0.0;//statement to quash undesired reading !
  }
  
  Serial.print(vout);
  Serial.print("->");
  Serial.println(vin);
   
 // TV.select_font(font8x8);
  //TV.print(9,40,"Volts=");
 // TV.println(vin);
  //TV.select_font(font4x6);
 // TV.println(9,85,"TVOut Port By TCPMeta");
  delay(500);
}

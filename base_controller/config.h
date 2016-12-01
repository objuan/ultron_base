/*
 *  SCHEMA PIN
 *  INTERRUPTS  2,3
 *  PWM         3,5,6,9,10,11 
 */


/*  ENCODERS POPOLU
 *  Red motor power (connects to one motor terminal)
    Black motor power (connects to the other motor terminal)
    Green encoder GND
    Blue encoder Vcc (3.5 – 20 V)
    Yellow encoder A output
    White encoder B output

 */

// #define DEBUG_MODE 1
// #define DEBUG_MOTOR 1
// #define DEBUG_ENC 1
  
/* Serial port baud rate */
#define BAUDRATE     115200 // 57600

/* Maximum PWM signal */
#define MAX_PWM        255

/* Run the PID loop at 30 times per second */
#define PID_RATE       30     // Hz

/* Stop the robot if it hasn't received a movement command
in this number of milliseconds */
#define AUTO_STOP_INTERVAL 2000

// ========================
// COMMAND
// =======================

#define LEFT            0
#define RIGHT           1

// =========================
// MOTOR

#define LEFT_ENG_DIR 8
#define LEFT_ENG_PWM 9 

#define RIGHT_ENG_DIR 12
#define RIGHT_ENG_PWM 11

// =========================
// ENCODER

//below can be changed, but should be PORTD pins; 
//otherwise additional changes in the code are required

// Change these pin numbers to the pins connected to your encoder.
//   Best Performance: both pins have interrupt capability
//   Good Performance: only the first pin has interrupt capability
//   Low Performance:  neither pin has interrupt capability
//   avoid using pins with LEDs attached
// NOTA, 2,3 hanno gli interrupt
#define LEFT_ENC_PIN_A PD2  //pin 2
#define LEFT_ENC_PIN_B PD4  //pin 3
  
//below can be changed, but should be PORTC pins
#define RIGHT_ENC_PIN_A PC3  //pin A4
#define RIGHT_ENC_PIN_B PC5   //pin A5




// ========================
// DEBUG MODES
// =======================


//#define DEBUG_MOTOR 1
//#define DEBUG_BATTERY 1
//#define DEBUG_ENC 1
//#define DEBUG_RANGE 1

#if (DEBUG_MOTOR) || (DEBUG_BATTERY) || (DEBUG_ENC) || (DEBUG_RANGE)
  #define DEBUG_MODE 1
#endif

//#define ROS_LOG_ENABLED 1

// ========================
// GENERAL SETUP
// =======================

/* Serial port baud rate */
#define BAUDRATE     115200 // 57600

/* Run the PID loop at 30 times per second */
#define PID_RATE       30     // Hz

/* Convert the rate into an interval */
#define PID_INTERVAL_FLOAT  (1000.0 / PID_RATE)


// ============ MOTOR ======

// Gear Motor 3DDx73L 6533 counts per revolution
#define ENCODER_RESOLUTION 6533
#define GEAR_REDUCTION  1


// =========================
//    ROS
// =========================

#define ROBOT_NAME "PETROROV V1.0"

#define ROS_TOPIC_VELOCITY_IN "/petrorov/cmd_vel"
#define ROS_TOPIC_CONNECT "/petrorov/base/connect"

#define ROS_TOPIC_SENSOR "/petrorov/base/sensor_value"
#define ROS_TOPIC_MOTOR_STATE "/petrorov/base/motor_state"
#define ROS_TOPIC_RANGE_SENSOR "/petrorov/base/range"

#define ROS_TOPIC_GET_INFO_SRV "/petrorov/base/srv/getInfo"
#define ROS_TOPIC_RESET_POS_SRV "/petrorov/base/srv/resetPos"
#define ROS_TOPIC_STOP_SRV "/petrorov/base/srv/stop"

// =========================
//    CMD
// =========================

#define MOTOR_LEFT            0
#define MOTOR_RIGHT           1

// =========================
// MOTOR
// =========================

/*
 *  SCHEMA PIN
 *  INTERRUPTS  2,3
 *  PWM         3,5,6,9,10,11 
 */

// STESSO ORDINE DELLA SCHEDA

#define LEFT_ENG_DIR 10
#define LEFT_ENG_PWM 11

#define RIGHT_ENG_DIR 8
#define RIGHT_ENG_PWM 9


// INVERT SIGN, 1 , -1

#define MOTOR_LEFT_SIGN 1
#define MOTOR_RIGHT_SIGN -1

// 16MHz / 1 (prescaler) / 2 (phase-correct) / 400 (top) = 20kHz
#define MOTOR_INPUT_LIMIT 255

/* Maximum PWM signal */
#define MAX_PWM      255 // MOTOR_INPUT_LIMIT // 255
// =========================
// ENCODER
// =========================


/*  ENCODERS POPOLU
 *  Red motor power (connects to one motor terminal)
    Black motor power (connects to the other motor terminal)
    Green encoder GND
    Blue encoder Vcc (3.5 – 20 V)
    Yellow encoder A output
    White encoder B output

 */

//below can be changed, but should be PORTD pins; 
//otherwise additional changes in the code are required

// Change these pin numbers to the pins connected to your encoder.
//   Best Performance: both pins have interrupt capability
//   Good Performance: only the first pin has interrupt capability
//   Low Performance:  neither pin has interrupt capability
//   avoid using pins with LEDs attached
// NOTA, 2,3 hanno gli interrupt
#define LEFT_ENC_PIN_A PD3  //pin 2
#define LEFT_ENC_PIN_B PD5  //pin 3
  
//below can be changed, but should be PORTC pins
#define RIGHT_ENC_PIN_A PC2  //pin A4
#define RIGHT_ENC_PIN_B PC4   //pin A5

// =========================
// BATTERY LEVEL
// =========================

#define BATTERY_PIN_ANAG_A A0
#define BATTERY_PIN_ANAG_B A1


// =========================
// ULTRASONIC
// =========================

//  (Trig PIN,Echo PIN,NAME)
//#define RANGE_SENSOR_LIST      "4,5,FRONT,6,7,BACK"
#define RANGE_SENSOR_LIST      "24,22,/ultrasound_front"

// =======================


extern char log_msg[1000];
extern char tmp_msg1[100];
extern char tmp_msg2[100];
extern char tmp_msg3[100];



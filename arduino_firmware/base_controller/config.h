
// ========================
// DEBUG MODES
// =======================


//#define DEBUG_MOTOR 1
//#define DEBUG_ENC 1

//#define DEBUG_BATTERY 1

//#define DEBUG_RANGE 1
//#define DEBUG_IMU 1

#if (DEBUG_MOTOR) || (DEBUG_BATTERY) || (DEBUG_ENC) || (DEBUG_RANGE) || (DEBUG_IMU)
  #define DEBUG_MODE 1
#endif

//#define ROS_LOG_ENABLED 1

// ========================
// GENERAL SETUP
// =======================

/* Serial port baud rate */
#define BAUDRATE     115200 // 57600 115200

// OK OFF
#define BATTERY_ENABLED 1
#define RANGE_ENABLE
//#define IMU_ENABLE    1

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
#define ROS_TOPIC_IMU "/petrorov/base/imu"

#define ROS_TOPIC_GET_INFO_SRV "/petrorov/base/srv/getInfo"
#define ROS_TOPIC_RESET_POS_SRV "/petrorov/base/srv/resetPos"
#define ROS_TOPIC_STOP_SRV "/petrorov/base/srv/stop"
#define ROS_TOPIC_GET_DATA_ENCODERS_SRV "/petrorov/base/srv/getDataEncoders"


// =========================
// MOTOR
// =========================

#define MOTOR_FRONT_LEFT            0
#define MOTOR_BACK_LEFT             1
#define MOTOR_FRONT_RIGHT           2
#define MOTOR_BACK_RIGHT            3

/*
 *  SCHEMA PIN
 *  INTERRUPTS  2,3
 *  PWM         3,5,6,9,10,11 
 */

// STESSO ORDINE DELLA SCHEDA

#define FRONT_LEFT_DIR 5
#define FRONT_LEFT_PWM 6
#define BACK_LEFT_DIR 7
#define BACK_LEFT_PWM 8

#define FRONT_RIGHT_DIR 9
#define FRONT_RIGHT_PWM 10
#define BACK_RIGHT_DIR 11
#define BACK_RIGHT_PWM 12


// INVERT SIGN, 1 , -1

#define MOTOR_LEFT_SIGN 1
#define MOTOR_RIGHT_SIGN 1

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
#define ENC_FRONT_LEFT_PIN_A 20 // PD3  //pin 2
#define ENC_FRONT_LEFT_PIN_B 22 // PD5  //pin 3
  
//below can be changed, but should be PORTC pins
#define ENC_BACK_LEFT_PIN_A   21 // PC2//pin A4
#define ENC_BACK_LEFT_PIN_B   23  // PC4 pin A5

#define ENC_FRONT_RIGHT_PIN_A   19 // PC2//pin A4
#define ENC_FRONT_RIGHT_PIN_B   25  // PC4 pin A5

#define ENC_BACK_RIGHT_PIN_A   18 // PC2//pin A4
#define ENC_BACK_RIGHT_PIN_B   24  // PC4 pin A5

// =========================
// BATTERY LEVEL
// =========================


#define BATTERY_PIN_ANAG_A A6
#define BATTERY_PIN_ANAG_B A7


// =========================
// ULTRASONIC
// =========================

//  (Trig PIN,Echo PIN,NAME)
//#define RANGE_SENSOR_LIST      "4,5,FRONT,6,7,BACK"
//#define RANGE_SENSOR_LIST      "48,49,/ultrasound_front"
#define RANGE_SENSOR_LIST      "50,51,/ultrasound_back"
#define RANGE_SENSOR_LIST      "48,49,/ultrasound_front,50,51,/ultrasound_back"

// =======================

// =========================
// IMU , MEGA CONFIG
// =========================


#define IMU_INT_PIN   19  // 19
#define IMU_LED_PIN   13 // (Arduino is 13, Teensy is 11, Teensy++ is 6)
#define IMU_SDA_PIN   20
#define IMU_SCL_PIN   21



extern char log_msg[1000];
extern char tmp_msg1[100];
extern char tmp_msg2[100];
extern char tmp_msg3[100];



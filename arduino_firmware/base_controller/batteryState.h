/*
# Power supply status constants
uint8 POWER_SUPPLY_STATUS_UNKNOWN = 0
uint8 POWER_SUPPLY_STATUS_CHARGING = 1
uint8 POWER_SUPPLY_STATUS_DISCHARGING = 2
uint8 POWER_SUPPLY_STATUS_NOT_CHARGING = 3
uint8 POWER_SUPPLY_STATUS_FULL = 4

# Power supply health constants
uint8 POWER_SUPPLY_HEALTH_UNKNOWN = 0
uint8 POWER_SUPPLY_HEALTH_GOOD = 1
uint8 POWER_SUPPLY_HEALTH_OVERHEAT = 2
uint8 POWER_SUPPLY_HEALTH_DEAD = 3
uint8 POWER_SUPPLY_HEALTH_OVERVOLTAGE = 4
uint8 POWER_SUPPLY_HEALTH_UNSPEC_FAILURE = 5
uint8 POWER_SUPPLY_HEALTH_COLD = 6
uint8 POWER_SUPPLY_HEALTH_WATCHDOG_TIMER_EXPIRE = 7
uint8 POWER_SUPPLY_HEALTH_SAFETY_TIMER_EXPIRE = 8

# Power supply technology (chemistry) constants
uint8 POWER_SUPPLY_TECHNOLOGY_UNKNOWN = 0
uint8 POWER_SUPPLY_TECHNOLOGY_NIMH = 1
uint8 POWER_SUPPLY_TECHNOLOGY_LION = 2
uint8 POWER_SUPPLY_TECHNOLOGY_LIPO = 3
uint8 POWER_SUPPLY_TECHNOLOGY_LIFE = 4
uint8 POWER_SUPPLY_TECHNOLOGY_NICD = 5
uint8 POWER_SUPPLY_TECHNOLOGY_LIMN = 6

Header  header
float32 voltage          # Voltage in Volts (Mandatory)
float32 current          # Negative when discharging (A)  (If unmeasured NaN)
float32 charge           # Current charge in Ah  (If unmeasured NaN)
float32 capacity         # Capacity in Ah (last full capacity)  (If unmeasured NaN)
float32 design_capacity  # Capacity in Ah (design capacity)  (If unmeasured NaN)
float32 percentage       # Charge percentage on 0 to 1 range  (If unmeasured NaN)
uint8   power_supply_status     # The charging status as reported. Values defined above
uint8   power_supply_health     # The battery health metric. Values defined above
uint8   power_supply_technology # The battery chemistry. Values defined above
bool    present          # True if the battery is present

float32[] cell_voltage   # An array of individual cell voltages for each cell in the pack
                         # If individual voltages unknown but number of cells known set each to NaN
string location          # The location into which the battery is inserted. (slot number or plug)
string serial_number     # The best approximation of the battery serial number

*/

class BatteryState
{
  public:
    BatteryState(String name,int analogInput,float arduinoToOutFactor,
      int power_supply_technology){
      msg.power_supply_technology=power_supply_technology;
      msg.power_supply_health = sensor_msgs::BatteryState::POWER_SUPPLY_HEALTH_GOOD;

      min_value = 4;
      max_value = 12;
      _name=name;
      _analogInput=analogInput;
      _arduinoToOutFactor=arduinoToOutFactor;
      pinMode(analogInput, INPUT);
    }
    
    float readValue(){
      float value = analogRead(_analogInput);

      // Convert the analog reading (which goes from 0 - 1023) to a voltage (0 - 5V):
      vout = (value * 5.0) / 1023.0; // see text
      
      vin = vout * _arduinoToOutFactor;
      if (vin<0.09) {
        vin=0.0;//statement to quash undesired reading !
      }
      //STATE
      if (vin < min_value)
        msg.power_supply_status = sensor_msgs::BatteryState::POWER_SUPPLY_STATUS_DISCHARGING;
      else  if (vin > max_value)
        msg.power_supply_status = sensor_msgs::BatteryState::POWER_SUPPLY_STATUS_FULL;
      else
        msg.power_supply_status = sensor_msgs::BatteryState::POWER_SUPPLY_STATUS_FULL;
        
      return vin;
    }

    sensor_msgs::BatteryState &getMessage()
    {
      msg.voltage = vin;
      msg.present = vin> 0;
      // state
      /* uint8 POWER_SUPPLY_STATUS_UNKNOWN = 0
uint8 POWER_SUPPLY_STATUS_CHARGING = 1
uint8 POWER_SUPPLY_STATUS_DISCHARGING = 2
uint8 POWER_SUPPLY_STATUS_NOT_CHARGING = 3
uint8 POWER_SUPPLY_STATUS_FULL = 4
*/
      return msg;
    }
    
    String toString(){
      return String(_name + "->" + String(vin));
    }
 
  private:
    String _name;
    int _analogInput;
    float _arduinoToOutFactor;
    float vout;
    float vin;
    float min_value;
    float max_value;
    sensor_msgs::BatteryState msg;
};



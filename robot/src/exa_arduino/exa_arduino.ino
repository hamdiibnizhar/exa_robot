//preproccessing activation
//#define USE_ODOMETRY//

#define USE_SPEED_CONTROL

//#define USE_CASTOR
//#ifdef USE_CASTOR
//  #define USE_FRONT_WHEEL
//#else
//  #define USE_FRONT_WHEEL
//  #define USE_REAR_WHEEL
//#endif

#define USE_FRONT_WHEEL
#define USE_REAR_WHEEL

#define USE_OLD_COMMAND

//#define USE_SLOW_MOVEMENT


#include "ros.h"
//#include "ros/ros.h"
//#include <std_msgs/String.h>
//#include <std_msgs/Int32.h>
//#include <std_msgs/Int64.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Float32.h>
#include "geometry_msgs/Twist.h"
#include <SPI.h>

#ifdef USE_ODOMETRY
#include "rotary_encoder.h"
#endif

//#define USE_PACK_DATA_GEIGER
#define SHOW_LOG_MONITOR
#define USE_LONG_DATA
//#define USE_GM_IN_CALLBACK

#define USE_CPM
//#define USE_CPS
//define USE_COUNT_TO_MINUTE

#define LOG_PERIOD 5000  //Logging period in milliseconds, recommended value 15000-60000.
#define MAX_PERIOD 60000  //Maximum logging period without modifying this sketch
#define MAX_DOSIS 9460.8  // batas aman dosis radiasi = 9460,8 Sv/detik ( berdasarkan Perka BAPETEN -> 0.3 mSV/tahun, untuk perkalenderan 1 tahun =365 hari) 
#define CAP_PERIOD 5000


#ifdef USE_SPEED_CONTROL
int   SPEED = 0;
byte  SPEED_MAX = 100;            //in percent
byte  SPEED_MIN = 0;              //in percent
byte  SPEED_MAX_SET_TO = 80;      //in percent 78
byte  PWM_MAX_DECIMAL = 255;
byte  PWM_MIN_DECIMAL = 0;
double  MAX_VALUE_CONTROLLER = 1;
double  MIN_VALUE_CONTROLLER = 0;
double  turn_speed_const = 
                            //60/SPEED_MAX_SET_TO;
                            0.8;
double  run_speed_const  = 
                            //48/SPEED_MAX_SET_TO;
                            0.5;

byte adj_speed = (byte)map(SPEED_MAX_SET_TO, SPEED_MIN, SPEED_MAX, PWM_MIN_DECIMAL, PWM_MAX_DECIMAL);

#endif

byte vector_speed = 0;
float move_const = 0.5;

//const int motor11 = 22;  // Pin 14 of L293
//const int motor12  = 23;  // Pin 10 of L293
//const int motor1 = 24;
//const int motor2 = 25;
//const int motor21  = 26;  // Pin 14 of L293
//const int motor22  = 27;  // Pin 10 of L293
////motorbelakang
//const int motor31  = 28; // Pin  7 of L293
//const int motor32  = 29;  // Pin  2 of L293
//const int motor3 = 30;
//const int motor4 = 31;
//const int motor41  = 32; // Pin  7 of L293
//const int motor42  = 33;  // Pin  2 of L293

#ifdef USE_LONG_DATA
typedef std_msgs::Float64   sDataPack;
#else
typedef std_msgs::Float32   sDataPack;
#endif

unsigned long   counts, counts_minutes;                 //variable for GM Tube events
//unsigned long   cpm, cps;                    //variable for CPM
float   cpm, cps;
//unsigned long   cps;
unsigned int    multiplier;             //variable for calculation CPM in this sketch
unsigned long   previousMillis = 0, previousMillis_inMinutes = 0;     //variable for time measurement
unsigned long   Dosis;                  //variable for dose measurement
unsigned int    t;                      //variable for waktu batas
unsigned int    eror;                   //variable for waktu batas
unsigned int    n_cacah = 1;

//OLD
////motor front configuration
//const int input_1_A_front = 23;
//const int input_2_A_front  = 22;
//const int input_3_B_front  = 33;
//const int input_4_B_front  = 32;
//
//const int enable_A_front = 24;              //kuning
//const int enable_B_front = 25;              //jingga

////motorbelakang
//const int input_1_A_rear  = 26;
//const int input_2_A_rear  = 27;
//const int input_3_B_rear  = 29;
//const int input_4_B_rear  = 28;
//
//const int enable_A_rear = 30;               //hitamn
//const int enable_B_rear = 31;               //putih
//
//const int enable_A_rear = 6;
//const int enable_B_rear = 7;

//NEW with speed conf
//motor front configuration
const int input_1_A_front  = 23;
const int input_2_A_front  = 22;
const int input_3_B_front  = 29;
const int input_4_B_front  = 28;

const int enable_A_front = 4;
const int enable_B_front = 5;

//motorbelakang
const int input_1_A_rear  = 24;
const int input_2_A_rear  = 25;
const int input_3_B_rear  = 27;
const int input_4_B_rear  = 26;

const int enable_A_rear = 6;
const int enable_B_rear = 7;


#ifdef USE_ODOMETRY
const int front_left_encoder_A = 34;
const int front_left_encoder_B = 35;
const int front_right_encoder_A = 36;
const int front_right_encoder_B = 37;

const int rear_left_encoder_A = 38;
const int rear_left_encoder_B = 39;
const int rear_right_encoder_A = 40;
const int rear_right_encoder_B = 41;

int front_left_wheel_revolution = 0;
int front_right_wheel_revolution = 0;
int rear_left_wheel_revolution = 0;
int rear_right_wheel_revolution = 0;

int signal_revolution = 11;

int rot1, rot2, rot3, rot4;
#endif

#ifdef LOG_PERIOD
  int number = 0;
#endif

//define


void tube_impulse(){       //subprocedure for capturing events from Geiger Kit
  counts++;
}


//pin configurations
void pin_configuration()
{
  pinMode(input_1_A_front, OUTPUT);
  pinMode(input_2_A_front, OUTPUT);
  pinMode(input_3_B_front, OUTPUT);
  pinMode(input_4_B_front, OUTPUT);
  pinMode(input_1_A_rear, OUTPUT);
  pinMode(input_2_A_rear, OUTPUT);
  pinMode(input_3_B_rear, OUTPUT);
  pinMode(input_4_B_rear, OUTPUT);
  pinMode(enable_A_front, OUTPUT);
  pinMode(enable_B_front, OUTPUT);
  pinMode(enable_A_rear, OUTPUT);
  pinMode(enable_B_rear, OUTPUT);
  pinMode(2, INPUT);

#ifdef USE_ODOMETRY
  // pinMode(front_left_encoder_A, INPUT);
  // pinMode(front_left_encoder_B, INPUT);
  // pinMode(front_right_encoder_A, INPUT);
  // pinMode(front_right_encoder_B, INPUT);

  // pinMode(rear_left_encoder_A, INPUT);
  // pinMode(rear_left_encoder_B, INPUT);
  // pinMode(rear_right_encoder_A, INPUT);
  // pinMode(rear_right_encoder_B, INPUT);
#endif
}

//objects define

ros::NodeHandle  nh;

#if !defined USE_PACK_DATA_GEIGER
sDataPack   cps_msgs;
sDataPack   cpm_msgs;
#else
struct _geiger_pack
{
  sDataPack cps;
  sDataPack cpm;
};
//_geiger_pack geiger_counter_pack;
 sDataPack   geiger_counter_array[2];
#endif

#ifdef USE_ODOMETRY
Rotary_Encoder frontLeftEncoder(front_left_encoder_A, front_left_encoder_B);
Rotary_Encoder frontRightEncoder(front_right_encoder_A, front_right_encoder_B);
Rotary_Encoder rearLeftEncoder(rear_left_encoder_A, rear_left_encoder_B);
Rotary_Encoder rearRightEncoder(rear_right_encoder_A, rear_right_encoder_B);
#endif

ros::Subscriber<geometry_msgs::Twist>  velocity_cmd("exa_robot/cmd_vel", &callback_msg);
//ros::Subscriber<geometry_msgs::Twist>  velocity_cmd("cmd_vel", &callback_msg);

#if !defined USE_PACK_DATA_GEIGER
#ifdef USE_CPS
ros::Publisher    CPS("/exa_robot/geiger_sensor_cps", &cps_msgs);
#endif
#ifdef USE_CPM
ros::Publisher    CPM("/exa_robot/geiger_sensor_cpm", &cpm_msgs);
#endif
#else
 ros::Publisher    geiger_counter("geiger_counter", &geiger_counter_msgs[2]);
//ros::Publisher    geiger_counter("/exa_robot/geiger_sensor", &geiger_counter_pack);
#endif

//boolean "maju" = true, "mundur" = true, berhenti = true;
//This will run only one time.
void setup()
{
  pin_configuration();
  nh.initNode();
  nh.subscribe(velocity_cmd);
#if !defined USE_PACK_DATA_GEIGER
#ifdef USE_CPS
  nh.advertise(CPS);
#endif
#ifdef USE_CPM  
  nh.advertise(CPM);
#endif  
#else
  nh.advertise(geiger_counter);
#endif
  counts = 0;
  cpm = 0;
  multiplier = MAX_PERIOD / LOG_PERIOD;      //calculating multiplier, depend on your log period
  Serial.begin(57600);
#ifdef SHOW_LOG_MONITOR  
  Serial.print("with limit ");
  Serial.println(LOG_PERIOD);
#endif
//  Serial.begin(9600); //TX1 RX1 18 19
  attachInterrupt(0, tube_impulse, FALLING); //define external interrupts 
}

void loop()
{
#if !defined USE_GM_IN_CALLBACK  
  geiger_counter_calculation();
#endif  
  nh.spinOnce();
  delay(90);
}

void geiger_counter_calculation()
{
  unsigned long currentMillis = millis();
  if((currentMillis - previousMillis) >= LOG_PERIOD)
  {
      cpm   = (counts) * multiplier;
      
#if defined USE_COUNT_TO_MINUTE      
      counts_minutes += counts;
#endif      
      
#if !defined USE_PACK_DATA_GEIGER
#ifdef USE_CPS
      cps   = cpm/60;      
      cps_msgs.data = cps;
      CPS.publish(&cps_msgs);
#endif
#ifdef USE_CPM      
      cpm_msgs.data = cpm;
      CPM.publish(&cpm_msgs);
#endif      
#else      
      // geiger_counter_msgs[0].data = cps;
      // geiger_counter.publish(&geiger_counter_msgs[0]);
      geiger_counter_pack.cps.data = cps;
      // geiger_counter.publish(&geiger_counter_pack);
#endif
      
#ifdef SHOW_LOG_MONITOR
      number++;
      Serial.print(number);     
      Serial.print("\t");
      Serial.print("time:");
      Serial.print("\t");
      Serial.print(currentMillis - previousMillis);
      Serial.print("\t");
      Serial.print("counts:");     
      Serial.print("\t");
      Serial.print(counts);
      Serial.print("\t");
      Serial.print("cps:");     
      Serial.print("\t");
      Serial.print(cps);
      Serial.print("\t");
      Serial.print("cpm:");
      Serial.print("\t");
      Serial.println(cpm);
#endif
      previousMillis = currentMillis;
      counts = 0;
  }
#if defined USE_COUNT_TO_MINUTE
  if((currentMillis - previousMillis_inMinutes) >= MAX_PERIOD)
  {
    cpm = (counts_minutes) * multiplier;
    previousMillis_inMinutes = currentMillis;
#if !defined USE_PACK_DATA_GEIGER    
    cpm_msgs.data = cpm;
    CPM.publish(&cpm_msgs);
#else    
    // geiger_counter_msgs[1].data = cpm;
    // geiger_counter.publish(&geiger_counter_msgs[1]);
    geiger_counter_pack.cpm.data = cpm;
    // geiger_counter.publish(&geiger_counter_pack.cpm);
#endif

#ifdef SHOW_LOG_MONITOR
      Serial.print("minutes : ");      
      Serial.println(counts_minutes);
      Serial.print("cpm : ");     
      Serial.println(cpm);
#endif
    
    counts_minutes = 0;
  }
#endif  

#if defined USE_PACK_DATA_GEIGER
  geiger_counter.publish(&geiger_counter_pack);
#endif

#ifdef USE_ODOMETRY
  // rot1 = digitalRead(rear);
  // Serial.println(rot1);
#endif
}

#ifdef USE_ODOMETRY
void read_encoder()
{
  frontLeftEncoder.update();
  frontRightEncoder.update();
  rearLeftEncoder.update();
  rearRightEncoder.update();
}
#endif

int min_x_value = 0.0,
    max_x_value = 0 ,
    min_z_value = 0.0,
    max_z_value = 0;


double vector_value(auto x, auto z)
{
  return sqrt((x*x) + (z*z));
}

void steering(auto param_z , auto param_x)
{
//  Serial.print("test : ");
//  Serial.println(param_z);
//  if (param_x < -7 || param_x > 7) param_x = 0;
//  if (param_z < -7 || param_z > 7) param_z = 0;
//if (param_x > -0.1 && param_x < 0) param_x = 0;
 
#ifdef USE_SPEED_CONTROL
  vector_speed = (byte)map(vector_value(param_z, param_x), MIN_VALUE_CONTROLLER, MAX_VALUE_CONTROLLER, SPEED_MIN, adj_speed);
#endif

  if (param_x > min_x_value)
  {
    if (param_z > min_z_value)
    {
#ifdef USE_SLOW_MOVEMENT
      turn_right_forward(vector_speed);
#else
      turn_right(vector_speed * turn_speed_const);
#endif      
    }
    if (param_z < -min_z_value)
    {      
#ifdef USE_SLOW_MOVEMENT
      turn_left_forward(vector_speed);
#else
      turn_left(vector_speed  * turn_speed_const);
#endif
    }
    if (param_z == 0)
    {
      run_forward(vector_speed * run_speed_const);
    }
  }
  //reverse
  if (param_x < -min_x_value)
  {
    if (param_z > min_z_value)
    {
#ifdef USE_SLOW_MOVEMENT
      turn_right_reverse(vector_speed);
#else
      turn_right(vector_speed * turn_speed_const);
#endif
    }
    if (param_z < -min_z_value)
    {
#ifdef USE_SLOW_MOVEMENT
      turn_left_reverse(vector_speed);
#else
      turn_left(vector_speed * turn_speed_const);
#endif
    }
    if (param_z == 0)
    {
      run_reverse(vector_speed * run_speed_const);
    }
  }
  if (param_x == -min_x_value)
  {
    if (param_z > min_z_value)
    {
      turn_right(vector_speed * turn_speed_const);
    }
    if (param_z < -min_z_value)
    {
      turn_left(vector_speed * turn_speed_const);
    }
    if (param_z == 0)
    {
      force_stop_all();
    }
  }

//
//#else
//
//  if (param_x > min_x_value)
//  {
//    if (param_z > min_z_value)
//    {
//      turn_right();
//    }
//    if (param_z < -min_z_value)
//    {
//      turn_left();
//    }
//    if (param_z == 0)
//    {
//      run_forward();
//    }
//  }
//  //reverse
//  if (param_x < -min_x_value)
//  {
//    if (param_z > min_z_value)
//    {
//      turn_right();
//    }
//    if (param_z < -min_z_value)
//    {
//      turn_left();
//    }
//    if (param_z == 0)
//    {
//      run_reverse();
//    }
//  }
//
//#endif //USE_SPEED_CONTROL

  if (abs(param_x) == 0 && abs(param_z) == 0)
  {
    force_stop_all();
  }
}

void callback_msg(const geometry_msgs::Twist& get_vel)
{
  steering(get_vel.angular.z, get_vel.linear.x);
#if defined USE_GM_IN_CALLBACK
  geiger_counter_calculation();
#endif
}

//#if defined USE_SPEED_CONTROL
#if defined USE_OLD_COMMAND

void motor_set(int m, String n, byte _speed_)
{
  if (m == 1 && n == "maju") motor_execution(enable_A_front, input_1_A_front, input_2_A_front, _speed_);
  if (m == 1 && n == "mundur") motor_execution(enable_A_front, input_2_A_front, input_1_A_front, _speed_);
  if (m == 2 && n == "maju") motor_execution(enable_B_front, input_3_B_front, input_4_B_front, _speed_);
  if (m == 2 && n == "mundur") motor_execution(enable_B_front, input_4_B_front, input_3_B_front, _speed_);
  if (m == 3 && n == "maju") motor_execution(enable_A_rear, input_1_A_rear, input_2_A_rear, _speed_);
  if (m == 3 && n == "mundur") motor_execution(enable_A_rear, input_2_A_rear, input_1_A_rear, _speed_);
  if (m == 4 && n == "maju") motor_execution(enable_B_rear, input_3_B_rear, input_4_B_rear, _speed_);
  if (m == 4 && n == "mundur") motor_execution(enable_B_rear, input_4_B_rear, input_3_B_rear, _speed_);
  if (m == 1 && n == "berhenti") motor_stop(enable_A_front);
  if (m == 2 && n == "berhenti") motor_stop(enable_B_front);
  if (m == 3 && n == "berhenti") motor_stop(enable_A_rear);
  if (m == 4 && n == "berhenti") motor_stop(enable_B_rear);
}

void run_forward(byte speed_)
{
#ifdef USE_CASTOR
#ifdef USE_FRONT_WHEEL   
  motor_set(1, "maju", speed_);
  motor_set(2, "maju", speed_);
#endif  
#ifdef USE_REAR_WHEEL  
  motor_set(3, "maju", speed_);
  motor_set(4, "maju", speed_);
#endif
#else
  motor_set(1, "maju", speed_);
  motor_set(2, "maju", speed_);
  motor_set(3, "maju", speed_);
  motor_set(4, "maju", speed_);
#endif  
}

void run_reverse(byte speed_)
{
#ifdef USE_CASTOR  
#ifdef USE_FRONT_WHEEL   
  motor_set(1, "mundur", speed_);
  motor_set(2, "mundur", speed_);
#endif  
#ifdef USE_REAR_WHEEL  
  motor_set(3, "mundur", speed_);
  motor_set(4, "mundur", speed_);
#endif
#else
  motor_set(1, "mundur", speed_);
  motor_set(2, "mundur", speed_);
  motor_set(3, "mundur", speed_);
  motor_set(4, "mundur", speed_);
#endif   
}

void turn_right(byte speed_)
{
#ifdef USE_CASTOR  
#ifdef USE_FRONT_WHEEL   
  motor_set(1, "mundur", speed_);
  motor_set(2, "maju", speed_);
#endif  
#ifdef USE_REAR_WHEEL  
  motor_set(3, "mundur", speed_);
  motor_set(4, "maju", speed_);
#endif
#else
  motor_set(1, "mundur", speed_);
  motor_set(2, "maju", speed_);
  motor_set(3, "mundur", speed_);
  motor_set(4, "maju", speed_);
#endif 
}

void turn_left(byte speed_)
{
#ifdef USE_CASTOR  
#ifdef USE_FRONT_WHEEL   
  motor_set(1, "maju", speed_);
  motor_set(2, "mundur", speed_);
#endif  
#ifdef USE_REAR_WHEEL
  motor_set(3, "maju", speed_);
  motor_set(4, "mundur", speed_);
#endif  
#else
  motor_set(1, "maju", speed_);
  motor_set(2, "mundur", speed_);
  motor_set(3, "maju", speed_);
  motor_set(4, "mundur", speed_);
#endif
}

void turn_right_forward(byte speed_)
{
#ifdef USE_CASTOR  
#ifdef USE_FRONT_WHEEL   
  motor_set(1, "maju", speed_ * move_const);
  motor_set(2, "maju", speed_);
#endif  
#ifdef USE_REAR_WHEEL  
  motor_set(3, "maju", speed_ * move_const);
  motor_set(4, "maju", speed_);
#endif  
#else
  motor_set(1, "maju", speed_ * move_const);
  motor_set(2, "maju", speed_);
  motor_set(3, "maju", speed_);
  motor_set(4, "maju", speed_);
#endif
}

void turn_left_forward(byte speed_)
{
#ifdef USE_CASTOR  
#ifdef USE_FRONT_WHEEL   
  motor_set(1, "maju", speed_);
  motor_set(2, "maju", speed_ * move_const);
#endif  
#ifdef USE_REAR_WHEEL
  motor_set(3, "maju", speed_);
  motor_set(4, "maju", speed_ * move_const);
#endif  
#else
  motor_set(1, "maju", speed_);
  motor_set(2, "maju", speed_ * move_const);
  motor_set(3, "maju", speed_);
  motor_set(4, "maju", speed_);
#endif
}

void turn_right_reverse(byte speed_)
{
#ifdef USE_CASTOR  
#ifdef USE_FRONT_WHEEL   
  motor_set(1, "mundur", speed_ * move_const);
  motor_set(2, "mundur", speed_);
#endif  
#ifdef USE_REAR_WHEEL  
  motor_set(3, "mundur", speed_ * move_const);
  motor_set(4, "mundur", speed_);
#endif  
#else
  motor_set(1, "mundur", speed_);
  motor_set(2, "mundur", speed_);
  motor_set(3, "mundur", speed_ * move_const);
  motor_set(4, "mundur", speed_);
#endif
}

void turn_left_reverse(byte speed_)
{
#ifdef USE_CASTOR  
#ifdef USE_FRONT_WHEEL   
  motor_set(1, "mundur", speed_);
  motor_set(2, "mundur", speed_ * move_const);
#endif  
#ifdef USE_REAR_WHEEL
  motor_set(3, "mundur", speed_);
  motor_set(4, "mundur", speed_ * move_const);
#endif  
#else
  motor_set(1, "mundur", speed_);
  motor_set(2, "mundur", speed_);
  motor_set(3, "mundur", speed_);
  motor_set(4, "mundur", speed_ * move_const);
#endif
}

void force_stop_all()
{
  motor_set(1, "berhenti", 0);
  motor_set(2, "berhenti", 0);
  motor_set(3, "berhenti", 0);
  motor_set(4, "berhenti", 0);
}

#else //USE_OLD_COMMAND

void run_forward(byte speed_)
{
#ifdef USE_CASTOR  
#ifdef USE_FRONT_WHEEL
  motor_execution(enable_A_front, input_1_A_front, input_2_A_front, speed_);
  motor_execution(enable_B_front, input_3_B_front, input_4_B_front, speed_);
#endif
#ifdef USE_REAR_WHEEL
  motor_execution(enable_A_rear, input_1_A_rear, input_2_A_rear, speed_);
  motor_execution(enable_B_rear, input_3_B_rear, input_4_B_rear, speed_);
#endif
#else
  motor_execution(enable_A_front, input_1_A_front, input_2_A_front, speed_);
  motor_execution(enable_B_front, input_3_B_front, input_4_B_front, speed_);
  motor_execution(enable_A_rear, input_1_A_rear, input_2_A_rear, speed_);
  motor_execution(enable_B_rear, input_3_B_rear, input_4_B_rear, speed_);
#endif
}

void run_reverse(byte speed_)
{
#ifdef USE_CASTOR  
#ifdef USE_FRONT_WHEEL  
  motor_execution(enable_A_front, input_2_A_front, input_1_A_front, speed_);
  motor_execution(enable_B_front, input_4_B_front, input_3_B_front, speed_);
#endif  
#ifdef USE_REAR_WHEEL  
  motor_execution(enable_A_rear, input_2_A_rear, input_1_A_rear, speed_);
  motor_execution(enable_B_rear, input_4_B_rear, input_3_B_rear, speed_);
#endif
#else
  motor_execution(enable_A_front, input_2_A_front, input_1_A_front, speed_);
  motor_execution(enable_B_front, input_4_B_front, input_3_B_front, speed_);
  motor_execution(enable_A_rear, input_2_A_rear, input_1_A_rear, speed_);
  motor_execution(enable_B_rear, input_4_B_rear, input_3_B_rear, speed_);
#endif
}

void turn_right(byte speed_)
{
#ifdef USE_CASTOR  
#ifdef USE_FRONT_WHEEL  
  motor_execution(enable_A_front, input_2_A_front, input_1_A_front, speed_);
  motor_execution(enable_B_front, input_3_B_front, input_4_B_front, speed_);
#endif  
#ifdef USE_REAR_WHEEL  
  motor_execution(enable_A_rear, input_1_A_rear, input_2_A_rear, speed_);
  motor_execution(enable_B_rear, input_3_B_rear, input_4_B_rear, speed_);
#endif  
#else
  motor_execution(enable_A_front, input_2_A_front, input_1_A_front, speed_);
  motor_execution(enable_B_front, input_3_B_front, input_4_B_front, speed_);
  motor_execution(enable_A_rear, input_1_A_rear, input_2_A_rear, speed_);
  motor_execution(enable_B_rear, input_3_B_rear, input_4_B_rear, speed_);
#endif
}

void turn_left(byte speed_)
{
#ifdef USE_CASTOR  
#ifdef USE_FRONT_WHEEL  
  motor_execution(enable_A_front, input_1_A_front, input_2_A_front, speed_);
  motor_execution(enable_B_front, input_4_B_front, input_3_B_front, speed_);
#endif  
#ifdef USE_REAR_WHEEL  
  motor_execution(enable_A_rear, input_1_A_rear, input_2_A_rear, speed_);
  motor_execution(enable_B_rear, input_3_B_rear, input_4_B_rear, speed_);
#endif  
#else
  motor_execution(enable_A_front, input_1_A_front, input_2_A_front, speed_);
  motor_execution(enable_B_front, input_4_B_front, input_3_B_front, speed_);
  motor_execution(enable_A_rear, input_1_A_rear, input_2_A_rear, speed_);
  motor_execution(enable_B_rear, input_3_B_rear, input_4_B_rear, speed_);
#endif
}

void turn_right_reverse(byte speed_)
{
#ifdef USE_CASTOR  
#ifdef USE_FRONT_WHEEL
  motor_execution(enable_A_front, input_2_A_front, input_1_A_front, speed_);
  motor_execution(enable_B_front, input_4_B_front, input_3_B_front, speed_);
#endif  
#ifdef USE_REAR_WHEEL  
  motor_execution(enable_A_rear, input_1_A_rear, input_2_A_rear, speed_);
  motor_execution(enable_B_rear, input_4_B_rear, input_3_B_rear, speed_);
#endif  
#else
  motor_execution(enable_A_front, input_2_A_front, input_1_A_front, speed_);
  motor_execution(enable_B_front, input_4_B_front, input_3_B_front, speed_);
  motor_execution(enable_A_rear, input_1_A_rear, input_2_A_rear, speed_);
  motor_execution(enable_B_rear, input_4_B_rear, input_3_B_rear, speed_);
#endif
}

void turn_left_reverse(byte speed_)
{
#ifdef USE_CASTOR  
#ifdef USE_FRONT_WHEEL
  motor_execution(enable_A_front, input_2_A_front, input_1_A_front, speed_);
  motor_execution(enable_B_front, input_4_B_front, input_3_B_front, speed_);
#endif  
#ifdef USE_REAR_WHEEL  
  motor_execution(enable_A_rear, input_2_A_rear, input_1_A_rear, speed_);
  motor_execution(enable_B_rear, input_3_B_rear, input_4_B_rear, speed_);
#endif  
#else
  motor_execution(enable_A_front, input_2_A_front, input_1_A_front, speed_);
  motor_execution(enable_B_front, input_4_B_front, input_3_B_front, speed_);
  motor_execution(enable_A_rear, input_2_A_rear, input_1_A_rear, speed_);
  motor_execution(enable_B_rear, input_3_B_rear, input_4_B_rear, speed_);
#endif
}

void force_stop_all()
{
  motor_stop(enable_A_front);
  motor_stop(enable_B_front);
  motor_stop(enable_A_rear);
  motor_stop(enable_B_rear);
}

#endif //USE_OLD_COMMAND

//#else 
//
//#ifdef USE_OLD_COMMAND
//
//void motor_set(int m, String n)
//{
//  if (m == 1 && n == "maju") motor_execution(enable_A_front, input_1_A_front, input_2_A_front);
//  if (m == 1 && n == "mundur") motor_execution(enable_A_front, input_2_A_front, input_1_A_front);
//  if (m == 2 && n == "maju") motor_execution(enable_B_front, input_3_B_front, input_4_B_front);
//  if (m == 2 && n == "mundur") motor_execution(enable_B_front, input_4_B_front, input_3_B_front);
//  if (m == 3 && n == "maju") motor_execution(enable_A_rear, input_1_A_rear, input_2_A_rear);
//  if (m == 3 && n == "mundur") motor_execution(enable_A_rear, input_2_A_rear, input_1_A_rear);
//  if (m == 4 && n == "maju") motor_execution(enable_B_rear, input_3_B_rear, input_4_B_rear);
//  if (m == 4 && n == "mundur") motor_execution(enable_B_rear, input_4_B_rear, input_3_B_rear);
//  if (m == 1 && n == "berhenti") motor_stop(enable_A_front);
//  if (m == 2 && n == "berhenti") motor_stop(enable_B_front);
//  if (m == 3 && n == "berhenti") motor_stop(enable_A_rear);
//  if (m == 4 && n == "berhenti") motor_stop(enable_B_rear);
//}
//
//void run_forward()
//{
//#ifdef USE_FRONT_WHEEL  
//  motor_set(1, "maju");
//  motor_set(2, "maju");
//#endif  
//#ifdef USE_REAR_WHEEL  
//  motor_set(3, "maju");
//  motor_set(4, "maju");
//#endif  
//}
//
//void run_reverse()
//{
//#ifdef USE_FRONT_WHEEL  
//  motor_set(1, "mundur");
//  motor_set(2, "mundur");
//#endif  
//#ifdef USE_REAR_WHEEL  
//  motor_set(3, "mundur");
//  motor_set(4, "mundur");
//#endif 
//}
//
//void turn_right()
//{
//#ifdef USE_FRONT_WHEEL  
//  motor_set(1, "mundur");
//  motor_set(2, "maju");
//#endif  
//#ifdef USE_REAR_WHEEL  
//  motor_set(3, "maju");
//  motor_set(4, "maju");
//#endif  
//}
//
//void turn_left()
//{
//#ifdef USE_FRONT_WHEEL  
//  motor_set(1, "maju");
//  motor_set(2, "mundur");
//#endif  
//#ifdef USE_REAR_WHEEL  
//  motor_set(3, "maju");
//  motor_set(4, "maju");
//#endif  
//}
//
//void turn_right_reverse()
//{
//#ifdef USE_FRONT_WHEEL   
//  motor_set(1, "mundur");
//  motor_set(2, "mundur");
//#endif  
//#ifdef USE_REAR_WHEEL  
//  motor_set(3, "mundur");
//  motor_set(4, "maju");
//#endif  
//}
//
//void turn_left_reverse()
//{
//#ifdef USE_FRONT_WHEEL   
//  motor_set(1, "mundur");
//  motor_set(2, "mundur");
//#endif  
//#ifdef USE_REAR_WHEEL
//  motor_set(3, "maju");
//  motor_set(4, "mundur");
//#endif  
//}
//
//
//void force_stop_all()
//{
//  motor_set(1, "berhenti");
//  motor_set(2, "berhenti");
//  motor_set(3, "berhenti");
//  motor_set(4, "berhenti");
//}
//
//#else
//
//void run_forward()
//{
//#ifdef USE_FRONT_WHEEL 
//  motor_execution(enable_A_front, input_1_A_front, input_2_A_front);
//  motor_execution(enable_B_front, input_3_B_front, input_4_B_front);
//#endif
//#ifdef USE_REAR_WHEEL  
//  motor_execution(enable_A_rear, input_1_A_rear, input_2_A_rear);
//  motor_execution(enable_B_rear, input_3_B_rear, input_4_B_rear);
//#endif
//}
//
//void run_reverse()
//{
//#ifdef USE_FRONT_WHEEL   
//  motor_execution(enable_A_front, input_2_A_front, input_1_A_front);
//  motor_execution(enable_B_front, input_4_B_front, input_3_B_front);
//#endif  
//#ifdef USE_REAR_WHEEL  
//  motor_execution(enable_A_rear, input_2_A_rear, input_1_A_rear);
//  motor_execution(enable_B_rear, input_4_B_rear, input_3_B_rear);
//#endif  
//}
//
//void turn_right()
//{
//#ifdef USE_FRONT_WHEEL   
//  motor_execution(enable_A_front, input_2_A_front, input_1_A_front);
//  motor_execution(enable_B_front, input_3_B_front, input_4_B_front);
//#endif  
//#ifdef USE_REAR_WHEEL  
//  motor_execution(enable_A_rear, input_1_A_rear, input_2_A_rear);
//  motor_execution(enable_B_rear, input_3_B_rear, input_4_B_rear);
//#endif  
//}
//
//void turn_left()
//{
//#ifdef USE_FRONT_WHEEL   
//  motor_execution(enable_A_front, input_1_A_front, input_2_A_front);
//  motor_execution(enable_B_front, input_4_B_front, input_3_B_front);
//#endif  
//#ifdef USE_REAR_WHEEL  
//  motor_execution(enable_A_rear, input_1_A_rear, input_2_A_rear);
//  motor_execution(enable_B_rear, input_3_B_rear, input_4_B_rear);
//#endif  
//}
//
//void turn_right_reverse()
//{
//#ifdef USE_FRONT_WHEEL
//  motor_execution(enable_A_front, input_2_A_front, input_1_A_front);
//  motor_execution(enable_B_front, input_4_B_front, input_3_B_front);
//#endif  
//#ifdef USE_REAR_WHEEL  
//  motor_execution(enable_A_rear, input_1_A_rear, input_2_A_rear);
//  motor_execution(enable_B_rear, input_4_B_rear, input_3_B_rear);
//#endif  
//}
//
//void turn_left_reverse()
//{
//#ifdef USE_FRONT_WHEEL
//  motor_execution(enable_A_front, input_2_A_front, input_1_A_front);
//  motor_execution(enable_B_front, input_4_B_front, input_3_B_front);
//#endif  
//#ifdef USE_REAR_WHEEL  
//  motor_execution(enable_A_rear, input_2_A_rear, input_1_A_rear);
//  motor_execution(enable_B_rear, input_3_B_rear, input_4_B_rear);
//#endif  
//}
//
//void force_stop_all()
//{
//  motor_stop(enable_A_front);
//  motor_stop(enable_B_front);
//  motor_stop(enable_A_rear);
//  motor_stop(enable_B_rear);
//}
//#endif //USE_OLD_COMMAND
//#endif //USE_SPEED_CONTROL


void motor_execution(const int a, const int b, const int c, byte _motor_speed)
{
#ifdef USE_SPEED_CONTROL
  analogWrite(a, _motor_speed);
#else
  digitalWrite(a, HIGH);
#endif
  digitalWrite(b, LOW);
  digitalWrite(c, HIGH);
}

void motor_stop(const int a)
{
  digitalWrite(a, LOW);
}

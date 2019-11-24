//preproccessing activation
//#define USE_ODOMETRY//
#define USE_OLD_COMMAND

#include "ros.h"
//#include "ros/ros.h"
#include <std_msgs/String.h>
#include <std_msgs/Int32.h>
#include <SPI.h>
#include "geometry_msgs/Twist.h"

#ifdef USE_ODOMETRY
#include "rotary_encoder.h"
#endif


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

//motor front configuration
const int input_1_A_front = 23;
const int input_2_A_front  = 22;
const int input_3_B_front  = 33;
const int input_4_B_front  = 32;
const int enable_A_front = 24;
const int enable_B_front = 25;

//motorbelakang
const int input_1_A_rear  = 26;
const int input_2_A_rear  = 27;
const int input_3_B_rear  = 29;
const int input_4_B_rear  = 28;
const int enable_A_rear = 30;
const int enable_B_rear = 31;

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

//define


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

#ifdef USE_ODOMETRY
Rotary_Encoder frontLeftEncoder(front_left_encoder_A, front_left_encoder_B);
Rotary_Encoder frontRightEncoder(front_right_encoder_A, front_right_encoder_B);
Rotary_Encoder rearLeftEncoder(rear_left_encoder_A, rear_left_encoder_B);
Rotary_Encoder rearRightEncoder(rear_right_encoder_A, rear_right_encoder_B);
#endif

ros::Subscriber<geometry_msgs::Twist>  velocity_cmd("exa_robot/cmd_vel", &callback_msg);

//boolean "maju" = true, "mundur" = true, berhenti = true;
//This will run only one time.
void setup()
{
  pin_configuration();
  nh.initNode();
  nh.subscribe(velocity_cmd);
  Serial.begin(57600);
}

void loop()
{

#ifdef USE_ODOMETRY
  // rot1 = digitalRead(rear);
  // Serial.println(rot1);
#endif
  nh.spinOnce();
  delay(90);
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


void steering(auto param_z , auto param_x)
{
//  Serial.print("test : ");
//  Serial.println(param_z);
//  if (param_x < -7 || param_x > 7) param_x = 0;
//  if (param_z < -7 || param_z > 7) param_z = 0;
//if (param_x > -0.1 && param_x < 0) param_x = 0;
  if (param_x > min_x_value)
  {
    if (param_z > min_z_value)
    {
      turn_right();
    }
    if (param_z < -min_z_value)
    {
      turn_left();
    }
    if (param_z == 0)
    {
      run_forward();
    }
  }
  //reverse
  if (param_x < -min_x_value)
  {
    if (param_z > min_z_value)
    {
      turn_right();
    }
    if (param_z < -min_z_value)
    {
      turn_left();
    }
    if (param_z == 0)
    {
      run_reverse();
    }
  }

//  if(param_z > 0)
//  {
//    turn_right();
//  }
//  
  

//  if (param_x < -min_x_value && param_x > min_x_value)
//  if(param_x == 0)
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
//      force_stop_all();
//    }
//  }
  //stop
  if (param_x == 0 && param_z == 0)
  {
    force_stop_all();
  }
}

void callback_msg(const geometry_msgs::Twist& get_vel)
{
  const geometry_msgs::Twist vel = get_vel;
  steering(get_vel.angular.z, get_vel.linear.x);
}

#ifdef USE_OLD_COMMAND

void motor_set(int m, String n)
{
  if (m == 1 && n == "maju") motor_execution(enable_A_front, input_1_A_front, input_2_A_front);
  if (m == 1 && n == "mundur") motor_execution(enable_A_front, input_2_A_front, input_1_A_front);
  if (m == 2 && n == "maju") motor_execution(enable_B_front, input_3_B_front, input_4_B_front);
  if (m == 2 && n == "mundur") motor_execution(enable_B_front, input_4_B_front, input_3_B_front);
  if (m == 3 && n == "maju") motor_execution(enable_A_rear, input_1_A_rear, input_2_A_rear);
  if (m == 3 && n == "mundur") motor_execution(enable_A_rear, input_2_A_rear, input_1_A_rear);
  if (m == 4 && n == "maju") motor_execution(enable_B_rear, input_3_B_rear, input_4_B_rear);
  if (m == 4 && n == "mundur") motor_execution(enable_B_rear, input_4_B_rear, input_3_B_rear);
  if (m == 1 && n == "berhenti") motor_stop(enable_A_front);
  if (m == 2 && n == "berhenti") motor_stop(enable_B_front);
  if (m == 3 && n == "berhenti") motor_stop(enable_A_rear);
  if (m == 4 && n == "berhenti") motor_stop(enable_B_rear);
}

void run_forward()
{
  motor_set(1, "maju");
  motor_set(2, "maju");
  motor_set(3, "maju");
  motor_set(4, "maju");
}

void run_reverse()
{
  motor_set(1, "mundur");
  motor_set(2, "mundur");
  motor_set(3, "mundur");
  motor_set(4, "mundur");
}

void turn_right()
{
  motor_set(1, "mundur");
  motor_set(2, "maju");
  motor_set(3, "maju");
  motor_set(4, "maju");
}

void turn_left()
{
  motor_set(1, "maju");
  motor_set(2, "mundur");
  motor_set(3, "maju");
  motor_set(4, "maju");
}

void force_stop_all()
{
  motor_set(1, "berhenti");
  motor_set(2, "berhenti");
  motor_set(3, "berhenti");
  motor_set(4, "berhenti");
}

#else

void run_forward()
{
  motor_execution(enable_A_front, input_1_A_front, input_2_A_front);
  motor_execution(enable_B_front, input_3_B_front, input_4_B_front);
  motor_execution(enable_A_rear, input_1_A_rear, input_2_A_rear);
  motor_execution(enable_B_rear, input_3_B_rear, input_4_B_rear);
}

void run_reverse()
{
  motor_execution(enable_A_front, input_2_A_front, input_1_A_front);
  motor_execution(enable_B_front, input_4_B_front, input_3_B_front);
  motor_execution(enable_A_rear, input_2_A_rear, input_1_A_rear);
  motor_execution(enable_B_rear, input_4_B_rear, input_3_B_rear);
}

void turn_right()
{
  motor_execution(enable_A_front, input_2_A_front, input_1_A_front);
  motor_execution(enable_B_front, input_3_B_front, input_4_B_front);
  motor_execution(enable_A_rear, input_1_A_rear, input_2_A_rear);
  motor_execution(enable_B_rear, input_3_B_rear, input_4_B_rear);
}

void turn_left()
{
  motor_execution(enable_A_front, input_1_A_front, input_2_A_front);
  motor_execution(enable_B_front, input_4_B_front, input_3_B_front);
  motor_execution(enable_A_rear, input_1_A_rear, input_2_A_rear);
  motor_execution(enable_B_rear, input_3_B_rear, input_4_B_rear);
}

void force_stop_all()
{
  motor_stop(enable_A_front);
  motor_stop(enable_B_front);
  motor_stop(enable_A_rear);
  motor_stop(enable_B_rear);
}

#endif

void motor_execution(const int a, const int b, const int c)
{
  digitalWrite(a, HIGH);
  digitalWrite(b, LOW);
  digitalWrite(c, HIGH);
}

void motor_stop(const int a)
{
  digitalWrite(a, LOW);
}


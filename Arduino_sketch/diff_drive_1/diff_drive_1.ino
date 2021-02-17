#include <ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float32.h>

#define BAUDRATE 115200 // скорость последовательного порта
///#define DEBUG // включение режима отладки

#define MOTOR_COUNT 2 // кол-во моторов
#define ENCODER_COUNT 2 // кол-во датчиков энкодеров на каждом из моторов

#define DATA_COUNT 2 // кол-во данных (нужные, текущеного)
#define NEEDED 0     // индекс ячейки необходимого значения
#define CURRENT 1    //индекс ячейки текущеного значения

#define ENC_A 0 // индекс 1-ого датчика энкодера
#define ENC_B 1 // индекс 2-ого датчика энкодера

#define LEFT  0 // индекс левого мотора
#define RIGHT 1 // индекс правого мотора

#define FORWARD 1  // команда вперед
#define BACKWARD -1 // команда назад 
#define STOP 0
/////////////////////////////////////////////////////////////////////////////////////////
#define MOTOR_LEFT_1_PIN PB9
#define MOTOR_LEFT_2_PIN PB8
#define MOTOR_LEFT_INVERSE false
#define ENC_LEFT_A_PIN PA9
#define ENC_LEFT_B_PIN PA10

#define MOTOR_RIGHT_1_PIN PB7
#define MOTOR_RIGHT_2_PIN PB6
#define MOTOR_RIGHT_INVERSE false
#define ENC_RIGHT_A_PIN PA8
#define ENC_RIGHT_B_PIN PB15
//////
#define WHEEL_DEAMETR 0.14
#define TIKS_PER_REVOLUTION 1536.0 * 2
#define BASE_WIDTH 0.285

#define PWM_MODE_16

#define PUB_TIME_MS 10

/////////////////////////////////////////////////////////////////////////////////////////

long enc_counter[MOTOR_COUNT];
unsigned long simple_enc_counter[MOTOR_COUNT];
bool sign[MOTOR_COUNT][ENCODER_COUNT];
float speed_wheel[MOTOR_COUNT];

float linear = 0;
float angular = 0;

unsigned long last_pub_time_ms = 0;

#if defined (PWM_MODE_8)
  #define PWM_MODE OUTPUT
  #define MAX_PWM 255
  #define MIN_PWM -255
  #define PWM_write(p,v) analogWrite(p,constrain(v,MIN_PWM, MAX_PWM))
#endif
#if defined (PWM_MODE_16)
  #define PWM_MODE PWM
  #define MAX_PWM 65535
  #define MIN_PWM -65535
  #define PWM_write(p,v) pwmWrite(p,constrain(v,MIN_PWM, MAX_PWM))
#endif

ros::NodeHandle nh;

void update_speeds(const geometry_msgs::Twist& vel)
{
  linear = vel.linear.x;
  angular = vel.angular.z;
}


std_msgs::Float32 right_wheel_distance_val;
std_msgs::Float32 left_wheel_distance_val;

ros::Subscriber<geometry_msgs::Twist> cmd_vel_sub("/banana_robot/cmd_vel", &update_speeds);
ros::Publisher right_wheel_distance_pub("/banana_robot/right_wheel_distance", &right_wheel_distance_val);
ros::Publisher left_wheel_distance_pub("/banana_robot/left_wheel_distance", &left_wheel_distance_val);
void setup()
{
  Serial.begin(BAUDRATE);
  pinMode(ENC_LEFT_A_PIN, INPUT);
  pinMode(ENC_LEFT_B_PIN, INPUT);
  pinMode(MOTOR_LEFT_1_PIN, PWM_MODE);
  pinMode(MOTOR_LEFT_2_PIN, PWM_MODE);

  pinMode(ENC_RIGHT_A_PIN, INPUT);
  pinMode(ENC_RIGHT_B_PIN, INPUT);
  pinMode(MOTOR_RIGHT_1_PIN, PWM_MODE);
  pinMode(MOTOR_RIGHT_2_PIN, PWM_MODE);

  sign[LEFT][ENC_A] = digitalRead(ENC_LEFT_A_PIN);
  sign[LEFT][ENC_B] = digitalRead(ENC_LEFT_B_PIN);
  sign[RIGHT][ENC_A] = digitalRead(ENC_RIGHT_A_PIN);
  sign[RIGHT][ENC_B] = digitalRead(ENC_RIGHT_B_PIN);

  attachInterrupt(ENC_LEFT_A_PIN,  left_enc_A,  CHANGE);
  attachInterrupt(ENC_LEFT_B_PIN,  left_enc_B,  CHANGE);
  attachInterrupt(ENC_RIGHT_A_PIN, right_enc_A, CHANGE);
  attachInterrupt(ENC_RIGHT_B_PIN, right_enc_B, CHANGE);

  nh.getHardware() -> setBaud(57600);
  nh.initNode();
  nh.subscribe(cmd_vel_sub);
  nh.advertise(right_wheel_distance_pub);
  nh.advertise(left_wheel_distance_pub);

  PWM_write(MOTOR_RIGHT_1_PIN, 0);
  PWM_write(MOTOR_RIGHT_2_PIN, 0);
  PWM_write(MOTOR_LEFT_1_PIN, 0);
  PWM_write(MOTOR_LEFT_2_PIN, 0);
}

void left_enc_A()
{
  sign[LEFT][ENC_A] = digitalRead(ENC_LEFT_A_PIN);
  if (sign[LEFT][ENC_B] == sign[LEFT][ENC_A])
  {
    enc_counter[LEFT]++;
  }
  else
  {
    enc_counter[LEFT]--;
  }
  simple_enc_counter[LEFT]++;
}

void left_enc_B()
{
  sign[LEFT][ENC_B] = digitalRead(ENC_LEFT_B_PIN);
  if (sign[LEFT][ENC_B] != sign[LEFT][ENC_A])
  {
    enc_counter[LEFT]++;
  }
  else
  {
    enc_counter[LEFT]--;
  }
}

void right_enc_A()
{
  sign[RIGHT][ENC_A] = digitalRead(ENC_RIGHT_A_PIN);
  if (sign[RIGHT][ENC_B] != sign[RIGHT][ENC_A])
  {
    enc_counter[RIGHT]++;
  }
  else
  {
    enc_counter[RIGHT]--;
  }
  simple_enc_counter[RIGHT]++;
}

void right_enc_B()
{
  sign[RIGHT][ENC_B] = digitalRead(ENC_RIGHT_B_PIN);
  if (sign[RIGHT][ENC_B] == sign[RIGHT][ENC_A])
  {
    enc_counter[RIGHT]++;
  }
  else
  {
    enc_counter[RIGHT]--;
  }
}

void motorLEFTrun(int pwms, int dir)
{
  
  if (dir ==  FORWARD)
  {
    PWM_write(MOTOR_LEFT_1_PIN, 0);
    PWM_write(MOTOR_LEFT_2_PIN, pwms);
  }
  else if (dir ==  BACKWARD)
  {
    PWM_write(MOTOR_LEFT_1_PIN, -pwms);
    PWM_write(MOTOR_LEFT_2_PIN, 0);
  }
  else if (dir == STOP)
  {
    PWM_write(MOTOR_LEFT_1_PIN, 0);
    PWM_write(MOTOR_LEFT_2_PIN, 0);
  }
}

void motorRIGHTrun(int pwms, int dir)
{
  
  if (dir == FORWARD)
  {
    PWM_write(MOTOR_RIGHT_1_PIN, 0);
    PWM_write(MOTOR_RIGHT_2_PIN, pwms);
  }
  else if (dir ==  BACKWARD)
  {
    PWM_write(MOTOR_RIGHT_1_PIN, -pwms);
    PWM_write(MOTOR_RIGHT_2_PIN, 0);

  }
  else if (dir == STOP)
  {
    PWM_write(MOTOR_RIGHT_1_PIN, 0);
    PWM_write(MOTOR_RIGHT_2_PIN, 0);
  }
}

double impulse2meters(float x)
{
  return (x / TIKS_PER_REVOLUTION) * PI * WHEEL_DEAMETR;
}

double impulse2rad(float x)
{
  return (x / TIKS_PER_REVOLUTION) * 2.0 * PI;
}

int getMotorValue(float value) 
{
  return map(value*1000, -1000, 1000, MIN_PWM, MAX_PWM);
}

float getRotationDir(float value) {
  if (value >= 0) {
    if (value == 0) {
      return STOP;
    }
    else
    {
      return FORWARD;
    }
  }
  else
  {
    return BACKWARD;
  }
}



void loop()
{
  float V = -linear;                      //линейная скорость
  float W = angular;                     //угловая скорость
  float r = WHEEL_DEAMETR / 2.0;        //радиус колеса
  float L = BASE_WIDTH;                  //база робота

  // вычисление требуемой скорости вращения колес (рад/с)
  speed_wheel[LEFT] = r * ((1.0 / r) * V - (L / r) * W);
  speed_wheel[RIGHT] = r * ((1.0 / r) * V + (L / r) * W);

//  Serial.print(V);
//  Serial.print(" ");
//  Serial.print(W);
//  Serial.print(" ");
//  Serial.print(speed_wheel[LEFT]);
//  Serial.print(" ");
//  Serial.print(speed_wheel[RIGHT]);
//  Serial.print(" ");
//  Serial.print(getMotorValue(speed_wheel[LEFT]));
//  Serial.print(" ");
//  Serial.print(getMotorValue(speed_wheel[RIGHT]));
//  Serial.print(" ");
//  Serial.print(getRotationDir(speed_wheel[LEFT]));
//  Serial.print(" ");
//  Serial.println(getRotationDir(speed_wheel[RIGHT]));

  motorLEFTrun(getMotorValue(speed_wheel[LEFT]), getRotationDir(speed_wheel[LEFT]));
  motorRIGHTrun(getMotorValue(speed_wheel[RIGHT]), getRotationDir(speed_wheel[RIGHT]));

  if (millis() - last_pub_time_ms >= PUB_TIME_MS)
  {
    right_wheel_distance_val.data = impulse2meters(enc_counter[RIGHT]);
    left_wheel_distance_val.data = impulse2meters(enc_counter[LEFT]);
  
    right_wheel_distance_pub.publish(&right_wheel_distance_val);
    left_wheel_distance_pub.publish(&left_wheel_distance_val);  
    last_pub_time_ms = millis();
  }
  nh.spinOnce();
}

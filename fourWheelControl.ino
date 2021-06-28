#include <SPI.h>
#include "LS7366.h"
#include "SRampGenerator.h"
#include "PIDController.h"
#include "ReceivePackage.h"


// ========================== Motor mode ==============================
#define Motor_enable 18
#define Motor_Brake 19

#define MotorA_PWM 9
#define MotorA_dir 8
#define MotorB_PWM 7
#define MotorB_dir 6
#define MotorC_PWM 5
#define MotorC_dir 4
#define MotorD_PWM 3
#define MotorD_dir 2

// ========================== SPI =================================
#define CS1 10   //right Encoder
#define CS3 22   //left  Encoder

SRampGenerator rampGenerator;
ReceivePackage recData; 
LS7366 rightEncoder(CS1);
LS7366 leftEncoder(CS3);
PIDContorller pidLeft(1000, 60, 0, 4095, -4095);  //Kp, Ki, Kd, max, min
PIDContorller pidRight(1000, 60, 00, 4095, -4095); //Kp, Ki, Kd, max, min


long count_CSLeft = 0;
long count_CSRight = 0;
long count_CSLeft_old = 0;
long count_CSRight_old = 0;


double theta = 0;
double x_world = 0;
double y_world = 0;
bool   init_angle = false;
double error_left_I = 0;
double error_right_I = 0;
double old_want_left = 0;
double old_want_right = 0;

// ==========================  PID ================================
float vl = 0, vr = 0;
IntervalTimer speed_timer;
float _want_vl = 0, _want_vr = 0;
float _now_vl = 0 , _now_vr = 0;
float pre_vl_error = 0, pre_vr_error = 0;



// ========================== Parameter ===========================
const double Two_Wheel_Length = 0.53;        // 兩輪間距 (m) 
const float  Wheel_R = 0.145;                // 輪半徑  (m)  
const int    wheelPluse = 4000.0;             // 皮帶輪1:2 pluse數
const float  ADJUST_R_WHEEL_RATE = 1.0;      // 左排輪子校正係數
const float  ADJUST_L_WHEEL_RATE = 1.0;      // 右排輪子校正係數


//V & W
double v_right = 0;
double v_left = 0;

//encoder
//int st_v = 0,nd_v = 0,rd_v = 0,th_v = 0;


// ========================== ReceivePackage ===========================
int connection = -1;//判斷連線
float vx;//上層給速度x
float vy;//上層給速度y(不用)
float w;//上層給w



unsigned long lastCheckTime = 0; //判斷有無連線用 (儲存上次判斷的當下時間) 
bool speedTimerFlag = false;

void speedTimerISR() 
{    
  //speed_timer 的 Interupt Sub-Routine
  encoder_receive();
  setWheelCar(vx,w);
  speedTimerFlag = true;
}


void setup() {
  Serial.begin(115200);
  Serial1.begin(115200);
  SPI.begin();

  analogWriteResolution(12);   //ADC解析度調整為12bit 0.00122
  //PWM
  pinMode(MotorA_PWM, OUTPUT);
  pinMode(MotorB_PWM, OUTPUT);
  pinMode(MotorC_PWM, OUTPUT);
  pinMode(MotorD_PWM, OUTPUT);
  //Direction
  pinMode(MotorA_dir, OUTPUT);
  pinMode(MotorB_dir, OUTPUT);
  pinMode(MotorC_dir, OUTPUT);
  pinMode(MotorD_dir, OUTPUT);
  //enable break
  pinMode(Motor_enable, OUTPUT);
  pinMode(Motor_Brake, OUTPUT);
  //SPI
  pinMode(CS1, OUTPUT);
  pinMode(CS3, OUTPUT);
  digitalWrite(CS1, LOW);
  digitalWrite(CS3, LOW);

  leftEncoder.write_mode_register_0(FILTER_1 | DISABLE_INDX | FREE_RUN | QUADRX1);
  leftEncoder.write_mode_register_1(NO_FLAGS | EN_CNTR | BYTE_4);
  leftEncoder.clear_counter();
  leftEncoder.clear_status_register();

  rightEncoder.write_mode_register_0(FILTER_1 | DISABLE_INDX | FREE_RUN | QUADRX1);
  rightEncoder.write_mode_register_1(NO_FLAGS | EN_CNTR | BYTE_4);
  rightEncoder.clear_counter();
  rightEncoder.clear_status_register();
  rightEncoder.clear_counter();

  //BK & EN
  digitalWrite(Motor_enable, LOW);
  digitalWrite(Motor_Brake, LOW);

  //timer
  speed_timer.begin(speedTimerISR, 5000);   //5ms 進一次 speedTimerISR
  
  //  Serial.println("Start gen");
  //  rampGenereator.generateVelocityProfile(500,100);
  //  for(int i=0;i<rampGenereator.getTotalTimeFrames();i++){
  //    Serial.println(rampGenereator.getV());
  //  }
}


void loop() {
  if (millis() - lastCheckTime > 100) {
    recData.CheckConnect(connection);
    lastCheckTime = millis();
  }
  if (speedTimerFlag) {
    speedTimerFlag = false;
  }

}


void setWheelCar(float v, float w)  // V[m/s]  W[rad/s]
{
  //Two_Wheel_Kinematics
   vr = v + (w * Two_Wheel_Length / 2.0f);  //Vr = Right PID Target
   vl = v - (w * Two_Wheel_Length / 2.0f);  //Vl = Left PID Target

  _want_vr = vr * ADJUST_R_WHEEL_RATE;  
  _want_vl = vl * ADJUST_L_WHEEL_RATE;  

  float pid_output_Left  = pidLeft.calculate(_want_vl,_now_vl);   //PID control
  float pid_output_Right = pidRight.calculate(_want_vr,_now_vr);  //PID control

  new_VtoPwm_Left(pid_output_Left);
  new_VtoPwm_Right(pid_output_Right);



}


/**************************SetPWM************************************/
uint16_t  new_VtoPwm_Left(float V) {
  if(V >= 0){
    digitalWrite(MotorA_dir,LOW);
    digitalWrite(MotorB_dir,LOW);
  }
  else{
    digitalWrite(MotorA_dir,HIGH);
    digitalWrite(MotorB_dir,HIGH);
  }

  analogWrite(MotorA_PWM, abs(V));
  analogWrite(MotorB_PWM, abs(V)); 
}


uint16_t  new_VtoPwm_Right(float V) {
  if(V >= 0){
    digitalWrite(MotorC_dir,HIGH);
    digitalWrite(MotorD_dir,HIGH);
  }
  else{
    digitalWrite(MotorC_dir,LOW);
    digitalWrite(MotorD_dir,LOW);
  }
  analogWrite(MotorC_PWM, abs(V));
  analogWrite(MotorD_PWM, abs(V)); 
}


// ========================== EncoderReceive ===========================
void encoder_receive() 
{ 
  //里程計累積function 10ms running
  count_CSRight =(long)rightEncoder.read_counter(); 
  count_CSLeft = (long)leftEncoder.read_counter();

  //左右輪Encoder讀回來每5ms的Pluse數
  long count_CSL_dis = count_CSLeft - count_CSLeft_old;  
  long count_CSR_dis = count_CSRight - count_CSRight_old;

  // 輪胎圓周(m)/一圈轉幾格pulse /4000 WheelPluse
  double ressolution = ( 2 * PI * Wheel_R) / wheelPluse; 

  //回授的Encoder速度
  float vl = -(count_CSL_dis  * ressolution / 0.005); //單位 m/s(速度) !!注意正負號
  float vr =  count_CSR_dis  * ressolution / 0.005; //單位 m/s(速度)

  _now_vl = vl;  
  _now_vr = vr;

  count_CSLeft_old = count_CSLeft;
  count_CSRight_old = count_CSRight;

  // Serial1.print("real Vl = ");Serial1.println(_now_vl);
  // Serial1.print("real Vr = ");Serial1.println(_now_vr); 
}


// ========================== *ReceivePackage ===========================
void serialEvent()
{
  if(Serial.available()>0)
  {
    unsigned char Data = Serial.read();
    recData.ReceiveData(Data,&vx,&vy,&w); //only use vx and w
    Serial1.println(vx);
    Serial1.println(" ");
    Serial1.println(w);
  }
}




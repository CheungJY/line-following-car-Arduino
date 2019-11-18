#include <PID_v1.h>

const double i_time=50;//设定单位时间

int flag_left_A=0;
int flag_left_B=0;//标志位设定
int flag_right_A=0;
int flag_right_B=0;

#define Encoder_left_A 9
#define Encoder_left_B 8
#define Encoder_right_A 10
#define Encoder_right_B 13


#define IN1 12
#define IN2 11
#define IN3 4  
#define IN4 7  

#define PWM_right 6
#define PWM_left 5

double real_PWM_left;
double real_PWM_right;//真实转速对应的PWM
double give_PWM_left;
double give_PWM_right;//驱动马达所用的PWM值
double want_PWM_left =60;
double want_PWM_right =60;//设定值
//问题：真实转速转为PWM后是否严格线性


double val_left_A=0;
double val_left_B=0;//用来储存A相B相记录的脉冲数
double val_right_A=0;
double val_right_B=0;

double rpm_left;   
double rpm_right;        //存储转速的变量
unsigned int val_left=0;
unsigned long times;
unsigned long newtime;        //时间变量

double kp_left = 10;
double ki_left = 3;
double kd_left = 1;

double kp_right = 10;
double ki_right = 3;
double kd_right = 5;

PID PID_left(&real_PWM_left,&give_PWM_left,&want_PWM_left,kp_left,ki_left,kd_left,DIRECT);
PID PID_right(&real_PWM_right,&give_PWM_right,&want_PWM_right,kp_right,ki_right,kd_right,DIRECT);

void setup()
{
   Serial.begin(9600);//串口初始化
   
   pinMode(IN1,OUTPUT);
   pinMode(IN2,OUTPUT);
   pinMode(IN3,OUTPUT);
   pinMode(IN4,OUTPUT);
   
   pinMode(PWM_left,OUTPUT);
   pinMode(PWM_right,OUTPUT);
   
   pinMode(Encoder_left_A,INPUT);
   pinMode(Encoder_left_B,INPUT);
   pinMode(Encoder_right_A,INPUT);
   pinMode(Encoder_right_B,INPUT);

   PID_left.SetTunings(kp_left,ki_left,kd_left);
   PID_left.SetOutputLimits(0,255);
   PID_left.SetSampleTime(50);
   PID_left.SetMode(AUTOMATIC);
   PID_right.SetTunings(kp_right,ki_right,kd_right);
   PID_right.SetOutputLimits(0,255);
   PID_right.SetSampleTime(50);
   PID_right.SetMode(AUTOMATIC);
}
void loop()
{
   digitalWrite(IN1,HIGH);//方向
   digitalWrite(IN2,LOW);
   digitalWrite(IN3,HIGH);
   digitalWrite(IN4,LOW);



   
   analogWrite(PWM_left,give_PWM_left);//速度 0~255
   analogWrite(PWM_right,give_PWM_right);

   
   
   newtime=times=millis(); //让这两个时间起点相同
   
   while((newtime-times)<i_time)
   {
      if(digitalRead(Encoder_left_A)==HIGH && flag_left_A==0)
      {
        val_left_A++;
        flag_left_A=1;
        }
      if(digitalRead(Encoder_right_A)==HIGH && flag_right_A==0)//
      {
        val_right_A++;
        flag_right_A=1;
        }
      if(digitalRead(Encoder_left_A)==LOW && flag_left_A==1)
      {
        val_left_A++;
        flag_left_A=0;
        }
       if(digitalRead(Encoder_right_A)==LOW && flag_right_A==1)//
      {
        val_right_A++;
        flag_right_A=0;
        }  
     
        if(digitalRead(Encoder_left_B)==HIGH && flag_left_B==0)
      {
        val_left_B++;
        flag_left_B=1;
        }
       if(digitalRead(Encoder_right_B)==HIGH && flag_right_B==0)//
      {
        val_right_B++;
        flag_right_B=1;
        }  
      if(digitalRead(Encoder_left_B)==LOW && flag_left_B==1)
      {
        val_left_B++;
        flag_left_B=0;
        }
      if(digitalRead(Encoder_right_B)==LOW && flag_right_B==1)//
      {
        val_right_B++;
        flag_right_B=0;
        }  
        newtime=millis();//结束时间
    }                                        //计录AB两相的脉冲数

    
    rpm_left=60*(val_left_A+val_left_B)/1000*52*(1.0*i_time);
    rpm_right=60*(val_right_A+val_right_B)/1000*52*(1.0*i_time);//计算转速

   
    //Serial.println(rpm_right);
    //Serial.println(rpm_left);//输出转速数值
    val_right_A=val_right_B=val_left_A=val_left_B=0;//清零储存脉冲数的变量

    real_PWM_left=map(rpm_left,0,96000,0,255);
    real_PWM_right=map(rpm_right,0,96000,0,255);//映射成PWM 其中96000要根据最大转速对应的rpm调整

    //Serial.println(real_PWM_right);
    //Serial.println(real_PWM_left);//输出转速数值

    PID_left.Compute();
    PID_right.Compute();
    Serial.println(real_PWM_left);
    //Serial.println(give_PWM_left);//输出pid后的控制数值
}

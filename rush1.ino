/******************作者 张家悦*****************/
/******************最后编辑时间19.11.22*****************/
#include <LiquidCrystal_I2C.h>
#include <PID_v1.h>
#include <Wire.h>


const double i_time=50;//设定单位时间

int test=0;

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
double want_PWM_left =0;
double want_PWM_right =0;//设定值


double val_left_A=0;
double val_left_B=0;//用来储存A相B相记录的脉冲数
double val_right_A=0;
double val_right_B=0;

double rpm_left;   
double rpm_right;        //存储转速的变量
unsigned int val_left=0;
unsigned long times;
unsigned long newtime;        //时间变量

double kp_left = 45;
double ki_left = 0.5;
double kd_left = 0.2;

double kp_right = 60;
double ki_right = 0.5;
double kd_right = 0.3;

double real_difference = 0;
double real_difference_abs = 0;
double want_difference = 0;
double give_difference;

double kp_difference = 0.1;
double ki_difference = 100;
double kd_difference = 50;

PID PID_left(&real_PWM_left,&give_PWM_left,&want_PWM_left,kp_left,ki_left,kd_left,DIRECT);
PID PID_right(&real_PWM_right,&give_PWM_right,&want_PWM_right,kp_right,ki_right,kd_right,DIRECT);

PID PID_difference(&real_difference_abs,&give_difference,&want_difference,kp_difference,ki_difference,kd_difference,DIRECT);


LiquidCrystal_I2C lcd(0x3F,16,4);//此处0x3F为检测到的地址

char cmd;//命令

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

   lcd.init();
   lcd.backlight();
   lcd.setCursor(0, 0);
   lcd.print("   H I T S Z");
   lcd.setCursor(0, 1); 
   lcd.print("     W T R");

   digitalWrite(IN1,HIGH);//方向
   digitalWrite(IN2,LOW);
   digitalWrite(IN3,HIGH);
   digitalWrite(IN4,LOW);
}
void loop()
{  
   if(Serial.available()>0)
   {
   cmd = Serial.read();//读取蓝牙模块发送到串口的数据  
   }
   switch(cmd)
   {

    case 01://FORWARD
    FORWARD();
    break;
    case 02:
    TURNLEFT();
    break;
    case 03:
    TURNRIGHT();
    break;
    case 04:
    BACK();
    break;
    /*
    case 06:
    analogWrite(PWM_left,255);//速度 0~255
    analogWrite(PWM_right,255);
    break;
    */
    default:
    STOP();
    break;
    
   }
   
}






void Encoder()
{
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

   
    val_right_A=val_right_B=val_left_A=val_left_B=0;//清零储存脉冲数的变量

    real_PWM_left=map(rpm_left,0,96000,0,255);
    real_PWM_right=map(rpm_right,0,96000,0,255);//映射成PWM 其中96000要根据最大转速对应的rpm调整

}

void Calculate()
{
  Encoder();
  PID_left.Compute();
  PID_right.Compute();
  analogWrite(PWM_left,give_PWM_left);//速度 0~255
  analogWrite(PWM_right,give_PWM_right);
}

void speed_up_Calculate()
{
  Encoder();
  PID_left.Compute();
  PID_right.Compute();

  /*
  real_difference = (real_PWM_left - real_PWM_right + real_difference);
  if (real_difference > 0)
  {
  real_difference_abs = real_difference;
  PID_difference.Compute();
  give_PWM_right = give_PWM_right+ slope(give_difference);
  }
  
  if (real_difference < 0)
  real_difference_abs = -real_difference;
  PID_difference.Compute();
  give_PWM_left = give_PWM_left+ slope(give_difference);

  */

 if (real_PWM_right<10)
 give_PWM_left=give_PWM_left - 18;
// give_PWM_left=slope(give_PWM_left);
  
  analogWrite(PWM_left,give_PWM_left);//速度 0~255
  analogWrite(PWM_right,give_PWM_right);
}

double slope(double t)
{
  if (t>=255){
  t = 255;}
  else if(0<=t<255) {
  t = t;}
  else if(t<0){
  t = 0;  }
  
  
}
void STOP()
{
   lcd.setCursor(0, 0);
   lcd.print("   =========");
   lcd.setCursor(0, 1); 
   lcd.print("     W T R");
  analogWrite(PWM_left,0);//速度 0~255
  analogWrite(PWM_right,0);
}
void FORWARD()
{
   digitalWrite(IN1,HIGH);//方向
   digitalWrite(IN2,LOW);
   digitalWrite(IN3,HIGH);
   digitalWrite(IN4,LOW);
   lcd.setCursor(0, 0);
   lcd.print("   <<<<=>>>>");
   lcd.setCursor(0, 1); 
   lcd.print("     W T R");
  want_PWM_left =15;
  want_PWM_right =15;
  speed_up_Calculate();
  
}
void TURNRIGHT()
{
   digitalWrite(IN1,HIGH);//方向
   digitalWrite(IN2,LOW);
   digitalWrite(IN3,HIGH);
   digitalWrite(IN4,LOW);
   lcd.setCursor(0, 0);
   lcd.print("   >=>=>=>=>");
   lcd.setCursor(0, 1); 
   lcd.print("     W T R");  
  want_PWM_left =25;
  want_PWM_right =5;
  Calculate();
}
void TURNLEFT()
{
   digitalWrite(IN1,HIGH);//方向
   digitalWrite(IN2,LOW);
   digitalWrite(IN3,HIGH);
   digitalWrite(IN4,LOW);
   lcd.setCursor(0, 0);
   lcd.print("   <=<=<=<=<");
   lcd.setCursor(0, 1); 
   lcd.print("     W T R");    
  want_PWM_left =5;
  want_PWM_right =25;
  Calculate();
}
void BACK()
{
   digitalWrite(IN1,LOW);
   digitalWrite(IN2,HIGH);
   digitalWrite(IN3,LOW);
   digitalWrite(IN4,HIGH);

   lcd.setCursor(0,0);
   lcd.print("   >>>>=<<<<");
   lcd.setCursor(0,1);
   lcd.print("     W T R");

   want_PWM_left=10;
   want_PWM_right=10;

   Calculate();

   
}

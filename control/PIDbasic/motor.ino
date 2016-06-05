
#include <PID_v1.h>
#include <SPI.h>
#include <Ethernet.h>
#include <EthernetUdp.h>
//#include <time.h>

#define PWM_R 9
#define PWM_L 5

#define EN_R_BWD 8
#define EN_R_FWD 4

#define EN_L_BWD 6
#define EN_L_FWD 7

#define CURRENT_CHARGE 0
#define CURRENT_LOAD 1

#define encoder0PinA  2
#define encoder0PinB  11

#define encoder1PinA  3
#define encoder1PinB  10



volatile long encoder0Pos=0;
long newposition;
long oldposition = 0;
 long newtime;
 long oldtime = 0;
long vel;


//Define Variables we'll be connecting to
double Setpoint, Input, Output;
double MotorL = 0;
double MotorR = 0;
float avg = 0;
long n = 0;

//Specify the links and initial tuning parameters
double Kp=2, Ki=5, Kd=1;
PID myPIDL(&Input, &MotorL, &Setpoint, Kp, Ki, Kd, DIRECT);
PID myPIDR(&Input, &MotorR, &Setpoint, Kp, Ki, Kd, DIRECT);
void count(void); // code for counting the increasing values of encoder ticks
// the setup function runs once when you press reset or power the board
void setup() {

 //initialize the variables we're linked to
 // Input = analogRead(PIN_INPUT);
 Serial.begin(9600);
pinMode(encoder0PinA, INPUT);
 digitalWrite(encoder0PinA, HIGH);       // turn on pullup resistor
  pinMode(encoder0PinB, INPUT);
  digitalWrite(encoder0PinB, HIGH);       // turn on pullup resistor
  attachInterrupt(0, doEncoder, RISING);  // encoDER ON PIN 2

  //encorderValue=0;
  Setpoint = 250;

  //turn the PID on
  myPIDL.SetMode(AUTOMATIC);
  myPIDR.SetMode(AUTOMATIC);

  myPIDL.SetOutputLimits(255, -255);
  myPIDR.SetOutputLimits(255, -255); 

  
  analogWrite(PWM_R, 0);  
  analogWrite(PWM_L, 0);
  digitalWrite(EN_L_FWD, LOW);
  digitalWrite(EN_L_BWD, LOW);
  digitalWrite(EN_R_FWD, LOW);
  digitalWrite(EN_R_BWD, LOW);

  pinMode(PWM_L, OUTPUT);
  pinMode(PWM_R, OUTPUT);
  pinMode(EN_L_FWD, OUTPUT);
  pinMode(EN_L_BWD, OUTPUT);
  pinMode(EN_R_FWD, OUTPUT);
  pinMode(EN_R_BWD, OUTPUT);
}

void setMotor(const unsigned char cucPWM, const unsigned char cucFWD , const unsigned char cucBWD, const int ciSpeed)
{
  if (ciSpeed < 0)
  {
    digitalWrite(cucFWD, LOW); 
    digitalWrite(cucBWD, LOW);  
    digitalWrite(cucFWD, LOW); 
    digitalWrite(cucBWD, HIGH);  
  }
  else
  {
    digitalWrite(cucFWD, LOW); 
    digitalWrite(cucBWD, LOW);  
    digitalWrite(cucFWD, HIGH); 
    digitalWrite(cucBWD, LOW);  
  }

  analogWrite(cucPWM, abs(ciSpeed));   
}

void doEncoder()
{
  if (digitalRead(encoder0PinA) == digitalRead(encoder0PinB)) {
    encoder0Pos++;
  } else {
    encoder0Pos--;
  }
}

float getAvg(const float prev_avg, const long x, long n)
{
  return (prev_avg*n + x)/(n+1);
}

void loop() {
            
             
             newposition = encoder0Pos;
             newtime = millis();
             if ( newposition > oldposition){
             vel = (newposition-oldposition) * 1000 /(newtime-oldtime);
            }
             Serial.print ("speed = ");
             Serial.println (vel);
           
             Serial.println (newposition);
             Serial.println (oldposition);
              Serial.println (newtime);
             Serial.println (oldtime);
            oldposition = newposition;
            oldtime = newtime;
            delay(200);
          //  avg = getAvg(avg, vel, n++);
            Input = vel/4;
             Serial.print ("Input = ");
              Serial.println (Input);
              myPIDL.Compute();
             myPIDR.Compute();
              Serial.print ("MotorLspeed = ");
             Serial.println (MotorL);
               Serial.print ("MotorRspeed = ");
             Serial.println (MotorR);
             //setMotor(PWM_R, EN_R_FWD, EN_R_BWD, MotorL);
             //setMotor(PWM_L, EN_L_FWD, EN_L_BWD, MotorR);   
             if ( MotorL == 0)
             {
                MotorL = 200;
                MotorR = 200;
             }
           setMotor(PWM_R, EN_R_FWD, EN_R_BWD, MotorL);
           setMotor(PWM_L, EN_L_FWD, EN_L_BWD, MotorR); 
}

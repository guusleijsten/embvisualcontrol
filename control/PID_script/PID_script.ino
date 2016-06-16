#include <PID_v1.h>
#define SLAVE_ADDRESS 0x04

#define InA2            6                       // INA motor pin
#define InB2            7                       // INB motor pin 
#define PWM2            9                       // PWM motor pin
#define encodPinA2      3                       // encoder A pin
#define encodPinB2      10                      // encoder B pin

#define InA1            8                       // INA motor pin
#define InB1            4                       // INB motor pin 
#define PWM1            5                       // PWM motor pin
#define encodPinA1      2                       // encoder A pin
#define encodPinB1      11                      // encoder B pin

#define Vpin            1                       // battery monitoring analog pin
#define Apin            0                       // motor current monitoring analog pin

#define CURRENT_LIMIT   1000                    // high current warning
#define LOW_BAT         10000                   // low bat warning
#define LOOPTIME        100                     // PID loop time
#define NUMREADINGS     10                      // samples for Amp average

#define PI              3.14159

int readings[NUMREADINGS];
int flag2;                                      // flag for whether wheel2 is behind 
int count;                                      // rev counter difference
float Kc = 0.2;                                 // PID for compensation
unsigned long lastMilli = 0;                    // loop timing 
unsigned long lastMilliPrint = 0;               // loop timing
int speed_req = 130;                            // speed (Set Point)
int speed_act1 = 0;                             // speed (actual value)
int speed_act2 = 0;                             // speed (actual value)
float distance1 = 0;
float distance2 = 0;
int PWM_val1 = 0;                               // (25% = 64; 50% = 127; 75% = 191; 100% = 255)
int PWM_val2 = 0;                               // (25% = 64; 50% = 127; 75% = 191; 100% = 255)
int voltage = 0;                                // in mV
int current = 0;                                // in mA
volatile long count1 = 0;                       // rev counter
volatile long count2 = 0;                       // rev counter
float Kp1 =   1;//.4;                           // PID proportional control Gain
float Kd1 =    1;                               // PID Derivitave control gain
float Kp2 =   1;//.4;                           // PID proportional control Gain
float Kd2 =    1;                               // PID Derivitave control gain

void setup()
{
  analogReference(EXTERNAL);                    // Current external ref is 3.3V
  Serial.begin(9600);
  pinMode(InA1, OUTPUT);
  pinMode(InB1, OUTPUT);
  pinMode(PWM1, OUTPUT);
  pinMode(encodPinA1, INPUT); 
  pinMode(encodPinB1, INPUT); 
  digitalWrite(encodPinA1, HIGH);               // turn on pullup resistor
  digitalWrite(encodPinB1, HIGH);
  attachInterrupt(0, rencoder1, RISING);
  attachInterrupt(1, rencoder2, RISING);
  for(int i=0; i<NUMREADINGS; i++)
    readings[i] = 0;                            // initialize readings to 0

  analogWrite(PWM1, PWM_val1);
  digitalWrite(InA1, LOW);
  digitalWrite(InB1, HIGH);

  pinMode(InA2, OUTPUT);
  pinMode(InB2, OUTPUT);
  pinMode(PWM2, OUTPUT);
  pinMode(encodPinA2, INPUT); 
  pinMode(encodPinB2, INPUT); 
  digitalWrite(encodPinA2, HIGH);               // turn on pullup resistor
  digitalWrite(encodPinB2, HIGH);               // turn on pullup resistor
  analogWrite(PWM2, PWM_val2);
  digitalWrite(InA2, LOW);
  digitalWrite(InB2, HIGH);
}

void loop()
{
  getParam();                                                        // check keyboard
  if((millis()-lastMilli) >= LOOPTIME)
  {                                                                  // enter timed loop
    lastMilli = millis();
    getPowerData();
    getMotorData1();
    getMotorData2();
    if (distance1 > 2 || distance2 > 2) {
      speed_req = 0;
    }
    count = compensate();                                            // calculate speed, volts and Amps
    PWM_val1= updatePid1(PWM_val1, speed_req, speed_act1);           // compute PWM value
    analogWrite(PWM1, PWM_val1);                                     // send PWM to motor
    PWM_val2= updatePid2(PWM_val2, speed_req, speed_act2);           // compute PWM value
    analogWrite(PWM2, PWM_val2);
  }
  printMotorInfo();                                                  // display data
}

void getMotorData1()                                                 // calculate speed, volts and Amps
{
  static long countAnt1 = 0;                                         // last count
  speed_act1 = ((count1 - countAnt1)*(60*(1000/LOOPTIME)))/(370);    // 16 pulses X 29 gear ratio = 464 counts per output shaft rev
  countAnt1 = count1;                  
  distance1 = (count2*0.6)/1000;                                     // calculate distance traveled by wheel1
}

void getMotorData2()                                                 // calculate speed, volts and Amps
{
  static long countAnt2 = 0;                                         // last count
  speed_act2 = ((count2 - countAnt2)*(60*(1000/LOOPTIME)))/(370);    // 16 pulses X 29 gear ratio = 464 counts per output shaft rev
  countAnt2 = count2;
  distance2 = (count2*0.6)/1000;                                     // calculate distance traveled by wheel2
}

void getPowerData()
{
  voltage = int(analogRead(Vpin) * 3.22 * 12.2/2.2);                 // battery voltage: mV=ADC*3300/1024, voltage divider 10K+2K
  current = int(analogRead(Apin) * 3.22 * .77 *(1000.0/132.0));      // motor current - output: 130mV per Amp
  current = digital_smooth(current, readings);                       // remove signal noise
}

int updatePid1(int command, int targetValue, int currentValue)       // compute PWM value
{
  float pidTerm = 0;                                                 // PID correction
  int error=0;                                  
  static int last_error=0;                             
  error = abs(targetValue) - abs(currentValue); 
  pidTerm = (Kp1 * error) + (Kd1 * (error - last_error));
  if ( !flag2 )
     pidTerm = pidTerm - Kc * count;  

  last_error = error;
  return constrain(command + int(pidTerm), 0, 255);
}

int updatePid2(int command, int targetValue, int currentValue)       // compute PWM value
{
  float pidTerm = 0;                                                 // PID correction
  int error=0;                                  
  static int last_error = 0;                             
  error = abs(targetValue) - abs(currentValue); 
  pidTerm = (Kp2 * error) + (Kd2 * (error - last_error));  
   if ( flag2 )
     pidTerm = pidTerm - Kc * count;  

  last_error = error;
  return constrain(command + int(pidTerm), 0, 255);
}

void printMotorInfo()                                                // display data
{
  if((millis()-lastMilliPrint) >= 500)   {                     
    lastMilliPrint = millis();
    Serial.print("SP:");            Serial.println(speed_req);Serial.print("flag2:");     Serial.println(flag2); 
    Serial.print("RPM1:");          Serial.print(speed_act1); Serial.print("          "); Serial.print("RPM2:");          Serial.println(speed_act2);
    Serial.print("PWM1:");          Serial.print(PWM_val1);   Serial.print("          "); Serial.print("PWM2:");          Serial.println(PWM_val2);  
    Serial.print("count1:");        Serial.print(count1);     Serial.print("          "); Serial.print("count2:");        Serial.println(count2); 
    Serial.print("dist1:");         Serial.print(distance1);  Serial.print("           ");Serial.print("dist2:");         Serial.println(distance2); 
    //Serial.print("V:");            Serial.println(float(voltage)/1000);
    //Serial.print("mA:");           Serial.println(current);

    if (current > CURRENT_LIMIT)
      Serial.println("*** CURRENT_LIMIT ***");                
    if (voltage > 1000 && voltage < LOW_BAT)
      Serial.println("*** LOW_BAT ***");                
  }
}

void rencoder2()                                      // pulse and direction, direct port reading to save cycles
{
  if (digitalRead(encodPinA2) == 1 &&  digitalRead(encodPinB2) == 0)
    count2++;
  else
    count2--;
}

void rencoder1()                                      // pulse and direction, direct port reading to save cycles
{
  if (digitalRead(encodPinA1) == 1 &&  digitalRead(encodPinB1) == 1)
    count1++;
  else
    count1--;
}


int getParam()
{
  char param, cmd;
  if(!Serial.available())
    return 0;
  delay(10);                  
  param = Serial.read();                              // get parameter byte
  if(!Serial.available())
    return 0;
  cmd = Serial.read();                                // get command byte
  Serial.flush();
  switch (param)
  {
    case 'v':                                         // adjust speed
      if(cmd=='+')  {
        speed_req += 20;
        if(speed_req>140)
          speed_req=140;
      }
      if(cmd=='-')    {
        speed_req -= 20;
        if(speed_req<0)
          speed_req=0;
      }
      break;
    case 's':                                        // adjust direction
      if(cmd=='+'){
        digitalWrite(InA1, LOW);
        digitalWrite(InB1, HIGH);
      }
      if(cmd=='-')   {
        digitalWrite(InA1, HIGH);
        digitalWrite(InB1, LOW);
      }
      break;
    case 'o':                                        // user should type "oo"
      digitalWrite(InA1, LOW);
      digitalWrite(InB1, LOW);
      speed_req = 0;
      break;
    default: 
      Serial.println("???");
  }
}

int digital_smooth(int value, int *data_array)       // remove signal noise
{
static int ndx=0;                                                         
static int count=0;                          
static int total=0;                          
  total -= data_array[ndx];               
  data_array[ndx] = value;                
  total += data_array[ndx];               
  ndx = (ndx+1) % NUMREADINGS;                                
  if(count < NUMREADINGS)
    count++;
  return total/count;
}

int compensate()
{
  int count;
  if ( count2 > count1 ) 
    flag2 = 1;
  else if ( count2 < count1 )
    flag2 = 0;
  return abs (count2 - count1);
}


#include <PID_v1.h>

#define PWM_R 9
#define PWM_L 5

#define EN_R_BWD 8
#define EN_R_FWD 4

#define EN_L_BWD 6
#define EN_L_FWD 7

#define EN_L_STEP 2
#define EN_L_READ 11

#define EN_R_STEP 3
#define EN_R_READ 10

#define CURRENT_CHARGE 0
#define CURRENT_LOAD 1

//PID variables
//Define Variables we'll be connecting to
double Setpoint, Input, Output;

//Variables for calculating step duration
unsigned long prevL = 0, prevR = 0;
bool fStepL = 0, fStepR = 0;//flags

//Variables for averaging speed. (makes it more stable)
unsigned long nL = 0, nR = 0;
float AvgL = 0, PavgL, AvgR = 0, PavgR;

//Variables for setting motor speed!
double MotorL = 0.0;
double MotorR = 200.0;

//Specify the links and initial tuning parameters
PID mRightPID(&Input, &Output, &Setpoint,2,5,1, DIRECT);

// the setup function runs once when you press reset or power the board
void setup() {
  Serial.begin(9600);
  
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
  
  pinMode(EN_L_STEP, INPUT);
  pinMode(EN_L_READ, INPUT);
  
  pinMode(EN_R_STEP, INPUT);
  pinMode(EN_R_READ, INPUT);

  prevL = millis();
  prevR = millis();

  Setpoint = 7;//Fullspeed = 6
  mRightPID.SetControllerDirection(DIRECT);
  //Turn the PID on.
  mRightPID.SetMode(AUTOMATIC);
}

void setMotor(const unsigned char cucPWM, const unsigned char cucFWD, const unsigned char cucBWD, const int ciSpeed)
{
  
  digitalWrite(cucFWD, LOW);
  digitalWrite(cucBWD, LOW);
	if (ciSpeed < 0)
	{
		digitalWrite(cucBWD, HIGH);
	}
	else if (ciSpeed > 0)
	{
		digitalWrite(cucFWD, HIGH);
	}

	analogWrite(cucPWM, abs(ciSpeed));
}

long stepL(const unsigned char cucSTEP, const unsigned char cucREAD)
{
  bool step = digitalRead(cucSTEP);
  bool read = digitalRead(cucREAD);
  if (!read && step)
  {
    if(fStepL) {
      fStepL = 0;
      return millis() - prevL;
    }
  } else if(read && !step) {
    if (!fStepL) {
      fStepL= 1;
      prevL = millis();
    }
  }
  return -1;
}

long stepR(const unsigned char cucSTEP, const unsigned char cucREAD)
{
  bool step = digitalRead(cucSTEP);
  bool read = digitalRead(cucREAD);
  if (!read && step)
  {
    if (fStepR) {
      //Serial.println("Step");
      fStepR = 0;
      return millis() - prevR;
    }
  } else if (read && !step) {
    if(!fStepR) {
      //Serial.println("Read");
      fStepR = 1;
      prevR = millis();
    }
  }
  return -1;
}

// Returns the new average after including x
float getAvg(const float prev_avg, const long x, long n)
{
  return (prev_avg*n + x)/(n+1);
}

void loop() {
  setMotor(PWM_L, EN_L_FWD, EN_L_BWD, (int) MotorL);
  
  long stepTimeL = stepL(EN_L_STEP, EN_L_READ);
  if (stepTimeL > 0) {
    PavgL = AvgL;
    AvgL = getAvg(PavgL, stepTimeL, nL++);
  }
  if (nL % 10 == 1) {
    Serial.print("Left  ");
    Serial.println(AvgL);
  }
  
  setMotor(PWM_L, EN_L_BWD, EN_L_BWD, (int) MotorL);
  
  long stepTimeR = stepR(EN_R_STEP, EN_R_READ);
  if (stepTimeR > 0) {
    PavgR = AvgR;
    AvgR = getAvg(PavgR, stepTimeR, nR++);
  }
  if (nR % 10 == 1) {
    Input = AvgR;
    Serial.print("in ");
    Serial.println(AvgR);
    mRightPID.Compute();
    Serial.print("Right ");
    Serial.println(Output);
  }

  setMotor(PWM_R, EN_R_FWD, EN_R_BWD, (int) MotorR - Output);
	//delay(200);
	
}

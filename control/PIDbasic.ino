#define PWM_R 9
#define PWM_L 5

#define EN_R_BWD 8
#define EN_R_FWD 4

#define EN_L_BWD 6
#define EN_L_FWD 7

#define CURRENT_CHARGE 0
#define CURRENT_LOAD 1

// the setup function runs once when you press reset or power the board
void setup() {
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

void setMotor(const unsigned char cucPWM, const unsigned char cucFWD, const unsigned char cucBWD, const int ciSpeed)
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

void loop() {
	int MotorL = 0;
	int MotorR = 0;

	setMotor(PWM_R, EN_R_FWD, EN_R_BWD, MotorL);
	setMotor(PWM_L, EN_L_FWD, EN_L_BWD, MotorR);
}
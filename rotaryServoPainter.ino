#include <Servo.h> 
#include "Trace.h"
#include "Terminal.h"

//lenghts of arm parts in mm
//#define UPPER_LEN 173
//#define LOWER_LEN 183

#define UPPER_LEN 173
#define LOWER_LEN 173

#define MIN_SHOULDER_ANGLE 0
#define MIN_SHOULDER_POS 2052

#define MAX_SHOULDER_ANGLE PI
#define MAX_SHOULDER_POS 842

#define MIN_ELBOW_ANGLE 0
#define MIN_ELBOW_POS 2100

#define MAX_ELBOW_ANGLE PI
#define MAX_ELBOW_POS 928

#define RAD_TO_DEG 57.295779513082320876798154814105

Servo shoulder, elbow, finger;
float targetX = 0, targetY = 200;

//
float lerp(float x, float in_min, float in_max, float out_min, float out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

//pos in mm
void gotoPos(float x, float y)
{
	tracef("gotoPos x:%f y:%f\r\n", x, y);
	
	float dist = sqrt(x*x + y*y);
	float targetAngle = atan2(y, x);
	
	//get elbow angle to yield correct dist to target
	float triangleAngle2 = acos((UPPER_LEN*UPPER_LEN + LOWER_LEN*LOWER_LEN - dist*dist)/(2*UPPER_LEN*LOWER_LEN));
	float elbowAngle = PI - triangleAngle2;
	
	float triangleAngle1 = asin(LOWER_LEN*sin(elbowAngle)/dist);
	float shoulderAngle = targetAngle - triangleAngle1;
	
	tracef("shoulderAngle:%f elbowAngle:%f\r\n", shoulderAngle*RAD_TO_DEG, elbowAngle*RAD_TO_DEG);

	if (isnan(shoulderAngle) || isnan(elbowAngle))
	{
		return;
	}	
	
	uint16_t shoulderPos = lerp(shoulderAngle, MIN_SHOULDER_ANGLE, MAX_SHOULDER_ANGLE, MIN_SHOULDER_POS, MAX_SHOULDER_POS);
	uint16_t elbowPos = lerp(elbowAngle, MIN_ELBOW_ANGLE, MAX_ELBOW_ANGLE, MIN_ELBOW_POS, MAX_ELBOW_POS);
	
	tracef("shoulderPos:%d elbowPos:%d\r\n", shoulderPos, elbowPos);
	shoulder.writeMicroseconds(constrain(shoulderPos, 850, 2100));
	elbow.writeMicroseconds(constrain(elbowPos, 850, 2100));
}

#define JOYSTICK_X_PIN A1
#define JOYSTICK_Y_PIN A0
#define BTN_PIN A2

#define FINGER_UP_POS 1200
#define FINGER_DOWN_POS 1370

//
void setup()
{
	Serial.begin(115200);
	
	shoulder.attach(23);
	elbow.attach(22);
	finger.attach(21);
	
	shoulder.writeMicroseconds(1500);
	elbow.writeMicroseconds(1500);
	finger.writeMicroseconds(FINGER_UP_POS);
	
	pinMode(BTN_PIN, INPUT_PULLUP);
	
	uint8_t fingerDown = 0;
	
	while(1)
	{
		int joyX = analogRead(JOYSTICK_X_PIN) - 512;
		int joyY = analogRead(JOYSTICK_Y_PIN) - 512;
		//tracef("x:%d y:%d\r\n", x, y);

		if (digitalRead(BTN_PIN) == 0)
		{
			fingerDown = !fingerDown;
			finger.writeMicroseconds(fingerDown ? FINGER_DOWN_POS : FINGER_UP_POS);
			delay(200);
		}
		
		#define THRESHOLD 300
		//#define SPEED 0.2
		#define SPEED 0.8
		//#define SPEED 1.0
		
		if (sqrt(joyX*joyX + joyY*joyY) > 400)
		{
			float joyAngle = atan2(joyY, joyX);
			targetX += -cos(joyAngle) * SPEED;
			targetY += sin(joyAngle) * SPEED;
		}
		
		processSerial();
		
		targetX = constrain(targetX, -180, 100);
		targetY = constrain(targetY, 150, 280);
		
		gotoPos(targetX, targetY);
		delay(10);
	}
}

//
void loop() {}

/*
 * Miguel Sanchez 2106
   Mauro Manco 2016 Porting on ESP8266

   Please note PID gains kp, ki, kd need to be tuned to each different setup.

   Modified by Claudio Arena in 2021
   Added many options useful for use with an OnStep project.
   Multiple PID settings for different operating modes.
   Various Debug options. Base PID library also modified to include extra debug and functionalities.
   PID response can be tuned using the debug option and the Serial Oscilloscope software.
*/

#include <EEPROM.h>
#include <PID_v1.h> //modified library that uses micros instead of millis for timing
#include "constantsAndPins.h"
#include <ESP8266WiFi.h>

void ICACHE_RAM_ATTR encoderInt();
void ICACHE_RAM_ATTR countStep();

void setPins() {
	pinMode(encoder0PinA, INPUT);
	pinMode(encoder0PinB, INPUT);
	pinMode(Step, INPUT);
	pinMode(DIR, INPUT);
	pinMode(M1, OUTPUT);
	digitalWrite(M1, HIGH);
	pinMode(M2, OUTPUT);
	digitalWrite(M2, HIGH);
	pinMode(M_STEPS, INPUT);
	pinMode(EN, INPUT);

#if defined(HWD_DEBUG_ENCODER) || defined(HWD_DEBUG_STEPS)
	pinMode(HW_DEBUG_PIN, OUTPUT);
#endif

	analogWriteFreq(25000);  // set PWM to 25Khz
	analogWriteRange(255);   // set PWM to 255 levels

	attachInterrupt(encoder0PinA, encoderInt, CHANGE);
	attachInterrupt(encoder0PinB, encoderInt, CHANGE);
	attachInterrupt(Step, countStep, RISING);
}

void setup() {
	//Disable software watchdog
	ESP.wdtDisable();

	WiFi.begin();
	WiFi.mode(WIFI_AP);
	WiFi.softAPdisconnect(true);

#ifdef DEBUG
	Serial.begin(1000000);
	help();
#endif
	EEPROM.begin(512);
	recoverPIDfromEEPROM();

	//Setup the pid 
	myPID.SetMode(AUTOMATIC);
	setPIDParameters();
	myPID.SetOutputLimits(OUT_LIM_MIN, OUT_LIM_MAX);
	setPins();
}

void loop() {
	loop1();
}

void setPIDParameters() {
	if (pid_mode == TRACKING) {
		myPID.SetSampleTime(pidTrackingSampleTime);
		//myPID.SetDFilterTime(1000L*10);
		//myPID.SetOutFilterTime(1000L * 3);
		myPID.SetTunings(kp_t, ki_t, kd_t);
	}
	else {
		myPID.SetSampleTime(pidSlewingSampleTime);
		//myPID.SetDFilterTime(1000L*10);
		//myPID.SetOutFilterTime(1000L * 3);
		myPID.SetTunings(kp_s, ki_s, kd_s);
	}
}

void loop1() {
	while (true) {
		curTime = millis() + motorSafe; //Offset time to make sure this works from the start
		ESP.wdtFeed(); //Feed the hardware watchdog

#ifdef DEBUG
		if (Serial.available()) process_line(); // it may induce a glitch to move motion, so use it sparingly
#endif

		//If motor enable
		if (GPIP(EN) == HIGH) {
			setMotorMode(true);

			if (multistep_pin_state != pid_mode) {//Went from slewing to tracking or other way around
				pid_mode = multistep_pin_state;
				setPIDParameters();
			}

			//noInterrupts();
			input = encoder0Pos;
			setpoint = target1;
			//interrupts();
			myPID.Compute();
			//safeMotor();
			pwmOut(output);
		}
		else {
			setMotorMode(false);
			//setpoint = encoder0Pos;
		}

#ifdef SIMULATE_TRACKING
		debug_SimulateTracking();
#endif

#ifdef DEBUG
		if (auto1) if (curTime % 3000 == 0) target1 = random(20000); // that was for self test with no input from main controller
		if (auto2) if (curTime % 1000 == 0) printPos();
#endif

#ifdef PID_TUNE_OUT
		debug_pid_tune();
#endif
	}
}

void setMotorMode(bool enable) {
	if (enable && mode == MANUAL) {
		myPID.SetMode(AUTOMATIC);
		mode = AUTOMATIC;
		attachInterrupt(Step, countStep, RISING);
	}
	if (enable == false && mode == AUTOMATIC) {
		myPID.SetMode(MANUAL);
		mode = MANUAL;
		detachInterrupt(Step);
		pwmOut(0);
	}
}

void debug_SimulateTracking() {
	unsigned long timeChange = (curTime - lastTrackTime);
	if (track && timeChange >= trackInterval) {
		if (pid_mode == TRACKING) {
			target1 = target1 + SINGLESTEP_VALUE;
		}
		else {
			target1 = target1 + MULTISTEP_VALUE;
		}
		lastTrackTime = curTime;
	}
}

void debug_pid_tune() {
	//if(counting && abs(input-target1)<15) counting=false;
	if (counting && (skip++ % POS_SKIP) == 0) {
		pos[p] = input;
		if (p < POS_RECORD - 1) p++;
		else counting = false;
	}
}
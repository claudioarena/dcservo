const int QEM[16] = { 0,-1,1,2,1,0,2,-1,-1,2,0,1,2,1,-1,0 };               // Quadrature Encoder Matrix
static unsigned char New, Old;

void ICACHE_RAM_ATTR encoderInt() { // handle pin change interrupt for D2
#ifdef HWD_DEBUG_ENCODER
	GPOS = (1 << HW_DEBUG_PIN);  //digitalWrite(HW_DEBUG_PIN, 1);
#endif

	Old = New;
	New = GPIP(encoder0PinA) * 2 + GPIP(encoder0PinB); // GPIP = digitalRead

#ifdef HWD_DEBUG_ENCODER
	if (QEM[Old * 4 + New] == 2) {
		GPOC = (1 << HW_DEBUG_PIN); //digitalWrite(HW_DEBUG_PIN, 0);
		GPOS = (1 << HW_DEBUG_PIN);  //digitalWrite(HW_DEBUG_PIN, 1);
		GPOC = (1 << HW_DEBUG_PIN); //digitalWrite(HW_DEBUG_PIN, 0);
	}
#endif

#ifdef INVERT_DIR
	encoder0Pos -= QEM[Old * 4 + New];
#else
	encoder0Pos += QEM[Old * 4 + New];
#endif

#ifdef HWD_DEBUG_ENCODER
	GPOC = (1 << HW_DEBUG_PIN); //digitalWrite(HW_DEBUG_PIN, 0);
#endif
}

void ICACHE_RAM_ATTR countStep() {
#ifdef HWD_DEBUG_STEPS
	GPOS = (1 << HW_DEBUG_PIN);  //digitalWrite(HW_DEBUG_PIN, 1);
#endif
	multistep_pin_state = (GP16I & 0x01); // digitalRead(M_STEP)

	if (GPIP(DIR) == LOW) {// GPIP = digitalRead
		if (multistep_pin_state == SLEWING) {
			target1 = target1 - MULTISTEP_VALUE;
		}
		else {
			target1 = target1 - SINGLESTEP_VALUE;
		}
		//directionLast = -1;
	}
	else {
		if (multistep_pin_state == SLEWING) {
			target1 = target1 + MULTISTEP_VALUE;
		}
		else {
			target1 = target1 + SINGLESTEP_VALUE;
		}
		//directionLast = 1;
	}

#ifdef HWD_DEBUG_STEPS
	GPOC = (1 << HW_DEBUG_PIN); //digitalWrite(HW_DEBUG_PIN, 0);
#endif
}

void pwmOut(int out) {
	//if (out > -50 && out < -50) out = 0;
	//if (out < 40 && out > 4) out = 40;
	//if (out > -40 && out < -4) out = -40;

#ifdef INVERT_DIR
	out = -out;
#endif

//	if (abs(out) < 40) { out = 0; }

	if (out > 0) {
		//GPOC = (1 << M1);
		analogWrite(M1, OUT_LIM_MAX); // digitalWrite(M1, 0); gives problems with ESP8266(PWM keeps being generated)
		analogWrite(M2, OUT_LIM_MAX-out);
	}
	else {
		//GPOC = (1 << M2); //digitalWrite(M2, 0);
		analogWrite(M2, OUT_LIM_MAX); // digitalWrite(M2, 0); gives problems with ESP8266(PWM keeps being generated)
		analogWrite(M1, out-OUT_LIM_MIN);
	}
}

void safeMotor() {
	/* Motor safe code */
	bool onTarget = (input == setpoint); //If we're on target, we're safe
	bool moving = (input != lastEncPos); //If motor has moved, we're safe

			if (onTarget || moving) {
				lastSafeCheck = curTime;
				lastEncPos = input;
				myPID.SetMode(AUTOMATIC);
#ifdef DEBUG
				SafeMessageSent = false;
#endif
			}
			else {
				if (curTime - motorSafe > lastSafeCheck) { //Not safe for more than motorSafe interval
					// we will set the target to current position to stop output
#ifdef DEBUG
					if (SafeMessageSent == false) {
						Serial.println(F("Will decrease output!!!"));
						SafeMessageSent = true;
					}
#endif
					//target1 = input - (output / OUT_LIM_MAX * 5);
					myPID.SetMode(MANUAL);
					if (output > (OUT_LIM_MAX / STARTUP_RATIO)) {
						output = OUT_LIM_MAX / STARTUP_RATIO;
					}

					else if (output < (OUT_LIM_MIN / STARTUP_RATIO)) {
						output = OUT_LIM_MIN / STARTUP_RATIO;
					}
				}
			}

}
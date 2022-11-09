void help() {
	Serial.println(F("\nPID DC motor controller and stepper interface emulator"));
	Serial.println(F("Available serial commands: (lines end with CRLF or LF)"));
	Serial.println(F("P123.34 sets proportional term to 123.34"));
	Serial.println(F("I123.34 sets integral term to 123.34"));
	Serial.println(F("D123.34 sets derivative term to 123.34"));
	Serial.println(F("? prints out current encoder, output and setpoint values"));
	Serial.println(F("! switches between slewing and tracking PID parameter sets"));
	Serial.println(F("X123 sets the target destination for the motor to 123 encoder pulses"));
	Serial.println(F("T starts a sequence of random destinations (between 0 and 2000) every 3 seconds. T again will disable that"));
	Serial.println(F("Q prints out the current values of P, I and D parameters"));
	Serial.println(F("E prints out the set values of P, I and D parameters"));
	Serial.println(F("W stores current values of P, I and D parameters into EEPROM"));
	Serial.println(F("H prints this help message again"));
	Serial.println(F("A toggles on/off showing regulator status every second"));
#ifdef SIMULATE_TRACKING
	Serial.println(F("U toggles on/off debug from PID library"));
	Serial.println(F("Y toggles on/off simulated tracking"));
	Serial.println(F("L sets the simulated tracking interval"));
#endif
	Serial.println(F("B closes the connection\n"));
}

void printPos() {
	Serial.print(F("Position="));
	Serial.print(input);
	Serial.print(F(" PID_output="));
	Serial.print(output);
	Serial.print(F(" Target="));
	Serial.print(setpoint);
	Serial.print(F(" LastCheck="));
	Serial.print(lastSafeCheck);
	Serial.print(F(" LastEncPosCheck="));
	Serial.print(lastEncPos);
	Serial.print(F(" SpeedMode="));
	if (pid_mode == SLEWING) {
		Serial.print("SLEW");
	}
	else {
		Serial.print("TRACK");
	}
	Serial.print(F(" PID par. set="));
	if (pid_parameters_mode == SLEWING) {
		Serial.print("SLEW");
	}
	else {
		Serial.print("TRACK");
	}
	Serial.print(F(" EN="));
	Serial.println(GPIP(EN));
}

void setParameter(char p, float value) {
	if (pid_parameters_mode == TRACKING) {
		switch (p) {
		case 'P':
			kp_t = value;
			break;
		case 'I':
			ki_t = value;
			break;
		case 'D':
			kd_t = value;
			break;
		}		
		setPIDParameters();
	}
	else {
		switch (p) {
		case 'P':
			kp_s = value;
			break;
		case 'I':
			ki_s = value;
			break;
		case 'D':
			kd_s = value;
			break;
		}
		setPIDParameters();
	}
}

float getParameter(char p) {
	float value = 0;

	if (pid_parameters_mode == TRACKING) {
		switch (p) {
		case 'P':
			value = kp_t;
			break;
		case 'I':
			value = ki_t;
			break;
		case 'D':
			value = kd_t;
			break;
		}
	}
	else {
		switch (p) {
		case 'P':
			value = kp_s;
			break;
		case 'I':
			value = ki_s;
			break;
		case 'D':
			value = kd_s ;
			break;
		}
	}
	return value;
}

void process_line() {
	char cmd = Serial.read();
	if (cmd > 'Z') cmd -= 32;
	switch (cmd) {
	case 'P': setParameter('P', Serial.parseFloat()); break;
	case 'I': setParameter('I', Serial.parseFloat()); break;
	case 'D': setParameter('D', Serial.parseFloat()); break;
	case '!': //Change PID parameters mode accessed
		pid_parameters_mode = !pid_parameters_mode;
#ifdef DEBUG
		Serial.println("Switching PID parameters mode");
		Serial.print("PID parameters mode: ");
		if (pid_parameters_mode == TRACKING) {
			Serial.println(" TRACKING");
		}
		else {
			Serial.println(" SLEWING");
		}
#endif
	break;

	case '?': printPos(); break;
	case 'X': target1 = Serial.parseInt(); p = 0; counting = true; for (int i = 0; i < POS_RECORD-1; i++) pos[i] = 0; break;
	case 'T': auto1 = !auto1; break;
	case 'A': auto2 = !auto2; break;
#ifdef SIMULATE_TRACKING
	case 'U': myPID.toggleDebugPrint(); break;
	case 'Y': track = !track; break;
	case 'L': trackInterval = Serial.parseInt(); break;
#endif
	case 'Q': Serial.print("P="); Serial.print(myPID.GetKp()); Serial.print(" I="); Serial.print(myPID.GetKi()); Serial.print(" D="); Serial.println(myPID.GetKd()); break;
	case 'E': Serial.print("P_s="); Serial.print(getParameter('P')); Serial.print(" I_s="); Serial.print(getParameter('I')); Serial.print(" D_s="); Serial.println(getParameter('D')); break;
	case 'H': help(); break;
	case 'W': writetoEEPROM(); break;
	case 'K': eedump(); break;
	case 'R': recoverPIDfromEEPROM(); break;
	case 'S': for (int i = 0; i < p; i++) Serial.println(pos[i]); break;
	}
	//while (Serial.read() != 10); // dump extra characters till LF is seen (you can use CRLF or just LF)
}



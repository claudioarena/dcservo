void eeput(double value, int dir) { // Snow Leopard keeps me grounded to 1.0.6 Arduino, so I have to do this :-(
	char * addr = (char *) &value;
	for (int i = dir; i < dir + 8; i++) {
#ifdef EEPROM_DEBUG
		Serial.print("EEPROM write: ");
		Serial.print(i);
		Serial.print(", ");
		Serial.println(addr[i - dir], HEX);
#endif
		EEPROM.write(i, addr[i - dir]);
		EEPROM.commit();

	}
}

void writetoEEPROM() { // keep PID set values in EEPROM so they are kept when arduino goes off
	eeput(kp_t, 0);
	eeput(ki_t, 8);
	eeput(kd_t, 16);
	eeput(kp_s, 24);
	eeput(ki_s, 32);
	eeput(kd_s, 40);

	double cks = 0;
	for (int i = 0; i < 48; i++)
	{
		double d = EEPROM.read(i);
		cks += d;
#ifdef EEPROM_DEBUG
		Serial.print("EEPROM readback: ");
		Serial.print(i);
		Serial.print(", ");
		Serial.println((char)d, HEX);
#endif
	}
	
	eeput(cks, 48);

#ifdef EEPROM_DEBUG
	Serial.print("EEPROM calculated cks: ");
	Serial.println(cks);
	char * addrd = (char *)&cks;
	Serial.println(addrd[0], HEX);
	Serial.println(addrd[1], HEX);
	Serial.println(addrd[2], HEX);
	Serial.println(addrd[3], HEX);
	Serial.println(addrd[4], HEX);
	Serial.println(addrd[5], HEX);
	Serial.println(addrd[6], HEX);
	Serial.println(addrd[7], HEX);
#endif

#ifdef DEBUG
	Serial.println("\nPID values stored to EEPROM");
#endif
}

double eeget(int dir) { // Snow Leopard keeps me grounded to 1.0.6 Arduino, so I have to do this :-(
	double value;
	char * addr = (char *)&value;
	for (int i = dir; i < dir + 8; i++) {
		addr[i - dir] = EEPROM.read(i);
#ifdef EEPROM_DEBUG
		Serial.print("EEPROM read: ");
		Serial.print(i);
		Serial.print(", ");
		Serial.println(addr[i - dir], HEX);
#endif
	}
	return value;
}

void recoverPIDfromEEPROM() {
	double cks = 0;
	double cksEE;
	for (int i = 0; i < 48; i++) {
		char d = EEPROM.read(i);
		cks += d;
#ifdef EEPROM_DEBUG
		Serial.print("EEPROM readback: ");
		Serial.print(i);
		Serial.print(", ");
		Serial.println(d, HEX);
#endif
	}
	cksEE = eeget(48);
#ifdef EEPROM_DEBUG
	Serial.print("EEPROM read back cks: ");
	Serial.println(cksEE);
	Serial.print("EEPROM calculated cks: ");
	Serial.println(cks);
#endif
	if (cks == cksEE) {
#ifdef DEBUG
		Serial.println(F("*** Found PID values on EEPROM"));
		kp_t = eeget(0);
		ki_t = eeget(8);
		kd_t = eeget(16);
		kp_s = eeget(24);
		ki_s = eeget(32);
		kd_s = eeget(40);
		myPID.SetTunings(kp_t, ki_t, kd_t, proportionalMode);
#endif
	}
	else {
#ifdef DEBUG
		Serial.println(F("*** Bad checksum"));
#endif
	}
}

void eedump() {
	for (int i = 0; i < 48; i++) {
		Serial.print(EEPROM.read(i), HEX);
		Serial.print(" ");
	}
	Serial.println();
}

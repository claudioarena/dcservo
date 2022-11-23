/// DEBUG OPTIONS ///
#define DEBUG
#define DEBUG_TIMING
//#define PID_TUNE_OUT	//Used to register value during movement, for PID settings tune up
//#define EEPROM_DEBUG	//Used to try and figure out problems in saving/loading of values to EEPROM
//#define HWD_DEBUG_STEPS		//Used to toggle hardware pin to troubleshoot ISR
//#define HWD_DEBUG_ENCODER
//#define INVERT_DIR //Inverts the direction the motor turns on +/- steps.
#define SIMULATE_TRACKING

/// PINS ///
#define encoder0PinA		13		// D7
#define encoder0PinB		12		// D6
#define Step				14		// D5
#define M1					0		// D3
#define M2					2		// D4
#define DIR					4		// D2
#define M_STEPS				16		// D0 - HARDCODED
#define EN					5		// D81

#define OUT_LIM_MAX			255
#define OUT_LIM_MIN			-255
#define MULTISTEP_VALUE		32		//32 steps for each step pulse when in multi step mode
#define SINGLESTEP_VALUE	1		//1 steps for each step pulse when not multi step mode
#define HW_DEBUG_PIN		15		//D8, For debug, toggle pin when in ISR

/// OPTIONS AND STATES ///
#define INTERRUPT_METHOD    0
#define LOOP_METHOD         1
#define STEP_MODE           LOOP_METHOD //ESP32 has some issues with multiple interrupts.
//If steps max frequency is not too high, we can process steps in the main loop.
//Loop time is ~ 5 micro seconds, when no motor encoder interrupts. Interrupts take ~2.5 us to enter the ISR, and another 1 us to execute.
//With interrupts from motor spinning, seems to be below 200 us always (encoder ~20kHz, single pin), with no debug or serial prints!
#define STARTUP_RATIO		5.0		//If stalled (NOT USED), output will be OUT_LIM_MAX/STARTUP_RATIO or OUT_LIM_MIN/STARTUP_RATIO

#define POS_RECORD			100	//Points regitered for PID tuning
#define POS_SKIP			200		//Records point only once every this many loops
long pos[POS_RECORD]; int p = 0;

#define TRACKING            1
#define SLEWING             0
bool pid_mode = TRACKING; //change PID parameters depending on the multistep pin status (tracking/slewing) - state machine status
bool pid_parameters_mode = TRACKING; //keeps tracks of which PID parameters we wish to change using serial commands
bool multistep_pin_state = TRACKING; //directly changes during a step interrupt, to follow the multistep pin status

#ifdef SIMULATE_TRACKING
unsigned long lastTrackTime, trackInterval = 100;
bool track = false;
#endif

#define AUTOMATIC 1
#define MANUAL 0
bool mode = AUTOMATIC;

int proportionalMode = P_ON_E; //Default value
//int proportionalMode = P_ON_M;

/// PID SETTINGS ///

//float kp = 2.0, ki = 0.1, kd = 0.02; //No load, 'fast' regime
//float kp = 2.0, ki = 300, kd = 0.02; //No load, 'slow'/'close' regime

//Proportional on error
float kp_s = 0.5, ki_s = 0.3, kd_s = 0.04; //PID for slewing/large movements. D acceptable: >0.04
float kp_t = 2.0, ki_t = 500, kd_t = 0.04; //PID for tracking/small movements. D acceptable: >0.04

//Proportional on measurement
//float kp_s = 0.5, ki_s = 1, kd_s = 0.02; //PID for slewing/large movements
//float kp_t = 1.0, ki_t = 1000, kd_t = 0.02; //PID for tracking/small movements

long pidTrackingSampleTime = 500L;
long pidSlewingSampleTime = 1000L;

/// VARIABLES ///

double input = 0, output = 0, setpoint = 0;
double lastFilteredOutput;
PID myPID(&input, &output, &setpoint, kp_s, ki_s, kd_s, proportionalMode, DIRECT);
//PID myPID(&encoder0Pos, &output, &target1, kp_s, ki_s, kd_s, proportionalMode, DIRECT);

volatile long encoder0Pos = 0;
volatile int directionLast = -1;
boolean auto1 = false, auto2 = false, counting = false;
long previousMillis = 0;        // will store last time LED was updated

long lastEncPos = 0;          // will store the target value when last max output was measured
unsigned long curTime = 0UL;  // will store current time to avoid multiple millis() calls
unsigned long old_t, new_t = 0; //For debug speed of loop in non-interrupt step mode
unsigned long lastSafeCheck = 0;  // will store last value when 255 output was measured
unsigned long motorSafe = 2000UL; // will store the interval to protect the motor - 2 second
int lastMax = 0;              // have to also store the max to avoid +-255 values
byte SafeMessageSent = false; //helps sends out the motor safe serial command only once

volatile long target1 = 0;  // destination location at any moment

bool newStep = LOW;
bool oldStep = LOW;
bool dir = false;
byte skip = 0;

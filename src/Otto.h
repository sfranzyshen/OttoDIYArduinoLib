// OttoDIY Arduino Library project 2024

// Zowi (c) BQ. Released under a GPL licencse 2015

#ifndef Otto_h
#define Otto_h

#include <Arduino.h>

#ifndef Otto_config_h				  // no pre-config ... default to starter kit biped
	#include "Otto_config.h"
	#define  Otto_code			NON_BLOCKING
	//#define  Otto_code			BLOCKING
	#define  Otto_model			BIPED // 4x 180° Servos
	#define  Otto_SERVOS		4
	#define  Otto_sound			SOUND_BUZZER
	//#define  Otto_sound			SOUND_NONE
	#define  Otto_mouth			MOUTH_8X8_MONO_SPI
	//#define  Otto_mouth			MOUTH_NONE
#endif // Otto_config_h

#ifndef Otto_model
	#error "Otto_model is not defined! Please define a Otto Model first"
#endif

#ifndef Otto_SERVOS
	#if Otto_model == BIPED
		#define Otto_SERVOS	4
	#elif Otto_model == BIPED_ARMS
		#define Otto_SERVOS	6
	#elif Otto_model == WHEELS
		#define Otto_SERVOS	2
	#elif Otto_model == WHEELS_ARMS
		#define Otto_SERVOS	4
	#elif Otto_model == NINJA
		#define Otto_SERVOS	4
	#elif Otto_model == NINJA_ARMS
		#define Otto_SERVOS	6
	#else
		#error "Otto_model is not defined! Unknown Otto Model defined"
	#endif	
#endif

#ifndef Otto_code
	#define  Otto_code	BLOCKING	// if not set, default to blocking
#endif

#ifndef Otto_sound
	#define  Otto_sound	SOUND_NONE	// if not set, default to none
#endif

#ifndef Otto_mouth
	#define  Otto_mouth	MOUTH_NONE	// if not set, default to none
#endif

#if defined(ARDUINO_ARCH_AVR)
    #include <Arduino_FreeRTOS.h>    // add the FreeRTOS functions
	#undef delay		     // undefine freertos's delay() wrapper 
	#include <queue.h>
    #include <Servo.h>               // Servo Library
#elif defined(ARDUINO_ARCH_ESP8266)  // https://github.com/alexCajas/esp8266RTOSArduCore/
    #include <ESP32Servo.h>          // Esp-idf Servo Library *untested
#elif defined(ARDUINO_ARCH_ESP32)
    #include <ESP32Servo.h>          // Esp-idf Servo Library
#elif defined(ARDUINO_ARCH_RP2040)   // https://github.com/earlephilhower/arduino-pico
    #include <FreeRTOS.h>            // add the FreeRTOS functions
    #include <Servo.h>               // Servo Library
#endif // ARDUINO_ARCH

#include "Oscillator.h"
#include <EEPROM.h>

#if Otto_sound == SOUND_BUZZER
    #include "Otto_sounds.h"
#endif // SOUND_BUZZER

#if Otto_mouth == MOUTH_8X8_MONO_SPI
	#include "Otto_mouths.h"
	#include "Otto_matrix.h"
#endif // MOUTH_8X8_MONO_SPI

#include "Otto_gestures.h"

// Constants
#define FORWARD     	1
#define BACKWARD    	-1
#define LEFT        	1
#define RIGHT       	-1
#define SMALL       	5
#define MEDIUM      	15
#define BIG         	30

// Servo delta limit default. degree / sec
#define SERVO_LIMIT_DEFAULT 240

class Otto {
public:
    // Otto initialization
    int init(const char * name = "Otto");
    void init(int YL, int YR, int RL, int RR, bool load_calibration, int Buzzer); // compatibility wrapper

    // Attach & detach functions
    void attachServos();
    void detachServos();

    // Oscillator Trims
    void setTrims(int YL, int YR, int RL, int RR);
    void saveTrimsOnEEPROM();

    // Servo limiter
    void enableServoLimit(int speed_limit_degree_per_sec = SERVO_LIMIT_DEFAULT);
    void disableServoLimit();

    // Predetermined Motion Functions
    void _moveServos(int time, int servo_target[]);
    void _moveSingle(int position, int servo_number);
    void oscillateServos(int A[4], int O[4], int T, double phase_diff[4], float cycle = 1.0);
    int Servos_init(int YL, int YR, int RL, int RR, bool load_calibration);
	
    // HOME = Otto at rest position
    void home();
    bool getRestState();
    void setRestState(bool state);

    // Predetermined Motion Functions
    void jump(float steps = 1, int T = 2000);
    void walk(float steps = 4, int T = 1000, int dir = FORWARD);
    void turn(float steps = 4, int T = 2000, int dir = LEFT);
    void bend(int steps = 1, int T = 1400, int dir = LEFT);
    void shakeLeg(int steps = 1, int T = 2000, int dir = RIGHT);
    void updown(float steps = 1, int T = 1000, int h = 20);
    void swing(float steps = 1, int T = 1000, int h = 20);
    void tiptoeSwing(float steps = 1, int T = 900, int h = 20);
    void jitter(float steps = 1, int T = 500, int h = 20);
    void ascendingTurn(float steps = 1, int T = 900, int h = 20);
    void moonwalker(float steps = 1, int T = 900, int h = 20, int dir = LEFT);
    void crusaito(float steps = 1, int T = 900, int h = 20, int dir = FORWARD);
    void flapping(float steps = 1, int T = 1000, int h = 20, int dir = FORWARD);

#if Otto_mouth == MOUTH_8X8_MONO_SPI

    // Mouth & Animations functions for MOUTH_8X8_MONO_SPI
	int Mouth_init(int DIN, int CS, int CLK, int rotate);
    int Mouth_intensity(int intensity, int minimalDuration = FPS30);
    int Mouth_setled(byte X, byte Y, byte value, int minimalDuration = FPS30);
    int Mouth_write(const char * s, byte scrollspeed, bool noblock = Otto_code);
    int Mouth(unsigned long int mouth, bool predefined = true, int minimalDuration = FPS30);
    int Mouth_animation(unsigned long int anim, int index, int minimalDuration = FPS30);
    int Mouth_animate(unsigned long int anim, int speed = FPS30, bool bounce = false, bool loop = true);
    int Mouth_clear(int minimalDuration = FPS30);
    int Mouth_queueSize();
    bool Mouth_isEmptyQueue();
    int Mouth_clearQueue();
	
    // Mouth & Matrix functions for MOUTH_8X8_MONO_SPI compatibility wrappers
    void initMATRIX(int DIN, int CS, int CLK, int rotate);
    void matrixIntensity(int intensity);
    void setLed(byte X, byte Y, byte value);
    void writeText(const char* s, byte scrollspeed);
    void putMouth(unsigned long int mouth, bool predefined = true);
    void putAnimationMouth(unsigned long int anim, int index);
    void clearMouth();

#else // dummy compatibility wrappers
    void initMATRIX(int DIN, int CS, int CLK, int rotate);
    void matrixIntensity(int intensity);
    void setLed(byte X, byte Y, byte value);
    void writeText(const char* s, byte scrollspeed);
    void putMouth(unsigned long int mouth, bool predefined = true);
    void putAnimationMouth(unsigned long int anim, int index);
    void clearMouth();
	
#endif // MOUTH_8X8_MONO_SPI

#if Otto_sound == SOUND_BUZZER
    // Sound functions for SOUND_BUZZER
    int Sound_init(int Buzzer);
    void _tone(float frequency, long noteDuration, int silentDuration); // compatibility wrapper
    int Sound_tone(float frequency, long noteDuration, int silentDuration, bool noblock = Otto_code);
    void bendTones(float initFrequency, float finalFrequency, float prop, long noteDuration, int silentDuration); // compatibility wrapper
    int Sound_bendTones(float initFrequency, float finalFrequency, float prop, long noteDuration, int silentDuration, bool noblock = Otto_code);
    void sing(int songName); // compatibility wrapper
    int Sound_sing(int songName, bool noblock = Otto_code);
    int Sound_queueSize();
    bool Sound_isEmptyQueue();
    int Sound_clearQueue();
#else // dummy compatibility wrappers
    void _tone(float frequency, long noteDuration, int silentDuration); // dummy compatibility wrapper
    void bendTones(float initFrequency, float finalFrequency, float prop, long noteDuration, int silentDuration); // dummy compatibility wrapper
    void sing(int songName); // dummy compatibility wrapper
#endif // SOUND_BUZZER

    // Gestures
    void playGesture(int gesture); // compatibility wrapper
    int  Gesture(int gesture, bool noblock = Otto_code);

private:
#if Otto_sound == SOUND_BUZZER
    static void toneTaskWrapper(void *pvParameters);	// Static wrapper function for toneTask	
    void toneTask(void *pvParameters);    		// Function prototypes
    TaskHandle_t toneTaskHandle = NULL;    		// Define the task handler for playing tones
    QueueHandle_t toneQueueHandle = NULL;           	// Define the queue handler
    int pinBuzzer;
#endif // SOUND_BUZZER

    Oscillator servo[4];
    int servo_pins[4];
    int servo_trim[4];
    unsigned long final_time;
    unsigned long partial_time;
    float increment[4];
    bool isOttoResting;
    void _execute(int A[4], int O[4], int T, double phase_diff[4], float steps = 1.0);
	
#if Otto_mouth == MOUTH_8X8_MONO_SPI
    Otto_Matrix ledmatrix;
    static void mouthTaskWrapper(void *pvParameters);	// Static wrapper function for mouthTask	
    void mouthTask(void *pvParameters);    				// Function prototypes
    TaskHandle_t mouthTaskHandle = NULL;    			// Define the task handler for displaying mouths
    QueueHandle_t mouthQueueHandle = NULL;   			// Define the command queue handler
    unsigned long int getMouthShape(int number);
    unsigned long int getAnimShape(int anim, int index);
#endif // MOUTH_8X8_MONO_SPI

};

#endif // Otto_h

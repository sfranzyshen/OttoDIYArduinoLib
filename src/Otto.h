// OttoDIY Arduino Library project 2024

// Zowi (c) BQ. Released under a GPL licencse 04 December 2015

#ifndef Otto_h
#define Otto_h

#include <Arduino.h>

#ifndef Otto_config_h				 // no pre-config ... default to starter kit biped
	#include "Otto_config.h"
	#define  Otto_code		BLOCKING
	#define  Otto_model		BIPED 	 // 4x 180° Servos
	#define  Otto_sound		SOUND_BUZZER
	#define  Otto_mouth		LED_MATRIX_8X8_MONO_SPI
	#define  Otto_SERVOS		4
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
#include "Otto_sounds.h"
#include "Otto_gestures.h"
#include "Otto_mouths.h"
#include "Otto_matrix.h"

// Constants
#define FORWARD     	1
#define BACKWARD    	-1
#define LEFT        	1
#define RIGHT       	-1
#define SMALL       	5
#define MEDIUM      	15
#define BIG         	30


// Structure to hold tone parameters
struct ToneParameters {
  float frequency;
  long noteDuration;
  int silentDuration;
};

// Servo delta limit default. degree / sec
#define SERVO_LIMIT_DEFAULT 240

class Otto {
public:
    // Otto initialization
    int init(int YL, int YR, int RL, int RR, bool load_calibration, int Buzzer);

    // Attach & detach functions
    void attachServos();
    void detachServos();

    // Oscillator Trims
    void setTrims(int YL, int YR, int RL, int RR);
    void saveTrimsOnEEPROM();

    // Predetermined Motion Functions
    void _moveServos(int time, int servo_target[]);
    void _moveSingle(int position, int servo_number);
    void oscillateServos(int A[4], int O[4], int T, double phase_diff[4], float cycle);

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

    // Mouth & Animations
    void putMouth(unsigned long int mouth, bool predefined = true);
    void putAnimationMouth(unsigned long int anim, int index);
    void clearMouth();

    // Sounds
    void _tone(float frequency, long noteDuration, int silentDuration);
    int Sound_tone(float frequency, long noteDuration, int silentDuration, bool noblock);
    void bendTones(float initFrequency, float finalFrequency, float prop, long noteDuration, int silentDuration);
    void sing(int songName);
    int getToneQueueSize();
    bool isEmptyToneQueue();
    int clearToneQueue();

    // Gestures
    void playGesture(int gesture);
    void initMATRIX(int DIN, int CS, int CLK, int rotate);
    void matrixIntensity(int intensity);
    void setLed(byte X, byte Y, byte value);
    void writeText(const char* s, byte scrollspeed);

    // Servo limiter
    void enableServoLimit(int speed_limit_degree_per_sec = SERVO_LIMIT_DEFAULT);
    void disableServoLimit();

private:
    int pinBuzzer;
    static void toneTaskWrapper(void *pvParameters);// Static wrapper function for toneTask	
    void toneTask(void *pvParameters);    			// Function prototypes
    TaskHandle_t toneTaskHandle = NULL;    			// Define the task handler for playing tones
    QueueHandle_t toneQueueHandle = NULL;           // Define the queue handler
    Oscillator servo[4];
    Otto_Matrix ledmatrix;
    int servo_pins[4];
    int servo_trim[4];
    unsigned long final_time;
    unsigned long partial_time;
    float increment[4];
    bool isOttoResting;
    unsigned long int getMouthShape(int number);
    unsigned long int getAnimShape(int anim, int index);
    void _execute(int A[4], int O[4], int T, double phase_diff[4], float steps);
};

#endif

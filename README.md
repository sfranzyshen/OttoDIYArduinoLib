# OttoDIY Core Code Improvement Project [![License: GPL v3](https://img.shields.io/badge/License-GPLv3-blue.svg)](https://www.gnu.org/licenses/gpl-3.0) ![version](https://img.shields.io/badge/version-0.1.0-blue) 
### Development Branch :star: Star us on GitHub, it helps!

<a href="https://discord.com/channels/691410809586450483/1228242677963685918"><img src="https://images.squarespace-cdn.com/content/v1/5cd3e3917046805e4d6700e3/1560193788834-KYURUXVSZAIE4XX1ZB2F/ke17ZwdGBToddI8pDm48kK6MRMHcYvpidTm-7i2qDf_lfiSMXz2YNBs8ylwAJx2qLijIv1YpVq4N1RMuCCrb3iJz4vYg48fcPCuGX417dnbl3kVMtgxDPVlhqW83Mmu6GipAUbdvsXjVksOX7D692AoyyEsAbPHhHcQMU6bWQFI/join_discord_button_small.png" width="25%"></a>

This branch of the repository has the [OttoDIY robot](https://www.ottodiy.com/) "development" Arduino library for all the [OttoDIY Models](https://github.com/OttoDIY/OttoDIYLib/tree/devel#supported-models) and [compatible boards](https://github.com/OttoDIY/OttoDIYLib/tree/devel?tab=readme-ov-file#supported-platforms)
If you are looking for the production software please refer to the [main branch](https://github.com/OttoDIY/OttoDIYLib).

### Versioning for the project will follow a three-numeral format separated by decimal points, for example, 0.1.0

The first numeral represents the major software version number. Versions with the same major number should maintain full compatibility with each other.
The second numeral indicates the stability number. Odd numbers denote development releases, while even numbers signify stable releases.
The last numeral is the minor release number, incrementally increased to distinguish it from the previous minor release." We are currently at 0.1.0 to 
indicate that there is no version or release yet ... but we are in development. 
#### Our initial development release, version 1.1.1, will aim to maintain complete backwards compatibility with the existing OttoDIY Arduino library.

## Project Objective:
To enhance the core code running OttoDIY in order to address its current limitations and make it more competitive in the educational tech arena.

## Project Scope:
* #### Code Refactoring:
  Evaluate the existing codebase to identify areas for refactoring.
  Refactor the code to improve its structure, readability, and maintainability.
* #### Functionality Enhancement:
  Identify key functionalities that need improvement or addition.
  Enhance existing functionalities to make them more robust and responsive.
  Implement new features to address current limitations and meet user needs.
* #### Reliability Improvement:
  Address any known issues or bugs that affect the reliability of the code.
  Improve error handling and error reporting mechanisms to enhance the stability of the system.
* #### Documentation Update:
  Review and update existing documentation to reflect changes made to the codebase.
  Ensure comprehensive documentation covering installation, usage, and troubleshooting.
* #### Testing and Quality Assurance:
  Develop and execute test cases to ensure the reliability and functionality of the updated code.
  Implement automated testing where feasible to streamline the testing process.
  Conduct thorough quality assurance to identify and address any issues before deployment.
* #### Deployment Plan:
  Develop a plan for deploying the updated code to users.
  Consider methods for migrating existing users to the new version smoothly.
  Provide support and assistance to users during the transition period.

### Constraints and Assumptions:
* #### Limited resources (e.g., developers, testers, time).
* #### Compatibility with existing hardware and software configurations.
* #### User acceptance of the updated code.
* #### Adherence to project timeline.

## Supported Models
  - Biped (Classic) & Biped w/ Arms
  - Wheels & Wheels w/ Arms
  - Ninja & Ninja w/ Arms

## Supported Platforms
  - Arduino Nano, Uno, Micro, Mega, Mini, Leonardo, Nano Every (ATMega*)
  - Arduino Nano Connect, Raspberry Pi Pico (RP2040)
  - Arduino Nano Esp32, ESP32 (ESP32*)
  - ESP8266 (ESP8266)

## Supported Hardware
  ### Outputs (Mouths, Faces, Sounds, and Eyes)  
  - Ultrasonic sensor 3, 4 and 5pin with rgb leds (eyes) ws281x (rgb)
  - Led matrix 8x8 (mouths) spi, i2c (mono) and ws281x (rgb)
  - Led matrix 8x16 (eyes) spi, i2c (mono) and ws281x (rgb)
  - Led matrix 6x20 (eyes) ws281x (rgb)
  - Led matrix 6x10 (mouths) ws281x (rgb)
  - Led matrix 10x16 (eyes) ws281x (rgb)
  - Oled display 128x64 (eyes, faces) spi & i2c (mono)
  - Oled display 128x32 (eyes, mouths) spi & i2c (mono)
  ### Inputs (Distance, Line-follower, Noise, Button, Touch, ...)
  - Ultrasonic sensor 3, 4 and 5pin (gpio, i2c)
  - IR Line-follower Sensor 3 and 4 pin (gpio, analog)
  - microphone 3, 4 and 5 pin (gpio, analog)
  - IR distance sensor 3 pin (analog)
    
## Source File Structure
#### In the development phase, files will be further divided into more files compared to the release file set.

* `Otto.h` and `Otto.cpp` contains all the main otto functions
* `Otto_gestures.h` contains all the otto gestures functions
* `Otto_mouths.h` contains all the mouth functions
* `Otto_moves.h` contains all the otto movement functions
* `Otto_eyes.h` contains all the otto eye functions
* `Otto_sounds.h` contains all the otto sound functions
* `Display.h` and `Display.cpp` contains all the oled and led matrix functions
* `Distance.h` and `Distance.cpp` contains all the distance messurement functions
* `Servo.h` and `Servo.cpp` contains all the servo functions
* `Oscillator.h` and `Oscillator.cpp` is the main algorithm for the servos "smooth" movement
* `Sound.h` and `Sound.cpp` contains all the code for making sounds
* `SerialCommand.h` and `SerialCommand.cpp` is for Bluetooth communication vis Software serial or native Bluetooth

### Current Otto API: (Typical Biped with buzzer and mouth matrix)
```cpp
  #include <Otto.h>
  Otto Otto;

  #define YL      2	// left leg pin
  #define YR      3	// right leg pin
  #define RL      4	// left foot pin
  #define RR      5	// right foot pin
  #define Buzzer  13 	// buzzer pin

  #define CLK     A1    // Clock pin
  #define CS      A2    // Chip Select pin
  #define DIN     A3    // Data In pin
  #define Rotate  0     // 8x8 LED Matrix orientation  Top  = 1, Bottom = 2, Left = 3, Right = 4
	
  void Otto.init(int YL, int YR, int RL, int RR, bool load_calibration, int Buzzer);
  void Otto.initMATRIX(int DIN, int CS, int CLK, int rotate);
  void Otto.matrixIntensity(int intensity);
  void Otto.setLed(byte X, byte Y, byte value);
  void Otto.writeText (const char * s, byte scrollspeed);	
	
  void Otto.setTrims(int TYL, int TYR, int TRL, int TRR);
  void Otto.saveTrimsOnEEPROM();
	
  void Otto._moveServos(int time, int  servo_target[]);
  void Otto._moveSingle(int position,int  servo_number);
  void Otto.oscillateServos(int A[4], int O[4], int T, double phase_diff[4], float cycle);
  bool Otto.getRestState();
  void Otto.setRestState(bool state);
  void Otto.attachServos();
  void Otto.detachServos();
  void Otto.enableServoLimit(int speed_limit_degree_per_sec = SERVO_LIMIT_DEFAULT);
  void Otto.disableServoLimit();
	
  void Otto.home();
  void Otto.jump(float steps=1, int T = 2000);
  void Otto.walk(float steps=4, int T=1000, int dir = FORWARD);
  void Otto.turn(float steps=4, int T=2000, int dir = LEFT);
  void Otto.bend (int steps=1, int T=1400, int dir=LEFT);
  void Otto.shakeLeg (int steps=1, int T = 2000, int dir=RIGHT);
  void Otto.updown(float steps=1, int T=1000, int h = 20);
  void Otto.swing(float steps=1, int T=1000, int h=20);
  void Otto.tiptoeSwing(float steps=1, int T=900, int h=20);
  void Otto.jitter(float steps=1, int T=500, int h=20);
  void Otto.ascendingTurn(float steps=1, int T=900, int h=20);
  void Otto.moonwalker(float steps=1, int T=900, int h=20, int dir=LEFT);
  void Otto.crusaito(float steps=1, int T=900, int h=20, int dir=FORWARD);
  void Otto.flapping(float steps=1, int T=1000, int h=20, int dir=FORWARD);
	
  void Otto.putMouth(unsigned long int mouth, bool predefined = true);
  void Otto.putAnimationMouth(unsigned long int anim, int index);
  void Otto.clearMouth();
    
  void Otto._tone(float noteFrequency, long noteDuration, int silentDuration);
  void Otto.bendTones(float initFrequency, float finalFrequency, float prop, long noteDuration, int silentDuration);
  void Otto.sing(int songName);
    
  void Otto.playGesture(int gesture);
```	
### Proposed Otto API: (Biped with arms model, RGB ultrasonic eyes, buzzer sound, and a mono spi matrix mouth)
```cpp
  #include "Otto_config.h"

  #define Otto_model  BIPED_ARMS
  #define Otto_sound  BUZZER
  #define Otto_mouth  LED_MATRIX_8X8_MONO_SPI
  #define Otto_eyes   LED_USONIC_RGB_NEO

  #include <Otto.h>
  Otto Otto;

  #define LL        2  // left leg pin
  #define LR        3  // right leg pin
  #define FL        4  // left foot pin
  #define FR        5  // right foot pin
  #define AL        6  // left arm pin
  #define AR        7  // right arm pin
  #define Buzzer    13 // buzzer pin

  #define EYES_NEO  12 // ultrasonic eyes neopixel pin

  #define CLK       A1 // Clock pin
  #define CS        A2 // Chip Select pin
  #define DIN       A3 // Data In pin
  #define Rotate    0  // 8x8 LED Matrix orientation  Top  = 1, Bottom = 2, Left = 3, Right = 4
	
  int Otto.init(const char * name); // every Otto should have a unique name. Can be used for Wifi, led matrix, etc.
  int Otto.Servo_init(LL, LR, FL, FR, AL, AR); // initialize the servos
  int Otto.Mouth_init(int DIN, int CS, int CLK, int rotate); // initialize the mouth display
  int Otto.Sound_init(Buzzer); // initialize the sound output device
  int Otto.Eyes_init(EYES_NEO); // initialize the eyes out device
	
  int Otto.Servo_setTrims(int TLL, int TLR, int TFL, int TFR, int TAL, int TAR);
  int Otto.Servo_saveTrims();

  int Otto.home(int noblock = 0);
  int Otto.jump(float steps = 1, int T = 2000, int noblock = FALSE);
  int Otto.walk(float steps = 4, int T = 1000, int dir = FORWARD, int noblock = FALSE);
  int Otto.turn(float steps = 4, int T = 2000, int dir = LEFT, int noblock = FALSE);
  int Otto.bend (int steps = 1, int T = 1400, int dir = LEFT, int noblock = FALSE);
  int Otto.shakeLeg (int steps = 1, int T = 2000, int dir = RIGHT, int noblock = FALSE);
  int Otto.updown(float steps = 1, int T = 1000, int h = , int noblock = FALSE);
  int Otto.swing(float steps = 1, int T = 1000, int h = 20, int noblock = FALSE);
  int Otto.tiptoeSwing(float steps = 1, int T = 900, int h = 20, int noblock = FALSE);
  int Otto.jitter(float steps = 1, int T = 500, int h = 20, int noblock = FALSE);
  int Otto.ascendingTurn(float steps = 1, int T = 900, int h = 20, int noblock = FALSE);
  int Otto.moonwalker(float steps = 1, int T = 900, int h = 20, int dir = LEFT, int noblock = FALSE);
  int Otto.crusaito(float steps = 1, int T = 900, int h = 20, int dir = FORWARD, int noblock = FALSE);
  int Otto.flapping(float steps = 1, int T = 1000, int h = 20, int dir = FORWARD, int noblock = FALSE);

  int Otto.Sound_tone(float noteFrequency, long noteDuration, int silentDuration, int noblock = FALSE);
  int Otto.Sound_bendTones(float initFrequency, float finalFrequency, float prop, long noteDuration, int silentDuration, int noblock = FALSE);
  int Otto.Sound_sing(int songName, int noblock = FALSE);

  int Otto.Gesture(int gesture, int noblock = FALSE);

  int Otto.Mouth_intensity(int intensity);
  int Otto.Mouth_setled(byte X, byte Y, byte value);
  int Otto.Mouth_write(const char * s, byte scrollspeed, int noblock = FALSE);
  int Otto.Mouth(unsigned long int mouth, bool predefined = true);
  int Otto.Mouth_animation(unsigned long int anim, int noblock = FALSE);
  int Otto.Mouth_clear();

  int Otto.Eyes_intensity(int intensity);
  int Otto.Eyes_setled(byte X, byte Y, byte value);
  int Otto.Eyes_write(const char * s, byte scrollspeed, int noblock = FALSE);
  int Otto.Eyes(unsigned long int mouth, bool predefined = true);
  int Otto.Eyes_animation(unsigned long int anim, int noblock = FALSE);
  int Otto.Eyes_clear();
```


## License

The OttoDIYLib is licensed under the terms of the GPL Open Source
license and is available for free.

Welcome to the Otto DIY community

<a href="https://discord.gg/CZZytnw"><img src="https://images.squarespace-cdn.com/content/v1/5cd3e3917046805e4d6700e3/1560193788834-KYURUXVSZAIE4XX1ZB2F/ke17ZwdGBToddI8pDm48kK6MRMHcYvpidTm-7i2qDf_lfiSMXz2YNBs8ylwAJx2qLijIv1YpVq4N1RMuCCrb3iJz4vYg48fcPCuGX417dnbl3kVMtgxDPVlhqW83Mmu6GipAUbdvsXjVksOX7D692AoyyEsAbPHhHcQMU6bWQFI/join_discord_button_small.png" width="25%"></a>

Big thanks to all our contributors
- @JavierIH
- @Obijuan
- @jarsoftelectrical
- @stembotvn
- @sfranzyshen
- @tehniq3
- @logix5
- @DiegoSSJ
- @loreman
- @justinotherguy
- @bhm93
- @wendtbw
- @agomezgar
- @BodoMinea
- @chico
- @PinkDev1
- @MXZZ
- @Pawka
- @per1234
- @FedericoBusero
- @hulkco
- @mishafarms
- @nisha-appanah
- @pabloevaristo
- @ProgrammerBruce
- @Nca78
- @dleval
- @coliss86
- @namepatrik

## How to Contribute:
Contributing to this software is warmly welcomed. There are 3 ways you can contribute to this project:
1. Test and if find a problem then post an issue.
2. Helps us solve the issues or other bugs.
3. Improve the code.
You can do this [basically by forking](https://help.github.com/en/articles/fork-a-repo), committing modifications and then [pulling requests](https://help.github.com/en/articles/about-pull-requests).Please add a change log and your contact into file header.

Thanks for your contribution in advance.

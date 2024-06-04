# OttoDIY Core Code Improvement Project 
<a href="https://creativecommons.org/licenses/by-sa/4.0/"><img src="https://certification.oshwa.org/images/by-sa.png" height="23px"></a>[![OSHW Certification Mark](https://raw.githubusercontent.com/OttoDIY/OttoDIYLib/devel/certification-mark-CZ000001.png)](https://certification.oshwa.org/cz000001.html)[![License: GPL v3](https://img.shields.io/badge/License-GPLv3-blue.svg)](https://www.gnu.org/licenses/gpl-3.0) [![Version](https://img.shields.io/badge/Version-13.1.0-blue)](https://github.com/OttoDIY/OttoDIYLib/tree/devel) [![Built with: Arduino](https://img.shields.io/badge/Built_with-Arduino-blue)](https://www.arduino.cc/reference/en/) [![Built with: FreeRTOS](https://img.shields.io/badge/Built_with-FreeRTOS-green)](https://www.freertos.org/index.html)
### Development Branch :star: Star us on GitHub, it helps!
First join the server <a href="https://discord.gg/CZZytnw"><img src="https://images.squarespace-cdn.com/content/v1/5cd3e3917046805e4d6700e3/1560193788834-KYURUXVSZAIE4XX1ZB2F/ke17ZwdGBToddI8pDm48kK6MRMHcYvpidTm-7i2qDf_lfiSMXz2YNBs8ylwAJx2qLijIv1YpVq4N1RMuCCrb3iJz4vYg48fcPCuGX417dnbl3kVMtgxDPVlhqW83Mmu6GipAUbdvsXjVksOX7D692AoyyEsAbPHhHcQMU6bWQFI/join_discord_button_small.png" width="15%"></a> Then join the discussion <a href="https://discord.com/channels/691410809586450483/1228242677963685918"><img src="https://images.squarespace-cdn.com/content/v1/5cd3e3917046805e4d6700e3/1560193788834-KYURUXVSZAIE4XX1ZB2F/ke17ZwdGBToddI8pDm48kK6MRMHcYvpidTm-7i2qDf_lfiSMXz2YNBs8ylwAJx2qLijIv1YpVq4N1RMuCCrb3iJz4vYg48fcPCuGX417dnbl3kVMtgxDPVlhqW83Mmu6GipAUbdvsXjVksOX7D692AoyyEsAbPHhHcQMU6bWQFI/join_discord_button_small.png" width="15%"></a>
### Warning this code is not ready for use!
This branch of the repository contains the "development" Arduino library for the [OttoDIY robot](https://www.ottodiy.com/), with support for all [OttoDIY models](https://github.com/OttoDIY/OttoDIYLib/tree/devel#supported-models) and [compatible boards](https://github.com/OttoDIY/OttoDIYLib/tree/devel?tab=readme-ov-file#supported-platforms). This branch is currently not functional. If you are looking for the production software please refer to the [main branch](https://github.com/OttoDIY/OttoDIYLib).

#### Versioning for the project will follow a three-numeral format separated by decimal points, for example, 13.1.0
The first numeral represents the major software version number. Versions with the same major number maintain full compatibility with each other. The second numeral indicates the stability number. Odd numbers denote development releases, while even numbers signify stable releases. The last numeral is the minor release number, incrementally increased to distinguish it from the previous minor release. We are currently at 13.1.0 to indicate that there is no release yet ... but that we are in development. 

#### Our initial development release, version 13.1.1, will maintain full backward compatibility with the existing OttoDIY Arduino library 13.0.1, and this commitment will continue with all subsequent 13.x.x releases.

### Project Objective:
To enhance the core code running OttoDIY robot in order to address its current limitations and make it more competitive in the educational tech arena.

### Project Scope:
- [X] **Code Refactoring:**
  Evaluate the existing codebase to identify areas for refactoring. Refactor the code to improve its structure, readability, and maintainability. ([release as version 13.0.1](https://github.com/OttoDIY/OttoDIYLib/tree/version-13.0.1))
- [ ] **Functionality Enhancement:**
  Identify key functionalities that need improvement or addition. Enhance existing functionalities to make them more robust and responsive. Implement new features to address current limitations and meet user needs.
- [ ] **Reliability Improvement:**
  Address any known issues or bugs that affect the reliability of the code. Improve error handling and error reporting mechanisms to enhance the stability of the system.
- [ ] **Documentation Update:**
  Review and update existing documentation to reflect changes made to the codebase. Ensure comprehensive documentation covering installation, usage, and troubleshooting.
- [ ] **Testing and Quality Assurance:**
  Develop and execute test cases to ensure the reliability and functionality of the updated code. Implement automated testing where feasible to streamline the testing process. Conduct thorough quality assurance to identify and address any issues before deployment.
- [X] **Deployment Plan:**
  (* see below) Develop a plan for deploying the updated code to users. Consider methods for migrating existing users to the new version smoothly. Provide support and assistance to users during the transition period.

### **Deployment Plan:**
Once we reach the 13.1.1 milestone, we will begin parallel development, working simultaneously on three versions of the code.

**13.0.x** - This version of the code is the current "main" branch of the OttoDIYLib repository. It includes only changes to support additional platforms. No new features or core changes will be made to this branch, as it is feature-frozen and maintains 101% compatibility.

**13.1.x** - This version maintains full compatibility with version 13.0.x. It is the current "devel" branch of the OttoDIYLib repository, containing changes to support additional platforms and introducing the new non-blocking coding paradigm. This version will include additions and core changes to add hardware options and unify the separate models (biped, wheels, and ninja) into a single library, maintaining 99.9% compatibility.

**14.1.x** - This version of the code is based on the 13.1.x branch of the OttoDIYLib repository. It supports a non-blocking coding paradigm by default and drops support for 13.x.x compatibility, resulting in 0% compatibility with the 13.x.x versions.

Over time, when 13.1.x becomes mature and well-tested, it will be branched to 13.2.1 and replace 13.0.x as the "main" branch. At the same time, 13.1.x will also be branched to 13.3.1 to start a new development cycle. When 14.1.x matures, it will become 14.2.1 and 14.3.1, but these will remain "devel" branches. At some point in the distant future, we may drop support for version 13 altogether, allowing version 14 to take over as the "main" branch.

### Constraints and Assumptions:
* Limited resources (e.g., developers, testers, time).
* Compatibility with existing hardware and software configurations.
* User acceptance of the updated code.
* Adherence to project timeline.

### Supported Models
  - Biped (Classic) & Biped w/ Arms
  - Wheels & Wheels w/ Arms
  - Ninja & Ninja w/ Arms

### Supported Platforms
  - Arduino Nano, Uno, Micro, Mega, Mini, Leonardo, Nano Every [(AVR*)](https://github.com/arduino/ArduinoCore-avr)
  - Arduino Nano Connect, Raspberry Pi Pico [(RP2040)](https://github.com/earlephilhower/arduino-pico) *using earlephilhower's arduino-pico core
  - Arduino Nano Esp32, ESP32 [(ESP32*)](https://github.com/espressif/arduino-esp32)
  - ESP8266 [(ESP8266)](https://github.com/esp8266/Arduino)

### Supported Hardware
  #### Outputs (Mouth, Eyes, and Sound)  
  - Ultrasonic sensor 3, 4 and 5pin with rgb leds (eyes) ws281x (rgb)
  - Led matrix 8x8 (mouth) spi, i2c (mono) and ws281x (rgb)
  - Led matrix 8x16 (eyes) spi, i2c (mono) and ws281x (rgb)
  - Led matrix 6x20 (eyes) ws281x (rgb)
  - Led matrix 6x10 (mouth) ws281x (rgb)
  - Led matrix 10x16 (eyes) ws281x (rgb)
  - Oled display 128x64 (eyes, mouth) spi & i2c (mono)
  - Oled display 128x32 (eyes, mouth) spi & i2c (mono)

  #### Inputs (Distance, Line-follower, Noise, Button, Touch, ...)
  - Ultrasonic sensor 3, 4 and 5pin (gpio, i2c)
  - IR Line-follower Sensor 3 and 4 pin (gpio, analog)
  - microphone 3, 4 and 5 pin (gpio, analog)
  - IR distance sensor 3 pin (analog)
    
### Source File Structure
#### During the development phase, files will be further divided into several smaller files compared to the release file set.

* `Otto_config.h` contains pre-config directives
* `Otto_models.h` contains model pre-config directives
* `Otto_sound.h` contains sound pre-config directives
* `Otto_mouth.h` contains mouth pre-config directives
* `Otto_version.h` contains software version directive
### 
* `Otto.h` and `Otto.cpp` contain all the main user (API) otto functions
### 
* `Otto_gestures.h` and `Otto_gestures.cpp`contain all the otto gestures functions
* `Otto_mouths.h` and `Otto_mouths.cpp` contain all the mouth functions
* `Otto_moves.h` and `Otto_moves.cpp` contain all the otto movement functions
* `Otto_eyes.h` and `Otto_eyes.cpp` contain all the otto eye functions
* `Otto_sounds.h` and ` Otto_sounds.cpp` contain all the otto sound functions
### 
* `Display.h` and `Display.cpp` contain all the oled, led, and ... matrix (output) functions
* `Distance.h` and `Distance.cpp` contain all the distance messurement (input) functions
* `LineSensor.h` and `LineSensor.cpp` contain all the line-sensor messurement (input) functions
* `Servo.h` and `Servo.cpp` contain all the servo (output) functions
* `Sound.h` and `Sound.cpp` contain all the sound (output) functions
* `Oscillator.h` and `Oscillator.cpp` is the main algorithm for the servos "smooth" movement

### New Otto API: 
#### (Biped with arms model, RGB ultrasonic eyes, buzzer sound, and a mono spi matrix mouth)
```cpp
  #include "Otto_config.h"	// first, include preprocessor config directives

  // Otto Config		// next define Otto's config (can be blockly generated)
  #define Otto_code	        BLOCKING
```
The "Otto_code" directive establishes the overall coding style for a sketch. This setting can be overridden at the function call level. By default, it operates in BLOCKING mode. However, when set to NON_BLOCKING, all functions will default to non-blocking behavior.
```cpp
  #define Otto_model	  	BIPED_ARMS	// 6x 180° Servos
  #define Otto_SERVOS	  	6  // total number of servos
  #define Otto_sound	  	SOUND_BUZZER
  #define Otto_mouth	  	LED_MATRIX_8X8_MONO_SPI
  #define Otto_eyes	  	LED_USONIC_RGB_NEO

  // Otto Servos		define Otto's servos (can be blockly generated)
  #define Otto_LL	  	2  // left leg pin
  #define Otto_LR	  	3  // right leg pin
  #define Otto_FL	  	4  // left foot pin
  #define Otto_FR	 	5  // right foot pin
  #define Otto_AL	 	6  // left arm pin
  #define Otto_AR	  	7  // right arm pin

  // Otto Sound			define Otto's sound (can be blockly generated)
  #define Otto_Buzzer     	13 // buzzer pin

  // Otto Eyes		    	define Otto's eyes (can be blockly generated)
  #define Otto_EYES_PIN	  	12 // ultrasonic eyes neopixel pin
  #define Otto_EYES_START 	0  // ultrasonic eyes neopixel start pixel
  #define Otto_EYES_END	  	5  // ultrasonic eyes neopixel end pixel

  // Otto Mouth			define Otto's mouth (can be blockly generated)
  #define Otto_MOUTH_CLK       	A1 // Clock pin
  #define Otto_MOUTH_CS        	A2 // Chip Select pin
  #define Otto_MOUTH_DIN       	A3 // Data In pin
  #define Otto_MOUTH_Rotate    	0  // Rotate Top = 1, Bottom = 2, Left = 3, Right = 4
```
Following the configuration declaration, include the Otto.h API. This API utilizes the compiler preprocessor to selectively include or exclude code segments. If none of the above directives are present in the sketch, the API defaults to the "current" Otto API configuration, which is set to a standard biped for compatibility.
```cpp
  #include <Otto.h>
  Otto Otto;
```
  Now, all functions return an (int) value. This return value can be utilized for error handling. Alternatively, for compatibility,it can be ignored.
```cpp
  int Otto.init(const char * name);   // every Otto should have a unique name. Can be used for Wifi, etc.

// other hardware inits are called by Otto.init() not directly
  int Otto.Servos_init(LL, LR, FL, FR, AL, AR);
  int Otto.Mouth_init(int DIN, int CS, int CLK, int rotate);	
  int Otto.Sound_init(int Buzzer);				
  int Otto.Eyes_init(int EYES_NEO_PIN, int EYES_NEO_START, int EYES_NEO_END);
```
Many functions in the Otto API now include an optional (bool) noblock parameter, which defaults to BLOCKING for compatibility. However, the underlying code has been modified to be non-blocking. This modification allows for greater flexibility and can be extended to user-level code. For younger beginner students, understanding blocking, sequentially executed commands is straightforward. However, for older, more experienced students, blocking code can be seen as a limitation. This limitation may exclude the current Otto from being used in the classroom.
```cpp
  int Otto.home(bool noblock = BLOCKING);
  int Otto.jump(float steps = 1, int T = 2000, bool noblock = BLOCKING);
  int Otto.walk(float steps = 4, int T = 1000, int dir = FORWARD, bool noblock = BLOCKING);
  int Otto.turn(float steps = 4, int T = 2000, int dir = LEFT, bool noblock = BLOCKING);
  int Otto.bend (int steps = 1, int T = 1400, int dir = LEFT, bool noblock = BLOCKING);
  int Otto.shakeLeg (int steps = 1, int T = 2000, int dir = RIGHT, bool noblock = BLOCKING);
  int Otto.updown(float steps = 1, int T = 1000, int h = , bool noblock = BLOCKING);
  int Otto.swing(float steps = 1, int T = 1000, int h = 20, bool noblock = BLOCKING);
  int Otto.tiptoeSwing(float steps = 1, int T = 900, int h = 20, bool noblock = BLOCKING);
  int Otto.jitter(float steps = 1, int T = 500, int h = 20, bool noblock = BLOCKING);
  int Otto.ascendingTurn(float steps = 1, int T = 900, int h = 20, bool noblock = BLOCKING);
  int Otto.moonwalker(float steps = 1, int T = 900, int h = 20, int dir = LEFT, bool noblock = BLOCKING);
  int Otto.crusaito(float steps = 1, int T = 900, int h = 20, int dir = FORWARD, bool noblock = BLOCKING);
  int Otto.flapping(float steps = 1, int T = 1000, int h = 20, int dir = FORWARD, bool noblock = BLOCKING);

  int Otto.Sound_tone(float noteFreq, long noteDur, int silentDur, bool noblock = BLOCKING);
  int Otto.Sound_bendTones(float initFreq, float finalFreq, float prop, long noteDur, int silentDur, bool noblock = BLOCKING);
  int Otto.Sound_sing(int songName, bool noblock = BLOCKING);
  int Otto.Sound_queueSize();
  bool Otto.Sound_isEmptyQueue();
  int Otto.Sound_clearQueue();

  int Otto.Gesture(int gesture, bool noblock = BLOCKING);

  int Otto.Mouth_intensity(int intensity);
  int Otto.Mouth_setled(byte X, byte Y, byte value);
  int Otto.Mouth_write(const char * s, byte scrollspeed, bool noblock = BLOCKING);
  int Otto.Mouth(unsigned long int mouth, bool predefined = true);
  int Otto.Mouth_animation(unsigned long int anim, int index);
  int Otto.Mouth_animate(unsigned long int anim, byte speed, bool loop = false, bool bounce = false, bool noblock = BLOCKING);
  int Otto.Mouth_clear();
  int Otto.Mouth_queueSize();
  bool Otto.Mouth_isEmptyQueue();
  int Otto.Mouth_clearQueue();

  int Otto.Eyes_intensity(int intensity);
  int Otto.Eyes_setled(byte X, byte Y, byte value);
  int Otto.Eyes_write(const char * s, byte scrollspeed, bool noblock = BLOCKING);
  int Otto.Eyes(unsigned long int eyes, bool predefined = true);
  int Otto.Eyes_animation(unsigned long int anim, int index);
  int Otto.Eyes_animate(unsigned long int anim, byte speed, bool loop = false, bool bounce = false, bool noblock = BLOCKING);
  int Otto.Eyes_clear();
  int Otto.Eyes_queueSize();
  bool Otto.Eyes_isEmptyQueue();
  int Otto.Eyes_clearQueue();


  int Otto.Servos_moveServos(int time, int servo_target[], bool noblock = BLOCKING);
  int Otto.Servos_moveSingle(int position, int servo_number, bool noblock = BLOCKING);
  int Otto.Servos_oscillateServos(int A[6], int O[6], int T, double phase_diff[6], float cycle, bool noblock = BLOCKING);
  bool Otto.Servos_getRestState();
  int Otto.Servos_setRestState(bool state);
  int Otto.Servos_attachServos();
  int Otto.Servos_detachServos();
  int Otto.Servos_enableServoLimit(int speed_limit_degree_per_sec = SERVO_LIMIT_DEFAULT);
  int Otto.Servos_disableServoLimit();
  int Otto.Servos_setTrims(int TLL, int TLR, int TFL, int TFR, int TAL, int TAR);
  int Otto.Servos_saveTrims();
```
The following functions are provided for compatibility with the "current" Otto library. These functions serve as aliases or wrappers to the new API. We will maintain support for the original Otto library with all 13.x.x versions, but support will be dropped with version 14.x.x and above.
```cpp
  void Otto.init(int YL, int YR, int RL, int RR, bool load_calibration, int Buzzer); 
  void Otto.setTrims(int TLL, int TLR, int TFL, int TFR);			
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
  void Otto.initMATRIX(int DIN, int CS, int CLK, int rotate);
  void Otto.matrixIntensity(int intensity);
  void Otto.setLed(byte X, byte Y, byte value);
  void Otto.writeText (const char * s, byte scrollspeed);
  void Otto.putMouth(unsigned long int mouth, bool predefined = true);
  void Otto.putAnimationMouth(unsigned long int anim, int index);
  void Otto.clearMouth();
  void Otto._tone(float noteFrequency, long noteDuration, int silentDuration);
  void Otto.bendTones(float initFrequency, float finalFrequency, float prop, long noteDuration, int silentDuration);
  void Otto.sing(int songName);
  void Otto.playGesture(int gesture);

```
### New Otto Background API: 
#### Built with FreeRTOS (full details coming soon ... in the meantime ... read the code)
<img src="https://predictabledesigns.com/wp-content/uploads/2020/06/word-image-19.jpeg)" width="10%">

### Development Tool-Chain:
#### 3D Models [Tinkercad](https://www.tinkercad.com/users/9Wbn1uShxa2?size=small)
#### Source Code [Github](https://github.com/OttoDIY/OttoDIYLib/tree/devel)
#### Simulation [Wokwi](https://wokwi.com/makers/sfranzyshen)
#### Disscusion [Discord](https://discord.com/channels/691410809586450483/1228242677963685918)


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

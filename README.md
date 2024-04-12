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
  ### Inputs (Distance, Line-follower, Noise, Button, and Touch)
  - Ultrasonic sensor 3, 4 and 5pin (gpio, i2c)
  - IR Line-follower Sensor 3 and 4 pin (gpio, analog)
  - microphone 3, 4 and 5 pin (gpio, analog)
  - IR distance sensor 3 pin (analog)
    
## Structure
#### In the development phase, files will be further divided into more files compared to the release file set.

* `Otto.h` and `Otto.cpp` contains all the main otto functions
* `Otto_gestures.h` contains all the otto gestures functions
* `Otto_mouths.h` contains all the mouth functions
* `Otto_moves.h` contains all the otto movement functions
* `Otto_eyes.h` contains all the otto eye functions
* `Otto_sounds.h` contains all the otto sound functions
* `Display.h` and `Display.cpp` contains all the oled and led matrix functions
* `Distance.h` and `Distance.cpp` contains all the distance messurement functions
* `Oscillator.h` and `Oscillator.cpp` is the main algorithm for the servos "smooth" movement
* `Sound.h` and `Sound.cpp` contains all the code for making sounds
* `SerialCommand.h` and `SerialCommand.cpp` is for Bluetooth communication vis Software serial or native Bluetooth

### Adding library

```cpp
#include <Otto.h>
Otto Otto;
```

### Pins declaration

These are the default signal connections for the servos and buzzer for AVR Arduino boards in the examples, you can alternatively connect them in different pins if you also change the pin number.

```cpp
#define LeftLeg 2 // left leg pin
#define RightLeg 3 // right leg pin
#define LeftFoot 4 // left foot pin
#define RightFoot 5 // right foot pin
#define Buzzer 13 //buzzer pin
```

### Initialization

When starting the program, the 'init' function must be called with the use of servo motor calibration as a parameter. <br/>
It is best to place the servo motors in their home position after initialization with 'home' function.

```cpp
void setup() {
   Otto.init(LeftLeg, RightLeg, LeftFoot, RightFoot, true, Buzzer);
   Otto.home();
}
```
The `home()` function makes the servos move to the center position, Otto standing in the neutral position.

## Predetermined Functions:
Many preconfigured movements are available in the library:

### Movements:
These are actions that involve the use of the 4 servo motors with the oscillation library combined in synergy and with smooth movements. You can change the values inside the pratensis `()` to alter the speed, direction, and size of the movements.

#### Walk function

```cpp
Otto.walk(steps, time, dir);
```
- `steps` are just how many times you want to repeat that movement without the need of further coding or adding additional rows.
- `time` (noted as `T` below) translated in milliseconds is the duration of the movement. For a higher time value is slower the movement, try values between 500 to 3000.
- `dir` is the direction: `1` for forward or `-1` backward

Example:
```cpp
Otto.walk(2, 1000, 1);
```
In this example `2` is the number of steps, `1000` is "TIME" in milliseconds and it will walk forward.

For example changing T value: Slow=2000 Normal=1000 Fast= 500

```cpp
Otto.turn(steps, T, dir);
```
(# of steps, T, to the left or -1 to the right)

```cpp
Otto.bend (steps, T, dir);
```

(# of steps, T, 1 bends to the left or -1 to the right)

```cpp
Otto.shakeLeg (steps, T, dir);
```
(# of steps, T, 1 bends to the left or -1 to the right)

```cpp
Otto.jump(steps, T);
```
(# of steps up, T) this one does not have a dir parameter
Otto doesn't really jump ;P

### Dances:

Similar to movements but more fun! you can adjust a new parameter `h` "height or size of the movements" to make the dance more interesting.

```cpp
Otto.moonwalker(steps, T, h, dir);
```
(# of steps, T, h, 1 to the left or -1 to the right)

`h`: you can try change between 15 and 40


Example:
```cpp
Otto.moonwalker(3, 1000, 25,1);
```

```cpp
Otto.crusaito(steps, T, h, dir);
```
(# of steps, T, h, 1 to the left or -1 to the right)

`h`: you can try change between 20 to 50

```cpp
Otto.flapping(steps, T, h, dir);
```
(# of steps, T, h, 1 to the front or -1 to the back)

`h`: you can try change between 10 to 30

```cpp
Otto.swing(steps, T, h);
```
`h`: you can try change between 0 to 50

```cpp
Otto.tiptoeSwing(steps, T, h);
```
`h`: you can try change between 0 to 50

```cpp
Otto.jitter(steps, T, h);
```
`h`: you can try change between 5 to 25

```cpp
Otto.updown(steps, T, h);  
```
`h`: you can try change between 0 to 90

```cpp
Otto.ascendingTurn(steps, T, h);
```
`h`: you can try change between 5 to 15

### Sounds:

```
Otto.sing(songName);
```
By just changing what is inside the () we can change the sounds easily to 19 different ones.
Simple as copying and pasting in a new row to make the sounds as many times as you like.

- S_connection
- S_disconnection
- S_buttonPushed
- S_mode1
- S_mode2
- S_mode3
- S_surprise
- S_OhOoh
- S_OhOoh2
- S_cuddly
- S_sleeping
- S_happy
- S_superHappy
- S_happy_short
- S_sad
- S_confused
- S_fart1
- S_fart2
- S_fart3

Otto can emit several sounds with the 'sing' function:
```cpp
Otto._tone(10, 3, 1);
```

(noteFrequency, noteDuration, silentDuration)

```cpp
Otto.bendTones (100, 200, 1.04, 10, 10);
```
(initFrequency, finalFrequency, prop, noteDuration, silentDuration)


### Gestures:
Finally, our favorite, This is a combination of the 2 previous functions we learnt sing + walk
Their goal is to express emotions by combining sounds with movements at the same time and if you have the LED matrix you can show them in the robot mouth!

```cpp
Otto.playGesture(gesture);
```
- `Otto.playGesture(OttoHappy);`
- `Otto.playGesture(OttoSuperHappy);`
- `Otto.playGesture(OttoSad);`
- `Otto.playGesture(OttoVictory);`
- `Otto.playGesture(OttoAngry);`
- `Otto.playGesture(OttoSleeping);`
- `Otto.playGesture(OttoFretful);`
- `Otto.playGesture(OttoLove);`
- `Otto.playGesture(OttoConfused);`
- `Otto.playGesture(OttoFart);`
- `Otto.playGesture(OttoWave);`
- `Otto.playGesture(OttoMagic);`
- `Otto.playGesture(OttoFail);`

As you see it’s very simple, but what it does is quite advanced.

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

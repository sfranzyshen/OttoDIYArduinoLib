// OttoDIY Arduino Library project 2024

// Zowi (c) BQ. Released under a GPL licencse 04 December 2015

#include <Arduino.h>
#include "Otto.h"
#include "Oscillator.h"

//---------------------------------------------------------
//-- Otto Init: Initialize
//---------------------------------------------------------
// Description:
//    This function initializes Otto by setting up servo pins,
//    attaching servos, loading calibration data if specified,
//    and configuring the buzzer pin.
// Parameters:
//    YL: Pin for left hip servo
//    YR: Pin for right hip servo
//    RL: Pin for left foot servo
//    RR: Pin for right foot servo
//    load_calibration: Whether to load calibration data from EEPROM
//    Buzzer: Pin for the buzzer
//---------------------------------------------------------
void Otto::init(int YL, int YR, int RL, int RR, bool load_calibration, int Buzzer) {
    // Set servo pins
    servo_pins[0] = YL;
    servo_pins[1] = YR;
    servo_pins[2] = RL;
    servo_pins[3] = RR;

    // Attach servos
    attachServos();

    // Set rest state to false
    isOttoResting = false;

    // Load calibration data if specified
    if (load_calibration) {
        for (int i = 0; i < 4; i++) {
            int servo_trim = EEPROM.read(i);
            if (servo_trim > 128) servo_trim -= 256;
            servo[i].SetTrim(servo_trim);
        }
    }

    // Set buzzer pin
    pinBuzzer = Buzzer;
    pinMode(Buzzer, OUTPUT);
}

//---------------------------------------------------------
//-- Otto Display: Initialize Matrix
//---------------------------------------------------------
// Description:
//    This function initializes the LED matrix display.
// Parameters:
//    DIN: Data input pin for the LED matrix
//    CS: Chip select pin for the LED matrix
//    CLK: Clock pin for the LED matrix
//    rotate: Rotation value for the LED matrix (0, 1, 2, or 3)
//---------------------------------------------------------
void Otto::initMATRIX(int DIN, int CS, int CLK, int rotate) {
    // Initialize LED matrix display
    ledmatrix.init(DIN, CS, CLK, 1, rotate);
}

//---------------------------------------------------------
//-- Otto Display: Set Matrix Intensity
//---------------------------------------------------------
// Description:
//    This function sets the intensity of the LED matrix display.
// Parameters:
//    intensity: Intensity level (0 to 15) for the LED matrix
//---------------------------------------------------------
void Otto::matrixIntensity(int intensity) {
    // Set intensity of LED matrix display
    ledmatrix.setIntensity(intensity);
}

//---------------------------------------------------------
//-- Otto Servo: Attach Servos
//---------------------------------------------------------
// Description:
//    This function attaches the servos to their respective pins.
//---------------------------------------------------------
void Otto::attachServos() {
    // Attach each servo to its pin
    servo[0].attach(servo_pins[0]);
    servo[1].attach(servo_pins[1]);
    servo[2].attach(servo_pins[2]);
    servo[3].attach(servo_pins[3]);
}

//---------------------------------------------------------
//-- Otto Servo: Detach Servos
//---------------------------------------------------------
// Description:
//    This function detaches the servos.
//---------------------------------------------------------
void Otto::detachServos() {
    // Detach each servo
    servo[0].detach();
    servo[1].detach();
    servo[2].detach();
    servo[3].detach();
}

//---------------------------------------------------------
//-- Otto Servo: Set Servo Trims
//---------------------------------------------------------
// Description:
//    This function sets the trim values for each servo.
// Parameters:
//    YL: Trim value for the left hip servo
//    YR: Trim value for the right hip servo
//    RL: Trim value for the left foot servo
//    RR: Trim value for the right foot servo
//---------------------------------------------------------
void Otto::setTrims(int YL, int YR, int RL, int RR) {
    // Set trim value for each servo
    servo[0].SetTrim(YL);
    servo[1].SetTrim(YR);
    servo[2].SetTrim(RL);
    servo[3].SetTrim(RR);
}

//---------------------------------------------------------
//-- Otto Servo: Save Trims on EEPROM
//---------------------------------------------------------
// Description:
//    This function saves the trim values for each servo
//    on the EEPROM.
//---------------------------------------------------------
void Otto::saveTrimsOnEEPROM() {
    // Iterate through each servo and save its trim value
    for (int i = 0; i < 4; i++) {
        EEPROM.write(i, servo[i].getTrim());
    }
}

//---------------------------------------------------------
//-- Otto Movement: Move Servos
//---------------------------------------------------------
// Description:
//    This function moves the servos to the specified target positions over a given time.
// Parameters:
//    time: Total time for the movement to complete (in milliseconds)
//    servo_target: Array containing the target positions for each servo (0 to 180 degrees)
//---------------------------------------------------------
void Otto::_moveServos(int time, int servo_target[]) {
    // Attach servos if not already attached
    attachServos();

    // Set rest state to false if it's true
    if (getRestState()) {
        setRestState(false);
    }

    // Calculate final time
    unsigned long final_time = millis() + time;

    // Check if the movement time is greater than 10 milliseconds
    if (time > 10) {
        // Calculate increment for each servo
        float increment[4];
        for (int i = 0; i < 4; i++) {
            increment[i] = static_cast<float>(servo_target[i] - servo[i].getPosition()) / (time / 10.0);
        }

        // Move servos gradually
        while (millis() < final_time) {
            unsigned long partial_time = millis() + 10;
            for (int i = 0; i < 4; i++) {
                servo[i].SetPosition(servo[i].getPosition() + increment[i]);
            }
            while (millis() < partial_time); // Pause
        }
    } else {
        // Move servos directly to target position
        for (int i = 0; i < 4; i++) {
            servo[i].SetPosition(servo_target[i]);
        }
        while (millis() < final_time); // Pause
    }

    // Final adjustment to the target
    // If servo speed limiter is turned on, reaching the goal may take longer than requested time
    bool servoPositionsMatch = true;
    while (servoPositionsMatch) {
        servoPositionsMatch = false;
        for (int i = 0; i < 4; i++) {
            if (servo_target[i] != servo[i].getPosition()) {
                servoPositionsMatch = true;
                break;
            }
        }
        if (servoPositionsMatch) {
            // Move servos to target positions
            for (int i = 0; i < 4; i++) {
                servo[i].SetPosition(servo_target[i]);
            }
            unsigned long partial_time = millis() + 10;
            while (millis() < partial_time); // Pause
        }
    }
}

//---------------------------------------------------------
//-- Otto Movement: Move Single Servo
//---------------------------------------------------------
// Description:
//    This function moves a single servo to the specified position.
// Parameters:
//    position: Desired position for the servo (0 to 180 degrees)
//    servo_number: Index of the servo to be moved (0 to 3)
//---------------------------------------------------------
void Otto::_moveSingle(int position, int servo_number) {
    // Ensure position is within valid range
    if (position > 180 || position < 0) {
        position = 90; // Set to default position (90 degrees) if out of range
    }

    // Attach servos if not already attached
    attachServos();

    // Set rest state to false if it's true
    if (getRestState()) {
        setRestState(false);
    }

    // Set position for the specified servo
    if (servo_number >= 0 && servo_number < 4) {
        servo[servo_number].SetPosition(position);
    }
}

//---------------------------------------------------------
//-- Otto Movement: Oscillate Servos
//---------------------------------------------------------
// Description:
//    This function oscillates Otto's servos based on the specified parameters.
// Parameters:
//    A: Array of servo amplitudes
//    O: Array of servo offsets
//    T: Period of oscillation (in milliseconds)
//    phase_diff: Array of phase differences for each servo
//    cycle: Number of complete cycles to execute (default is 1)
//---------------------------------------------------------
void Otto::oscillateServos(int A[4], int O[4], int T, double phase_diff[4], float cycle = 1.0) {
    // Set servo parameters
    for (int i = 0; i < 4; i++) {
        servo[i].SetO(O[i]);
        servo[i].SetA(A[i]);
        servo[i].SetT(T);
        servo[i].SetPh(phase_diff[i]);
    }

    // Calculate reference time
    double ref = millis();

    // Run servo oscillation loop
    double endTime = T * cycle + ref;
    for (double currentTime = ref; currentTime <= endTime; currentTime = millis()) {
        for (int i = 0; i < 4; i++) {
            servo[i].refresh();
        }
    }
}

//---------------------------------------------------------
//-- Otto Movement: Execute Motion
//---------------------------------------------------------
// Description:
//    This function executes a motion by oscillating the servos according to the provided parameters.
// Parameters:
//    A: Array of amplitudes for each servo.
//    O: Array of offsets for each servo.
//    T: Period of oscillation.
//    phase_diff: Array of phase differences for each servo.
//    steps: Number of complete cycles to execute (default value is 1.0).
//---------------------------------------------------------
void Otto::_execute(int A[4], int O[4], int T, double phase_diff[4], float steps = 1.0) {
    // Attach servos before executing motion
    attachServos();
    
    // Check and unset rest state if set
    if (getRestState())
        setRestState(false);

    // Calculate the number of complete cycles to execute
    int completeCycles = static_cast<int>(steps);

    // Execute complete cycles
    for (int i = 0; i < completeCycles; i++)
        oscillateServos(A, O, T, phase_diff);

    // Execute the final incomplete cycle if necessary
    float remainingSteps = steps - completeCycles;
    if (remainingSteps > 0)
        oscillateServos(A, O, T, phase_diff, remainingSteps);
}

//---------------------------------------------------------
//-- Otto Movement: Get Rest State
//---------------------------------------------------------
// Description:
//    This function retrieves the current rest state of Otto.
// Returns:
//    Boolean value representing Otto's rest state.
//---------------------------------------------------------
bool Otto::getRestState() {
    return isOttoResting;
}

//---------------------------------------------------------
//-- Otto Movement: Set Rest State
//---------------------------------------------------------
// Description:
//    This function sets the rest state of Otto.
// Parameters:
//    state: Boolean value indicating the desired rest state.
//---------------------------------------------------------
void Otto::setRestState(bool state) {
    isOttoResting = state;
}

//---------------------------------------------------------
//-- Otto Movement: Home
//---------------------------------------------------------
// Description:
// - Moves Otto to the rest position
//---------------------------------------------------------
void Otto::home() {
    // Check if Otto is already at rest
    if (isOttoResting == false) {
        // Define servo positions for the rest position
        int homes[4] = {90, 90, 90, 90};
        // Move the servos to the rest position over 500 milliseconds
        _moveServos(500, homes);
        // Detach servos to save power
        detachServos();
        // Update flag to indicate Otto is at rest
        isOttoResting = true;
    }
}

//---------------------------------------------------------
//-- Otto Movement: Jump
//---------------------------------------------------------
// Parameters:
//    * steps: Number of steps
//    * T: Period
//---------------------------------------------------------
void Otto::jump(float steps, int T) {
    // Define servo positions for the upward motion
    int up[] = {90, 90, 150, 30};
    // Move servos to the upward position over time T
    _moveServos(T, up);
    
    // Define servo positions for the downward motion
    int down[] = {90, 90, 90, 90};
    // Move servos to the downward position over time T
    _moveServos(T, down);
}

//---------------------------------------------------------
//-- Otto Movement: Walking (Forward or Backward)
//---------------------------------------------------------
// Parameters:
//   * steps: Number of steps
//   * T: Period
//   * dir: Direction: FORWARD / BACKWARD
//---------------------------------------------------------
void Otto::walk(float steps, int T, int dir) {
    // Oscillator parameters for walking
    // Hip servos are in phase
    // Feet servos are in phase
    // Hip and feet are 90 degrees out of phase
    // -90: Walk forward
    //  90: Walk backward
    // Feet servos also have the same offset (for tiptoe a little bit)
    int A[4] = {30, 30, 20, 20};
    int O[4] = {0, 0, 4, -4};
    double phase_diff[4] = {0, 0, DEG2RAD(dir * -90), DEG2RAD(dir * -90)};

    // Execute the walking motion
    _execute(A, O, T, phase_diff, steps);
}

//---------------------------------------------------------
//-- Otto Movement: Turning (Left or Right)
//---------------------------------------------------------
// Parameters:
//   * Steps: Number of steps
//   * T: Period
//   * Dir: Direction: LEFT / RIGHT
//---------------------------------------------------------
void Otto::turn(float steps, int T, int dir) {
    // Same coordination as for walking (see Otto::walk)
    // The amplitudes of the hip's oscillators are not equal
    // When the right hip servo amplitude is higher, the steps taken by
    // the right leg are bigger than the left. So, the robot describes a left arc
    int A[4] = {30, 30, 20, 20};  // Default amplitudes for both hip servos
    int O[4] = {0, 0, 4, -4};      // Phase offsets for hip and feet oscillators
    double phase_diff[4] = {0, 0, DEG2RAD(-90), DEG2RAD(-90)};  // Phase differences for hip and feet

    // Adjust amplitudes for left or right turn
    if (dir == LEFT) {
        A[0] = 30;  // Left hip servo amplitude
        A[1] = 10;  // Right hip servo amplitude
    } else {
        A[0] = 10;  // Right hip servo amplitude
        A[1] = 30;  // Left hip servo amplitude
    }

    // Execute the turning motion
    _execute(A, O, T, phase_diff, steps);
}

//---------------------------------------------------------
//-- Otto Movement: Lateral Bend
//---------------------------------------------------------
// Parameters:
//    * Steps: Number of bends
//    * T: Period of one bend
//    * Dir: RIGHT=Right bend, LEFT=Left bend
//---------------------------------------------------------
void Otto::bend(int steps, int T, int dir) {
    // Parameters of all the movements. Default: Left bend
    int bend1[4] = {90, 90, 62, 35};    // Initial bend position
    int bend2[4] = {90, 90, 62, 105};   // Second bend position
    int homes[4] = {90, 90, 90, 90};    // Default home position

    // Adjust parameters for right direction if chosen
    if (dir == RIGHT) {
        bend1[2] = 180 - 35;
        bend1[3] = 180 - 60;  // Not 65. Otto is unbalanced
        bend2[2] = 180 - 105;
        bend2[3] = 180 - 60;
    }

    // Time of the bend movement. Fixed parameter to avoid falls
    int T2 = 800;

    // Bend movement
    for (int i = 0; i < steps; i++) {
        _moveServos(T2 / 2, bend1);  // Move to first bend position
        _moveServos(T2 / 2, bend2);  // Move to second bend position
        delay(T * 0.8);              // Delay between bends
        _moveServos(500, homes);     // Return to home position
    }
}

//---------------------------------------------------------
//-- Otto Movement: Shake a Leg
//---------------------------------------------------------
// Parameters:
//    * Steps: Number of shakes
//    * T: Period of one shake
//    * Dir: RIGHT=Right leg, LEFT=Left leg
//---------------------------------------------------------
void Otto::shakeLeg(int steps, int T, int dir) {
    // This variable changes the amount of shakes
    int numberLegMoves = 2;

    // Parameters of all the movements. Default: Right leg
    int shake_leg1[4] = {90, 90, 58, 35};    // Initial shake leg position
    int shake_leg2[4] = {90, 90, 58, 120};   // Second shake leg position
    int shake_leg3[4] = {90, 90, 58, 60};    // Third shake leg position
    int homes[4] = {90, 90, 90, 90};         // Default home position

    // Changes in the parameters if left leg is chosen
    if (dir == LEFT) {
        shake_leg1[2] = 180 - 35;
        shake_leg1[3] = 180 - 58;
        shake_leg2[2] = 180 - 120;
        shake_leg2[3] = 180 - 58;
        shake_leg3[2] = 180 - 60;
        shake_leg3[3] = 180 - 58;
    }

    // Time of the bend movement. Fixed parameter to avoid falls
    int T2 = 1000;
    // Time of one shake, constrained to avoid movements too fast.
    T = T - T2;
    T = max(T, 200 * numberLegMoves);

    for (int j = 0; j < steps; j++) {
        // Bend movement
        _moveServos(T2 / 2, shake_leg1);
        _moveServos(T2 / 2, shake_leg2);

        // Shake movement
        for (int i = 0; i < numberLegMoves; i++) {
            _moveServos(T / (2 * numberLegMoves), shake_leg3);
            _moveServos(T / (2 * numberLegMoves), shake_leg2);
        }
        _moveServos(500, homes); // Return to home position
    }

    delay(T);
}

//---------------------------------------------------------
//-- Otto Movement: Up & Down
//---------------------------------------------------------
// Parameters:
//    * Steps: Number of jumps
//    * T: Period
//    * h: Jump height (SMALL / MEDIUM / BIG or a number in degrees 0 - 90)
//---------------------------------------------------------
void Otto::updown(float steps, int T, int h) {
    //-- Both feet are 180 degrees out of phase
    //-- Feet amplitude and offset are the same
    //-- Initial phase for the right foot is -90, so that it starts
    //-- in one extreme position (not in the middle)
    int A[4] = {0, 0, h, h};                         // Amplitude for servos
    int O[4] = {0, 0, h, -h};                        // Offset for servos
    double phase_diff[4] = {0, 0, DEG2RAD(-90), DEG2RAD(90)};  // Phase difference for servos

    //-- Let's oscillate the servos!
    _execute(A, O, T, phase_diff, steps);
}

//---------------------------------------------------------
//-- Otto Movement: Swinging Side to Side
//---------------------------------------------------------
// Parameters:
//    * Steps: Number of steps
//    * T: Period
//    * h: Amount of swing (from 0 to 50 approx)
//---------------------------------------------------------
void Otto::swing(float steps, int T, int h) {
    //-- Both feet are in phase. The offset is half the amplitude
    //-- It causes the robot to swing from side to side
    int A[4] = {0, 0, h, h};                         // Amplitude for servos
    int O[4] = {0, 0, h / 2, -h / 2};                // Offset for servos
    double phase_diff[4] = {0, 0, DEG2RAD(0), DEG2RAD(0)};  // Phase difference for servos

    //-- Let's oscillate the servos!
    _execute(A, O, T, phase_diff, steps);
}

//---------------------------------------------------------
//-- Otto Movement: Swinging Side to Side Without Touching the Floor with the Heel
//---------------------------------------------------------
// Parameters:
//    * Steps: Number of steps
//    * T: Period
//    * h: Amount of swing (from 0 to 50 approx)
//---------------------------------------------------------
void Otto::tiptoeSwing(float steps, int T, int h) {
    //-- Both feet are in phase. The offset is not half the amplitude in order to tiptoe
    //-- It causes the robot to swing from side to side
    int A[4] = {0, 0, h, h};                  // Amplitude for servos
    int O[4] = {0, 0, h, -h};                 // Offset for servos
    double phase_diff[4] = {0, 0, 0, 0};      // Phase difference for servos

    //-- Let's oscillate the servos!
    _execute(A, O, T, phase_diff, steps);
}

//---------------------------------------------------------
//-- Otto Movement: Jitter
//---------------------------------------------------------
// Parameters:
//    * Steps: Number of jitters
//    * T: Period of one jitter
//    * h: Height (Values between 5 - 25)
//---------------------------------------------------------
void Otto::jitter(float steps, int T, int h) {
    //-- Both feet are 180 degrees out of phase
    //-- Feet amplitude and offset are the same
    //-- Initial phase for the right foot is -90, so that it starts
    //--   in one extreme position (not in the middle)
    //-- h is constrained to avoid hitting the feet
    h = min(25, h);
    int A[4] = {h, h, 0, 0};                // Amplitude for servos
    int O[4] = {0, 0, 0, 0};                // Offset for servos
    double phase_diff[4] = {DEG2RAD(-90), DEG2RAD(90), 0, 0};   // Phase difference for servos

    //-- Let's oscillate the servos!
    _execute(A, O, T, phase_diff, steps);
}

//---------------------------------------------------------
//-- Otto Movement: Ascending & turn (Jitter while up&down)
//---------------------------------------------------------
// Parameters:
//    * Steps: Number of bends
//    * T: Period of one bend
//    * h: Height (Values between 5 - 15)
//---------------------------------------------------------
void Otto::ascendingTurn(float steps, int T, int h) {
    //-- Both feet and legs are 180 degrees out of phase
    //-- Initial phase for the right foot is -90, so that it starts
    //--   in one extreme position (not in the middle)
    //-- h is constrained to avoid hitting the feet
    h = min(13, h);
    int A[4] = {h, h, h, h};                // Amplitude for servos
    int O[4] = {0, 0, h + 4, -h + 4};       // Offset for servos
    double phase_diff[4] = {DEG2RAD(-90), DEG2RAD(90), DEG2RAD(-90), DEG2RAD(90)};   // Phase difference for servos

    //-- Let's oscillate the servos!
    _execute(A, O, T, phase_diff, steps);
}

//---------------------------------------------------------
//-- Otto Movement: Moonwalker. Otto moves like Michael Jackson
//---------------------------------------------------------
// Parameters:
//    * Steps: Number of steps
//    * T: Period
//    * h: Height. Typical values between 15 and 40
//    * dir: Direction: LEFT / RIGHT
//---------------------------------------------------------
void Otto::moonwalker(float steps, int T, int h, int dir) {
    //-- This motion is similar to that of the caterpillar robots: A traveling
    //-- wave moving from one side to another
    //-- The two Otto's feet are equivalent to a minimal configuration. It is known
    //-- that 2 servos can move like a worm if they are 120 degrees out of phase
    //-- In the example of Otto, the two feet are mirrored so that we have:
    //--    180 - 120 = 60 degrees. The actual phase difference given to the oscillators
    //--  is 60 degrees.
    //--  Both amplitudes are equal. The offset is half the amplitude plus a little bit of
    //-   offset so that the robot tiptoes lightly

    int A[4] = {0, 0, h, h};                             // Amplitude for servos
    int O[4] = {0, 0, h / 2 + 2, -h / 2 - 2};            // Offset for servos
    int phi = -dir * 90;                                 // Phase difference for servos
    double phase_diff[4] = {0, 0, DEG2RAD(phi), DEG2RAD(-60 * dir + phi)};

    //-- Let's oscillate the servos!
    _execute(A, O, T, phase_diff, steps);
}

//-----------------------------------------------------------
//-- Otto Movement: Crusaito. A mixture between moonwalker and walk
//-----------------------------------------------------------
// Parameters:
//   * steps: Number of steps
//   * T: Period
//   * h: height (Values between 20 - 50)
//   * dir: Direction: LEFT / RIGHT
//-----------------------------------------------------------
void Otto::crusaito(float steps, int T, int h, int dir) {

  int A[4] = {25, 25, h, h};                            // Amplitude for servos
  int O[4] = {0, 0, h / 2 + 4, -h / 2 - 4};             // Offset for servos
  double phase_diff[4] = {90, 90, DEG2RAD(0), DEG2RAD(-60 * dir)}; // Phase difference for servos

  //-- Let's oscillate the servos!
  _execute(A, O, T, phase_diff, steps);
}

//---------------------------------------------------------
//-- Otto Movement: Flapping
//---------------------------------------------------------
// Parameters:
//   * steps: Number of steps
//   * T: Period
//   * h: Height (Values between 10 - 30)
//   * dir: Direction: FORWARD, BACKWARD
//---------------------------------------------------------
void Otto::flapping(float steps, int T, int h, int dir) {

  int A[4] = {12, 12, h, h};                                   // Amplitude for servos
  int O[4] = {0, 0, h - 10, -h + 10};                          // Offset for servos
  double phase_diff[4] = {DEG2RAD(0), DEG2RAD(180), DEG2RAD(-90 * dir), DEG2RAD(90 * dir)}; // Phase difference for servos

  //-- Let's oscillate the servos!
  _execute(A, O, T, phase_diff, steps);
}

//---------------------------------------------------------
//-- Otto Display: Set LED
//---------------------------------------------------------
// Description:
//    This function sets the state of a specific LED on the LED matrix.
// Parameters:
//    X: The x-coordinate of the LED.
//    Y: The y-coordinate of the LED.
//    value: The value to set for the LED (0 for OFF, 1 for ON).
//---------------------------------------------------------
void Otto::setLed(byte X, byte Y, byte value) {
    // Set the state of the LED at the specified coordinates
    ledmatrix.setDot(X, Y, value);
}

//---------------------------------------------------------
//-- Otto Display: Put Animation Mouth
//---------------------------------------------------------
// Description:
//    This function displays a specific frame of a mouth animation on the LED matrix.
// Parameters:
//    aniMouth: The index of the animation sequence.
//    index: The index of the frame within the animation sequence.
//---------------------------------------------------------
void Otto::putAnimationMouth(unsigned long int aniMouth, int index) {
    // Retrieve the frame data from PROGMEM and display it on the LED matrix
    ledmatrix.writeFull(PROGMEM_getAnything(&Gesturetable[aniMouth][index]));
}

//---------------------------------------------------------
//-- Otto Display: Put Mouth
//---------------------------------------------------------
// Parameters:
//    * mouth: The mouth pattern data
//    * predefined: Indicates whether the mouth pattern is predefined or custom
//---------------------------------------------------------
void Otto::putMouth(unsigned long int mouth, bool predefined) {
    if (predefined) {
        // If the mouth pattern is predefined, retrieve it from PROGMEM
        ledmatrix.writeFull(PROGMEM_getAnything(&Mouthtable[mouth]));
    } else {
        // If the mouth pattern is custom, directly write it
        ledmatrix.writeFull(mouth);
    }
}

//---------------------------------------------------------
//-- Otto Display: Clear Mouth
//---------------------------------------------------------
void Otto::clearMouth() {
    ledmatrix.clearMatrix();
}

//---------------------------------------------------------
//-- Otto Display: Write Text
//---------------------------------------------------------
// Parameters:
//    * s: The text to display
//    * scrollspeed: The scrolling speed
//---------------------------------------------------------
void Otto::writeText(const char *s, byte scrollspeed) {
    int a, b;

    // Determine the length of the text (up to 9 characters)
    for (a = 0; s[a] != '\0'; a++) {
        b = a + 1;
        if (b > 9) b = 9; // Limit the maximum length to 9 characters
    }

    // Display each character
    for (int charNUMBER = 0; charNUMBER < b; charNUMBER++) {
        if ((*s < 48) || (*s > 91)) { // Check if the character is not within the alphanumeric range
            if (*s == 32) { // If it's a space character
                ledmatrix.sendChar(44, charNUMBER, b, scrollspeed);
            } else { // If it's a non-alphanumeric character
                ledmatrix.sendChar(43, charNUMBER, b, scrollspeed);
            }
        } else { // If it's an alphanumeric character
            ledmatrix.sendChar((*s - 48), charNUMBER, b, scrollspeed);
        }
        s++; // Move to the next character
    }
}

//---------------------------------------------------------
//-- Otto Sound: Play Tone
//---------------------------------------------------------
// Parameters:
//   * noteFrequency: Frequency of the tone to play
//   * noteDuration: Duration of the tone
//   * silentDuration: Duration of silence after the tone
//---------------------------------------------------------
void Otto::_tone(float noteFrequency, long noteDuration, int silentDuration) {
    // Ensure silent duration is at least 1 millisecond
    if (silentDuration == 0) {
        silentDuration = 1;
    }

    // Play the tone for the specified duration
    tone(Otto::pinBuzzer, noteFrequency, noteDuration);
    delay(noteDuration);
    // Add a delay for the specified silent duration
    delay(silentDuration);
}

//---------------------------------------------------------
//-- Otto Sound: Bend Tones
//---------------------------------------------------------
// Parameters:
//   * initFrequency: Initial frequency
//   * finalFrequency: Final frequency
//   * prop: Proportion of change in frequency
//   * noteDuration: Duration of each note
//   * silentDuration: Duration of silence between notes
//---------------------------------------------------------
void Otto::bendTones(float initFrequency, float finalFrequency, float prop, long noteDuration, int silentDuration) {
    // Ensure silent duration is at least 1 millisecond
    if (silentDuration == 0) {
        silentDuration = 1;
    }

    // Bend the tones based on the frequency change proportion
    if (initFrequency < finalFrequency) {
        // Ascending frequency
        for (int i = initFrequency; i < finalFrequency; i *= prop) {
            _tone(i, noteDuration, silentDuration);
        }
    } else {
        // Descending frequency
        for (int i = initFrequency; i > finalFrequency; i /= prop) {
            _tone(i, noteDuration, silentDuration);
        }
    }
}

//---------------------------------------------------------
//-- Otto Sound: Sing
//---------------------------------------------------------
// Parameters:
//   * songName: The name of the song to sing (defined constants)
//---------------------------------------------------------
void Otto::sing(int songName) {
    switch (songName) {
        case S_connection:
            _tone(note_E5, 50, 30);
            _tone(note_E6, 55, 25);
            _tone(note_A6, 60, 10);
            break;
        case S_disconnection:
            _tone(note_E5, 50, 30);
            _tone(note_A6, 55, 25);
            _tone(note_E6, 50, 10);
            break;
        case S_buttonPushed:
            bendTones(note_E6, note_G6, 1.03, 20, 2);
            delay(30);
            bendTones(note_E6, note_D7, 1.04, 10, 2);
            break;
        case S_mode1:
            bendTones(note_E6, note_A6, 1.02, 30, 10);
            break;
        case S_mode2:
            bendTones(note_G6, note_D7, 1.03, 30, 10);
            break;
        case S_mode3:
            _tone(note_E6, 50, 100);
            _tone(note_G6, 50, 80);
            _tone(note_D7, 300, 0);
            break;
        case S_surprise:
            bendTones(800, 2150, 1.02, 10, 1);
            bendTones(2149, 800, 1.03, 7, 1);
            break;
        case S_OhOoh:
            bendTones(880, 2000, 1.04, 8, 3);
            delay(200);
            for (int i = 880; i < 2000; i = i * 1.04) {
                _tone(note_B5, 5, 10);
            }
            break;
        case S_OhOoh2:
            bendTones(1880, 3000, 1.03, 8, 3);
            delay(200);
            for (int i = 1880; i < 3000; i = i * 1.03) {
                _tone(note_C6, 10, 10);
            }
            break;
        case S_cuddly:
            bendTones(700, 900, 1.03, 16, 4);
            bendTones(899, 650, 1.01, 18, 7);
            break;
        case S_sleeping:
            bendTones(100, 500, 1.04, 10, 10);
            delay(500);
            bendTones(400, 100, 1.04, 10, 1);
            break;
        case S_happy:
            bendTones(1500, 2500, 1.05, 20, 8);
            bendTones(2499, 1500, 1.05, 25, 8);
            break;
        case S_superHappy:
            bendTones(2000, 6000, 1.05, 8, 3);
            delay(50);
            bendTones(5999, 2000, 1.05, 13, 2);
            break;
        case S_happy_short:
            bendTones(1500, 2000, 1.05, 15, 8);
            delay(100);
            bendTones(1900, 2500, 1.05, 10, 8);
            break;
        case S_sad:
            bendTones(880, 669, 1.02, 20, 200);
            break;
        case S_confused:
            bendTones(1000, 1700, 1.03, 8, 2);
            bendTones(1699, 500, 1.04, 8, 3);
            bendTones(1000, 1700, 1.05, 9, 10);
            break;
        case S_fart1:
            bendTones(1600, 3000, 1.02, 2, 15);
            break;
        case S_fart2:
            bendTones(2000, 6000, 1.02, 2, 20);
            break;
        case S_fart3:
            bendTones(1600, 4000, 1.02, 2, 20);
            bendTones(4000, 3000, 1.02, 2, 20);
            break;
    }
}

//---------------------------------------------------------
//-- Otto Gesture: Play Gesture
//---------------------------------------------------------
// Parameters:
//   * gesture: The name of the gesture to play (defined constants)
//---------------------------------------------------------
void Otto::playGesture(int gesture) {
  int gesturePOSITION[4];
  
  switch(gesture) {
    case OttoHappy: 
      _tone(note_E5, 50, 30);
      putMouth(smile);
      sing(S_happy_short);
      swing(1, 800, 20); 
      sing(S_happy_short);
      home();
      putMouth(happyOpen);
      break;

    case OttoSuperHappy:
      putMouth(happyOpen);
      sing(S_happy);
      putMouth(happyClosed);
      tiptoeSwing(1, 500, 20);
      putMouth(happyOpen);
      sing(S_superHappy);
      putMouth(happyClosed);
      tiptoeSwing(1, 500, 20); 
      home();  
      putMouth(happyOpen);
      break;

    case OttoSad: 
      putMouth(sad);
      gesturePOSITION[0] = 110;
      gesturePOSITION[1] = 70;
      gesturePOSITION[2] = 20;
      gesturePOSITION[3] = 160;
      _moveServos(700, gesturePOSITION);     
      bendTones(880, 830, 1.02, 20, 200);
      putMouth(sadClosed);
      bendTones(830, 790, 1.02, 20, 200);  
      putMouth(sadOpen);
      bendTones(790, 740, 1.02, 20, 200);
      putMouth(sadClosed);
      bendTones(740, 700, 1.02, 20, 200);
      putMouth(sadOpen);
      bendTones(700, 669, 1.02, 20, 200);
      putMouth(sad);
      delay(500);
      home();
      delay(300);
      putMouth(happyOpen);
      break;

    case OttoSleeping:
      gesturePOSITION[0] = 100;
      gesturePOSITION[1] = 80;
      gesturePOSITION[2] = 60;
      gesturePOSITION[3] = 120;
      _moveServos(700, gesturePOSITION);     
      for(int i = 0; i < 4; i++) {
        putAnimationMouth(dreamMouth, 0);
        bendTones(100, 200, 1.04, 10, 10);
        putAnimationMouth(dreamMouth, 1);
        bendTones(200, 300, 1.04, 10, 10);  
        putAnimationMouth(dreamMouth, 2);
        bendTones(300, 500, 1.04, 10, 10);   
        delay(500);
        putAnimationMouth(dreamMouth, 1);
        bendTones(400, 250, 1.04, 10, 1); 
        putAnimationMouth(dreamMouth, 0);
        bendTones(250, 100, 1.04, 10, 1); 
        delay(500);
      } 
      putMouth(lineMouth);
      sing(S_cuddly);
      home();  
      putMouth(happyOpen);
      break;

    case OttoFart:
      gesturePOSITION[0] = 90;
      gesturePOSITION[1] = 90;
      gesturePOSITION[2] = 145;
      gesturePOSITION[3] = 122;
      _moveServos(500, gesturePOSITION);
      delay(300);     
      putMouth(lineMouth);
      sing(S_fart1);  
      putMouth(tongueOut);
      delay(250);
      gesturePOSITION[0] = 90;
      gesturePOSITION[1] = 90;
      gesturePOSITION[2] = 80;
      gesturePOSITION[3] = 122;
      _moveServos(500, gesturePOSITION);
      delay(300);
      putMouth(lineMouth);
      sing(S_fart2); 
      putMouth(tongueOut);
      delay(250);
      gesturePOSITION[0] = 90;
      gesturePOSITION[1] = 90;
      gesturePOSITION[2] = 145;
      gesturePOSITION[3] = 80;
      _moveServos(500, gesturePOSITION);
      delay(300);
      putMouth(lineMouth);
      sing(S_fart3);
      putMouth(tongueOut);    
      delay(300);
      home(); 
      delay(500); 
      putMouth(happyOpen);
      break;

    case OttoConfused:
      gesturePOSITION[0] = 110;
      gesturePOSITION[1] = 70;
      gesturePOSITION[2] = 90;
      gesturePOSITION[3] = 90;
      _moveServos(300, gesturePOSITION); 
      putMouth(confused);
      sing(S_confused);
      delay(500);
      home();  
      putMouth(happyOpen);
      break;

    case OttoLove:
      putMouth(heart);
      sing(S_cuddly);
      crusaito(2, 1500, 15, 1);
      home(); 
      sing(S_happy_short);  
      putMouth(happyOpen);
      break;

    case OttoAngry: 
      gesturePOSITION[0] = 90;
      gesturePOSITION[1] = 90;
      gesturePOSITION[2] = 70;
      gesturePOSITION[3] = 110;
      _moveServos(300, gesturePOSITION); 
      putMouth(angry);
      _tone(note_A5, 100, 30);
      bendTones(note_A5, note_D6, 1.02, 7, 4);
      bendTones(note_D6, note_G6, 1.02, 10, 1);
      bendTones(note_G6, note_A5, 1.02, 10, 1);
      delay(15);
      bendTones(note_A5, note_E5, 1.02, 20, 4);
      delay(400);
      gesturePOSITION[0] = 110;
      gesturePOSITION[1] = 110;
      gesturePOSITION[2] = 90;
      gesturePOSITION[3] = 90;
      _moveServos(200, gesturePOSITION); 
      bendTones(note_A5, note_D6, 1.02, 20, 4);
      gesturePOSITION[0] = 70;
      gesturePOSITION[1] = 70;
      gesturePOSITION[2] = 90;
      gesturePOSITION[3] = 90;
      _moveServos(200, gesturePOSITION); 
      bendTones(note_A5, note_E5, 1.02, 20, 4);
      home();  
      putMouth(happyOpen);
      break;

    case OttoFretful: 
      putMouth(angry);
      bendTones(note_A5, note_D6, 1.02, 20, 4);
      bendTones(note_A5, note_E5, 1.02, 20, 4);
      delay(300);
      putMouth(lineMouth);
      for(int i = 0; i < 4; i++) {
        gesturePOSITION[0] = 90;
        gesturePOSITION[1] = 90;
        gesturePOSITION[2] = 90;
        gesturePOSITION[3] = 110;
        _moveServos(100, gesturePOSITION);   
        home();
      }
      putMouth(angry);
      delay(500);
      home();  
      putMouth(happyOpen);
      break;

    case OttoMagic:
      for(int i = 0; i < 4; i++) { 
        int noteM = 400; 
        for(int index = 0; index < 6; index++) {
          putAnimationMouth(adivinawi, index);
          bendTones(noteM, noteM + 100, 1.04, 10, 10);    // 400 -> 1000 
          noteM += 100;
        }
        clearMouth();
        bendTones(noteM - 100, noteM + 100, 1.04, 10, 10);  // 900 -> 1100
        for(int index = 0; index < 6; index++) {
          putAnimationMouth(adivinawi, index);
          bendTones(noteM, noteM + 100, 1.04, 10, 10);    // 1000 -> 400 
          noteM -= 100;
        }
      } 
      delay(300);
      putMouth(happyOpen);
      break;

    case OttoWave:
      for(int i = 0; i < 2; i++) { 
        int noteW = 500; 
        for(int index = 0; index < 10; index++) {
          putAnimationMouth(wave, index);
          bendTones(noteW, noteW + 100, 1.02, 10, 10); 
          noteW += 101;
        }
        for(int index = 0; index < 10; index++) {
          putAnimationMouth(wave, index);
          bendTones(noteW, noteW + 100, 1.02, 10, 10); 
          noteW += 101;
        }
        for(int index = 0; index < 10; index++) {
          putAnimationMouth(wave, index);
          bendTones(noteW, noteW - 100, 1.02, 10, 10); 
          noteW -= 101;
        }
        for(int index = 0; index < 10; index++) {
          putAnimationMouth(wave, index);
          bendTones(noteW, noteW - 100, 1.02, 10, 10); 
          noteW -= 101;
        }
      }    
      clearMouth();
      delay(100);
      putMouth(happyOpen);
      break;

    case OttoVictory:
      putMouth(smallSurprise);
      for (int i = 0; i < 60; ++i) {
        int pos[] = {90, 90, 90 + i, 90 - i};  
        _moveServos(10, pos);
        _tone(1600 + i * 20, 15, 1);
      }
      putMouth(bigSurprise);
      for (int i = 0; i < 60; ++i) {
        int pos[] = {90, 90, 150 - i, 30 + i};  
        _moveServos(10, pos);
        _tone(2800 + i * 20, 15, 1);
      }
      putMouth(happyOpen);
      tiptoeSwing(1, 500, 20);
      sing(S_superHappy);
      putMouth(happyClosed);
      tiptoeSwing(1, 500, 20); 
      home();
      clearMouth();
      putMouth(happyOpen);
      break;

    case OttoFail:
      putMouth(sadOpen);
      gesturePOSITION[0] = 90;
      gesturePOSITION[1] = 90;
      gesturePOSITION[2] = 70;
      gesturePOSITION[3] = 35;
      _moveServos(300, gesturePOSITION);
      _tone(900, 200, 1);
      putMouth(sadClosed);
      gesturePOSITION[0] = 90;
      gesturePOSITION[1] = 90;
      gesturePOSITION[2] = 55;
      gesturePOSITION[3] = 35;
      _moveServos(300, gesturePOSITION);
      _tone(600, 200, 1);
      putMouth(confused);
      gesturePOSITION[0] = 90;
      gesturePOSITION[1] = 90;
      gesturePOSITION[2] = 42;
      gesturePOSITION[3] = 35;
      _moveServos(300, gesturePOSITION);
      _tone(300, 200, 1);
      gesturePOSITION[0] = 90;
      gesturePOSITION[1] = 90;
      gesturePOSITION[2] = 34;
      gesturePOSITION[3] = 35;
      _moveServos(300, gesturePOSITION);
      putMouth(xMouth);
      detachServos();
      _tone(150, 2200, 1);
      delay(600);
      clearMouth();
      putMouth(happyOpen);
      home();
      break;
  }
}

//---------------------------------------------------------
//-- Otto Servo: Enable Servo Limit
//---------------------------------------------------------
// Parameters:
//    * diff_limit: The difference limit for servo movement
//---------------------------------------------------------
void Otto::enableServoLimit(int diff_limit) {
    for (int i = 0; i < 4; i++) {
        // Set the difference limit for each servo
        servo[i].SetLimiter(diff_limit);
    }
}

//---------------------------------------------------------
//-- Otto Servo: Disable Servo Limit
//---------------------------------------------------------
// Description:
//    This function disables the servo limits previously set.
//---------------------------------------------------------
void Otto::disableServoLimit() {
    for (int i = 0; i < 4; i++) {
        // Disable the servo limit for each servo
        servo[i].DisableLimiter();
    }
}

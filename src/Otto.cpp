// OttoDIY Arduino Library project 2024

// Zowi (c) BQ. Released under a GPL licencse 2015

#include "Otto.h"

//---------------------------------------------------------
//-- Otto Init: Initialize
//---------------------------------------------------------
// Description:
//    This function initializes Otto by setting up freertos task & queues, servo pins,
//    attaching servos, loading calibration data if specified, and configuring the buzzer pin.
// Parameters:
//    name: Name for the Otto
// Returns:
//    Integer value representing Otto's init status. 0 = successful -# = error
//---------------------------------------------------------
int Otto::init(const char * name) {
  // fixme
  return -1;
}

//---------------------------------------------------------
//-- Otto Init: Initialize compatibility wrapper
//---------------------------------------------------------
// Description:
//    This function initializes Otto by setting up freertos task & queues, servo pins,
//    attaching servos, loading calibration data if specified, and configuring the buzzer pin.
// Parameters:
//    YL: Pin for left hip servo
//    YR: Pin for right hip servo
//    RL: Pin for left foot servo
//    RR: Pin for right foot servo
//    load_calibration: Whether to load calibration data from EEPROM
//    Buzzer: Pin for the buzzer
//---------------------------------------------------------
void Otto::init(int YL, int YR, int RL, int RR, bool load_calibration, int Buzzer) {
#if Otto_sound == SOUND_BUZZER
	Sound_init(Buzzer);
#endif // SOUND_BUZZER	
    Servos_init(YL, YR, RL, RR, load_calibration);
}

int Otto::Servos_init(int YL, int YR, int RL, int RR, bool load_calibration) {
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
	return 0; // success
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
void Otto::oscillateServos(int A[4], int O[4], int T, double phase_diff[4], float cycle) {
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
void Otto::_execute(int A[4], int O[4], int T, double phase_diff[4], float steps) {
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

#if Otto_mouth == MOUTH_8X8_MONO_SPI

//---------------------------------------------------------
//-- Otto Mouth: return the number of entries in the mouth queue
//---------------------------------------------------------
// Returns:
//    Integer value representing the number of entries in the mouth queue.
//-------------------------------------------------------
int Otto::Mouth_queueSize() {
  if(mouthQueueHandle != NULL) {
    return uxQueueMessagesWaiting(mouthQueueHandle);
  } else {
      Serial.println(F("Error: Mouth command queue doesn't exist."));
      return -2; // Queue doesn't exist
  }
}

//---------------------------------------------------------
//-- Otto Mouth: check if the mouth queue is empty
//---------------------------------------------------------
// Returns:
//    Boolen value representing if the mouth queue is empty.
//-------------------------------------------------------
bool Otto::Mouth_isEmptyQueue() {
  if(Mouth_queueSize() == 0) {
    return true; // true if queue empty
  } else {
    return false; // false if busy or error (Queue doesn't exist)
  }
}

//---------------------------------------------------------
//-- Otto : clear all mouths currently in the mouth queue.
//---------------------------------------------------------
// Returns:Mouth
//    Integer value representing the number of entries in the mouth queue.
//-------------------------------------------------------
// Function to clear all mouths currently in the queue
int Otto::Mouth_clearQueue() {
  if(mouthQueueHandle != NULL) {
    if(xQueueReset(mouthQueueHandle) == pdPASS) {
      return 0; // Successfully cleared the queue
    } else {
      Serial.println(F("Error: Failed to clear the mouth command queue!"));
      return -1; // Failed to clear the queue
    }
  }
  Serial.println(F("Error: Mouth queue doesn't exist."));
  return -2; // Queue doesn't exist
}

//---------------------------------------------------------
//-- Otto Mouth: Initialize Matrix compatibility wrapper
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
	Mouth_init(DIN, CS, CLK, rotate);
}

//---------------------------------------------------------
//-- Otto Mouth: Initialize Matrix
//---------------------------------------------------------
// Description:
//    This function initializes the LED matrix display.
// Parameters:
//    DIN: Data input pin for the LED matrix
//    CS: Chip select pin for the LED matrix
//    CLK: Clock pin for the LED matrix
//    rotate: Rotation value for the LED matrix (0, 1, 2, or 3)
// Returns:
//    Integer value representing status 0 = success -# = error.
//---------------------------------------------------------
int Otto::Mouth_init(int DIN, int CS, int CLK, int rotate) {
	
	// check to see if matrix already initialized 
	if(ledmatrix.initialized) {
        Serial.println(F("Error: matrix already initialized"));
        return -3; // matrix already initialized
	}

    // Initialize LED matrix display
	// fixme: relocate to mouth task startup ... needs global matrix setings ...
    ledmatrix.init(DIN, CS, CLK, 1, rotate);
 
    // Check if the initialize matrix was successful
    if(!ledmatrix.initialized) {
      Serial.println(F("Error: Failed to initialize matrix!"));
      return -4; // Failed to initialize LED matrix
    }
	
    // Create the mouth queue
    mouthQueueHandle = xQueueCreate(10, sizeof(struct MouthQueueMsg));

    // Check if the queue creation was successful
    if (mouthQueueHandle == NULL) {
      Serial.println(F("Error: Failed to create mouth queue!"));
      return -2; // Failed to create queue
    }
	
    // Create the mouth task
    xTaskCreate(mouthTaskWrapper, "Mouth Task", Mouth_Stack, this, 1, &mouthTaskHandle);

    // Check if the task creation was successful
    if (mouthTaskHandle == NULL) {
        Serial.println(F("Error: Failed to create mouth task!"));
        return -1; // Failed to create task
    }

	return 0; // Success 
}

//---------------------------------------------------------
//-- Otto Mouth: Set Matrix Intensity compatibility wrapper
//---------------------------------------------------------
// Description:
//    This function sets the intensity of the LED matrix display.
// Parameters:
//    intensity: Intensity level (0 to 15) for the LED matrix
//---------------------------------------------------------
void Otto::matrixIntensity(int intensity) {
	Mouth_intensity(intensity);
}

//---------------------------------------------------------
//-- Otto Mouth: Set Matrix Intensity
//---------------------------------------------------------
// Description:
//    This function sets the intensity of the LED matrix display.
// Parameters:
//    intensity: Intensity level (0 to 15) for the LED matrix
//    minimalDuration: the minimal duration to display in ms (defaults to 30 frames per second)
// Returns:
//    Integer value of entries in the mouth queue
//---------------------------------------------------------
int Otto::Mouth_intensity(int intensity, int minimalDuration) {
  int count = 0;
  MouthQueueMsg msg;

  // Check input level for valid range
  if((intensity < 0) || (intensity > 15)) {
      Serial.println(F("Error: Intensity out of range (0 to 15)."));
        return -4; // Intensity out of range			
  }
  
  // Check if the mouth queue exist
  if(mouthQueueHandle == NULL) {
      Serial.println(F("Error: Mouth queue doesn't exist."));
      return -2; // Queue doesn't exist
  }

  // Check the available space in the mouth queue
  if(uxQueueSpacesAvailable(mouthQueueHandle) == 0) {
	while(uxQueueSpacesAvailable(mouthQueueHandle) == 0) {
		delay(1); // delay 1ms
		count++;
		if(count > 100) { // max delay 100ms
			Serial.println(F("Error: No space available in the mouth queue."));
			return -3; // No space available in the mouth queue			
		}
	}
  }

  // Create a MouthQueueMsg structure for intensity command
  msg.command = MOUTH_INTENSITY;
  msg.cmd.intensity.intensity = intensity;
  msg.cmd.intensity.duration = minimalDuration;

  // Queue up the mouth parameters
  if(xQueueSendToBack(mouthQueueHandle, &msg, portMAX_DELAY) != pdTRUE) {
    Serial.println(F("Error: Failed to send to mouth queue."));
    return -1; // Failed to send to mouth queue
  }
  return uxQueueMessagesWaiting(mouthQueueHandle); // Return the number of entries in the mouth queue
}

//---------------------------------------------------------
//-- Otto Mouth: Set LED compatibility wrapper
//---------------------------------------------------------
// Description:
//    This function sets the state of a specific LED on the LED matrix.
// Parameters:
//    X: The x-coordinate of the LED.
//    Y: The y-coordinate of the LED.
//    value: The value to set for the LED (0 for OFF, 1 for ON).
//---------------------------------------------------------
void Otto::setLed(byte X, byte Y, byte value) {
	Mouth_setled(X, Y, value);
}

//---------------------------------------------------------
//-- Otto Mouth: Set LED
//---------------------------------------------------------
// Description:
//    This function sets the state of a specific LED on the LED matrix.
// Parameters:
//    X: The x-coordinate of the LED.
//    Y: The y-coordinate of the LED.
//    value: The value to set for the LED (0 for OFF, 1 for ON).
//    minimalDuration: the minimal duration to display in ms (defaults to 30 frames per second)
// Returns:
//    Integer value of entries in the mouth queue
//---------------------------------------------------------
int Otto::Mouth_setled(byte X, byte Y, byte value, int minimalDuration) {
  int count = 0;
  MouthQueueMsg msg;

  // Check if the mouth queue exist
  if(mouthQueueHandle == NULL) {
      Serial.println(F("Error: Mouth queue doesn't exist."));
      return -2; // Queue doesn't exist
  }

  // Check the available space in the mouth queue
  if(uxQueueSpacesAvailable(mouthQueueHandle) == 0) {
	while(uxQueueSpacesAvailable(mouthQueueHandle) == 0) {
		delay(1); // delay 1ms
		count++;
		if(count > 100) { // max delay 100ms
			Serial.println(F("Error: No space available in the mouth queue."));
			return -3; // No space available in the mouth queue			
		}
	}
  }

  // Create a MouthQueueMsg structure for setled command
  msg.command = MOUTH_SETLED;
  msg.cmd.setled.X = X;
  msg.cmd.setled.Y = Y;
  msg.cmd.setled.value = value;
  msg.cmd.setled.duration = minimalDuration;
  
  // Queue up the mouth parameters
  if(xQueueSendToBack(mouthQueueHandle, &msg, portMAX_DELAY) != pdTRUE) {
    Serial.println(F("Error: Failed to send to mouth queue."));
    return -1; // Failed to send to mouth queue
  }
  return uxQueueMessagesWaiting(mouthQueueHandle); // Return the number of entries in the mouth queue
}

//---------------------------------------------------------
//-- Otto Mouth: Put Animation Mouth frame compatibility wrapper
//---------------------------------------------------------
// Description:
//    This function displays a specific frame of a mouth animation on the LED matrix.
// Parameters:
//    aniMouth: The index of the animation sequence.
//    index: The index of the frame within the animation sequence.
//---------------------------------------------------------
void Otto::putAnimationMouth(unsigned long int aniMouth, int index) {
	Mouth_animation(aniMouth, index);
}

//---------------------------------------------------------
//-- Otto Mouth: Put Animation Mouth frame
//---------------------------------------------------------
// Description:
//    This function displays a specific frame of a mouth animation on the LED matrix.
// Parameters:
//    aniMouth: The index of the animation sequence.
//    index: The index of the frame within the animation sequence.
//    minimalDuration: the minimal duration to display in ms (defaults to 30 frames per second)
// Returns:
//    Integer value of entries in the mouth queue
//---------------------------------------------------------
int Otto::Mouth_animation(unsigned long int aniMouth, int index, int minimalDuration) {
  int count = 0;
  MouthQueueMsg msg;
  
  // Check if the mouth queue exist
  if(mouthQueueHandle == NULL) {
      Serial.println(F("Error: Mouth queue doesn't exist."));
      return -2; // Queue doesn't exist
  }

  // Check the available space in the mouth queue
  if(uxQueueSpacesAvailable(mouthQueueHandle) == 0) {
	while(uxQueueSpacesAvailable(mouthQueueHandle) == 0) {
		delay(1); // delay 1ms
		count++;
		if(count > 100) { // max delay 100ms
			Serial.println(F("Error: No space available in the mouth queue."));
			return -3; // No space available in the mouth queue			
		}
	}
  }

  // Create a MouthParameters structure - Retrieve the anim frame data from PROGMEM
  msg.command = MOUTH_MOUTH;
  msg.cmd.mouth.clear = false;
  msg.cmd.mouth.mouth = {PROGMEM_getAnything(&Gesturetable[aniMouth][index])};
  msg.cmd.mouth.duration = minimalDuration;

  // Queue up the mouth parameters
  if(xQueueSendToBack(mouthQueueHandle, &msg, portMAX_DELAY) != pdTRUE) {
    Serial.println(F("Error: Failed to send to mouth queue."));
    return -1; // Failed to send to mouth queue
  }
  return uxQueueMessagesWaiting(mouthQueueHandle); // Return the number of entries in the mouth queue
}

//---------------------------------------------------------
//-- Otto Mouth: Put Mouth compatibility wrapper
//---------------------------------------------------------
// Parameters:
//    * mouth: The mouth pattern data
//    * predefined: Indicates whether the mouth pattern is predefined or custom
//---------------------------------------------------------
void Otto::putMouth(unsigned long int mouth, bool predefined) {
	Mouth(mouth, predefined);
}

//---------------------------------------------------------
//-- Otto Mouth: Put Mouth
//---------------------------------------------------------
// Parameters:
//    * mouth: The mouth pattern data
//    * predefined: Indicates whether the mouth pattern is predefined or custom
//    * minimalDuration: the minimal duration to display in ms (defaults to 30 frames per second)
// Returns:
//    Integer value of entries in the mouth queue
//---------------------------------------------------------
int Otto::Mouth(unsigned long int mouth, bool predefined, int minimalDuration) {
  int count = 0;
  MouthQueueMsg msg;
  
  // Check if the mouth queue exist
  if(mouthQueueHandle == NULL) {
      Serial.println(F("Error: Mouth queue doesn't exist."));
      return -2; // Queue doesn't exist
  }

  // Check the available space in the mouth queue
  if(uxQueueSpacesAvailable(mouthQueueHandle) == 0) {
	while(uxQueueSpacesAvailable(mouthQueueHandle) == 0) {
		delay(1); // delay 1ms
		count++;
		if(count > 100) { // max delay 100ms
			Serial.println(F("Error: No space available in the mouth queue."));
			return -3; // No space available in the mouth queue			
		}
	}
  }

  // Create a MouthParameters structure
  msg.command = MOUTH_MOUTH;
  msg.cmd.mouth.clear = false;
  
  if (predefined) {
      // If the mouth pattern is predefined, retrieve it from PROGMEM
	  msg.cmd.mouth.mouth = {PROGMEM_getAnything(&Mouthtable[mouth])};
  } else {
      // If the mouth pattern is custom, directly write it
      msg.cmd.mouth.mouth = {mouth};
  }

  msg.cmd.mouth.duration = minimalDuration;
	
  // Queue up the mouth parameters
  if(xQueueSendToBack(mouthQueueHandle, &msg, portMAX_DELAY) != pdTRUE) {
    Serial.println(F("Error: Failed to send to mouth queue."));
    return -1; // Failed to send to mouth queue
  }
  return uxQueueMessagesWaiting(mouthQueueHandle); // Return the number of entries in the mouth queue
}

//---------------------------------------------------------
//-- Otto Mouth: Clear Mouth compatibility wrapper
//---------------------------------------------------------
void Otto::clearMouth() {
    Mouth_clear();
}

//---------------------------------------------------------
//-- Otto Mouth: Clear Mouth
// Parameters:
//    * minimalDuration: the minimal duration to display in ms (defaults to 30 frames per second)
// Returns:
//    Integer value of entries in the mouth queue
//---------------------------------------------------------
int Otto::Mouth_clear(int minimalDuration) {
  int count = 0;
  MouthQueueMsg msg;

  // Check if the mouth queue exist
  if(mouthQueueHandle == NULL) {
      Serial.println(F("Error: Mouth queue doesn't exist."));
      return -2; // Queue doesn't exist
  }

  // Check the available space in the mouth queue
  if(uxQueueSpacesAvailable(mouthQueueHandle) == 0) {
	while(uxQueueSpacesAvailable(mouthQueueHandle) == 0) {
		delay(1); // delay 1ms
		count++;
		if(count > 100) { // max delay 100ms
			Serial.println(F("Error: No space available in the mouth queue."));
			return -3; // No space available in the mouth queue			
		}
	}
  }
  
  // Create a MouthParameters structure
  msg.command = MOUTH_MOUTH;
  msg.cmd.mouth.mouth = 123;
  msg.cmd.mouth.clear = true;
  msg.cmd.mouth.duration = minimalDuration;

  // Queue up the mouth parameters
  if(xQueueSendToBack(mouthQueueHandle, &msg, portMAX_DELAY) != pdTRUE) {
    Serial.println(F("Error: Failed to send to mouth queue."));
    return -1; // Failed to send to mouth queue
  }
  return uxQueueMessagesWaiting(mouthQueueHandle); // Return the number of entries in the mouth queue
}

//---------------------------------------------------------
//-- Otto Mouth: Write Text compatibility wrapper
//---------------------------------------------------------
// Parameters:
//    * s: The text to display
//    * scrollspeed: The scrolling speed
//---------------------------------------------------------
void Otto::writeText(const char *s, byte scrollspeed) {
    Mouth_write(s, scrollspeed);
}

//---------------------------------------------------------
//-- Otto Mouth: Write Text
//---------------------------------------------------------
// Parameters:
//    * s: The text to display
//    * scrollspeed: The scrolling speed
// Return:
//    * the number of entries in the mouth queue
//---------------------------------------------------------
int Otto::Mouth_write(const char *s, byte scrollspeed, bool noblock) {
  int count = 0;
  MouthQueueMsg msg;
  msg.command = MOUTH_WRITE;
  msg.cmd.write.string = s;
  msg.cmd.write.speed = scrollspeed;

  // Check if the mouth queue exist
  if(mouthQueueHandle == NULL) {
      Serial.println(F("Error: Mouth queue doesn't exist."));
      return -2; // Queue doesn't exist
  }

  // Check the available space in the mouth queue
  if(uxQueueSpacesAvailable(mouthQueueHandle) == 0) {
	while(uxQueueSpacesAvailable(mouthQueueHandle) == 0) {
		delay(1); // delay 1ms
		count++;
		if(count > 100) { // max delay 100ms
			Serial.println(F("Error: No space available in the mouth queue."));
			return -3; // No space available in the mouth queue			
		}
	}
  }

  // Queue up the mouth parameters
  if(xQueueSendToBack(mouthQueueHandle, &msg, portMAX_DELAY) != pdTRUE) {
    Serial.println(F("Error: Failed to send to mouth queue."));
    return -1; // Failed to send to mouth queue
  }
  // fixme: handle noblock param to at least wait until queue is empty ...
  return uxQueueMessagesWaiting(mouthQueueHandle); // Return the number of entries in the mouth queue
}

//---------------------------------------------------------
//-- Otto Mouth: Animate Mouth
//---------------------------------------------------------
// Parameters:
//    * anim: The index of the animation sequence
//    * speed: The playback speed
//    * bounce: does the animation alternate direction when looping
//    * loop: does the animation loop when the queue is idle if not stays on last frame
// Return:
//    * the number of entries in the mouth queue
//---------------------------------------------------------
int Otto::Mouth_animate(unsigned long int anim, int speed, bool bounce, bool loop) {
  int count = 0;
  MouthQueueMsg msg;

  // Check if the mouth queue exist
  if(mouthQueueHandle == NULL) {
      Serial.println(F("Error: Mouth queue doesn't exist."));
      return -2; // Queue doesn't exist
  }

  // Check the available space in the mouth queue
  if(uxQueueSpacesAvailable(mouthQueueHandle) == 0) {
	while(uxQueueSpacesAvailable(mouthQueueHandle) == 0) {
		delay(1); // delay 1ms
		count++;
		if(count > 100) { // max delay 100ms
			Serial.println(F("Error: No space available in the mouth queue."));
			return -3; // No space available in the mouth queue			
		}
	}
  }

  msg.command = MOUTH_ANIMATE;
  msg.cmd.animate.anim = anim;
  msg.cmd.animate.speed = speed;
  msg.cmd.animate.bounce = bounce;
  msg.cmd.animate.loop = loop;
  
  // Queue up the mouth parameters
  if(xQueueSendToBack(mouthQueueHandle, &msg, portMAX_DELAY) != pdTRUE) {
    Serial.println(F("Error: Failed to send to mouth queue."));
    return -1; // Failed to send to mouth queue
  }
  return uxQueueMessagesWaiting(mouthQueueHandle); // Return the number of entries in the mouth queue
  
}

// Define c++ wrapper function for c mouthTask
void Otto::mouthTaskWrapper(void *pvParameters) {
  Otto* ottoInstance = static_cast<Otto*>(pvParameters);
  ottoInstance->mouthTask(pvParameters);
}

// Task to play the mouth functions
void Otto::mouthTask(void *pvParameters) {
  // contructor for the mouth message queue
  MouthQueueMsg Queuemsg;
  
  // Initialize LED matrix display
  //fixme: needs gloabl defines for mouth i2c/spi
  //ledmatrix.init(DIN, CS, CLK, 1, rotate);
  
  while(true) {
    if(xQueueReceive(mouthQueueHandle, &Queuemsg, portMAX_DELAY) == pdTRUE) {
        if(Queuemsg.command == MOUTH_MOUTH) { // Process StructType mouth
			if(Queuemsg.cmd.mouth.clear) {
			    ledmatrix.clearMatrix();
			    vTaskDelay(max (1U, (Queuemsg.cmd.mouth.duration / portTICK_PERIOD_MS) ));
			} else {
				ledmatrix.writeFull(Queuemsg.cmd.mouth.mouth);
			    vTaskDelay(max (1U, (Queuemsg.cmd.mouth.duration / portTICK_PERIOD_MS) ));
			}
        } else if(Queuemsg.command == MOUTH_INTENSITY) { // Process StructType intensity
            ledmatrix.setIntensity(Queuemsg.cmd.intensity.intensity); // Set intensity of LED matrix display
			vTaskDelay(max (1U, (Queuemsg.cmd.intensity.duration / portTICK_PERIOD_MS) ));
        } else if(Queuemsg.command == MOUTH_SETLED) { // Process StructType setled
	        ledmatrix.setDot(Queuemsg.cmd.setled.X, Queuemsg.cmd.setled.Y, Queuemsg.cmd.setled.value); // Set value of LED matrix at X/Y
			vTaskDelay(max (1U, (Queuemsg.cmd.setled.duration / portTICK_PERIOD_MS) ));
        } else if(Queuemsg.command == MOUTH_WRITE) { // Process StructType write
            int a, b;
            const char *originalString = Queuemsg.cmd.write.string;
            bool continueLoop = true;

            // Determine the length of the text (up to 12 characters)
            for(a = 0; Queuemsg.cmd.write.string[a] != '\0'; a++) {
                b = a + 1;
                if(b > 12) b = 12; // Limit the maximum length to 12 characters
            }

            // loop scolling letters
            while(continueLoop) {
                // Display each character
                for(int charNUMBER = 0; charNUMBER < b; charNUMBER++) {
                    if((*Queuemsg.cmd.write.string < 48) || (*Queuemsg.cmd.write.string > 91)) { // Check if the character is not within the alphanumeric range
                        if(*Queuemsg.cmd.write.string == 32) { // If it's a space character
                            ledmatrix.sendChar(44, charNUMBER, b, Queuemsg.cmd.write.speed);
                        } else { // If it's a non-alphanumeric character
                            ledmatrix.sendChar(43, charNUMBER, b, Queuemsg.cmd.write.speed);
                        }
                    } else { // If it's an alphanumeric character
                        ledmatrix.sendChar((*Queuemsg.cmd.write.string - 48), charNUMBER, b, Queuemsg.cmd.write.speed);
                    }
                    Queuemsg.cmd.write.string++; // Move to the next character
                }
				// Reset the string back to the beginning
                Queuemsg.cmd.write.string = originalString;
				
				// Check if something else is in the queue to break out of the loop
                if(uxQueueMessagesWaiting(mouthQueueHandle) > 0) {
                    continueLoop = false;
                }
			}
        } else if (Queuemsg.command == MOUTH_ANIMATE) { // Process StructType animate
            bool continueLoop = true;

            // loop animation
            while(continueLoop) {			
                for(int index = 0; index < 10; index++) { // zero to nine frames 10 total
		    		ledmatrix.writeFull(PROGMEM_getAnything(&Gesturetable[Queuemsg.cmd.animate.anim][index]));
		    	    vTaskDelay(max (1U, (Queuemsg.cmd.animate.speed / portTICK_PERIOD_MS) ));
		    	}
		    	if(Queuemsg.cmd.animate.bounce) {
                    for(int index = 10; index < 0; index--) { // zero to nine frames 10 total
	    	    		ledmatrix.writeFull(PROGMEM_getAnything(&Gesturetable[Queuemsg.cmd.animate.anim][index]));
		        	    vTaskDelay(max (1U, (Queuemsg.cmd.animate.speed / portTICK_PERIOD_MS) ));
			        }
			    }
				// Check if something else is in the queue to break out of the loop
                if(uxQueueMessagesWaiting(mouthQueueHandle) > 0) {
                    continueLoop = false;
                }
                // if loop is false break out of the loop
                if(Queuemsg.cmd.animate.loop == false) {
                    continueLoop = false;
                }				
			}
        }
    }
  } // endless loop
}

#else // dummy compatibility wrappers for MOUTH_8X8_MONO_SPI
void Otto::initMATRIX(int DIN, int CS, int CLK, int rotate) {
	return;
}

void Otto::matrixIntensity(int intensity) {
	return;
}

void Otto::setLed(byte X, byte Y, byte value) {
	return;
}

void Otto::writeText(const char* s, byte scrollspeed) {
	// fixme: delay (block) length of scroll animation ...
	return;
}

void Otto::putMouth(unsigned long int mouth, bool predefined = true){
	return;
}

void Otto::putAnimationMouth(unsigned long int anim, int index) {
	return;
}

void Otto::clearMouth(){
	return;
}

#endif // MOUTH_8X8_MONO_SPI

#if Otto_sound == SOUND_BUZZER

int Otto::Sound_init(int Buzzer) {

    // Create the tone queue
    toneQueueHandle = xQueueCreate(10, sizeof(ToneParameters));

    // Check if the queue creation was successful
    if (toneQueueHandle == NULL) {
      Serial.println(F("Error: Failed to create tone queue!"));
      return -2; // Failed to create queue
    }
    // Create the tone task
    xTaskCreate(soundTaskWrapper, "Sound Task", Sound_Stack, this, 1, &soundTaskHandle);

    // Check if the task creation was successful
    if (toneTaskHandle == NULL) {
        Serial.println(F("Error: Failed to create tone task!"));
        return -1; // Failed to create queue
    }
	
    // Set buzzer pin
    pinBuzzer = Buzzer;
    pinMode(Buzzer, OUTPUT);
	return 0; // success
}

//---------------------------------------------------------
//-- Otto Sound: return the number of entries in the tone queue
//---------------------------------------------------------
// Returns:
//    Integer value representing the number of entries in the tone queue.
//-------------------------------------------------------
int Otto::Sound_queueSize() {
  if (toneQueueHandle != NULL) {
    return uxQueueMessagesWaiting(toneQueueHandle);
  } else {
      Serial.println(F("Error: Tone queue doesn't exist."));
      return -2; // Queue doesn't exist
  }
}

//---------------------------------------------------------
//-- Otto Sound: check if the tone queue is empty
//---------------------------------------------------------
// Returns:
//    Boolen value representing if the tone queue is empty.
//-------------------------------------------------------
bool Otto::Sound_isEmptyQueue() {
  if(Sound_queueSize() == 0) {
    return true; // true if queue empty
  } else {
    return false; // false if busy or error (Queue doesn't exist)
  }
}

//---------------------------------------------------------
//-- Otto Sound: clear all tones currently in the tone queue.
//---------------------------------------------------------
// Returns:
//    Integer value representing the number of entries in the tone queue.
//-------------------------------------------------------
// Function to clear all tones currently in the queue
int Otto::Sound_clearQueue() {
  if (toneQueueHandle != NULL) {
    if (xQueueReset(toneQueueHandle) == pdPASS) {
      return 0; // Successfully cleared the queue
    } else {
      Serial.println(F("Error: Failed to clear the tone queue!"));
      return -1; // Failed to clear the queue
    }
  }
  Serial.println(F("Error: Tone queue doesn't exist."));
  return -2; // Queue doesn't exist
}

//---------------------------------------------------------
//-- Otto Sound: Play Tone compatibility wrapper
//---------------------------------------------------------
// Parameters:
//   * noteFrequency: Frequency of the tone to play
//   * noteDuration: Duration of the tone
//   * silentDuration: Duration of silence after the tone
//-------------------------------------------------------
void Otto::_tone(float frequency, long noteDuration, int silentDuration) {
  Sound_tone(frequency, noteDuration, silentDuration, Otto_code);
}

//---------------------------------------------------------
//-- Otto Sound: Play Tone and returns the number of entries in the tone queue 
//---------------------------------------------------------
// Parameters:
//   * noteFrequency: Frequency of the tone to play
//   * noteDuration: Duration of the tone
//   * silentDuration: Duration of silence after the tone
//   * noblock: code flow style blocking or non-blocking
// Returns:
//    Integer value representing the number of entries in the tone queue or error #.
//-------------------------------------------------------
int Otto::Sound_tone(float frequency, long noteDuration, int silentDuration, bool noblock) {
  int count = 0;

  // Check if the tone queue exist
  if(toneQueueHandle == NULL) {
      Serial.println(F("Error: Tone queue doesn't exist."));
      return -2; // Queue doesn't exist
  }

  // Check the available space in the tone queue
  if(uxQueueSpacesAvailable(toneQueueHandle) == 0) {
	//Serial.println(F("Stall: waiting for available space in the tone queue."));
	while(uxQueueSpacesAvailable(toneQueueHandle) == 0) {
		delay(1); // delay 1ms
		count++;
		//Serial.print(F("."));
		if(count > 200) { // max delay 200ms
			//Serial.println();
			Serial.println(F("Error: No space available in the tone queue."));
			return -3; // No space available in the tone queue			
		}
	}
	//Serial.println();
  }

  // Create a ToneParameters structure
  ToneParameters toneParams = {frequency, noteDuration, silentDuration};

  // Queue up the tone parameters
  if(xQueueSendToBack(toneQueueHandle, &toneParams, portMAX_DELAY) != pdTRUE) {
    Serial.println(F("Error: Failed to send to tone queue."));
    return -1; // Failed to send to tone queue
  }

  if(noblock) { // non-blocking code flow
    //int count = uxQueueMessagesWaiting(toneQueueHandle);
    //Serial.print("Tone queue size: ");
    //Serial.println(count);
    //return count; // Return the number of entries in the tone queue
    return uxQueueMessagesWaiting(toneQueueHandle); // Return the number of entries in the tone queue
  } else {  // blocking code flow
    while(uxQueueMessagesWaiting(toneQueueHandle) != 0) { 
      // wait for queue to empty
    }
    delay(noteDuration + silentDuration); // delay (block) for note duration + silence
    return 0;
  }
}

//---------------------------------------------------------
//-- Otto Sound: Bend Tones compatibility wrapper
//---------------------------------------------------------
// Parameters:
//   * initFrequency: Initial frequency
//   * finalFrequency: Final frequency
//   * prop: Proportion of change in frequency
//   * noteDuration: Duration of each note
//   * silentDuration: Duration of silence between notes
//---------------------------------------------------------
void Otto::bendTones(float initFrequency, float finalFrequency, float prop, long noteDuration, int silentDuration) {
	Sound_bendTones(initFrequency, finalFrequency, prop, noteDuration, silentDuration, Otto_code);
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
int Otto::Sound_bendTones(float initFrequency, float finalFrequency, float prop, long noteDuration, int silentDuration, bool noblock) {
	int result;
	//int total = 0;
	
    // Ensure silent duration is at least 1 millisecond
    if (silentDuration == 0) {
        silentDuration = 1;
    }

    // Bend the tones based on the frequency change proportion
    if (initFrequency < finalFrequency) {
        // Ascending frequency
        for (int i = initFrequency; i < finalFrequency; i *= prop) {
			//total = total + (noteDuration + silentDuration);
            result = Sound_tone(i, noteDuration, silentDuration, noblock);
	        if(result < 0) return result;
        }
    } else {
        // Descending frequency
        for (int i = initFrequency; i > finalFrequency; i /= prop) {
			//total = total + (noteDuration + silentDuration);
            result = Sound_tone(i, noteDuration, silentDuration, noblock);
	        if(result < 0) return result;
        }
    }
    //Serial.print(F("bendTones: total ms bendtone queue = "));
    //Serial.println(total);
    return 0; // sucess
}

// Define c++ wrapper function for c toneTask
void Otto::toneTaskWrapper(void *pvParameters) {
  Otto* ottoInstance = static_cast<Otto*>(pvParameters);
  ottoInstance->toneTask(pvParameters);
}

// Task to play the tone
void Otto::toneTask(void *pvParameters) {
  ToneParameters toneParams;

  while (true) {
    if (xQueueReceive(toneQueueHandle, &toneParams, portMAX_DELAY) == pdTRUE) {
	  if(toneParams.frequency != 0) { // ignore empty (rests ) notes
	    tone(Otto::pinBuzzer, toneParams.frequency, toneParams.noteDuration);
	    vTaskDelay(max (1U, (toneParams.noteDuration / portTICK_PERIOD_MS) ));
	  }
	  vTaskDelay(max (1U, (toneParams.silentDuration / portTICK_PERIOD_MS) ));
    }
  }
}

//---------------------------------------------------------
//-- Otto Sound: Sing compatibility wrapper
//---------------------------------------------------------
// Parameters:
//   * songName: The name of the song to sing (defined constants)
//---------------------------------------------------------
void Otto::sing(int songName) {
	Sound_sing(songName, Otto_code);
}

//---------------------------------------------------------
//-- Otto Sound: Sing
//---------------------------------------------------------
// Parameters:
//   * songName: The name of the song to sing (defined constants)
//   * noblock: code flow style blocking or non-blocking
// Returns:
//    Integer value representing success (0) or error #.
//---------------------------------------------------------
int Otto::Sound_sing(int songName, bool noblock) {
    int result = 0;
    switch (songName) {
        case S_connection:	// 230ms
            result = Sound_tone(note_E5, 50, 30, noblock);
            if(result < 0) return result;
            result = Sound_tone(note_E6, 55, 25, noblock);
            if(result < 0) return result;
            result = result = Sound_tone(note_A6, 60, 10, noblock);
            if(result < 0) return result;
            break;
        case S_disconnection:	// 220ms
            result = Sound_tone(note_E5, 50, 30, noblock);
            if(result < 0) return result;
            result = Sound_tone(note_A6, 55, 25, noblock);
            if(result < 0) return result;
            result = Sound_tone(note_E6, 50, 10, noblock);
            if(result < 0) return result;
            break;
        case S_buttonPushed:	// 312ms
            result = Sound_bendTones(note_E6, note_G6, 1.03, 20, 2, noblock);
            if(result < 0) return result;
            result = Sound_tone(note_0, 0, 30); // rests  note 
            if(result < 0) return result;
            result = Sound_bendTones(note_E6, note_D7, 1.04, 10, 2, noblock);
            if(result < 0) return result;
            break;
        case S_mode1:	// 600ms
            result = Sound_bendTones(note_E6, note_A6, 1.02, 30, 10, noblock);
            if(result < 0) return result;
            break;
        case S_mode2:	// 560ms
            result = Sound_bendTones(note_G6, note_D7, 1.03, 30, 10, noblock);
            if(result < 0) return result;
            break;
        case S_mode3:	// 580ms
            result = Sound_tone(note_E6, 50, 100, noblock);
            if(result < 0) return result;
            result = Sound_tone(note_G6, 50, 80, noblock);
            if(result < 0) return result;
            result = Sound_tone(note_D7, 300, 0, noblock);
            if(result < 0) return result;
            break;
        case S_surprise:	// 825ms
            result = Sound_bendTones(800, 2150, 1.02, 10, 1, noblock);
            if(result < 0) return result;
            result = Sound_bendTones(2149, 800, 1.03, 7, 1, noblock);
            if(result < 0) return result;
            break;
        case S_OhOoh:	// 242ms
            result = Sound_bendTones(880, 2000, 1.04, 8, 3, noblock);
            if(result < 0) return result;
            result = Sound_tone(note_0, 0, 100, noblock); // rests  note 
            if(result < 0) return result;
            result = Sound_tone(note_0, 0, 100, noblock); // rests  note 
            if(result < 0) return result;
            for (int i = 880; i < 2000; i = i * 1.04) {
            result = Sound_tone(note_B5, 5, 10, noblock);
            if(result < 0) return result;
            }
            break;
        case S_OhOoh2:	// 176ms
            result = Sound_bendTones(1880, 3000, 1.03, 8, 3, noblock);
            if(result < 0) return result;
            result = Sound_tone(note_0, 0, 100, noblock); // rests  note 
            if(result < 0) return result;
            result = Sound_tone(note_0, 0, 100, noblock); // rests  note 
            if(result < 0) return result;
            for (int i = 1880; i < 3000; i = i * 1.03) {
            result = Sound_tone(note_C6, 10, 10, noblock);
            if(result < 0) return result;
            }
            break;
        case S_cuddly:	// 955ms
            result = Sound_bendTones(700, 900, 1.03, 16, 4, noblock);
            if(result < 0) return result;
            result = Sound_bendTones(899, 650, 1.01, 18, 7, noblock);
            if(result < 0) return result;
            break;
        case S_sleeping:	// 1243ms
            result = Sound_bendTones(100, 500, 1.04, 10, 10, noblock);
            if(result < 0) return result;
            result = Sound_tone(note_0, 0, 100, noblock); // rests  note 
            if(result < 0) return result;
            result = Sound_tone(note_0, 0, 100, noblock); // rests  note 
            if(result < 0) return result;
            result = Sound_tone(note_0, 0, 100, noblock); // rests  note 
            if(result < 0) return result;
            result = Sound_tone(note_0, 0, 100, noblock); // rests  note 
            if(result < 0) return result;
            result = Sound_tone(note_0, 0, 100, noblock); // rests  note 
            if(result < 0) return result;
            result = Sound_bendTones(400, 100, 1.04, 10, 1, noblock);
            if(result < 0) return result;
            break;
        case S_happy:	// 671ms
            result = Sound_bendTones(1500, 2500, 1.05, 20, 8, noblock);
            if(result < 0) return result;
            result = Sound_bendTones(2499, 1500, 1.05, 25, 8, noblock);
            if(result < 0) return result;
            break;
        case S_superHappy:	// 598ms
            result = Sound_bendTones(2000, 6000, 1.05, 8, 3, noblock);
            if(result < 0) return result;
            result = Sound_tone(note_0, 0, 50); // rests  note 
            if(result < 0) return result;
            result = Sound_bendTones(5999, 2000, 1.05, 13, 2, noblock);
            if(result < 0) return result;
            break;
        case S_happy_short:	// 246ms
            result = Sound_bendTones(1500, 2000, 1.05, 15, 8, noblock);
            if(result < 0) return result;
            result = Sound_tone(note_0, 0, 100); // rests  note 
            if(result < 0) return result;
            result = Sound_bendTones(1900, 2500, 1.05, 10, 8, noblock);
            if(result < 0) return result;
            break;
        case S_sad:	// 3080ms
            result = Sound_bendTones(880, 669, 1.02, 20, 200, noblock);
            if(result < 0) return result;
            break;
        case S_confused:	// 740ms
            result = Sound_bendTones(1000, 1700, 1.03, 8, 2, noblock);
            if(result < 0) return result;
            result = Sound_bendTones(1699, 500, 1.04, 8, 3, noblock);
            if(result < 0) return result;
            result = Sound_bendTones(1000, 1700, 1.05, 9, 10, noblock);
            if(result < 0) return result;
            break;
        case S_fart1:	// 561ms
            result = Sound_bendTones(1600, 3000, 1.02, 2, 15, noblock);
            if(result < 0) return result;
            break;
        case S_fart2:	// 1232ms
            result = Sound_bendTones(2000, 6000, 1.02, 2, 20, noblock);
            if(result < 0) return result;
            break;
        case S_fart3:	// 1364ms
            result = Sound_bendTones(1600, 4000, 1.02, 2, 20, noblock);
            if(result < 0) return result;
            result = Sound_bendTones(4000, 3000, 1.02, 2, 20, noblock);
            if(result < 0) return result;
            break;
    }
    return 0; // return sucess
}
#else // SOUND_BUZZER dummy compatibility wrappers

//---------------------------------------------------------
//-- Otto Sound: Play Tone dummy compatibility wrapper
//---------------------------------------------------------
// Parameters:
//   * noteFrequency: Frequency of the tone to play
//   * noteDuration: Duration of the tone
//   * silentDuration: Duration of silence after the tone
//-------------------------------------------------------
void Otto::_tone(float frequency, long noteDuration, int silentDuration) {
	if(!Otto_code) delay(noteDuration + silentDuration); // block for length of note to stay compatable
}

//---------------------------------------------------------
//-- Otto Sound: Bend Tones dummy compatibility wrapper
//---------------------------------------------------------
// Parameters:
//   * initFrequency: Initial frequency
//   * finalFrequency: Final frequency
//   * prop: Proportion of change in frequency
//   * noteDuration: Duration of each note
//   * silentDuration: Duration of silence between notes
//---------------------------------------------------------
void Otto::bendTones(float initFrequency, float finalFrequency, float prop, long noteDuration, int silentDuration) {
	if(!Otto_code) {

		if (silentDuration == 0) {
			silentDuration = 1;
		}

		// Bend the tones based on the frequency change proportion
		if (initFrequency < finalFrequency) {
			// Ascending frequency
			for (int i = initFrequency; i < finalFrequency; i *= prop) {
				delay(noteDuration + silentDuration); // block for length of note to stay compatable
			}
		} else {
			// Descending frequency
			for (int i = initFrequency; i > finalFrequency; i /= prop) {
				delay(noteDuration + silentDuration); // block for length of note to stay compatable
			}
		}
	}
}

//---------------------------------------------------------
//-- Otto Sound: Sing dummy compatibility wrapper
//---------------------------------------------------------
// Parameters:
//   * songName: The name of the song to sing (defined constants)
//---------------------------------------------------------
void Otto::sing(int songName) {
	if(!Otto_code) {
		int song[19] = {230, 220, 312, 600, 560, 580, 825, 242, 176, 955, 1243, 671, 598, 246, 3080, 740, 561, 1232, 1364};
		delay(song[songName]); // block for length of song to stay compatable
	}
	return;
}

#endif // SOUND_BUZZER

//---------------------------------------------------------
//-- Otto Gesture: Play Gesture compatibility wrapper
//---------------------------------------------------------
// Parameters:
//   * gesture: The name of the gesture to play (defined constants)
//---------------------------------------------------------
void Otto::playGesture(int gesture) {
	Gesture(gesture, Otto_code);
}
	
//---------------------------------------------------------
//-- Otto Gesture: Play Gesture
//---------------------------------------------------------
// Parameters:
//   * gesture: The name of the gesture to play (defined constants)
// Returns:
//    Integer value representing success (0) or error #.
//---------------------------------------------------------
int Otto::Gesture(int gesture, bool noblock) {
  int gesturePOSITION[4];
  int result = 0;
  
  switch(gesture) {
    case OttoHappy: 
#if Otto_sound == SOUND_BUZZER
      result = Sound_tone(note_E5, 50, 30, noblock);
      //if(result < 0) return result;
#endif // SOUND_BUZZER
      putMouth(smile);
#if Otto_sound == SOUND_BUZZER
      result = Sound_sing(S_happy_short, noblock);
	  //if(result < 0) return result;
#endif // SOUND_BUZZER
      swing(1, 800, 20); 
#if Otto_sound == SOUND_BUZZER
      result = Sound_sing(S_happy_short, noblock);
	  //if(result < 0) return result;
#endif // SOUND_BUZZER
      home();
      putMouth(happyOpen);
      break;

    case OttoSuperHappy:
      putMouth(happyOpen);
#if Otto_sound == SOUND_BUZZER
      result = Sound_sing(S_happy, noblock);
	  //if(result < 0) return result;
#endif // SOUND_BUZZER
      putMouth(happyClosed);
      tiptoeSwing(1, 500, 20);
      putMouth(happyOpen);
#if Otto_sound == SOUND_BUZZER
      result = Sound_sing(S_superHappy, noblock);
	  //if(result < 0) return result;
#endif // SOUND_BUZZER
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
#if Otto_sound == SOUND_BUZZER
      result = Sound_bendTones(880, 830, 1.02, 20, 200, noblock);
	  //if(result < 0) return result;
#endif // SOUND_BUZZER
      putMouth(sadClosed);
#if Otto_sound == SOUND_BUZZER
      result = Sound_bendTones(830, 790, 1.02, 20, 200, noblock);  
	  //if(result < 0) return result;
#endif // SOUND_BUZZER
      putMouth(sadOpen);
#if Otto_sound == SOUND_BUZZER
      result = Sound_bendTones(790, 740, 1.02, 20, 200, noblock);
	  //if(result < 0) return result;
#endif // SOUND_BUZZER
      putMouth(sadClosed);
#if Otto_sound == SOUND_BUZZER
      result = Sound_bendTones(740, 700, 1.02, 20, 200, noblock);
	  //if(result < 0) return result;
#endif // SOUND_BUZZER
      putMouth(sadOpen);
#if Otto_sound == SOUND_BUZZER
      result = Sound_bendTones(700, 669, 1.02, 20, 200, noblock);
	  //if(result < 0) return result;
#endif // SOUND_BUZZER
      putMouth(sad);
      delay(500); // fixme no delays
      home();
      delay(300); // fixme no delays
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
#if Otto_sound == SOUND_BUZZER
        result = Sound_bendTones(100, 200, 1.04, 10, 10, noblock);
	  //if(result < 0) return result;
#endif // SOUND_BUZZER
        putAnimationMouth(dreamMouth, 1);
#if Otto_sound == SOUND_BUZZER
        result = Sound_bendTones(200, 300, 1.04, 10, 10, noblock);  
	  //if(result < 0) return result;
#endif // SOUND_BUZZER
        putAnimationMouth(dreamMouth, 2);
#if Otto_sound == SOUND_BUZZER
        result = Sound_bendTones(300, 500, 1.04, 10, 10, noblock);   
	  //if(result < 0) return result;
#endif // SOUND_BUZZER
        delay(500); // fixme no delays
        putAnimationMouth(dreamMouth, 1);
#if Otto_sound == SOUND_BUZZER
        result = Sound_bendTones(400, 250, 1.04, 10, 1, noblock); 
	  //if(result < 0) return result;
#endif // SOUND_BUZZER
        putAnimationMouth(dreamMouth, 0);
#if Otto_sound == SOUND_BUZZER
        result = Sound_bendTones(250, 100, 1.04, 10, 1, noblock); 
	  //if(result < 0) return result;
#endif // SOUND_BUZZER
        delay(500); // fixme no delays
      } 
      putMouth(lineMouth);
#if Otto_sound == SOUND_BUZZER
      result = Sound_sing(S_cuddly, noblock);
	  //if(result < 0) return result;
#endif // SOUND_BUZZER
      home();  
      putMouth(happyOpen);
      break;

    case OttoFart:
      gesturePOSITION[0] = 90;
      gesturePOSITION[1] = 90;
      gesturePOSITION[2] = 145;
      gesturePOSITION[3] = 122;
      _moveServos(500, gesturePOSITION);
      delay(300); // fixme no delays
      putMouth(lineMouth);
#if Otto_sound == SOUND_BUZZER
      result = Sound_sing(S_fart1, noblock);  
	  //if(result < 0) return result;
#endif // SOUND_BUZZER
      putMouth(tongueOut);
      delay(250); // fixme no delays
      gesturePOSITION[0] = 90;
      gesturePOSITION[1] = 90;
      gesturePOSITION[2] = 80;
      gesturePOSITION[3] = 122;
      _moveServos(500, gesturePOSITION);
      delay(300); // fixme no delays
      putMouth(lineMouth);
#if Otto_sound == SOUND_BUZZER
      result = Sound_sing(S_fart2, noblock); 
	  //if(result < 0) return result;
#endif // SOUND_BUZZER
      putMouth(tongueOut);
      delay(250); // fixme no delays
      gesturePOSITION[0] = 90;
      gesturePOSITION[1] = 90;
      gesturePOSITION[2] = 145;
      gesturePOSITION[3] = 80;
      _moveServos(500, gesturePOSITION);
      delay(300); // fixme no delays
      putMouth(lineMouth);
#if Otto_sound == SOUND_BUZZER
      result = Sound_sing(S_fart3, noblock);
	  //if(result < 0) return result;
#endif // SOUND_BUZZER
      putMouth(tongueOut);    
      delay(300); // fixme no delays
      home(); 
      delay(500); // fixme no delays
      putMouth(happyOpen);
      break;

    case OttoConfused:
      gesturePOSITION[0] = 110;
      gesturePOSITION[1] = 70;
      gesturePOSITION[2] = 90;
      gesturePOSITION[3] = 90;
      _moveServos(300, gesturePOSITION); 
      putMouth(confused);
#if Otto_sound == SOUND_BUZZER
      result = Sound_sing(S_confused, noblock);
	  //if(result < 0) return result;
#endif // SOUND_BUZZER
      delay(500); // fixme no delays
      home();  
      putMouth(happyOpen);
      break;

    case OttoLove:
      putMouth(heart);
#if Otto_sound == SOUND_BUZZER
      result = Sound_sing(S_cuddly, noblock);
	  //if(result < 0) return result;
#endif // SOUND_BUZZER
      crusaito(2, 1500, 15, 1);
      home(); 
#if Otto_sound == SOUND_BUZZER
      result = Sound_sing(S_happy_short, noblock);  
	  //if(result < 0) return result;
#endif // SOUND_BUZZER
      putMouth(happyOpen);
      break;

    case OttoAngry: 
      gesturePOSITION[0] = 90;
      gesturePOSITION[1] = 90;
      gesturePOSITION[2] = 70;
      gesturePOSITION[3] = 110;
      _moveServos(300, gesturePOSITION); 
      putMouth(angry);
#if Otto_sound == SOUND_BUZZER
      result = Sound_tone(note_A5, 100, 30, noblock);
	  //if(result < 0) return result;
      result = Sound_bendTones(note_A5, note_D6, 1.02, 7, 4, noblock);
 	  //if(result < 0) return result;
      result = Sound_bendTones(note_D6, note_G6, 1.02, 10, 1, noblock);
	  //if(result < 0) return result;
      result = Sound_bendTones(note_G6, note_A5, 1.02, 10, 1, noblock);
	  //if(result < 0) return result;
      result = Sound_tone(note_0, 0, 15, noblock); // rests note 15ms replace delay 
	  //if(result < 0) return result;
      result = Sound_bendTones(note_A5, note_E5, 1.02, 20, 4, noblock);
	  //if(result < 0) return result;
#endif // SOUND_BUZZER
      delay(400); // fixme no delays
      gesturePOSITION[0] = 110;
      gesturePOSITION[1] = 110;
      gesturePOSITION[2] = 90;
      gesturePOSITION[3] = 90;
      _moveServos(200, gesturePOSITION); 
#if Otto_sound == SOUND_BUZZER
      result = Sound_bendTones(note_A5, note_D6, 1.02, 20, 4, noblock);
	  //if(result < 0) return result;
#endif // SOUND_BUZZER
      gesturePOSITION[0] = 70;
      gesturePOSITION[1] = 70;
      gesturePOSITION[2] = 90;
      gesturePOSITION[3] = 90;
      _moveServos(200, gesturePOSITION); 
#if Otto_sound == SOUND_BUZZER
      result = Sound_bendTones(note_A5, note_E5, 1.02, 20, 4, noblock);
	  //if(result < 0) return result;
#endif // SOUND_BUZZER
      home();  
      putMouth(happyOpen);
      break;

    case OttoFretful: 
      putMouth(angry);
#if Otto_sound == SOUND_BUZZER
      result = Sound_bendTones(note_A5, note_D6, 1.02, 20, 4, noblock);
	  //if(result < 0) return result;
      result = Sound_bendTones(note_A5, note_E5, 1.02, 20, 4, noblock);
	  //if(result < 0) return result;
#endif // SOUND_BUZZER
      delay(300); // fixme no delays
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
      delay(500); // fixme no delays
      home();  
      putMouth(happyOpen);
      break;

    case OttoMagic:
      for(int i = 0; i < 4; i++) { 
#if Otto_sound == SOUND_BUZZER
        int noteM = 400; 
#endif // SOUND_BUZZER	  
        for(int index = 0; index < 6; index++) {
          putAnimationMouth(adivinawi, index);
#if Otto_sound == SOUND_BUZZER
          result = Sound_bendTones(noteM, noteM + 100, 1.04, 10, 10, noblock); // 400 -> 1000
	      //if(result < 0) return result;
          noteM += 100;
#endif // SOUND_BUZZER	  
        }
        clearMouth();
#if Otto_sound == SOUND_BUZZER
        result = Sound_bendTones(noteM - 100, noteM + 100, 1.04, 10, 10, noblock); // 900 -> 1100
	    //if(result < 0) return result;
#endif // SOUND_BUZZER	  
        for(int index = 0; index < 6; index++) {
          putAnimationMouth(adivinawi, index);
#if Otto_sound == SOUND_BUZZER
          result = Sound_bendTones(noteM, noteM + 100, 1.04, 10, 10, noblock); // 1000 -> 400
	      //if(result < 0) return result; 
          noteM -= 100;
#endif // SOUND_BUZZER	  
        }
      } 
      delay(300); // fixme no delays
      putMouth(happyOpen);
      break;

    case OttoWave:
      for(int i = 0; i < 2; i++) { 
#if Otto_sound == SOUND_BUZZER
        int noteW = 500; 
#endif // SOUND_BUZZER	  
        for(int index = 0; index < 10; index++) {
          putAnimationMouth(wave, index);
#if Otto_sound == SOUND_BUZZER
          result = Sound_bendTones(noteW, noteW + 100, 1.02, 10, 10, noblock);
	      //if(result < 0) return result;
          noteW += 101;
#endif // SOUND_BUZZER	  
        }
        for(int index = 0; index < 10; index++) {
          putAnimationMouth(wave, index);
#if Otto_sound == SOUND_BUZZER
          result = Sound_bendTones(noteW, noteW + 100, 1.02, 10, 10, noblock);
	      //if(result < 0) return result;
          noteW += 101;
#endif // SOUND_BUZZER	  
        }
        for(int index = 0; index < 10; index++) {
          putAnimationMouth(wave, index);
#if Otto_sound == SOUND_BUZZER
          result = Sound_bendTones(noteW, noteW - 100, 1.02, 10, 10, noblock);
	      //if(result < 0) return result; 
          noteW -= 101;
#endif // SOUND_BUZZER	  
        }
        for(int index = 0; index < 10; index++) {
          putAnimationMouth(wave, index);
#if Otto_sound == SOUND_BUZZER
          result = Sound_bendTones(noteW, noteW - 100, 1.02, 10, 10, noblock);
	      //if(result < 0) return result;
          noteW -= 101;
#endif // SOUND_BUZZER	  
        }
      }    
      clearMouth();
      delay(100); // fixme no delays
      putMouth(happyOpen);
      break;

    case OttoVictory:
      putMouth(smallSurprise);
      for (int i = 0; i < 60; ++i) {
        int pos[] = {90, 90, 90 + i, 90 - i};  
        _moveServos(10, pos);
#if Otto_sound == SOUND_BUZZER
        result = Sound_tone(1600 + i * 20, 15, 1, noblock);
        //if(result < 0) return result;
#endif // SOUND_BUZZER	  
      }
      putMouth(bigSurprise);
      for (int i = 0; i < 60; ++i) {
        int pos[] = {90, 90, 150 - i, 30 + i};  
        _moveServos(10, pos);
#if Otto_sound == SOUND_BUZZER
        result = Sound_tone(2800 + i * 20, 15, 1, noblock);
        //if(result < 0) return result;
#endif // SOUND_BUZZER	  
      }
      putMouth(happyOpen);
      tiptoeSwing(1, 500, 20);
#if Otto_sound == SOUND_BUZZER
      result = Sound_sing(S_superHappy, noblock);
	  //if(result < 0) return result;
#endif // SOUND_BUZZER
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
#if Otto_sound == SOUND_BUZZER
      result = Sound_tone(900, 200, 1, noblock);
      //if(result < 0) return result;
#endif // SOUND_BUZZER	  
      putMouth(sadClosed);
      gesturePOSITION[0] = 90;
      gesturePOSITION[1] = 90;
      gesturePOSITION[2] = 55;
      gesturePOSITION[3] = 35;
      _moveServos(300, gesturePOSITION);
#if Otto_sound == SOUND_BUZZER
      result = Sound_tone(600, 200, 1, noblock);
      //if(result < 0) return result;
#endif // SOUND_BUZZER	  
      putMouth(confused);
      gesturePOSITION[0] = 90;
      gesturePOSITION[1] = 90;
      gesturePOSITION[2] = 42;
      gesturePOSITION[3] = 35;
      _moveServos(300, gesturePOSITION);
#if Otto_sound == SOUND_BUZZER
      result = Sound_tone(300, 200, 1, noblock);
      //if(result < 0) return result;
#endif // SOUND_BUZZER	  
      gesturePOSITION[0] = 90;
      gesturePOSITION[1] = 90;
      gesturePOSITION[2] = 34;
      gesturePOSITION[3] = 35;
      _moveServos(300, gesturePOSITION);
      putMouth(xMouth);
      detachServos();
#if Otto_sound == SOUND_BUZZER
      result = Sound_tone(150, 2200, 1, noblock);
      //if(result < 0) return result;
#endif // SOUND_BUZZER	  
      delay(600); // fixme no delays
      clearMouth();
      putMouth(happyOpen);
      home();
      break;
  }
  return 0; // success
}


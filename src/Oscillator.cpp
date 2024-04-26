// OttoDIY Arduino Library project 2024

// (c) Juan Gonzalez-Gomez (Obijuan), Dec 2011 GPL license

#include "Arduino.h"
#include "Oscillator.h"

//---------------------------------------------------------
//-- Oscillator Function: Determine Next Sample
//---------------------------------------------------------
// Description:
//    This function determines whether it's time to take the next sample for servo movement.
//    It compares the current time with the previous time when a sample was taken.
// Return Value:
//    - true if it's time to take the next sample, false otherwise
//---------------------------------------------------------
bool Oscillator::next_sample() {
    // Get the current time
    _currentMillis = millis();
    
    // Check if enough time has elapsed since the last sample
    if (_currentMillis - _previousMillis > _samplingPeriod) {
        // Update the previous sample time
        _previousMillis = _currentMillis;
        return true; // It's time to take the next sample
    }
    
    return false; // Not yet time to take the next sample
}

//---------------------------------------------------------
//-- Oscillator Function: Attach Servo
//---------------------------------------------------------
// Description:
//    This function attaches a servo to the oscillator.
// Parameters:
//    - pin: The pin to which the servo is attached
//    - rev: Boolean flag indicating whether the servo movement is reversed
//---------------------------------------------------------
void Oscillator::attach(int pin, bool rev) {
    // Check if the servo is not already attached
    if (!_servo.attached()) {
        // Attach the servo to the specified pin
        _servo.attach(pin);
        
        // Set initial servo position to 90 degrees
        _pos = 90;
        _servo.write(90);
        
        // Initialize timing parameters
        _previousServoCommandMillis = millis();
        _samplingPeriod = 30;
        _period = 2000;
        _numberSamples = _period / _samplingPeriod;
        _inc = 2 * M_PI / _numberSamples;
        _previousMillis = 0;
        
        // Set default oscillator parameters
        _amplitude = 45;
        _phase = 0;
        _phase0 = 0;
        _offset = 0;
        _stop = false;
        _rev = rev;
    }
}

//---------------------------------------------------------
//-- Oscillator Function: Detach Servo
//---------------------------------------------------------
// Description:
//    This function detaches the servo associated with the oscillator.
//---------------------------------------------------------
void Oscillator::detach() {
    // Check if the servo is attached before detaching
    if (_servo.attached())
        _servo.detach();
}

//---------------------------------------------------------
//-- Oscillator Function: Set Period
//---------------------------------------------------------
// Description:
//    This function sets the period of the oscillator.
//---------------------------------------------------------
void Oscillator::SetT(unsigned int T) {
    // Update the period
    _period = T;
    
    // Calculate the number of samples based on the period and sampling period
    _numberSamples = _period / _samplingPeriod;
    
    // Calculate the increment based on the number of samples
    _inc = 2 * M_PI / _numberSamples;
}

//---------------------------------------------------------
//-- Oscillator Function: Set Position
//---------------------------------------------------------
// Description:
//    This function sets the position of the servo motor to
//    the specified value.
//---------------------------------------------------------
void Oscillator::SetPosition(int position) {
    // Write the specified position to the servo motor
    write(position);
}

//---------------------------------------------------------
//-- Oscillator Function: Refresh
//---------------------------------------------------------
// Description:
//    This function updates the position of the servo based on
//    the current oscillator parameters.
//---------------------------------------------------------
void Oscillator::refresh() {
    // Check if it's time to update the position
    if (next_sample()) {
        // Check if the oscillator is not stopped
        if (!_stop) {
            // Calculate the new position based on the oscillator parameters
            int pos = round(_amplitude * sin(_phase + _phase0) + _offset);

            // Apply reversal if necessary
            if (_rev)
                pos = -pos;

            // Write the position to the servo, adjusting for the neutral position
            write(pos + 90);
        }

        // Update the phase for the next sample
        _phase += _inc;
    }
}

//---------------------------------------------------------
//-- Oscillator Function: Write
//---------------------------------------------------------
// Description:
//    This function writes the specified position to the servo,
//    considering the differential limit if it's set.
// Parameters:
//    position: The target position to write to the servo
//---------------------------------------------------------
void Oscillator::write(int position) {
    // Get the current time in milliseconds
    long currentMillis = millis();

    // Apply differential limit if set
    if (_diff_limit > 0) {
        // Calculate the maximum change in position based on the time elapsed
        int limit = max(1, (((int)(currentMillis - _previousServoCommandMillis)) * _diff_limit) / 1000);

        // Adjust the position if it exceeds the limit
        if (abs(position - _pos) > limit) {
            _pos += position < _pos ? -limit : limit;
        } else {
            _pos = position;
        }
    } else {
        _pos = position; // Set position directly if no differential limit
    }

    // Update the previous servo command time
    _previousServoCommandMillis = currentMillis;

    // Write the adjusted position to the servo
    _servo.write(_pos + _trim);
}

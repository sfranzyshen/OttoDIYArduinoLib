// OttoDIY Arduino Library project 2024

// MaxMatrix - Copyright 2013 Oscar Kin-Chung Au

#include "Arduino.h"
#include "Otto_matrix.h"

Otto_Matrix::Otto_Matrix() : initialized(false) {}

// Initialize MAX7219 LED Matrix
void Otto_Matrix::init(byte _data, byte _load, byte _clock, byte _num, int _rotation) {
    // Assign pin values and rotation
    data = _data;
    load = _load;
    clock = _clock;
    num = _num;
    rotation = (_rotation > 4 || _rotation == 0) ? 1 : _rotation;

    // Initialize buffer arrays
    for (int i = 0; i < 8; i++)
        buffer[i] = 0;
    for (int i = 0; i < 80; i++)
        CHARbuffer[i] = 0;

    // Initialize SPI communication for ESP32
#if defined(ESP32)
    SPI.begin(clock, -1, data, load);
    SPI.setDataMode(SPI_MODE0);
    SPI.setClockDivider(SPI_CLOCK_DIV128);
    SPI.setHwCs(true);
#else
    // Initialize pin modes for non-ESP32 platforms
    pinMode(data, OUTPUT);
    pinMode(clock, OUTPUT);
    pinMode(load, OUTPUT);
    digitalWrite(clock, HIGH);
#endif

    // Configure MAX7219 registers
    setCommand(max7219_reg_scanLimit, 0x07);
    setCommand(max7219_reg_decodeMode, 0x00);
    setCommand(max7219_reg_shutdown, 0x01);
    setCommand(max7219_reg_displayTest, 0x00);

    // Clear display buffer and set intensity
    clearMatrix();
    setIntensity(0x0f);

    // Set the initialized flag to true after successful initialization
    initialized = true;
}

// Set LED Matrix Intensity
void Otto_Matrix::setIntensity(byte intensity) {
    // Send command to set intensity
    setCommand(max7219_reg_intensity, intensity);
}

// Clear LED Matrix
void Otto_Matrix::clearMatrix() {
    // Turn off all LEDs by setting all columns of each digit to zero
    for (int i = 0; i < 8; i++)
        setColumnAll(i, 0);

    // Reset internal buffer arrays
    for (int i = 0; i < 8; i++)
        buffer[i] = 0;

    for (int i = 0; i < 80; i++)
        CHARbuffer[i] = 0;
}

// Set command for MAX7219
void Otto_Matrix::setCommand(byte command, byte value) {
#if defined(ESP32)
    // For ESP32, use SPI.transfer16 to send command and value in one transfer
    SPI.transfer16(command << 8 | value);
#else
    // For other platforms, use shiftOut to send command and value byte by byte
    digitalWrite(load, LOW);  // Start data transfer
    for (int i = 0; i < num; i++) {
        shiftOut(data, clock, MSBFIRST, command);  // Send command byte
        shiftOut(data, clock, MSBFIRST, value);    // Send value byte
    }
    digitalWrite(load, LOW);   // End data transfer
    digitalWrite(load, HIGH);  // Latch data to MAX7219 chips
#endif
}

// Set column in LED matrix display
void Otto_Matrix::setColumn(byte col, byte value) {
    int n = col / 8;   // Calculate MAX7219 chip index
    int c = col % 8;   // Calculate column index within chip

#if defined(ESP32)
    // For ESP32, use SPI.transfer16 to send column index and value to the correct chip
    for (int i = 0; i < num; i++) {
        if (i == n)
            SPI.transfer16((c + 1) << 8 | value);
    }
#else
    // For other platforms, use shiftOut to send column index and value byte by byte
    digitalWrite(load, LOW);  // Start data transfer
    for (int i = 0; i < num; i++) {
        if (i == n) {
            shiftOut(data, clock, MSBFIRST, c + 1);  // Send column index byte
            shiftOut(data, clock, MSBFIRST, value);  // Send value byte
        }
    }
    digitalWrite(load, LOW);   // End data transfer
    digitalWrite(load, HIGH);  // Latch data to MAX7219 chips
#endif

    // Update buffer with the new value for the column
    buffer[col] = value;
}

// Set column in all MAX7219 chips in LED matrix display
void Otto_Matrix::setColumnAll(byte col, byte value) {
#if defined(ESP32)
    // For ESP32, use SPI.transfer16 to send column index and value to all chips
    for (int i = 0; i < num; i++) {
        SPI.transfer16((col + 1) << 8 | value);
        buffer[col * i] = value;
    }
#else
    // For other platforms, use shiftOut to send column index and value byte by byte
    digitalWrite(load, LOW);  // Start data transfer
    for (int i = 0; i < num; i++) {
        shiftOut(data, clock, MSBFIRST, col + 1);  // Send column index byte
        shiftOut(data, clock, MSBFIRST, value);    // Send value byte
        buffer[col * i] = value;                   // Update buffer with new value for column
    }
    digitalWrite(load, LOW);   // End data transfer
    digitalWrite(load, HIGH);  // Latch data to MAX7219 chips
#endif
}

// Set value of a single LED (dot) in LED matrix display
void Otto_Matrix::setDot(byte col, byte row, byte value) {
    // Update buffer with new value for the specified LED
    bitWrite(buffer[col], row, value);

    // Determine which MAX7219 chip the column belongs to
    int n = col / 8;
    int c = col % 8;

#if defined(ESP32)
    // For ESP32, use SPI.transfer16 to send column index and buffer value to all chips
    for (int i = 0; i < num; i++) {
        if (i == n)
            SPI.transfer16((c + 1) << 8 | buffer[col]);  // Send column index and buffer value to the corresponding chip
        else
            SPI.transfer16(0);  // Send zeros to other chips
    }
#else
    // For other platforms, use shiftOut to send column index and buffer value byte by byte
    digitalWrite(load, LOW);  // Start data transfer
    for (int i = 0; i < num; i++) {
        if (i == n) {
            shiftOut(data, clock, MSBFIRST, c + 1);      // Send column index byte
            shiftOut(data, clock, MSBFIRST, buffer[col]);  // Send buffer value byte
        } else {
            shiftOut(data, clock, MSBFIRST, 0);   // Send zeros for column index
            shiftOut(data, clock, MSBFIRST, 0);   // Send zeros for buffer value
        }
    }
    digitalWrite(load, LOW);   // End data transfer
    digitalWrite(load, HIGH);  // Latch data to MAX7219 chips
#endif
}

// Write a 5x6 pattern represented by an unsigned long
// value to the LED matrix display
void Otto_Matrix::writeFull(unsigned long value) {
    // Depending on the rotation, set the dots in the LED matrix
    if (rotation == 1) {
        for (int r = 0; r < 5; r++) {
            for (int c = 0; c < 6; c++) {
                setDot(6 - c, 7 - r, (1L & (value >> r * 6 + c)));  // Set dot in reverse order
            }
        }
    }

    if (rotation == 2) {
        for (int r = 0; r < 5; r++) {
            for (int c = 0; c < 6; c++) {
                setDot(1 + c, r, (1L & (value >> r * 6 + c)));  // Rotate pattern 90 degrees clockwise
            }
        }
    }

    if (rotation == 3) {
        for (int r = 0; r < 5; r++) {
            for (int c = 0; c < 6; c++) {
                setDot(r, 6 - c, (1L & (value >> r * 6 + c)));  // Rotate pattern 180 degrees clockwise
            }
        }
    }

    if (rotation == 4) {
        for (int r = 0; r < 5; r++) {
            for (int c = 0; c < 6; c++) {
                setDot(7 - r, 1 + c, (1L & (value >> r * 6 + c)));  // Rotate pattern 270 degrees clockwise
            }
        }
    }
}

/**
 * Display a character on the LED matrix and scroll it if needed.
 * 
 * This function displays a character on the LED matrix at the specified position
 * and scrolls it horizontally if it's part of a sequence of characters. The scrolling
 * speed can be adjusted, and the LED matrix will clear after displaying all characters.
 * 
 * @param data The index of the character in the predefined font array.
 * @param pos The position of the character in the sequence (starting from 0).
 * @param number The total number of characters in the sequence.
 * @param scrollspeed The speed of horizontal scrolling (in milliseconds per step).
 */
void Otto_Matrix::sendChar(const byte data, byte pos, byte number, byte scrollspeed) {
    // Adjust scroll speed within valid range
    if (scrollspeed < 50) 
        scrollspeed = 50;
    if (scrollspeed > 150) 
        scrollspeed = 150;

    // Calculate character position within buffer
    int charPos = pos * 8;

    // Retrieve character data from predefined font and store in CHARbuffer
    CHARbuffer[0 + charPos] = 0;
    for (int i = 0; i < 6; i++) {
        CHARbuffer[i + 1 + charPos] = pgm_read_byte(&Character_font_6x8[data].data[i]);
    }
    CHARbuffer[7 + charPos] = 0;

    // Display character if it's the last one in the sequence
    if (number == (pos + 1)) {
        for (int c = 0; c < 8; c++) {
            byte value = CHARbuffer[c];
            for (int r = 0; r < 8; r++) {
                // Set dot on LED matrix based on rotation
                if (rotation == 1)
                    setDot(c, 7 - r, (0b00000001 & (value >> r)));
                if (rotation == 2)
                    setDot(7 - c, r, (0b00000001 & (value >> r)));
                if (rotation == 3)
                    setDot(r, c, (0b00000001 & (value >> r)));
                if (rotation == 4)
                    setDot(7 - r, 7 - c, (0b00000001 & (value >> r)));
            }
        }
        
        // Display character for a brief duration
        //delay(500);
	    vTaskDelay(max (1U, (500 / portTICK_PERIOD_MS) ));
        
        // Scroll characters to the left
        for (int i = 0; i < ((number * 8) - 1); i++) {
            CHARbuffer[i] = CHARbuffer[i + 1];
            for (int c = 0; c < 8; c++) {
                byte value = CHARbuffer[(1 + c) + i];
                for (int r = 0; r < 8; r++) {
                    // Set dot on LED matrix based on rotation
                    if (rotation == 1)
                        setDot(c, 7 - r, (0b00000001 & (value >> r)));
                    if (rotation == 2)
                        setDot(7 - c, r, (0b00000001 & (value >> r)));
                    if (rotation == 3)
                        setDot(r, c, (0b00000001 & (value >> r)));
                    if (rotation == 4)
                        setDot(7 - r, 7 - c, (0b00000001 & (value >> r)));
                }
            }
            // Apply scroll speed delay
            //delay(scrollspeed);
			vTaskDelay(max (1U, (scrollspeed / portTICK_PERIOD_MS) ));
        }
        
        // Clear LED matrix after displaying all characters
        clearMatrix();
    }
}

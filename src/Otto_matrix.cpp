// OttoDIY Arduino Library project 2024

// MaxMatrix - Copyright 2013 Oscar Kin-Chung Au

#include "Arduino.h"
#include "Otto_matrix.h"

Otto_Matrix::Otto_Matrix() {}

void Otto_Matrix::init(byte _data, byte _load, byte _clock, byte _num, int _rotation) {
    data = _data;
    load = _load;
    clock = _clock;
    num = _num;
    rotation = (_rotation > 4 || _rotation == 0) ? 1 : _rotation;

    for (int i = 0; i < 8; i++)
        buffer[i] = 0;

    for (int i = 0; i < 80; i++)
        CHARbuffer[i] = 0;

#if defined(ESP32)
    SPI.begin(clock, -1, data, load);
    SPI.setDataMode(SPI_MODE0);
    SPI.setClockDivider(SPI_CLOCK_DIV128);
    SPI.setHwCs(true);
#else
    pinMode(data, OUTPUT);
    pinMode(clock, OUTPUT);
    pinMode(load, OUTPUT);
    digitalWrite(clock, HIGH);
#endif

    setCommand(max7219_reg_scanLimit, 0x07);
    setCommand(max7219_reg_decodeMode, 0x00);
    setCommand(max7219_reg_shutdown, 0x01);
    setCommand(max7219_reg_displayTest, 0x00);

    clearMatrix();
    setIntensity(0x0f);
}

void Otto_Matrix::setIntensity(byte intensity) {
    setCommand(max7219_reg_intensity, intensity);
}

void Otto_Matrix::clearMatrix() {
    for (int i = 0; i < 8; i++)
        setColumnAll(i, 0);

    for (int i = 0; i < 8; i++)
        buffer[i] = 0;

    for (int i = 0; i < 80; i++)
        CHARbuffer[i] = 0;
}

void Otto_Matrix::setCommand(byte command, byte value) {
#if defined(ESP32)
    SPI.transfer16(command << 8 | value);
#else
    digitalWrite(load, LOW);
    for (int i = 0; i < num; i++) {
        shiftOut(data, clock, MSBFIRST, command);
        shiftOut(data, clock, MSBFIRST, value);
    }
    digitalWrite(load, LOW);
    digitalWrite(load, HIGH);
#endif
}

void Otto_Matrix::setColumn(byte col, byte value) {
    int n = col / 8;
    int c = col % 8;
#if defined(ESP32)
    for (int i = 0; i < num; i++) {
        if (i == n)
            SPI.transfer16((c + 1) << 8 | value);
    }
#else
    digitalWrite(load, LOW);
    for (int i = 0; i < num; i++) {
        if (i == n) {
            shiftOut(data, clock, MSBFIRST, c + 1);
            shiftOut(data, clock, MSBFIRST, value);
        }
    }
    digitalWrite(load, LOW);
    digitalWrite(load, HIGH);
#endif

    buffer[col] = value;
}

void Otto_Matrix::setColumnAll(byte col, byte value) {
#if defined(ESP32)
    for (int i = 0; i < num; i++) {
        SPI.transfer16((col + 1) << 8 | value);
        buffer[col * i] = value;
    }
#else
    digitalWrite(load, LOW);
    for (int i = 0; i < num; i++) {
        shiftOut(data, clock, MSBFIRST, col + 1);
        shiftOut(data, clock, MSBFIRST, value);
        buffer[col * i] = value;
    }
    digitalWrite(load, LOW);
    digitalWrite(load, HIGH);
#endif
}

void Otto_Matrix::setDot(byte col, byte row, byte value) {
    bitWrite(buffer[col], row, value);

    int n = col / 8;
    int c = col % 8;
#if defined(ESP32)
    for (int i = 0; i < num; i++) {
        if (i == n)
            SPI.transfer16((c + 1) << 8 | buffer[col]);
        else
            SPI.transfer16(0);
    }
#else
    digitalWrite(load, LOW);
    for (int i = 0; i < num; i++) {
        if (i == n) {
            shiftOut(data, clock, MSBFIRST, c + 1);
            shiftOut(data, clock, MSBFIRST, buffer[col]);
        } else {
            shiftOut(data, clock, MSBFIRST, 0);
            shiftOut(data, clock, MSBFIRST, 0);
        }
    }
    digitalWrite(load, LOW);
    digitalWrite(load, HIGH);
#endif
}

void Otto_Matrix::writeFull(unsigned long value) {
    if (rotation == 1) {
        for (int r = 0; r < 5; r++) {
            for (int c = 0; c < 6; c++) {
                setDot(6 - c, 7 - r, (1L & (value >> r * 6 + c)));
            }
        }
    }

    if (rotation == 2) {
        for (int r = 0; r < 5; r++) {
            for (int c = 0; c < 6; c++) {
                setDot(1 + c, r, (1L & (value >> r * 6 + c)));
            }
        }
    }

    if (rotation == 3) {
        for (int r = 0; r < 5; r++) {
            for (int c = 0; c < 6; c++) {
                setDot(r, 6 - c, (1L & (value >> r * 6 + c)));
            }
        }
    }

    if (rotation == 4) {
        for (int r = 0; r < 5; r++) {
            for (int c = 0; c < 6; c++) {
                setDot(7 - r, 1 + c, (1L & (value >> r * 6 + c)));
            }
        }
    }
}

void Otto_Matrix::sendChar(const byte data, byte pos, byte number, byte scrollspeed) {
    if (scrollspeed < 50) scrollspeed = 50;
    if (scrollspeed > 150) scrollspeed = 150;

    int charPos;
    charPos = pos * 8;

    CHARbuffer[0 + charPos] = 0;
    CHARbuffer[1 + charPos] = pgm_read_byte(&Character_font_6x8[data].data[0]);
    CHARbuffer[2 + charPos] = pgm_read_byte(&Character_font_6x8[data].data[1]);
    CHARbuffer[3 + charPos] = pgm_read_byte(&Character_font_6x8[data].data[2]);
    CHARbuffer[4 + charPos] = pgm_read_byte(&Character_font_6x8[data].data[3]);
    CHARbuffer[5 + charPos] = pgm_read_byte(&Character_font_6x8[data].data[4]);
    CHARbuffer[6 + charPos] = pgm_read_byte(&Character_font_6x8[data].data[5]);
    CHARbuffer[7 + charPos] = 0;

    if (number == (pos + 1)) {
        for (int c = 0; c < 8; c++) {
            byte value = CHARbuffer[c];
            for (int r = 0; r < 8; r++) {
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
        delay(500);
        for (int i = 0; i < ((number * 8) - 1); i++) {
            CHARbuffer[i] = CHARbuffer[i + 1];
            for (int c = 0; c < 8; c++) {
                byte value = CHARbuffer[(1 + c) + i];
                for (int r = 0; r < 8; r++) {
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
            delay(scrollspeed);
        }
        clearMatrix();
    }
}

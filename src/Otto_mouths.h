// OttoDIY Arduino Library project 2024

// Zowi (c) BQ. Released under a GPL licencse 04 December 2015

#ifndef Otto_mouths_h
#define Otto_mouths_h

// Mouths sorted by numbers
#define zero              0
#define one               1                
#define two               2               
#define three             3  
#define four              4
#define five              5 
#define six               6 
#define seven             7  
#define eight             8 
#define nine              9
#define smile            10                  
#define happyOpen        11  
#define happyClosed      12  
#define heart            13
#define bigSurprise      14  
#define smallSurprise    15
#define tongueOut        16
#define vamp1            17  
#define vamp2            18  
#define lineMouth        19
#define confused         20  
#define diagonal         21          
#define sad              22
#define sadOpen          23 
#define sadClosed        24
#define okMouth          25 
#define xMouth           26
#define interrogation    27
#define thunder          28
#define culito           29
#define angry            30

// mouth animations
#define littleUuh        0
#define dreamMouth       1
#define adivinawi        2
#define wave             3

typedef struct {
    uint8_t Character[1];
    uint8_t data[6];
} LED_Matrix_Font_6x8_TypeDef;

// Terminal Font
const LED_Matrix_Font_6x8_TypeDef Character_font_6x8[] PROGMEM = {
    {'0', 0x00, 0x7C, 0x82, 0x82, 0x7C, 0x00},
    {'1', 0x00, 0x42, 0xFE, 0x02, 0x00, 0x00},
    {'2', 0x00, 0x46, 0x8A, 0x92, 0x62, 0x00},
    {'3', 0x00, 0x44, 0x92, 0x92, 0x6C, 0x00},
    {'4', 0x00, 0x1C, 0x64, 0xFE, 0x04, 0x00},
    {'5', 0x00, 0xF2, 0x92, 0x92, 0x8C, 0x00},
    {'6', 0x00, 0x7C, 0x92, 0x92, 0x4C, 0x00},
    {'7', 0x00, 0xC0, 0x8E, 0x90, 0xE0, 0x00},
    {'8', 0x00, 0x6C, 0x92, 0x92, 0x6C, 0x00},
    {'9', 0x00, 0x64, 0x92, 0x92, 0x7C, 0x00},
    {':', 0x00, 0x00, 0x14, 0x00, 0x00, 0x00},
    {';', 0x00, 0x02, 0x24, 0x00, 0x00, 0x00},
    {'<', 0x00, 0x10, 0x28, 0x44, 0x82, 0x00},
    {'=', 0x00, 0x28, 0x28, 0x28, 0x28, 0x00},
    {'>', 0x00, 0x82, 0x44, 0x28, 0x10, 0x00},
    {'?', 0x00, 0x20, 0x4a, 0x30, 0x00, 0x00},
    {'@', 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
    {'A', 0x00, 0x7E, 0x88, 0x88, 0x7E, 0x00},
    {'B', 0x00, 0xFE, 0x92, 0x92, 0x6C, 0x00},
    {'C', 0x00, 0x7C, 0x82, 0x82, 0x44, 0x00},
    {'D', 0x00, 0xFE, 0x82, 0x82, 0x7C, 0x00},
    {'E', 0x00, 0xFE, 0x92, 0x92, 0x82, 0x00},
    {'F', 0x00, 0xFE, 0x90, 0x90, 0x80, 0x00},
    {'G', 0x00, 0x7C, 0x82, 0x92, 0x5C, 0x00},
    {'H', 0x00, 0xFE, 0x10, 0x10, 0xFE, 0x00},
    {'I', 0x00, 0x82, 0xFE, 0x82, 0x00, 0x00},
    {'J', 0x00, 0x0C, 0x02, 0x02, 0xFC, 0x00},
    {'K', 0x00, 0xFE, 0x10, 0x28, 0xC6, 0x00},
    {'L', 0x00, 0xFE, 0x02, 0x02, 0x02, 0x00},
    {'M', 0x00, 0xFE, 0x40, 0x30, 0x40, 0xFE},
    {'N', 0x00, 0xFE, 0x40, 0x30, 0x08, 0xFE},
    {'O', 0x00, 0x7C, 0x82, 0x82, 0x82, 0x7C},
    {'P', 0x00, 0xFE, 0x90, 0x90, 0x60, 0x00},
    {'Q', 0x00, 0x7C, 0x82, 0x8A, 0x84, 0x7A},
    {'R', 0x00, 0xFE, 0x98, 0x94, 0x62, 0x00},
    {'S', 0x00, 0x64, 0x92, 0x92, 0x4C, 0x00},
    {'T', 0x00, 0x80, 0xFE, 0x80, 0x80, 0x00},
    {'U', 0x00, 0xFC, 0x02, 0x02, 0xFC, 0x00},
    {'V', 0x00, 0xF0, 0x0C, 0x02, 0x0C, 0xF0},
    {'W', 0x00, 0xFE, 0x04, 0x38, 0x04, 0xFE},
    {'X', 0x00, 0xC6, 0x38, 0x38, 0xC6, 0x00},
    {'Y', 0xC0, 0x20, 0x1E, 0x20, 0xC0, 0x00},
    {'Z', 0x00, 0x86, 0x9A, 0xB2, 0xC2, 0x00},
    {'!', 0x00, 0x00, 0x7a, 0x00, 0x00, 0x00},
    {' ', 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
};

const unsigned long int Gesturetable[4][10] PROGMEM = {
    {0b00000000000000001100001100000000, // littleUuh_code1
     0b00000000000000000110000110000000, // littleUuh_code2
     0b00000000000000000011000011000000, // littleUuh_code3
     0b00000000000000000110000110000000, // littleUuh_code4
     0b00000000000000001100001100000000, // littleUuh_code5
     0b00000000000000011000011000000000, // littleUuh_code6
     0b00000000000000110000110000000000, // littleUuh_code7
     0b00000000000000011000011000000000} // littleUuh_code8
    ,
    {0b00000000000000000000110000110000, // dreamMouth_code1
     0b00000000000000010000101000010000, // dreamMouth_code2
     0b00000000011000100100100100011000, // dreamMouth_code3
     0b00000000000000010000101000010000} // dreamMouth_code4
    ,
    {0b00100001000000000000000000100001, // adivinawi_code1
     0b00010010100001000000100001010010, // adivinawi_code2
     0b00001100010010100001010010001100, // adivinawi_code3
     0b00000000001100010010001100000000, // adivinawi_code4
     0b00000000000000001100000000000000, // adivinawi_code5
     0b00000000000000000000000000000000} // adivinawi_code6
    ,
    {0b00001100010010100001000000000000, // wave_code1
     0b00000110001001010000100000000000, // wave_code2
     0b00000011000100001000010000100000, // wave_code3
     0b00000001000010000100001000110000, // wave_code4
     0b00000000000001000010100100011000, // wave_code5
     0b00000000000000100001010010001100, // wave_code6
     0b00000000100000010000001001000110, // wave_code7
     0b00100000010000001000000100000010, // wave_code8
     0b00110000001000000100000010000001, // wave_code9
     0b00011000100100000010000001000000  // wave_code10
    }
};

const int NUMBER_OF_ELEMENTS = 31;
const unsigned long int Mouthtable[NUMBER_OF_ELEMENTS] PROGMEM = {
  0b00001100010010010010010010001100, // zero_code
  0b00000100001100000100000100001110, // one_code
  0b00001100010010000100001000011110, // two_code
  0b00001100010010000100010010001100, // three_code
  0b00010010010010011110000010000010, // four_code
  0b00011110010000011100000010011100, // five_code
  0b00000100001000011100010010001100, // six_code
  0b00011110000010000100001000010000, // seven_code
  0b00001100010010001100010010001100, // eight_code
  0b00001100010010001110000010001110, // nine_code
  0b00000000100001010010001100000000, // smile_code
  0b00000000111111010010001100000000, // happyOpen_code
  0b00000000111111011110000000000000, // happyClosed_code
  0b00010010101101100001010010001100, // heart_code
  0b00001100010010100001010010001100, // bigSurprise_code
  0b00000000000000001100001100000000, // smallSurprise_code
  0b00111111001001001001000110000000, // tongueOut_code
  0b00111111101101101101010010000000, // vamp1_code
  0b00111111101101010010000000000000, // vamp2_code
  0b00000000000000111111000000000000, // lineMouth_code
  0b00000000001000010101100010000000, // confused_code
  0b00100000010000001000000100000010, // diagonal_code
  0b00000000001100010010100001000000, // sad_code
  0b00000000001100010010111111000000, // sadOpen_code
  0b00000000001100011110110011000000, // sadClosed_code
  0b00000001000010010100001000000000, // okMouth_code
  0b00100001010010001100010010100001, // xMouth_code
  0b00001100010010000100000100000100, // interrogation_code
  0b00000100001000011100001000010000, // thunder_code
  0b00000000100001101101010010000000, // culito_code
  0b00000000011110100001100001000000  // angry_code
};
     
#endif // Otto_mouths_h

// OttoDIY Arduino Library project 2024

// Zowi (c) BQ. Released under a GPL licencse 04 December 2015

#ifndef Otto_mouths_h
#define Otto_mouths_h

// Structure to hold mouth command parameters
struct MouthParam {
	unsigned long int mouth;
	bool clear = false;
	int duration;
};

// Structure to hold mouth intensity parameters
struct MouthintensityParam {
	int intensity;
	int duration;
};

// Structure to hold mouth setled parameters
struct MouthsetledParam {
	byte X;
	byte Y;
	byte value;
	int duration;
};

// Structure to hold mouth write parameters
struct MouthwriteParam {
	const char * string;
	byte speed;
};

// Structure to hold mouth animate parameters
struct MouthanimateParam {
	unsigned long int anim;
	int speed;
	bool loop;
	bool bounce;
};

// Define a Union for mouth commands
union MouthQueueCmd {
    struct MouthParam mouth;				// 1
    struct MouthintensityParam intensity;	// 2
    struct MouthsetledParam setled;			// 3
    struct MouthwriteParam write;			// 4
    struct MouthanimateParam animate;		// 5
	
    MouthQueueCmd() {}  // Default constructor
    ~MouthQueueCmd() {} // Destructor
};

// Define mouth command numbers
#define MOUTH_MOUTH		1
#define MOUTH_INTENSITY	2
#define MOUTH_SETLED	3
#define MOUTH_WRITE		4
#define MOUTH_ANIMATE	5

// Define the wrapper structure for mouth messages
struct MouthQueueMsg {
    int command; // 1 for mouth, 2 for intensity, ...
    union MouthQueueCmd cmd;
};

const unsigned long int Gesturetable[4][10] PROGMEM = {
    {0b00000000000000001100001100000000, // littleUuh_code1
     0b00000000000000000110000110000000, // littleUuh_code2
     0b00000000000000000011000011000000, // littleUuh_code3
     0b00000000000000000110000110000000, // littleUuh_code4
     0b00000000000000001100001100000000, // littleUuh_code5
     0b00000000000000011000011000000000, // littleUuh_code6
     0b00000000000000110000110000000000, // littleUuh_code7
     0b00000000000000011000011000000000, // littleUuh_code8
     0b00000000000000011000011000000000, // littleUuh_code9
     0b00000000000000011000011000000000} // littleUuh_code10
    ,
    {0b00000000000000000000110000110000, // dreamMouth_code1
     0b00000000000000010000101000010000, // dreamMouth_code2
     0b00000000011000100100100100011000, // dreamMouth_code3
     0b00000000000000010000101000010000, // dreamMouth_code4
     0b00000000000000010000101000010000, // dreamMouth_code5
     0b00000000000000010000101000010000, // dreamMouth_code6
     0b00000000000000010000101000010000, // dreamMouth_code7
     0b00000000000000010000101000010000, // dreamMouth_code8
     0b00000000000000010000101000010000, // dreamMouth_code9
     0b00000000000000010000101000010000} // dreamMouth_code10
    ,
    {0b00100001000000000000000000100001, // adivinawi_code1
     0b00010010100001000000100001010010, // adivinawi_code2
     0b00001100010010100001010010001100, // adivinawi_code3
     0b00000000001100010010001100000000, // adivinawi_code4
     0b00000000000000001100000000000000, // adivinawi_code5
     0b00000000000000000000000000000000, // adivinawi_code6
     0b00000000000000000000000000000000, // adivinawi_code7
     0b00000000000000000000000000000000, // adivinawi_code8
     0b00000000000000000000000000000000, // adivinawi_code9
     0b00000000000000000000000000000000} // adivinawi_code10
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

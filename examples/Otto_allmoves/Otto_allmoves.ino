//----------------------------------------------------------------
//-- Otto All moves test
//-- Otto DIY invests time and resources providing open source code and hardware, 
//-- please support by purchasing kits from https://www.ottodiy.com/
//-- Make sure to have installed all libraries: https://github.com/OttoDIY/OttoDIYLib
//-----------------------------------------------------------------
#include <Otto.h>

Otto Otto;  //This is Otto!

#define LeftLeg 2 
#define RightLeg 3
#define LeftFoot 4 
#define RightFoot 5 
#define Buzzer  13 
#define DIN A3 // Data In pin
#define CS A2  // Chip Select pin
#define CLK A1 // Clock pin
#define Orientation 1 // 8x8 LED Matrix orientation  Top  = 1, Bottom = 2, Left = 3, Right = 4 

///////////////////////////////////////////////////////////////////
//-- Setup ------------------------------------------------------//
///////////////////////////////////////////////////////////////////
void setup(){
  Serial.begin(9600);
  Serial.println("Startup");

  Otto.init(LeftLeg, RightLeg, LeftFoot, RightFoot, true, Buzzer); //Set the servo pins and Buzzer pin

/*
  if(Otto.init("ScOtto") != 0) { 
    Serial.println(F("Otto Init Failed"));
  } else {
    Serial.println(F("Otto Init Succeed"));
  }
*/

  
  Otto.initMATRIX( DIN, CS, CLK, Orientation);
  //Otto.sing(S_connection); //Otto wake up!
  
  Otto.home();
  delay(50);
  //Otto.playGesture(OttoHappy);
  Serial.println("Startup done.");
}

///////////////////////////////////////////////////////////////////
//-- Principal Loop ---------------------------------------------//
///////////////////////////////////////////////////////////////////
void loop() { 
  Serial.println("Loop ...");
  //Otto.walk(1,1000,1); //2 steps, "TIME". IF HIGHER THE VALUE THEN SLOWER (from 600 to 1400), 1 FORWARD
  //Otto.walk(2,1000,-1); //2 steps, T, -1 BACKWARD 
  //Otto.turn(2,1000,1);//3 steps turning LEFT
  //Otto._tone(30, 300, 10);
  //Otto.bendTones (100, 200, 1.04, 10, 10);
 // Otto.home();
  //delay(100);  
 // Otto.turn(2,1000,-1);//3 steps turning RIGHT 
 // Otto.bend (1,500,1); //usually steps =1, T=2000
 // Otto.bend (1,2000,-1);     
 // Otto.shakeLeg (1,1500, 1);
 // Otto.home();
  //delay(100);
 // Otto.shakeLeg (1,2000,-1);
  //Otto.moonwalker(3, 1000, 25,1); //LEFT
  //Otto.moonwalker(3, 1000, 25,-1); //RIGHT  
  //Otto.crusaito(2, 1000, 20,1);
 // Otto.crusaito(2, 1000, 20,-1);
 // delay(100); 
 // Otto.flapping(2, 1000, 20,1);
 // Otto.flapping(2, 1000, 20,-1);
 // delay(100);        
 // Otto.swing(2, 1000, 20);
 // Otto.tiptoeSwing(2, 1000, 20);
 // Otto.jitter(2, 1000, 20); //(small T)
 // Otto.updown(2, 1500, 20);  // 20 = H "HEIGHT of movement"T 
 // Otto.ascendingTurn(2, 1000, 50);
 // Otto.jump(1,500); // It doesn't really jumpl ;P
  Otto.home();
  delay(100);
  Serial.println(F("S_connection"));
  Otto.sing(S_connection);
  delay(2500);
  Serial.println(F("S_cuddly"));
  Otto.sing(S_cuddly);
  delay(2500);
  Serial.println(F("S_OhOoh"));
  Otto.sing(S_OhOoh);
  delay(2500);
  Serial.println(F("S_OhOoh2"));
  Otto.sing(S_OhOoh2);
  delay(2500);
  Serial.println(F("S_surprise"));
  Otto.sing(S_surprise);
  delay(2500);
  Serial.println(F("S_buttonPushed"));
  Otto.sing(S_buttonPushed); 
  delay(2500);
  Serial.println(F("S_mode1"));
  Otto.sing(S_mode1);        
  delay(2500);
  Serial.println(F("S_mode2"));
  Otto.sing(S_mode2);
  delay(2500);
  Serial.println(F("S_mode3"));
  Otto.sing(S_mode3);
  delay(2500);
  Serial.println(F("S_sleeping"));
  Otto.sing(S_sleeping);
  delay(2500);
  Serial.println(F("S_fart1"));
  Otto.sing(S_fart1);
  delay(2500);
  Serial.println(F("S_fart2"));
  Otto.sing(S_fart2);
  delay(2500);
  Serial.println(F("S_fart3"));
  Otto.sing(S_fart3);
  delay(2500);
  Serial.println(F("S_happy"));
  Otto.sing(S_happy);
  delay(2500);
  Serial.println(F("S_happy_short"));
  Otto.sing(S_happy_short);            
  delay(2500);
  Serial.println(F("S_superHappy"));
  Otto.sing(S_superHappy);
  delay(2500);
  Serial.println(F("S_sad"));
  Otto.sing(S_sad);
  delay(2500);
  Serial.println(F("S_confused"));
  Otto.sing(S_confused);
  delay(2500);
  Serial.println(F("S_disconnection"));
  Otto.sing(S_disconnection);
 
 // Otto._tone(150, 120, 1);
 // Otto._tone(0, 0, 1200);
 // Otto._tone(150, 120, 1);

  //Otto.bendTones (100, 400, 1.04, 120, 1);

  delay(2500);  
/*
 Otto.playGesture(OttoHappy);
  Otto.playGesture(OttoSuperHappy);
  Otto.playGesture(OttoSad);
  Otto.playGesture(OttoVictory); 
  Otto.playGesture(OttoAngry); 
  Otto.playGesture(OttoSleeping);
  Otto.playGesture(OttoFretful);
  Otto.playGesture(OttoLove);
  Otto.playGesture(OttoConfused);        
  Otto.playGesture(OttoFart);
  Otto.playGesture(OttoWave);
  Otto.playGesture(OttoMagic);
  Otto.playGesture(OttoFail);
  */
  Otto.home();
  delay(100);  
//  Otto.putMouth(zero);
//  delay(100);
//  Otto.putMouth(one);
//  delay(100);
//  Otto.putMouth(two);
//  delay(100);
//  Otto.putMouth(three);
//  delay(100);
//  Otto.putMouth(four);
//  delay(100);
//  Otto.putMouth(five);
//  delay(100);
//  Otto.putMouth(6);
//  delay(100);
// Otto.putMouth(7);
//  delay(100);
//  Otto.putMouth(8);
//  delay(100);
// Otto.putMouth(9);
//  delay(100);
//  Otto.putMouth(smile);
 // delay(100);
 // Otto.putMouth(happyOpen);
 // delay(100);
//Otto.putMouth(happyClosed);
 // delay(100);
//  Otto.putMouth(heart);
//  delay(100);
 // Otto.putMouth(angry);
 // delay(100);
 // Otto.putMouth(smallSurprise);
 // delay(100);
 // Otto.putMouth(bigSurprise);
 // delay(100);
 // Otto.putMouth(tongueOut);
 // delay(100);
 // Otto.putMouth(confused);
//  delay(100);
//  Otto.putMouth(21); //diagonal
//  delay(100);
//  Otto.putMouth(27); //interrogation
 // delay(100);
 // Otto.putMouth(23); //sad open
 // delay(100);
 // Otto.putMouth(24); //sad closed
//  delay(100);
 // Otto.putMouth(vamp1);
 // delay(100);
//  Otto.putMouth(vamp2);
 // delay(100);
//  Otto.putMouth(xMouth);
//  delay(100);
//  Otto.putMouth(okMouth);
//  delay(100);
//  Otto.putMouth(thunder);
//  delay(100);
//  Otto.putMouth(lineMouth);
 // delay(100);
//  Otto.putMouth(culito);
//  delay(100); 
 // Otto.putAnimationMouth(littleUuh,0);
//  delay(1000); 
//  Otto.putAnimationMouth(dreamMouth, 0);
//  delay(1000); 
//  Otto.putAnimationMouth(dreamMouth, 1);
//  delay(1000); 
//  Otto.putAnimationMouth(dreamMouth, 2);
//  delay(1000); 
}

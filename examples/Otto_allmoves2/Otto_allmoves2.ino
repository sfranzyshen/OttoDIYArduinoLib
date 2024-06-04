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
  Otto.home();
  delay(50);
  Serial.println("Startup done.");
}

///////////////////////////////////////////////////////////////////
//-- Principal Loop ---------------------------------------------//
///////////////////////////////////////////////////////////////////
void loop() { 
  Serial.println("Loop ...");
  /*
  Otto.walk(1,1000,1); //2 steps, "TIME". IF HIGHER THE VALUE THEN SLOWER (from 600 to 1400), 1 FORWARD
  Otto.walk(2,1000,-1); //2 steps, T, -1 BACKWARD 
  Otto.turn(2,1000,1);//3 steps turning LEFT
  Otto._tone(30, 300, 10);
  Otto.bendTones (100, 200, 1.04, 10, 10);
  Otto.home();
  delay(100);  
  Otto.turn(2,1000,-1);//3 steps turning RIGHT 
  Otto.bend (1,500,1); //usually steps =1, T=2000
  Otto.bend (1,2000,-1);     
  Otto.shakeLeg (1,1500, 1);
  Otto.home();
  delay(100);
  Otto.shakeLeg (1,2000,-1);
  Otto.moonwalker(3, 1000, 25,1); //LEFT
  Otto.moonwalker(3, 1000, 25,-1); //RIGHT  
  Otto.crusaito(2, 1000, 20,1);
  Otto.crusaito(2, 1000, 20,-1);
  delay(100); 
  Otto.flapping(2, 1000, 20,1);
  Otto.flapping(2, 1000, 20,-1);
  delay(100);        
  Otto.swing(2, 1000, 20);
  Otto.tiptoeSwing(2, 1000, 20);
  Otto.jitter(2, 1000, 20); //(small T)
  Otto.updown(2, 1500, 20);  // 20 = H "HEIGHT of movement"T 
  Otto.ascendingTurn(2, 1000, 50);
  Otto.jump(1,500); // It doesn't really jumpl ;P
*/
  /*
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
*/
  Serial.println(F("OttoHappy"));
  Otto.playGesture(OttoHappy);
  delay(2500);  
  Serial.println(F("OttoSuperHappy"));
  Otto.playGesture(OttoSuperHappy);
  delay(2500);  
  Serial.println(F("OttoSad"));
  Otto.playGesture(OttoSad);
  delay(2500);  
  Serial.println(F("OttoVictory"));
  Otto.playGesture(OttoVictory); 
  delay(2500);  
  Serial.println(F("OttoAngry"));
  Otto.playGesture(OttoAngry); 
  delay(2500);  
  Serial.println(F("OttoSleeping"));
  Otto.playGesture(OttoSleeping);
  delay(2500);  
  Serial.println(F("OttoFretful"));
  Otto.playGesture(OttoFretful);
  delay(2500);  
  Serial.println(F("OttoLove"));
  Otto.playGesture(OttoLove);
  delay(2500);  
  Serial.println(F("OttoConfused"));
  Otto.playGesture(OttoConfused);        
  delay(2500);  
  Serial.println(F("OttoFart"));
  Otto.playGesture(OttoFart);
  delay(2500);  
  Serial.println(F("OttoWave"));
  Otto.playGesture(OttoWave);
  delay(2500);  
  Serial.println(F("OttoMagic"));
  Otto.playGesture(OttoMagic);
  delay(2500);  
  Serial.println(F("OttoFail"));
  Otto.playGesture(OttoFail);
  delay(2500);  
  Serial.println(F("home"));
  Otto.home();
  delay(100);  

/*
  Serial.println(F("writeText"));
  Otto.writeText("SCOTTO", 50);
  delay(5000);
  Serial.println(F("putMouth zero"));
  Otto.putMouth(zero);
  delay(100);
  Serial.println(F("putMouth one"));
  Otto.putMouth(one);
  delay(100);
  Serial.println(F("putMouth two"));
  Otto.putMouth(two);
  delay(100);
  Serial.println(F("putMouth three"));
  Otto.putMouth(three);
  delay(100);
  Serial.println(F("putMouth four"));
  Otto.putMouth(four);
  delay(100);
  Serial.println(F("putMouth five"));
  Otto.putMouth(five);
  delay(100);
  Serial.println(F("putMouth six"));
  Otto.putMouth(6);
  delay(100);
  Serial.println(F("putMouth seven"));
  Otto.putMouth(7);
  delay(100);
  Serial.println(F("putMouth eight"));
  Otto.putMouth(8);
  delay(100);
  Serial.println(F("putMouth nine"));
  Otto.putMouth(9);
  delay(100);
  Serial.println(F("putMouth smile"));
  Otto.putMouth(smile);
  delay(100);
  Serial.println(F("putMouth happyOpen"));
  Otto.putMouth(happyOpen);
  delay(100);
  Serial.println(F("putMouth happyClosed"));
  Otto.putMouth(happyClosed);
  delay(100);
  Serial.println(F("putMouth heart"));
  Otto.putMouth(heart);
  delay(100);
  Serial.println(F("putMouth angry"));
  Otto.putMouth(angry);
  delay(100);
  Serial.println(F("putMouth smallSurprise"));
  Otto.putMouth(smallSurprise);
  delay(100);
  Serial.println(F("putMouth bigSurprise"));
  Otto.putMouth(bigSurprise);
  delay(100);
  Serial.println(F("putMouth tongueOut"));
  Otto.putMouth(tongueOut);
  delay(100);
  Serial.println(F("putMouth confused"));
  Otto.putMouth(confused);
  delay(100);
  Serial.println(F("putMouth diagonal"));
  Otto.putMouth(diagonal); //diagonal
  delay(100);
  Serial.println(F("putMouth interrogation"));
  Otto.putMouth(interrogation); //interrogation
  delay(100);
  Serial.println(F("putMouth sadOpen"));
  Otto.putMouth(sadOpen); //sad open
  delay(100);
  Serial.println(F("putMouth sadClosed"));
  Otto.putMouth(sadClosed); //sad closed
  delay(100);
  Serial.println(F("putMouth vamp1"));
  Otto.putMouth(vamp1);
  delay(100);
  Serial.println(F("putMouth vamp2"));
  Otto.putMouth(vamp2);
  delay(100);
  Serial.println(F("putMouth xMouth"));
  Otto.putMouth(xMouth);
  delay(100);
  Serial.println(F("putMouth okMouth"));
  Otto.putMouth(okMouth);
  delay(100);
  Serial.println(F("putMouth thunder"));
  Otto.putMouth(thunder);
  delay(100);
  Serial.println(F("putMouth lineMouth"));
  Otto.putMouth(lineMouth);
  delay(100);
  Serial.println(F("putMouth culito"));
  Otto.putMouth(culito);
  delay(100);
  
  Serial.println(F("putAnimationMouth littleUuh"));
  for(int index = 0; index < 8; index++) {
    Otto.putAnimationMouth(littleUuh, index);
      delay(100);
  }
  delay(100);
  
  Serial.println(F("putAnimationMouth dreamMouth"));
  for(int index = 0; index < 4; index++) {
    Otto.putAnimationMouth(dreamMouth, index);
      delay(10);
  }
  delay(100);

  Serial.println(F("putAnimationMouth adivinawi"));
  for(int index = 0; index < 6; index++) {
    Otto.putAnimationMouth(adivinawi, index);
      delay(100);
  }
  delay(100);

  Serial.println(F("putAnimationMouth wave"));
  for(int index = 0; index < 10; index++) {
    Otto.putAnimationMouth(wave, index);
      delay(100);
  }
  
  Serial.println(F("Mouth_animate wave"));
  Otto.Mouth_animate(wave, FPS30, true, true);
  delay(5000);
  
  Serial.println(F("Mouth_animate adivinawi"));
  Otto.Mouth_animate(adivinawi, FPS30, true, true);
  delay(5000);
  
  Serial.println(F("Mouth_animate dreamMouth"));
  Otto.Mouth_animate(dreamMouth, FPS30, true, true);
  delay(5000);
  
  Serial.println(F("Mouth_animate littleUuh"));
  Otto.Mouth_animate(littleUuh, FPS30, true, true);
  delay(5000);
  */
}

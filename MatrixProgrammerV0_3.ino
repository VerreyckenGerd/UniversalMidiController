//Oberheim Matrix 1000 midi controller Verreycken Gerd 2023
#include <HC595.h>
const int chipCount = 2;  // Number of serialy connected 74HC595 (8 maximum)
const int latchPin = 40;  // Pin ST_CP (12) of the 74HC595
const int clockPin = 39;  // Pin SH_CP (11) of the 74HC595
const int dataPin = 38;   // Pin DS (14) of the 74HC595
HC595 ledArray(chipCount,latchPin,clockPin,dataPin);

#include <LiquidCrystal.h>
LiquidCrystal lcd(12,11,5,4,3,2);
int antiflicker=255;

#define ENCODER_DO_NOT_USE_INTERRUPTS
#include <Encoder.h>
Encoder myEnc(7, 8);

long oldPosition=2;
bool encoderUP;
bool encoderDOWN;

#include <SD.h>

int patchnumber=0;
int lastbuttonpressed;
int value[144];
int GROUP=1;
int SHIFT=0;

#include <Multiplexer4067.h>
Multiplexer4067 mplex1 = Multiplexer4067(37,36,35,34,A0);
Multiplexer4067 mplex2 = Multiplexer4067(37,36,35,34,A1);

const byte NPots2 = 16;
const byte muxPotPin2[NPots2] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11 ,12 ,13 ,14, 15};
int analog2[NPots2];int previousanalog2[NPots2];

const byte Nbuttons1 = 64;
const byte muxButPin1[Nbuttons1] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15};
int digital1[Nbuttons1];int previousdigital1[Nbuttons1];
uint8_t adres;uint8_t waarde;uint8_t path;uint8_t source;uint8_t dest;uint8_t amount;

byte varThreshold = 4;

bool synth1; bool synth2; 

char potname2[NPots2*8][17] = {
  //sattelite 1 Declare the value names here
 //1234567890123456
  "DCO 1 SHAPE     ","NOT USED        ", //1*
  "DCO1 PULSE WIDTH","NOT USED        ", //2
  "DCO1 LFO2 > PWM ","NOT USED        ", //3
  "DCO 1 FREQUENCY ","NOT USED        ", //4
  "DCO 1LFO 1 MOD  ","NOT USED        ", //5*
  "DCO1 BEND/VIBRAT","NOT USED        ", //6
  "SYNC MODE       ","NOT USED        ", //7
  "DCO 2 DETUNE    ","NOT USED        ", //8
  "DCO 2 SHAPE     ","NOT USED        ", //9*
  "DCO2 PULSE WIDTH","NOT USED        ", //10
  "DCO2 LFO2 > PWM ","NOT USED        ", //11
  "DCO 2 FREQUENCY ","NOT USED        ", //12
  "DCO 2 LFO 1 MOD ","NOT USED        ", //13*
  "DCO2 BEND/VIBRAT","NOT USED        ", //14
  "TRACK GEN SOURCE","NOT USED        ", //15
  "MIX DCO1 - DCO2 ","NOT USED        ", //16

  "LFO 1 SHAPE     ","NOT USED        ", //1b*
  "LFO 1 SAMPLE SRC","NOT USED        ", //2b
  "LFO 1 SPEED     ","NOT USED        ", //3b
  "LFO 1 AT>SPEED  ","NOT USED        ", //4b
  "LFO 1 AMP       ","NOT USED        ", //5b*
  "LFO 1 RAMP1>AMP ","NOT USED        ", //6b
  "LFO 1 RETRIG    ","NOT USED        ", //7b
  "RAMP 1 SPEED    ","RAMP 1 TRIG MODE", //8b
  "LFO 2 SHAPE     ","NOT USED        ", //9b*
  "LFO 2 SAMPLE SRC","NOT USED        ", //10b
  "LFO 2 SPEED     ","NOT USED        ", //11b
  "LFO 2 KEYB>SPEED","NOT USED        ", //12b
  "LFO 2 AMP       ","NOT USED        ", //13b*
  "LFO 2 RAMP2>AMP ","NOT USED        ", //14b
  "LFO 2 RETRIG    ","NOT USED        ", //15b
  "RAMP 2 SPEED    ","RAMP 2 TRIG MODE", //16b

  "FILTER CUTOFF   ","NOT USED        ", //1c*
  "FILTER RESONANCE","NOT USED        ", //2c
  "ENV 1 > CUTOFF  ","NOT USED        ", //3c
  "AFTERT. > CUTOFF","NOT USED        ", //4c
  "FILTER FM       ","NOT USED        ", //5c*
  "ENV 3 > FM      ","NOT USED        ", //6c
  "AFTERT. > FM    ","NOT USED        ", //7c
  "PORTAMENTO RATE ","NOT USED        ", //8c
  "VCA 1 LEVEL     ","NOT USED        ", //9c*
  "VELOCITY > VCA 1","NOT USED        ", //10c
  "ENV 2 > VCA 2   ","NOT USED        ", //11c
  "VEL PORT. RATE  ","NOT USED        ", //12c
  "TRACK GEN. TP1  ","NOT USED        ", //13c*
  "TRACK GEN. TP2  ","NOT USED        ", //14c
  "TRACK GEN. TP3  ","NOT USED        ", //15c
  "TRACK GEN. TP4  ","TRACK GEN. TP5  ", //16c

  "ENV 1 ATTACK    ","ENV 1 DELAY     ", //1d*
  "ENV 1 DECAY     ","ENV 1 VEL > AMP ", //2d
  "ENV 1 SUSTAIN   ","ENV 1 TRIG MODE ", //3d
  "ENV 1 RELEASE   ","1 LFO1 TRIG MODE", //4d
  "ENV 2 ATTACK    ","ENV 1 DELAY     ", //5d*
  "ENV 2 DECAY     ","ENV 1 VEL > AMP ", //6d
  "ENV 2 SUSTAIN   ","ENV 1 TRIG MODE ", //7d
  "ENV 2 RELEASE   ","2 LFO1 TRIG MODE", //8d
  "ENV 3 ATTACK    ","ENV 1 DELAY     ", //9d*
  "ENV 3 DECAY     ","ENV 1 VEL > AMP ", //10d
  "ENV 3 SUSTAIN   ","ENV 1 TRIG MODE ", //11d
  "ENV 3 RELEASE   ","3 LFO1 TRIG MODE", //12d
  "ENV 1 AMPLITUDE ","NOT USED        ", //13d*
  "ENV 2 AMPLITUDE ","NOT USED        ", //14d
  "ENV 3 AMPLITUDE ","NOT USED        ", //15d
  "KEYBOARD MODE   ","NOT USED        ", //16d
  
};

uint8_t pathhex[10] = {
0x00,//1
0x01,//2
0x02,//3
0x03,//4
0x04,//5
0x05,//6
0x06,//7
0x07,//8
0x08,//9
0x09,//10
};

uint8_t sourcehex[21] = {
0x00,//1
0x01,//2
0x02,//3
0x03,//4
0x04,//5
0x05,//6
0x06,//7
0x07,//8
0x08,//9
0x09,//10
0x0a,//11
0x0b,//12
0x0c,//13
0x0d,//14
0x0e,//15
0x0f,//16
0x10,//17
0x11,//18
0x12,//19
0x13,//20
0x14,//21
};

uint8_t desthex[33] = {
0x00,//1
0x01,//2
0x02,//3
0x03,//4
0x04,//5
0x05,//6
0x06,//7
0x07,//8
0x08,//9
0x09,//10
0x0a,//11
0x0b,//12
0x0c,//13
0x0d,//14
0x0e,//15
0x0f,//16
0x10,//17
0x11,//18
0x12,//19
0x13,//20
0x14,//21
0x15,//22
0x16,//23
0x17,//24
0x18,//25
0x19,//26
0x1a,//27
0x1b,//28
0x1c,//29
0x1d,//30
0x1e,//31
0x1f,//32
0x20,//33
};

uint8_t potadres1[NPots2*8] = {
0x05,0x00,//1*  DCO 1 SHAPE     
0x03,0x00,//2   DCO1 PULSE WIDTH
0x04,0x00,//3   DCO1 LFO2 > PWM 
0x00,0x00,//4   DCO 1 FREQUENCY 
0x01,0x00,//5*  DCO 1LFO 1 MOD  
0x00,0x00,//6   DCO1 BEND/VIBRAT te verwijderen/vervangen
0x02,0x00,//7   SYNC MODE       
0x0c,0x00,//8   DCO 2 DETUNE    
0x0f,0x00,//9*  DCO 2 SHAPE     
0x0d,0x00,//10  DCO2 PULSE WIDTH
0x0e,0x00,//11  DCO2 LFO2 > PWM 
0x0a,0x00,//12  DCO 2 FREQUENCY 
0x0b,0x00,//13* DCO 2 LFO 1 MOD 
0x00,0x00,//14  DCO2 BEND/VIBRAT te verwijderen/vervangen
0x00,0x00,//15  TRACK GEN SOURCE
0x14,0x00,//16  MIX DCO1 - DCO2 

0x52,0x00,//1b*  LFO 1 SHAPE     
0x58,0x00,//2b   LFO 1 SAMPLE SRC
0x50,0x00,//3b   LFO 1 SPEED     
0x51,0x00,//4b   LFO 1 AT>SP     
0x54,0x00,//5b*  LFO 1 AMP       
0x55,0x00,//6b   LFO 1 RAMP1>AMP 
0x53,0x00,//7b   LFO 1 RETRIG    
0x28,0x29,//8b   RAMP 1 SPEED    ","RAMP 1 TRIG MODE
0x5c,0x00,//9b*  LFO 2 SHAPE     
0x62,0x00,//10b  LFO 2 SAMPLE SRC
0x5a,0x00,//11b  LFO 2 SPEED     
0x5b,0x00,//12b  LFO 2 KB>SP
0x5e,0x00,//13b* LFO 2 AMP
0x5f,0x00,//14b  LFO 2 RAMP2>AMP
0x5d,0x00,//15b  LFO 2 RETRIG 
0x2a,0x2b,//16b  RAMP 2 SPEED    ","RAMP 2 TRIG MODE

0x15,0x00,//1c*  FILTER CUTOFF
0x18,0x00,//2c   FILTER RESONANCE
0x16,0x00,//3c   ENV 1 > CUTOFF  
0x17,0x00,//4c   AFTERT. > CUTOFF
0x1e,0x00,//5c*  FILTER FM       
0x1f,0x00,//6c   ENV 3 > FM  
0x20,0x00,//7c   AFTERT. > FM    
0x2c,0x00,//8c   PORTAMENTO RATE 
0x1b,0x00,//9c*  VCA 1 LEVEL     
0x1c,0x00,//10c  VELOCITY > VCA 1
0x1d,0x00,//11c  ENV 2 > VCA 2   
0x2d,0x00,//12c  VEL PORT. RATE  
0x22,0x00,//13c* TRACK GEN. TP1  
0x23,0x00,//14c  TRACK GEN. TP2  
0x24,0x00,//15c  TRACK GEN. TP3  
0x25,0x26,//16c  TRACK GEN. TP4  TRACK GEN. TP5  

0x33,0x32,//1d*  ENV 1 ATTACK    ","ENV 1 DELAY     
0x34,0x38,//2d   ENV 1 DECAY     ","ENV 1 VEL > AMP 
0x35,0x39,//3d   ENV 1 SUSTAIN   ","ENV 1 TRIG MODE 
0x36,0x3b,//4d   ENV 1 RELEASE   ","1 LFO1 TRIG MODE
0x3d,0x3c,//5d*  ENV 2 ATTACK    ","ENV 2 DELAY
0x3e,0x42,//6d   ENV 2 DECAY     ","ENV 2 VEL > AMP 
0x3f,0x43,//7d   ENV 2 SUSTAIN   ","ENV 2 TRIG MODE
0x40,0x45,//8d   ENV 2 RELEASE   ","2 LFO1 TRIG MODE
0x47,0x46,//9d*  ENV 3 ATTACK    ","ENV 3 DELAY
0x48,0x4c,//10d  ENV 3 DECAY     ","ENV 3 VEL > AMP 
0x49,0x4d,//11d  ENV 3 SUSTAIN   ","ENV 3 TRIG MODE
0x4a,0x4f,//12d  ENV 3 RELEASE   ","3 LFO1 TRIG MODE
0x37,0x00,//13d* ENV 1 AMPLITUDE 
0x41,0x00,//14d  ENV 2 AMPLITUDE 
0x4b,0x00,//15d  ENV 3 AMPLITUDE 
0x30,0x00,//16d  KEYBOARD MODE
};

int potrange1[NPots2*16] = {
0,63,0,0,//1*  DCO 1 SHAPE     
0,63,0,0,//2   DCO1 PULSE WIDTH
-63,63,0,0,//3   DCO1 LFO2 > PWM 
0,63,0,0,//4   DCO 1 FREQUENCY 
-63,63,0,0,//5*  DCO 1LFO 1 MOD  
0,3,0,0,//6   DCO1 BEND/VIBRAT
0,3,0,0,//7   SYNC MODE       
-31,31,0,0,//8   DCO 2 DETUNE    
0,63,0,0,//9*  DCO 2 SHAPE     
0,63,0,0,//10  DCO2 PULSE WIDTH
-63,63,0,0,//11  DCO2 LFO2 > PWM 
0,63,0,0,//12  DCO 2 FREQUENCY 
-63,63,0,0,//13* DCO 2 LFO 1 MOD 
0,3,0,0,//14  DCO2 BEND/VIBRAT
0,2,0,0,//15  TRACK GEN SOURCE
0,63,0,0,//16  MIX DCO1 - DCO2 

0,6,0,0,//1b*  LFO 1 SHAPE     
0,9,0,0,//2b   LFO 1 SAMPLE SRC
0,63,0,0,//3b   LFO 1 SPEED     
-63,63,0,0,//4b   LFO 1 AT>SP     
0,63,0,0,//5b*  LFO 1 AMP       
-63,63,0,0,//6b   LFO 1 RAMP1>AMP 
0,63,0,0,//7b   LFO 1 RETRIG    
0,63,0,3,//8b   RAMP 1 SPEED    ","RAMP 1 TRIG MODE
0,6,0,0,//9b*  LFO 2 SHAPE     
0,9,0,0,//10b  LFO 2 SAMPLE SRC
0,63,0,0,//11b  LFO 2 SPEED     
-63,63,0,0,//12b  LFO 2 AT>SP
0,63,0,0,//13b* LFO 2 AMP
-63,63,0,0,//14b  LFO 2 RAMP1>AMP
0,63,0,0,//15b  LFO 2 RETRIG 
0,63,0,3,//16b  RAMP 2 SPEED    ","RAMP 2 TRIG MODE

0,127,0,0,//1c*  FILTER CUTOFF
0,63,0,0,//2c   FILTER RESONANCE
-63,63,0,0,//3c   ENV 1 > CUTOFF  
-63,63,0,0,//4c   AFTERT. > CUTOFF
0,63,0,0,//5c*  FILTER FM       
-63,63,0,0,//6c   ENV 3 > FM  
-63,63,0,0,//7c   AFTERT. > FM    
0,63,0,0,//8c   PORTAMENTO RATE 
0,63,0,0,//9c*  VCA 1 LEVEL     
-63,63,0,0,//10c  VELOCITY > VCA 1
-63,63,0,0,//11c  ENV 2 > VCA 2   
-63,63,0,0,//12c  VEL PORT. RATE  
0,63,0,0,//13c* TRACK GEN. TP1  
0,63,0,0,//14c  TRACK GEN. TP2  
0,63,0,0,//15c  TRACK GEN. TP3  
0,63,0,63,//16c  TRACK GEN. TP4  TRACK GEN. TP5  

0,63,0,63,//1d*  ENV 1 ATTACK    ","ENV 1 DELAY     
0,63,-63,63,//2d   ENV 1 DECAY     ","ENV 1 VEL > AMP 
0,63,0,7,//3d   ENV 1 SUSTAIN   ","ENV 1 TRIG MODE 
0,63,0,2,//4d   ENV 1 RELEASE   ","1 LFO1 TRIG MODE
0,63,0,63,//5d*  ENV 2 ATTACK    ","ENV 2 DELAY
0,63,-63,63,//6d   ENV 2 DECAY     ","ENV 2 VEL > AMP 
0,63,0,7,//7d   ENV 2 SUSTAIN   ","ENV 2 TRIG MODE
0,63,0,2,//8d   ENV 2 RELEASE   ","2 LFO1 TRIG MODE
0,63,0,63,//9d*  ENV 3 ATTACK    ","ENV 3 DELAY
0,63,-63,63,//10d  ENV 3 DECAY     ","ENV 3 VEL > AMP 
0,63,0,7,//11d  ENV 3 SUSTAIN   ","ENV 3 TRIG MODE
0,63,0,2,//12d  ENV 3 RELEASE   ","3 LFO1 TRIG MODE
0,63,0,0,//13d* ENV 1 AMPLITUDE 
0,63,0,0,//14d  ENV 2 AMPLITUDE 
0,63,0,0,//15d  ENV 3 AMPLITUDE 
0,3,0,0,//16d  KEYBOARD MODE
};

char buttonname1[Nbuttons1][17] = {
  //sattelite 1 Declare the value names here
 //1234567890123456
  "DCO 1 OFF       ", //1
  "DCO 1 PULSE     ", //2
  "DCO 1 WAVE      ", //3
  "DCO 1 BOTH      ", //4
  "DCO 2 OFF       ", //5
  "DCO 2 PULSE     ", //6
  "DCO 2 WAVE      ", //7
  "DCO 2 BOTH      ", //8
  "DCO 2 NOISE     ", //9
  "DCO 1 PORTAMENTO", //10
  "DCO 2 PORTAMENTO", //11
  "DCO 2 KEY TRACK ", //12
  "DCO 1 & 2       ", //13
  "LFO & RAMP      ", //14
  "FILTER & VCA    ", //15
  "ENVELOPPES      ", //16
  "LFO 1 LAG       ", //1b
  "LFO 2 LAG       ", //2b
  "DCO 1 CLICK     ", //3b
  "DCO 2 CLICK     ", //4b
  "LFO 1 TRIG OFF  ", //5b
  "LFO 1 TRIG SINGL", //6b
  "LFO 1 TRIG MULTI", //7b
  "LFO 1 TRIG PED 2", //8b
  "LFO 2 TRIG OFF  ", //9b
  "LFO 2 TRIG SINGL", //10b
  "LFO 2 TRIG MULTI", //11b
  "LFO 2 TRIG PED 2", //12b
  "DCO 1 & 2       ", //13b
  "LFO & RAMP      ", //14b
  "FILTER & VCA    ", //15b
  "ENVELOPPES      ", //16b
  "MOD OFF         ", //1c
  "MOD PITCH BEND  ", //2c
  "MOD VIBRATO     ", //3c
  "MOD BOTH        ", //4c
  "NONE            ", //5c
  "PORTAMENTO      ", //6c
  "KB TRACK        ", //7c
  "NOT USED        ", //8c
  "PORT MODE LINEAR", //9c
  "PORT MODE CONST.", //10c
  "PORT MODE EXPO. ", //11c
  "PORT MODE LEGATO", //12c
  "DCO 1 & 2       ", //13c
  "LFO & RAMP      ", //14c
  "FILTER & VCA    ", //15c
  "ENVELOPPES      ", //16c
  "ENV 1 NORMAL    ", //1d
  "ENV 1 DADR  MODE", //2d
  "ENV 1 FREERUN   ", //3d
  "ENV 1 BOTH  MODE", //4d
  "ENV 2 NORMAL    ", //5d
  "ENV 2 DADR  MODE", //6d
  "ENV 2 FREERUN   ", //7d
  "ENV 2 BOTH  MODE", //8d
  "ENV 3 NORMAL    ", //9d
  "ENV 3 DADR  MODE", //10d
  "ENV 3 FREERUN   ", //11d
  "ENV 3 BOTH  MODE", //12d
  "DCO 1 & 2       ", //13d
  "LFO & RAMP      ", //14d
  "FILTER & VCA    ", //15d
  "ENVELOPPES      ", //16d
};

char sourcename[21][17] = {
"UNUSED          ",//0
"ENVELOPPE 1     ",//1
"ENVELOPPE 2     ",//2
"ENVELOPPE 3     ",//3
"LFO 1           ",//4
"LFO 2           ",//5
"Vibrato         ",//6
"Ramp 1          ",//7
"Ramp 2          ",//8
"Keyboard        ",//9
"Portamonto      ",//10
"Tracking Gen.   ",//11
"Keyboard Gate   ",//12
"Velocity        ",//13
"Release Velocity",//14
"Aftertouch      ",//15
"Pedal 1         ",//16
"Pedal 2         ",//17
"Lever 1         ",//18
"Lever 2         ",//19
"Lever 3         ",//20
};

char destinationname[33][17] = {
"UNUSED          ",//0
"DCO 1 FREQUENCY ",//1
"DCO1 PULSE WIDTH",//2
"DCO 1 WAVE SHAPE",//3
"DCO 2 FREQUENCY ",//4
"DCO2 PULSE WIDTH",//5
"DCO 2 WAVE SHAPE",//6
"DCO 1 / 2 MIX   ",//7
"VCF FM AMOUNT   ",//8
"VCF CUTOFF FREQ.",//9
"VCF RESONANC    ",//10
"VCA 1 LEVEL     ",//11
"VCA 1 LEVEL     ",//12
"ENV 1 DELAY     ",//13
"ENV 1 ATTACK    ",//14
"ENV 1 DECAY     ",//15
"ENV 1 RELEASE   ",//16
"ENV 1 AMPLITUDE ",//17
"ENV 2 DELAY     ",//18
"ENV 2 ATTACK    ",//19
"ENV 2 DECAY     ",//20
"ENV 2 RELEASE   ",//21
"ENV 2 AMPLITUDE ",//22
"ENV 3 DELAY     ",//23
"ENV 3 ATTACK    ",//24
"ENV 3 DECAY     ",//25
"ENV 3 RELEASE   ",//26
"ENV 3 AMPLITUDE ",//27
"LFO 1 SPEED     ",//28
"LFO 1 AMPLITUDE ",//29
"LFO 2 SPEED     ",//30
"LFO 2 AMPLITUDE ",//31
"PORTAMENTO RATE ",//32
};


int CCnumber2[NPots2]= {17,18,19,20};

int CCnumber5[Nbuttons1]= {5,6,7,8,9,10,11,12,13,14};

int index;int kladindex; int kladwaarde;
int matrix=1;

void setup() {
  
  pinMode(6, INPUT);
  pinMode(7, INPUT);
  pinMode(8, INPUT);

  pinMode(22, INPUT);//MOD MATRIX SOURCE
  pinMode(23, INPUT);//MOD MATRIX AMOUNT
  pinMode(24, INPUT);//MOD MATRIX DESTINATION
  pinMode(25, INPUT);//SYNTH 1
  pinMode(26, INPUT);//SYNTH 2
  pinMode(27, INPUT);//SHIFT
  pinMode(28, INPUT);//MOD MATRIX BUS
  pinMode(29, INPUT);//MENU

  pinMode(42, OUTPUT);
  pinMode(43, OUTPUT);//MOD MATRIX BUS
  pinMode(44, OUTPUT);//SHIFT
  pinMode(45, OUTPUT);//SYNTH 2
  pinMode(46, OUTPUT);//SYNTH 1
  pinMode(47, OUTPUT);//MOD MATRIX DESTINATION
  pinMode(48, OUTPUT);//MOD MATRIX AMOUNT
  pinMode(49, OUTPUT);//MOD MATRIX SOURCE

lcd.begin(16, 2);

lcd.print("Matrix Prog V0.0");delay(1000);

mplex1.begin();mplex2.begin();
Serial1.begin(31250);
Serial2.begin(31250);
GROUP=1;ledupdate();ledArray.setPin(muxButPin1[12], ON);

for (int i = 0; i < 10; i++) {
if (value[118+i]==0){value[118+i]=65;}}  

for (int i = 0; i < NPots2; i++) {
analog2[i]=mplex2.readChannel(muxPotPin2[i]);
previousanalog2[i]=analog2[i];
}

}



void loop() { 

if (digitalRead(27)==true&&SHIFT==0){SHIFT=1;digitalWrite(44,HIGH);delay(300);}
if (digitalRead(27)==true&&SHIFT==1){SHIFT=0;digitalWrite(44,LOW);delay(300);}
//SYNTH SELECTION START//  
if (digitalRead(25)==HIGH&&synth1==false){digitalWrite(46,HIGH);synth1=true;delay(300);}//WHEN SYNTH 1 PRESSED
if (digitalRead(25)==HIGH&&synth1==true){digitalWrite(46,LOW);synth1=false;delay(300);}
if (digitalRead(26)==HIGH&&synth2==false){digitalWrite(45,HIGH);synth2=true;delay(300);}//WHEN SYNTH 2 PRESSED
if (digitalRead(26)==HIGH&&synth2==true){digitalWrite(45,LOW);synth2=false;delay(300);}
//SYNTH SELECTION STOP//  

digitalWrite(43,LOW);

//BEGIN EDIT MOD MATRIX BUS
if (digitalRead(28)==HIGH){//WHEN MOD BUS IS PRESSED
digitalWrite(43,HIGH);//MOD BUS led on
lcd.setCursor(0, 0);lcd.print("                ");lcd.setCursor(0, 0);lcd.print("MODBUS:");lcd.setCursor(0, 1);lcd.print("                ");lcd.setCursor(0, 1);lcd.print(matrix);
while(digitalRead(6)==LOW){//when encoder not pressed
checkEncoder();
if (encoderUP==true&&matrix<10){matrix++;lcd.setCursor(0, 1);lcd.print("                ");lcd.setCursor(0, 1);lcd.print(matrix);}
if (encoderDOWN==true&&matrix>1){matrix--;lcd.setCursor(0, 1);lcd.print("                ");lcd.setCursor(0, 1);lcd.print(matrix);}  
}
digitalWrite(43,LOW);//MOD BUS led off on exit
}
//END EDIT MOD MATRIX BUS

//BEGIN EDIT MOD MATRIX SOURCE
if (digitalRead(22)==HIGH){//WHEN MOD SOURCE IS PRESSED
digitalWrite(49,HIGH);//MOD SOURCE led on
lcd.setCursor(0, 0);lcd.print("                ");lcd.setCursor(0, 0);lcd.print("SOURCE BUS: ");lcd.setCursor(12, 0);lcd.print("  ");lcd.setCursor(12, 0);lcd.print(matrix);lcd.setCursor(0, 1);lcd.print("                ");lcd.setCursor(0, 1);lcd.print(sourcename[value[98+matrix]]);
while(digitalRead(6)==LOW){//when encoder not pressed
checkEncoder();

if (encoderUP==true&&value[98+matrix]<9){value[98+matrix]++;lcd.setCursor(0, 1);lcd.print("                ");lcd.setCursor(0, 1);lcd.print(sourcename[value[98+matrix]]);
path=pathhex[matrix-1];
source=sourcehex[value[98+matrix]];
dest=desthex[value[108+matrix]];
int tijdelijk=map(value[118+matrix],1,128,-63,63);
if (tijdelijk<0){tijdelijk=map(tijdelijk,-64,-1,64,127);}
amount=tijdelijk;
writeMatrix();
}

if (encoderDOWN==true&&value[98+matrix]>0){value[98+matrix]--;lcd.setCursor(0, 1);lcd.print("                ");lcd.setCursor(0, 1);lcd.print(sourcename[value[98+matrix]]);
path=pathhex[matrix-1];
source=sourcehex[value[98+matrix]];
dest=desthex[value[108+matrix]];
int tijdelijk=map(value[118+matrix],1,128,-63,63);
if (tijdelijk<0){tijdelijk=map(tijdelijk,-64,-1,64,127);}
amount=tijdelijk;
writeMatrix();
}  
}
digitalWrite(49,LOW);//MOD SOURCE led off on exit
}
//END EDIT MOD MATRIX SOURCE

//BEGIN EDIT MOD MATRIX DESTINATION
if (digitalRead(24)==HIGH){//WHEN MOD DESTINATION IS PRESSED
digitalWrite(47,HIGH);//MOD DESTINATION led on
lcd.setCursor(0, 0);lcd.print("                ");lcd.setCursor(0, 0);lcd.print("DEST.  BUS: ");lcd.setCursor(12, 0);lcd.print("  ");lcd.setCursor(12, 0);lcd.print(matrix);lcd.setCursor(0, 1);lcd.print("                ");lcd.setCursor(0, 1);lcd.print(destinationname[value[108+matrix]]);
while(digitalRead(6)==LOW){//when encoder not pressed
checkEncoder();
if (encoderUP==true&&value[108+matrix]<32){value[108+matrix]++;lcd.setCursor(0, 1);lcd.print("                ");lcd.setCursor(0, 1);lcd.print(destinationname[value[108+matrix]]);
path=pathhex[matrix-1];
source=sourcehex[value[98+matrix]];
dest=desthex[value[108+matrix]];
int tijdelijk=map(value[118+matrix],1,128,-63,63);
if (tijdelijk<0){tijdelijk=map(tijdelijk,-64,-1,64,127);}
amount=tijdelijk;
writeMatrix();
}
if (encoderDOWN==true&&value[108+matrix]>0){value[108+matrix]--;lcd.setCursor(0, 1);lcd.print("                ");lcd.setCursor(0, 1);lcd.print(destinationname[value[108+matrix]]);
path=pathhex[matrix-1];
source=sourcehex[value[98+matrix]];
dest=desthex[value[108+matrix]];
int tijdelijk=map(value[118+matrix],1,128,-63,63);
if (tijdelijk<0){tijdelijk=map(tijdelijk,-64,-1,64,127);}
amount=tijdelijk;
writeMatrix();
}  
}
digitalWrite(47,LOW);//MOD DESTINATION led off on exit
}
//END EDIT MOD MATRIX DESTINATION

//BEGIN EDIT MOD MATRIX AMOUNT
if (digitalRead(23)==HIGH){//WHEN MOD AMOUNT IS PRESSED
digitalWrite(48,HIGH);//MOD AMOUNT led on
lcd.setCursor(0, 0);lcd.print("                ");lcd.setCursor(0, 0);lcd.print("AMOUNT BUS: ");lcd.setCursor(12, 0);lcd.print("  ");lcd.setCursor(12, 0);lcd.print(matrix);lcd.setCursor(0, 1);lcd.print("                ");lcd.setCursor(0, 1);lcd.print(map(value[118+matrix],1,128,-63,63));
while(digitalRead(6)==LOW){//when encoder not pressed
checkEncoder();
if (encoderUP==true&&value[118+matrix]<128){  
value[118+matrix]++;lcd.setCursor(0, 1);lcd.print("                ");lcd.setCursor(0, 1);lcd.print(map(value[118+matrix],1,128,-63,63));
path=pathhex[matrix-1];
source=sourcehex[value[98+matrix]];
dest=desthex[value[108+matrix]];
int tijdelijk=map(value[118+matrix],1,128,-63,63);
if (tijdelijk<0){tijdelijk=map(tijdelijk,-64,-1,64,127);}
amount=tijdelijk;
writeMatrix();
}
if (encoderDOWN==true&&value[118+matrix]>1){
value[118+matrix]--;lcd.setCursor(0, 1);lcd.print("                ");lcd.setCursor(0, 1);lcd.print(map(value[118+matrix],1,128,-63,63));
path=pathhex[matrix-1];
source=sourcehex[value[98+matrix]];
dest=desthex[value[108+matrix]];
int tijdelijk=map(value[118+matrix],1,128,-63,63);
if (tijdelijk<0){tijdelijk=map(tijdelijk,-64,-1,64,127);}
amount=tijdelijk;
writeMatrix();
}
}
digitalWrite(48,LOW);//MOD AMOUNT led off on exit
}
//END EDIT MOD MATRIX AMOUNT

/*SAVE PATCH TO UNIT BEGIN
if (digitalRead(26)==HIGH&&digitalRead(27)==HIGH){//if save pressed
digitalWrite(45,HIGH);//save led on
                            //0123456789012345 
lcd.setCursor(0, 0);lcd.print("SAVE to bank:  "); 
int storebank=0;lcd.setCursor(14, 0);lcd.print(storebank);
lcd.setCursor(0, 1);lcd.print("SAVE to prog:  "); 
int storelocation=0;lcd.setCursor(14, 1);lcd.print(storelocation);

while(digitalRead(6)==LOW){//when encoder not pressed
checkEncoder();
if (encoderDOWN==true&&storebank>0){storebank--;lcd.setCursor(14, 0);lcd.print("  ");lcd.setCursor(14, 0);lcd.print(storebank);}
if (encoderUP==true&&storebank<1){storebank++;lcd.setCursor(14, 0);lcd.print("  ");lcd.setCursor(14, 0);lcd.print(storebank);}
}
delay(500);
while(digitalRead(6)==LOW){//when encoder not pressed
checkEncoder();
if (encoderDOWN==true&&storelocation>0){storelocation--;lcd.setCursor(14, 1);lcd.print("  ");lcd.setCursor(14, 1);lcd.print(storelocation);}
if (encoderUP==true&&storelocation<99){storelocation++;lcd.setCursor(14, 1);lcd.print("  ");lcd.setCursor(14, 1);lcd.print(storelocation);}
}
uint8_t bank=storebank;
uint8_t location=storelocation;
Serial1.write(0xF0); 
Serial1.write(0x10);
Serial1.write(0x06); 
Serial1.write(0x0e);
Serial1.write(location);
Serial1.write(bank);
Serial1.write(0x7f);
Serial1.write(0x7f);
digitalWrite(45,LOW);//save led off
lcd.setCursor(0, 0);lcd.print("Patch saved to: ");
lcd.setCursor(0, 1);lcd.print("Matrix 1000: ");lcd.print(storebank);if(storelocation<10){lcd.print("0");}lcd.print(storelocation);
}
*///SAVE PATCH TO UNIT END

/*SAVE PATCH TO SD CARD BEGIN
if (digitalRead(26)==HIGH&&digitalRead(27)==LOW){//if save pressed
digitalWrite(45,HIGH);//save led on
while (!SD.begin(53)) {lcd.setCursor(0, 0);lcd.print(" insert SD card ");}  //check if sd card is detected and report on lcd if not
                            
                   
                            
                             //1234567890123456
lcd.setCursor(0, 0);lcd.print("SAVE to:        ");
lcd.setCursor(9, 0);lcd.print(patchnumber+1);lcd.print("             ");DisplayPatchName();checkfile();
while(digitalRead(6)==LOW){//when encoder not pressed

if (digitalRead(28)==HIGH){digitalWrite(43,HIGH);delay(50);break;}  //cancel routine
checkEncoder();
if (encoderDOWN==true&&patchnumber>0){patchnumber--;lcd.setCursor(9, 0);lcd.print("   ");lcd.setCursor(9, 0);lcd.print(patchnumber+1);DisplayPatchName();checkfile();}
if (encoderUP==true&&patchnumber<999){patchnumber++;lcd.setCursor(9, 0);lcd.print("   ");lcd.setCursor(9, 0);lcd.print(patchnumber+1);DisplayPatchName();checkfile();}
                          }
//START EDITING PATCH NAME
String filename = "";if (patchnumber < 10) {filename += "00";} else if (patchnumber < 100) {filename += "0";}filename += String(patchnumber) + ".txt";
File myFile = SD.open(filename);
myFile.seek(128);
int rangeSize = 16;
char buffer[rangeSize + 1];
myFile.readBytes(buffer, rangeSize);
buffer[rangeSize] = '\0';
char editname[17];
strcpy(editname, buffer);
myFile.close();
int cursorposition=0;       //"                "
lcd.setCursor(0, 0);lcd.print("V               ");
while(digitalRead(26)==LOW){//while save not pressed
checkEncoder();
if (digitalRead(6)==HIGH&&encoderDOWN==true&&cursorposition>0){cursorposition--;lcd.setCursor(0, 0);lcd.print("                ");lcd.setCursor(cursorposition, 0);lcd.print("V");}
if (digitalRead(6)==HIGH&&encoderUP==true&&cursorposition<15){cursorposition++;lcd.setCursor(0, 0);lcd.print("                ");lcd.setCursor(cursorposition, 0);lcd.print("V");}
if (digitalRead(6)==LOW&&encoderDOWN==true){editname[cursorposition]--;
while (!(isAlphaNumeric(editname[cursorposition]) || editname[cursorposition] == ' ')){editname[cursorposition]--;}
lcd.setCursor(cursorposition, 1);lcd.print(editname[cursorposition]);}
if (digitalRead(6)==LOW&&encoderUP==true){editname[cursorposition]++;
while (!(isAlphaNumeric(editname[cursorposition]) || editname[cursorposition] == ' ')){editname[cursorposition]++;}
lcd.setCursor(cursorposition, 1);lcd.print(editname[cursorposition]);}
}
for (int i = 0; i < 16; i++) {
value[i+128]=editname[i];}  


//END EDITING PATCH NAME
filename = "";
if (patchnumber < 10) {
  filename += "00";
} else if (patchnumber < 100) {
  filename += "0";
}
filename += String(patchnumber) + ".txt"; 

if (SD.exists(filename)) { // check if file exists
    File dataFile = SD.open(filename, FILE_WRITE); // open file in write mode
    SD.remove(filename); // delete file
    dataFile.close();
                                 //1234567890123456
    lcd.setCursor(0, 2);lcd.print("Old file deleted");delay(1000);
    

                         }
File dataFile = SD.open(filename, FILE_WRITE);
  for (int i = 0; i < 128; i++) {
  char schmier = value[i];
  dataFile.print(schmier); 
  }
  for (int i = 128; i < 144; i++) {
  char schmier = value[i];
  dataFile.print(schmier); 
  }
  dataFile.close();
                               //1234567890123456
  lcd.setCursor(0, 2);lcd.print("  Patch saved!  ");delay(1000);
  lcd.clear();
  DisplayPatchName();
                          }
*/ //END SAVE PATCH TO SD CARD



/*BEGIN LOAD PATCH
if (digitalRead(25)==HIGH){//if load pressed
digitalWrite(46,HIGH);//load led on
while (!SD.begin(53)) {lcd.setCursor(0, 0);lcd.print(" insert SD card ");}  //check if sd card is detected and report on lcd if not
                            //"       :        ""
lcd.setCursor(0, 0);lcd.print("Load Patch:      ");
lcd.setCursor(12, 0);lcd.print(patchnumber+1);lcd.print("             ");DisplayPatchName();checkfile();
while(digitalRead(6)==LOW){//when encoder not pressed
if (digitalRead(28)==HIGH){digitalWrite(43,HIGH);delay(50);break;}  //cancel routine
checkEncoder();
if (encoderDOWN==true&&patchnumber>0){patchnumber--;lcd.setCursor(12, 0);lcd.print("   ");lcd.setCursor(12, 0);lcd.print(patchnumber+1);DisplayPatchName();checkfile();loadpatch();}
if (encoderUP==true&&patchnumber<999){patchnumber++;lcd.setCursor(12, 0);lcd.print("   ");lcd.setCursor(12, 0);lcd.print(patchnumber+1);DisplayPatchName();checkfile();loadpatch();}
}
digitalWrite(46,LOW);//load led off
  lcd.clear();
  DisplayPatchName();
}
*///END LOAD PATCH


//begin readig pots2

for (int i = 0; i < NPots2; i++) {
analog2[i]=mplex2.readChannel(muxPotPin2[i]);
int potVar = abs(analog2[i] - previousanalog2[i]);
if (potVar >= varThreshold){

if(antiflicker!=(i*2)+SHIFT){lcd.setCursor(0,0);lcd.print("                ");lcd.setCursor(0,0);lcd.print(potname2[(i*2)+((GROUP-1)*32)+SHIFT]);}
antiflicker=(i*2)+SHIFT;

adres=potadres1[(i*2)+((GROUP-1)*32)+SHIFT];

int tijdelijk=map(analog2[i],0,1023,potrange1[(i*4)+((GROUP-1)*64)+(SHIFT*2)],potrange1[(i*4)+((GROUP-1)*64)+1+(SHIFT*2)]);

lcd.setCursor(0,1);lcd.print("                ");lcd.setCursor(0,1);lcd.print(tijdelijk);

value[potadres1[i*2]]=tijdelijk;//WRITE POT VALUES TO value[]

if (tijdelijk<0){tijdelijk=map(tijdelijk,-64,-1,64,127);}
waarde=tijdelijk;

writeMidibuttons1();

kladindex=i+((GROUP-1)*16);kladwaarde=waarde;displayChoice();

}
previousanalog2[i]=analog2[i];
}
 //end readig pots2



//begin reading the buttons 1
for (int i = 0; i < Nbuttons1; i++) {index=i;
digital1[i]=mplex1.readChannel(muxButPin1[i]);
if(digital1[i]>63){
if(i==12){GROUP=1;ledupdate();lcd.setCursor(0,0);lcd.print("                ");lcd.setCursor(0,0);lcd.print(buttonname1[i]);lcd.setCursor(0,1);lcd.print("                ");lcd.setCursor(0,1);lcd.print("GROUP SELECTED");ledArray.setPin(muxButPin1[12], ON);ledArray.setPin(muxButPin1[13], OFF);ledArray.setPin(muxButPin1[14], OFF);ledArray.setPin(muxButPin1[15], OFF);}  
if(i==13){GROUP=2;ledupdate();lcd.setCursor(0,0);lcd.print("                ");lcd.setCursor(0,0);lcd.print(buttonname1[i]);lcd.setCursor(0,1);lcd.print("                ");lcd.setCursor(0,1);lcd.print("GROUP SELECTED");ledArray.setPin(muxButPin1[12], OFF);ledArray.setPin(muxButPin1[13], ON);ledArray.setPin(muxButPin1[14], OFF);ledArray.setPin(muxButPin1[15], OFF);}  
if(i==14){GROUP=3;ledupdate();lcd.setCursor(0,0);lcd.print("                ");lcd.setCursor(0,0);lcd.print(buttonname1[i]);lcd.setCursor(0,1);lcd.print("                ");lcd.setCursor(0,1);lcd.print("GROUP SELECTED");ledArray.setPin(muxButPin1[12], OFF);ledArray.setPin(muxButPin1[13], OFF);ledArray.setPin(muxButPin1[14], ON);ledArray.setPin(muxButPin1[15], OFF);}  
if(i==15){GROUP=4;ledupdate();lcd.setCursor(0,0);lcd.print("                ");lcd.setCursor(0,0);lcd.print(buttonname1[i]);lcd.setCursor(0,1);lcd.print("                ");lcd.setCursor(0,1);lcd.print("GROUP SELECTED");ledArray.setPin(muxButPin1[12], OFF);ledArray.setPin(muxButPin1[13], OFF);ledArray.setPin(muxButPin1[14], OFF);ledArray.setPin(muxButPin1[15], ON);}  
if(i==0){switch(GROUP){case 1:lcd.setCursor(0,0);lcd.print("                ");lcd.setCursor(0,0);lcd.print(buttonname1[i]);lcd.setCursor(0,1);lcd.print("                ");lcd.setCursor(0,1);lcd.print(" ");
                       adres=0x06;waarde=0x00;writeMidibuttons1();value[6]=0;ledupdate();delay(300);break;
                       case 2:lcd.setCursor(0,0);lcd.print("                ");lcd.setCursor(0,0);lcd.print(buttonname1[i+16]);lcd.setCursor(0,1);lcd.print("                ");lcd.setCursor(0,1);lcd.print(" ");
                       adres=0x57;
                       if(value[87]==0){waarde=0x01;writeMidibuttons1();value[87]=1;ledupdate();delay(300);}
                       else{waarde=0x00;writeMidibuttons1();value[87]=0;ledupdate();delay(300);}
                       break;
                       case 3:lcd.setCursor(0,0);lcd.print("                ");lcd.setCursor(0,0);lcd.print(buttonname1[i+32]);lcd.setCursor(0,1);lcd.print("                ");lcd.setCursor(0,1);lcd.print(" ");
                       adres=0x19;
                       waarde=0x00;writeMidibuttons1();value[25]=0;ledupdate();delay(300);
                       break;
                       case 4:lcd.setCursor(0,0);lcd.print("                ");lcd.setCursor(0,0);lcd.print(buttonname1[i+48]);lcd.setCursor(0,1);lcd.print("                ");lcd.setCursor(0,1);lcd.print(" ");
                       adres=0x3A;
                       waarde=0x00;writeMidibuttons1();value[58]=0;ledupdate();delay(300);
                       break;
                       default:break;
                      }
        }
if(i==1){switch(GROUP){case 1:lcd.setCursor(0,0);lcd.print("                ");lcd.setCursor(0,0);lcd.print(buttonname1[i]);lcd.setCursor(0,1);lcd.print("                ");lcd.setCursor(0,1);lcd.print(" ");
                       adres=0x06;waarde=0x01;writeMidibuttons1();value[6]=1;ledupdate();delay(300);break;
                       case 2:lcd.setCursor(0,0);lcd.print("                ");lcd.setCursor(0,0);lcd.print(buttonname1[i+16]);lcd.setCursor(0,1);lcd.print("                ");lcd.setCursor(0,1);lcd.print(" ");
                       adres=0x61;
                       if(value[97]==0){waarde=0x01;writeMidibuttons1();value[97]=1;ledupdate();delay(300);}
                       else{waarde=0x00;writeMidibuttons1();value[97]=0;ledupdate();delay(300);}
                       break;
                       case 3:lcd.setCursor(0,0);lcd.print("                ");lcd.setCursor(0,0);lcd.print(buttonname1[i+32]);lcd.setCursor(0,1);lcd.print("                ");lcd.setCursor(0,1);lcd.print(" ");
                       adres=0x19;
                       waarde=0x01;writeMidibuttons1();value[25]=1;ledupdate();delay(300);
                       break;
                       case 4:lcd.setCursor(0,0);lcd.print("                ");lcd.setCursor(0,0);lcd.print(buttonname1[i+48]);lcd.setCursor(0,1);lcd.print("                ");lcd.setCursor(0,1);lcd.print(" ");
                       adres=0x3A;
                       waarde=0x01;writeMidibuttons1();value[58]=1;ledupdate();delay(300);
                       break;
                       default:break;
                      }
        }
if(i==2){switch(GROUP){case 1:lcd.setCursor(0,0);lcd.print("                ");lcd.setCursor(0,0);lcd.print(buttonname1[i]);lcd.setCursor(0,1);lcd.print("                ");lcd.setCursor(0,1);lcd.print(" ");
                       adres=0x06;waarde=0x02;writeMidibuttons1();value[6]=2;ledupdate();delay(300);break;
                       case 2:lcd.setCursor(0,0);lcd.print("                ");lcd.setCursor(0,0);lcd.print(buttonname1[i+16]);lcd.setCursor(0,1);lcd.print("                ");lcd.setCursor(0,1);lcd.print(" ");
                       adres=0x09;
                       if(value[9]==0){waarde=0x01;writeMidibuttons1();value[9]=1;ledupdate();delay(300);}
                       else{waarde=0x00;writeMidibuttons1();value[9]=0;ledupdate();delay(300);}
                       break;
                       case 3:lcd.setCursor(0,0);lcd.print("                ");lcd.setCursor(0,0);lcd.print(buttonname1[i+32]);lcd.setCursor(0,1);lcd.print("                ");lcd.setCursor(0,1);lcd.print(" ");
                       adres=0x19;
                       waarde=0x02;writeMidibuttons1();value[25]=2;ledupdate();delay(300);
                       break;
                       case 4:lcd.setCursor(0,0);lcd.print("                ");lcd.setCursor(0,0);lcd.print(buttonname1[i+48]);lcd.setCursor(0,1);lcd.print("                ");lcd.setCursor(0,1);lcd.print(" ");
                       adres=0x3A;
                       waarde=0x02;writeMidibuttons1();value[58]=2;ledupdate();delay(300);
                       break;
                       default:break;
                      }
        }
if(i==3){switch(GROUP){case 1:lcd.setCursor(0,0);lcd.print("                ");lcd.setCursor(0,0);lcd.print(buttonname1[i]);lcd.setCursor(0,1);lcd.print("                ");lcd.setCursor(0,1);lcd.print(" ");
                       adres=0x06;waarde=0x03;writeMidibuttons1();value[6]=3;ledupdate();delay(300);break;
                       case 2:lcd.setCursor(0,0);lcd.print("                ");lcd.setCursor(0,0);lcd.print(buttonname1[i+16]);lcd.setCursor(0,1);lcd.print("                ");lcd.setCursor(0,1);lcd.print(" ");
                       adres=0x13;
                       if(value[19]==0){waarde=0x01;writeMidibuttons1();value[19]=1;ledupdate();delay(300);}
                       else{waarde=0x00;writeMidibuttons1();value[19]=0;ledupdate();delay(300);}
                       break;
                       case 3:lcd.setCursor(0,0);lcd.print("                ");lcd.setCursor(0,0);lcd.print(buttonname1[i+32]);lcd.setCursor(0,1);lcd.print("                ");lcd.setCursor(0,1);lcd.print(" ");
                       adres=0x19;
                       waarde=0x03;writeMidibuttons1();value[25]=3;ledupdate();delay(300);
                       break;
                       case 4:lcd.setCursor(0,0);lcd.print("                ");lcd.setCursor(0,0);lcd.print(buttonname1[i+48]);lcd.setCursor(0,1);lcd.print("                ");lcd.setCursor(0,1);lcd.print(" ");
                       adres=0x3A;
                       waarde=0x03;writeMidibuttons1();value[58]=3;ledupdate();delay(300);
                       break;
                       default:break;
                      }
        }
if(i==4){switch(GROUP){case 1:lcd.setCursor(0,0);lcd.print("                ");lcd.setCursor(0,0);lcd.print(buttonname1[i]);lcd.setCursor(0,1);lcd.print("                ");lcd.setCursor(0,1);lcd.print(" ");
                       adres=0x10;
                       if(value[16]<4){waarde=0x00;writeMidibuttons1();value[16]=0;ledupdate();delay(300);}
                       else{waarde=0x04;writeMidibuttons1();value[16]=4;ledupdate();delay(300);}break;
                       case 2:lcd.setCursor(0,0);lcd.print("                ");lcd.setCursor(0,0);lcd.print(buttonname1[i+16]);lcd.setCursor(0,1);lcd.print("                ");lcd.setCursor(0,1);lcd.print(" ");
                       adres=0x56;
                       waarde=0x00;writeMidibuttons1();value[86]=0;ledupdate();delay(300);
                       break;
                       case 3:lcd.setCursor(0,0);lcd.print("                ");lcd.setCursor(0,0);lcd.print(buttonname1[i+32]);lcd.setCursor(0,1);lcd.print("                ");lcd.setCursor(0,1);lcd.print(" ");
                       adres=0x1A;
                       waarde=0x00;writeMidibuttons1();value[26]=0;ledupdate();delay(300);
                       break;
                       case 4:lcd.setCursor(0,0);lcd.print("                ");lcd.setCursor(0,0);lcd.print(buttonname1[i+48]);lcd.setCursor(0,1);lcd.print("                ");lcd.setCursor(0,1);lcd.print(" ");
                       adres=0x43;
                       waarde=0x00;writeMidibuttons1();value[68]=0;ledupdate();delay(300);
                       break;
                       default:break;
                      }
        }
if(i==5){switch(GROUP){case 1:lcd.setCursor(0,0);lcd.print("                ");lcd.setCursor(0,0);lcd.print(buttonname1[i]);lcd.setCursor(0,1);lcd.print("                ");lcd.setCursor(0,1);lcd.print(" ");
                       adres=0x10;
                       if(value[16]<4){waarde=0x01;writeMidibuttons1();value[16]=1;ledupdate();delay(300);}
                       else{waarde=0x05;writeMidibuttons1();value[16]=5;ledupdate();delay(300);}break;
                       case 2:lcd.setCursor(0,0);lcd.print("                ");lcd.setCursor(0,0);lcd.print(buttonname1[i+16]);lcd.setCursor(0,1);lcd.print("                ");lcd.setCursor(0,1);lcd.print(" ");
                       adres=0x56;
                       waarde=0x01;writeMidibuttons1();value[86]=1;ledupdate();delay(300);
                       break;
                       case 3:lcd.setCursor(0,0);lcd.print("                ");lcd.setCursor(0,0);lcd.print(buttonname1[i+32]);lcd.setCursor(0,1);lcd.print("                ");lcd.setCursor(0,1);lcd.print(" ");
                       adres=0x1A;
                       waarde=0x01;writeMidibuttons1();value[26]=1;ledupdate();delay(300);
                       break;
                       case 4:lcd.setCursor(0,0);lcd.print("                ");lcd.setCursor(0,0);lcd.print(buttonname1[i+48]);lcd.setCursor(0,1);lcd.print("                ");lcd.setCursor(0,1);lcd.print(" ");
                       adres=0x43;
                       waarde=0x01;writeMidibuttons1();value[68]=1;ledupdate();delay(300);
                       break;
                       default:break;
                      }
        }  
if(i==6){switch(GROUP){case 1:lcd.setCursor(0,0);lcd.print("                ");lcd.setCursor(0,0);lcd.print(buttonname1[i]);lcd.setCursor(0,1);lcd.print("                ");lcd.setCursor(0,1);lcd.print(" ");
                       adres=0x10;
                       if(value[16]<4){waarde=0x02;writeMidibuttons1();value[16]=2;ledupdate();delay(300);}
                       else{waarde=0x06;writeMidibuttons1();value[16]=6;ledupdate();delay(300);}break;
                       case 2:lcd.setCursor(0,0);lcd.print("                ");lcd.setCursor(0,0);lcd.print(buttonname1[i+16]);lcd.setCursor(0,1);lcd.print("                ");lcd.setCursor(0,1);lcd.print(" ");
                       adres=0x56;
                       waarde=0x02;writeMidibuttons1();value[86]=2;ledupdate();delay(300);
                       break;
                       case 3:lcd.setCursor(0,0);lcd.print("                ");lcd.setCursor(0,0);lcd.print(buttonname1[i+32]);lcd.setCursor(0,1);lcd.print("                ");lcd.setCursor(0,1);lcd.print(" ");
                       adres=0x1A;
                       waarde=0x02;writeMidibuttons1();value[26]=2;ledupdate();delay(300);
                       break;
                       case 4:lcd.setCursor(0,0);lcd.print("                ");lcd.setCursor(0,0);lcd.print(buttonname1[i+48]);lcd.setCursor(0,1);lcd.print("                ");lcd.setCursor(0,1);lcd.print(" ");
                       adres=0x43;
                       waarde=0x02;writeMidibuttons1();value[68]=2;ledupdate();delay(300);
                       break;
                       default:break;
                      }
        }
if(i==7){switch(GROUP){case 1:lcd.setCursor(0,0);lcd.print("                ");lcd.setCursor(0,0);lcd.print(buttonname1[i]);lcd.setCursor(0,1);lcd.print("                ");lcd.setCursor(0,1);lcd.print(" ");
                       adres=0x10;
                       if(value[16]<4){waarde=0x03;writeMidibuttons1();value[16]=3;ledupdate();delay(300);}
                       else{waarde=0x07;writeMidibuttons1();value[16]=7;ledupdate();delay(300);}break;
                       case 2:lcd.setCursor(0,0);lcd.print("                ");lcd.setCursor(0,0);lcd.print(buttonname1[i+16]);lcd.setCursor(0,1);lcd.print("                ");lcd.setCursor(0,1);lcd.print(" ");
                       adres=0x56;
                       waarde=0x03;writeMidibuttons1();value[86]=3;ledupdate();delay(300);
                       break;
                       case 3:lcd.setCursor(0,0);lcd.print("                ");lcd.setCursor(0,0);lcd.print(buttonname1[i+32]);lcd.setCursor(0,1);lcd.print("                ");lcd.setCursor(0,1);lcd.print(" ");
                       adres=0x1A;
                       waarde=0x03;writeMidibuttons1();value[26]=3;ledupdate();delay(300);
                       break;
                       case 4:lcd.setCursor(0,0);lcd.print("                ");lcd.setCursor(0,0);lcd.print(buttonname1[i+48]);lcd.setCursor(0,1);lcd.print("                ");lcd.setCursor(0,1);lcd.print(" ");
                       adres=0x43;
                       waarde=0x03;writeMidibuttons1();value[68]=3;ledupdate();delay(300);
                       break;
                       default:break;
                      }
        }
if(i==8){switch(GROUP){case 1:lcd.setCursor(0,0);lcd.print("                ");lcd.setCursor(0,0);lcd.print(buttonname1[i]);lcd.setCursor(0,1);lcd.print("                ");lcd.setCursor(0,1);lcd.print(" ");
                       adres=0x10;
                       if(value[16]<4){value[16]=value[16]+4;}
                       else {value[16]=value[16]-4;} 
                       if(value[16]==0){waarde=0x00;writeMidibuttons1();ledupdate();delay(300);}
                       if(value[16]==1){waarde=0x01;writeMidibuttons1();ledupdate();delay(300);}
                       if(value[16]==2){waarde=0x02;writeMidibuttons1();ledupdate();delay(300);}
                       if(value[16]==3){waarde=0x03;writeMidibuttons1();ledupdate();delay(300);}
                       if(value[16]==4){waarde=0x04;writeMidibuttons1();ledupdate();delay(300);}
                       if(value[16]==5){waarde=0x05;writeMidibuttons1();ledupdate();delay(300);}
                       if(value[16]==6){waarde=0x06;writeMidibuttons1();ledupdate();delay(300);}
                       if(value[16]==7){waarde=0x07;writeMidibuttons1();ledupdate();delay(300);}
                       break;
                       case 2:lcd.setCursor(0,0);lcd.print("                ");lcd.setCursor(0,0);lcd.print(buttonname1[i+16]);lcd.setCursor(0,1);lcd.print("                ");lcd.setCursor(0,1);lcd.print(" ");
                       adres=0x60;
                       waarde=0x00;writeMidibuttons1();value[96]=0;ledupdate();delay(300);
                       break;
                       case 3:lcd.setCursor(0,0);lcd.print("                ");lcd.setCursor(0,0);lcd.print(buttonname1[i+32]);lcd.setCursor(0,1);lcd.print("                ");lcd.setCursor(0,1);lcd.print(" ");
                       adres=0x2E;
                       waarde=0x00;writeMidibuttons1();value[46]=0;ledupdate();delay(300);
                       break;
                       case 4:lcd.setCursor(0,0);lcd.print("                ");lcd.setCursor(0,0);lcd.print(buttonname1[i+48]);lcd.setCursor(0,1);lcd.print("                ");lcd.setCursor(0,1);lcd.print(" ");
                       adres=0x4E;
                       waarde=0x00;writeMidibuttons1();value[78]=0;ledupdate();delay(300);
                       break;
                       default:break;
                      }
        }
if(i==9){switch(GROUP){case 1:lcd.setCursor(0,0);lcd.print("                ");lcd.setCursor(0,0);lcd.print(buttonname1[i]);lcd.setCursor(0,1);lcd.print("                ");lcd.setCursor(0,1);lcd.print(" ");
                       adres=0x08;
                       if(value[8]==0){waarde=0x01;writeMidibuttons1();value[8]=1;ledupdate();delay(300);}
                       else{waarde=0x00;writeMidibuttons1();value[8]=0;ledupdate();delay(300);}
                       break;
                       case 2:lcd.setCursor(0,0);lcd.print("                ");lcd.setCursor(0,0);lcd.print(buttonname1[i+16]);lcd.setCursor(0,1);lcd.print("                ");lcd.setCursor(0,1);lcd.print(" ");
                       adres=0x60;
                       waarde=0x01;writeMidibuttons1();value[96]=1;ledupdate();delay(300);
                       break;
                       case 3:lcd.setCursor(0,0);lcd.print("                ");lcd.setCursor(0,0);lcd.print(buttonname1[i+32]);lcd.setCursor(0,1);lcd.print("                ");lcd.setCursor(0,1);lcd.print(" ");
                       adres=0x2E;
                       waarde=0x01;writeMidibuttons1();value[46]=1;ledupdate();delay(300);
                       break;
                       case 4:lcd.setCursor(0,0);lcd.print("                ");lcd.setCursor(0,0);lcd.print(buttonname1[i+48]);lcd.setCursor(0,1);lcd.print("                ");lcd.setCursor(0,1);lcd.print(" ");
                       adres=0x4E;
                       waarde=0x01;writeMidibuttons1();value[78]=1;ledupdate();delay(300);
                       break;
                       default:break;
                      }
        } 
if(i==10){switch(GROUP){case 1:lcd.setCursor(0,0);lcd.print("                ");lcd.setCursor(0,0);lcd.print(buttonname1[i]);lcd.setCursor(0,1);lcd.print("                ");lcd.setCursor(0,1);lcd.print(" ");
                       adres=0x12;
                       if(value[18]==0||value[18]==2){waarde=0x01;writeMidibuttons1();value[18]=1;ledupdate();delay(300);}
                       else{waarde=0x00;writeMidibuttons1();value[18]=0;ledupdate();delay(300);}
                       break;
                       case 2:lcd.setCursor(0,0);lcd.print("                ");lcd.setCursor(0,0);lcd.print(buttonname1[i+16]);lcd.setCursor(0,1);lcd.print("                ");lcd.setCursor(0,1);lcd.print(" ");
                       adres=0x60;
                       waarde=0x02;writeMidibuttons1();value[96]=2;ledupdate();delay(300);
                       break;
                       case 3:lcd.setCursor(0,0);lcd.print("                ");lcd.setCursor(0,0);lcd.print(buttonname1[i+32]);lcd.setCursor(0,1);lcd.print("                ");lcd.setCursor(0,1);lcd.print(" ");
                       adres=0x2E;
                       waarde=0x03;writeMidibuttons1();value[46]=3;ledupdate();delay(300);
                       break;
                       case 4:lcd.setCursor(0,0);lcd.print("                ");lcd.setCursor(0,0);lcd.print(buttonname1[i+48]);lcd.setCursor(0,1);lcd.print("                ");lcd.setCursor(0,1);lcd.print(" ");
                       adres=0x4E;
                       waarde=0x02;writeMidibuttons1();value[78]=2;ledupdate();delay(300);
                       break;
                       default:break;
                       }
        }
if(i==11){switch(GROUP){case 1:lcd.setCursor(0,0);lcd.print("                ");lcd.setCursor(0,0);lcd.print(buttonname1[i]);lcd.setCursor(0,1);lcd.print("                ");lcd.setCursor(0,1);lcd.print(" ");
                       adres=0x12;
                       if(value[18]==0||value[18]==1){waarde=0x02;writeMidibuttons1();value[18]=2;ledupdate();delay(300);}
                       else{waarde=0x00;writeMidibuttons1();value[18]=0;ledupdate();delay(300);}
                       break;
                       case 2:lcd.setCursor(0,0);lcd.print("                ");lcd.setCursor(0,0);lcd.print(buttonname1[i+16]);lcd.setCursor(0,1);lcd.print("                ");lcd.setCursor(0,1);lcd.print(" ");
                       adres=0x60;
                       waarde=0x03;writeMidibuttons1();value[96]=3;ledupdate();delay(300);
                       break;
                       case 3:lcd.setCursor(0,0);lcd.print("                ");lcd.setCursor(0,0);lcd.print(buttonname1[i+32]);lcd.setCursor(0,1);lcd.print("                ");lcd.setCursor(0,1);lcd.print(" ");
                       adres=0x2F;
                       if(value[47]==0){waarde=0x01;writeMidibuttons1();value[47]=1;ledupdate();delay(300);}
                       else{waarde=0x00;writeMidibuttons1();value[47]=0;ledupdate();delay(300);}
                       break;
                       case 4:lcd.setCursor(0,0);lcd.print("                ");lcd.setCursor(0,0);lcd.print(buttonname1[i+48]);lcd.setCursor(0,1);lcd.print("                ");lcd.setCursor(0,1);lcd.print(" ");
                       adres=0x4E;
                       waarde=0x03;writeMidibuttons1();value[78]=3;ledupdate();delay(300);
                       break;
                       default:break;
                       }
        }             
//if(value[0+muxButPin1[i]]>63&&i<12){value[0+muxButPin1[i]]=0;ledArray.setPin(muxButPin1[i], OFF);delay(200);writeMidibuttons1();}
//if(value[0+muxButPin1[i]]<63&&i<12){value[0+muxButPin1[i]]=127;ledArray.setPin(muxButPin1[i], ON);delay(200);writeMidibuttons1();/*multiswitch1();*/}
// previousdigital1[i]=digital1[i];lcd.setCursor(0,0);lcd.print("                ");lcd.setCursor(0,0);lcd.print(buttonname1[i]);
// lcd.setCursor(0,1);lcd.print("                ");lcd.setCursor(0,1);if(value[0+muxButPin1[i]]==0){lcd.print("OFF");}else{lcd.print("ON");}
}  
}
//end reading the buttons 1


}

void multiswitch1(){
switch (index){
case 0: value[0+muxButPin1[4]]=0;ledArray.setPin(muxButPin1[4], OFF);index=4;writeMidibuttons1();value[0+muxButPin1[8]]=0;ledArray.setPin(muxButPin1[8], OFF);index=8;writeMidibuttons1();break;
case 4: value[0+muxButPin1[0]]=0;ledArray.setPin(muxButPin1[0], OFF);index=0;writeMidibuttons1();value[0+muxButPin1[8]]=0;ledArray.setPin(muxButPin1[8], OFF);index=8;writeMidibuttons1();break;
case 8: value[0+muxButPin1[0]]=0;ledArray.setPin(muxButPin1[0], OFF);index=0;writeMidibuttons1();value[0+muxButPin1[4]]=0;ledArray.setPin(muxButPin1[4], OFF);index=4;writeMidibuttons1();break; 
default:break;          }  
}

void writeMidibuttons1(){

if (synth1==true){ 
Serial1.write(0xF0);Serial1.write(0x10);Serial1.write(0x06);Serial1.write(0x06);Serial1.write(adres);Serial1.write(waarde);Serial1.write(0xF7);
}
if (synth2==true){ 
Serial2.write(0xF0);Serial2.write(0x10);Serial2.write(0x06);Serial2.write(0x06);Serial2.write(adres);Serial2.write(waarde);Serial2.write(0xF7);
}
}

void writeMatrix(){
if (synth1==true){   
Serial1.write(0xF0);    //1
Serial1.write(0x10);    //2
Serial1.write(0x06);    //3
Serial1.write(0x0B);    //4
Serial1.write(path);   //5
Serial1.write(source);  //6
Serial1.write(amount);  //7
Serial1.write(dest);  //8
Serial1.write(0xF7);    //9
}
if (synth2==true){   
Serial2.write(0xF0);    //1
Serial2.write(0x10);    //2
Serial2.write(0x06);    //3
Serial2.write(0x0B);    //4
Serial2.write(path);   //5
Serial2.write(source);  //6
Serial2.write(amount);  //7
Serial2.write(dest);  //8
Serial2.write(0xF7);    //9
}
}

void checkfile() { 
String filename = "";
if (patchnumber < 10) {
  filename += "00";
} else if (patchnumber < 100) {
  filename += "0";
}
filename += String(patchnumber) + ".txt";
                                                                                                       //1234567890123456
if (SD.exists(filename)) {} else {lcd.setCursor(0, 1);lcd.print("   Empty slot   ");}
}

void checkEncoder() {
    long newPosition = myEnc.read();
    encoderUP = false;
    encoderDOWN = false;
    if (newPosition > oldPosition) {
        if (newPosition - oldPosition >= 4) {
            encoderUP = true;
            encoderDOWN = false;
            oldPosition = newPosition;
        }
    }
    else if (newPosition < oldPosition) {
        if (oldPosition - newPosition >= 4) {
            encoderUP = false;
            encoderDOWN = true;
            oldPosition = newPosition;
        }
    }
}

void DisplayPatchName(){
String filename = "";if (patchnumber < 10) {filename += "00";} else if (patchnumber < 100) {filename += "0";}filename += String(patchnumber) + ".txt";
File myFile = SD.open(filename);
if (myFile) {
    myFile.seek(128);
    int rangeSize = 16;
    char buffer[rangeSize + 1];
    myFile.readBytes(buffer, rangeSize);
    buffer[rangeSize] = '\0';
    lcd.setCursor(0, 1);
    lcd.print(buffer);
    myFile.close();
            }
}

void loadpatch() {
  String filename = "";if (patchnumber < 10) {filename += "00";} else if (patchnumber < 100) {filename += "0";}filename += String(patchnumber) + ".txt";
  File myFile = SD.open(filename);
  char buffer[144];
  for (int i = 0; i < 144; i++) {
    myFile.seek(i);
    int rangeSize = 1;
    myFile.readBytes(&buffer[i], rangeSize);
  }
myFile.close();
for (int i = 0; i < 144; i++) {
value[i]=buffer[i];  
}  

for (int i = 0; i < Nbuttons1; i++) { delay(5);
byte statusByte = 0xB0;
byte dataByte1 = CCnumber5[i];
byte dataByte2 = value[muxButPin1[i]];
Serial1.write(statusByte);
Serial1.write(dataByte1);
Serial1.write(dataByte2);
if(value[muxButPin1[i]]>63){ledArray.setPin(muxButPin1[i], ON);}
else{ledArray.setPin(muxButPin1[i], OFF);}
}

for (int i = 0; i < NPots2; i++) { delay(5);
byte statusByte = 0xB0;
byte dataByte1 = CCnumber2[i];
byte dataByte2 = value[muxPotPin2[i]+16];
Serial1.write(statusByte);
Serial1.write(dataByte1);
Serial1.write(dataByte2);
}


}
void ledupdate(){
switch (GROUP){
case 1:
if (value[6]==0){ledArray.setPin(muxButPin1[0], ON);ledArray.setPin(muxButPin1[1], OFF);ledArray.setPin(muxButPin1[2], OFF);ledArray.setPin(muxButPin1[3], OFF);}
if (value[6]==1){ledArray.setPin(muxButPin1[0], OFF);ledArray.setPin(muxButPin1[1], ON);ledArray.setPin(muxButPin1[2], OFF);ledArray.setPin(muxButPin1[3], OFF);}
if (value[6]==2){ledArray.setPin(muxButPin1[0], OFF);ledArray.setPin(muxButPin1[1], OFF);ledArray.setPin(muxButPin1[2], ON);ledArray.setPin(muxButPin1[3], OFF);}
if (value[6]==3){ledArray.setPin(muxButPin1[0], OFF);ledArray.setPin(muxButPin1[1], OFF);ledArray.setPin(muxButPin1[2], OFF);ledArray.setPin(muxButPin1[3], ON);}
if (value[16]==0||value[16]==4){ledArray.setPin(muxButPin1[4], ON);ledArray.setPin(muxButPin1[5], OFF);ledArray.setPin(muxButPin1[6], OFF);ledArray.setPin(muxButPin1[7], OFF);}
if (value[16]==1||value[16]==5){ledArray.setPin(muxButPin1[4], OFF);ledArray.setPin(muxButPin1[5], ON);ledArray.setPin(muxButPin1[6], OFF);ledArray.setPin(muxButPin1[7], OFF);}
if (value[16]==2||value[16]==6){ledArray.setPin(muxButPin1[4], OFF);ledArray.setPin(muxButPin1[5], OFF);ledArray.setPin(muxButPin1[6], ON);ledArray.setPin(muxButPin1[7], OFF);}
if (value[16]==3||value[16]==7){ledArray.setPin(muxButPin1[4], OFF);ledArray.setPin(muxButPin1[5], OFF);ledArray.setPin(muxButPin1[6], OFF);ledArray.setPin(muxButPin1[7], ON);}
if (value[16]>3){ledArray.setPin(muxButPin1[8], ON);}else{ledArray.setPin(muxButPin1[8], OFF);}
if (value[8]==1){ledArray.setPin(muxButPin1[9], ON);}else{ledArray.setPin(muxButPin1[9], OFF);}
if (value[18]==0){ledArray.setPin(muxButPin1[10], OFF);ledArray.setPin(muxButPin1[11], OFF);}
if (value[18]==1){ledArray.setPin(muxButPin1[10], ON);ledArray.setPin(muxButPin1[11], OFF);}
if (value[18]==2){ledArray.setPin(muxButPin1[10], OFF);ledArray.setPin(muxButPin1[11], ON);}
break;
case 2:
if (value[87]==1){ledArray.setPin(muxButPin1[0], ON);}else{ledArray.setPin(muxButPin1[0], OFF);}
if (value[97]==1){ledArray.setPin(muxButPin1[1], ON);}else{ledArray.setPin(muxButPin1[1], OFF);}
if (value[9]==1){ledArray.setPin(muxButPin1[2], ON);}else{ledArray.setPin(muxButPin1[2], OFF);}
if (value[19]==1){ledArray.setPin(muxButPin1[3], ON);}else{ledArray.setPin(muxButPin1[3], OFF);}
if (value[86]==0){ledArray.setPin(muxButPin1[4], ON);}else{ledArray.setPin(muxButPin1[4], OFF);}
if (value[86]==1){ledArray.setPin(muxButPin1[5], ON);}else{ledArray.setPin(muxButPin1[5], OFF);}
if (value[86]==2){ledArray.setPin(muxButPin1[6], ON);}else{ledArray.setPin(muxButPin1[6], OFF);}
if (value[86]==3){ledArray.setPin(muxButPin1[7], ON);}else{ledArray.setPin(muxButPin1[7], OFF);}
if (value[96]==0){ledArray.setPin(muxButPin1[8], ON);}else{ledArray.setPin(muxButPin1[8], OFF);}
if (value[96]==1){ledArray.setPin(muxButPin1[9], ON);}else{ledArray.setPin(muxButPin1[9], OFF);}
if (value[96]==2){ledArray.setPin(muxButPin1[10], ON);}else{ledArray.setPin(muxButPin1[10], OFF);}
if (value[96]==3){ledArray.setPin(muxButPin1[11], ON);}else{ledArray.setPin(muxButPin1[11], OFF);}
break;
case 3:
if (value[25]==0){ledArray.setPin(muxButPin1[0], ON);}else{ledArray.setPin(muxButPin1[0], OFF);}
if (value[25]==1){ledArray.setPin(muxButPin1[1], ON);}else{ledArray.setPin(muxButPin1[1], OFF);}
if (value[25]==2){ledArray.setPin(muxButPin1[2], ON);}else{ledArray.setPin(muxButPin1[2], OFF);}
if (value[25]==3){ledArray.setPin(muxButPin1[3], ON);}else{ledArray.setPin(muxButPin1[3], OFF);}
if (value[26]==0){ledArray.setPin(muxButPin1[4], ON);}else{ledArray.setPin(muxButPin1[4], OFF);}
if (value[26]==1){ledArray.setPin(muxButPin1[5], ON);}else{ledArray.setPin(muxButPin1[5], OFF);}
if (value[26]==2){ledArray.setPin(muxButPin1[6], ON);}else{ledArray.setPin(muxButPin1[6], OFF);}
if (value[26]==3){ledArray.setPin(muxButPin1[7], ON);}else{ledArray.setPin(muxButPin1[7], OFF);}
if (value[46]==0){ledArray.setPin(muxButPin1[8], ON);}else{ledArray.setPin(muxButPin1[8], OFF);}
if (value[46]==1){ledArray.setPin(muxButPin1[9], ON);}else{ledArray.setPin(muxButPin1[9], OFF);}
if (value[46]==3){ledArray.setPin(muxButPin1[10], ON);}else{ledArray.setPin(muxButPin1[10], OFF);}
if (value[47]==1){ledArray.setPin(muxButPin1[11], ON);}else{ledArray.setPin(muxButPin1[11], OFF);}
break;
case 4:
if (value[58]==0){ledArray.setPin(muxButPin1[0], ON);}else{ledArray.setPin(muxButPin1[0], OFF);}
if (value[58]==1){ledArray.setPin(muxButPin1[1], ON);}else{ledArray.setPin(muxButPin1[1], OFF);}
if (value[58]==2){ledArray.setPin(muxButPin1[2], ON);}else{ledArray.setPin(muxButPin1[2], OFF);}
if (value[58]==3){ledArray.setPin(muxButPin1[3], ON);}else{ledArray.setPin(muxButPin1[3], OFF);}
if (value[68]==0){ledArray.setPin(muxButPin1[4], ON);}else{ledArray.setPin(muxButPin1[4], OFF);}
if (value[68]==1){ledArray.setPin(muxButPin1[5], ON);}else{ledArray.setPin(muxButPin1[5], OFF);}
if (value[68]==2){ledArray.setPin(muxButPin1[6], ON);}else{ledArray.setPin(muxButPin1[6], OFF);}
if (value[68]==3){ledArray.setPin(muxButPin1[7], ON);}else{ledArray.setPin(muxButPin1[7], OFF);}
if (value[78]==0){ledArray.setPin(muxButPin1[8], ON);}else{ledArray.setPin(muxButPin1[8], OFF);}
if (value[78]==1){ledArray.setPin(muxButPin1[9], ON);}else{ledArray.setPin(muxButPin1[9], OFF);}
if (value[78]==2){ledArray.setPin(muxButPin1[10], ON);}else{ledArray.setPin(muxButPin1[10], OFF);}
if (value[78]==3){ledArray.setPin(muxButPin1[11], ON);}else{ledArray.setPin(muxButPin1[11], OFF);}
break;
default:break;
}  
}

void displayChoice(){
switch(kladindex){
case 6:if (kladwaarde==0){lcd.setCursor(0,1);lcd.print("                ");lcd.setCursor(0,1);lcd.print("OFF");}
       if (kladwaarde==1){lcd.setCursor(0,1);lcd.print("                ");lcd.setCursor(0,1);lcd.print("SOFT");}
       if (kladwaarde==2){lcd.setCursor(0,1);lcd.print("                ");lcd.setCursor(0,1);lcd.print("MEDIUM");}
       if (kladwaarde==3){lcd.setCursor(0,1);lcd.print("                ");lcd.setCursor(0,1);lcd.print("HARD");}
       break;
case 5:if (kladwaarde==0){lcd.setCursor(0,1);lcd.print("                ");lcd.setCursor(0,1);lcd.print("OFF");}
       if (kladwaarde==1){lcd.setCursor(0,1);lcd.print("                ");lcd.setCursor(0,1);lcd.print("P-BEND");}
       if (kladwaarde==2){lcd.setCursor(0,1);lcd.print("                ");lcd.setCursor(0,1);lcd.print("VIBRATO");}
       if (kladwaarde==3){lcd.setCursor(0,1);lcd.print("                ");lcd.setCursor(0,1);lcd.print("BOTH");}       
       break;   
case 13:if (kladwaarde==0){lcd.setCursor(0,1);lcd.print("                ");lcd.setCursor(0,1);lcd.print("OFF");}
        if (kladwaarde==1){lcd.setCursor(0,1);lcd.print("                ");lcd.setCursor(0,1);lcd.print("P-BEND");}
        if (kladwaarde==2){lcd.setCursor(0,1);lcd.print("                ");lcd.setCursor(0,1);lcd.print("VIBRATO");}
        if (kladwaarde==3){lcd.setCursor(0,1);lcd.print("                ");lcd.setCursor(0,1);lcd.print("BOTH");}       
        break; 
case 14:if (kladwaarde==0){lcd.setCursor(0,1);lcd.print("                ");lcd.setCursor(0,1);lcd.print("NONE");}
        if (kladwaarde==1){lcd.setCursor(0,1);lcd.print("                ");lcd.setCursor(0,1);lcd.print("PORTAMENTO");}
        if (kladwaarde==2){lcd.setCursor(0,1);lcd.print("                ");lcd.setCursor(0,1);lcd.print("KEYBOARD TRACK");}   
        break;
case 16:if (kladwaarde==0){lcd.setCursor(0,1);lcd.print("                ");lcd.setCursor(0,1);lcd.print("TRIANGLE");}
        if (kladwaarde==1){lcd.setCursor(0,1);lcd.print("                ");lcd.setCursor(0,1);lcd.print("SAW");}
        if (kladwaarde==2){lcd.setCursor(0,1);lcd.print("                ");lcd.setCursor(0,1);lcd.print("INVERTED SAW");}   
        if (kladwaarde==3){lcd.setCursor(0,1);lcd.print("                ");lcd.setCursor(0,1);lcd.print("SQUARE");}
        if (kladwaarde==4){lcd.setCursor(0,1);lcd.print("                ");lcd.setCursor(0,1);lcd.print("RANDOM");}
        if (kladwaarde==5){lcd.setCursor(0,1);lcd.print("                ");lcd.setCursor(0,1);lcd.print("NOISE");}  
        if (kladwaarde==6){lcd.setCursor(0,1);lcd.print("                ");lcd.setCursor(0,1);lcd.print("S&H");}
        break;  
case 24:if (kladwaarde==0){lcd.setCursor(0,1);lcd.print("                ");lcd.setCursor(0,1);lcd.print("TRIANGLE");}
        if (kladwaarde==1){lcd.setCursor(0,1);lcd.print("                ");lcd.setCursor(0,1);lcd.print("SAW");}
        if (kladwaarde==2){lcd.setCursor(0,1);lcd.print("                ");lcd.setCursor(0,1);lcd.print("INVERTED SAW");}   
        if (kladwaarde==3){lcd.setCursor(0,1);lcd.print("                ");lcd.setCursor(0,1);lcd.print("SQUARE");}
        if (kladwaarde==4){lcd.setCursor(0,1);lcd.print("                ");lcd.setCursor(0,1);lcd.print("RANDOM");}
        if (kladwaarde==5){lcd.setCursor(0,1);lcd.print("                ");lcd.setCursor(0,1);lcd.print("NOISE");}  
        if (kladwaarde==6){lcd.setCursor(0,1);lcd.print("                ");lcd.setCursor(0,1);lcd.print("S&H");}
        break; 
case 17:if (kladwaarde==0){lcd.setCursor(0,1);lcd.print("                ");lcd.setCursor(0,1);lcd.print("Unused");}
        if (kladwaarde==1){lcd.setCursor(0,1);lcd.print("                ");lcd.setCursor(0,1);lcd.print("Enveloppe 1");}
        if (kladwaarde==2){lcd.setCursor(0,1);lcd.print("                ");lcd.setCursor(0,1);lcd.print("Enveloppe 2");}   
        if (kladwaarde==3){lcd.setCursor(0,1);lcd.print("                ");lcd.setCursor(0,1);lcd.print("Enveloppe 3");}
        if (kladwaarde==4){lcd.setCursor(0,1);lcd.print("                ");lcd.setCursor(0,1);lcd.print("LFO 1");}
        if (kladwaarde==5){lcd.setCursor(0,1);lcd.print("                ");lcd.setCursor(0,1);lcd.print("LFO 2");}  
        if (kladwaarde==6){lcd.setCursor(0,1);lcd.print("                ");lcd.setCursor(0,1);lcd.print("Vibrato");}
        if (kladwaarde==7){lcd.setCursor(0,1);lcd.print("                ");lcd.setCursor(0,1);lcd.print("Ramp 1");}
        if (kladwaarde==8){lcd.setCursor(0,1);lcd.print("                ");lcd.setCursor(0,1);lcd.print("Ramp 2");}  
        if (kladwaarde==9){lcd.setCursor(0,1);lcd.print("                ");lcd.setCursor(0,1);lcd.print("Keyboard");}        
        break;    
case 25:if (kladwaarde==0){lcd.setCursor(0,1);lcd.print("                ");lcd.setCursor(0,1);lcd.print("Unused");}
        if (kladwaarde==1){lcd.setCursor(0,1);lcd.print("                ");lcd.setCursor(0,1);lcd.print("Enveloppe 1");}
        if (kladwaarde==2){lcd.setCursor(0,1);lcd.print("                ");lcd.setCursor(0,1);lcd.print("Enveloppe 2");}   
        if (kladwaarde==3){lcd.setCursor(0,1);lcd.print("                ");lcd.setCursor(0,1);lcd.print("Enveloppe 3");}
        if (kladwaarde==4){lcd.setCursor(0,1);lcd.print("                ");lcd.setCursor(0,1);lcd.print("LFO 1");}
        if (kladwaarde==5){lcd.setCursor(0,1);lcd.print("                ");lcd.setCursor(0,1);lcd.print("LFO 2");}  
        if (kladwaarde==6){lcd.setCursor(0,1);lcd.print("                ");lcd.setCursor(0,1);lcd.print("Vibrato");}
        if (kladwaarde==7){lcd.setCursor(0,1);lcd.print("                ");lcd.setCursor(0,1);lcd.print("Ramp 1");}
        if (kladwaarde==8){lcd.setCursor(0,1);lcd.print("                ");lcd.setCursor(0,1);lcd.print("Ramp 2");}  
        if (kladwaarde==9){lcd.setCursor(0,1);lcd.print("                ");lcd.setCursor(0,1);lcd.print("Keyboard");}        
        break;  
case 63:if (kladwaarde==0){lcd.setCursor(0,1);lcd.print("                ");lcd.setCursor(0,1);lcd.print("Reassign");}
        if (kladwaarde==1){lcd.setCursor(0,1);lcd.print("                ");lcd.setCursor(0,1);lcd.print("Rotate");}
        if (kladwaarde==2){lcd.setCursor(0,1);lcd.print("                ");lcd.setCursor(0,1);lcd.print("Unison");}   
        if (kladwaarde==3){lcd.setCursor(0,1);lcd.print("                ");lcd.setCursor(0,1);lcd.print("Reassign+Rotate");}   
        break;                                                
default:break;
}
}
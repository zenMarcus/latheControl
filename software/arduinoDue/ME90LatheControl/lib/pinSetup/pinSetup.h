#ifndef HEADER_FILE
#define HEADER_FILE
// arduino pin definition and init.
//SPI pin

//Spindle motor
  //phase current Sensing
  #define IsenC A0
  #define IsenB A1
  #define IsenA A2

  //voltage sense
  #define Vsen_PVDD A3
  #define VsenC A4
  #define VsenB A5
  #define VsenA A6

  //Thermistor
  #define TH1 A10

  //Spindle Speed
  #define spd A11

  //DRV8305 digital pins
  #define SCS 26 //chip select for SPI
  #define PWRGD 28
  #define ENGate 30
  #define nFault 32

//encoders
  //encoder pwm signal (2mm / 14bit)
  #define PWMx 35
  #define PWMz 37
  #define SpnLFLAG 45
  #define SpnDFLAG 51

  //slave select for encoder counters
  #define SS_spin 33
  #define SS_z 48
  #define SS_x 46

  //slave select for encoder SSI interface
  #define CsnX 47 //(not use but needs init)
  #define CsnZ 49 //(not use but needs init)
  #define CsnSpn 50 //(not use but needs init)
  #define CsnMot 31

//Switches
  #define RUN_PIN 23
  #define DIR_PIN 25
  #define eStop 27

void pinSetup(){
  pinMode(nFault, INPUT);
  pinMode(PWRGD, INPUT);
  pinMode(RUN, INPUT_PULLUP);
  pinMode(DIR, INPUT_PULLUP);
  pinMode(eStop, INPUT_PULLUP);
  pinMode(PWMx, INPUT);
  pinMode(PWMz, INPUT);
  pinMode(SpnDFLAG, INPUT);
  pinMode(SpnLFLAG, INPUT);
  pinMode(SCS, OUTPUT);
  pinMode(ENGate, OUTPUT);
  pinMode(SS_spin, OUTPUT);
  pinMode(SS_x, OUTPUT);
  pinMode(SS_z, OUTPUT);
  pinMode(CsnX, OUTPUT);
  pinMode(CsnZ, OUTPUT);
  pinMode(CsnMot, OUTPUT);
  pinMode(CsnSpn, OUTPUT);
  pinMode(6, OUTPUT);
  pinMode(7, OUTPUT);
  pinMode(8, OUTPUT);

  //testPins
  pinMode(9,OUTPUT);
  pinMode(13,OUTPUT);
  pinMode(12,OUTPUT);
  pinMode(11,OUTPUT);


}

#endif
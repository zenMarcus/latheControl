#ifndef pinSetup_h
#define pinSetup_h
// arduino pin definition and init.
//SPI pin

//find ADC pins for the following in ADC.cpp
  //phase current Sensing 
  //bus and A B phase voltage
  //Spindle Speed

  //motor Thermistor
  #define TH1 A11

  //DRV8305 digital pins
  #define SCS 31     //chip select for SPI
  #define PWRGD 29
  #define ENGate 30
  #define nFault 49

//encoders
  //encoder pwm signal (2mm / 14bit)

  #define SpnLFLAG 44
  #define SpnDFLAG 50

  //slave select for encoder counters
  #define SS_spin 33
  #define SS_z 26
  #define SS_x 24

  //slave select for encoder SSI? interface
  #define CsnX 4 //(not use but needs init)
  #define CsnZ 4 //(not use but needs init)
  //#define CsnSpn 50 //(not use but needs init) deprecated?
  #define CsnMot 28 // motor encoder chip select

//Switches
  #define RUN_PIN 23
  #define DIR_PIN 25
  #define eStop 27  

void pinSetup();

#endif

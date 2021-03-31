#include <Arduino.h> 
#include <RunningAverage.h>
#include <SPI.h>
#include "config.h"
#include "pinSetup.h"
#include "AS5048A.h"
#include "DRV8305.h"
#include "pwm.h"
#include "adc.h"
#include "pid.h"
#include "interrupts.h"
#include "foc.h"

#define MOTOR_LIMIT_TEMP 75 //in deg C
#define ADC_RES 12

//define SPI confings for all devices
//SPISettings LS73663(1000000, MSBFIRST, SPI_MODE1);//rnd vals
//SPISettings drv8305SPISettings(1000000, MSBFIRST, SPI_MODE1);

// definitions here
//uint16_t rotorAngle = 0;
int16_t motorVelocity = 0;
uint16_t encoderOffset = 0;

uint8_t FOCCadence = 4;   // foc is called every FOCCadence interrupts min is 3 beacuse of slow SPI
uint8_t velPidCadence = 3;// velocity pid is called every N Foc interrupts
int16_t vectorAmp = 0;    // raw adc 0 - 4096 //patchy var to replace vectorAmplitude to int (PID will need to run on int math)

struct drv8305param drvParam;
struct controlStatus controlStatus;

AS5048A motorEncoder(CsnMot); //construct motor encoder object
RunningAverage setSpeedAverage(10); //set speed to be sampled as running Average
RunningAverage inputCurrentAverage(30);
RunningAverage potAvergage(10);
RunningAverage TH1Average(5); //set speed to be sampled as running Average

//uC  
bool toggle = 0; //for loop timing measure


bool motorOverheat = 0;
bool rotorAligned = 0;
volatile bool eStopStatus;
float motorTemperature = 0;

//spindle
bool spindleDirection = 1;  // 1 = conventional lathe FWD
//comms
String OutString = "";


//Debug variables------------------------------------------------------------------------/

volatile bool l,h,c; //for ISR test only remove when validated

struct debugVars logger;

uint16_t tempAngle = 0;
uint16_t vPidOutput = 0;


//----------------------------------------------------------------------------------------/


void eStopPressed(){
  //disables Gates when eStop is pushed+
  //digitalWrite (ENGate, 0);

  eStopStatus = 0;
  Serial.println("eSTOP engaged!"); //debug Line

}

void updateMotorTemperature() {
  TH1Average.addValue(analogRead(TH1));
  motorTemperature = TH1Average.getAverage(); // add regression curve;
  if (motorTemperature > MOTOR_LIMIT_TEMP) {
    motorOverheat = 1;
  } else {
    motorOverheat = 0;
  }
}

void startSpindle (){
  /*if safety checks on
      eStop
      motorOverheat
    are passed starts spindle PID
    otherwise do nothing
  */
  if (eStopStatus == 1){
    // digitalWrite(ENGate, HIGH);
    // should i add a velocity ramp up? y[0,1] = 3x^4 - 2x^3
    //torquePID.SetMode(AUTOMATIC);
    //velocityPID.SetMode(AUTOMATIC);
    //Serial.println("ready to GO!");
  }

}


//----------------------------------------------------------------------------------------/
void setup() {
  pinSetup();
  analogReadResolution(ADC_RES);

  setup_pwm();
  updatePWM(0, 0, 0);

  Serial.begin(115200);
  while (!Serial);
  Serial.println("serial up");

  SPI.begin();

  // set the PID constants
  drvParam.pid_KP = 2000;
  drvParam.pid_KI = 6;
  drvParam.pid_KD = 0;

  drvParam.velPid_KP = 28000;
  drvParam.velPid_KI = 500; 
  drvParam.velPid_KD = 500;

  // initialize the spindle motor drive
  delay(100);
  initSpindleDrv();
  delay(100);

  motorEncoder.init();
  Serial.println("motor encoder initialized");

  fillSineTable();

  // call this every time for the moment, but will need to be called upon
  // request
  delay(10);

  //encoderOffset = mapEncoder(); //i may need to remove the cogging torque (fft, notch filter?)
  encoderOffset = alignRotor();
  Serial.println("encoder offset = " + String(encoderOffset));

  // configure the PIDs
  setupFoc();

  //this will start the PWM interrupts
  setup_adc();

  // test version of a startup current bias on the ADC channels for current
  for (int i=0;i<128;i++)
  {
    drvParam.adcCurrentBiasA += getCurrentA();
    drvParam.adcCurrentBiasB += getCurrentB();
    drvParam.adcCurrentBiasC += getCurrentC();
  }
  drvParam.adcCurrentBiasA /=128;
  drvParam.adcCurrentBiasB /=128;
  drvParam.adcCurrentBiasC /=128;
  Serial.println("current A bias = " + String(drvParam.adcCurrentBiasA));
  Serial.println("current B bias = " + String(drvParam.adcCurrentBiasB));
  Serial.println("current C bias = " + String(drvParam.adcCurrentBiasC));

  controlStatus.inReverse = false; //keep it ther till it works

  
  /*
  if (digitalRead(RUN_PIN == 0)){ // reads eStopStatus only if the RUN is disabled
    eStopStatus = digitalRead(eStop);
  }
  else{
    eStopStatus = 0; //otherwist imposes it
  }
  */

  logger.logState=0;

}


void loop() {
  

  //float displayVelocity = (motorVelocity / (4 * 180)) * 3662; // test line: convert deltaTheta in RPM
  //Serial.println(String(displayVelocity) + ",0,50"  ); // String(rotorAngle) + ',' + 
  
  delay(1);
  //Serial.println(vectorAmp);

  if((millis() > 6000) && (logger.logState ==0)){
    Serial.println("start log");
    logger.logState = 1;
  }

Serial.println(String(controlStatus.Id) +','+ 
               String(controlStatus.Iq) +','+ 
               String(controlStatus.IqRef) +','+ 
               String(10*controlStatus.velMec) +','+
               String(10*controlStatus.velRef)  //+','+
               //String(controlStatus.Ia) //+','+
               //String(controlStatus.Vq)
              );// + ",-512,512" );
/*
  if(logger.logState == 2){
    logger.logState = 3;
    Serial.println("a,b,c,alpha,beta,Id,Iq");
    for (int t=0; t<1024; t++){
      Serial.println(
        String(logger.logA[t]) +','+
        String(logger.logB[t]) +','+
        String(logger.logC[t]) +','+
        String(logger.logAlpha[t]) +','+
        String(logger.logBeta[t]) +','+
        String(logger.logId[t]) +','+
        String(logger.logIq[t]) +','+
        String(logger.logTheta[t])

      );
    }
  }
*/

delay(5);


  /* reactivate this when ready to work on it
  if (digitalRead(RUN_PIN) == 1){
    startSpindle();
  }
  else{
    // stopSpindle() should i add a velocity ramp down? y[0,1] =1 - (3x^4 - 2x^3)
    torquePID.SetMode(MANUAL);
    velocityPID.SetMode(MANUAL);
  }

  if (digitalRead(RUN_PIN == 0)){ // reset an eStop Event only when the RUN is disabled
    eStopStatus = digitalRead(eStop); //and the eStop has been reset
  }
  */
  toggle = !toggle;
}
//----------------------------------------------------------------------------------------/

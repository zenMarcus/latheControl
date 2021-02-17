#include <Arduino.h> 
#include <RunningAverage.h>
#include <PID_v1.h>
#include <SPI.h>
#include "config.h"
#include "pinSetup.h"
#include "AS5048A.h"
#include "DRV8305.h"
#include "pwm.h"
#include "adc.h"
#include "interrupts.h"
#include "foc.h"

#define MOTOR_LIMIT_TEMP 75 //in deg C
#define ADC_RES 12

//define SPI confings for all devices
//SPISettings LS73663(1000000, MSBFIRST, SPI_MODE1);//rnd vals

//SPISettings drv8305SPISettings(1000000, MSBFIRST, SPI_MODE1);

// definitions here
uint16_t FOCFreq = 3;
uint16_t torquePIDFreq = PWM_FREQ / 14;
uint16_t velocityPIDFreq = PWM_FREQ / 21;

// PIDs
double reqSpindleTorque, quadratureCurrent, directCurrent,vectorAmplitude, inputCurrent;
double tKp = 1.0,
       tKi = 0.00,
       tKd = 0.000;

double reqMotorVelocity, velSetpoint, curMotorVelocity;
double vKp = 0.00, 
       vKi = 0.00, 
       vKd = 0.00;

AS5048A motorEncoder(CsnMot);
RunningAverage setSpeedAverage(10); //set speed to be sampled as running Average
RunningAverage inputCurrentAverage(30);
RunningAverage potAvergage(10);
RunningAverage TH1Average(5); //set speed to be sampled as running Average

//uC  
bool toggle = 0; //for loop timing measure


//PID torquePID(input(Isens 0.0 : 15.0 A) , output (0.0 : 1.0 ), setpoint (0.0A - 5.0A))
PID torquePID(&inputCurrent, &vectorAmplitude, &reqSpindleTorque, tKp, tKi, tKd, DIRECT);

//PID velPID(input(velocity RPM) , output (0.0A - 5.0A), setpoint (-200.0 : 1450.0 RPM))
PID velocityPID(&curMotorVelocity, &reqSpindleTorque, &reqMotorVelocity, vKp, vKi, vKd, DIRECT);


bool motorOverheat = 0;
bool rotorAligned = 0;
volatile bool eStopStatus;
float motorTemperature = 0;

// unused
// const int pwmFreq = 32000; //in Hz
// word test = 0;

//spindle
bool spindleDirection = 1;  // 1 = conventional lathe FWD
//comms
String OutString = "";


//Debug variables------------------------------------------------------------------------/

volatile bool l,h,c; //for ISR test only remove when validated

uint16_t tempAngle = 0;
String logTable[1024];
int logState = 0;
int ilog = 0;

//----------------------------------------------------------------------------------------/


void eStopPressed(){
  //disables Gates when eStop is pushed+
  //digitalWrite (ENGate, 0);
  torquePID.SetMode(MANUAL);
  velocityPID.SetMode(MANUAL);
  eStopStatus = 0;
  Serial.println("eSTOP engaged!"); //debug Line

}

void updateReqMotorVelocity (){
  //check the required direction
  spindleDirection = digitalRead (DIR_PIN);

  setSpeedAverage.addValue(analogRead(spd));
  if (spindleDirection == 1){
    reqMotorVelocity = (setSpeedAverage.getAverage() / ADC_SAMPLES) * maxMotorVelocity;
  }
  else{
    //add reverse limiter
    reqMotorVelocity = -(setSpeedAverage.getAverage() / ADC_SAMPLES) * maxMotorVelocity;
  }
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
    // should i add a velocity ramp up? y[0,1] = 3x^2 - 2x^3
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
  rotorAligned = alignRotor();
  //Serial.println(FOCoffset); //debug line
  
  //this will start the PWM interrupts
  setup_adc();

  /*
  if (digitalRead(RUN_PIN == 0)){ // reads eStopStatus only if the RUN is disabled
    eStopStatus = digitalRead(eStop);
  }
  else{
    eStopStatus = 0; //otherwist imposes it
  }
  */



  torquePID.SetMode(MANUAL);
  torquePID.SetTunings(tKp, tKi, tKd);
  torquePID.SetOutputLimits(-maxVectorAmplitude, maxVectorAmplitude);
  torquePID.SetSampleTime(1);
}


void loop() {

  //digitalWrite(9,toggle);
  //updateReqMotorVelocity(); //call this around 10Hz ??
  //Serial.print(getMotorTemp2());
  //Serial.print('\t');
  //Serial.println(tempAngle);
  delay(3);

  if((millis() > 4000) && (logState ==0)){
    Serial.println("start log");
    logState = 1;
  }

  if(logState == 2){
    logState = 3;
    for (int t=0; t<1024; t++){
      Serial.println(logTable[t]);
    }
  }


//  delay(5);


  /* reactivate this when ready to work on it
  if (digitalRead(RUN_PIN) == 1){
    startSpindle();
  }
  else{
    // stopSpindle() should i add a velocity ramp down? y[0,1] =1 - (3x^2 - 2x^3)
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

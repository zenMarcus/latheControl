#include <Arduino.h>
#include <pinSetup.h>
#include <RunningAverage.h>
#include <pwm_lib.h>
#include <PID_v1.h>
#include <AS5048A.h>

using namespace arduino_due::pwm_lib;


#define MOTOR_LIMIT_TEMP 75 //in deg C
#define ADC_RES 12



RunningAverage setSpeedAverage(10); //set speed to be sampled as running Average
RunningAverage TH1Average(5); //set speed to be sampled as running Average

//uC
bool toggle = 0; //for timing measure
int ADCsamples = pow(2,ADC_RES);
const int ISenseRange = 40; // -20A - 0A - 20A mapped to 0.00V - 1.65V -3.33V
const int ISenseOffset = 20;// to shift the ADC value as above
// PIDs
double reqSpindleTorque, phaseCurrent, vectorAmplitude;
double tKp = 0.00,
       tKi = 0.00,
       tKd = 0.00;

double reqMotorVelocity, velSetpoint, curMotorVelocity;
double vKp = 0.00,
       vKi = 0.00,
       vKd = 0.00;
//PID torquePID(input(Isens 0.0 : 15.0 A) , output (0.0 : 1.0 ), setpoint (0.0A - 5.0A))
PID torquePID(&phaseCurrent, &vectorAmplitude, &reqSpindleTorque, tKp, tKi, tKd, DIRECT);

//PID velPID(input(velocity RPM) , output (0.0A - 5.0A), setpoint (-200.0 : 1450.0 RPM))
PID velocityPID(&curMotorVelocity, &reqSpindleTorque, &reqMotorVelocity, vKp, vKi, vKd, DIRECT);
int FOCFreq = 2351; //349th prime
int torquePIDFreq = 1409; //223th prime
int velocityPIDFreq = 283; //61th prime


//motor
bool motorOverheat = 0;
volatile bool eStopStatus;
float motorTemperature = 0;
float rotorAngle = 0;
const int pwmFreq = 32000; //in Hz
const int sineTableSize = 1024;
int pwmSineTable[sineTableSize];
int pwmPeriod = 0; //will be calculates in setup in hundredths of uS
int phaseOffsetB = sineTableSize / 3;
int phaseOffsetC = (sineTableSize / 3) * 2;
const int motorEncResolution = 16384; //14bit encoder
const double maxMotorCurrent = 5.0;
const int maxMotorVelocity = 1450; //in RPM

//spindle
bool spindleDirection = 1;  // 1 = conventional lathe FWD
//comms
String OutString = "";

//phase PWM objects
// defined for DUE http://www.robgray.com/temp/Due-pinout-WEB.png
pwm<pwm_pin::PWML7_PC24> inH_A; // PWM_CH7 on pin 6 on arduino DUE
pwm<pwm_pin::PWML6_PC23> inH_B; // PWM_CH6 on pin 7 on arduino DUE
pwm<pwm_pin::PWML5_PC22> inH_C; // PWM_CH5 on pin 8 on arduino DUE

//Timer interrupts------------------------------------------------------------------------/

volatile bool l,h,c; //for ISR test only remove when validated

void TC3_Handler()   // FOC ISR
{
  TC_GetStatus(TC1, 0); // TC_GetStatus to "accept" interrupt parameters (TCn, channel)
  digitalWrite(13, l = !l); //debug
  int FOCIndex;
  // rotorAngle = motorEncoder.getPostion();
  // calculate FOCIndex

  // set the vector according to the required direction (rotorAngle +- PI/2)
  if (vectorAmplitude < 0){
    FOCIndex = ((rotorAngle / motorEncResolution) * sineTableSize ) - (sineTableSize/4);
  }
  else{
    FOCIndex = ((rotorAngle / motorEncResolution) * sineTableSize ) + (sineTableSize/4);
  }

  // set appropriate duty cycle
  inH_A.set_duty(pwmSineTable[ FOCIndex ] * vectorAmplitude);
  inH_B.set_duty(pwmSineTable[ (FOCIndex + phaseOffsetB) & (sineTableSize-1)] * vectorAmplitude );
  inH_B.set_duty(pwmSineTable[ (FOCIndex + phaseOffsetC) & (sineTableSize-1)] * vectorAmplitude );
}

void TC4_Handler() // torque PID ISR
{
  TC_GetStatus(TC1, 1);
  digitalWrite(12, h = !h);
  //Measuring currents and scaling them
  float IphaseA = ((analogRead(IsenA) / ADCsamples) * ISenseRange ) - ISenseOffset;
  float IphaseB = ((analogRead(IsenB) / ADCsamples) * ISenseRange ) - ISenseOffset;
  float IphaseC = ((analogRead(IsenC) / ADCsamples) * ISenseRange ) - ISenseOffset;

  /*  vector sum of the phase currents
  CurrentSinComp = IphaseA * sin(0) + IphaseB * sin(2/3PI) + IphaseC * sin(4/3PI);
  CurrentCosComp = IphaseA * cos(0) + IphaseB * cos(2/3PI) + IphaseC * cos(4/3PI);
  numerically expressed : */
  float CurrentSinComp = IphaseB * (0.866) + IphaseC * (-0.866);
  float CurrentCosComp = IphaseA + IphaseB * (-0.5) + IphaseC * (-0.5);
  // then calculate the magnitude
  phaseCurrent = sqrt ( sq(CurrentSinComp) * sq(CurrentCosComp) );

  torquePID.Compute();

}

void TC5_Handler() // velocity PID ISR
{
  TC_GetStatus(TC1, 2);
  digitalWrite(11, c = !c);
  //curMotorVelocity = (curMotorPosition - prevMotorPosition) / (1/velocityPIDFreq);
  velocityPID.Compute();
}

void startTimer(Tc *tc, uint32_t channel, IRQn_Type irq, uint32_t frequency) {
  pmc_set_writeprotect(false);
  pmc_enable_periph_clk((uint32_t)irq);
  TC_Configure(tc, channel, TC_CMR_WAVE | TC_CMR_WAVSEL_UP_RC | TC_CMR_TCCLKS_TIMER_CLOCK4);
  uint32_t rc = VARIANT_MCK/128/frequency; //128 because we selected TIMER_CLOCK4 above
  TC_SetRA(tc, channel, rc/2); //50% high, 50% low
  TC_SetRC(tc, channel, rc);
  TC_Start(tc, channel);
  tc->TC_CHANNEL[channel].TC_IER=TC_IER_CPCS;
  tc->TC_CHANNEL[channel].TC_IDR=~TC_IER_CPCS;
  NVIC_EnableIRQ(irq);
}

//----------------------------------------------------------------------------------------/

void eStopPressed(){
  //disables Gates when eStop is pushed
  digitalWrite (ENGate, 0);
  torquePID.SetMode(MANUAL);
  velocityPID.SetMode(MANUAL);
  eStopStatus = 0;
  SerialUSB.println("eSTOP engaged!"); //debug Line

}

void serialDataSend (){
  //transmit data package

}

void updateReqMotorVelocity (){
  spindleDirection = digitalRead (DIR_PIN);

  setSpeedAverage.addValue(analogRead(spd));
  if (spindleDirection == 1){
    reqMotorVelocity = (setSpeedAverage.getAverage() / ADCsamples) * maxMotorVelocity;
  }
  else{
    //add reverse limiter
    reqMotorVelocity = -(setSpeedAverage.getAverage() / ADCsamples) * maxMotorVelocity;
  }
}

void updateMotorTemperature (){
  TH1Average.addValue(analogRead(TH1));
  motorTemperature = TH1Average.getAverage(); // add regression curve;
  if (motorTemperature > MOTOR_LIMIT_TEMP){
    motorOverheat = 1;
  }
  else{
    motorOverheat = 0;
  }
}

void fillSineTable (){
  for (int i=0; i < sineTableSize; i++){
      pwmSineTable[i] = (sin( (float(i) / sineTableSize) * 2*PI ) + 1)/2 * pwmPeriod;
      SerialUSB.println(pwmSineTable[i]);
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
    digitalWrite(ENGate, HIGH);
    // should i add a velocity ramp up? y[0,1] = 3x^2 - 2x^3
    torquePID.SetMode(AUTOMATIC);
    velocityPID.SetMode(AUTOMATIC);
    //SerialUSB.println("ready to GO!");
  }

}

//----------------------------------------------------------------------------------------/
void setup() {

  analogReadResolution(ADC_RES);
  pinSetup();
  SerialUSB.begin(1200);
  delay(2000);

  torquePID.SetMode(MANUAL);
  torquePID.SetTunings(tKp, tKi, tKd);
  torquePID.SetOutputLimits(-maxMotorCurrent, maxMotorCurrent);
  torquePID.SetSampleTime(0);

  velocityPID.SetMode(MANUAL);
  velocityPID.SetTunings(vKp, vKi, vKd);
  velocityPID.SetOutputLimits(-maxMotorVelocity , maxMotorVelocity);
  velocityPID.SetSampleTime(0);

  //while(!SerialUSB);
  //SerialUSB.println("serial");

  if (digitalRead(RUN_PIN == 0)){ // reads eStopStatus only if the RUN is disabled
    eStopStatus = digitalRead(eStop);
  }
  else{
    eStopStatus = 0; //otherwist imposes it
  }


  //external interrupts
  attachInterrupt(digitalPinToInterrupt(eStop), eStopPressed, FALLING);

  //timer interrupts
  startTimer(TC1, 0, TC3_IRQn, FOCFreq); //FOC service routine
  startTimer(TC1, 1, TC4_IRQn, torquePIDFreq); //torque PID isr
  startTimer(TC1, 2, TC5_IRQn, velocityPIDFreq); //velocity PID isr

  //initialize pwm pins
  pwmPeriod = (1.0 / pwmFreq) * pow(10,8); //calculate period in hundredths of uS
  //SerialUSB.println(pwmPeriod); //debug line
  inH_A.start(pwmPeriod, 0); //test line
  inH_B.start(pwmPeriod, 0); //test line
  inH_C.start(pwmPeriod, 0); //test line

  fillSineTable();

}

int j;
void loop() {

  digitalWrite(9,toggle);
  //updateReqMotorVelocity(); //call this around 10Hz ??

  //SerialUSB.println(reqMotorVelocity);
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

  toggle = !toggle;
}
//----------------------------------------------------------------------------------------/
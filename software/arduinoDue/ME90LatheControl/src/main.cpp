#include <Arduino.h> 
#include <pwm_lib.h>
#include <RunningAverage.h>
#include <PID_v1.h>
#include <SPI.h>
#include <AS5048A.h>

#include <pinSetup.h>
#include <interrupts.h>
#include <DRV8305.h>

using namespace arduino_due::pwm_lib; 

#define MOTOR_LIMIT_TEMP 75 //in deg C
#define ADC_RES 12
#define poles 14

//define SPI confings for all devices
//SPISettings LS73663(1000000, MSBFIRST, SPI_MODE1);//rnd vals

//SPISettings drv8305SPISettings(1000000, MSBFIRST, SPI_MODE1);
AS5048A motorEncoder(CsnMot);

RunningAverage setSpeedAverage(10); //set speed to be sampled as running Average
RunningAverage inputCurrentAverage(30);
RunningAverage potAvergage(10);
RunningAverage TH1Average(5); //set speed to be sampled as running Average


//uC  
bool toggle = 0; //for timing measure
int ADCsamples = 4096;//pow(2,ADC_RES);
const int ISenseRange = 40; // -20A - 0A - 20A mapped to 0.00V - 1.65V -3.33V
const int ISenseOffset = 20;// to shift the ADC value as above
// PIDs
double reqSpindleTorque, quadratureCurrent, directCurrent,vectorAmplitude, inputCurrent;
double tKp = 1.0,
       tKi = 0.00,
       tKd = 0.000;

double reqMotorVelocity, velSetpoint, curMotorVelocity;
double vKp = 0.00, 
       vKi = 0.00, 
       vKd = 0.00;

volatile uint16_t curMotorPosition = 0;
volatile uint16_t prevMotorPosition = 0;

//PID torquePID(input(Isens 0.0 : 15.0 A) , output (0.0 : 1.0 ), setpoint (0.0A - 5.0A))
PID torquePID(&inputCurrent, &vectorAmplitude, &reqSpindleTorque, tKp, tKi, tKd, DIRECT);

//PID velPID(input(velocity RPM) , output (0.0A - 5.0A), setpoint (-200.0 : 1450.0 RPM))
PID velocityPID(&curMotorVelocity, &reqSpindleTorque, &reqMotorVelocity, vKp, vKi, vKd, DIRECT);
int FOCFreq = 3201; //349th prime
int torquePIDFreq = 409; //223th prime
int velocityPIDFreq = 283; //61th prime


//motor
bool motorOverheat = 0;
bool rotorAligned = 0;
volatile bool eStopStatus;
float motorTemperature = 0;
volatile float rotorAngle = 0;
const int pwmFreq = 32000; //in Hz
const int sineTableSize = 1024;
int pwmSineTable[sineTableSize];
int pwmPeriod = 0; //will be calculates in setup in hundredths of uS
int phaseOffsetB = sineTableSize / 3;
int phaseOffsetC = (sineTableSize / 3) * 2;
const int motorEncResolution = 16384; //14bit encoder
const double maxVectorAmplitude = 0.1;
const int maxMotorVelocity = 50; //1450; //in RPM
uint16_t FOCoffset = 0;
word test = 0;

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
// put back in TC3_Handler after testing

String logTable[1024];
int logState = 0;
int ilog = 0;

void TC3_Handler()   // FOC ISR
{
  TC_GetStatus(TC1, 0); // TC_GetStatus to "accept" interrupt parameters (TCn, channel)
  digitalWrite(13, HIGH);//l = !l); //debug line for timing
  uint16_t FOCIndex;
  uint16_t magAngle;
  
  
  rotorAngle = float(motorEncoder.getRotation());

  //calculate magnetic angle
  magAngle = (int(rotorAngle) * poles) & (motorEncResolution - 1);
  //scale it range 0-sineTableSize
  magAngle = (magAngle / float(motorEncResolution)) * (sineTableSize-1);

  //calculate FOCIndex
  //set the vector according to the required direction (rotorAngle +- PI/2)
  
  if (vectorAmplitude < 0){
    FOCIndex = magAngle + (sineTableSize/4);
  }
  else{
    //FOCIndex = (((rotorAngle / motorEncResolution) * (sineTableSize-1) ) * poles) - (sineTableSize/4);
    FOCIndex = magAngle - (sineTableSize/4);
  }

  //...takes about 70us to get here
  //set appropriate duty cycle
  inH_A.set_duty(pwmSineTable[ FOCIndex & (sineTableSize-1) ] * abs(vectorAmplitude));
  inH_B.set_duty(pwmSineTable[ (FOCIndex + phaseOffsetB) & (sineTableSize-1)] * abs(vectorAmplitude) );
  inH_C.set_duty(pwmSineTable[ (FOCIndex + phaseOffsetC) & (sineTableSize-1)] * abs(vectorAmplitude) );
  
  prevMotorPosition = curMotorPosition;
 //Measuring currents and scaling them
  float IphaseA = (float(analogRead(IsenA) / float(ADCsamples-1)) * ISenseRange ) - ISenseOffset;
  float IphaseB = (float(analogRead(IsenB) / float(ADCsamples-1)) * ISenseRange ) - ISenseOffset;
  //float IphaseC = -1*(IphaseA + IphaseB); 
  float IphaseC = (float(analogRead(IsenC) / float(ADCsamples-1)) * ISenseRange ) - ISenseOffset;

  // Clarke transform (power invariant)
  float X = (2*IphaseA - IphaseB - IphaseC)*0.4082;
  float Y = (IphaseB - IphaseC)*0.707106;
  //Z = (IphaseA + IphaseB + IphaseC)*(1/sqrt(3)); //redundant

  //float X = IphaseA * 1.5;
  //float Y = IphaseB * 0.866025403784439 - IphaseC * 0.866025403784439;

  // Park transform

  float theta = (1-(float(magAngle) / sineTableSize)) * 6.283185307179586;
  float co = cos(theta);
  float si = sin(theta);


  directCurrent = co*X + si*Y;
  quadratureCurrent = co*Y - si*X;

  if((ilog < 1024) && (logState == 1)){
    logTable[ilog] = String(ilog) + "," + String(theta) + "," + IphaseA + "," + IphaseB + "," +  IphaseC + "," + X + "," + Y + "," + directCurrent + "," + quadratureCurrent;
    ilog++;
  }
  if((ilog >= 1023) && (logState == 1)){ 
    logState = 2;
  }
  digitalWrite(13, LOW);

}

void TC4_Handler() // torque PID ISR
{
  TC_GetStatus(TC1, 1);
  digitalWrite(12, HIGH);//h = !h);
  
  digitalWrite(12, LOW);

  //torquePID.Compute();
}

void TC5_Handler() // velocity PID ISR
{
  TC_GetStatus(TC1, 2);
  curMotorPosition = int(rotorAngle);

  //digitalWrite(11, c = !c);
  potAvergage.addValue( (float(analogRead(A8))/4096) * 0.25 );
  vectorAmplitude = potAvergage.getAverage(); //direct control for a quick test

  //calculate motor velocity in steps / s
  curMotorVelocity = (curMotorPosition - prevMotorPosition) * velocityPIDFreq;
    if(abs(curMotorVelocity) < abs(motorEncResolution/2)*velocityPIDFreq){
      setSpeedAverage.addValue(curMotorVelocity);
    }

  prevMotorPosition = rotorAngle;
  //velocityPID.Compute();
}



//----------------------------------------------------------------------------------------/

bool alignRotor(){
    // align magnetic field and record encoder pos for FOC
    float vectorRamp = 0;
    word rotation = motorEncoder.getRawRotation();

    Serial.println("alignign rotor");
    //ramp up the appropriate duty cycle to line up one magnetic pole
    while(vectorRamp <= 0.25){
        inH_A.set_duty(pwmSineTable[ 0 ] * float(vectorRamp));
        inH_B.set_duty(pwmSineTable[ (0 + phaseOffsetB) & (sineTableSize-1)] * float(vectorRamp));
        inH_C.set_duty(pwmSineTable[ (0 + phaseOffsetC) & (sineTableSize-1)] * float(vectorRamp));
        vectorRamp += 0.0001;
        //Serial.println(rotation); // degbug line
        delay(1);
    }
 
    delay(1000); //give the rotor time to catch up, just in case 
    rotation = motorEncoder.getRawRotation();
    Serial.print("pole found at:");
    Serial.println(rotation);
    
    motorEncoder.setZeroPosition(rotation); //record the angle of the pole alignament
    
    Serial.print("now set to");
    Serial.println(motorEncoder.getRotation());

    bool aligned = true;

    // now let the rotor rest
    inH_A.set_duty(0); //test line
    inH_B.set_duty(0); //test line
    inH_C.set_duty(0); //test line

    return aligned;

}

void eStopPressed(){
  //disables Gates when eStop is pushed+
  //digitalWrite (ENGate, 0);
  torquePID.SetMode(MANUAL);
  velocityPID.SetMode(MANUAL);
  eStopStatus = 0;
  Serial.println("eSTOP engaged!"); //debug Line

}

void serialDataSend (){
  //transmit data package

}

void updateReqMotorVelocity (){
  //check the required direction
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
      //pwmSineTable[i] = (sin( (float(i) / float(sineTableSize)) * 2*PI ) + 1)/2 * pwmPeriod;
      float theta = float (i) / (sineTableSize - 1);
      float sinTheta = (1+ sin(theta * 2 * PI)) / 2;
      pwmSineTable[i] = int(sinTheta * pwmPeriod);
      //Serial.println(pwmSineTable[i]); //debug line
  }
  Serial.println("LUT filled");
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

  Serial.begin(115200);
  while(!Serial);
  Serial.println("serial up");

  SPI.begin();

  // initialize the spindle motor drive
  delay(100);
  initSpindleDrv();
  delay(100);

  motorEncoder.init();

  Serial.println("motor encoder initialized");

  torquePID.SetMode(AUTOMATIC);
  torquePID.SetTunings(tKp, tKi, tKd);
  torquePID.SetOutputLimits(-maxVectorAmplitude, maxVectorAmplitude);
  torquePID.SetSampleTime(1);
  //PID lib is in millisecond. will need fixing

  /*
  if (digitalRead(RUN_PIN == 0)){ // reads eStopStatus only if the RUN is disabled
    eStopStatus = digitalRead(eStop);
  }
  else{
    eStopStatus = 0; //otherwist imposes it
  }
  */

  //initialize pwm pins
  pwmPeriod = (1.0 / pwmFreq) * pow(10,8); //calculate period in hundredths of uS
  Serial.print("pwm period = ");
  Serial.println(pwmPeriod); //debug line

  // start the PWMs at 0 duty
  inH_A.start(pwmPeriod, 0);
  inH_B.start(pwmPeriod, 0);
  inH_C.start(pwmPeriod, 0);

  Serial.println("pwm set");

  fillSineTable(); //now use the period to fill the FOC phase lookup table

  //call this every time for the moment, but will need to be called upon request
  delay(10);
  rotorAligned = alignRotor();
  
  //configure the timer and external interrupts
  configInterrupts(FOCFreq, torquePIDFreq, velocityPIDFreq);

  //Serial.println(FOCoffset); //debug line
  torquePID.SetMode(MANUAL);
  torquePID.SetTunings(tKp, tKi, tKd);
  torquePID.SetOutputLimits(-maxVectorAmplitude, maxVectorAmplitude);
  torquePID.SetSampleTime(1);

}


void loop() {

  //digitalWrite(9,toggle);
  //updateReqMotorVelocity(); //call this around 10Hz ??

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
  

  //Serial.print(quadratureCurrent);
  //Serial.print(',');
  //Serial.print(directCurrent);
  //Serial.print(',');
  //Serial.print(inputCurrent);

  //Serial.print(60*(setSpeedAverage.getAverage() / motorEncResolution));
  //Serial.print('\n');

  delay(5);

  //Serial.println(reqMotorVelocity);

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

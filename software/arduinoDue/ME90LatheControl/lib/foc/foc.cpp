#include <Arduino.h>
#include "pinSetup.h"
#include "AS5048A.h"
#include "config.h"
#include "pwm.h"
#include "adc.h"
#include "interrupts.h"
#include "linEncoder.h"

uint16_t rotorAngle = 0;
int pwmSineTable[SINE_TABLE_SIZE];

int phaseOffsetB = SINE_TABLE_SIZE / 3;
int phaseOffsetC = (SINE_TABLE_SIZE / 3) * 2;

uint16_t FOCOffset = motorEncResolution/4;
uint16_t sineIndexScaler = motorEncResolution / SINE_TABLE_SIZE;

uint16_t updateFoc(){
    
    REG_PIOD_SODR |= (0x01 << 7); //pin 11 high for timing
    uint16_t FOCIndex;
    uint16_t FOCAngle;
    uint16_t magAngle;
    
    rotorAngle = motorEncoder.getRotation();//this takes 26.4us ! will cange to a HW interrupt
    uint16_t thetaAbs = (encoderOffset + rotorAngle) & (motorEncResolution-1);
    rotorAngle = rotorAngle + _lin_encoder[thetaAbs >> 4]; //the linearization map is encoderRes/16 long
    
    //calculate magnetic angle
    magAngle = (rotorAngle * POLES) & (motorEncResolution - 1);
    //calculate FOCAngle (rename to quadratureAngle ??) ---
    //set the vector according to the required direction (rotorAngle +- PI/2)
    if (vectorAmp < 0){
        FOCAngle = (magAngle + FOCOffset) & (motorEncResolution-1);
    }
    else{
        FOCAngle = (magAngle - FOCOffset) & (motorEncResolution-1);
    }

    //scale it in range 0-SINE_TABLE_SIZE-1
    FOCIndex = FOCAngle / sineIndexScaler;

    //update the pwm with appropriate duty
    int scaler = abs(vectorAmp); //just removing the sign
    updatePWM((pwmSineTable[ FOCIndex & (SINE_TABLE_SIZE-1) ] * scaler) >> 12, // x >> 12 =  x / (2^12) = x / ADC_RES (but is faster and less readable)
             (pwmSineTable[ (FOCIndex + phaseOffsetB) & (SINE_TABLE_SIZE-1)] * scaler ) >> 12,
             (pwmSineTable[ (FOCIndex + phaseOffsetC) & (SINE_TABLE_SIZE-1)] * scaler ) >> 12
            );

    //this is going to be the next bit to work on. 
    // the code works but shall be refactored to remove all float math

    // Measuring currents and scaling them
    //float IphaseA = getCurrentA();
    //float IphaseB = getCurrentB();
    //float IphaseC = getCurrentC();
    /*/ float IphaseC = -IphaseA - IphaseB;
    // Clarke transform (power invariant)
    float X = (2 * IphaseA - IphaseB - IphaseC) * 0.4082;
    float Y = (IphaseB - IphaseC) * 0.707106; // 1/sqrt(2)
    // Z = (IphaseA + IphaseB + IphaseC)*(1/sqrt(3)); //redundant
    // Park transform - replace trig with LUT
    float theta = (1 - (float(magAngle) / SINE_TABLE_SIZE)) * 6.283185307179586;
    
    //timing measurement
    
    float co = cos(theta);
    float si = sin(theta);

    directCurrent = co * X + si * Y;
    quadratureCurrent = co * Y - si * X;
    */
    // digitalWrite(11, LOW); 
    /*/debug logger
    if((ilog < 1024) && (logState == 1)){
        logTable[ilog] = String(ilog) + "," + String(theta) + "," + IphaseA + "," + IphaseB + "," +  IphaseC + "," + X + "," + Y + "," + directCurrent + "," + quadratureCurrent;
        ilog++;
    }
    if((ilog >= 1023) && (logState == 1)){ 
        logState = 2;
    }
    */
    REG_PIOD_CODR |= (0x01 << 7); //pin 11 low for timing

    return rotorAngle;
}

// align magnetic field and record encoder pos for FOC !! still with floats !!
uint16_t alignRotor(){

    Serial.println("alignign rotor");
    //ramp up the appropriate duty cycle to line up one magnetic pole
    for(int vectorRamp=0;vectorRamp<1024;vectorRamp++){
        updatePWM((pwmSineTable[ 0 & (SINE_TABLE_SIZE-1) ] * vectorRamp) >> 12,
                  (pwmSineTable[ (0 + phaseOffsetB) & (SINE_TABLE_SIZE-1)] * vectorRamp ) >> 12,
                  (pwmSineTable[ (0 + phaseOffsetC) & (SINE_TABLE_SIZE-1)] * vectorRamp ) >> 12
                 );
       //Serial.println(rotation); // degbug line
        delay(1);
    }
 
    delay(1000); //give the rotor time to catch up, just in case 
    word rotation = motorEncoder.getRawRotation();
    Serial.print("pole found at absolute angle:");
    Serial.println(rotation);
    
    motorEncoder.setZeroPosition(rotation); //record the angle of the pole alignament
    
    Serial.print("now set to");
    Serial.println(motorEncoder.getRotation()); //this should say 0

    // let the rotor rest
    updatePWM(0,0,0);
    return rotation;

}

// maps encoder linearity vs magnetic poles
uint16_t mapEncoder(){
    word rotation = motorEncoder.getRawRotation(); //not needed?

    Serial.println("alignign rotor...");
    //ramp up the appropriate duty cycle to line up one magnetic pole
    for(int vectorRamp=0;vectorRamp<1280;vectorRamp++){
        updatePWM((pwmSineTable[ 0 & (SINE_TABLE_SIZE-1) ] * vectorRamp) >> 12,
                  (pwmSineTable[ (0 + phaseOffsetB) & (SINE_TABLE_SIZE-1)] * vectorRamp ) >> 12,
                  (pwmSineTable[ (0 + phaseOffsetC) & (SINE_TABLE_SIZE-1)] * vectorRamp ) >> 12
                 );
       //Serial.println(rotation); // degbug line
        delay(1);
    }
 
    delay(100); //give the rotor time to catch up, just in case 
    rotation = motorEncoder.getRawRotation();
    Serial.print("pole found at absolute value: ");
    Serial.println(rotation);
    
    motorEncoder.setZeroPosition(rotation); //set the offset to align encoder 0 to magnetic pole alignament
    Serial.print("offset: ");
    Serial.println(rotation);

    // spin the rotor in open loop for a full revolution and record the encoder value
    for (int i=0;i<SINE_TABLE_SIZE*POLES;i++){
        updatePWM((pwmSineTable[ i & (SINE_TABLE_SIZE-1) ] * 1280) >> 12,
                  (pwmSineTable[ (i + phaseOffsetB) & (SINE_TABLE_SIZE-1)] * 1280 ) >> 12,
                  (pwmSineTable[ (i + phaseOffsetC) & (SINE_TABLE_SIZE-1)] * 1280 ) >> 12
                 );
        Serial.println(String(i) + "," + String(motorEncoder.getRawRotation())); // printing the abs linearity map
        delayMicroseconds(25);
    }


    // now let the rotor rest
    updatePWM(0,0,0);
    return rotation;
}

// fills sine LUT - shifted positive and scaled to the max pwm dutycycle value 
void fillSineTable (){
  for (int i=0; i < SINE_TABLE_SIZE; i++){
      float theta = float (i) / SINE_TABLE_SIZE;
      float sinTheta = (1+ sin(theta * 2 * PI)) / 2;
      pwmSineTable[i] = int(sinTheta * MAX_PWM_DUTY);
      //Serial.println(pwmSineTable[i]); //debug line
  }
  Serial.println("LUT filled");
}
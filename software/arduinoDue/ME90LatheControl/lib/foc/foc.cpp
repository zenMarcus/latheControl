#include <Arduino.h>
#include "pinSetup.h"
#include "AS5048A.h"
#include "config.h"
#include "pwm.h"
#include "adc.h"
#include "interrupts.h"
#include "linEncoder.h"

int pwmSineTable[SINE_TABLE_SIZE];

static int phaseOffsetB = SINE_TABLE_SIZE / 3;
static int phaseOffsetC = (SINE_TABLE_SIZE / 3) * 2;

static uint16_t FOCOffset = motorEncResolution/4;
static uint16_t sineIndexScaler = motorEncResolution / SINE_TABLE_SIZE;

// TODO remove magic numbers
// updates the motor velocity as average of the last n (theta - prevTheta), n is a power of 2
// the idea is to avoid dividing as it's slow and quantized (integers), use deltaTheta as a proxy for velocity
// this may not work on the spindle encoder, it may neet to read position every n interrupts
void updateMotorVelocity(){
    const uint8_t power = 2;                           // the exponent of our power of 2 
    const uint8_t samples = 2 << power;                // forces the samples to be a power of 2
    static int16_t deltaThetaBuf[samples];              // define a buffer to average, no need to be circular
    static uint8_t i = 0;                               // just a counter to mange the buffer
    int16_t deltaTheta = rotorAngle - prevRotorAngle;   // theta is just the common name for the angular position

    if (abs(deltaTheta) < (motorEncResolution >> 2)){   // this means there was no wrap around
        deltaThetaBuf[i] = deltaTheta;                  // straight in the buffer
    
    } else if( deltaTheta < 0){                         // if there was a wrap around we need to know the rotation direction to calc deltaTheta
            deltaThetaBuf[i] = rotorAngle + (motorEncResolution - prevRotorAngle);
    } else{
            deltaThetaBuf[i] = (motorEncResolution - rotorAngle) + prevRotorAngle;
    }

    i++;
    i = i & (samples-1);                                  // increment counter and wrap around
    for(uint8_t j=0;j<samples;j++){                     // calc sum of buffer
        motorVelocity = motorVelocity + deltaThetaBuf[j];
    }
    motorVelocity = motorVelocity >> power;             // and average it, here you get why we like a power of 2
}

uint16_t updateFoc(){
    uint16_t FOCIndex;
    uint16_t qVectorAngle;
    uint16_t magAngle;

    REG_PIOD_SODR |= (0x01 << 7);   //set pin 11 high for timing

    // Measuring currents and scaling them
    int16_t IphaseA = getCurrentA();
    int16_t IphaseB = getCurrentB();
    int16_t IphaseC = getCurrentC();

    rotorAngle = motorEncoder.getRotation();                                            //this takes 26.4us! annoying but good enough for now
    uint16_t thetaAbs = (encoderOffset + rotorAngle) & (motorEncResolution-1);          //calc the absolute encoder angle
    rotorAngle = (rotorAngle + _lin_encoder[thetaAbs >> 4]) & (motorEncResolution-1);   //apply the linearization map, map size is encoderRes/16 long
    
    updateMotorVelocity();          // calc velocity
    prevRotorAngle = rotorAngle;    // update position for next loop

    // now we do the FOC magig
    //calculate magnetic angle
    magAngle = (rotorAngle * POLES) & (motorEncResolution - 1);
    
    //calculate quadratureAngle according to the required direction (magnetic angle +- PI/2)
    if (vectorAmp < 0){
        qVectorAngle = (magAngle + FOCOffset) & (motorEncResolution-1);
    }
    else{
        qVectorAngle = (magAngle - FOCOffset) & (motorEncResolution-1);
    }

    //scale it in range 0-SINE_TABLE_SIZE-1
    FOCIndex = qVectorAngle / sineIndexScaler;
    
    
    //update the pwm with appropriate duty
    int scaler = abs(vectorAmp); //just removing the sign
    updatePWM((pwmSineTable[ FOCIndex & (SINE_TABLE_SIZE-1) ] * scaler) >> 12, // x >> 12 =  x / (2^12) = x / ADC_RES (but is faster and less readable)
             (pwmSineTable[ (FOCIndex + phaseOffsetB) & (SINE_TABLE_SIZE-1)] * scaler ) >> 12,
             (pwmSineTable[ (FOCIndex + phaseOffsetC) & (SINE_TABLE_SIZE-1)] * scaler ) >> 12
            );

    // TODO: this is going to be the next bit to work on. 
    // the code works but shall be refactored to remove all float math


    /*/ float IphaseC = -IphaseA - IphaseB;

    // Clarke transform (power invariant)
    float X = (2 * IphaseA - IphaseB - IphaseC) * 0.4082; //(1 / sqrt(6));
    float Y = (IphaseB - IphaseC) * 0.707106; // 1/sqrt(2)
    // Z = (IphaseA + IphaseB + IphaseC)*(1/sqrt(3)); //redundant

    // Park transform - replace trig with LUT
    float theta = (1 - (float(magAngle) / SINE_TABLE_SIZE)) * 6.283185307179586;    
    float co = cos(theta);
    float si = sin(theta);

    directCurrent = co * X + si * Y;
    quadratureCurrent = co * Y - si * X;
    */
    //debug logger
    if((ilog < 1024) && (logState == 1)){
        logA[ilog] = IphaseA;
        logB[ilog] = IphaseB;
        logC[ilog] = IphaseC;
        ilog++;
    }
    if((ilog >= 1023) && (logState == 1)){ 
        logState = 2;
    }
    
    REG_PIOD_CODR |= (0x01 << 7); //pin 11 low for timing
    return rotorAngle;
}
// TODO: need to join align rotor and map encoder with a switch
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
    
    motorEncoder.setZeroPosition(rotation); //set the offset to align encoder 0 to magnetic pole alignament
    
    Serial.print("now set to");
    Serial.println(motorEncoder.getRotation()); //this should say 0

    // let the rotor rest
    updatePWM(0,0,0);
    return rotation;

}



// maps encoder linearity vs magnetic poles
uint16_t mapEncoder(){
    //word rotation = motorEncoder.getRawRotation(); //not needed?

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
    word rotation = motorEncoder.getRawRotation();
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
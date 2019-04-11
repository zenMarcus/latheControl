#include <Arduino.h>
#include "pinSetup.h"
#include "AS5048A.h"
#include "config.h"
#include "pwm.h"
#include "adc.h"
#include "interrupts.h"

float rotorAngle = 0;
//uint16_t FOCoffset = 0;
int pwmSineTable[SINE_TABLE_SIZE];

int phaseOffsetB = SINE_TABLE_SIZE / 3;
int phaseOffsetC = (SINE_TABLE_SIZE / 3) * 2;

uint16_t updateFoc(){
    digitalWrite(11, HIGH);
    
    uint16_t FOCIndex;
    uint16_t magAngle;
  
    rotorAngle = float(motorEncoder.getRotation());
    //vectorAmplitude = float(getMotorTemp2()) / 4096 / 4;
    vectorAmplitude =  0.2;
    //calculate magnetic angle
    magAngle = (int(rotorAngle) * POLES) & (motorEncResolution - 1);
    //scale it range 0-SINE_TABLE_SIZE
    magAngle = (magAngle / float(motorEncResolution)) * (SINE_TABLE_SIZE-1);

    //calculate FOCIndex
    //set the vector according to the required direction (rotorAngle +- PI/2)
    if (vectorAmplitude < 0){
        FOCIndex = magAngle + (SINE_TABLE_SIZE/4);
    }
    else{
        //FOCIndex = (((rotorAngle / motorEncResolution) * (SINE_TABLE_SIZE-1) ) * POLES) - (SINE_TABLE_SIZE/4);
        FOCIndex = magAngle - (SINE_TABLE_SIZE/4);
    }

    updatePWM((pwmSineTable[ FOCIndex & (SINE_TABLE_SIZE-1) ] * abs(vectorAmplitude)),
             (pwmSineTable[ (FOCIndex + phaseOffsetB) & (SINE_TABLE_SIZE-1)] * abs(vectorAmplitude) ),
             (pwmSineTable[ (FOCIndex + phaseOffsetC) & (SINE_TABLE_SIZE-1)] * abs(vectorAmplitude) )
            );


    // Measuring currents and scaling them
    float IphaseA = getCurrentA();
    float IphaseB = getCurrentB();
    float IphaseC = getCurrentC();
    // float IphaseC = -IphaseA - IphaseB;
    // Clarke transform (power invariant)
    float X = (2 * IphaseA - IphaseB - IphaseC) * 0.4082;
    float Y = (IphaseB - IphaseC) * 0.707106;
    // Z = (IphaseA + IphaseB + IphaseC)*(1/sqrt(3)); //redundant
    // Park transform - replace trig with LUT
    float theta = (1 - (float(magAngle) / SINE_TABLE_SIZE)) * 6.283185307179586;
    digitalWrite(11, LOW);
    float co = cos(theta);
    float si = sin(theta);

    directCurrent = co * X + si * Y;
    quadratureCurrent = co * Y - si * X;

    //debug logger
    if((ilog < 1024) && (logState == 1)){
        logTable[ilog] = String(ilog) + "," + String(theta) + "," + IphaseA + "," + IphaseB + "," +  IphaseC + "," + X + "," + Y + "," + directCurrent + "," + quadratureCurrent;
        ilog++;
    }
    if((ilog >= 1023) && (logState == 1)){ 
        logState = 2;
    }
    
   return rotorAngle;
}

// align magnetic field and record encoder pos for FOC
bool alignRotor(){
    float vectorRamp = 0;
    word rotation = motorEncoder.getRawRotation();

    Serial.println("alignign rotor");
    //ramp up the appropriate duty cycle to line up one magnetic pole
    while(vectorRamp <= 0.25){
        updatePWM( (pwmSineTable[ 0 ] * float(vectorRamp)),
                  (pwmSineTable[ (0 + phaseOffsetB) & (SINE_TABLE_SIZE-1)] * float(vectorRamp)),
                  (pwmSineTable[ (0 + phaseOffsetC) & (SINE_TABLE_SIZE-1)] * float(vectorRamp))
                  );

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
    updatePWM(0,0,0);

    return aligned;

}

// fills sine LUT - shifted positive and scaled to the max pwm dutycycle value 
void fillSineTable (){
  for (int i=0; i < SINE_TABLE_SIZE; i++){
      float theta = float (i) / (SINE_TABLE_SIZE - 1);
      float sinTheta = (1+ sin(theta * 2 * PI)) / 2;
      pwmSineTable[i] = int(sinTheta * MAX_PWM_DUTY);
      //Serial.println(pwmSineTable[i]); //debug line
  }
  Serial.println("LUT filled");
}
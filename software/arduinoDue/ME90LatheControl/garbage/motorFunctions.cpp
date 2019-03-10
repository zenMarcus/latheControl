#include "Arduino.h"
#include "pinSetup.h"

bool alignRotor(){
    // align magnetic field and record encoder pos for FOC
    float vectorRamp = 0;
    word rotation = motorEncoder.getRawRotation();

    Serial.println("alignign rotor");
    //ramp up the appropriate duty cycle to line up one magnetic pole
    while(vectorRamp <= 0.25){
        inH_A.set_duty(pwmSineTable[ 0 ] * float(vectorRamp));
        inH_B.set_duty(pwmSineTable[ (0 + phaseOffsetB & (sineTableSize-1))] * float(vectorRamp));
        inH_C.set_duty(pwmSineTable[ (0 + phaseOffsetC & (sineTableSize-1))] * float(vectorRamp));
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

    bool aligned = TRUE;

    // now let the rotor rest
    inH_A.set_duty(0); //test line
    inH_B.set_duty(0); //test line
    inH_C.set_duty(0); //test line

    return aligned;

}
#include <Arduino.h>
#include "pinSetup.h"
#include "AS5048A.h"
#include "config.h"
#include "pwm.h"
#include "adc.h"
#include "pid.h"
#include "linEncoder.h"
#include "sinetable.h"

int pwmSineTable[SINE_TABLE_SIZE];

// the motor seem to be wired backwards so offsets are negative here
// this makes little or no difference but helps to validate the FOC vs the analytical model
static int phaseOffsetB = -SINE_TABLE_SIZE / 3;
static int phaseOffsetC = -(SINE_TABLE_SIZE / 3) * 2;

// static uint16_t FOCOffset = motorEncResolution/4;
// static uint16_t sineIndexScaler = motorEncResolution / SINE_TABLE_SIZE; // a scaler from the encoder value down to the corresponding index the pwm look up table
uint16_t prevETheta = 0;

int32_t PWMA, PWMB, PWMC;
int32_t Vd, Vq;
int32_t Iq, Id;
int32_t Ia, Ib;
int32_t Va, Vb, VaScaled;
int32_t vbSqrt;
int16_t refIQ;

PID qPID(0, 0, 0);
PID dPID(0, 0, 0);
PID vPID(0, 0, 0);

// updates the motor velocity as average of the last n (theta - prevTheta), n is a power of 2
// the idea is to avoid dividing as it's slow and quantized (integers), use deltaTheta as a proxy for velocity
// knowing that the deltaTime is constant we can calculate the RPM velocity only where needed, outside of an interrupt
// this may not work on the spindle encoder, it may need to read position every n interrupts
void updateMotorVelocity(){
    int16_t deltaTheta = controlStatus.eTheta - prevETheta;   // theta is just the common name for the angular position

    if (abs(deltaTheta) < (motorEncResolution >> 2)){   // this means there was no wrap around
        motorVelocity = deltaTheta;                     // straight difference
    
    } else if( deltaTheta < 0){                         // if there was a wrap around we need to know the rotation direction to calc deltaTheta
            motorVelocity = controlStatus.eTheta + (motorEncResolution - prevETheta);
    } else{
            motorVelocity = (motorEncResolution - controlStatus.eTheta) + prevETheta;
    }
    if(motorVelocity > -15 && motorVelocity < 15) motorVelocity = 0; //TODO brutal quantization filter , do we need this
    controlStatus.velElec = (controlStatus.velElec + motorVelocity) >> 1;
    // calculate the mech. velocity and finite filter it to smooth it out
    controlStatus.velMec = ((3 * controlStatus.velElec + controlStatus.velElec)>>2)  / POLES;
    if(controlStatus.velMec > -4 && controlStatus.velMec < 4) controlStatus.velMec = 0; //then remove the quantization artefacts at 0 velocity 
}

void setupFoc()
{
    // TODO these max and min limits are for testing. Find out the real ones! 
    qPID.setCoeffs(drvParam.pid_KP, drvParam.pid_KI, drvParam.pid_KD);
    qPID.setMinValue(-750);
    qPID.setMaxValue(750); //max PWM duty cycle (top at 1000)

    dPID.setCoeffs(drvParam.pid_KP, drvParam.pid_KI, drvParam.pid_KD);
    dPID.setMinValue(-750);
    dPID.setMaxValue(750); //max PWM duty cycle (top at 1000)

    vPID.setCoeffs(drvParam.velPid_KP, drvParam.velPid_KI, drvParam.velPid_KD);
    vPID.setMinValue(-750);
    vPID.setMaxValue(750); //max refIq (in ADC counts -2048 to 2048 = -20A to 20A) //TODO, check this out on the datasheet
    
    refIQ = 0;

    controlStatus.IdRef = 0;
    controlStatus.IqRef = 0;
    controlStatus.velRef = 0;
}

uint16_t updateFoc(){

    //uint16_t controlStatus.eTheta;

    REG_PIOD_SODR |= (0x01 << 7);   //set pin 11 high for timing
    //TODO Refactor vectorAmp to velocity reference and test for reverse operation
    vectorAmp = getMotorTemp2();     // the test speed pot is wired to the temp channel 

    controlStatus.mTheta = motorEncoder.getRotation();                                            // this takes 26.4us! annoying but good enough for now
    
    // linearize the encoder postion
    uint16_t thetaAbs = (controlStatus.mTheta) & (motorEncResolution-1);          // calc the absolute encoder angle
    controlStatus.mTheta = (controlStatus.mTheta + _lin_encoder[thetaAbs >> 4]) & (motorEncResolution-1);   // apply the linearization map, map size is encoderRes/16 long
    
    // calculate magnetic angle
    controlStatus.eTheta = (controlStatus.mTheta * POLES) & (motorEncResolution - 1);
    
    // update the velocities 
    updateMotorVelocity();
    prevETheta = controlStatus.eTheta;    // update position for next loop vel calc

    // now we do the FOC magic

    // update the currents (all is kept in adc counts so real values depend on shunt amps.)
    int16_t IphaseA = getCurrentA();
    int16_t IphaseB = getCurrentB();
    int16_t IphaseC = getCurrentC();  // keep the reading for now but IphaseC = -IphaseA - IphaseB may work better

    // previus biased finite filter, the currents are pretty noisy, these are only for info anyways
    controlStatus.Ia = ((3 * controlStatus.Ia) + IphaseA) >> 2;
    controlStatus.Ib = ((3 * controlStatus.Ib) + IphaseB) >> 2;
    controlStatus.Ic = ((3 * controlStatus.Ic) + IphaseC) >> 2;

    // Clarke transform  A,B,C -> alpha,beta
    int32_t alpha = IphaseA;
    int32_t beta = ((IphaseB - IphaseC) * 37837l )/ 65536l; // 37837l / 65536l = 1/sqrt(3)

    // Park transform - from alpha,beta to Id,Iq (512 is the LUT size, 128 is a Pi/2 offeset)
    int32_t si = _sin_times32768[controlStatus.eTheta >> 5];                    //look up the sine value in a 512 table so scale down by 32
    int32_t co = _sin_times32768[((controlStatus.eTheta >> 5) + 128) & 511];    //and the cosine value (shifting by 1/4 of table size)
 
    // using a 2 sample finite filter biased to previous sample ((n-1) + n)/2 (hence the >>1)
    controlStatus.Id =(controlStatus.Id + ((co * alpha + si * beta) >> 15)) >>1;              //calculate and divide by 2^15 = 32768
    controlStatus.Iq =(controlStatus.Iq + ((co * beta - si * alpha) >> 15)) >>1;

    // set the references for the PIDs
    controlStatus.velRef = (3 * controlStatus.velRef + (vectorAmp >> 5)) >> 2 ;     // TODO replace this with proper velocity POT
    if (controlStatus.velRef > 75) controlStatus.velRef = 75;   //TODO 75 encoder counts per interrupt = 1450 rpm this should not be a magic number
    //controlStatus.IqRef = vectorAmp >> 2; //debug line to switch to torque control

    // calculate the velocity PID
    vPID.setRef(controlStatus.velRef);
    controlStatus.IqRef = vPID.calculatePID(controlStatus.velMec);

    // TODO implement bidirectional control
    // if (controlStatus.inReverse) controlStatus.IqRef = -refIQ;
    // else controlStatus.IqRef = refIQ;    

    // Use PI loops to get Vd, Vq values
    // the required torque is the ouptut of velocity PID
    qPID.setRef(controlStatus.IqRef);
    Vq = qPID.calculatePID(controlStatus.Iq);
 
    controlStatus.IdRef = 0; // always zero until we'll be looking to do field weakening (probably never for this project)
    dPID.setRef(controlStatus.IdRef);
    Vd = dPID.calculatePID(controlStatus.Id);

    //Vd and Vq control PWM duty but they could be negative now or after inverse park.
    //This is OK though with SVM technique, not sure about traditional PWM
    controlStatus.Vd = Vd;
    controlStatus.Vq = Vq;

    // update the current theta based on dTheta. its divided by 8 as the code till here takes around 1/8 of the reading interval (omega = dTheta / dTime)
    controlStatus.eTheta = (controlStatus.eTheta + (controlStatus.velElec >> 3)) & (motorEncResolution-1);
    si = _sin_times32768[controlStatus.eTheta >> 5];                    //look up the sine value in a 512 table so scale down by 32
    co = _sin_times32768[((controlStatus.eTheta >> 5) + 128) & 511];    //and the cosine value (shifting by 1/4 of table size = 128)

    //Vq and Vd above were scaled as raw PWM values [0, 1000]
    VaScaled = Vd * co - Vq * si; // keep VaScaled apart we'll reuse it in the next PWM calcs
    Va = VaScaled / 32768;
    Vb = (Vd * si + Vq * co) / 32768;

    //Inverse Clarke to go from Va, Vb to phase PWMs
    PWMA = Va;    
    vbSqrt = (Vb * 56757L); //multiply by sqrt(3) * 32768
    
    //vbSqrt and VaScaled are both scaled up 32768 times at this point
    
    PWMB = (-VaScaled + vbSqrt) / 65536; //now they should be /32768 to get back but there is 
    PWMC = (-VaScaled - vbSqrt) / 65536; //also a /2 in the equation so combined thats /65536

    //SVM style PWM output - This is optional. Can be commented out for traditional PWM
    if (PWMA <= PWMB)
    {
        if (PWMA <= PWMC) //A is smallest of all
        {
            PWMB -= PWMA;
            PWMC -= PWMA;
            PWMA = 0;
        }
        else //C is smallest then
        {
            PWMA -= PWMC;
            PWMB -= PWMC;
            PWMC = 0;
        }
    }
    else
    {
        if (PWMB <= PWMC) //B is smallest
        {
            PWMA -= PWMB;
            PWMC -= PWMB;
            PWMB = 0;
        }
        else //C is the smallest
        {
            PWMA -= PWMC;
            PWMB -= PWMC;
            PWMC = 0;
        }
    }
    
    updatePWM(PWMA,PWMB,PWMC);


    //store the current values for debug in the logger structure
    if((logger.ilog < 1024) && (logger.logState == 1)){
        logger.logA[logger.ilog] = IphaseA;
        logger.logB[logger.ilog] = IphaseB;
        logger.logC[logger.ilog] = IphaseC;
        logger.logAlpha[logger.ilog] = alpha;
        logger.logBeta[logger.ilog] = beta;
        logger.logId[logger.ilog] = controlStatus.Id;
        logger.logIq[logger.ilog] = controlStatus.Iq;
        logger.logTheta[logger.ilog] = controlStatus.eTheta;
        logger.ilog++;
    }
    if((logger.ilog >= 1023) && (logger.logState == 1)){ 
        logger.logState = 2;
    }
    REG_PIOD_CODR |= (0x01 << 7); //pin 11 low for timing

    return controlStatus.mTheta;
}

// TODO: need to join align rotor and map encoder with a switch
// align magnetic field and record encoder pos for FOC !! still with floats !!
uint16_t alignRotor(){

    Serial.println("alignign rotor");
    //TODO try B -> A technique
    for(int vectorRamp=0;vectorRamp<512;vectorRamp++){
        updatePWM((pwmSineTable[ 0 & (SINE_TABLE_SIZE-1) ] * vectorRamp) >> 12,
                  0,
                  0
                 );
       //Serial.println(rotation); // degbug line
        delay(1);
    }
 
    delay(500); //give the rotor time to catch up, just in case 
    word rotation = motorEncoder.getRawRotation();
    Serial.print("pole found at absolute angle:");
    Serial.println(rotation);
    
    // set the offset to align encoder 0 to magnetic pole alignament
    motorEncoder.setZeroPosition(rotation - ((2^14)/4)/POLES); 
    
    Serial.print("now set to");
    Serial.println(motorEncoder.getRotation()); //this should say 0

    // let the rotor rest
    updatePWM(0,0,0);
    return rotation; // This should be stored as encoder offset

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
// currently only used for encoder mapping
void fillSineTable (){
  for (int i=0; i < SINE_TABLE_SIZE; i++){
      float theta = float (i) / SINE_TABLE_SIZE;
      float sinTheta = (1+ sin(theta * 2 * PI)) / 2;
      pwmSineTable[i] = int(sinTheta * MAX_PWM_DUTY);
      //Serial.println(pwmSineTable[i]); //debug line
  }
  Serial.println("LUT filled");
}
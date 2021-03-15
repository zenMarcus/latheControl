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
static uint8_t velCount = 0;

// the motor seem to be wired backwards so offsets are negative here
// this makes little or no difference but helps to validate the FOC vs the analytical model
static int phaseOffsetB = -SINE_TABLE_SIZE / 3;
static int phaseOffsetC = -(SINE_TABLE_SIZE / 3) * 2;

static uint16_t FOCOffset = motorEncResolution/4;
static uint16_t sineIndexScaler = motorEncResolution / SINE_TABLE_SIZE; // a scaler from the encoder value down to the corresponding index the pwm look up table

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

// TODO remove magic numbers
// updates the motor velocity as average of the last n (theta - prevTheta), n is a power of 2
// the idea is to avoid dividing as it's slow and quantized (integers), use deltaTheta as a proxy for velocity
// this may not work on the spindle encoder, it may neet to read position every n interrupts
void updateMotorVelocity(){
    const uint8_t power = 1;                            // the exponent of our power of 2 
    const uint8_t samples = 2 << power;                 // forces the samples to be a power of 2
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
    i = i & (samples-1);                                // increment counter and wrap around
    for(uint8_t j=0;j<samples;j++){                     // calc sum of buffer
        motorVelocity = motorVelocity + deltaThetaBuf[j];
    }
    motorVelocity = motorVelocity >> power;             // and average it, here you get why we like a power of 2
}

void setupFoc()
{
    qPID.setCoeffs(drvParam.pid_KP, drvParam.pid_KI, drvParam.pid_KD);
    qPID.setMinValue(-400);
    qPID.setMaxValue(400); //max PWM duty cycle

    dPID.setCoeffs(drvParam.pid_KP, drvParam.pid_KI, drvParam.pid_KD);
    dPID.setMinValue(-400);
    dPID.setMaxValue(400); //max PWM duty cycle

    vPID.setCoeffs(drvParam.velPid_KP, drvParam.velPid_KI, drvParam.velPid_KD);
    vPID.setMinValue(-350);
    vPID.setMaxValue(350); //max refIq
    
    refIQ = 0;

    // focCounter = 0; // not sure how/if this is used

    controlStatus.IdRef = 0;
    controlStatus.IqRef = 0;
    controlStatus.velRef = 0;
}

uint16_t updateFoc(){
    uint16_t FOCIndex;
    uint16_t qVectorAngle;
    uint16_t magAngle;

    REG_PIOD_SODR |= (0x01 << 7);   //set pin 11 high for timing
    //TODO Refactor vectorAmp to IqSetpoint
    vectorAmp = getMotorTemp2();     // the test speed pot is wired to the temp channel 

    rotorAngle = motorEncoder.getRotation();                                            // this takes 26.4us! annoying but good enough for now
    uint16_t thetaAbs = (encoderOffset + rotorAngle) & (motorEncResolution-1);          // calc the absolute encoder angle
    rotorAngle = (rotorAngle + _lin_encoder[thetaAbs >> 4]) & (motorEncResolution-1);   // apply the linearization map, map size is encoderRes/16 long
    
    updateMotorVelocity();          // TODO change to electrical velocity
    prevRotorAngle = rotorAngle;    // update position for next loop vel calc
    controlStatus.velElec = motorVelocity;

    // now we do the FOC magic
    // calculate magnetic angle
    magAngle = (rotorAngle * POLES) & (motorEncResolution - 1);
    
    // calculate quadratureAngle according to the required direction (magnetic angle +- PI/2)
    if (vectorAmp < 0){
        qVectorAngle = (magAngle + FOCOffset) & (motorEncResolution-1);
    }
    else{
        qVectorAngle = (magAngle - FOCOffset) & (motorEncResolution-1);
    }

    // scale it in range 0-SINE_TABLE_SIZE-1
    FOCIndex = qVectorAngle / sineIndexScaler;    
    
    //update the pwm with appropriate duty
    // int scaler = abs(vectorAmp); //just removing the sign
    // updatePWM((pwmSineTable[ FOCIndex & (SINE_TABLE_SIZE-1) ] * scaler) >> 12, // x >> 12 =  x / (2^12) = x / ADC_RES (but is faster and less readable)
    //          (pwmSineTable[ (FOCIndex + phaseOffsetB) & (SINE_TABLE_SIZE-1)] * scaler ) >> 12,
    //          (pwmSineTable[ (FOCIndex + phaseOffsetC) & (SINE_TABLE_SIZE-1)] * scaler ) >> 12
    //         );

    // update the currents (in adc counts)
    int16_t IphaseA = getCurrentA();
    int16_t IphaseB = getCurrentB();
    int16_t IphaseC = getCurrentC();  // keep the reading for now but IphaseC = -IphaseA - IphaseB may work better

    // TODO try if filtered signals work better
    // biased finite filter, the currents are pretty noisy
    //controlStatus.Ia = ((3 * controlStatus.Ia) + IphaseA) >> 2;
    //controlStatus.Ib = ((3 * controlStatus.Ib) + IphaseB) >> 2;
    //controlStatus.Ic = ((3 * controlStatus.Ic) + IphaseC) >> 2;

    // TODO Clarke transform (power invariant version) A,B,C -> alpha,beta
    // Ia = sin(magAngle + pi/2) rather than sin(magAngle) it may be due to the open loop FOC setup above... we'll see
    // Id - Iq and alpha - beta seem exchanged with each other, don't trying to fix this now
    int32_t alpha = IphaseA;//((2 * IphaseA - IphaseB - IphaseC) * 100000l)/ 244948l;
    int32_t beta = ((IphaseB - IphaseC) * 37837l )/ 65536l; // 37837l / 65536l = 1/sqrt(3)
    // gamma = (IphaseA + IphaseB + IphaseC)*(1/sqrt(3)); //redundant

    // Park transform - from alpha,beta to Id,Iq replace 511, 128 magic numbers
    int32_t si = _sin_times32768[magAngle >> 5];                    //look up the sine value in a 512 table so scale down by 32
    int32_t co = _sin_times32768[((magAngle >> 5) + 128) & 511];    //and the cosine value (shifting by 1/4 of table size)
    // using a 2 sample finite filter
    controlStatus.Id =((3*controlStatus.Id + (co * alpha + si * beta)) >> 15) >>2;              //calculate and divide by 2^15 = 32768
    controlStatus.Iq =((3*controlStatus.Iq + (co * beta - si * alpha)) >> 15) >>2;

    controlStatus.IdRef = 0; // always zero until we'll be looking to do field weakening
    controlStatus.velRef = (3 * controlStatus.velRef + (vectorAmp >> 2)) >>2 ; //remove when the vel pid works
    if (controlStatus.velRef < 6 && controlStatus.velRef > -6) controlStatus.velRef=0;
    // check if it's time to run the velocity pid
    if ((velCount % velPidCadence == 0)){
        vPID.setRef(controlStatus.velRef);
        refIQ = vPID.calculatePID(controlStatus.velElec);
    }
    // velCount++;
    //vPID.setRef(controlStatus.velRef);
    //refIQ = vPID.calculatePID(controlStatus.velElec);

    if (controlStatus.inReverse) controlStatus.IqRef = -refIQ;
    else controlStatus.IqRef = refIQ;    

    //Use PI loops to get Vd, Vq values (why use a temp variable?)
    qPID.setRef(controlStatus.IqRef);
    Vq = qPID.calculatePID(controlStatus.Iq);
 
    dPID.setRef(controlStatus.IdRef);
    Vd = dPID.calculatePID(controlStatus.Id);

    //Vd and Vq control PWM intensity but they could be negative now or after inverse park.
    //This is OK though. It works out.
    controlStatus.Vd = Vd;
    controlStatus.Vq = Vq;

    //TODO may be better to approximate straight after encoder reading
    magAngle = (magAngle + (motorVelocity >> 3)) & (motorEncResolution-1);
    si = _sin_times32768[magAngle >> 5];                    //look up the sine value in a 512 table so scale down by 32
    co = _sin_times32768[((magAngle >> 5) + 128) & 511];    //and the cosine value (shifting by 1/4 of table size)

    //Vq and Vd above were scaled as raw PWM values [0, 1000]
    VaScaled = Vd * co - Vq * si;
    Va = VaScaled / 32768;
    Vb = (Vd * si + Vq * co) / 32768;

    //Step 10 - Inverse Clarke to go from Va, Vb to phase PWMs
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
        logger.logTheta[logger.ilog] = magAngle;
        logger.ilog++;
    }
    if((logger.ilog >= 1023) && (logger.logState == 1)){ 
        logger.logState = 2;
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
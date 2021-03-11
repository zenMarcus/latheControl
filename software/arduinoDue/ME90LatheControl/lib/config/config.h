/*
 * Various configuration options for the motor control program. As much as possible all settings that are done
 * at compile time are stored here.
 */

#ifndef CONFIG_H_
#define CONFIG_H_
//#include <Arduino.h>
#include "AS5048A.h"
#include "pinSetup.h"

//The two values PWM_FREQ and MAX_PWM_DUTY should form an even divisor to the 84Mhz main clock
//The clock used can be from 1x divisor to 255 but will be integer increments
//The equation is 84Mhz / (Freq * Duty * 2)
//So, with 7KHz PWM we get 84Mhz / 7Khz = 12k. It divides evenly.
//The max duty being 1000 means 12k turns into 12. Then, center aligned mode brings it down to 6.
//The register can take a value of 6 so this works out.
//10500 instead gives a register value of 4 so that works too.
//Don't try frequencies that don't divide cleanly into 84Mhz / (Duty * 2)
#define PWM_FREQ			21000 //10500  //21kHz is just out of audible range
#define MAX_PWM_DUTY		1000 //800 //max value duty cycle can take
#define PWM_CLOCK			(PWM_FREQ * MAX_PWM_DUTY * 2ul) //number of PWM ticks per second - autocalculated - don't modify this line
#define PWM_PICO_PER_TICK	(1000000000ul / (PWM_CLOCK / 1000ul)) //Number of picoSeconds per PWM clock tick - trust me, it's relevant

//target dead time in nano seconds. So, 1000 is a micro second. You will need to tune this to the IGBT hardware.
#define PWM_TARGET_DEADTIME	20
//autocalculated from constant parameters. Don't change this, change the input parameters above
#define PWM_DEADTIME		(((PWM_TARGET_DEADTIME * 1000ul) / PWM_PICO_PER_TICK) + 1)

//If the PWM pulse would be shorter than this many nano seconds then suppress it.
#define PWM_BUFFER_NANOS	50
//this is an autocalculated value based on a bunch of the above constants. Do not change this equation.
#define PWM_BUFFER			(((PWM_BUFFER_NANOS * 500ul) / PWM_PICO_PER_TICK) + (PWM_DEADTIME * 2))

//The ADC ports are 12 bits and so range from 0 to 4096 in value
//The current sensors rest around in the middle at 2013
//These below values form a window. If the ADC values of current sensor 1 goes outside this window the PWM fault line will trigger and PWM will shut down
//this forms a reasonably fast fault mechanism but it is not as reliable or fast as a hardware solution. But, if you're using a DMOC then the
//DMOC power hardware has hardware based faulting anyway. This is just a backup mechanism.
//The current sensors on a dmoc have a scaling factor of about 0.5 so each value here is 1/2 an amp. Plan accordingly.
#define ADC_CURR_LOWTHRESH	1100 //right around -453A
#define ADC_CURR_HIGHTHRESH	2900 //right around +447A

#define DRIVE_ENABLE		42
#define ADC_TEMP1			0
#define ADC_TEMP2			4
#define ADC_CURRENT1		1
#define ADC_CURRENT2		2
#define ADC_BUSV			3
#define ADC_MOTORTEMP1		5
#define ADC_MOTORTEMP2		7
#define DIN0				48
#define DIN1				49
#define DIN2				50
#define DIN3				51

#define EEPROM_PAGE			20

#define CFG_BUILD_NUM		1000

#define EEPROM_VER			0x15

// my declarations here
#define POLES 14
#define ADC_SAMPLES 4096
#define SINE_TABLE_SIZE 1024 //MAX_PWM_DUTY+1; //is this right?

extern AS5048A motorEncoder;
extern uint16_t rotorAngle;
extern uint16_t prevRotorAngle;
extern int16_t motorVelocity;
extern uint16_t FOCCadence; 
extern int16_t vectorAmp;           //this is used in the foc
extern uint16_t encoderOffset;

extern uint16_t torquePIDFreq; 
extern uint16_t velocityPIDFreq;

const double maxVectorAmplitude = 0.1; //this may be removed?
const int motorEncResolution = 16384; //14bit encoder
const int maxMotorVelocity = 50; //1450; //in RPM

const int ISenseRange = 40; // -20A - 0A - 20A mapped to 0.00V - 1.65V -3.33V
const int ISenseOffset = 20;// to shift the ADC value as above


// PIDs
extern double reqSpindleTorque, quadratureCurrent, directCurrent,vectorAmplitude, inputCurrent;
extern double tKp, tKi, tKd;

extern double reqMotorVelocity, velSetpoint, curMotorVelocity;
extern double vKp, vKi, vKd;

//debug
extern    uint16_t tempAngle;

struct debugVars
{
    int16_t logA[1024];
    int16_t logB[1024];
    int16_t logC[1024];
    int16_t logAlpha[1024];
    int16_t logBeta[1024];
    int16_t logId[1024];
    int16_t logIq[1024];
    int16_t logTheta[1024];
    int logState;
    int ilog;

};
extern debugVars logger;

struct controlStatus
{
    int16_t Ia;     // current a
    int16_t Ib;     // current b
    int16_t Ic;     // current c
    int32_t Id;     // direct current
    int32_t Iq;     // quadrature current
    int16_t IdDes;  // desired Id
    int16_t IqDes;  // desired Iq
    int16_t Vd;     // direct voltage
    int16_t Vq;     // quadrature voltage
    bool inReverse;  // true = reverse
};
extern controlStatus controlStatus;

// drv8305 paramenters
struct drv8305param
{
    const uint16_t currentBias = 2048 + 58;  // the Shunt amp is biased to 1.65V so need shifting by half range + a little offset on the adc
};

extern drv8305param drvParam;
#endif


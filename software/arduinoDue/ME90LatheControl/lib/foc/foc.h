#ifndef FOC_H_
#define FOC_H_
uint16_t alignRotor();
// change to int alignRotor(float amplitude, uint_8 steps)
// to return the alignament error befor recalibration ?

uint16_t mapEncoder();

void fillSineTable();
// change to void fillSineTable(uint16_t length, unit16_t maxValue)

void updateMotorVelocity();

uint16_t updateFoc();
#endif
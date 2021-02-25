#ifndef FOC_H_
#define FOC_H_
bool alignRotor();
// change to int alignRotor(float amplitude, uint_8 steps)
// to return the alignament error befor recalibration ?

void mapEncoder();

void fillSineTable();
// change to void fillSineTable(uint16_t length, unit16_t maxValue)

uint16_t updateFoc();
#endif
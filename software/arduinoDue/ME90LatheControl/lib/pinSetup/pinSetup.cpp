#include "pinSetup.h"
#include <Arduino.h>

void pinSetup() {
  pinMode(nFault, INPUT);
  pinMode(PWRGD, INPUT);
  pinMode(RUN_PIN, INPUT_PULLUP);
  pinMode(DIR_PIN, INPUT_PULLUP);
  pinMode(eStop, INPUT_PULLUP);

  pinMode(SpnDFLAG, INPUT);
  pinMode(SpnLFLAG, INPUT);
  pinMode(SCS, OUTPUT);
  digitalWrite(SCS, HIGH); // SPI slave sel. should be kept high till used

  pinMode(ENGate, OUTPUT);
  digitalWrite(ENGate, 0);

  pinMode(SS_spin, OUTPUT);
  digitalWrite(SS_spin, HIGH);

  pinMode(SS_x, OUTPUT);
  digitalWrite(SS_x, HIGH);

  pinMode(SS_z, OUTPUT);
  digitalWrite(SS_z, HIGH);

  digitalWrite(SS_z, HIGH);

  pinMode(CsnX, OUTPUT);
  digitalWrite(CsnX, HIGH);

  pinMode(CsnZ, OUTPUT);
  // digitalWrite(CsnZ, HIGH); not sure on SSI

  pinMode(CsnMot, OUTPUT);
  // digitalWrite(CsnMot, HIGH);

  pinMode(CsnSpn, OUTPUT);
  // digitalWrite(CsnSpn, HIGH);

  // TODO check PWM pins and update/remove this
  pinMode(6, OUTPUT); // pwm
  pinMode(7, OUTPUT); // pwm
  pinMode(8, OUTPUT); // pwm

  // testPins
  pinMode(9, OUTPUT);
  pinMode(13, OUTPUT);
  pinMode(12, OUTPUT);
  pinMode(11, OUTPUT);
}

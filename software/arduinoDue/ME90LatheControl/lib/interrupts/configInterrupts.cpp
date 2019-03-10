#include <Arduino.h>
#include "pinSetup.h"

void startTimer(Tc *tc, uint32_t channel, IRQn_Type irq, uint32_t frequency) {
  pmc_set_writeprotect(false);
  pmc_enable_periph_clk((uint32_t)irq);
  TC_Configure(tc, channel, TC_CMR_WAVE | TC_CMR_WAVSEL_UP_RC | TC_CMR_TCCLKS_TIMER_CLOCK4);
  uint32_t rc = VARIANT_MCK/128/frequency; //128 because we selected TIMER_CLOCK4 above
  TC_SetRA(tc, channel, rc/2); //50% high, 50% low
  TC_SetRC(tc, channel, rc);
  TC_Start(tc, channel);
  tc->TC_CHANNEL[channel].TC_IER=TC_IER_CPCS;
  tc->TC_CHANNEL[channel].TC_IDR=~TC_IER_CPCS;
  NVIC_EnableIRQ(irq);
}

void configInterrupts (int _FOCFreq, int _torquePIDFreq, int _velocityPIDFreq){
  //external interrupts
  
  // will turn this on when it's time
  //attachInterrupt(digitalPinToInterrupt(eStop), eStopPressed, FALLING);

  //timer interrupts
  Serial.println("attaching interrupts");
  startTimer(TC1, 0, TC3_IRQn, _FOCFreq); //FOC service routine
  startTimer(TC1, 1, TC4_IRQn, _torquePIDFreq); //torque PID isr
  startTimer(TC1, 2, TC5_IRQn, _velocityPIDFreq); //velocity PID isr
}


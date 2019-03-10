#ifndef interrupts_h
#define interrupts_h

//#define LIBRARY_VERSION 1.0.0
	void startTimer(Tc *tc, uint32_t channel, IRQn_Type irq, uint32_t frequency);

    void configInterrupts(int _FOCFreq, int _torquePIDFreq, int _velocityPIDFreq);
#endif
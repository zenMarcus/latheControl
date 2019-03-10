#include <Arduino.h>
#include "pinSetup.h"

void initSpindleDrv(){
  uint16_t drvCmd = (0 << 15) | (0x7 << 11) | 0x0296; //config drv8305 to 3 pwm mode
  uint16_t drvRes = 0; //to store the drive response bytes
  //Serial.println(drvCmd, BIN);
  /*
  digitalWrite (SCS, HIGH);
  SPI.beginTransaction(drv8305SPISettings);
  digitalWrite (SCS, LOW);
  // reading only, so data sent does not matter ?
  drvRes = SPI.transfer16(drvCmd);
  digitalWrite (SCS, HIGH);
  SPI.endTransaction();
  
  Serial.print("drv8305 status: ");
  Serial.println(drvRes, BIN);
  delay(100);
  */
  digitalWrite (ENGate, HIGH);
  Serial.println("gates enabled");
}
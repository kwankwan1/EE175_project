#pragma once

#if (ARDUINO >= 100)
 #include "Arduino.h"
#else
 #include "WProgram.h"
#endif

#include <Wire.h>

#define MAX30105_ADDRESS          0x57 //7-bit I2C Address
//Note that MAX30102 has the same I2C address and Part ID

#define I2C_SPEED_STANDARD        100000
#define I2C_SPEED_FAST            400000

//Define the size of the I2C buffer based on the platform the user has
#if defined(__AVR_ATmega328P__) || defined(__AVR_ATmega168__)

  //I2C_BUFFER_LENGTH is defined in Wire.H
  #define I2C_BUFFER_LENGTH BUFFER_LENGTH

#elif defined(__SAMD21G18A__)

  //SAMD21 uses RingBuffer.h
  #define I2C_BUFFER_LENGTH SERIAL_BUFFER_SIZE

#else

  //The catch-all default is 32
  #define I2C_BUFFER_LENGTH 32

#endif

class HeartRate {
 public: 
  HeartRate(void);

  boolean begin(TwoWire &wirePort = Wire, uint32_t i2cSpeed = I2C_SPEED_STANDARD, uint8_t i2caddr = MAX30105_ADDRESS);
  void setup(byte powerLevel = 0x1F, byte sampleAverage = 4, byte ledMode = 3, int sampleRate = 400, int pulseWidth = 411, int adcRange = 4096);
  uint32_t getIR(void); //Returns immediate IR value
  bool safeCheck(uint8_t maxTimeToCheck); //Given a max amount of time, check for new data
  void softReset();
  void setPulseAmplitudeRed(uint8_t value);
  void setPulseAmplitudeIR(uint8_t value);

  //FIFO Configuration (page 18)
  void setFIFOAverage(uint8_t samples);
  void enableFIFORollover();
  void disableFIFORollover();
  void setFIFOAlmostFull(uint8_t samples);
  
  
  //FIFO Reading
  uint16_t check(void); //Checks for new data and fills FIFO
  uint8_t available(void); //Tells caller how many new samples are available (head - tail)
  void clearFIFO(void); //Sets the read/write pointers to zero

  // I2C communication
  uint8_t readRegister8(uint8_t address, uint8_t reg);
  void writeregister(uint8_t address, uint8_t reg, uint8_t value);
  void bitMask(uint8_t reg, uint8_t mask, uint8_t thing);

 private:
  TwoWire *Port; //The generic connection to user's chosen I2C hardware
  uint8_t addr;
  
  //activeLEDs is the number of channels turned on, and can be 1 to 3. 2 is common for Red+IR.
  byte activeLEDs; //Gets set during setup. Allows check() to calculate how many bytes to read from FIFO
  uint8_t revisionID; 
  uint32_t readFIFOSample();

   #define STORAGE_SIZE 4 //Each long is 4 bytes so limit this to fit on your micro
  typedef struct Record
  {
    uint32_t red[STORAGE_SIZE];
    uint32_t IR[STORAGE_SIZE];
    byte head;
    byte tail;
  } sense_struct; //This is our circular buffer of readings from the sensor

  sense_struct sense;

};

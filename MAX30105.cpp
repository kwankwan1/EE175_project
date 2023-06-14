#include "MAX30105.h"
// FIFO 
static const uint8_t FIFOWRITEPTR = 	0x04;
static const uint8_t FIFOOVERFLOW = 	0x05;
static const uint8_t FIFOREADPTR = 	0x06;
static const uint8_t FIFODATA =		0x07;

//Mask
static const uint8_t ROLLOVER_MASK = 	0xEF;
static const uint8_t SAMPLEAVG_MASK =	(byte)~0b11100000;
static const uint8_t RESET_MASK = 		0xBF;
static const uint8_t MODE_MASK = 		0xF8;
static const uint8_t ADCRANGE_MASK = 	0x9F;
static const uint8_t SAMPLERATE_MASK = 0xE3;
static const uint8_t PULSEWIDTH_MASK = 0xFC;


// Configuration 
static const uint8_t FIFOCONFIG = 		0x08;
static const uint8_t MODECONFIG = 		0x09;
static const uint8_t PARTICLECONFIG = 	0x0A;    
static const uint8_t LED1_PULSEAMP = 	0x0C;
static const uint8_t LED2_PULSEAMP = 	0x0D;
static const uint8_t MULTILEDCONFIG1 = 0x11;
static const uint8_t REVISIONID = 		0xFE;
static const uint8_t PARTID = 			0xFF;    


//Multi-LED Mode 
static const uint8_t SLOT1_MASK = 		0xF8;
static const uint8_t SLOT_RED_LED = 			0x01;
static const uint8_t SLOT_IR_LED = 				0x02;

static const uint8_t RESET = 			0x40;

HeartRate::HeartRate() {
  // Constructor
}

boolean HeartRate::begin(TwoWire &wirePort, uint32_t i2cSpeed, uint8_t i2caddr) {

  Port = &wirePort; //Grab which port the user wants us to use

  Port->begin();
  Port->setClock(i2cSpeed);

  addr = i2caddr;

  if (readRegister8(addr, PARTID) != 0x15) {    //default value ID = 0x15
    return false;
  }

  revisionID = readRegister8(addr, REVISIONID);
  return true;
}



void HeartRate::softReset(void) {
  bitMask(MODECONFIG, RESET_MASK, RESET);

  // Poll for bit to clear, reset is then complete
  // Timeout after 100ms
  unsigned long startTime = millis();
  while (millis() - startTime < 100)
  {
    uint8_t response = readRegister8(addr, MODECONFIG);
    if ((response & RESET) == 0) break; //We're done!
    delay(1); //Let's not over burden the I2C bus
  }
}

void HeartRate::setPulseAmplitudeRed(uint8_t amplitude) {
  writeregister(addr, LED1_PULSEAMP, amplitude);
}

void HeartRate::setPulseAmplitudeIR(uint8_t amplitude) {
  writeregister(addr, LED2_PULSEAMP, amplitude);
}

void HeartRate::clearFIFO(void) {
  writeregister(addr, FIFOWRITEPTR, 0);
  writeregister(addr, FIFOOVERFLOW, 0);
  writeregister(addr, FIFOREADPTR, 0);
}

void HeartRate::setup(byte powerLevel, byte sampleAverage, byte ledMode, int sampleRate, int pulseWidth, int adcRange) {
  softReset(); //Reset all configuration, threshold, and data registers to POR values

  bitMask(FIFOCONFIG, SAMPLEAVG_MASK, 0x40); //setFIFOAverage
  bitMask(FIFOCONFIG, ROLLOVER_MASK, 0x10); //enable
  bitMask(MODECONFIG, MODE_MASK, 0x07);  //multiled = 0x07
  activeLEDs = ledMode; //Used to control how many bytes to read from FIFO buffer
  bitMask(PARTICLECONFIG, ADCRANGE_MASK, 0x20);  //adc range = 4096
  bitMask(PARTICLECONFIG, SAMPLERATE_MASK, 0x0C); //setSampleRate 400
  bitMask(PARTICLECONFIG, PULSEWIDTH_MASK, 0x03); //PULSEWIDTH = 411
  setPulseAmplitudeRed(powerLevel);
  setPulseAmplitudeIR(powerLevel);
  bitMask(MULTILEDCONFIG1, SLOT1_MASK, SLOT_RED_LED);
  bitMask(MULTILEDCONFIG1, SLOT1_MASK, SLOT_IR_LED);

  clearFIFO(); //Reset the FIFO before we begin checking the sensor
}

//Tell caller how many samples are available
uint8_t HeartRate::available(void)
{
  int8_t numberOfSamples = sense.head - sense.tail;
  if (numberOfSamples < 0) numberOfSamples += STORAGE_SIZE;

  return (numberOfSamples);
}

//Report the most recent IR value
uint32_t HeartRate::getIR(void)
{
  //Check the sensor for new data for 250ms
  if(safeCheck(250))
    return (sense.IR[sense.head]);
  else
    return(0); //Sensor failed to find new data
}

uint16_t HeartRate::check(void)
{
  //Read register FIDO_DATA in (3-byte * number of active LED) chunks
  //Until FIFO_RD_PTR = FIFO_WR_PTR

  byte readPointer = readRegister8(addr, FIFOREADPTR); 
  byte writePointer = readRegister8(addr, FIFOWRITEPTR); 

  int numberOfSamples = 0;

  //Do we have new data?
  if (readPointer != writePointer)
  {
    //Calculate the number of readings we need to get from sensor
    numberOfSamples = writePointer - readPointer;
    if (numberOfSamples < 0) numberOfSamples += 32; //Wrap condition

    //We now have the number of readings, now calc bytes to read
    //For this example we are just doing Red and IR (3 bytes each)
    int bytesLeftToRead = numberOfSamples * activeLEDs * 3;

    //Get ready to read a burst of data from the FIFO register
    Port->beginTransmission(MAX30105_ADDRESS);
    Port->write(FIFODATA);
    Port->endTransmission();

    //We may need to read as many as 288 bytes so we read in blocks no larger than I2C_BUFFER_LENGTH
    //I2C_BUFFER_LENGTH changes based on the platform. 64 bytes for SAMD21, 32 bytes for Uno.
    //Wire.requestFrom() is limited to BUFFER_LENGTH which is 32 on the Uno
    while (bytesLeftToRead > 0)
    {
      int toGet = bytesLeftToRead;
      if (toGet > I2C_BUFFER_LENGTH)
      {
        //If toGet is 32 this is bad because we read 6 bytes (Red+IR * 3 = 6) at a time
        //32 % 6 = 2 left over. We don't want to request 32 bytes, we want to request 30.
        //32 % 9 (Red+IR+GREEN) = 5 left over. We want to request 27.

        toGet = I2C_BUFFER_LENGTH - (I2C_BUFFER_LENGTH % (activeLEDs * 3)); //Trim toGet to be a multiple of the samples we need to read
      }

      bytesLeftToRead -= toGet;

      //Request toGet number of bytes from sensor
      Port->requestFrom(MAX30105_ADDRESS, toGet);
      
      while (toGet > 0)
      {
        sense.head++; //Advance the head of the storage struct
        sense.head %= STORAGE_SIZE; //Wrap condition

        byte temp[sizeof(uint32_t)]; //Array of 4 bytes that we will convert into long
        uint32_t tempLong;
        sense.red[sense.head] = readFIFOSample(); //Store this reading into the sense array
        
        if (activeLEDs > 1)
        {
          sense.IR[sense.head] = readFIFOSample();
        }
        toGet -= activeLEDs * 3;
      }

    } //End while (bytesLeftToRead > 0)

  } //End readPtr != writePtr

  return (numberOfSamples); //Let the world know how much new data we found
}

//Check for new data but give up after a certain amount of time
bool HeartRate::safeCheck(uint8_t maxTimeToCheck)
{
  uint32_t markTime = millis();
  
  while(1)
  {
	if(millis() - markTime > maxTimeToCheck) return(false);

	if(check() == true) //We found new data!
	  return(true);

	delay(1);
  }
}

//Mask the given register 
void HeartRate::bitMask(uint8_t reg, uint8_t mask, uint8_t thing)
{
  // Grab current register context
  uint8_t originalContents = readRegister8(addr, reg);

  // Zero-out the portions of the register we're interested in
  originalContents = originalContents & mask;

  // Change contents
  writeregister(addr, reg, originalContents | thing);
}


uint8_t HeartRate::readRegister8(uint8_t address, uint8_t reg) {
  Port->beginTransmission(address);
  Port->write(reg);
  Port->endTransmission(false);

  Port->requestFrom((uint8_t)address, (uint8_t)1); // Request 1 byte
  if (Port->available())
  {
    return(Port->read());
  }
  return (0); //Fail
}

void HeartRate::writeregister(uint8_t address, uint8_t reg, uint8_t value) {
  Port->beginTransmission(address);
  Port->write(reg);
  Port->write(value);
  Port->endTransmission();
}

uint32_t HeartRate::readFIFOSample() {
    byte temp[4]; 
    uint32_t temp32;
    temp[3] = 0;
    temp[2] = Port->read();
    temp[1] =Port->read();
    temp[0] = Port->read();
    memcpy(&temp32, temp, 4);	
    return temp32 & 0x3FFFF;	
}


#include "Marlin.h"
#include "L6470.h"
#include "Configuration.h"


//dSPIN_support.ino - Contains functions used to implement the high-level commands,
//   as well as utility functions for converting real-world units (eg, steps/s) to
//   values usable by the dsPIN controller. Also contains the specialized configuration
//   function for the dsPIN chip and the onboard peripherals needed to use it.

// The value in the ACC register is [(steps/s/s)*(tick^2)]/(2^-40) where tick is 
//  250ns (datasheet value)- 0x08A on boot.
// Multiply desired steps/s/s by .137438 to get an appropriate value for this register.
// This is a 12-bit value, so we need to make sure the value is at or below 0xFFF.
// Changed constant to 0.068719476736 after post on sparkfun site by Nick Gammon
unsigned long AccCalc(float stepsPerSecPerSec) {
//  float temp = stepsPerSecPerSec * 0.137438;
  float temp = stepsPerSecPerSec * 0.068719476736;
  if( (unsigned long) long(temp) > 0x00000FFF) return 0x00000FFF;
  else return (unsigned long) long(temp + 0.5);
}

// The calculation for DEC is the same as for ACC. Value is 0x08A on boot.
// This is a 12-bit value, so we need to make sure the value is at or below 0xFFF.
// Changed constant to 0.068719476736 after post on sparkfun site by Nick Gammon
unsigned long DecCalc(float stepsPerSecPerSec) {
  //float temp = stepsPerSecPerSec * 0.137438;
  float temp = stepsPerSecPerSec * 0.068719476736;
  if( (unsigned long) long(temp) > 0x00000FFF) return 0x00000FFF;
  else return (unsigned long) long(temp + 0.5);
}

// The value in the MAX_SPD register is [(steps/s)*(tick)]/(2^-18) where tick is 
//  250ns (datasheet value)- 0x041 on boot.
// Multiply desired steps/s by .065536 to get an appropriate value for this register
// This is a 10-bit value, so we need to make sure it remains at or below 0x3FF
unsigned long MaxSpdCalc(float stepsPerSec)
{
  float temp = stepsPerSec * .065536;
  if( (unsigned long) long(temp) > 0x000003FF) return 0x000003FF;
  else return (unsigned long) long(temp + 0.5);
}

// The value in the Max Run Speed parameter is [(steps/s)*(tick)]/(2^-28) where tick is 
//  250ns (datasheet value)- 0x041 on boot.
// Multiply desired steps/s by .065536 to get an appropriate value for this register
// This is a 20-bit value, so we need to make sure it remains at or below 0x3FFFF
unsigned long MaxRunSpdCalc(float stepsPerSec)
{
  float temp = stepsPerSec * 67.108864;
  if( (unsigned long) long(temp) > 0x0003FFFF) return 0x0003FFFF;
  else return (unsigned long) long(temp + 0.5);
}


// The value in the MIN_SPD register is [(steps/s)*(tick)]/(2^-24) where tick is 
//  250ns (datasheet value)- 0x000 on boot.
// Multiply desired steps/s by 4.1943 to get an appropriate value for this register
// This is a 12-bit value, so we need to make sure the value is at or below 0xFFF.
unsigned long MinSpdCalc(float stepsPerSec)
{
  float temp = stepsPerSec * 4.1943;
  if( (unsigned long) long(temp) > 0x00000FFF) return 0x00000FFF;
  else return (unsigned long) long(temp + 0.5);
}

// The value in the FS_SPD register is ([(steps/s)*(tick)]/(2^-18))-0.5 where tick is 
//  250ns (datasheet value)- 0x027 on boot.
// Multiply desired steps/s by .065536 and subtract .5 to get an appropriate value for this register
// This is a 10-bit value, so we need to make sure the value is at or below 0x3FF.
unsigned long FSCalc(float stepsPerSec)
{
  float temp = (stepsPerSec * .065536)-.5;
  if( (unsigned long) long(temp) > 0x000003FF) return 0x000003FF;
  else return (unsigned long) long(temp);
}

// The value in the INT_SPD register is [(steps/s)*(tick)]/(2^-24) where tick is 
//  250ns (datasheet value)- 0x408 on boot.
// Multiply desired steps/s by 4.1943 to get an appropriate value for this register
// This is a 14-bit value, so we need to make sure the value is at or below 0x3FFF.
unsigned long IntSpdCalc(float stepsPerSec)
{
  float temp = stepsPerSec * 4.1943;
  if( (unsigned long) long(temp) > 0x00003FFF) return 0x00003FFF;
  else return (unsigned long) long(temp);
}

// When issuing RUN command, the 20-bit speed is [(steps/s)*(tick)]/(2^-28) where tick is 
//  250ns (datasheet value).
// Multiply desired steps/s by 67.106 to get an appropriate value for this register
// This is a 20-bit value, so we need to make sure the value is at or below 0xFFFFF.
unsigned long SpdCalc(float stepsPerSec)
{
  float temp = stepsPerSec * 67.106;
  if( (unsigned long) long(temp) > 0x000FFFFF) return 0x000FFFFF;
  else return (unsigned long)temp;
}

// This simple function shifts a byte out over SPI and receives a byte over
//  SPI. Unusually for SPI devices, the dSPIN requires a toggling of the
//  CS (slaveSelect) pin after each byte array sent. 

// Each byte in the array maps to a device in the SPI daisy chain - they are just 8bit shift registers connected together.
byte dSPIN_Xfer(int device, byte data)
{
  byte reply;
  // generate delay using 13 NOP operations. NOP's take 62.5ns each so 13 * 62.5ns == 812.5ns. The L6470 requires Chip Select (L6470_SS) to remain high for at least 800 nsec (tdisCS in datasheet)
  __asm__("nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t");
  WRITE(L6470_SS, LOW);
  // SPI.transfer() both shifts a byte out on the MOSI pin AND receives a
  //  byte in on the MISO pin.
  for(int i = MOTOR_COUNT - 1; i >= 0; i--) {
    if(i == device)
      reply = SPI.transfer(data);
    else
      SPI.transfer(NULL);
  }
  WRITE(L6470_SS, HIGH);
  return reply;
}


// Generalization of the subsections of the register read/write functionality.
//  We want the end user to just write the value without worrying about length,
//  so we pass a bit length parameter from the calling function.
unsigned long dSPIN_Param(int device, unsigned long value, byte bit_len)
{
  unsigned long ret_val=0;        // We'll return this to generalize this function
                                  //  for both read and write of registers.
  byte byte_len = bit_len/8;      // How many BYTES do we have?
  if (bit_len%8 > 0) byte_len++;  // Make sure not to lose any partial byte values.
  // Let's make sure our value has no spurious bits set, and if the value was too
  //  high, max it out.
  unsigned long mask = 0xffffffff >> (32-bit_len);
  if (value > mask) value = mask;
  // The following three if statements handle the various possible byte length
  //  transfers- it'll be no less than 1 but no more than 3 bytes of data.
  // dSPIN_Xfer() sends a byte out through SPI and returns a byte received
  //  over SPI- when calling it, we typecast a shifted version of the masked
  //  value, then we shift the received value back by the same amount and
  //  store it until return time.
  if (byte_len == 3) {
    ret_val |= dSPIN_Xfer(device, (byte)(value>>16)) << 16;
    //Serial.println(ret_val, HEX);
  }
  if (byte_len >= 2) {
    ret_val |= dSPIN_Xfer(device, (byte)(value>>8)) << 8;
    //Serial.println(ret_val, HEX);
  }
  if (byte_len >= 1) {
    ret_val |= dSPIN_Xfer(device, (byte)value);
    //Serial.println(ret_val, HEX);
  }
  // Return the received values. Mask off any unnecessary bits, just for
  //  the sake of thoroughness- we don't EXPECT to see anything outside
  //  the bit length range but better to be safe than sorry.
  return (ret_val & mask);
}

// Much of the functionality between "get parameter" and "set parameter" is
//  very similar, so we deal with that by putting all of it in one function
//  here to save memory space and simplify the program.
unsigned long dSPIN_ParamHandler(int device, byte param, unsigned long value) {
  unsigned long ret_val = 0;   // This is a temp for the value to return.
  // This switch structure handles the appropriate action for each register.
  //  This is necessary since not all registers are of the same length, either
  //  bit-wise or byte-wise, so we want to make sure we mask out any spurious
  //  bits and do the right number of transfers. That is handled by the dSPIN_Param()
  //  function, in most cases, but for 1-byte or smaller transfers, we call
  //  dSPIN_Xfer() directly.
  switch (param)
  {
    // ABS_POS is the current absolute offset from home. It is a 22 bit number expressed
    //  in two's complement. At power up, this value is 0. It cannot be written when
    //  the motor is running, but at any other time, it can be updated to change the
    //  interpreted position of the motor.
    case dSPIN_ABS_POS:
      ret_val = dSPIN_Param(device, value, 22);
      break;
    // EL_POS is the current electrical position in the step generation cycle. It can
    //  be set when the motor is not in motion. Value is 0 on power up.
    case dSPIN_EL_POS:
      ret_val = dSPIN_Param(device, value, 9);
      break;
    // MARK is a second position other than 0 that the motor can be told to go to. As
    //  with ABS_POS, it is 22-bit two's complement. Value is 0 on power up.
    case dSPIN_MARK:
      ret_val = dSPIN_Param(device, value, 22);
      break;
    // SPEED contains information about the current speed. It is read-only. It does 
    //  NOT provide direction information.
    case dSPIN_SPEED:
      ret_val = dSPIN_Param(device, 0, 20);
      break; 
    // ACC and DEC set the acceleration and deceleration rates. Set ACC to 0xFFF 
    //  to get infinite acceleration/decelaeration- there is no way to get infinite
    //  deceleration w/o infinite acceleration (except the HARD STOP command).
    //  Cannot be written while motor is running. Both default to 0x08A on power up.
    // AccCalc() and DecCalc() functions exist to convert steps/s/s values into
    //  12-bit values for these two registers.
    case dSPIN_ACC: 
      ret_val = dSPIN_Param(device, value, 12);
      break;
    case dSPIN_DEC: 
      ret_val = dSPIN_Param(device, value, 12);
      break;
    // MAX_SPEED is just what it says- any command which attempts to set the speed
    //  of the motor above this value will simply cause the motor to turn at this
    //  speed. Value is 0x041 on power up.
    // MaxSpdCalc() function exists to convert steps/s value into a 10-bit value
    //  for this register.
    case dSPIN_MAX_SPEED:
      ret_val = dSPIN_Param(device, value, 10);
      break;
    // MIN_SPEED controls two things- the activation of the low-speed optimization
    //  feature and the lowest speed the motor will be allowed to operate at. LSPD_OPT
    //  is the 13th bit, and when it is set, the minimum allowed speed is automatically
    //  set to zero. This value is 0 on startup.
    // MinSpdCalc() function exists to convert steps/s value into a 12-bit value for this
    //  register. SetLSPDOpt() function exists to enable/disable the optimization feature.
    case dSPIN_MIN_SPEED: 
      ret_val = dSPIN_Param(device, value, 12);
      break;
    // FS_SPD register contains a threshold value above which microstepping is disabled
    //  and the dSPIN operates in full-step mode. Defaults to 0x027 on power up.
    // FSCalc() function exists to convert steps/s value into 10-bit integer for this
    //  register.
    case dSPIN_FS_SPD:
      ret_val = dSPIN_Param(device, value, 10);
      break;
    // KVAL is the maximum voltage of the PWM outputs. These 8-bit values are ratiometric
    //  representations: 255 for full output voltage, 128 for half, etc. Default is 0x29.
    // The implications of different KVAL settings is too complex to dig into here, but
    //  it will usually work to max the value for RUN, ACC, and DEC. Maxing the value for
    //  HOLD may result in excessive power dissipation when the motor is not running.
    case dSPIN_KVAL_HOLD:
      ret_val = dSPIN_Xfer(device, (byte)value);
      break;
    case dSPIN_KVAL_RUN:
      ret_val = dSPIN_Xfer(device, (byte)value);
      break;
    case dSPIN_KVAL_ACC:
      ret_val = dSPIN_Xfer(device, (byte)value);
      break;
    case dSPIN_KVAL_DEC:
      ret_val = dSPIN_Xfer(device, (byte)value);
      break;
    // INT_SPD, ST_SLP, FN_SLP_ACC and FN_SLP_DEC are all related to the back EMF
    //  compensation functionality. Please see the datasheet for details of this
    //  function- it is too complex to discuss here. Default values seem to work
    //  well enough.
    case dSPIN_INT_SPD:
      ret_val = dSPIN_Param(device, value, 14);
      break;
    case dSPIN_ST_SLP: 
      ret_val = dSPIN_Xfer(device, (byte)value);
      break;
    case dSPIN_FN_SLP_ACC: 
      ret_val = dSPIN_Xfer(device, (byte)value);
      break;
    case dSPIN_FN_SLP_DEC: 
      ret_val = dSPIN_Xfer(device, (byte)value);
      break;
    // K_THERM is motor winding thermal drift compensation. Please see the datasheet
    //  for full details on operation- the default value should be okay for most users.
    case dSPIN_K_THERM: 
      ret_val = dSPIN_Xfer(device, (byte)value & 0x0F);
      break;
    // ADC_OUT is a read-only register containing the result of the ADC measurements.
    //  This is less useful than it sounds; see the datasheet for more information.
    case dSPIN_ADC_OUT:
      ret_val = dSPIN_Xfer(device, 0);
      break;
    // Set the overcurrent threshold. Ranges from 375mA to 6A in steps of 375mA.
    //  A set of defined constants is provided for the user's convenience. Default
    //  value is 3.375A- 0x08. This is a 4-bit value.
    case dSPIN_OCD_TH: 
      ret_val = dSPIN_Xfer(device, (byte)value & 0x0F);
      break;
    // Stall current threshold. Defaults to 0x40, or 2.03A. Value is from 31.25mA to
    //  4A in 31.25mA steps. This is a 7-bit value.
    case dSPIN_STALL_TH: 
      ret_val = dSPIN_Xfer(device, (byte)value & 0x7F);
      break;
    // STEP_MODE controls the microstepping settings, as well as the generation of an
    //  output signal from the dSPIN. Bits 2:0 control the number of microsteps per
    //  step the part will generate. Bit 7 controls whether the BUSY/SYNC pin outputs
    //  a BUSY signal or a step synchronization signal. Bits 6:4 control the frequency
    //  of the output signal relative to the full-step frequency; see datasheet for
    //  that relationship as it is too complex to reproduce here.
    // Most likely, only the microsteps per step value will be needed; there is a set
    //  of constants provided for ease of use of these values.
    case dSPIN_STEP_MODE:
      ret_val = dSPIN_Xfer(device, (byte)value);
      break;
    // ALARM_EN controls which alarms will cause the FLAG pin to fall. A set of constants
    //  is provided to make this easy to interpret. By default, ALL alarms will trigger the
    //  FLAG pin.
    case dSPIN_ALARM_EN: 
      ret_val = dSPIN_Xfer(device, (byte)value);
      break;
    // CONFIG contains some assorted configuration bits and fields. A fairly comprehensive
    //  set of reasonably self-explanatory constants is provided, but users should refer
    //  to the datasheet before modifying the contents of this register to be certain they
    //  understand the implications of their modifications. Value on boot is 0x2E88; this
    //  can be a useful way to verify proper start up and operation of the dSPIN chip.
    case dSPIN_CONFIG: 
      ret_val = dSPIN_Param(device, value, 16);
      break;
    // STATUS contains read-only information about the current condition of the chip. A
    //  comprehensive set of constants for masking and testing this register is provided, but
    //  users should refer to the datasheet to ensure that they fully understand each one of
    //  the bits in the register.
    case dSPIN_STATUS:  // STATUS is a read-only register
      ret_val = dSPIN_Param(device, 0, 16);
      break;
    default:
      ret_val = dSPIN_Xfer(device, (byte)(value));
      break;
  }
  return ret_val;
}

// Multimotor daisy chain version: 
// Each byte in the array maps to a device in the SPI daisy chain - they are just 8bit shift registers connected together.
void dSPIN_Xfer_All(byte data[], byte reply[])
{
  //byte reply[MOTOR_COUNT];

  __asm__("nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t");
  WRITE(L6470_SS, LOW);
  // SPI.transfer() both shifts a byte out on the MOSI pin AND receives a
  //  byte in on the MISO pin.
  for(int i = MOTOR_COUNT - 1; i >= 0; i--) {
    reply[i] = SPI.transfer(data[i]);  // first byte sent eventually goes to the last motor, the first reply comes from last motor
  }
  WRITE(L6470_SS, HIGH); // Latch the sent bytes into the devices
  //return reply;
}


// Multimotor daisy chain version: bit_len is enforced to be the same for all devices in this function.
// Generalization of the subsections of the register read/write functionality.
//  We want the end user to just write the value without worrying about length,
//  so we pass a bit length parameter from the calling function.
void dSPIN_Param_All(unsigned long value[], unsigned long ret_val[], byte bit_len)
{
//  unsigned long ret_val=0;        // We'll return this to generalize this function
  //unsigned long ret_val[MOTOR_COUNT];        // We'll return this to generalize this function
                                  //  for both read and write of registers.
  byte send_val[MOTOR_COUNT];
  byte reply[MOTOR_COUNT];                                  
                                  
  byte byte_len = bit_len/8;      // How many BYTES do we have?
  if (bit_len%8 > 0) byte_len++;  // Make sure not to lose any partial byte values.
  // Let's make sure our value has no spurious bits set, and if the value was too
  //  high, max it out.
  unsigned long mask = 0xffffffff >> (32-bit_len);
  for(int i = 0; i < MOTOR_COUNT; i++) {
    if (value[i] > mask) value[i] = mask;
  }  
  
  if (byte_len == 3) {
    for(int i = 0; i < MOTOR_COUNT; i++) {
  
        send_val[i] = (byte)(value[i] >> 16);
        //Serial.println(ret_val, HEX);
      }

    dSPIN_Xfer_All(send_val, reply);
  
    for(int i = 0; i < MOTOR_COUNT; i++) {
      ret_val[i] |= reply[i] << 16;     
    }  
  }

  if (byte_len >= 2) {
    for(int i = 0; i < MOTOR_COUNT; i++) {
      send_val[i] = (byte)(value[i] >> 8);
    }
        
    dSPIN_Xfer_All(send_val, reply);

    for(int i = 0; i < MOTOR_COUNT; i++) {
      ret_val[i] |= reply[i] << 8;
    }    
  }

  if (byte_len >= 1) {
    for(int i = 0; i < MOTOR_COUNT; i++) {
      send_val[i] = (byte)(value[i]);
    }
      
    dSPIN_Xfer_All(send_val, reply);
            
    for(int i = 0; i < MOTOR_COUNT; i++) {
      ret_val[i] = reply[i];
    }
  }
  
  for(int i = 0; i < MOTOR_COUNT; i++) {
    ret_val[i] = (ret_val[i] & mask);
  }    
}


// Handle params for all devices at one time. Note that when sending using the 
// daisy chain to all devices at once, only a single command can be used.
void dSPIN_ParamHandler_All(byte param, unsigned long value[], unsigned long ret_val[]) {
//  unsigned long ret_val = 0;   // This is a temp for the value to return.
  //unsigned long ret_val[MOTOR_COUNT];   // This is a temp for the value to return.
  // This switch structure handles the appropriate action for each register.
  //  This is necessary since not all registers are of the same length, either
  //  bit-wise or byte-wise, so we want to make sure we mask out any spurious
  //  bits and do the right number of transfers. That is handled by the dSPIN_Param()
  //  function, in most cases, but for 1-byte or smaller transfers, we call
  //  dSPIN_Xfer() directly.
  
  switch (param)
  {
    // ABS_POS is the current absolute offset from home. It is a 22 bit number expressed
    //  in two's complement. At power up, this value is 0. It cannot be written when
    //  the motor is running, but at any other time, it can be updated to change the
    //  interpreted position of the motor.
    case dSPIN_ABS_POS:
      dSPIN_Param_All(value, ret_val, 22);
      break;
    // EL_POS is the current electrical position in the step generation cycle. It can
    //  be set when the motor is not in motion. Value is 0 on power up.
    case dSPIN_EL_POS:
      dSPIN_Param_All(value, ret_val, 9);
      break;
    // MARK is a second position other than 0 that the motor can be told to go to. As
    //  with ABS_POS, it is 22-bit two's complement. Value is 0 on power up.
    case dSPIN_MARK:
      dSPIN_Param_All(value, ret_val, 22);
      break;
    // SPEED contains information about the current speed. It is read-only. It does 
    //  NOT provide direction information.
    case dSPIN_SPEED:
      dSPIN_Param_All(0, ret_val, 20);
      break; 
    // ACC and DEC set the acceleration and deceleration rates. Set ACC to 0xFFF 
    //  to get infinite acceleration/decelaeration- there is no way to get infinite
    //  deceleration w/o infinite acceleration (except the HARD STOP command).
    //  Cannot be written while motor is running. Both default to 0x08A on power up.
    // AccCalc() and DecCalc() functions exist to convert steps/s/s values into
    //  12-bit values for these two registers.
    case dSPIN_ACC: 
      dSPIN_Param_All(value, ret_val, 12);
      break;
    case dSPIN_DEC: 
      dSPIN_Param_All(value, ret_val, 12);
      break;
    // MAX_SPEED is just what it says- any command which attempts to set the speed
    //  of the motor above this value will simply cause the motor to turn at this
    //  speed. Value is 0x041 on power up.
    // MaxSpdCalc() function exists to convert steps/s value into a 10-bit value
    //  for this register.
    case dSPIN_MAX_SPEED:
      dSPIN_Param_All(value, ret_val, 10);
      break;
    // MIN_SPEED controls two things- the activation of the low-speed optimization
    //  feature and the lowest speed the motor will be allowed to operate at. LSPD_OPT
    //  is the 13th bit, and when it is set, the minimum allowed speed is automatically
    //  set to zero. This value is 0 on startup.
    // MinSpdCalc() function exists to convert steps/s value into a 12-bit value for this
    //  register. SetLSPDOpt() function exists to enable/disable the optimization feature.
    case dSPIN_MIN_SPEED: 
      dSPIN_Param_All(value, ret_val, 12);
      break;
    // FS_SPD register contains a threshold value above which microstepping is disabled
    //  and the dSPIN operates in full-step mode. Defaults to 0x027 on power up.
    // FSCalc() function exists to convert steps/s value into 10-bit integer for this
    //  register.
    case dSPIN_FS_SPD:
      dSPIN_Param_All(value, ret_val, 10);
      break;
    // KVAL is the maximum voltage of the PWM outputs. These 8-bit values are ratiometric
    //  representations: 255 for full output voltage, 128 for half, etc. Default is 0x29.
    // The implications of different KVAL settings is too complex to dig into here, but
    //  it will usually work to max the value for RUN, ACC, and DEC. Maxing the value for
    //  HOLD may result in excessive power dissipation when the motor is not running.
/*    case dSPIN_KVAL_HOLD:
      ret_val = dSPIN_Xfer_All((byte*)value);
      break;
    case dSPIN_KVAL_RUN:
      ret_val = dSPIN_Xfer_All((byte*)value);
      break;
    case dSPIN_KVAL_ACC:
      ret_val = dSPIN_Xfer_All((byte*)value);
      break;
    case dSPIN_KVAL_DEC:
      ret_val = dSPIN_Xfer_All((byte*)value);
      break;
    // INT_SPD, ST_SLP, FN_SLP_ACC and FN_SLP_DEC are all related to the back EMF
    //  compensation functionality. Please see the datasheet for details of this
    //  function- it is too complex to discuss here. Default values seem to work
    //  well enough.
    case dSPIN_INT_SPD:
      ret_val = dSPIN_Param_All(value, 14);
      break;
    case dSPIN_ST_SLP: 
      ret_val = dSPIN_Xfer_All((byte*)value);
      break;
    case dSPIN_FN_SLP_ACC: 
      ret_val = dSPIN_Xfer_All((byte*)value);
      break;
    case dSPIN_FN_SLP_DEC: 
      ret_val = dSPIN_Xfer_All((byte*)value);
      break;
*/      
    // K_THERM is motor winding thermal drift compensation. Please see the datasheet
    //  for full details on operation- the default value should be okay for most users.
//    case dSPIN_K_THERM: 
//      ret_val = dSPIN_Xfer_All((byte*)value & 0x0F);
//      break;
    // ADC_OUT is a read-only register containing the result of the ADC measurements.
    //  This is less useful than it sounds; see the datasheet for more information.
/*    case dSPIN_ADC_OUT:
      ret_val = dSPIN_Xfer_All({0, 0, 0});
      break;
*/      
    // Set the overcurrent threshold. Ranges from 375mA to 6A in steps of 375mA.
    //  A set of defined constants is provided for the user's convenience. Default
    //  value is 3.375A- 0x08. This is a 4-bit value.
//    case dSPIN_OCD_TH: 
//      ret_val = dSPIN_Xfer((byte)value & 0x0F);
//      break;
    // Stall current threshold. Defaults to 0x40, or 2.03A. Value is from 31.25mA to
    //  4A in 31.25mA steps. This is a 7-bit value.
//    case dSPIN_STALL_TH: 
//      ret_val = dSPIN_Xfer((byte)value & 0x7F);
//      break;
    // STEP_MODE controls the microstepping settings, as well as the generation of an
    //  output signal from the dSPIN. Bits 2:0 control the number of microsteps per
    //  step the part will generate. Bit 7 controls whether the BUSY/SYNC pin outputs
    //  a BUSY signal or a step synchronization signal. Bits 6:4 control the frequency
    //  of the output signal relative to the full-step frequency; see datasheet for
    //  that relationship as it is too complex to reproduce here.
    // Most likely, only the microsteps per step value will be needed; there is a set
    //  of constants provided for ease of use of these values.
/*    case dSPIN_STEP_MODE:
      ret_val = dSPIN_Xfer_All((byte*)value);
      break;
    // ALARM_EN controls which alarms will cause the FLAG pin to fall. A set of constants
    //  is provided to make this easy to interpret. By default, ALL alarms will trigger the
    //  FLAG pin.
    case dSPIN_ALARM_EN: 
      ret_val = dSPIN_Xfer_All((byte*)value);
      break;
    // CONFIG contains some assorted configuration bits and fields. A fairly comprehensive
    //  set of reasonably self-explanatory constants is provided, but users should refer
    //  to the datasheet before modifying the contents of this register to be certain they
    //  understand the implications of their modifications. Value on boot is 0x2E88; this
    //  can be a useful way to verify proper start up and operation of the dSPIN chip.
    case dSPIN_CONFIG: 
      ret_val = dSPIN_Param_All(value, 16);
      break;
*/      
    // STATUS contains read-only information about the current condition of the chip. A
    //  comprehensive set of constants for masking and testing this register is provided, but
    //  users should refer to the datasheet to ensure that they fully understand each one of
    //  the bits in the register.
    case dSPIN_STATUS:  // STATUS is a read-only register
      //unsigned long zero_array[MOTOR_COUNT];
      //    ret_val = dSPIN_Param(device, 0, 16);
      dSPIN_Param_All(zero_array, ret_val, 16);
      break;
    default:
      byte send_value[MOTOR_COUNT];
      byte reply_value[MOTOR_COUNT];
      for(int j= 0; j < MOTOR_COUNT; j++) {
        send_value[j] = (byte)value[j];
      }
      dSPIN_Xfer_All(send_value, reply_value);
      for(int j= 0; j < MOTOR_COUNT; j++) {
        ret_val[j] = (unsigned long)reply_value[j];
      }
      
      break;
  }
  //return ret_val;
}

// Realize the "set parameter" function, to write to the various registers in
//  the dSPIN chip.
void dSPIN_SetParam(int device, byte param, unsigned long value) {
  dSPIN_Xfer(device, dSPIN_SET_PARAM | param);
  dSPIN_ParamHandler(device, param, value);
}

// Realize the "set parameter" function, to write to the various registers in
//  the dSPIN chip.
void dSPIN_SetParam_All(byte param, unsigned long value[], unsigned long result[]) {
  byte param_reply[MOTOR_COUNT];
  byte param_array[MOTOR_COUNT];
  
  for(int i = 0; i < MOTOR_COUNT; i++) { 
    param_array[i] = dSPIN_SET_PARAM | param;
  }  
  
  dSPIN_Xfer_All(param_array, param_reply);
  dSPIN_ParamHandler_All(param, value, result);
}


// Realize the "get parameter" function, to read from the various registers in
//  the dSPIN chip.
unsigned long dSPIN_GetParam(int device, byte param) {
  dSPIN_Xfer(device, dSPIN_GET_PARAM | param);
  return dSPIN_ParamHandler(device, param, 0);
}

// Realize the "get parameter" function, to read from the various registers in
//  the dSPIN chip.
void dSPIN_GetParam_All(byte param, unsigned long result[]) {
  byte byte_reply[MOTOR_COUNT];
  byte param_array[MOTOR_COUNT];
  //unsigned long zero_array[MOTOR_COUNT] = {0,0,0,0};  
  
  for(int i = 0; i < MOTOR_COUNT; i++) { 
    param_array[i] = dSPIN_GET_PARAM | param;
  }
  dSPIN_Xfer_All(param_array, byte_reply); // reply is discarded
  dSPIN_ParamHandler_All(param, zero_array, result);
}


/*
// Realize the "get parameter" function, to read from the various registers in
//  the dSPIN chip.
unsigned long dSPIN_GetParam_All(byte param[]) {
  dSPIN_Xfer_All(dSPIN_GET_PARAM | param);
  return dSPIN_ParamHandler_All(param, 0);
}
*/

// Enable or disable the low-speed optimization option. If enabling,
//  the other 12 bits of the register will be automatically zero.
//  When disabling, the value will have to be explicitly written by
//  the user with a SetParam() call. See the datasheet for further
//  information about low-speed optimization.
void SetLSPDOpt(int device, boolean enable, unsigned long speed) {
  dSPIN_Xfer(device, dSPIN_SET_PARAM | dSPIN_MIN_SPEED);
  //if (enable) dSPIN_Param(0x1000, 13);
  //else dSPIN_Param(0, 13);
  if (enable) dSPIN_SetParam(device, dSPIN_MIN_SPEED, 0x1000 && MinSpdCalc(speed));
  else dSPIN_SetParam(device, dSPIN_MIN_SPEED, MinSpdCalc(speed));
}
  
// RUN sets the motor spinning in a direction (defined by the constants
//  FWD and REV). Maximum speed and minimum speed are defined
//  by the MAX_SPEED and MIN_SPEED registers; exceeding the FS_SPD value
//  will switch the device into full-step mode.
// The SpdCalc() function is provided to convert steps/s values into
//  appropriate integer values for this function.
void dSPIN_Run(int device, byte dir, unsigned long spd) {
  dSPIN_Xfer(device, dSPIN_RUN | dir);
  if (spd > 0xFFFFF) spd = 0xFFFFF;
  dSPIN_Xfer(device, (byte)(spd >> 16));
  dSPIN_Xfer(device, (byte)(spd >> 8));
  dSPIN_Xfer(device, (byte)(spd));
}

void dSPIN_Run_All(byte dir[], unsigned long spd[]) {
  byte send_val[MOTOR_COUNT];
  byte ret_val[MOTOR_COUNT];

  for(int i = 0; i < MOTOR_COUNT; i++) {
	  send_val[i] = dSPIN_RUN | dir[i];
  }
  dSPIN_Xfer_All(send_val, ret_val);

  for(int i = 0; i < MOTOR_COUNT; i++) {
	  if (spd[i] > 0xFFFFF) spd[i] = 0xFFFFF;
	  send_val[i] = (byte)(spd[i] >> 16);
  }
  dSPIN_Xfer_All(send_val, ret_val);

  for(int i = 0; i < MOTOR_COUNT; i++) {
	  send_val[i] = (byte)(spd[i] >> 8);
  }
  dSPIN_Xfer_All(send_val, ret_val);

  for(int i = 0; i < MOTOR_COUNT; i++) {
	  send_val[i] = (byte)(spd[i]);
  }
  dSPIN_Xfer_All(send_val, ret_val);
}

// STEP_CLOCK puts the device in external step clocking mode. When active,
//  pin 25, STCK, becomes the step clock for the device, and steps it in
//  the direction (set by the FWD and REV constants) imposed by the call
//  of this function. Motion commands (RUN, MOVE, etc) will cause the device
//  to exit step clocking mode.

void dSPIN_Step_Clock(int device, byte dir) {
  dSPIN_Xfer(device, dSPIN_STEP_CLOCK | dir);
}

// MOVE will send the motor n_step steps (size based on step mode) in the
//  direction imposed by dir (FWD or REV constants may be used). The motor
//  will accelerate according the acceleration and deceleration curves, and
//  will run at MAX_SPEED. Stepping mode will adhere to FS_SPD value, as well.
void dSPIN_Move(int device, byte dir, unsigned long n_step) {
  dSPIN_Xfer(device, dSPIN_MOVE | dir);
  if (n_step > 0x3FFFFF) n_step = 0x3FFFFF;
  dSPIN_Xfer(device, (byte)(n_step >> 16));
  dSPIN_Xfer(device, (byte)(n_step >> 8));
  dSPIN_Xfer(device, (byte)(n_step));
}

// Multimotor version
// MOVE will send the motor n_step steps (size based on step mode) in the
//  direction imposed by dir (FWD or REV constants may be used). The motor
//  will accelerate according the acceleration and deceleration curves, and
//  will run at MAX_SPEED. Stepping mode will adhere to FS_SPD value, as well.
void dSPIN_Move_All(byte dir[], unsigned long n_step[]) {
  byte send_value[MOTOR_COUNT];
  byte result[MOTOR_COUNT];
  
  for(int i = 0; i < MOTOR_COUNT; i++) {
    send_value[i] = dSPIN_MOVE | dir[i];
  }
  dSPIN_Xfer_All(send_value, result);
  
  for(int i = 0; i < MOTOR_COUNT; i++) {
    if (n_step[i] > 0x3FFFFF) n_step[i] = 0x3FFFFF;
    send_value[i] = (byte)(n_step[i] >> 16);
  }
  dSPIN_Xfer_All(send_value, result);
  
  for(int i = 0; i < MOTOR_COUNT; i++) {
    if (n_step[i] > 0x3FFFFF) n_step[i] = 0x3FFFFF;
    send_value[i] = (byte)(n_step[i] >> 8);
  }
  dSPIN_Xfer_All(send_value, result);
  
  for(int i = 0; i < MOTOR_COUNT; i++) {
    if (n_step[i] > 0x3FFFFF) n_step[i] = 0x3FFFFF;
    send_value[i] = (byte)(n_step[i]);
  }
  dSPIN_Xfer_All(send_value, result);
  
/*
  dSPIN_Xfer(device, dSPIN_MOVE | dir);
  if (n_step > 0x3FFFFF) n_step = 0x3FFFFF;
  dSPIN_Xfer(device, (byte)(n_step >> 16));
  dSPIN_Xfer(device, (byte)(n_step >> 8));
  dSPIN_Xfer(device, (byte)(n_step));
*/  
  
}


// GOTO operates much like MOVE, except it produces absolute motion instead
//  of relative motion. The motor will be moved to the indicated position
//  in the shortest possible fashion.
void dSPIN_GoTo(int device, unsigned long pos) {
  
  dSPIN_Xfer(device, dSPIN_GOTO);
  if (pos > 0x3FFFFF) pos = 0x3FFFFF;
  dSPIN_Xfer(device, (byte)(pos >> 16));
  dSPIN_Xfer(device, (byte)(pos >> 8));
  dSPIN_Xfer(device, (byte)(pos));
}

// Same as GOTO, but with user constrained rotational direction.
void dSPIN_GoTo_DIR(int device, byte dir, unsigned long pos) {
  
  dSPIN_Xfer(device, dSPIN_GOTO_DIR);
  if (pos > 0x3FFFFF) pos = 0x3FFFFF;
  dSPIN_Xfer(device, (byte)(pos >> 16));
  dSPIN_Xfer(device, (byte)(pos >> 8));
  dSPIN_Xfer(device, (byte)(pos));
}

// GoUntil will set the motor running with direction dir (REV or
//  FWD) until a falling edge is detected on the SW pin. Depending
//  on bit SW_MODE in CONFIG, either a hard stop or a soft stop is
//  performed at the falling edge, and depending on the value of
//  act (either RESET or COPY) the value in the ABS_POS register is
//  either RESET to 0 or COPY-ed into the MARK register.
void dSPIN_GoUntil(int device, byte act, byte dir, unsigned long spd) {
  dSPIN_Xfer(device, dSPIN_GO_UNTIL | act | dir);
  if (spd > 0x3FFFFF) spd = 0x3FFFFF;
  dSPIN_Xfer(device, (byte)(spd >> 16));
  dSPIN_Xfer(device, (byte)(spd >> 8));
  dSPIN_Xfer(device, (byte)(spd));
}

// Similar in nature to GoUntil, ReleaseSW produces motion at the
//  higher of two speeds: the value in MIN_SPEED or 5 steps/s.
//  The motor continues to run at this speed until a rising edge
//  is detected on the switch input, then a hard stop is performed
//  and the ABS_POS register is either COPY-ed into MARK or RESET to
//  0, depending on whether RESET or COPY was passed to the function
//  for act.
void dSPIN_ReleaseSW(int device, byte act, byte dir) {
  dSPIN_Xfer(device, dSPIN_RELEASE_SW | act | dir);
}

// GoHome is equivalent to GoTo(0), but requires less time to send.
//  Note that no direction is provided; motion occurs through shortest
//  path. If a direction is required, use GoTo_DIR().
void dSPIN_GoHome(int device) {
  dSPIN_Xfer(device, dSPIN_GO_HOME);
}

// GoMark is equivalent to GoTo(MARK), but requires less time to send.
//  Note that no direction is provided; motion occurs through shortest
//  path. If a direction is required, use GoTo_DIR().
void dSPIN_GoMark(int device) {
  dSPIN_Xfer(device, dSPIN_GO_MARK);
}

// Sets the ABS_POS register to 0, effectively declaring the current
//  position to be "HOME".
void dSPIN_ResetPos(int device) {
  dSPIN_Xfer(device, dSPIN_RESET_POS);
}

// Reset device to power up conditions. Equivalent to toggling the STBY
//  pin or cycling power.
void dSPIN_ResetDev(int device) {
  dSPIN_Xfer(device, dSPIN_RESET_DEVICE);
}
  
// Bring the motor to a halt using the deceleration curve.
void dSPIN_SoftStop(int device) {
  dSPIN_Xfer(device, dSPIN_SOFT_STOP);
}

byte softStop_temp[MOTOR_COUNT];
void dSPIN_SoftStop_All() {
  byte send_val[MOTOR_COUNT];
  for(int i = 0; i < MOTOR_COUNT; i++) {
    send_val[i] = dSPIN_SOFT_STOP;
  }
  dSPIN_Xfer_All(send_val, softStop_temp);
}

// Stop the motor with infinite deceleration.
void dSPIN_HardStop(int device) {
  dSPIN_Xfer(device, dSPIN_HARD_STOP);
}

// Decelerate the motor and put the bridges in Hi-Z state.
void dSPIN_SoftHiZ(int device) {
  dSPIN_Xfer(device, dSPIN_SOFT_HIZ);
}

// Put the bridges in Hi-Z state immediately with no deceleration.
void dSPIN_HardHiZ(int device) {
  dSPIN_Xfer(device, dSPIN_HARD_HIZ);
}

// Fetch and return the 16-bit value in the STATUS register. Resets
//  any warning flags and exits any error states. Using GetParam()
//  to read STATUS does not clear these values.
unsigned int dSPIN_GetStatus(int device) {
  int temp = 0;
  dSPIN_Xfer(device, dSPIN_GET_STATUS);
  temp = dSPIN_Xfer(device, 0)<<8;
  temp |= dSPIN_Xfer(device, 0);
  return temp;
}


void dSPIN_GetStatus_All(unsigned int result[]) {
  byte send_value[MOTOR_COUNT];
  byte getStatus_temp[MOTOR_COUNT];
  
  for(int i = 0; i < MOTOR_COUNT; i++) {
    send_value[i] = dSPIN_GET_STATUS;
  }
  
  dSPIN_Xfer_All(send_value, getStatus_temp);

  dSPIN_Xfer_All(zero_byte_array, getStatus_temp);
  for(int i = 0; i < MOTOR_COUNT; i++) {
    result[i] = getStatus_temp[i] << 8;
  }
  
  dSPIN_Xfer_All(zero_byte_array, getStatus_temp);
  for(int i = 0; i < MOTOR_COUNT; i++) {
    result[i] |= getStatus_temp[i];
  }
}


boolean dSPIN_IsBusy(int device) {
  unsigned int status = dSPIN_GetStatus(device);
  return ((status & dSPIN_STATUS_BUSY)  == 0);
}

boolean dSPIN_IsBusy_Status(unsigned int status) {
  return ((status & dSPIN_STATUS_BUSY)  == 0);
}


// This is the generic initialization function to set up the Arduino to
//  communicate with the dSPIN chip. 
void dSPIN_init()
{
  // set up the input/output pins for the application.
  //pinMode(10, OUTPUT);  // The SPI peripheral REQUIRES the hardware SS pin-
                        //  pin 10- to be an output. This is in here just
                        //  in case some future user makes something other
                        //  than pin 10 the SS pin.
  //pinMode(SLAVE_SELECT_PIN, OUTPUT);
  SET_OUTPUT(L6470_SS);
//  digitalWrite(SLAVE_SELECT_PIN, HIGH);
  WRITE(L6470_SS, HIGH);
  //pinMode(MOSI, OUTPUT);
    SET_OUTPUT(L6470_MOSI);
  //pinMode(MISO, INPUT);
    SET_INPUT(L6470_MISO);
  //pinMode(SCK, OUTPUT);
    SET_OUTPUT(L6470_SCK);
    
    pinMode(X_L6470_RESET_PIN, OUTPUT);
    pinMode(Y_L6470_RESET_PIN, OUTPUT);
    pinMode(Z_L6470_RESET_PIN, OUTPUT);
    //pinMode(E_L6470_RESET_PIN, OUTPUT);    // Extrider reset is connected to Z axis.
    
    digitalWrite(X_L6470_RESET_PIN, HIGH);
    digitalWrite(Y_L6470_RESET_PIN, HIGH);
    digitalWrite(Z_L6470_RESET_PIN, HIGH);    
    //digitalWrite(E_L6470_RESET_PIN, HIGH);    
    delay(1);
    digitalWrite(X_L6470_RESET_PIN, LOW);
    digitalWrite(Y_L6470_RESET_PIN, LOW);
    digitalWrite(Z_L6470_RESET_PIN, LOW);
    //digitalWrite(E_L6470_RESET_PIN, LOW);
    delay(1);    
    digitalWrite(X_L6470_RESET_PIN, HIGH);
    digitalWrite(Y_L6470_RESET_PIN, HIGH);
    digitalWrite(Z_L6470_RESET_PIN, HIGH);
    //digitalWrite(E_L6470_RESET_PIN, HIGH);
    
  //pinMode(dSPIN_BUSYN_X, INPUT);
  //pinMode(dSPIN_BUSYN_Y, INPUT);
  //pinMode(dSPIN_BUSYN_Z, INPUT);
  //pinMode(dSPIN_RESET, OUTPUT);
  //pinMode(dSPIN_FLAG, INPUT);
  //pinMode(dSPIN_BUSY, INPUT);
  
  //digitalWrite(LED, HIGH);
  
  // reset the dSPIN chip. This could also be accomplished by
  //  calling the "dSPIN_ResetDev()" function after SPI is initialized.
//  digitalWrite(dSPIN_RESET, HIGH);
//  delay(1);
//  digitalWrite(dSPIN_RESET, LOW);
//  delay(1);
//  digitalWrite(dSPIN_RESET, HIGH);
//  delay(1);
  
  // initialize SPI for the dSPIN chip's needs:
  //  most significant bit first,
  //  SPI clock not to exceed 5MHz,
  //  SPI_MODE3 (clock idle high, latch data on rising edge of clock)  
  SPI.begin();
  SPI.setBitOrder(MSBFIRST);
  SPI.setClockDivider(SPI_CLOCK_DIV8); // or 2, 8, 16, 32, 64
  SPI.setDataMode(SPI_MODE3);
  
  //attachInterrupt(0, dspin_busyChange, CHANGE);
  //attachInterrupt(1, dspin_flagChange, CHANGE);
  //interrupts(); 
}

void dSPIN_setup() 
{
  // Standard serial port initialization for debugging.
  //Serial.begin(9600);
    
 // dSPIN_init() is implemented in the dSPIN_support.ino file. It includes
  //  all the necessary port setup and SPI setup to allow the Arduino to
  //  control the dSPIN chip and relies entirely upon the pin redefinitions
  //  in dSPIN_example.ino
  //dSPIN_init();
  
  // First things first: let's check communications. The CONFIG register should
  //  power up to 0x2E88, so we can use that to check the communications.
  //  On the test jig, this causes an LED to light up.
  dSPIN_GetParam(MOTOR_X, dSPIN_CONFIG);
  dSPIN_GetParam(MOTOR_Y, dSPIN_CONFIG);
  dSPIN_GetParam(MOTOR_Z, dSPIN_CONFIG);
  dSPIN_GetParam(MOTOR_E, dSPIN_CONFIG);
  
  SERIAL_ECHO_START;
  SERIAL_ECHOLN("");
  SERIAL_ECHOPGM("X :");
  SERIAL_ECHOLN(dSPIN_GetParam(MOTOR_X, dSPIN_CONFIG));
  SERIAL_ECHOPGM("Y :");
  SERIAL_ECHOLN(dSPIN_GetParam(MOTOR_Y, dSPIN_CONFIG));
  SERIAL_ECHOPGM("Z :");  
  SERIAL_ECHOLN(dSPIN_GetParam(MOTOR_Z, dSPIN_CONFIG));
  SERIAL_ECHOPGM("E :");  
  SERIAL_ECHOLN(dSPIN_GetParam(MOTOR_E, dSPIN_CONFIG));
  SERIAL_ECHOLN("");

  //Serial.println(dSPIN_GetParam(MOTOR_X, dSPIN_CONFIG), HEX);
  //Serial.println(dSPIN_GetParam(MOTOR_Y, dSPIN_CONFIG), HEX);
  //Serial.println(dSPIN_GetParam(MOTOR_Z, dSPIN_CONFIG), HEX);
  
  // The following function calls are for this demo application- you will
  //  need to adjust them for your particular application, and you may need
  //  to configure additional registers.
  
  // First, let's set the step mode register:
  //   - dSPIN_SYNC_EN controls whether the BUSY/SYNC pin reflects the step
  //     frequency or the BUSY status of the chip. We want it to be the BUSY
  //     status.
  //   - dSPIN_STEP_SEL_x is the microstepping rate- we'll go full step.
  //   - dSPIN_SYNC_SEL_x is the ratio of (micro)steps to toggles on the
  //     BUSY/SYNC pin (when that pin is used for SYNC). Make it 1:1, despite
  //     not using that pin.
  dSPIN_SetParam(MOTOR_X, dSPIN_STEP_MODE, !dSPIN_SYNC_EN | dSPIN_STEP_SEL_1_8 | dSPIN_SYNC_SEL_2);
  dSPIN_SetParam(MOTOR_Y, dSPIN_STEP_MODE, !dSPIN_SYNC_EN | dSPIN_STEP_SEL_1_8 | dSPIN_SYNC_SEL_2);
  dSPIN_SetParam(MOTOR_Z, dSPIN_STEP_MODE, !dSPIN_SYNC_EN | dSPIN_STEP_SEL_1_8 | dSPIN_SYNC_SEL_2);
  dSPIN_SetParam(MOTOR_E, dSPIN_STEP_MODE, !dSPIN_SYNC_EN | dSPIN_STEP_SEL_1_8 | dSPIN_SYNC_SEL_2);
  // Configure the MAX_SPEED register- this is the maximum number of (micro)steps per
  //  second allowed. You'll want to mess around with your desired application to see
  //  how far you can push it before the motor starts to slip. The ACTUAL parameter
  //  passed to this function is in steps/tick; MaxSpdCalc() will convert a number of
  //  steps/s into an appropriate value for this function. Note that for any move or
  //  goto type function where no speed is specified, this value will be used.
  dSPIN_SetParam(MOTOR_X, dSPIN_MAX_SPEED, MaxSpdCalc(15000)); // Max speed is written for every move. So the figure here doesn't matter
  dSPIN_SetParam(MOTOR_Y, dSPIN_MAX_SPEED, MaxSpdCalc(15000));
  dSPIN_SetParam(MOTOR_Z, dSPIN_MAX_SPEED, MaxSpdCalc(15000));
  dSPIN_SetParam(MOTOR_E, dSPIN_MAX_SPEED, MaxSpdCalc(15000));

  /* 
  float axis_steps_per_unit[] = DEFAULT_AXIS_STEPS_PER_UNIT;
  dSPIN_SetParam(MOTOR_X, dSPIN_MIN_SPEED, MinSpdCalc(DEFAULT_XYJERK * axis_steps_per_unit[MOTOR_X]));
  dSPIN_SetParam(MOTOR_Y, dSPIN_MIN_SPEED, MinSpdCalc(DEFAULT_XYJERK * axis_steps_per_unit[MOTOR_Y]));
  dSPIN_SetParam(MOTOR_Z, dSPIN_MIN_SPEED, MinSpdCalc(DEFAULT_ZJERK * axis_steps_per_unit[MOTOR_Z]));
  dSPIN_SetParam(MOTOR_E, dSPIN_MIN_SPEED, MinSpdCalc(DEFAULT_EJERK * axis_steps_per_unit[MOTOR_E]));
   */
  
  
  dSPIN_SetParam(MOTOR_X, dSPIN_MIN_SPEED, MinSpdCalc(0)); // Min speed is written for every move. So the figure here doesn't matter
  dSPIN_SetParam(MOTOR_Y, dSPIN_MIN_SPEED, MinSpdCalc(0));
  dSPIN_SetParam(MOTOR_Z, dSPIN_MIN_SPEED, MinSpdCalc(0));
  dSPIN_SetParam(MOTOR_E, dSPIN_MIN_SPEED, MinSpdCalc(0));
  
  // Configure the FS_SPD register- this is the speed at which the driver ceases
  //  microstepping and goes to full stepping. FSCalc() converts a value in steps/s
  //  to a value suitable for this register; to disable full-step switching, you
  //  can pass 0x3FF to this register.
  dSPIN_SetParam(MOTOR_X, dSPIN_FS_SPD, FSCalc(5000)); // Will probably never be reached on a RepRap.
  dSPIN_SetParam(MOTOR_Y, dSPIN_FS_SPD, FSCalc(5000));
  dSPIN_SetParam(MOTOR_Z, dSPIN_FS_SPD, FSCalc(5000));
  dSPIN_SetParam(MOTOR_E, dSPIN_FS_SPD, FSCalc(5000));
//  dSPIN_SetParam(dSPIN_FS_SPD, 0x3ff);
  // Configure the acceleration rate, in steps/tick/tick. There is also a DEC register;
  //  both of them have a function (AccCalc() and DecCalc() respectively) that convert
  //  from steps/s/s into the appropriate value for the register. Writing ACC to 0xfff
  //  sets the acceleration and deceleration to 'infinite' (or as near as the driver can
  //  manage). If ACC is set to 0xfff, DEC is ignored. To get infinite deceleration
  //  without infinite acceleration, only hard stop will work.
//  dSPIN_SetParam(dSPIN_ACC, 0xfff);
  dSPIN_SetParam(MOTOR_X, dSPIN_ACC, 0xf00);
  dSPIN_SetParam(MOTOR_Y, dSPIN_ACC, 0xf00);
  dSPIN_SetParam(MOTOR_Z, dSPIN_ACC, 0xf00);
  dSPIN_SetParam(MOTOR_E, dSPIN_ACC, 0xf00);
  
  dSPIN_SetParam(MOTOR_X, dSPIN_DEC, 0xf00);  
  dSPIN_SetParam(MOTOR_Y, dSPIN_DEC, 0xf00);  
  dSPIN_SetParam(MOTOR_Z, dSPIN_DEC, 0xf00);  
  dSPIN_SetParam(MOTOR_E, dSPIN_DEC, 0xf00);  
  // Configure the overcurrent detection threshold. The constants for this are defined
  //  in the dSPIN_example.ino file.
  dSPIN_SetParam(MOTOR_X, dSPIN_OCD_TH, dSPIN_OCD_TH_3000mA);
  dSPIN_SetParam(MOTOR_Y, dSPIN_OCD_TH, dSPIN_OCD_TH_3000mA);
  dSPIN_SetParam(MOTOR_Z, dSPIN_OCD_TH, dSPIN_OCD_TH_3000mA);
  dSPIN_SetParam(MOTOR_E, dSPIN_OCD_TH, dSPIN_OCD_TH_3000mA);
  
  dSPIN_SetParam(MOTOR_X, dSPIN_STALL_TH, 128);
  dSPIN_SetParam(MOTOR_Y, dSPIN_STALL_TH, 128);
  dSPIN_SetParam(MOTOR_Z, dSPIN_STALL_TH, 128);
  dSPIN_SetParam(MOTOR_E, dSPIN_STALL_TH, 128);
  
  // Set up the CONFIG register as follows:
  //  PWM frequency divisor = 1
  //  PWM frequency multiplier = 2 (62.5kHz PWM frequency)
  //  Slew rate is 290V/us
  //  Do NOT shut down bridges on overcurrent
  //  Disable motor voltage compensation
  //  Hard stop on switch low
  //  16MHz internal oscillator, nothing on output
  dSPIN_SetParam(MOTOR_X, dSPIN_CONFIG, 
                   dSPIN_CONFIG_PWM_DIV_1 | dSPIN_CONFIG_PWM_MUL_2 | dSPIN_CONFIG_SR_290V_us
  //               | dSPIN_CONFIG_OC_SD_DISABLE | dSPIN_CONFIG_VS_COMP_DISABLE 
                 | dSPIN_CONFIG_SW_HARD_STOP | dSPIN_CONFIG_INT_16MHZ);
  dSPIN_SetParam(MOTOR_Y, dSPIN_CONFIG, 
                   dSPIN_CONFIG_PWM_DIV_1 | dSPIN_CONFIG_PWM_MUL_2 | dSPIN_CONFIG_SR_290V_us
  //               | dSPIN_CONFIG_OC_SD_DISABLE | dSPIN_CONFIG_VS_COMP_DISABLE 
                 | dSPIN_CONFIG_SW_HARD_STOP | dSPIN_CONFIG_INT_16MHZ);
  dSPIN_SetParam(MOTOR_Z, dSPIN_CONFIG, 
                     dSPIN_CONFIG_PWM_DIV_1 | dSPIN_CONFIG_PWM_MUL_2 | dSPIN_CONFIG_SR_290V_us
//                   dSPIN_CONFIG_PWM_DIV_1 | dSPIN_CONFIG_PWM_MUL_2 | dSPIN_CONFIG_SR_290V_us
//                 | dSPIN_CONFIG_OC_SD_DISABLE | dSPIN_CONFIG_VS_COMP_DISABLE
                 | dSPIN_CONFIG_SW_HARD_STOP | dSPIN_CONFIG_INT_16MHZ);
  dSPIN_SetParam(MOTOR_E, dSPIN_CONFIG, 
                   dSPIN_CONFIG_PWM_DIV_1 | dSPIN_CONFIG_PWM_MUL_2 | dSPIN_CONFIG_SR_290V_us
    //             | dSPIN_CONFIG_OC_SD_DISABLE | dSPIN_CONFIG_VS_COMP_DISABLE 
                 | dSPIN_CONFIG_SW_HARD_STOP | dSPIN_CONFIG_INT_16MHZ);
  // Configure the RUN KVAL. This defines the duty cycle of the PWM of the bridges
  //  during running. 0xFF means that they are essentially NOT PWMed during run; this
  //  MAY result in more power being dissipated than you actually need for the task.
  //  Setting this value too low may result in failure to turn.
  //  There are ACC, DEC, and HOLD KVAL registers as well; you may need to play with
  //  those values to get acceptable performance for a given application.
//  dSPIN_SetParam(dSPIN_KVAL_RUN, 0xFF);
  // Calling GetStatus() clears the UVLO bit in the status register, which is set by
  //  default on power-up. The driver may not run without that bit cleared by this
  //  read operation.
  
//  dSPIN_SetParam(dSPIN_KVAL_HOLD, 0xa);  //0xa
//  dSPIN_SetParam(dSPIN_KVAL_ACC, 0x10);  // 0xb
//  dSPIN_SetParam(dSPIN_KVAL_DEC, 0x10);  // 0xb
//  dSPIN_SetParam(dSPIN_KVAL_RUN, 0x10);  // 0xb
//  dSPIN_SetParam(dSPIN_ST_SLP, 0x5);     // 0x5
//  dSPIN_SetParam(dSPIN_INT_SPD, 0x1ad2);  // this should be 0x1c50 according to bemf cacl
//  dSPIN_SetParam(dSPIN_FN_SLP_ACC, 0xb);  //0xb
//  dSPIN_SetParam(dSPIN_FN_SLP_DEC, 0xb);  //0xb

  dSPIN_SetParam(MOTOR_X, dSPIN_KVAL_HOLD, 0x20);  //0xa
  dSPIN_SetParam(MOTOR_X, dSPIN_KVAL_ACC, 0x38);  // 0xb
  dSPIN_SetParam(MOTOR_X, dSPIN_KVAL_DEC, 0x34);  // 0xb
  dSPIN_SetParam(MOTOR_X, dSPIN_KVAL_RUN, 0x28);  // 0xb
  dSPIN_SetParam(MOTOR_X, dSPIN_ST_SLP, 0x6);     // 0x5
  dSPIN_SetParam(MOTOR_X, dSPIN_INT_SPD, 0x165a);  // this should be 0x1c50 according to bemf cacl
  dSPIN_SetParam(MOTOR_X, dSPIN_FN_SLP_ACC, 0x1d);  //0xb
  dSPIN_SetParam(MOTOR_X, dSPIN_FN_SLP_DEC, 0x1d);  //0xb

  dSPIN_SetParam(MOTOR_Y, dSPIN_KVAL_HOLD, 0x20);  //0xa
  dSPIN_SetParam(MOTOR_Y, dSPIN_KVAL_ACC, 0x38);  // 0xb
  dSPIN_SetParam(MOTOR_Y, dSPIN_KVAL_DEC, 0x34);  // 0xb
  dSPIN_SetParam(MOTOR_Y, dSPIN_KVAL_RUN, 0x28;  // 0xb
  dSPIN_SetParam(MOTOR_Y, dSPIN_ST_SLP, 0x6);     // 0x5
  dSPIN_SetParam(MOTOR_Y, dSPIN_INT_SPD, 0x165a);  // this should be 0x1c50 according to bemf cacl
  dSPIN_SetParam(MOTOR_Y, dSPIN_FN_SLP_ACC, 0x1d);  //0xb
  dSPIN_SetParam(MOTOR_Y, dSPIN_FN_SLP_DEC, 0x1d);  //0xb
// Serial connection 
  dSPIN_SetParam(MOTOR_Z, dSPIN_KVAL_HOLD, 0x18);  //0x20
  dSPIN_SetParam(MOTOR_Z, dSPIN_KVAL_ACC, 0x5b);  // 0x5b
  dSPIN_SetParam(MOTOR_Z, dSPIN_KVAL_DEC, 0x5b);  // 0x5b
  dSPIN_SetParam(MOTOR_Z, dSPIN_KVAL_RUN, 0x5b);  // 0x5b
  dSPIN_SetParam(MOTOR_Z, dSPIN_ST_SLP, 0x9);     // 0x9
  dSPIN_SetParam(MOTOR_Z, dSPIN_INT_SPD, 0x165a);  // this should be 0x1651 according to bemf cacl
  dSPIN_SetParam(MOTOR_Z, dSPIN_FN_SLP_ACC, 0x4d);  //0x4d
  dSPIN_SetParam(MOTOR_Z, dSPIN_FN_SLP_DEC, 0x4d);  //0x4d
  
  
  dSPIN_SetParam(MOTOR_E, dSPIN_KVAL_HOLD, 0x20);  // 0xa
  dSPIN_SetParam(MOTOR_E, dSPIN_KVAL_ACC, 0x3f);  // 0xb
  dSPIN_SetParam(MOTOR_E, dSPIN_KVAL_DEC, 0x28);  // 0xb
  dSPIN_SetParam(MOTOR_E, dSPIN_KVAL_RUN, 0x38);  // 0xb
  dSPIN_SetParam(MOTOR_E, dSPIN_ST_SLP, 0x6);     // 0x5
  dSPIN_SetParam(MOTOR_E, dSPIN_INT_SPD, 0x1651);  // this should be 0x1c50 according to bemf cacl
  dSPIN_SetParam(MOTOR_E, dSPIN_FN_SLP_ACC, 0x1d);  //0xb
  dSPIN_SetParam(MOTOR_E, dSPIN_FN_SLP_DEC, 0x1d);  //0xb
  
  
/* Optimised for Parallel connection
  dSPIN_SetParam(MOTOR_Z, dSPIN_KVAL_HOLD, 0x18);  //0xa
  dSPIN_SetParam(MOTOR_Z, dSPIN_KVAL_ACC, 0x20);  // 0xb
  dSPIN_SetParam(MOTOR_Z, dSPIN_KVAL_DEC, 0x18);  // 0xb
  dSPIN_SetParam(MOTOR_Z, dSPIN_KVAL_RUN, 0x18);  // 0xb
  dSPIN_SetParam(MOTOR_Z, dSPIN_ST_SLP, 0x9);     // 0x5
  dSPIN_SetParam(MOTOR_Z, dSPIN_INT_SPD, 0x165a);  // this should be 0x1c50 according to bemf cacl
  dSPIN_SetParam(MOTOR_Z, dSPIN_FN_SLP_ACC, 0x3c);  //0xb
  dSPIN_SetParam(MOTOR_Z, dSPIN_FN_SLP_DEC, 0x3c);  //0xb
*/
/* Optimised for Serial connection */
/*
  dSPIN_SetParam(MOTOR_Z, dSPIN_KVAL_HOLD, 0x30);  //0x30
  dSPIN_SetParam(MOTOR_Z, dSPIN_KVAL_ACC, 0x40);  // 0x40
  dSPIN_SetParam(MOTOR_Z, dSPIN_KVAL_DEC, 0x30);  // 0x30
  dSPIN_SetParam(MOTOR_Z, dSPIN_KVAL_RUN, 0x30);  // 0x30
  dSPIN_SetParam(MOTOR_Z, dSPIN_ST_SLP, 0x9);     // 0x9
  dSPIN_SetParam(MOTOR_Z, dSPIN_INT_SPD, 0x165a);  // this should be 0x1651 according to bemf cacl
  dSPIN_SetParam(MOTOR_Z, dSPIN_FN_SLP_ACC, 0x3c);  //0x3c
  dSPIN_SetParam(MOTOR_Z, dSPIN_FN_SLP_DEC, 0x3c);  //0x3c
*/  
  
/* Working but runs hot Mecury Motor 
  dSPIN_SetParam(MOTOR_E, dSPIN_KVAL_HOLD, 0x30);  // 0x23
  dSPIN_SetParam(MOTOR_E, dSPIN_KVAL_ACC, 0xd0);  // 0xc5
  dSPIN_SetParam(MOTOR_E, dSPIN_KVAL_DEC, 0xc5);  // 0xc5
  dSPIN_SetParam(MOTOR_E, dSPIN_KVAL_RUN, 0xd0);  // 0xc5
  dSPIN_SetParam(MOTOR_E, dSPIN_ST_SLP, 0x5);     // 0x5
  dSPIN_SetParam(MOTOR_E, dSPIN_INT_SPD, 0x1c56);  // this should be 0x1c56 according to bemf cacl
  dSPIN_SetParam(MOTOR_E, dSPIN_FN_SLP_ACC, 0x79);  //0x79
  dSPIN_SetParam(MOTOR_E, dSPIN_FN_SLP_DEC, 0x79);  //0x79
 *. 
/* Settings for Mercury Motor */
/*  dSPIN_SetParam(MOTOR_E, dSPIN_KVAL_HOLD, 0x80);  // 0x23
  dSPIN_SetParam(MOTOR_E, dSPIN_KVAL_ACC, 0xc0);  // 0xc5
  dSPIN_SetParam(MOTOR_E, dSPIN_KVAL_DEC, 0xbc);  // 0xc5
  dSPIN_SetParam(MOTOR_E, dSPIN_KVAL_RUN, 0xd0);  // 0xc5
  dSPIN_SetParam(MOTOR_E, dSPIN_ST_SLP, 0x5);     // 0x5
  dSPIN_SetParam(MOTOR_E, dSPIN_INT_SPD, 0x1c56);  // this should be 0x1c56 according to bemf cacl
  dSPIN_SetParam(MOTOR_E, dSPIN_FN_SLP_ACC, 0x79);  //0x79
  dSPIN_SetParam(MOTOR_E, dSPIN_FN_SLP_DEC, 0x79);  //0x79
*/

  // Thermal compensation for motor heating
  //dSPIN_SetParam(MOTOR_X, dSPIN_K_THERM, 0x2);  
  //dSPIN_SetParam(MOTOR_Y, dSPIN_K_THERM, 0x2);  
  //dSPIN_SetParam(MOTOR_Z, dSPIN_K_THERM, 0x2);  
  //dSPIN_SetParam(MOTOR_E, dSPIN_K_THERM, 0x2);  
  
  /*
  dSPIN_Step_Clock(MOTOR_X, 0);
  dSPIN_Step_Clock(MOTOR_Y, 0);
  dSPIN_Step_Clock(MOTOR_Z, 0);
  */
  
  dSPIN_GetStatus(MOTOR_X);
  dSPIN_GetStatus(MOTOR_Y);
  dSPIN_GetStatus(MOTOR_Z);
  dSPIN_GetStatus(MOTOR_E);
  
  //SetLSPDOpt(MOTOR_Z, true, MinSpdCalc(20));
  //Serial.println(dSPIN_GetParam(MOTOR_X, dSPIN_CONFIG), HEX);
  //Serial.println(dSPIN_GetParam(MOTOR_Y, dSPIN_CONFIG), HEX);
  //Serial.println(dSPIN_GetParam(MOTOR_Z, dSPIN_CONFIG), HEX);
}


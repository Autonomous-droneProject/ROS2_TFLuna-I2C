/* File Name: TFLI2C.cpp
 * Developer: Bud Ryerson (edited by Aldem Pido)
 * Date:      10 JUL 2021
 * Version:   0.1.1 - Fixed some typos in comments.
              Added a `p_` prefix to some pointer variables.
              Changed some register addresses from hard to symbolic.
              Changed TFL_DEFAULT_ADDR and TFL_DEFAULT_FPS
              to TFL_DEF_ADR and TFL_DEF_FPS in the header file.
              0.2.0 - Corrected (reversed) Enable/Disable commands
 * Described: Arduino Library for the Benewake TF-Luna Lidar sensor
              configured for the I2C interface
 *
 * Default settings for the TF-Luna are:
 *    0x10  -  default slave device I2C address `TFL_DEF_ADR`
 *    100Hz  - default data frame-rate `TFL_DEF_FPS`
 *
 *  There are is one important function: 'getData'.
 *  `getData( dist, flux, temp, addr)`
      Reads the disance measured, return signal strength and chip temperature.
      - dist : unsigned integer : distance measured by the device, in cm.
      - flux : unsigned integer : signal strength, quality or confidence
               If flux value too low, an error will occur.
      - temp : unsigned integer : temperature of the chip in 0.01 degrees C
      - addr : unsigned byte : address of slave device.
      Returns true, if no error occurred.
         If false, error is defined by a status code
         that can be displayed using 'printFrame()' function.

 * NOTE : If you only want to read distance, use getData( dist, addr)
 *
 *  There are several explicit commands
 */

#include "TFLI2C.hpp"

namespace tfluna {
  // Constructor/Destructor
TFLI2C::TFLI2C(std::string i2c_bus, uint8_t address) : i2c_bus_(i2c_bus.c_str()), address_(address){}
TFLI2C::~TFLI2C(){}

bool TFLI2C::init() {
  // Open i2c port
  if((fd_ = open(i2c_bus_, O_RDWR)) < 0) {
    return false;
  }
  // Flow control
  if(ioctl(fd_, I2C_SLAVE, address_) < 0) {
    return false;
  }
}

// - - - - - - - - - - - - - - - - - - - - - - - - - -
//             GET DATA FROM THE DEVICE
// - - - - - - - - - - - - - - - - - - - - - - - - - -
bool TFLI2C::getData( int16_t &dist, int16_t &flux, int16_t &temp)
{
    tfStatus = TFL_READY;    // clear status of any error condition

    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    // Step 1 - Use the `Wire` function `readReg` to fill the six byte
    // `dataArray` from the contiguous sequence of registers `TFL_DIST_LO`
    // to `TFL_TEMP_HI` that declared in the header file 'TFLI2C.h`.
    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    for (uint8_t reg = TFL_DIST_LO; reg <= TFL_TEMP_HI; reg++)
    {
      if( !readReg( reg, regReply)) return false;
          else dataArray[ reg] = regReply;
    }

    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    // Step 2 - Shift data from read array into the three variables
    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    dist = dataArray[ 0] + ( dataArray[ 1] << 8);
    flux = dataArray[ 2] + ( dataArray[ 3] << 8);
    temp = dataArray[ 4] + ( dataArray[ 5] << 8);

/*
    // Convert temperature from hundredths
    // of a degree to a whole number
    temp = int16_t( temp / 100);
    // Then convert Celsius to degrees Fahrenheit
    temp = uint8_t( temp * 9 / 5) + 32;
*/

    // - - Evaluate Abnormal Data Values - -
    // Signal strength <= 100
    if( flux < (int16_t)100)
    {
      tfStatus = TFL_WEAK;
      return false;
    }
    // Signal Strength saturation
    else if( flux == (int16_t)0xFFFF)
    {
      tfStatus = TFL_STRONG;
      return false;
    }
    else
    {
      tfStatus = TFL_READY;
      return true;
    }
}

// Get Data short version
bool TFLI2C::getData( int16_t &dist)
{
  static int16_t flux, temp;
  return getData( dist, flux, temp);
}

// - - - - - - - - - - - - - - - - - - - - - - - - - -
//              EXPLICIT COMMANDS
// - - - - - - - - - - - - - - - - - - - - - - - - - -

//  = =  GET DEVICE TIME (in milliseconds) = = =
//  Pass back time as an unsigned 16-bit variable
bool TFLI2C::Get_Time( uint16_t &tim)
{
    // Recast the address of the unsigned integer `tim`
    // as a pointer to an unsigned byte `p_tim`...
    uint8_t * p_tim = (uint8_t *) &tim;

    // ... then address the pointer as an array.
    if( !readReg( TFL_TICK_LO, regReply)) return false;
        else p_tim[ 0] = regReply;  // Read into `tim` array
    if( !readReg( TFL_TICK_HI, regReply)) return false;
        else p_tim[ 1] = regReply;  // Read into `tim` array
    return true;
}

//  = =  GET PRODUCTION CODE (Serial Number) = = =
// When you pass an array as a parameter to a function
// it decays into a pointer to the first element of the array.
// The 14 byte array variable `tfCode` declared in the example
// sketch decays to the array pointer `p_cod`.
bool TFLI2C::Get_Prod_Code( uint8_t * p_cod)
{
   for (uint8_t i = 0; i < 14; ++i)
    {
      if( !readReg( ( 0x10 + i), regReply)) return false;
        else p_cod[ i] = regReply;  // Read into product code array
    }
    return true;
}

//  = = = =    GET FIRMWARE VERSION   = = = =
// The 3 byte array variable `tfVer` declared in the
// example sketch decays to the array pointer `p_ver`.
bool TFLI2C::Get_Firmware_Version( uint8_t * p_ver)
{
    for (uint8_t i = 0; i < 3; ++i)
    {
      if( !readReg( ( 0x0A + i), regReply)) return false;
        else p_ver[ i] = regReply;  // Read into version array
    }
    return true;
}

//  = = = = =    SAVE SETTINGS   = = = = =
bool TFLI2C::Save_Settings()
{
    return( writeReg( TFL_SAVE_SETTINGS, 1));
}

//  = = = =   SOFT (SYSTEM) RESET   = = = =
bool TFLI2C::Soft_Reset()
{
    return( writeReg( TFL_SOFT_RESET, 2));
}

//  = = = = = =    SET I2C ADDRESS   = = = = = =
// Range: 0x08, 0x77. Must reboot to take effect.
bool TFLI2C::Set_I2C_Addr( uint8_t adrNew)
{
    return( writeReg( TFL_SET_I2C_ADDR, adrNew));
}

//  = = = = =   SET ENABLE   = = = = =
bool TFLI2C::Set_Enable()
{
    return( writeReg( TFL_DISABLE, 1));
}

//  = = = = =   SET DISABLE   = = = = =
bool TFLI2C::Set_Disable()
{
    return( writeReg( TFL_DISABLE, 0));
}

//  = = = = = =    SET FRAME RATE   = = = = = =
bool TFLI2C::Set_Frame_Rate( uint16_t &frm)
{
    // Recast the address of the unsigned integer `frm`
    // as a pointer to an unsigned byte `p_frm` ...
    uint8_t * p_frm = (uint8_t *) &frm;

    // ... then address the pointer as an array.
    if( !writeReg( ( TFL_FPS_LO), p_frm[ 0])) return false;
    if( !writeReg( ( TFL_FPS_HI), p_frm[ 1])) return false;
    return true;
}

//  = = = = = =    GET FRAME RATE   = = = = = =
bool TFLI2C::Get_Frame_Rate( uint16_t &frm)
{
    uint8_t * p_frm = (uint8_t *) &frm;
    if( !readReg( TFL_FPS_LO)) return false;
        else p_frm[ 0] = regReply;  // Read into `frm` array
    if( !readReg( TFL_FPS_HI)) return false;
        else p_frm[ 1] = regReply;  // Read into `frm` array
    return true;
}

//  = = = =   HARD RESET to Factory Defaults  = = = =
bool TFLI2C::Hard_Reset()
{
    return( writeReg( TFL_HARD_RESET, 1));
}

//  = = = = = =   SET CONTINUOUS MODE   = = = = = =
// Sample LiDAR chip continuously at Frame Rate
bool TFLI2C::Set_Cont_Mode()
{
    return( writeReg( TFL_SET_TRIG_MODE, 0));
}

//  = = = = = =   SET TRIGGER MODE   = = = = = =
// Device will sample only once when triggered
bool TFLI2C::Set_Trig_Mode()
{
    return( writeReg( TFL_SET_TRIG_MODE, 1));
}

//  = = = = = =   SET TRIGGER   = = = = = =
// Trigger device to sample once
bool TFLI2C::Set_Trigger()
{
    return( writeReg( TFL_TRIGGER, 1));
}
//
// = = = = = = = = = = = = = = = = = = = = = = = =

// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
//       READ OR WRITE A GIVEN REGISTER OF THE SLAVE DEVICE
// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
bool TFLI2C::readReg(uint8_t reg_addr, uint8_t& data)
{
  if(fd_ < 0) return false;

  i2c_msg messages[2];
  i2c_rdwr_ioctl_data ioctl_data;

  // Message 1: Write the register address
  messages[0].addr = this->address_; // Use the stored device address
  messages[0].flags = 0; // 0 for write
  messages[0].len = 1;
  messages[0].buf = &reg_addr;

  // Message 2: Read the data from the register
  messages[1].addr = this->address_; // Use the stored device address
  messages[1].flags = I2C_M_RD; // Flag for read
  messages[1].len = 1;
  messages[1].buf = &data;
  // Prepare the ioctl call
  ioctl_data.msgs = messages;
  ioctl_data.nmsgs = 2; // We have two messages

  if (ioctl(fd_, I2C_RDWR, &ioctl_data) < 0) {
      if (errno == EREMOTEIO) {
          tfStatus = TFL_I2CREAD;
      } else {
            tfStatus = TFL_I2CWRITE;
      }
      perror("ioctl(I2C_RDWR) failed");
      return false;
  }

  tfStatus = TFL_READY;
  return true;
}

bool TFLI2C::writeReg(uint8_t reg_addr, uint8_t data)
{
  if(fd_ < 0) return false;
  uint8_t buffer[2];
  buffer[0] = reg_addr;
  buffer[1] = data;
  if(write(fd_, buffer, 2) != 2) {
    tfStatus = TFL_I2CWRITE;        // then set status code...
    return false;   
  } else return true;
}

// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
// - - - - -    The following is for testing purposes    - - - -
// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

// Called by either `printFrame()` or `printReply()`
// Print status condition either `READY` or error type
std::string TFLI2C::printStatus()
{
  std::string output;
  output.clear();
  output.append("Status: ");
    if( tfStatus == TFL_READY)          output.append( "READY");
    else if( tfStatus == TFL_SERIAL)    output.append( "SERIAL");
    else if( tfStatus == TFL_HEADER)    output.append( "HEADER");
    else if( tfStatus == TFL_CHECKSUM)  output.append( "CHECKSUM");
    else if( tfStatus == TFL_TIMEOUT)   output.append( "TIMEOUT");
    else if( tfStatus == TFL_PASS)      output.append( "PASS");
    else if( tfStatus == TFL_FAIL)      output.append( "FAIL");
    else if( tfStatus == TFL_I2CREAD)   output.append( "I2C-READ");
    else if( tfStatus == TFL_I2CWRITE)  output.append( "I2C-WRITE");
    else if( tfStatus == TFL_I2CLENGTH) output.append( "I2C-LENGTH");
    else if( tfStatus == TFL_WEAK)      output.append( "Signal weak");
    else if( tfStatus == TFL_STRONG)    output.append( "Signal strong");
    else if( tfStatus == TFL_FLOOD)     output.append( "Ambient light");
    else if( tfStatus == TFL_INVALID)   output.append( "No Command");
    else output.append( "OTHER");
  return output;
}


// Print error type and HEX values
// of each byte in the data frame
std::string TFLI2C::printDataArray()
{
  std::string output;
  output.clear();
    // Print the Hex value of each byte of data
    output.append(" Data:");
    for( uint8_t i = 0; i < 6; i++)
    {
      output.append(" ");
      output.append( dataArray[ i] < 16 ? "0" : "");
      output.append( dataArray[ i] + "");
    }
    output.append("\n");
}

// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

}

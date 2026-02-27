/*
  MLX90615.cpp - Library for reading MLX90615 sensor.
*/

#include "MLX90615.h"

//================================================================================
//=====================  MLX90615 Device class functions.  =======================
//================================================================================

/*
 * \brief               MLX90615 Device class constructor.
 * \param [in] i2caddr  Device address (default: 0x5B)
 */
MLX90615::MLX90615(uint8_t i2caddr)
{
	_addr = i2caddr;
	_ready = false;
}

/*
 * \brief   Initialize the device and the I2C interface.
 */
boolean MLX90615::begin(void)
{
	_rwError = _pec = _crc8 = 0;
	return _ready = true;
}

/*
 * \brief   Turn the sensor to sleep mode.
 * \remarks
 * \li      Send command 0xC6 with PEC to enter sleep mode.
 * \li      SCL is held high during sleep mode (per datasheet).
 */
boolean MLX90615::sleep(void)
{
    CRC8 crc(MLX90615_CRC8POLY);

    // Build the CRC-8 of all bytes to be sent.
    crc.crc8(_addr << 1);
    _crc8 = crc.crc8(MLX90615_SLEEP_MODE);

    // Send the slave address then the sleep command with PEC.
    Wire.beginTransmission(_addr);
    Wire.write(MLX90615_SLEEP_MODE);
    Wire.write(_crc8);
    _rwError |= (1 << Wire.endTransmission(true)) >> 1;

    // Keep SCL high during sleep mode (per datasheet requirement).
    pinMode(A5, OUTPUT);
    pinMode(A4, OUTPUT);
    digitalWrite(A5, HIGH);
    digitalWrite(A4, LOW);

    // Clear r/w errors if using broadcast address.
    if(_addr == MLX90615_BROADCASTADDR) _rwError &= MLX90615_NORWERROR;

    return true;
}

/*
 * \brief   Wake the sensor from sleep mode.
 * \remarks
 * \li      Drive SCL low for at least 8ms, then release both I2C pins
 *          before reinitializing Wire. SDA was held low during sleep and
 *          must be released or Wire.begin() cannot reconfigure the bus.
 * \li      Data is ready ~0.3s after waking.
 */
boolean MLX90615::wakeUp(void)
{
    // Drive SCL low for at least 8ms to trigger wake-up.
    pinMode(A5, OUTPUT);
    digitalWrite(A5, LOW);
    delay(10);

    // Release both I2C pins before handing control back to Wire.
    // If SDA remains driven low, Wire.begin() cannot reinitialize the bus.
    pinMode(A4, INPUT);
    pinMode(A5, INPUT);

    // Reinitialize I2C and wait for sensor to be ready (typ. 0.3s).
    Wire.begin();
    delay(400);

    return MLX90615::begin();
}

/**
 *  \brief             Return a temperature from the specified source in specified units.
 *  \remarks
 *  \li                Temperature is stored in RAM as a 16 bit absolute value to a resolution of 0.02K
 *  \li                Linearized sensor die temperature is available as Ta (ambient).
 *  \li                One object temperature is linearized to the range -38.2C...125C
 *  \param [in] tsrc   Internal temperature source to read, default: OBJECT TEMPERATURE.
 *  \param [in] tunit  Temperature units to convert raw data to, default: degrees Celsius.
 *  \return            Temperature.
 */
double MLX90615::readTemp(tempSrc_t tsrc, tempUnit_t tunit)
{
	double temp;

	_rwError = 0;
	switch(tsrc)
	{
		case MLX90615_SRCA : temp = readRAM(MLX90615_T_AMBIENT); break;
		default: temp = readRAM(MLX90615_T_OBJECT);
	}
	temp *= MLX90615_TEMP_RES;
	switch(tunit)
	{
		case MLX90615_TC : return kelvin2celcius(temp);
		case MLX90615_TF : return celcius2farenheit(kelvin2celcius(temp));
	}
	return temp;
}

/**
 *  \brief            Convert temperature in degrees K to degrees C.
 *  \param [in] degK  Temperature in degrees Kelvin.
 *  \return           Temperature in degrees Centigrade.
 */
double MLX90615::kelvin2celcius(double degK) {return degK - 273.15;}

/**
 *  \brief            Convert temperature in degrees C to degrees F.
 *  \param [in] degC  Temperature in degrees Centigrade.
 *  \return           Temperature in degrees Fahrenheit.
 */
double MLX90615::celcius2farenheit(double degC) {return (degC * 1.8) + 32.0;}

/*
 * \brief               Set the emissivity of the object.
 * \remarks             The emissivity is stored as a 16 bit integer defined by the following:
 * \n<tt>              emissivity = dec2hex[round(16384 * emiss)]<tt>
 * \param [in] emiss    Physical emissivity value in range 0.1 ... 1.0, default 1.0
 */
void MLX90615::setEmissivity(float emiss)
{
	_rwError = 0;
	uint16_t e = int(emiss * 16384.0 + 0.5);
	if((emiss > 1.0) || (e < 1638)) _rwError |= MLX90615_INVALIDATA;
	else writeEEprom(MLX90615_EMISSIVITY, e);
}

/*
 * \brief               Get the emissivity of the object.
 * \remarks             The emissivity is stored as a 16 bit integer defined by the following:
 * \n<tt>              emissivity = dec2hex[round(16384 * emiss)]<tt>
 * \return              Physical emissivity value in range 0.1 ... 1.0
 */
float MLX90615::getEmissivity(void)
{
	_rwError = 0;
	uint16_t emissivity = readEEprom(MLX90615_EMISSIVITY);
	if(_rwError) return (float)1.0;
	return (float)emissivity / 16384.0;
}

/*
 *  \brief            Set the coefficients of the IIR digital filter.
 *  \remarks          The IIR digital filter coefficients are set by the 14,13 and 12 bits of ConfigRegister
 *  \n                The value of the coefficients is set as follows:
 *  \n <tt> \verbatim
 csb = 0   seq = 0 0 0   FORBIDDEN SEQUENCE !!!
       1         0 0 1    a1 = 1      a2 = 0 (IIR bypassed)
       2         0 1 0         0.5         0.5
       3         0 1 1         0.333(3)    0.666(6)
       4         1 0 0         0.25        0.75
       5         1 0 1         0.2         0.8
       6         1 1 0         0.166(6)    0.833(3)
       7         1 1 1         0.14286     0.87514 \endverbatim </tt>
 *  \param [in] csb   See page 10 of datasheet. Range 1...7, default = 1 (IIR bypassed)
 */
void MLX90615::setIIR(uint8_t csb)
{
	_rwError = 0;

	// Ensure legal range by clearing all but the LS 3 bits.
    csb &= 7; // 7 = 0111b

    // Prevents forbidden sequence; default to 1 (IIR bypassed).
    if (csb == 0) csb = 1;

	// Get the current value of ConfigRegister1
	uint16_t reg = readEEprom(MLX90615_CONFIG);

	// Clear bits 14:12, mask in the new value, then write it back.
	if(!_rwError) {
		reg &= 0x8fff;
		reg |= ((uint16_t)csb << 12);
		writeEEprom(MLX90615_CONFIG, reg);
	}
}

/*
 *  \brief    Get the coefficients of the IIR digital filter.
 *  \remarks  The IIR digital filter coefficients are set by the 14, 13 and 12 bits of ConfigRegister.
 *  \return   Filter coefficient table index. Range 1...7
 */
uint8_t MLX90615::getIIR(void)
{
	_rwError = 0;

	// Get the current value of ConfigRegister1
	uint8_t iir = (readEEprom(MLX90615_CONFIG) >> 12) & 7;

	if(_rwError) return 1;
	return iir;
}

/*
 * \brief               Return a 16 bit value read from EEPROM.
 * \param [in] addr     Register address to read from.
 * \return              Value read from EEPROM.
 */
uint16_t MLX90615::readEEprom(uint8_t addr)
{
	return read16(addr | MLX90615_EEPROMADDR);
}

/*
 * \brief           Write a 16 bit value to EEPROM after first clearing the memory.
 * \remarks
 * \li              Erase and write time 5ms per manufacturer specification.
 * \li              Manufacturer does not specify max or min erase/write times.
 * \param [in] reg  Address to write to.
 * \param [in] data Value to write.
 */
void MLX90615::writeEEprom(uint8_t reg, uint16_t data)
{
	uint16_t val;
	reg |= MLX90615_EEPROMADDR; // bitwise OR with register address to access EEPROM

	// Read current value; do nothing on a match or if there are read errors.
	val = read16(reg);
	if ((val != data) && !_rwError)
	{
		// On any R/W errors it is assumed the memory is corrupted.
		// Clear the memory and wait Terase (per manufacturer's documentation).
		write16(reg, 0);
		delay(10);
		if(_rwError) _rwError |= MLX90615_EECORRUPT;

		// Write the data and wait Twrite (per manufacturer's documentation)
		// and set the R/W error status bits.
		write16(reg, data);
		delay(10);
		if(_rwError) _rwError |= MLX90615_EECORRUPT;
	}
}

/*
 * \brief               Return a 16 bit value read from RAM.
 * \param [in] addr     Register address to read from.
 * \return              Value read from RAM.
 */
uint16_t MLX90615::readRAM(uint8_t addr)
{
	return read16(addr | MLX90615_RAMADDR);
}

/*
 *  \brief            Return a 16 bit value read from RAM or EEPROM.
 *  \param [in] cmd   Command to send (register to read from).
 *  \return           Value read from memory.
 */
uint16_t MLX90615::read16(uint8_t cmd)
{
	uint16_t val;
	CRC8 crc(MLX90615_CRC8POLY);

	// Send the slave address then the command and set any error status bits returned by write.
	Wire.beginTransmission(_addr);
	Wire.write(cmd);
	_rwError |= (1 << Wire.endTransmission(false)) >> 1;

    // Experimentally determined delay to prevent read errors.
	delayMicroseconds(MLX90615_XDLY);

	// Resend slave address then get the 3 returned bytes.
	Wire.requestFrom(_addr, (uint8_t)3);

	val = Wire.read();
	val |= Wire.read() << 8;

	// Read the PEC (CRC-8 of all bytes)
	_pec = Wire.read();

	// Clear r/w errors if using broadcast address.
    if(_addr == MLX90615_BROADCASTADDR) _rwError &= MLX90615_NORWERROR;

    // Build our own CRC-8 of all received bytes.
    crc.crc8(_addr << 1);
    crc.crc8(cmd);
    crc.crc8((_addr << 1) + 1);
    crc.crc8(lowByte(val));
    _crc8 = crc.crc8(highByte(val));

    // Set error status bit if CRC mismatch.
    if(_crc8 != _pec) _rwError |= MLX90615_RXCRC;

    return val;
}

/*
 *  \brief            Write a 16 bit value to memory.
 *  \param [in] cmd   Command to send (register to write to).
 *  \param [in] data  Value to write.
 */
void MLX90615::write16(uint8_t cmd, uint16_t data) {
    CRC8 crc(MLX90615_CRC8POLY);

    // Build the CRC-8 of all bytes to be sent.
    crc.crc8(_addr << 1);
    crc.crc8(cmd);
    crc.crc8(lowByte(data));
    _crc8 = crc.crc8(highByte(data));

    // Send the slave address then the command.
    Wire.beginTransmission(_addr);
    Wire.write(cmd);

    // Write the data low byte first.
    Wire.write(lowByte(data));
    Wire.write(highByte(data));

    // Then write the CRC and set the r/w error status bits.
    Wire.write(_pec = _crc8);
    _rwError |= (1 << Wire.endTransmission(true)) >> 1;

    // Clear r/w errors if using broadcast address.
    if(_addr == MLX90615_BROADCASTADDR) _rwError &= MLX90615_NORWERROR;
}

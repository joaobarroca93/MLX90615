/*
  MLX90615.h - Library for reading MLX90615 sensor.
*/

#ifndef MLX90615_h
#define MLX90615_h

#if (ARDUINO >= 100)
    #include "Arduino.h"
#else
    #include "WProgram.h"
#endif
#include <Wire.h>
#include "Crc8.h"

#define MLX90615_I2CDEFAULTADDR 0x5B    /**< Device default slave address */
#define MLX90615_BROADCASTADDR  0       /**< Device broadcast slave address */
#define MLX90615_CRC8POLY       7       /**< CRC polynomial = X8+X2+X1+1 */
#define MLX90615_XDLY           25      /**< Experimentally determined delay to prevent read
                                             errors after calling Wire.endTransmission()
                                             <em>(possibly due to incompatibility between Wire
                                             library and SMBus protocol)</em>. */
#define MLX90615_TEMP_RES       0.02    /**< Sensor temperature resolution: 0.02K per LSB */


// EEPROM ADDRESSES
#define MLX90615_EEPROMADDR     0x10
#define MLX90615_ADDR           0x00
#define MLX90615_PWM_T_MIN      0x00
#define MLX90615_PWM_T_RANGE    0x01
#define MLX90615_CONFIG         0x02
#define MLX90615_EMISSIVITY     0x03
#define MLX90615_ID_1           0x0E
#define MLX90615_ID_2           0x0F

// RAM ADDRESSES
#define MLX90615_RAMADDR        0x20
#define MLX90615_RAW_IR_DATA    0x05
#define MLX90615_T_AMBIENT      0x06
#define MLX90615_T_OBJECT       0x07

// COMMANDS
#define MLX90615_SLEEP_MODE     0xC6

// R/W Error flags - bitmask
#define MLX90615_NORWERROR  0       // R/W error bitmask - No Errors
#define MLX90615_RXCRC      0x10    // R/W error bitmask - Receiver CRC mismatch
#define MLX90615_INVALIDATA 0x20    // R/W error bitmask - RX/TX Data fails selection criteria
#define MLX90615_EECORRUPT  0x40    // R/W error bitmask - The EEPROM is likely to be corrupted

class MLX90615
{
	public:
		MLX90615(uint8_t i2caddr = MLX90615_I2CDEFAULTADDR);

		boolean begin(void);

		float getEmissivity(void);
		uint8_t getIIR(void);

		void setEmissivity(float emissivity = 1.0);
		void setIIR(uint8_t csb = 1);

		void writeEEprom(uint8_t, uint16_t);

		uint16_t readEEprom(uint8_t);
		uint16_t readRAM(uint8_t);

		/** Enumerations for temperature units. */
    	enum tempUnit_t {MLX90615_TK,                           /**< degrees Kelvin */
                     	MLX90615_TC,                           /**< degrees Centigrade */
                     	MLX90615_TF                            /**< degrees Fahrenheit */
    					};
	    /** Enumerations for temperature measurement source. */
	    enum tempSrc_t  {MLX90615_SRCA,                         /**< Chip (ambient) sensor */
	                     MLX90615_SRCO,                         /**< IR source */
	                    };

	    double readTemp(tempSrc_t = MLX90615_SRCO, tempUnit_t = MLX90615_TC);
	    double kelvin2celcius(double);
    	double celcius2farenheit(double);

	private:
		boolean _ready;
		uint8_t _addr;      // Slave Address
		uint8_t _rwError;   // R/W error flags
		uint8_t _crc8;      // 8 bit CRC
		uint8_t _pec;       // PEC

		uint16_t read16(uint8_t);
		void write16(uint8_t, uint16_t);
};

#endif

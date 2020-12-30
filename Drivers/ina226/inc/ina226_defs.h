/******************************************************************************
 *
 * @file	ina226_defs.h
 * @brief 	Texas Instruments INA226 register and constants definitions
 * @version	v1.00f00
 * @date	16, Dec. 2020
 * @author	CECARELLI, Federico (fededc88@gmail.com),
 * 			MOYA, Martin		(moyamartin1@gmail.com),
 * 			SANTOS, Lucio		(lusho2206@gmail.com)
 * @copyright GPL license, all text here must be included in any redistribution
 *****************************************************************************/

#ifndef __INA226_DEFS_H_
#define __INA226_DEFS_H_

#include "definitions.h"

/** 
 * INA226 Configuration Register address 
 *	Default value: 0b01000001 00100111 0x4127
 *	Type: R/W
 */
#define INA226_CONFIG_REG		(0x00)

/** 
 * INA226 Configuration Register default value
 */
#define INA226_CONFIG_DEFAULT	(0x4127)

/** 
 * INA226 Shunt Voltage Register address 
 *	Default value: 0b00000000 00000000 0x0000
 *	Type: R
 */
#define INA226_VSHUNT_REG		(0x01)

/** 
 * INA226 Bus Voltage Register address 
 * 	Default value: 0b00000000 00000000 0x0000
 *	Type: R
 */
#define INA226_VBUS_REG			(0x02)
/** 
 * INA226 Power Register address 
 * 	Default value: 0b00000000 00000000 0x0000
 *	Type: R
 */
#define INA226_PWR_REG			(0x03)
/** 
 * INA226 Current Register address 
 * 	Default value: 0b00000000 00000000 0x0000
 *	Type: R
 */
#define INA226_CURRENT_REG		(0x04)
/** 
 * INA226 Calibration Register address 
 * 	Default value: 0b00000000 00000000 0x0000
 *	Type: R
 */
#define INA226_CAL_REG			(0x05)
/** 
 * INA226 Mask/Enable Register address 
 * 	Default value: 0b00000000 00000000 0x0000
 *	Type: R
 */
#define INA226_MASK_EN_REG		(0x06)
/** 
 * INA226 Alert Limit Register address 
 * 	Default value: 0b00000000 00000000 0x0000
 *	Type: R
 */
#define INA226_ALERT_LIM_REG	(0x07)
/** 
 * INA226 Manufacturer ID Register address 
 * 	Default value: 0b0101010001001001 0x5449
 *	Type: R
 */
#define INA226_MAN_ID_REG		(0xFE)
/** 
 * INA226 Manufacturer ID Register Value
 */
#define INA226_MAN_ID_VAL		(0x5449)
/** 
 * INA226 Die ID Register address 
 * 	Default value: 0b0101010001001001 0x5449
 *	Type: R
 */
#define INA226_DIE_ID_REG		(0xFF)
/**
 * INA226 Die ID value
 */
#define INA226_DIE_ID_VAL		(0x2260)
/**
 * INA226 Bus Voltage LSB value
 */
#define INA226_VBUS_LSB_VAL		0.00125f
/**
 * INA226 Shunt Voltage LSB value
 */
#define INA226_VSHUNT_LSB_VAL	0.0000025f

/**
 * @enum _INA226_i2c_addresses
 * @brief Enum class that represent the available I2C addresses for the ina226, 
 * 		  these addresses depend on how pins A1 and A2 are connected. 
 * 		  The user has to take care when connecting multiple current sensor to 
 * 		  not overlap i2c addresses as there is no pool address manager 
 * 		  implemented from the driver side
 */
enum INA226_i2c_addresses {
	GND_GND_ADDRESS = 0x80, /**< A1=GND, A0=GND */
	GND_VS_ADDRESS = 0x82,	/**< A1=GND, A0=VS */
	GND_SDA_ADDRESS = 0x84, /**< A1=GND, A0=SDA */
	GND_SCL_ADDRESS = 0x86, /**< A1=GND, A0=SCL */
	VS_GND_ADDRESS = 0x88,	/**< A1=VS,	 A0=GND */
	VS_VS_ADDRESS = 0x8A,	/**< A1=VS,  A0=VS */
	VS_SDA_ADDRESS = 0x8C,	/**< A1=VS,  A0=SDA */
	VS_SCL_ADDRESS = 0x8E,	/**< A1=VS,  A0=SCL */
	SDA_GND_ADDRESS = 0x90,	/**< A1=SDA, A0=GND */
	SDA_VS_ADDRESS = 0x92,	/**< A1=SDA, A0=VS */
	SDA_SDA_ADDRESS = 0x94,	/**< A1=SDA, A0=SDA */ 
	SDA_SCL_ADDRESS = 0x96,	/**< A1=SDA, A0=SCL */
	SCL_GND_ADDRESS = 0x98,	/**< A1=SCL, A0=GND */
	SCL_VS_ADDRESS = 0x9A,	/**< A1=SCL, A0=VS */
	SCL_SDA_ADDRESS = 0x9C, /**< A1=SCL, A0=SDA */
	SCL_SCL_ADDRESS = 0x9E	/**< A1=SCL, A0=SCL */ 
};


/**
 * @struct INA226_config_bits
 * @brief A structure to represent the bits of the Configuration register
 */
struct INA226_config_bits {
	uint16_t MODE:3;		/**< Operating Mode */
	uint16_t VSHCT:3;		/**< Shunt Voltage conversion time */ 
	uint16_t VBUSCT:3;		/**< Bus voltage conversion time */
	uint16_t AVG:3;			/**< Averaging Mode see datasht for combinations */
	uint16_t Reserved:3; 	/**< Reserved bits */
	uint16_t RST:1;		 	/**< Reset bit. Set to '1' generates system rst */
};

/**
 * @union INA226_config
 * @brief A union representing the Configuration register data, this allows us 
 * 		  to handle the data structure in different ways
 */
union INA226_config {
	struct INA226_config_bits bits;
	union buffer_16b buffer;
};

/**
 * @enum INA226_avg that represents the different combinations for AVG bit 
 * settings
 */
enum INA226_avg{
	AVG1	= 0b000, /**< 1 Sample avg */
	AVG4	= 0b001, /**< 4 Samples avg */
	AVG16	= 0b010, /**< 16 Samples avg */
	AVG64	= 0b011, /**< 64 Samples avg */
	AVG128	= 0b100, /**< 128 Samples avg */
	AVG256	= 0b101, /**< 256 Samples avg */
	AVG512	= 0b110, /**< 512 Samples avg */
	AVG1024	= 0b111, /**< 1024 Samples avg */ 
} ;

/**
 * @enum INA226_ct 
 * @brief enum that represents the different combinations for conversion 
 * 		  time settings
 */
enum INA226_ct {
	t140US  = 0b000, /**< 140uS conversion time */
	t204US  = 0b001, /**< 204uS conversion time */
	t332US  = 0b010, /**< 332uS conversion time */
	t588US  = 0b011, /**< 558uS conversion time */
	t1100US = 0b100, /**< 1100uS conversion time DEFAULT */
	t2116US = 0b101, /**< 2116uS conversion time */
	t4156US = 0b110, /**< 4156uS conversion time */
	t8244US = 0b111, /**< 8244uS conversion time */
};

/**
 * @enum INA226_mode 
 * @brief enum that represents the different combinations for operating 
 * 		  mode settings
 */
enum INA226_mode {
	POWER_DOWN 			= 0b000 | 0b100, /**< power down */
	SHUNT_VOLTAGE_TRIG 	= 0b001, 		 /**< shunt triggered */
	BUS_VOLTAGE_TRIG	= 0b010, 		 /**< bus triggered */
	SHUNT_AND_BUS_TRIG	= 0b011, 		 /**< shunt and bus triggeredc */
	SHUNT_VOLTAGE_CONT	= 0b101, 		 /**< shunt continuous */
	BUS_VOLTAGE_CONT	= 0b110,		 /**< bus continuous */
	SHUNT_AND_BUS_CONT 	= 0b111,		 /**< Shunt and bus continuous (def) */

};

/**
 * @enum INA226_mask_enable
 * @brief enum that represents the Mask/Enable register bits (R/W) possible
 * 		  values. This register selects the function that is enabled to control 
 * 		  the Alert pin as well as how that pin works. 
 * 		  If multiple functions are enabled, the highest significant bit 
 * 		  position Alert Function takes priority and responds to the Alert 
 * 		  Limit Trigger
 */
enum INA226_mask_enable {
	SOL  =	0x8000,
	SUL  =	0x4000,
	BOL  =	0x2000,
	BUL  =	0x1000,
	POL  =	0x0800,
	AFF  =	0x0010,
	CVRF =	0x0008,
	OVF	 =	0x0004,
	APOL =	0x0002,
	LEN  =	0x0001,
	DEFAULT = 0x0000,
};

/**
 * @union INA226_config
 * @brief A union representing the Configuration register data, this allows us to
 * 		  handle the data structure in different ways
 */
union INA226_mask_enable_buffer{
	enum INA226_mask_enable bits;
	union buffer_16b buffer;
};


#endif /* ina226_defs.h */

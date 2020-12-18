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


/** 
 * INA226 Configuration Register address 
 *	Default value: 0b01000001 00100111 0x4127
 *	Type: R/W
 */
#define INA226_CONFIG_REG		0x00	

/** 
 * INA226 Shunt Voltage Register address 
 *	Default value: 0b00000000 00000000 0x0000
 *	Type: R
 */
#define INA226_SHUNT_V_REG		0x01

/** 
 * INA226 Bus Voltage Register address 
 * 	Default value: 0b00000000 00000000 0x0000
 *	Type: R
 */
#define INA226_BUS_V_REG		0x02
/** 
 * INA226 Power Register address 
 * 	Default value: 0b00000000 00000000 0x0000
 *	Type: R
 */
#define INA226_POWER_REG		0x03
/** 
 * INA226 Current Register address 
 * 	Default value: 0b00000000 00000000 0x0000
 *	Type: R
 */
#define INA226_SHUNT_I_REG		0x04
/** 
 * INA226 Calibration Register address 
 * 	Default value: 0b00000000 00000000 0x0000
 *	Type: R
 */
#define INA226_CAL_REG			0x05
/** 
 * INA226 Mask/Enable Register address 
 * 	Default value: 0b00000000 00000000 0x0000
 *	Type: R
 */
#define INA226_MASK_EN_REG		0x06
/** 
 * INA226 Alert Limit Register address 
 * 	Default value: 0b00000000 00000000 0x0000
 *	Type: R
 */
#define INA226_ALERT_LIM_REG	0x07
/** 
 * INA226 Manufacturer ID Register address 
 * 	Default value: 0b0101010001001001 0x5449
 *	Type: R
 */
#define INA226_MAN_ID_REG		0xFE
/** 
 * INA226 Die ID Register address 
 * 	Default value: 0b0101010001001001 0x5449
 *	Type: R
 */
#define INA226_DIE_ID_REG		0xFF

/**
 * @enum _INA226_I2C_ADDRESS
 * @brief Enum class that represent the available I2C addresses for the ina226, 
 * 		  these addresses depend on how pins A1 and A2 are connected. 
 * 		  The user has to take care when connecting multiple current sensor to 
 * 		  not overlap i2c addresses as there is no pool address manager 
 * 		  implemented from the driver side
 */
typedef enum {
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
} _INA226_I2C_ADDRESS;


/**
 * @struct _INA226_CONFIG_bits
 * @brief A structure to represent the bits of the Configuration register
 */
typedef struct {
	uint16_t RST:1;		 	/**< Reset bit. Set to '1' generates system rst */
	uint16_t Reserved:3; 	/**< Reserved bits */
	uint16_t AVG:3;			/**< Averaging Mode see datasht for combinations */
	uint16_t VBUSCT:3;		/**< Bus voltage conversion time */
	uint16_t VSHCT:3;		/**< Shunt Voltage conversion time */ 
	uint16_t MODE:3;		/**< Operating Mode */
} _INA226_CONFIG_bits;

/**
 * @union INA226_CONFIG
 * @brief A union representing the Configuration register data, this allows us to
 * 		  handle the data structure in different ways
 */
typedef union {
	_INA226_CONFIG_bits bits;
	uint16_t all;
	struct {
		uint8_t b_low;
		uint8_t b_high;
	}
} INA226_CONFIG;

/**
 * @enum that represents the different combinations for AVG bit settings
 */
typedef enum {
	AVG1	= 0b000, /**< 1 Sample avg */
	AVG4	= 0b001, /**< 4 Samples avg */
	AVG16	= 0b010, /**< 16 Samples avg */
	AVG64	= 0b011, /**< 64 Samples avg */
	AVG128	= 0b100, /**< 128 Samples avg */
	AVG256	= 0b101, /**< 256 Samples avg */
	AVG512	= 0b110, /**< 512 Samples avg */
	AVG1024	= 0b111, /**< 1024 Samples avg */ 
} INA226_AVG_settings;

/**
 * @enum that represents the different combinations for conversion time settings
 */
typedef enum {
	140US  = 0b000, /**< 140uS conversion time */
	204US  = 0b001, /**< 204uS conversion time */
	332US  = 0b010, /**< 332uS conversion time */
	588US  = 0b011, /**< 558uS conversion time */
	1100US = 0b100, /**< 1100uS conversion time DEFAULT */
	2116US = 0b101, /**< 2116uS conversion time */
	4156US = 0b110, /**< 4156uS conversion time */
	8244US = 0b111, /**< 8244uS conversion time */
} INA226_CT_settings;

/**
 * @enum that represents the different combinations for operating mode settings
 */
typedef enum {
	POWER_DOWN 			= 0b000 | 0b100, /**< power down */
	SHUNT_VOLTAGE_TRIG 	= 0b001, 		 /**< shunt triggered */
	BUS_VOLTAGE_TRIG	= 0b010, 		 /**< bus triggered */
	SHUNT_AND_BUS_TRIG	= 0b011, 		 /**< shunt and bus triggeredc */
	SHUNT_VOLTAGE_CONT	= 0b101, 		 /**< shunt continuous */
	BUS_VOLTAGE_CONT	= 0b110,		 /**< bus continuous */
	SHUNT_AND_BUS_CONT 	= 0b111,		 /**< Shunt and bus continuous (def) */

} INA226_MODE_settings;

/**
 * @struct _INA226_MASK_ENABLE_bits
 * @brief Structure to represent the Mask/Enable register bits (R/W). 
 * 		  This register selects the function that is enabled to control the 
 * 		  Alert pin as well as how that pin works. If multiple functions are 
 * 		  enabled, the highest significant bit position Alert Function takes 
 * 		  priority and responds to the Alert Limit Trigger
 */
typedef struct {
	uint16_t SOL:1;			/**< Shunt Voltage over-voltage */
	uint16_t SUL:1; 		/**< Shunt Voltage under-voltage */
	uint16_t BOL:1;		 	/**< Bus voltage over-voltage */
	uint16_t BUL:1;	 		/**< Bus voltage under-voltage */
	uint16_t POL:1;			/**< Power over-limit */
	uint16_t CNVR:1;		/**< Conversion Ready */
	uint16_t RESERVED:5;	
	uint16_t AFF:1;			/**< Alert function flag */
	uint16_t CVRF:1;		/**< Conversion ready flag */
	uint16_t OVF:1;			/**< Math Overflow flag */
	uint16_t APOL:1;		/**< Alert Polarity bit */
	uint16_t LEN:1;			/**< Alert Latch enable */
} _INA226_MASK_ENABLE_bits;

#endif /* ina226_defs.h */

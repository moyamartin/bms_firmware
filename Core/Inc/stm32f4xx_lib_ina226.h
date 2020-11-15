/*
 * TI-INA226 Bi-Directional Current and Power Monitor library
 *
 * This is a library for the TI-INA226 Bi-Directional Current and Power
 * Monitor with I2C Compatible Interface.
 *
 * Written by Federico David Ceccarelli, 
 + Feel free to submite any comment to fededc88@gmail.com
 *
 * GPL license, all text here must be included in any redistribution.
 *
 */
 
 #ifndef __LIB_INA226_H__
 #define __LIB_INA226_H__
 
 
 /** INA226 I2C address **/
 //  Uncomment for selection, only one should be selected.
 
 // The device 7 bits addresses are shifted 1 bit to the left preventing
 // shifting before calling the interface

											// A0 +  A1     SLAVE ADDRESS
  #define INA226_I2C_ADDRESS (0x80) 		// GND  GND 	    1000000 (0x40)
//#define INA226_I2C_ADDRESS (0x82) 		// GND  VS     		1000001
//#define INA226_I2C_ADDRESS (0x84) 		// GND  SDA 		1000010
//#define INA226_I2C_ADDRESS (0x86) 		// GND  SCL 		1000011
//#define INA226_I2C_ADDRESS (0x88) 		// VS   GND 		1000100
//#define INA226_I2C_ADDRESS (0x8A) 		// VS   VS 			1000101
//#define INA226_I2C_ADDRESS (0x8C) 		// VS   SDA 		1000110
//#define INA226_I2C_ADDRESS (0x8E) 		// VS   SCL 		1000111
//#define INA226_I2C_ADDRESS (0x90) 		// SDA  GND 		1001000
//#define INA226_I2C_ADDRESS (0x92) 		// SDA  VS 	 		1001001
//#define INA226_I2C_ADDRESS (0x94) 		// SDA  SDA 		1001010
//#define INA226_I2C_ADDRESS (0x96) 		// SDA  SCL 		1001011
//#define INA226_I2C_ADDRESS (0x98) 		// SCL  GND 		1001100
//#define INA226_I2C_ADDRESS (0x9A) 		// SCL  VS 			1001101
//#define INA226_I2C_ADDRESS (0x9C) 		// SCL  SDA 		1001110
//#define INA226_I2C_ADDRESS (0x9E) 		// SCL  SCL 		1001111
 
 
 /*********************************************************************
								Register Map
 *********************************************************************/
 
//		REGISTER NAME 	   	  POINTER		  		POWER-ON RESET	TYPE(1)
//						   	  ADDRESS 	  		BINARY 		  HEX

// Configuration Register address
 #define INA226_CONFIG_REG 	  (0x00) 	// 01000001 00100111 0x4127	R/W
// Shunt Voltage Register address
 #define INA226_SHUNT_V_REG   (0x01)	// 00000000 00000000 0x00	R
// Bus Voltage Register address
 #define INA226_BUS_V_REG 	  (0x02)    // 00000000 00000000 0x00	R
// Power Register address
 #define INA226_POWER_REG 	  (0x03)    // 00000000 00000000 0x00 	R
// Current Register address
 #define INA226_SHUNT_I_REG   (0x04)	// 00000000 00000000 0x00	R
// Calibration Register address
 #define INA226_CAL_REG		  (0x05)	// 00000000 00000000 0x00	R/W
// Mask/Enable Register address
 #define INA226_MASK_EN_REG   (0x06)	// 00000000 00000000 0x00	R/W
// Alert Limit Register address
 #define INA226_ALERT_LIM_REG (0x07) 	// 00000000 00000000 0x00	R/W
// Manufacturer ID Register address
 #define INA226_MAN_ID_REG 	  (0xFE)	// 0101010001001001  0x5449	R
// Die ID Register address
 #define INA226_DIE_ID_REG    (0xFF) 	// 0010001001100000  0x00	R

 /*********************************************************************
						Registers descriptors
 *********************************************************************/

/* Configuration Register (00h) (Read/Write) Descriptions */

typedef struct {
	uint16_t RST:1;
	uint16_t Reserved:3;
	uint16_t AVG:3;
	uint16_t VBUSCT:3;
	uint16_t VSHCT:3;
	uint16_t MODE:3;
} _INA226_CONFIG_bit;


typedef union {
	_INA226_CONFIG_bit bit;
	uint16_t	all;
	struct {
		uint8_t b_low;
		uint8_t b_high;
	};
}_INA226_CONFIG;


// bit 15:		RST, Reset
// bit 12-14:	Reserved
typedef enum {
	INA226_CFG_SPARE	=	0x4000,		// 1
}INA226_CFG_SPARE_typedef;
// bit 9-11: 	AVG, Averaging Mode

// AVG Bit Settings[11:9] Combinations
typedef enum{
	INA226_AVG_1	=	0x0000,		// 1
	INA226_AVG_4	=	0x0200,		// 4
	INA226_AVG_16	=	0x0400,		// 16
	INA226_AVG_64	=	0x0600,		// 64
	INA226_AVG_128	=	0x0800,     // 128
	INA226_AVG_256	=	0x0A00,     // 256
	INA226_AVG_512	=	0x0C00,     // 512
	INA226_AVG_1024 =	0x0E00,     // 1024
} INA226_AVG_typedef;

// bit 6-8:	VBUSCT, Bus Voltage Conversion Time
// VBUSCT Bit Settings [8:6] Combinations
enum{
	INA226_VBUSCT_CONV_T_140us		=	0x0000,		// 140 μs
	INA226_VBUSCT_CONV_T_204us		=	0x0040,		// 204 μs
	INA226_VBUSCT_CONV_T_332us		=	0x0080,		// 332 μs
	INA226_VBUSCT_CONV_T_588us		=	0x00C0,		// 588 μs
	INA226_VBUSCT_CONV_T_1_1ms 		=	0x0100, 	// 1.1 ms
	INA226_VBUSCT_CONV_T_2_116ms	=	0x0140,		// 2.116 ms
	INA226_VBUSCT_CONV_T_4_156ms 	=	0x0180,		// 4.156 ms
    INA226_VBUSCT_CONV_T_8_244ms 	=	0x01C0,		// 8.244 ms
};

// bit 3-5: VSHCT: Shunt Voltage Conversion Time
// VSHCT  Bit Settings [3:5] Combinations
enum{
	INA226_VSHCT_CONV_T_140us		=	0x0000,		// 140 μs
	INA226_VSHCT_CONV_T_204us		=	0x0008,		// 204 μs
	INA226_VSHCT_CONV_T_332us		=	0x0010,		// 332 μs
	INA226_VSHCT_CONV_T_588us		=	0x0018,		// 588 μs
	INA226_VSHCT_CONV_T_1_1ms 		=	0x0020, 	// 1.1 ms
	INA226_VSHCT_CONV_T_2_116ms		=	0x0028,		// 2.116 ms
	INA226_VSHCT_CONV_T_4_156ms 	=	0x0030,		// 4.156 ms
    INA226_VSHCT_CONV_T_8_244ms 	=	0x0038,		// 8.244 ms
};

// bit 0-2: MODE, Operating Mode
// Mode Settings [2:0] Combinations
enum{
	INA226_MODE_POWERDOWN				=	0x0000,	// Power-Down (or Shutdown)
	INA226_MODE_SVOLT_TRIGGERED			=	0x0001,	// Shunt Voltage, Triggered
	INA226_MODE_BVOLT_TRIGGERED			=	0x0002,	// Bus Voltage, Triggered
	INA226_MODE_SANDBVOLT_TRIGGERED		=	0x0003,	// Shunt and Bus, Triggered
	INA226_MODE_ADC_OFF					=	0x0004,	// Power-Down (or Shutdown)
	INA226_MODE_SVOLT_CONTINUOUS		=	0x0005,	// Shunt Voltage, Continuous
	INA226_MODE_BVOLT_CONTINUOUS		=	0x0006,	// Bus Voltage, Continuous
	INA226_MODE_SANDBVOLT_CONTINUOUS	=	0x0007,	// Shunt and Bus, Continuous
};


/* Mask/Enable Register (06h) (Read/Write) Descriptions */

// bit 15: SOL: Shunt Voltage Over-Voltage
// bit 14: SUL: Shunt Voltage Under-Voltage
// bit 13: BOL: Bus Voltage Over-Voltage
// bit 12: BUL: Bus Voltage Under-Voltage
// bit 11: POL: Power Over-Limit
// bit 10: CNVR: Conversion Ready
// bit 9-5: Reserved
// bit 4: AFF: Alert Function Flag
// bit 3: CVRF: Conversion Ready Flag
// bit 2: OVF: Math Overflow Flag
// bit 1: APOL: Alert Polarity bit; sets the Alert pin polarity.
			//	1 = Inverted (active-high open collector)
			//	0 = Normal (active-low open collector) (default)
//bit 0: LEN: Alert Latch Enable; configures the latching feature of the Alert pin and Alert Flag bits.
			//	1 = Latch enabled
			//	0 = Transparent (default)

enum{
	INA226_MASK_ENABLE_SOL	=	0x8000,
	INA226_MASK_ENABLE_SUL	=	0x4000,
	INA226_MASK_ENABLE_BOL	=	0x2000,
	INA226_MASK_ENABLE_BUL	=	0x1000,
	INA226_MASK_ENABLE_POL	=	0x0800,
	INA226_MASK_ENABLE_AFF	=	0x0010,
	INA226_MASK_ENABLE_CVRF	=	0X0008,
	INA226_MASK_ENABLE_OVF	=	0x0004,
	INA226_MASK_ENABLE_APOL =	0x0002,
	INA226_MASK_ENABLE_LEN =	0x0001,
};

/* Shunt Voltage Register (01h) (Read-Only) Descriptor */
#define INA226_Shunt_Voltage_LSB 0.0000025f  // LSB: 2.5 μV.

/* Bus Voltage Register (02h) (Read-Only) Descriptor */
#define INA226_VBUS_LSB 0.00125f // LSB = 1.25 mV.

/* Die ID Register (FFh) (Read-Only) Descriptor */
// Mask for Device ID Bits 4-15
#define INA226_DID_MASK	0xFFF0
// Mask for Revision ID Bits 4-0
#define INA226_RID_MASK	0x000F

// Time Out definition for polling operations
#define INA226_POLLING_TIMEOUT	1000	// 1s - in [ms]

/* Data Structures definitions*/
typedef struct {
	uint8_t	Reg_address;
	uint8_t data_h;
	uint8_t data_l;
} _INA226_BUFF;

typedef struct {
	uint8_t inited:1;
	uint8_t spare:7;
} _INA226_FLAGS;


/* Functions definitions */

HAL_StatusTypeDef ina226_I2C1_Init(void);
HAL_StatusTypeDef ina226_init(void);
HAL_StatusTypeDef ina226_set_config(_INA226_CONFIG *ina226_config);
HAL_StatusTypeDef ina226_calibrate(uint16_t Rshunt, float Max_Expected_I);
HAL_StatusTypeDef ina226_get_current_polling(void);
void ina226_Error_Handler(void);

#endif
 
//
// End of file.
//


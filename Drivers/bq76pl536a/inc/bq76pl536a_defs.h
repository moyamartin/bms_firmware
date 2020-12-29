
/******************************************************************************
 *
 * @file	bq76pl536a_defs.h
 * @brief 	Texas Instruments BQ76PL536A register and constants definitions
 * @version	v1.00f00
 * @date	27, Dec. 2020
 * @author	CECARELLI, Federico (fededc88@gmail.com),
 * 			MOYA, Martin		(moyamartin1@gmail.com),
 * 			SANTOS, Lucio		(lusho2206@gmail.com)
 * @copyright GPL license, all text here must be included in any redistribution
 *****************************************************************************/

#ifndef __BQ76PL536A_DEFS_H_
#define __BQ76PL536A_DEFS_H_

/** 
 * BQ76PL536A Status Register address 
 *	Default value: 0b00000000 0x00
 *	Type: R - GROUP 1
 */
#define DEVICE_STATUS_REG		(0x00)

/** 
 * BQ76PL536A GPAI Low Data Register address 
 *	Default value: 0b00000000 0x00
 *	Type: R - GROUP 1
 */
#define GPAI_LOW_REG			(0x01)

/** 
 * BQ76PL536A GPAI High Data Register address 
 *	Default value: 0b00000000 0x00
 *	Type: R - GROUP 1
 */
#define GPAI_HIGH_REG			(0x02)

/** 
 * BQ76PL536A VCELL1 Low Data Register address 
 *	Default value: 0b00000000 0x00
 *	Type: R - GROUP 1
 */
#define VCELL1_LOW_REG			(0x03)

/** 
 * BQ76PL536A VCELL1 High Data Register address 
 *	Default value: 0b00000000 0x00
 *	Type: R - GROUP 1
 */
#define VCELL1_HIGH_REG			(0x04)

/** 
 * BQ76PL536A VCELL2 Low Data Register address 
 *	Default value: 0b00000000 0x00
 *	Type: R - GROUP 1
 */
#define VCELL2_LOW_REG			(0x05)

/** 
 * BQ76PL536A VCELL2 High Data Register address 
 *	Default value: 0b00000000 0x00
 *	Type: R - GROUP 1
 */
#define VCELL2_HIGH_REG			(0x06)

/** 
 * BQ76PL536A VCELL3 Low Data Register address 
 *	Default value: 0b00000000 0x00
 *	Type: R - GROUP 1
 */
#define VCELL3_LOW_REG			(0x07)

/** 
 * BQ76PL536A VCELL3 High Data Register address 
 *	Default value: 0b00000000 0x00
 *	Type: R - GROUP 1
 */
#define VCELL3_HIGH_REG			(0x08)

/** 
 * BQ76PL536A VCELL4 Low Data Register address 
 *	Default value: 0b00000000 0x00
 *	Type: R - GROUP 1
 */
#define VCELL4_LOW_REG			(0x09)

/** 
 * BQ76PL536A VCELL4 High Data Register address 
 *	Default value: 0b00000000 0x00
 *	Type: R - GROUP 1
 */
#define VCELL4_HIGH_REG			(0x0a)

/** 
 * BQ76PL536A VCELL5 Low Data Register address 
 *	Default value: 0b00000000 0x00
 *	Type: R - GROUP 1
 */
#define VCELL5_LOW_REG			(0x0b)

/** 
 * BQ76PL536A VCELL5 High Data Register address 
 *	Default value: 0b00000000 0x00
 *	Type: R - GROUP 1
 */
#define VCELL5_HIGH_REG			(0x0c)

/** 
 * BQ76PL536A VCELL6 Low Data Register address 
 *	Default value: 0b00000000 0x00 
 *	Type: R - GROUP 1
 */
#define VCELL6_LOW_REG			(0x0d)

/** 
 * BQ76PL536A VCELL6 High Data Register address 
 *	Default value: 0b00000000 0x00
 *	Type: R - GROUP 1
 */
#define VCELL6_HIGH_REG			(0x0e)

/** 
 * BQ76PL536A TEMPERATURE1 Low Data Register address 
 *	Default value: 0b00000000 0x00
 *	Type: R - GROUP 1
 */
#define TEMPERATURE1_LOW_REG 	(0x0f)

/** 
 * BQ76PL536A TEMPERATURE1 High Data Register address 
 *	Default value: 0b00000000 0x00
 *	Type: R - GROUP 1
 */
#define TEMPERATURE1_HIGH_REG	(0x10)

/** 
 * BQ76PL536A TEMPERATURE2 Low Data Register address 
 *	Default value: 0b00000000 0x00
 *	Type: R - GROUP 1
 */
#define TEMPERATURE2_LOW_REG 	(0x11)

/** 
 * BQ76PL536A TEMPERATURE2 High Data Register address 
 :	Default value: 0b00000000 0x00
 *	Type: R - GROUP 1
 */
#define TEMPERATURE2_HIGH_REG	(0x12)

/** 
 * BQ76PL536A Alert Status Register address 
 *	Default value: 0b10000000 0x80
 *	Type: R/W - GROUP 2
 */
#define ALERT_STATUS_REG		(0x20)

/** 
 * BQ76PL536A Fault Status Register address 
 *	Default value: 0b00001000 0x08
 *	Type: R/W - GROUP 2
 */
#define ALERT_STATUS_REG		(0x21)

/** 
 * BQ76PL536A COV Fault Status Register address 
 *	Default value: 0b00000000 0x00
 *	Type: R - GROUP 1
 */
#define COV_STATUS_REG			(0x22) 

/** 
 * BQ76PL536A CUV Fault Status Register address 
 *	Default value: 0b00000000 0x00
 *	Type: R - GROUP 1
 */
#define CUV_STATUS_REG			(0x23)

/** 
 * BQ76PL536A Preresult A Status Register address 
 *	Default value: 0b00000000 0x00
 *	Type: R - GROUP 1
 */
#define PRESULT_A_REG			(0x24)

/** 
 * BQ76PL536A Preresult B Status Register address 
 *	Default value: 0b00000000 0x00
 *	Type: R - GROUP 1
 */
#define PRESULT_B_REG			(0x25)

/** 
 * BQ76PL536A ADC Control Register address 
 *	Default value: 0b00000000 0x00
 *	Type: R/W - GROUP 2
 */
#define ADC_CONTROL_REG			(0x30)

/** 
 * BQ76PL536A IO Control Register address 
 *	Default value: 0b00000000 0x00
 *	Type: R/W - GROUP 2
 */
#define IO_CONTROL_REG			(0x31)

/** 
 * BQ76PL536A CB Control Register address 
 *	Default value: 0b00000000 0x00
 *	Type: R/W - GROUP 2
 */
#define CB_CONTROL_REG			(0x32)

/** 
 * BQ76PL536A CB Time Register address 
 *	Default value: 0b00000000 0x00
 *	Type: R/W - GROUP 2
 */
#define CB_TIME_REG				(0x33)

/** 
 * BQ76PL536A ADC Convert address 
 *	Default value: 0b00000000 0x00
 *	Type: R/W - GROUP 2
 */
#define ADC_CONVERT_REG			(0x34)

/** 
 * BQ76PL536A Shadow Control address 
 *	Default value: 0b00000000 0x00
 *	Type: R/W - GROUP 2
 */
#define SHADOW_CONTROL_REG		(0x3b)

/** 
 * BQ76PL536A Reset address 
 *	Default value: 0b00000000 0x00
 *	Type: W - GROUP 2
 */
#define RESET_REG				(0x3c)

/** 
 * BQ76PL536A Test Select address 
 *	Default value: 0b00000000 0x00
 *	Type: R/W - GROUP 2
 */
#define TEST_SELECT_REG			(0x3d)

/** 
 * BQ76PL536A EPROM programming mode enable address 
 *	Default value: 0b00000000 0x00
 *	Type: R/W - GROUP 3
 */
#define E_EN_REG				(0x3f)

/** 
 * BQ76PL536A Function Configuration address 
 *	Default value: 0b00000000 0x00
 *	Type: R/W - GROUP 3
 */
#define FUNCTION_CONFIG_REG		(0x40)

/** 
 * BQ76PL536A I/O Pin Configuration address 
 *	Default value: 0b00000000 0x00
 *	Type: R/W - GROUP 3
 */
#define IO_CONFIG_REG			(0x41)

/** 
 * BQ76PL536A I/O Pin Configuration address 
 *	Default value: 0b00000000 0x00
 *	Type: R/W - GROUP 3
 */
#define FUNCTION_CONFIG_REG		(0x41)

#endif /* bq76pl536a_defs.h */

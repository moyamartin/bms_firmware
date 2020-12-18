/******************************************************************************
 *
 * @file	ina226.h
 * @brief 	Texas Instruments INA226 declarations of function and high level
 * 			structures
 * @version	v1.00f00
 * @date	17, Dec. 2020
 * @author	CECARELLI, Federico (fededc88@gmail.com),
 * 			MOYA, Martin		(moyamartin1@gmail.com),
 * 			SANTOS, Lucio		(lusho2206@gmail.com)
 * @copyright GPL license, all text here must be included in any redistribution
 *****************************************************************************/

#ifndef __INA226_H_
#define __INA226_H_

#include "ina226_defs.h"

/**
 * @struct INA226
 * @brief This structure defines a HAL layer for the device data structure
 */
typedef struct {
	_INA226_I2C_ADDRESS address;
	_INA226_CONFIG config_data;	
	_INA226_CT_settings ct_settings;
	_INA226_MODE_settings mode_settings;
	_INA226_MASK_ENABLE_bits mask_enable_bits;
} INA226;


/**
 *
 */
uint8_t ina226_init(INA226 * ina226_instance, 
					INA226_AVG_settings avg_settings,
					INA226_CT_settings vbusct_settings,
					INA226_CT_settings vshct_settings,
					INA226_MODE_settings mode_settings);


#endif /* ina226.h */

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

#include <stdint.h>
#include "ina226_defs.h"

#if defined(USE_HAL_DRIVER) && defined(STM32F407xx)
	// Support for stm32f4xx devices
	#include "arm_math.h"
	#include "stm32f4xx_hal.h"
	/* 
	 * All the instances of these drivers share only one I2C bus, that we asume
	 * that it is already initialized on the main before the infinite loop. 
	 * This design takes into account that the project has been created with 
	 * the STM32CubeMX which creates the function needed to initialize the i2c
	 * bus.
	 *
	 * We also asume that the I2C bus used is the hi2c1
	 */
	extern I2C_HandleTypeDef 	hi2c1;
	#define INA226_INTERFACE	hi2c1
	#define INA226_TIMEOUT		1000
#elif defined(__AVR__)

#endif

/**
 * @enum  INA226_status
 * @brief defines several possible status of the ina226 driver to inform the
 * 		  user the current result of an operation
 */
typedef enum {
	OK = 0,
	FAIL = -1,
	MAN_ID_MISMATCH = -2,
	DIE_ID_MISMATCH = -3,
	CONFIG_ERROR = -4,
	I2C_TRANSMISSION_ERROR = -5,
	CAL_ERROR = -6,
	MASK_EN_ERROR = -7,
	FLAGS_NOT_CLEARED = -8,
} INA226_status;

typedef struct {
	uint8_t reg_address;
	buffer_16b buffer;
} INA226_buff;

/**
 * @struct INA226
 * @brief This structure defines a HAL layer for the device data structure
 */
typedef struct {
	INA226_i2c_address address;
	INA226_config config;	
	INA226_mask_enable mask_enable;
	INA226_mode current_active_mode;
	float32_t r_shunt;
	float32_t max_expected_current;
	float32_t current_LSB;
} INA226;


/**
 * @func  ina226_init
 * @brief Initializes the ina226 instance passed as an argument with the
 * 		  i2c desired address and all the configurations that the user will
 * 		  use with the  sensor
 * @params[in] ina226: INA226 struct instance that will be used during
 * 			   program execution
 * @params[in] address: i2c address that the sensor will use, this variable
 *			   is limited by the available addresses defined in the
 *			   INA226_i2c_address enum
 * @params[in] avg: Number of averages that the sensor will make for
 * 			   each measurement, this value is limited by the
 * 			   INA226_avg_settings enum typedef
 * @params[in] vbusct: voltage bus conversion time, this value is defined by the
 * 			   INA226_ct_settings enum typedef.
 * @params[in] vshct: shunt voltage conversion time, this value is defined by
 * 			   INA226_ct_settings enum typedef
 * @params[in] mode: ina226 operation mode., this value is defined by the
 * 			   INA226_mode_settings enum typedef
 * @return INA226_status [OK|MAN_ID_MISMATCH|DIE_ID_MISMATCH|CONFIG_ERROR|
 * 						  CAL_ERROR|MASK_EN_ERROR]
 */
INA226_status ina226_init(INA226 * ina226, INA226_i2c_address address,
						  float32_t r_shunt, float32_t max_expected_current, 
						  INA226_avg avg, INA226_ct vbusct, INA226_ct vshct,
						  INA226_mode mode, INA226_mask_enable mask_enable);

/**
 * @func ina226_reset
 * @brief Generates a reset of TI's INA226 that is the same as power-on reset.
 * 		  Resets all register to default values.
 * @params[in] ina226: INA226 struct instance that will be reset
 * @returns [OK|CONFIG_ERROR]
 */
INA226_status ina226_reset(INA226 * ina226);

/**
 * @func ina226_get_current
 * @brief Reads the ina226 current register
 * @params[in] ina226: INA226 struct instance that holds the data structure for
 * 			   the sensor
 * @returns INA226_status indicating if the operation was successful or not
 * 			[OK|I2C_TRANSMISSION_ERROR]
 */
INA226_status ina226_get_current(INA226 * ina226, float32_t * current);

/**
 * @func ina226_get_vbus
 * @brief Reads the ina226 vbus register and the avg measured value
 * @params[in] ina226: INA226 struct instance that holds the data structure for
 * 			   the sensor
 * @returns INA226_status indicating if the operation was successful or not
 * 			[OK|I2C_TRANSMISSION_ERROR]
 */
INA226_status ina226_get_vbus(INA226 * ina226, float32_t * vbus);

/**
 * @func ina226_get_vbus
 * @brief Reads the ina226 vbus register and gets the avg measured value
 * @params[in] ina226: INA226 struct instance that holds the data structure for
 * 			   the sensor
 * @returns INA226_status indicating if the operation was successful or not
 * 			[OK|I2C_TRANSMISSION_ERROR]
 */
INA226_status ina226_get_vshunt(INA226 * ina226, float32_t * vshunt);

/**
 * @func ina226_get_pwr
 * @brief Reads the ina226 pwr register and gets the avg measured value
 * @params[in] ina226: INA226 struct instance that holds the data structure for
 * 			   the sensor
 * @returns INA226_status indicating if the operation was successful or not
 * 			[OK|I2C_TRANSMISSION_ERROR]
 */
INA226_status ina226_get_pwr(INA226 * ina226, float32_t * pwr);

/**
 * @func ina226_set_avg
 * @brief Sets the number of measurements to calculate the average
 * @params[in] ina226: INA226 struct instance that holds the data structure for
 * 					   the sensor
 * @params[in] avg: INA226_avg data structure that represents the avg value to
 * 			   set
 * @returns INA226_status indicating if the operation was successful or not
 * 			[OK|I2C_TRANSMISSION_ERROR]
 */
INA226_status ina226_set_avg(INA226 * ina226_instance, INA226_avg avg);

/**
 * @func ina226_set_vbus_ct
 * @brief Sets Vbus conversion time
 * @params[in] ina226: INA226 struct instance that holds the data structure for
 * 					   the sensor
 * @params[in] ct: INA226_ct data structure that represents the possible value
 * 			   conversion time
 * @returns ina226_state [OK|I2C_TRANSMISSION_ERROR]
 */
INA226_status ina226_set_vbus_ct(INA226 * ina226_instance, INA226_ct ct);

/**
 * @func ina226_set_vshunt_ct
 * @brief Sets Vshunt conversion time
 * @params[in] ina226: INA226 struct instance that holds the data structure for
 * 					   the sensor
 * @params[in] ct: INA226_ct data structure that represents the possible value
 * 			   conversion time
 * @returns ina226_state [OK|I2C_TRANSMISSION_ERROR]
 */
INA226_status ina226_set_vshunt_ct(INA226 * ina226_instance, INA226_ct ct);

/**
 * @func  ina226_set_mode
 * @brief Sets the operating mode for the ina226 instance
 * @params[in] ina226: INA226 data structure that holds the instance to modify
 * @params[in] mode: Operation mode modeled by the INA226_mode enum typedef
 * @return 	   INA226_status enum [OK|I2C_TRANSMISSION_ERROR]
 */
INA226_status ina226_set_mode(INA226 * ina226, INA226_mode mode);

/**
 * @func  ina226_set_calibration
 * @brief calibrates the ina226 sensor given the maximum expected current and
 * 		  the r_shunt value. This functions is public, because the idea is to 
 * 		  be able to recalibrate the sensor on the fly if the expected max 
 * 		  current changes
 * @params[in] ina226_instance: INA226 structure that holds the ina226 instance
 * 			   being used
 * @params[in] r_shunt: float32_t value of the r_shunt connected to IN+ and IN-
 * @params[in] max_expected_current: float32_t value that represents the maximum
 *			   expected current that the sensur will measure
 * @returns INA226_status [OK | I2C_TRANSMISSION_ERROR]
 */
INA226_status ina226_set_calibration(INA226 * ina226, 
									 float32_t r_shunt, 
									 float32_t max_expected_current);

/**
 * @func  ina226_set_maks_enable
 * @brief calibrates the ina226 sensor given the maximum expected current and
 * 		  the r_shunt value. This functions is public, because the idea is to 
 * 		  be able to recalibrate the sensor on the fly if the expected max 
 * 		  current changes
 * @params[in] ina226_instance: INA226 structure that holds the ina226 instance
 * 			   being used
 * @params[in] r_shunt: float32_t value of the r_shunt connected to IN+ and IN-
 * @params[in] max_expected_current: float32_t value that represents the maximum
 *			   expected current that the sensur will measure
 * @returns INA226_status [OK | I2C_TRANSMISSION_ERROR]
 */
INA226_status ina226_set_mask_enable(INA226 * ina226, 
									 INA226_mask_enable maks_enable);

#endif /* ina226.h */

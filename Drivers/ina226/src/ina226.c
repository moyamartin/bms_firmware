#include "ina226.h"

/**
 * @fn	  ina226_writereg
 * @brief Write a register of TIs INA226 device.
 * 		  The user of this library is responsible of defining this 
 * 		  function. As an example, we have only defined the functionalty for
 * 		  STM32F407xx
 */
static ina226_status ina226_writereg(INA226_i2c_address i2c_address,
									 uint8_t reg, uint16_t value);

/**
 * @fn	  ina226_readreg
 * @brief Read a register of TIs INA226 device.
 * 		  The user of this library is responsible of defining this 
 * 		  function. As an example, we have only defined the functionalty for
 * 		  STM32F407xx
 */
static uint16_t ina226_readreg(INA226_i2c_address i2c_address, uint8_t reg);

/**
 * @fn	  calculate_current_lsb
 * @brief Calculates the current Less Significant Bit value by taking as a
 * 		  parameter the maximum expected current, following the next equation
 *
 *				 	max_expected_current
 * 		  cal_val = --------------------
 *							2^15	
 *
 */
static float32_t calculate_current_lsb(float32_t max_expected_current)
{
	return max_expected_current >> 15;
}

/**
 * @fn	  calculate_cal_val	
 * @brief Calculates the calibration register's value given the r_shunt value
 * 		  and the current_LSB.
 *
 *						 0.00512 
 * 		  cal_val = -------------------
 *					current_LSB*r_shunt
 *
 */
static uint16_t calculate_cal_val(float32_t r_shunt, 
								  float32_t current_LSB)
{
	return (uint16_t) ((0.00512)/(current_LSB*r_shunt));
}

INA226_status ina226_init(INA226 * ina226, INA226_i2c_address address,
						  float32_t r_shunt, float32_t max_expected_current,	
						  INA226_avg avg,
						  INA226_ct vbusct, INA226_ct_settings vshct,
						  INA226_mode mode,
						  INA226_mask_enable mask_enable)
{
	INA226_config config_buffer;		
	config_buffer.bits.AVG = avg;
	config_buffer.bits.VBUSCT = vbusct;
	config_buffer.bits.VSHCT = vshct;
	config_buffer.bits.MODE = mode;

	/* 
	 * first check if we get the correct Die Ids and Manufacturing Ids from the
	 * chip
	 */
	if(ina226_readreg(address, INA226_MAN_ID_REG) != INA226_MAN_ID_VAL){
		return MAN_ID_MISMATCH;
	}

	if(ina226_readreg(address, INA226_DIE_ID_REG) != INA226_DIE_ID_REG){
		return DIE_ID_MISMATCH;
	}

	ina226->address = address;

	/*
	 * At this step we are having a proper communication with the ina226, so we
	 * can start sending all the configuration data
	 */

	// set ina226 configuration register if, and only if, the initialization
	// configuration is different from the default values
	if(config_buffer.buffer.all != INA226_CONFIG_DEFAULT){
		if(ina226_writereg(address, INA226_CONFIG_REG, 
						   ina226->configs.all) != 
				I2C_TRANSMISSION_ERROR){
			return CONFIG_ERROR;
		}
	}
	ina226->config = config_buffer;
	ina226->current_active_mode = mode;

	if(ina226_set_calibration(ina226_instance, r_shunt, mad_expected_current) 
			!= OK){
		return CAL_ERROR;
	}

	// set ina226 mask/enable register data
	if(ina226_writereg(address, INA226_MASK_EN_REG, mask_enable) != 
			I2C_TRANSMISSION_ERROR){
		return MASK_EN_ERROR;
	}
	ina226_instance->mask_enable = mask_enable;
	return OK;	
}


INA226_status ina226_reset(INA226 * ina226);
{
	INA226_config_bits bits_buffer = ina226.config.bits;
	bits_buffer.RST = 1;
	if(ina226_writreg(address, INA226_CONFIG_REG, bits_buffer) 
			!= OK){
		return CONFIG_ERROR;
	}
	ina226_instance.config.bits = bits_buffer;
	return OK;
}

float32_t ina226_get_current(INA226 * ina226)
{
	uint16_t current_reg_val = ina226_readreg(ina226->address, 
											  INA226_CURRENT_REG)
	return (float32_t) current_reg_val*ina226->current_LSB;
}

float32_t ina226_get_vbus(INA226 * ina226)
{
	uint16_t vbus_reg_val = ina226_readreg(ina226->address,
										   INA226_VBUS_REG);
	return (float32_t) vbus_reg_val*INA226_VBUS_LSB_VAL
}

float32_t ina226_get_vshunt(INA226 * ina226)
{
	uint16_t vshunt_reg_val = ina226_readreg(ina226->address,
											 INA226_VSHUNT_REG);
	return (float32_t) vshunt_reg_val*INA226_VSHUNT_LSB_VAL;
}

float32_t ina226_get_pwr(INA226 * ina226)
{
	uint16_t pwr_reg_val = ina226_readreg(ina226_address,
										  INA226_PWR_REG);
	return (float32_t) pwr_reg_val*ina226->current_LSB/1000;
}

INA226_status ina226_set_avg(INA226 * ina226, INA226_avg avg)
{
	ina226->config.bits.AVG = avg;	
	return ina226_writereg(ina226_instance->address, INA226_CONFIG_REG,
						   ina226_instance->config.buffer.all);
}

INA226_status ina226_set_vbus_ct(INA226 * ina226_instance, INA226_ct ct)
{
	ina226_instance->config.bits.VBUSCT = ct;
	return ina226_writereg(ina226_instance->address, INA226_CONFIG_REG,
						   ina226_instance->config.buffer.all);
}

INA226_status ina226_set_vshunt_ct(INA226 * ina226_instance, INA226_ct ct)
{
	ina226_instance->config.bits.VSHCT = ct;
	return ina226_writereg(ina226_instance->address, INA226_CONFIG_REG,
						   ina226_instance->config.buffer.all);
}

INA226_status ina226_set_mode(INA226 * ina226_instance, INA226_mode mode)
{
	ina226_instance->config.bits.mode = mode;
	return ina226_writereg(ina226_instance->address, INA226_CONFIG_REG, 
						   ina226_instance->config.buffer.all) 
}

INA226_status ina226_set_calibration(INA226 * ina226_instance, 
									 float32_t r_shunt,
									 float32_t max_expected_current)
{
	// Calculate new current_LSB
	ina226_instance->current_LSB = calculate_current_lsb(max_expected_current);
	// Calculate calibration value
	uint16_t cal_data_buffer = calculate_cal_val(r_shunt, 
												 ina226_instance->current_LSB);
	ina226_instance->r_shunt = r_shunt;
	ina226_instance->max_expected_cirrent = max_expected_current;
	return ina226_writereg(ina226_instance->address, INA226_CAL_REG, 
						   cal_data_buffer);
}

INA226_status ina226_set_mask_enable(INA226 * ina226,
									 INA226_mask_enable mask_enable)
{
	ina226_instance->mask_enable = mask_enable;
	return ina226_writereg(ina226_instance->address, INA226_MASK_EN_REG,
						   mask_enable);
}

INA226_status ina226_clear_flags(INA226 * ina226)
{
	uint16_t buffer = ina226_readreg(ina226->address, INA226_MASK_EN_REG);
	return OK;
}

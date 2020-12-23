#include "definitions.h"
#include "ina226.h"

/**
 * @fn	  ina226_writereg
 * @brief Write a register of TIs INA226 device.
 * 		  The user of this library is responsible of defining this 
 * 		  function. As an example, we have only defined the functionalty for
 * 		  STM32F407xx
 * @params[in] i2c_address: INA226_i2c_address enum address pointing to the
 * 			   desired address
 * @params[in] reg: register address to write to
 * @params[in] value: 16 bits value to write to register
 */
static INA226_status ina226_writereg(INA226_i2c_address i2c_address,
									 uint8_t reg, uint16_t value)
{
#if defined(USE_HAL_DRIVER) && defined(STM32F407xx)
	if(HAL_I2C_Mem_Write(&INA226_INTERFACE, (uint16_t) i2c_address,
							   reg, INA226_ADDRESS_SIZE, (uint8_t *) &value,
							   INA226_TX_BUFF_SIZE, INA226_TIMEOUT) != HAL_OK){
		return I2C_TRANSMISSION_ERROR;
	}
	return OK;

#elif defined(__AVR__)
#endif
}

/**
 * @fn	  ina226_readreg
 * @brief Read a register of TIs INA226 device.
 * 		  The user of this library is responsible of defining this 
 * 		  function. As an example, we have only defined the functionalty for
 * 		  STM32F407xx
 */
static INA226_status ina226_readreg(INA226_i2c_address i2c_address, 
									uint8_t reg, uint16_t * value)
{ 
#if defined(USE_HAL_DRIVER) && defined(STM32F407xx)
	uint16_t rx_data;
	if(HAL_I2C_Mem_Read(&INA226_INTERFACE, (uint16_t) i2c_address, 
							  reg, INA226_ADDRESS_SIZE, (uint8_t *)&rx_data, 
							  INA226_RX_BUFF_SIZE, INA226_TIMEOUT) != HAL_OK) {
		return I2C_TRANSMISSION_ERROR;
	}
	*value = ((rx_data >> 8) & 0x00FF) | ((rx_data << 8) & 0xFF00);
	return OK;
#elif defined(__AVR__)
#endif
}

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
	return max_expected_current/32768;
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
						  INA226_avg avg, INA226_ct vbusct, INA226_ct vshct, 
						  INA226_mode mode, INA226_mask_enable mask_enable)
{
	INA226_config config_buffer;		
	config_buffer.bits.AVG = avg;
	config_buffer.bits.Reserved = 0b100;
	config_buffer.bits.VBUSCT = vbusct;
	config_buffer.bits.VSHCT = vshct;
	config_buffer.bits.MODE = mode;

	/* 
	 * first check if we get the correct Die Ids and Manufacturing Ids from the
	 * chip
	 */
	uint16_t man_id_val;
	if(ina226_readreg(address, INA226_MAN_ID_REG, &man_id_val) != OK){
		return I2C_TRANSMISSION_ERROR;
	}

	if(man_id_val != INA226_MAN_ID_VAL){
		return MAN_ID_MISMATCH;
	}
	uint16_t die_id_val;
	if(ina226_readreg(address, INA226_DIE_ID_REG, &die_id_val) != OK){
		return I2C_TRANSMISSION_ERROR;
	}
	if(die_id_val != INA226_DIE_ID_VAL){
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
		if(ina226_writereg(ina226->address, INA226_CONFIG_REG, 
						   config_buffer.buffer.all) != 
				OK){
			return CONFIG_ERROR;
		}
	}
	uint16_t buffer;
	ina226_readreg(ina226->address, INA226_CONFIG_REG, &buffer);
	ina226->config = config_buffer;
	ina226->current_active_mode = mode;

	if(ina226_set_calibration(ina226, r_shunt, max_expected_current) 
			!= OK){
		return CAL_ERROR;
	}
	ina226_readreg(ina226->address, INA226_CAL_REG, &buffer);

	// set ina226 mask/enable register data
	if(ina226_writereg(ina226->address, INA226_MASK_EN_REG, mask_enable) != 
			OK){
		return MASK_EN_ERROR;
	}
	ina226->mask_enable = mask_enable;
	return OK;	
}


INA226_status ina226_reset(INA226 * ina226)
{
	ina226->config.bits.RST = 1;
	if(ina226_writereg(ina226->address, INA226_CONFIG_REG, 
					   ina226->config.buffer.all) != OK){
		return CONFIG_ERROR;
	}
	return OK;
}

INA226_status ina226_get_current(INA226 * ina226, float32_t * current)
{
	uint16_t current_reg_val;
	if(ina226_readreg(ina226->address, INA226_CURRENT_REG, 
					  &current_reg_val) != OK){
		return I2C_TRANSMISSION_ERROR;
	}
	*current = (float32_t) current_reg_val*ina226->current_LSB;
	return OK;
}

INA226_status ina226_get_vbus(INA226 * ina226, float32_t * vbus)
{
	uint16_t vbus_reg_val;
	if(ina226_readreg(ina226->address, INA226_VBUS_REG, &vbus_reg_val) != OK){
		return I2C_TRANSMISSION_ERROR;
	}
	*vbus = (float32_t) vbus_reg_val*INA226_VBUS_LSB_VAL;
	return OK;
}

INA226_status ina226_get_vshunt(INA226 * ina226, float32_t * vshunt)
{
	uint16_t vshunt_reg_val;
	if(ina226_readreg(ina226->address, INA226_VSHUNT_REG, 
					  &vshunt_reg_val) != OK){
		return I2C_TRANSMISSION_ERROR;
	}
	*vshunt = (float32_t) vshunt_reg_val*INA226_VSHUNT_LSB_VAL;
	return OK;
}

INA226_status ina226_get_pwr(INA226 * ina226, float32_t * pwr)
{
	uint16_t pwr_reg_val;
	if(ina226_readreg(ina226->address, INA226_PWR_REG, &pwr_reg_val) != OK) {
		return I2C_TRANSMISSION_ERROR;
	}
	// power_LSB = current_LSB*25
	*pwr= (float32_t) pwr_reg_val*ina226->current_LSB*25;
	return OK;
}

INA226_status ina226_set_avg(INA226 * ina226, INA226_avg avg)
{
	ina226->config.bits.AVG = avg;	
	return ina226_writereg(ina226->address, INA226_CONFIG_REG,
						   ina226->config.buffer.all);
}

INA226_status ina226_set_vbus_ct(INA226 * ina226, INA226_ct ct)
{
	ina226->config.bits.VBUSCT = ct;
	return ina226_writereg(ina226->address, INA226_CONFIG_REG,
						   ina226->config.buffer.all);
}

INA226_status ina226_set_vshunt_ct(INA226 * ina226, INA226_ct ct)
{
	ina226->config.bits.VSHCT = ct;
	return ina226_writereg(ina226->address, INA226_CONFIG_REG,
						   ina226->config.buffer.all);
}

INA226_status ina226_set_mode(INA226 * ina226, INA226_mode mode)
{
	ina226->config.bits.MODE = mode;
	return ina226_writereg(ina226->address, INA226_CONFIG_REG, 
						   ina226->config.buffer.all);
}

INA226_status ina226_set_calibration(INA226 * ina226, 
									 float32_t r_shunt,
									 float32_t max_expected_current)
{
	// Calculate new current_LSB
	ina226->current_LSB = calculate_current_lsb(max_expected_current);
	// Calculate calibration value
	uint16_t cal_data = calculate_cal_val(r_shunt, ina226->current_LSB);
	swap_16b(&cal_data);
	ina226->r_shunt = r_shunt;
	ina226->max_expected_current = max_expected_current;
	return ina226_writereg(ina226->address, INA226_CAL_REG, 
						   cal_data);
}

INA226_status ina226_set_mask_enable(INA226 * ina226,
									 INA226_mask_enable mask_enable)
{
	ina226->mask_enable = mask_enable;
	return ina226_writereg(ina226->address, INA226_MASK_EN_REG,
						   mask_enable);
}

INA226_status ina226_clear_flags(INA226 * ina226)
{
	uint16_t buffer;
	if(ina226_readreg(ina226->address, INA226_MASK_EN_REG, 
					  &buffer) != OK) {
		return FLAGS_NOT_CLEARED;
	}
	return OK;
}

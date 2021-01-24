#include "bq76pl536a.h"
#include "crc.h"

/**
 * @func calculate_ott
 * @brief calculates the ott_config value given the delay time in a uint16_t
 * 		  variable
 * @params[in] delay_time: uint16_t variable that holds the desired delay time
 * 			   in miliseconds
 * @return uint8_t ott value
 */
static uint8_t calculate_ott(uint16_t delay_time)
{
	delay_time = (delay_time > MAX_OTT_DELAY) ? MAX_OTT_DELAY : delay_time;
	return delay_time/OTT_LSB_VALUE;
}

/**
 * @func  calculate_ot
 * @brief calculates the ot_config temperature threshold value given the desired
 * 		  temperature threshold
 * @params[in] temperature_threshold: uint8_t variable that holds the desired
 * 			   temperature threshold value
 * @note the temperature has to be multiple of 5
 * @return uint8_t ot value
 */

static uint8_t calculate_ot(uint8_t temperature_threshold)
{
	if(temperature_threshold > MAX_CAL_TEMP){
		temperature_threshold = MAX_CAL_TEMP;
	}
	if(temperature_threshold < MIN_CAL_TEMP){
		temperature_threshold = MIN_CAL_TEMP;
	}
	return (1 + ((temperature_threshold - MIN_CAL_TEMP)/CAL_TEMP_LSB));
}

/**
 * @func  calculate_cuv
 * @brief calculates the cuv_config voltage threshold value given the desired
 * 		  floating undervoltage value
 * @params[in] undervoltage_value: float32_t variable that holds the desired
 * 			   undervoltage value detection
 * @return uint8_t cuv register value
 */
static uint8_t calculate_cuv(float32_t undervoltage_value)
{
	if(undervoltage_value < MIN_CUV_VALUE){
		undervoltage_value = MIN_CUV_VALUE;	
	} 
	if(undervoltage_value > MAX_CUV_VALUE){
		undervoltage_value = MAX_CUV_VALUE;
	}
	return (uint8_t)((undervoltage_value - MIN_CUV_VALUE)/CUV_LSB_VALUE);
}

/**
 * @func  calculate_cov
 * @brief calculates the cov_config voltage threshold value given the desired 
 * 		  floating overvoltage value
 * @params[in] overvoltage_value: float32_t variable that holds the desired
 * 			   overvoltage value detection
 * @return uint8_t cov register value
 */
static uint8_t calculate_cov(float32_t overvoltage_value)
{
	if(overvoltage_value < MIN_COV_VALUE) {
		overvoltage_value = MIN_COV_VALUE;
	}
	if(overvoltage_value > MAX_COV_VALUE) {
		overvoltage_value = MAX_COV_VALUE;
	}
	return (uint8_t)((overvoltage_value - MIN_COV_VALUE)/COV_LSB_VALUE);
}

/**
 * @func  calculate_covt
 * @brief calculates the covt_config delay time given the desired uint16_t 
 * 		  desired time delay
 * @params[in] delay_time: uint16_t variable that holds the desired
 * 			   delay_time
 * @return uint8_t covt register value
 */
static uint8_t calculate_delay(uint16_t delay_time)
{
	if(delay_time > MAX_DELAY_VALUE){
		return MAX_DELAY_VALUE/DELAY_LSB_VALUE;
	}
	return delay_time/DELAY_LSB_VALUE;
}

/**
 * @func init_write_packet
 * @brief It generates a BQ76_write_packet_format struct given the device 
 * 	  	  address, register address and register data
 * @params[in] device_address: uint8_t bq76 device address
 * @params[in] reg_address: uint8_t register address to write
 * @params[in] reg_data: uint8_t packet data to write
 * @return BQ76_write_packet_format
 */
static struct BQ76_write_packet_format init_write_packet(uint8_t device_address, 
														 uint8_t reg_address, 
														 uint8_t reg_data)
{
	struct BQ76_write_packet_format packet;
	packet.device_address = (device_address << 1) | 0x01;
	packet.reg_address = reg_address;
	packet.reg_data = reg_data;
    packet.crc = calculate_crc((uint8_t *) &packet, 
                               sizeof(struct BQ76_write_packet_format) - 1,
                               CRC_SMBUS_LUT);
	return packet;
}

/**
 * @func BQ76_read_packet_format
 * @brief It generates a BQ76_read_packet_format struct given the device 
 * 	  	  address, start register address and the length of the packet 
 * @params[in] device_address: uint8_t bq76 device address
 * @params[in] start_reg_address: uint8_t start register address to read
 * @params[in] read_length: uint8_t length of data to read
 * @return BQ76_read_packet_format
 */
static struct BQ76_read_packet_format init_read_packet(uint8_t device_address, 
													   uint8_t start_reg_addr, 
													   uint8_t read_length)
{
	struct BQ76_read_packet_format packet;
	packet.device_address = device_address << 1;
	packet.start_reg_address = start_reg_addr;
	packet.read_length = read_length;
	return packet;
}

/**
 * @func writespi
 * @brief Sends a write command from the SPI interface to the the BQ76PL536 
 * 		  device. The user of this library is responsible of defining this 
 * 		  function. As an example, we have only defined functionality for 
 * 		  STM32F407xx.
 * @params[in] spi_address: BQ76 device address
 * @params[in] reg_address: register address to write
 * @params[in] reg_data: data to write
 * @return BQ76_status [BQ76_OK|BQ76_SPI_TRANSMISSION_ERROR]
 */
static enum BQ76_status writespi(uint8_t spi_address, uint8_t reg_address, 
									  uint8_t reg_data)
{
	struct BQ76_write_packet_format packet = init_write_packet(spi_address, 
															   reg_address,
															   reg_data);	
#if defined(USE_HAL_DRIVER) && defined(STM32F407xx)
	HAL_GPIO_TogglePin(BQ76_CS_GPIO, BQ76_CS_PIN);
	if(HAL_SPI_Transmit(&BQ76_INTERFACE, (uint8_t *) &packet, 
					 	BQ76_TX_BUFF_SIZE, BQ76_TIMEOUT) != HAL_OK)
	{
		return BQ76_SPI_TRANSMISSION_ERROR;
	}
	HAL_GPIO_TogglePin(BQ76_CS_GPIO, BQ76_CS_PIN);
	return BQ76_OK;
#endif
}

/**
 * @func writereg
 * @brief Write a register of the BQ76PL536 regardless to which group belongs
 * 		  to, remember that according to the datasheet there are three kind of 
 * 		  groups (Group 1, Group 2 and Group 3) where the first are eead-only 
 * 		  groups, the second read/write groups, and group 3 are read/write 
 * 		  registers which they need a special write sequence 
 * 		  (first write 0x35 to SHDW_CONTROL, and then immediately write to the 
 * 		  desired register), any intervening write cancels the sequence
 * @params[in] spi_address: spi_address of the device
 * @params[in] reg_address: desired register to write
 * @params[in] reg_data: data to write
 */
static enum BQ76_status writereg(uint8_t spi_address, uint8_t reg_address,
						 		 uint8_t reg_data)
{
	enum BQ76_status write_status;
	if(reg_address >= FUNCTION_CONFIG_REG && reg_address <= OTT_CONFIG_REG){
		write_status = writespi(spi_address, SHADOW_CONTROL_REG, 
								SHDW_CONTROL_ENABLE);
	}
	write_status = writespi(spi_address, reg_address, reg_data);
	return write_status;
}

/**
 * @func readspi
 * @brief Write a register of the BQ76PL536 device.
 * 		  The user of this library is responsible of defining this function. As
 * 		  an example, we have only defined functionality for STM32F407xx.
 * 		  Make sure that data has the size of read_length, otherwise it would be
 * 		  unsafe because the SPI receive function will overwrite over the
 * 		  overflowed address
 * @params[in] spi_address: BQ76 device address
 * @params[in] reg: register address to write
 * @params[in] value: value to write
 * @return BQ76_status [BQ76_OK|BQ76_SPI_TRANSMISSION_ERROR]
 */
static enum BQ76_status readspi(uint8_t spi_address, uint8_t reg_address,
								uint8_t read_length, uint8_t * data)
{
#if defined(USE_HAL_DRIVER) && defined(STM32F407xx)
	struct BQ76_read_packet_format packet = init_read_packet(spi_address,
															 reg_address,
															 read_length);
	HAL_GPIO_TogglePin(BQ76_CS_GPIO, BQ76_CS_PIN);
	if(HAL_SPI_Transmit(&BQ76_INTERFACE, (uint8_t *) &packet, 
						BQ76_TX_BUFF_SIZE - 1, BQ76_TIMEOUT) != HAL_OK){
		return BQ76_SPI_TRANSMISSION_ERROR;
	}
	if(HAL_SPI_Receive(&BQ76_INTERFACE, packet.buffer, read_length + 1, 
				   BQ76_TIMEOUT) != HAL_OK){
		return BQ76_SPI_TRANSMISSION_ERROR;
	}
    HAL_GPIO_TogglePin(BQ76_CS_GPIO, BQ76_CS_PIN);
    if(calculate_crc((uint8_t *) &packet, BQ76_RX_BUFF_LENGTH(read_length) - 1, 
                     CRC_SMBUS_LUT) != packet.buffer[read_length]){
        return BQ76_CRC_MISMATCH;
    }
    memcpy(data, packet.buffer, read_length);
	return BQ76_OK;
#endif
}

enum BQ76_status bq76_init(struct BQ76 * device, uint8_t spi_address,
						   uint8_t balancing_time, float32_t cov_threshold, 
						   uint8_t covt_delay, float32_t cuv_threshold, 
						   uint8_t cuvt_delay, float32_t temp1_threshold, 
						   float32_t temp2_threshold, uint16_t temp_delay)
{
	// Send broadcast reset
	if(bq76_broadcast_reset() != BQ76_OK){
		return BQ76_BROADCAST_RESET_FAIL;
	}

	// set address to device
	if(bq76_set_address(device, spi_address) != BQ76_OK){
		return BQ76_ADDRESS_CONFIG_FAIL;
	}

	// config adc control register
	if(bq76_set_adc_control(device) != BQ76_OK){
		return BQ76_ADC_CONFIG_FAIL;
	}
	
	// config balancing time outputs
	if(bq76_set_cb_time(device, balancing_time) != BQ76_OK){
		return BQ76_CB_TIME_CONFIG_FAIL;
	}

	// set function configuration
	if(bq76_set_function_config(device) != BQ76_OK){
		return BQ76_FUNCTION_CONFIG_FAIL;
	}

	// set I/O configuration
	if(bq76_set_io_config(device) != BQ76_OK){
		return BQ76_IO_CONFIG_FAIL;
	}

	// set cov config
	if(bq76_set_cov_config(device, cov_threshold) != BQ76_OK){
		return BQ76_COV_CONFIG_FAIL;
	}

	// set covt config
	if(bq76_set_covt_config(device, covt_delay) != BQ76_OK){
		return BQ76_COVT_CONFIG_FAIL;
	}

	// set cuv config
	if(bq76_set_cuv_config(device, cuv_threshold) != BQ76_OK){
		return BQ76_CUV_CONFIG_FAIL;
	}

	// set cuvt config
	if(bq76_set_cuvt_config(device, cuvt_delay) != BQ76_OK){
		return BQ76_CUVT_CONFIG_FAIL;
	}

	// set ot config
	if(bq76_set_ot_config(device, temp1_threshold, temp2_threshold) != BQ76_OK){
		return BQ76_OT_CONFIG_FAIL;
	}

	// set ott config
	if(bq76_set_ott_config(device, temp_delay) != BQ76_OK){
		return BQ76_OTT_CONFIG_FAIL;
	}
	return BQ76_OK;
}

enum BQ76_status bq76_broadcast_reset()
{
	if(writereg(BROADCAST_ADDRESS, RESET_REG, RESET_DEVICE_VALUE) != BQ76_OK){
		return BQ76_SPI_TRANSMISSION_ERROR;
	}
	return BQ76_OK;
}

enum BQ76_status bq76_reset(struct BQ76 * device)
{
	if(writereg((uint8_t) device->address_control.ADDR, RESET_REG, 
				RESET_DEVICE_VALUE) != BQ76_OK){
		return BQ76_SPI_TRANSMISSION_ERROR;
	}
	return BQ76_OK;
}

enum BQ76_status bq76_set_address(struct BQ76 * device, uint8_t address)
{
	// Write new address to device
	// Here we are assuming that the device is previously reset, if you want to
	// modify the address call bq76_change_address
	if(writereg(0x00, ADDRESS_CONTROL_REG, 
				address) != BQ76_OK){
		return BQ76_SPI_TRANSMISSION_ERROR;
	}
    
    uint8_t address_buffer;
	// 1 - read new address
	if(readspi(address, ADDRESS_CONTROL_REG, 1, 
			   &address_buffer) != BQ76_OK){
		return BQ76_SPI_TRANSMISSION_ERROR;
	}
    device->address_control.ADDR_RQST = address_buffer >> 7;
    device->address_control.ADDR = address_buffer & 0x01F;
	return BQ76_OK;
}

enum BQ76_status bq76_set_adc_control(struct BQ76 * device)
{
	uint8_t adc_control_data = (device->adc_control.CELL_SEL << 6)	| 
				   			   (device->adc_control.GPAI << 5)		| 
				   			   (device->adc_control.TS << 4)		| 
				   			   (device->adc_control.GPAI << 3)		| 
				   			   device->adc_control.CELL_SEL;
    // write address control 
	if(writereg((uint8_t) device->address_control.ADDR, ADC_CONTROL_REG, 
				adc_control_data) != BQ76_OK){
		return BQ76_SPI_TRANSMISSION_ERROR;
	}


    // validate writteng adc value
    uint8_t adc_control_data_buffer;
    if(readspi((uint8_t) device->address_control.ADDR, ADC_CONTROL_REG, 1, 
               &adc_control_data_buffer) != BQ76_OK){
        return BQ76_SPI_TRANSMISSION_ERROR;
    }
    
	return BQ76_OK;
}

enum BQ76_status bq76_set_cb_time(struct BQ76 * device, uint8_t balancing_time)
{
	device->cb_time.CBT = (balancing_time > 63) ? 63 : balancing_time;
	uint8_t cb_time_data = device->cb_time.CBCT << 7 | device->cb_time.CBT;

	if(writereg((uint8_t) device->address_control.ADDR, CB_TIME_REG, 
				cb_time_data) != BQ76_OK){
		return BQ76_SPI_TRANSMISSION_ERROR;
	}
    
	return BQ76_OK;
}

enum BQ76_status bq76_set_function_config(struct BQ76 * device)
{
	uint8_t function_config_data =	(device->function_config.GPAI_REF << 5) | 
				 				    (device->function_config.GPAI_SRC << 4) |
									(device->function_config.CN << 2);
	if(writereg((uint8_t) device->address_control.ADDR, FUNCTION_CONFIG_REG,
				(uint8_t) function_config_data) != BQ76_OK){
		return BQ76_SPI_TRANSMISSION_ERROR;
	}
	return BQ76_OK;
}

enum BQ76_status bq76_set_io_config(struct BQ76 * device)
{
	uint8_t io_config_data = (device->io_config.CRCNOFLT << 7) | 
				   			 (device->io_config.CRC_DIS);
	if(writereg((uint8_t) device->address_control.ADDR, IO_CONFIG_REG,
				io_config_data) != BQ76_OK){
		return BQ76_SPI_TRANSMISSION_ERROR;
	}
	return BQ76_OK;
}

enum BQ76_status bq76_set_cov_config(struct BQ76 * device, 
									 float32_t voltage_threshold)
{
	device->cov_config.VTH = calculate_cov(voltage_threshold);
	uint8_t cov_config_data = (device->cov_config.DISABLE << 7) | 
							  (device->cov_config.VTH);
	if(writereg((uint8_t) device->address_control.ADDR, COV_CONFIG_REG, 
				cov_config_data) != BQ76_OK){
		return BQ76_SPI_TRANSMISSION_ERROR;
	}
	return BQ76_OK;
}

enum BQ76_status bq76_set_cuv_config(struct BQ76 * device, 
									 float32_t voltage_threshold)
{
	device->cov_config.VTH = calculate_cuv(voltage_threshold);
	uint8_t cov_config_data = (device->cov_config.DISABLE << 7) | 
							  device->cov_config.VTH;
	if(writereg((uint8_t) device->address_control.ADDR, CUV_CONFIG_REG,
			    cov_config_data) != BQ76_OK){
		return BQ76_SPI_TRANSMISSION_ERROR;
	}
	return BQ76_OK;
}

enum BQ76_status bq76_set_covt_config(struct BQ76 * device,	uint16_t delay)
{
	device->covt_config.DELAY = calculate_delay(delay);
	uint8_t covt_config_data = (device->covt_config.US_MS << 7) |
							   device->covt_config.DELAY;
	if(writereg((uint8_t) device->address_control.ADDR, COVT_CONFIG_REG, 
				covt_config_data) != BQ76_OK){
		return BQ76_SPI_TRANSMISSION_ERROR;
	}
	return BQ76_OK;
}

enum BQ76_status bq76_set_cuvt_config(struct BQ76 * device, uint16_t delay)
{
	device->cuvt_config.DELAY = calculate_delay(delay);
	uint8_t cuvt_config_data = (device->cuvt_config.US_MS << 7) |
							   (device->cuvt_config.DELAY);
	if(writereg((uint8_t) device->address_control.ADDR, CUVT_CONFIG_REG, 
				cuvt_config_data) != BQ76_OK){
		return BQ76_SPI_TRANSMISSION_ERROR;
	}
	return BQ76_OK;
}

enum BQ76_status bq76_set_ot_config(struct BQ76 * device, 
									float32_t ot1_threshold, 
									float32_t ot2_threshold)
{ 
	device->ot_config.OT1 = calculate_ot(ot1_threshold);
	device->ot_config.OT2 = calculate_ot(ot2_threshold);
	uint8_t ot_config_data = (device->ot_config.OT2 << 4) |
							 (device->ot_config.OT1);
	if(writereg((uint8_t) device->address_control.ADDR, OT_CONFIG_REG,
			   ot_config_data) != BQ76_OK){
		return BQ76_SPI_TRANSMISSION_ERROR;
	}
	return BQ76_OK;
}

enum BQ76_status bq76_set_ott_config(struct BQ76 * device,
									 uint16_t delay_value)
{
	// the LSB of ott_config representes 10mS
	device->ott_config = calculate_ott(delay_value);
	if(writereg((uint8_t) device->address_control.ADDR, OTT_CONFIG_REG,
				device->ott_config) != BQ76_OK){
		return BQ76_SPI_TRANSMISSION_ERROR;
	}
	return BQ76_OK;
}

enum BQ76_status bq76_read_n_cells(struct BQ76 * device)
{
    // the amount of cells to be read from a device is automatically set by the
    // adc_control register
    uint8_t n_cells = device->adc_control.CELL_SEL + 1;
    uint16_t raw_cell_voltage[n_cells];
    if(readspi((uint8_t) device->address_control.ADDR, VCELL1_LOW_REG, 
               2*n_cells, (uint8_t *) &raw_cell_voltage) != BQ76_OK){
        return BQ76_SPI_TRANSMISSION_ERROR;
    }

    for(int i = 0; int < n_cells; ++i){
        v_cells[i] = raw_cell_voltage[i]*6250/16383;
    }
    
    return BQ76_OK;
}

enum BQ76_status bq76_swrqst_adc_convert(struct BQ76 * device)
{
    if(writereg((uint8_t) device->address_control.ADDR, ADC_CONVERT_REG, 
                0x01) != BQ76_OK){
        return BQ76_SPI_TRANSMISSION_ERROR;
    }
    return BQ76_OK;
}

enum BQ76_status bq76_brdcst_adc_convert()
{
    if(writereg(BROADCAST_ADDRESS, ADC_CONVERT_REG, 
                0x01) != BQ76_OK){
        return BQ76_SPI_TRANSMISSION_ERROR;
    }
    return BQ76_OK;
}

#include "bq76pl536a.h"

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
		return 0x00;
	}
	if(overvoltage_value > MAX_COV_VALUE) {
		return 0x3C;
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
static uint8_t calculate_covt(uint16_t delay_time)
{
	if(delay_time > MAX_COVT_VALUE){
		return MAX_COVT_VALUE/COVT_LSB_VALUE;
	}
	return delay_time/COVT_LSB_VALUE;
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
	packet.device_address = device_address;
	packet.reg_address = reg_address;
	packet.reg_data = reg_data;
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
	packet.device_address = device_address;
	packet.start_reg_address = start_reg_addr;
	packet.read_length = read_length;
	return packet;
}

/**
 * @func writereg
 * @brief Write a register of the BQ76PL536 device.
 * 		  The user of this library is responsible of defining this function. As
 * 		  an example, we have only defined functionality for STM32F407xx
 * @params[in] spi_address: BQ76 device address
 * @params[in] reg: register address to write
 * @params[in] value: value to write
 * @return BQ76_status [OK|SPI_TRANSMISSION_ERROR]
 */
static enum BQ76_status writereg(uint8_t spi_address, uint8_t reg_address, 
									  uint8_t reg_data)
{
	struct BQ76_write_packet_format packet = init_write_packet(spi_address, 
															   reg_address,
															   reg_data);	
#if defined(USE_HAL_DRIVER) && defined(STM32F407xx)
	if(HAL_SPI_Transmit(&BQ76_INTERFACE, (uint8_t *) &packet, 
					 BQ76_TX_BUFF_SIZE, BQ76_TIMEOUT) != HAL_OK)
	{
		return SPI_TRANSMISSION_ERROR;
	}
	return OK;
#endif
}

/**
 * @func readreg
 * @brief Write a register of the BQ76PL536 device.
 * 		  The user of this library is responsible of defining this function. As
 * 		  an example, we have only defined functionality for STM32F407xx.
 * 		  Make sure that data has the size of read_length, otherwise it would be
 * 		  unsafe because the SPI receive function will overwrite over the
 * 		  overflowed address
 * @params[in] spi_address: BQ76 device address
 * @params[in] reg: register address to write
 * @params[in] value: value to write
 * @return BQ76_status [OK|SPI_TRANSMISSION_ERROR]
 */
static enum BQ76_status readreg(uint8_t spi_address, uint8_t reg_address,
								uint8_t read_length, uint8_t * data)
{
#if defined(USE_HAL_DRIVER) && defined(STM32F407xx)
	struct BQ76_read_packet_format packet = init_read_packet(spi_address,
															 reg_address,
															 read_length);
	if(HAL_SPI_Transmit(&BQ76_INTERFACE, (uint8_t *) &packet, BQ76_TX_BUFF_SIZE,
						BQ76_TIMEOUT) != HAL_OK){
		return SPI_TRANSMISSION_ERROR;
	}
	if(HAL_SPI_Receive(&BQ76_INTERFACE, data, read_length, 
					   BQ76_TIMEOUT) != HAL_OK){
		return SPI_TRANSMISSION_ERROR:
	}
	return OK;
#endif
}

enum BQ76_status bq76_init(struct BQ76 * device, uint8_t new_spi_address,
						   enum bat_series_inputs num_bat_series,
						   enum temp_sensor_inputs ts, uint8_t gpai,
						   uint8_t balancing_time_unit, uint8_t balancing_time,
						   uint8_t gpai_ref, uint8_t gpai_src,
						   enum series_cells series_cells,
						   uint8_t crc_enable, uint8_t crc_assert_pin,
						   uint8_t cov_disable, uint8_t cov_threshold,
						   )
{
	// Send broadcast reset
	if(broadcast_reset() != OK){
		return BROADCAST_RESET_FAIL;
	}

	// set address to device
	if(bq76_set_address(device, new_spi_address) != OK){
		return ADDRESS_CONFIG_FAIL;
	}

	// config adc control register
	struct adc_control adc_buffer;
	adc_buffer.CELL_SEL = num_bat_series;
	adc_buffer.GPAI = gpai;
	adc_buffer.TS = ts;
	if(bq76_set_adc_control(device, adc_buffer) != OK){
		return ADC_CONFIG_FAIL;
	}
	
	// config balancing time outputs
	if(bq76_set_cb_time(device, balancing_time_unit, balancing_time) != OK){
		return CB_TIME_CONFIG_FAIL;
	}

	// set function configuration
	if(bq76_set_cb_time(device, gpai_ref, gpai_src, series_cells) != OK){
		return FUNCTION_CONFIG_FAIL;
	}

	// set I/O configuration
	if(bq76_set_io_config(device, crc_enable, crc_assert_pin) != OK){
		return IO_CONFIG_FAIL;
	}

	// set cov config
	if(bq76_set_cov_config(device, cov_disable, voltage_threshold) != OK){
		return COV_CONFIG_FAIL;
	}
}

enum BQ76_status bq76_broadcast_reset()
{
	if(writereg(BROADCAST_ADDRESS, RESET_REG, RESET_DEVICE_VALUE) != OK){
		return SPI_TRANSMISSION_ERROR;
	}
	return OK;
}

enum BQ76_status bq76_reset(struct BQ76 * device)
{
	if(writereg((uint8_t) device->address_control.ADDR, RESET_REG, 
				RESET_DEVICE_VALUE) != OK){
		return SPI_TRANSMISSION_ERROR;
	}
	return OK;
}

enum BQ76_status bq76_set_address(struct BQ76 * device, uint8_t address)
{
	// Write new address to device
	if(writreg(device->address, ADDRESS_CONTROL_REG, address) != OK){
		return SPI_TRANSMISSION_ERROR;
	}
	// Check if the address has been correctly set
	struct address_control address_buffer;
	// 1 - read new address
	if(readreg(new_address, ADDRESS_CONTROL_REG, 1, &address_buffer) != OK){
		return SPI_TRANSMISSION_ERROR;
	}
	// 2 - check if the ADDR_RQST is not set or the ADDR[n] is different from 
	// the new address
	if(!address_buffer.ADDR_RQST || 
			(uint8_t) address_buffer.ADDR != new_address){
		return ADDRESS_CONFIG_FAIL;
	}
	return OK;
}

enum BQ76_status bq76_set_adc_control(struct BQ76 * device, 
									  struct adc_control adc)
{
	if(writereg((uint8_t) device->address_control.ADDR, ADC_CONTROL_REG, 
				(uint8_t) adc) != OK){
		return SPI_TRANSMISSSION_ERROR;
	}
	// update the BQ76 device_status local reg
	device->adc_control = adc;
	return OK;
}

enum BQ76_status bq76_set_cb_time(struct BQ76 * device,
								  uint8_t mins_secs, uint8_t balancing_time)
{
	struct cb_time;
	cb_time.CBCT = mins_secs;
	cb_time.CBT = (balancing_time > 63) ? 63 : balancing_time;
	if(writereg((uint8_t) device.address_control.ADDR, CB_TIME_REG, 
				(uint8_t) cb_time) != OK){
		return SPI_TRANSMISSION_ERROR;
	}
	device->cb_time = cb_time;
	return OK;
}

enum BQ76_status bq76_set_function_config(struct BQ76 * device,
										  uint8_t gpai_ref, uint8_t gpai_src,
										  enum series_cells series_cells)
{
	struct function_config function_config_buffer;
	function_config_buffer.CN = series_cells;
	function_config_buffer.GPAI_SRC = gpai_src;
	function_config_buffer.GPAI_REF = gpai_ref;
	if(writereg((uint8_t) device.address_control.ADDR, FUNCTION_CONFIG_REG,
				(uint8_t) function_config_buffer) != OK){
		return SPI_TRANSMISSION_ERROR;
	}
	device->function_config = function_config_buffer;
	return OK;
}

enum BQ76_status bq76_set_io_config(struct BQ76 * device, uint8_t crc_enable,
									uint8_t crc_assert_pin)
{
	struct io_config io_config_buffer;
	io_config_buffer.CRC_DIS = crc_enable;
	io_config_buffer.CRCNOFLT = crc_assert_pin;
	if(writereg((uint8_t) device.address_control.ADDR, IO_CONFIG_REG,
				(uint8_t) io_config_buffer) != OK){
		return SPI_TRANSMISSION_ERROR;
	}
	device->io_config = io_config_buffer;
	return OK;
}

enum BQ76_status bq76_set_cov_config(struct BQ76 * device, uint8_t disable,
									 float32_t voltage_threshold)
{
	struct cov_config_buffer;
	cov_config_buffer.COV = calculate_cov(voltage_threshold);
	cov_config_buffer.DISABLE = disable;
	if(writereg((uint8_t) device.address_control.ADDR, COV_CONFIG_REG, 
				(uint8_t) cov_config_buffer) != OK){
		return SPI_TRANSMISSION_ERROR;
	}
	device->cov_config = cov_config_buffer;
	return OK;
}

enum BQ76_status bq76_set_covt_config(struct BQ76 * device, uint8_t time_unit,
									  uint16_t delay)
{
	struct covt_config covt_config_buffer;
	covt_config_buffer.US_MS = time_unit;
	covt_config_buffer.COVD = calculate_cov(delay);
	if(writreg((uint8_t) device.address_control.ADDR, COVT_CONFIG_REG, 
				(uint8_t) covt_config_buffer) != OK){ 
		return SPI_TRANSMISSION_ERROR;
	}
	device->covt_config = covt_config_buffer;
	return OK;
}

enum 

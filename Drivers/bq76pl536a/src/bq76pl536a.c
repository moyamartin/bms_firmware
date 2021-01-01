#include "bq76pl536a.h"

/**
 * @func BQ76_write_packet_format
 * @brief It generates a BQ76_write_packet_format struct given the device 
 * 	  	  address, register address and register data
 * @params[in] device_address: uint8_t bq76 device address
 * @params[in] reg_address: uint8_t register address to write
 * @params[in] reg_data: uint8_t packet data to write
 * @return BQ76_write_packet_format
 */
static struct BQ76_write_packet_format init_write_packet(device_address, 
														 reg_address, reg_data)
{
	struct BQ76_write_packet_format packet;
	packet.device_address = device_address;
	packet.reg_address = reg_address;
	packet.reg_data = reg_data;
	return packet;
}

/**
 * @func BQ76_writereg
 * @brief Write a register of the BQ76PL536 device.
 * 		  The user of this library is responsible of defining this function. As
 * 		  an example, we have only defined functionality for STM32F407xx
 * @params[in] spi_address: BQ76 device address
 * @params[in] reg: register address to write
 * @params[in] value: value to write
 * @return BQ76_status
 */
static enum BQ76_status BQ76_writereg(uint8_t spi_address, uint8_t reg_address, 
									  uint8_t reg_data)
{
	struct BQ76_write_packet_format = init_write_packet(spi_address, 
														reg_address,
														reg_data);	
#if defined(USE_HAL_DRIVER) && defined(STM32F407xx)
	HAL_SPI_Transmit(&BQ76_INTERFACE, )
#endif
}



static enum BQ76_status BQ76_readreg(uint8_t spi_address, uint8_t reg_address,
									 uint8_t * value)
{

}

#include "bq76pl536a.h"

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
	if(HAL_SPI_Receive(&BQ76_INTERFACE, rx_packet, read_length, 
				BQ76_TIMEOUT) != HAL_OK){
		return SPI_TRANSMISSION_ERROR:
	}
	return OK;
}

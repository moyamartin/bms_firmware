
/******************************************************************************
 *
 * @file	bq76pl536a.h
 * @brief 	Texas Instruments BQ76PL536A high level data structures definitions
 * @version	v1.00f00
 * @date	27, Dec. 2020
 * @author	CECARELLI, Federico (fededc88@gmail.com),
 * 			MOYA, Martin		(moyamartin1@gmail.com),
 * 			SANTOS, Lucio		(lusho2206@gmail.com)
 * @copyright GPL license, all text here must be included in any redistribution
 *****************************************************************************/

#include "bq76pl536a_defs.h"

#if defined(USE_HAL_DRIVER) && defined(STM32F407xx)
	// Support for stm32f4xx devices
	#include "arm_math.h"
	#include "stm32f4xx_hal.h"

	/* 
	 * All the instances of these drivers share only one SPI bus, that we asume
	 * that it is already initialized on the main before the infinite loop. 
	 * This design takes into account that the project has been created with 
	 * the STM32CubeMX which creates the function needed to initialize the i2c
	 * bus.
	 *
	 * We also asume that the SPI bus used is the hspi1
	 */
	extern I2C_HandleTypeDef 	hspi1;
	#define BQ76_INTERFACE hspi1 
	#define BQ76_TIMEOUT 1000
	#define BQ76_TX_BUFF_SIZE sizeof(BQ76_write_packet_format)
	#define BQ76_RX_BUFF_SIZE 12
	#define BQ76_RX_BUFF_LENGTH(length) (sizeof(BQ76_write_packet_format) + length)
#elif defined (__AVR__)
	// AVR devices support
#endif

enum BQ76_Status {
	OK = 0,
	SPI_TRANSMISSION_ERROR = -1,
	BROADCAST_RESET_FAIL = -2,
	DEVICE_RESET_FAIL = -3,
	ADDRESS_CONFIG_FAIL = -4
};

struct BQ76_write_packet_format {
	uint8_t device_address;
	uint8_t reg_address;
	uint8_t reg_data;
}

struct BQ76_read_packet_format {
	uint8_t device_address;
	uint8_t start_reg_address;
	uint8_t read_length;
}

struct BQ76 {
	struct address_control address_control;
	struct device_status device_status;
	struct alert_status;
	struct cov_fault;
	struct cuv_fault;
	struct presult_a;
	struct presult_b;
	struct adc_control;
	struct io_control;
	struct cb_ctrl;
	struct cb_time;
	struct adc_convert;
	struct function_config;
	struct io_config;
	struct cov_config;
	struct covt_config;
	struct cuv_config;
	struct cuvt_config;
	uint8_t shadow_control; /**< to enable group3 regs write 0x35 to it*/
	uint8_t * host;
	struct BQ76 * south;
	struct BQ76 * north;
};

/**
 * @func bq76_init
 * @brief Initializes a bq76 struct
 * @params[in] device: pointer that points to a BQ76 struct
 * @return BQ76_status [OK|SPI_TRANSMISSION_ERROR]
 */
enum BQ76_status bq76_init(struct BQ76 * device);

/**
 * @func bq76_broadcast_reset
 * @brief Sends a reset broadcast to all connected devices with a valid address
 * @returns bq76_status [OK|SPI_TRANSMISSION_ERROR]
 */
enum BQ76_status bq76_broadcast_reset();

/**
 * @func bq76_reset
 * @brief sends a reset command to a specified device
 * @params[in] device: BQ76 pointer referencing to the desired device to be
 * 			   resetted
 * @return BQ76_status [OK|SPI_TRANSMISSION_ERROR]
 */
enum BQ76_status bq76_reset(struct BQ76 * device);

/**
 * @func bq76_set_address
 * @brief sets a device's address and check if it was properly set
 * @params[in] device: BQ76 pointer referencing to the desired device to be
 * 			   resetted
 * @params[in] new_address: uint8_t variable that holds the new address of the
 * 			   device
 * @return BQ76_status [OK|ADDRESS_CONFIG_FAIL|SPI_TRANSMISSION_ERROR]
 */
enum BQ76_status bq76_set_address(struct BQ76 * device);


#endif 

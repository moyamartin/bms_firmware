
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
	ADDRESS_CONFIG_FAIL = -4,
	ADC_CONFIG_FAIL = -5,
	CB_TIME_CONFIG_FAIL = -6,
	FUNCTION_CONFIG_FAIL = -7,
	IO_CONFIG_FAIL = -8,
	COV_CONFIG_FAIL = -9,
	COVT_CONFIG_FAIL = -10,
	CUV_CONFIG_FAIL = -11,
	CUVT_CONFIG_FAIL = -12,
	OT_CONFIG_FAIL = -13,
	OTT_CONFIG_FAIL = -14,
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
	struct adc_control adc_control;
	struct adc_convert adc_convert;
	struct address_control address_control;
	struct alert_status alert_status;
	struct cb_ctrl cb_ctrl;
	struct cb_time cb_time;
	struct cov_fault cov_fault;
	struct cuv_fault cuv_fault;
	struct delay_config covt_config;
	struct delay_config cuvt_config;
	struct device_status device_status;
	struct function_config function_config;
	struct io_config io_config;
	struct io_control io_control;
	struct ot_config ot_config;
	struct presult_a presult_a;
	struct presult_b presult_b;
	struct vth_config cov_config;
	struct vth_config cuv_config;
	uint8_t ott_config ott_config;
	uint8_t shadow_control shadow_control; /**< to enable group3 regs write 0x35 to it*/
	uint8_t user_register user_register[4];
};

/**
 * @func bq76_init
 * @brief Initializes a bq76 struct
 * @params[in] device: BQ76 pointer referencing to the desired device to be
 * 					   initialized 
 * @return BQ76_status [OK|SPI_TRANSMISSION_ERROR]
 */
enum BQ76_status bq76_init(struct BQ76 * device, uint8_t new_spi_address,
						   enum bat_series_inputs num_bat_series,
						   enum temp_sensor_inputs ts, uint8_t gpai,
						   uint8_t balancing_time_unit, uint8_t balancing_time,
						   uint8_t gpai_ref, uint8_t gpai_src,
						   enum series_cells series_cells,
						   uint8_t crc_enable, uint8_t crc_assert_pin,
						   uint8_t cov_disable, uint8_t cov_threshold,
						   uint8_t covt_time_unit, uint8_t covt_delay,
						   uint8_t cuv_disable, uint8_t cuv_threshold,
						   uint8_t temp1_threshold, uint8_t temp2_threshold,
						   uint16_t temp_delay);

/**
 * @func bq76_init
 * @brief Initializes a bq76 struct with default settings
 * @params[in] device: BQ76 pointer referencing to the desired device to be
 * 					   initialized 
 * @return BQ76_status [OK|SPI_TRANSMISSION_ERROR]
 */
enum BQ76_status bq76_init_default(struct BQ76 * device, 
								   uint8_t new_spi_address)

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
 * 					   modified
 * @return BQ76_status [OK|SPI_TRANSMISSION_ERROR]
 */
enum BQ76_status bq76_reset(struct BQ76 * device);

/**
 * @func bq76_set_address
 * @brief sets a device's address and check if it was properly set
 * @params[in] device: BQ76 pointer referencing to the desired device to be
 * 					   modified
 * @params[in] new_address: uint8_t variable that holds the new address of the
 * 			   device
 * @return BQ76_status [OK|ADDRESS_CONFIG_FAIL|SPI_TRANSMISSION_ERROR]
 */
enum BQ76_status bq76_set_address(struct BQ76 * device);

/**
 * @func  bq76_set_cb_time
 * @brief sets a device's balancing timeout
 * @params[in] device: BQ76 pointer referencing to the desired device to be
 * 					   modified
 * @params[in] mins_secs: uint8_t variable that holds a flag indicating if the
 * 			   time unit are minutes (true) or seconds (false)
 * @params[in] balancing_time: uint8_t variable holding the number of
 * 			   seconds/minutes that the balalncers should be on
 * @return BQ76_status [OK|SPI_TRANSMISSION_ERROR]
 */
enum BQ76_status bq76_set_cb_time(struct BQ76 * device,
								  uint8_t mins_secs, uint8_t balancing_time);


/**
 * @func  bq76_set_function_config
 * @brief sets a device's function config register
 * @params[in] device: BQ76 pointer referencing to the desired device to be
 * 					   modified
 * @params[in] gpai_ref: uint8_t variable that holds a flag indicating if the
 * 			   GPAI reference is connected to the ADC bandgap or to V_REG50
 * @params[in] gpai_src: uint8_t variable that holds a flag indicating whether
 * 			   if the GPAI is connected to external GPAI inputos or to BAT1 pin
 * @params[in] series_cells: uint8_t variable that indicates the number of
 * 			   series cells used
 * @return BQ76_status [OK|SPI_TRANSMISSION_ERROR]
 */
enum BQ76_status bq76_set_function_config(struct BQ76 * device,
										  uint8_t gpai_ref, uint8_t gpai_src,
										  enum series_cells series_cells);

/**
 * @func  bq76_set_io_config
 * @brief sets a device's I/O config register
 * @params[in] device: BQ76 pointer referencing to the desired device to be
 * 					   modified
 * @params[in] crc_enable: uint8_t variable that enables or disables CRC
 * 			   calculation (0: ENABLED, 1: DISABLED)
 * @params[in] crc_assert_pin: uint8_t variable that enables and disabled
 * 			   detected CRC errors asserting the FAULT pin 
 * 			   (0: ENABLED, 1: DISABLED)
 * @return BQ76_status [OK|SPI_TRANSMISSION_ERROR]
 */
enum BQ76_status bq76_set_io_config(struct BQ76 * device,
									uint8_t crc_enable, uint8_t crc_assert_pin);

/**
 * @func  bq76_set_cov_config
 * @brief sets a device's Cell Overvoltage Threshold value
 * @params[in] device: BQ76 pointer referencing to the desired device to be
 * 			   resetted
 * @params[in] disable: uint8_t variable that disables the cov function
 * @params[in] voltage_threshold: float32_t that holds the voltage_threshold
 * 			   value
 * @return BQ76_status [OK|SPI_TRANSMISSION_ERROR]
 */
enum BQ76_status bq76_set_cov_config(struct BQ76 * device, uint8_t disable, 
									 float32_t voltage_threshold);

/**
 * @func  bq76_set_cuv_config
 * @brief sets a device's Cell undervoltage threshold value
 * @params[in] device: BQ76 pointer referencing to the desired device to be
 * 					   modified
 */
enum BQ76_status bq76_set_cuv_config(struct BQ76 * device, uint8_t disable,
									 float32_t voltage_threshold);

/**
 * @func  bq76_set_covt_config
 * @brief sets a device's Cell Undervoltage delay time
 * @params[in] device: BQ76 pointer referencing to the desired device to be
 * 			   modified
 * @params[in] time_unit: uint8_t variable that determines the units of the
 * 			   delay time, microseconds or miliseconds 
 * 			   (0: microseconds, 1: miliseconds)
 * @params[in] delay: uint16_t variable that holds the 
 * 					  delay time
 * @return BQ76_status [OK|SPI_TRANSMISSION_ERROR]
 */
enum BQ76_status bq76_set_covt_config(struct BQ76 * device, uint8_t time_unit, 
									  uint16_t delay); 

/**
 * @func  bq76_set_cuvt_config
 * @brief sets a device's Cell Undervoltage delay time
 * @params[in] device: BQ76 pointer referencing to the desired device to be
 * 			   modified
 * @params[in] time_unit: uint8_t variable that determines the units of the
 * 			   delay time, microseconds or miliseconds 
 * 			   (0: microseconds, 1: miliseconds)
 * @params[in] delay: uint16_t variable that holds the 
 * 					  delay time
 * @return BQ76_status [OK|SPI_TRANSMISSION_ERROR]
 */
enum BQ76_status bq76_set_cuvt_config(struct BQ76 * device, uint8_t time_unit, 
									  uint16_t delay); 

/**
 * @func  bq76_set_ot_config
 * @brief sets a device's temperature sensor threshold value for sensors 1 and 2
 * @params[in] device: BQ76 pointer referencing to the desired device to be
 * 			   modified
 * @params[in] ot1: uint8_t value that sets the temp sensor 1 threshold value
 * @params[in] ot2:
 */
enum BQ76_status bq76_set_ot_config(struct BQ76 * device, uint8_t ot1, 
									uint8_t ot2);

/**
 * @func  bq76_set_ott_config
 * @brief sets a device's temperature sensor delay time detection for temp.
 * 		  sensors 1 and 2
 * @params[in] device: BQ776 pointer referencing to the desired device to be
 * 					   modified
 * @params[in] 
 */
enum BQ76_status bq76_set_ott_config(struct BQ76 * device, uint8_t delay_time);

#endif

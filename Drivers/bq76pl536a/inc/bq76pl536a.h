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
#elif defined (__AVR__)
	// AVR devices support
#endif

#define RX_BUFFER_SIZE	12

enum BQ76_Status {
	OK = 0,
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
	uint8_t read_data[12];
}

struct BQ76 {
	struct device_status;
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
	struct address_control;
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

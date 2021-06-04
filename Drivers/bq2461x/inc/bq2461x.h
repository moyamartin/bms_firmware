/******************************************************************************
 *
 * @file	bq2461x.h
 * @brief 	Texas Instruments BQ2461x declarations of function and high
 * 			level structures
 * @version	v1.00000
 * @date	17, may. 2021
 * @author	CECARELLI, Federico (fededc88@gmail.com),
 * 			MOYA, Martin		(moyamartin1@gmail.com),
 * 			SANTOS, Lucio		(lusho2206@gmail.com)
 * @copyright GPL license, all text here must be included in any redistribution
 *****************************************************************************/

#ifndef __BQ2461X_H__
#define __BQ2661X_H__

#include "bq2461x_defs.h"

#if defined(USE_HAL_DRIVER) && defined(STM32F407xx)
	// Support for stm32f4xx devices
	#include "stm32f4xx_hal.h"
    #include "main.h"

	/* 
	 * All the instances of these drivers share only one SPI bus, that we asume
	 * that it is already initialized on the main before the infinite loop. 
	 * This design takes into account that the project has been created with 
	 * the STM32CubeMX which creates the function needed to initialize the i2c
	 * bus.
	 *
	 * We also asume that the SPI bus used is the hspi1
	 */
#elif defined (__AVR__)
	// AVR devices support
#endif

/**
 * @struct BQ24_pin
 * @brief struct that represents BQ2461x I/O pins
 */
struct BQ24_pin {
    uint8_t logic;		// Pin ON Logic 
    uint8_t (*peek)(void);
    void (*set)(uint8_t);
};

/**
 * @struct BQ24
 * @brief BQ2461x handler
 */
struct BQ24 {
    struct BQ24_pin PG;
    struct BQ24_pin STAT1;
    struct BQ24_pin STAT2;
    struct BQ24_pin CE;
};

/* User Defined Functions ---------------------------------------------------*/

/**
 * @func  read_PG
 * @brief Reads PG pin from a TIs BQ2461x device.
 * 		  The user of this library is responsible of declareing and
 * 		  defining one read_PG function for each TI's BQ2461x
 * 		  implemented. As an example, we have only defined the
 * 		  functionalty for STM32F407xx.
 * @return uint8_t PG pin value
 */
uint8_t bq24_read_PG(void); 

/**
 * @func  read_STAT1
 * @brief Reads STAT1 pin from a TIs BQ2461x device.
 * 		  The user of this library is responsible of declareing and
 * 		  defining one read_STAT1 function for each TI's BQ2461x
 * 		  implemented. As an example, we have only defined the
 * 		  functionalty for STM32F407xx.
 * @return uint8_t STAT1 pin value
 */
uint8_t bq24_read_STAT1(void);

/**
 * @func  read_STAT2
 * @brief Reads STAT2 pin from a TIs BQ2461x device.
 * 		  The user of this library is responsible of declareing and
 * 		  defining one read_STAT2 function for each TI's BQ2461x
 * 		  implemented. As an example, we have only defined the
 * 		  functionalty for STM32F407xx.
 * @return uint8_t STAT2 pin value
 */
uint8_t bq24_read_STAT2(void);

/**
 * @func  read_CE
 * @brief Reads CE pin from a TIs BQ2461x device.
 * 		  The user of this library is responsible of declareing and
 * 		  defining one read_CE function for each TI's BQ2461x
 * 		  implemented. As an example, we have only defined the
 * 		  functionalty for STM32F407xx.
 * @return uint8_t CE pin value
 */
uint8_t bq24_read_CE(void);

/**
 * @func  write_CE
 * @brief Reads CE pin from a TIs BQ2461x device.
 * 		  The user of this library is responsible of declareing and
 * 		  defining one write_CE function for each TI's BQ2461x
 * 		  implemented. As an example, we have only defined the
 * 		  functionalty for STM32F407xx.
 */
void bq24_write_CE(uint8_t CE_Pin_State);

/* End of User Defined Functions ---------------------------------------------*/

/**
 * @func BQ24_read_charge_status
 * @brief Reads STAT1 & STATA2 pins & returns the BQ2461x chargien status.
 * @params[in] device: BQ24 handler pointer referencing to the desired BQ2461x
 *                     device
 * @return BQ24_charge_status [BQ24_CHARGE_COMPLETE|BQ24_CHARGE_IN_PROGRESS|
 *			       BQ24_FAULT|BQ24_INVALID_VALUE]
 */
enum BQ24_charge_status bq24_read_charge_status(struct BQ24 *device);

/**
 * @func is_power_good
 * @brief Reads PG pin & returns the BQ2461x power state. If charger finds 
 * @params[in] device: BQ24 handler pointer referencing to the desired BQ2461x
 *                     device
 * @return PG_status [ VALID_VCC: bq24618 has a valid VCC input | INVALID_VCC]
 */
enum BQ24_pg_status bq24_is_power_good(struct BQ24 *device);

/**
 * @func is_charge_enabled
 * @brief Reads CE pin & returns the BQ2461x charge enable state
 * @params[in] device: BQ24 handler pointer referencing to the desired BQ2461x
 *                     device
 * @return CE_status [ ce_ON | ce_OFF ]
 */
enum BQ24_ce_status bq24_is_charge_enabled(struct BQ24 *device);

/**
 * @func enable_charge
 * @brief Sets CE pin to ce_ON status. Used to enable the charge process.  A
 * 		high-level signal on this pin enables charge, provided all the other
 * 		conditions for charge are met (see Enable and Disable Charging).
 * @params[in] device: BQ24 handler pointer referencing to the desired BQ2461x
 *                     device
 * @return CE_status [ ce_ON | ce_OFF ]
 */
enum BQ24_ce_status bq24_enable_charge(struct BQ24 *device);

/**
 * @func disable_charge
 * @brief Sets CE pin to ce_OFF status. Used to disable the charge process. A
 * 		high-to-low transition on this pin also resets all timers and fault
 * 		conditions.
 * @params[in] device: BQ24 handler pointer referencing to the desired BQ2461x
 *                     device
 * @return CE_status [ ce_ON | ce_OFF ]
 */
enum BQ24_ce_status bq24_disable_charge(struct BQ24 *device);

/**
 * @func bq24_init
 * @brief Initializes a bq24 struct with the users desired configuration
 * @params[in] device: BQ24 handler pointer referencing to the desired device to
 *                         be initialized 
 * @params[in] pg_peek: pointer to pg_peek user defined function
 * @params[in] pg_logic: PG pin on value 
 * @params[in] stat1_peek: pointer to stat1_peek user defined function
 * @params[in] stat1_logic: STAT1 pin on value 
 * @params[in] stat2_peek: pointer to pstat2_pee user defined function
 * @params[in] stat2_logic: STAT2 pin on value 
 * @params[in] ce_peek: pointer to ce_peek user defined function
 * @params[in] ce_set: pointer to ce_set user defined function
 * @params[in] ce_logic: CE pin on value 
 * @return PG_status [ VALID_VCC | INVALID_VCC ]
 */
enum BQ24_pg_status bq24_init(struct BQ24 *device,
			    void (*pg_peek), uint8_t pg_logic,
                            void (*stat1_peek), uint8_t stat1_logic,
                            void (*stat2_peek), uint8_t stat2_logic,
                            void (*ce_peek), void (*ce_set), uint8_t ce_logic);


#endif

//
// End of file.
//

/******************************************************************************
 *
 * @file	bq24g1x.h
 * @brief 	Texas Instruments BQ2461x declarations of function and high
 *                      level structures
 * @version	v1.00f00
 * @date	17, Dec. 2020
 * @author	CECARELLI, Federico (fededc88@gmail.com),
 * 			MOYA, Martin		(moyamartin1@gmail.com),
 * 			SANTOS, Lucio		(lusho2206@gmail.com)
 * @copyright GPL license, all text here must be included in any redistribution
 *****************************************************************************/

#include "bq2461x.h"

uint8_t CHARGER_STATUS[4] = {BQ24_FAULT, BQ24_CHARGE_COMPLETE,
			    BQ24_CHARGE_IN_PROGRESS ,BQ24_UNDEFINED_VALUE };

/* User Defined Functions ---------------------------------------------------*/

/**
 * The user of this library is responsible of declareing and defining one of
 * each functions here for each TI's BQ2461x implemented.
 * As an example, we have defined the functionalty for STM32F407xx using the HAL
 * layer driver.
 * User sould not need to invoque these functions by their own
 */

/**
 * @func  read_PG
 * @brief Reads PG pin from a TIs BQ2461x device.
 * 		  The user of this library is responsible of declareing and
 * 		  defining one read_PG function for each TI's BQ2461x
 * 		  implemented. As an example, we have only defined the
 * 		  functionalty for STM32F407xx.
 * @return uint8_t PG pin value
 */
__weak uint8_t bq24_read_PG(void) {

#if defined(USE_HAL_DRIVER) && defined(STM32F407xx)

    if(HAL_GPIO_ReadPin(BQ24_PG_GPIO_Port, BQ24_PG_Pin) == GPIO_PIN_SET)
	return (uint8_t) GPIO_PIN_SET;
    else 
	return (uint8_t) GPIO_PIN_RESET;

#elif defined(__AVR__)
	// AVR devices support
#endif
}

/**
 * @func  read_STAT1
 * @brief Reads STAT1 pin from a TIs BQ2461x device.
 * 		  The user of this library is responsible of declareing and
 * 		  defining one read_STAT1 function for each TI's BQ2461x
 * 		  implemented. As an example, we have only defined the
 * 		  functionalty for STM32F407xx.
 * @return uint8_t STAT1 pin value
 */
__weak uint8_t bq24_read_STAT1(void) {

#if defined(USE_HAL_DRIVER) && defined(STM32F407xx)

    if(HAL_GPIO_ReadPin(BQ24_STAT1_GPIO_Port, BQ24_STAT1_Pin) == GPIO_PIN_SET)
	return (uint8_t) GPIO_PIN_SET;
    else
	return (uint8_t) GPIO_PIN_RESET;

#elif defined(__AVR__)
	// AVR devices support
#endif
}

/**
 * @func  read_STAT2
 * @brief Reads STAT2 pin from a TIs BQ2461x device.
 * 		  The user of this library is responsible of declareing and
 * 		  defining one read_STAT2 function for each TI's BQ2461x
 * 		  implemented. As an example, we have only defined the
 * 		  functionalty for STM32F407xx.
 * @return uint8_t STAT2 pin value
 */
__weak uint8_t bq24_read_STAT2(void) {

#if defined(USE_HAL_DRIVER) && defined(STM32F407xx)

    if(HAL_GPIO_ReadPin(BQ24_STAT2_GPIO_Port, BQ24_STAT2_Pin) == GPIO_PIN_SET)
	return (uint8_t) GPIO_PIN_SET;
    else
	return (uint8_t) GPIO_PIN_RESET;

#elif defined(__AVR__)
	// AVR devices support
#endif
}

/**
 * @func  read_CE
 * @brief Reads CE pin from a TIs BQ2461x device.
 * 		  The user of this library is responsible of declareing and
 * 		  defining one read_CE function for each TI's BQ2461x
 * 		  implemented. As an example, we have only defined the
 * 		  functionalty for STM32F407xx.
 * @return uint8_t CE pin value
 */
__weak uint8_t bq24_read_CE(void) {

#if defined(USE_HAL_DRIVER) && defined(STM32F407xx)

    if(HAL_GPIO_ReadPin(BQ24_CE_GPIO_Port, BQ24_CE_Pin) == GPIO_PIN_SET)
	return (uint8_t) GPIO_PIN_SET;
    else
	return (uint8_t) GPIO_PIN_RESET;

#elif defined(__AVR__)
	// AVR devices support
#endif
}

/**
 * @func  write_CE
 * @brief Reads CE pin from a TIs BQ2461x device.
 * 		  The user of this library is responsible of declareing and
 * 		  defining one write_CE function for each TI's BQ2461x
 * 		  implemented. As an example, we have only defined the
 * 		  functionalty for STM32F407xx.
 */
__weak void bq24_write_CE(uint8_t CE_Pin_State) {

#if defined(USE_HAL_DRIVER) && defined(STM32F407xx)

    if (CE_Pin_State == GPIO_PIN_SET)
	HAL_GPIO_WritePin(BQ24_CE_GPIO_Port, BQ24_CE_Pin, GPIO_PIN_SET);
    else
	HAL_GPIO_WritePin(BQ24_CE_GPIO_Port, BQ24_CE_Pin, GPIO_PIN_RESET);

#elif defined(__AVR__)
	// AVR devices support
#endif
    return;
}
/* End of User Defined Functions ---------------------------------------------*/

enum BQ24_charge_status bq24_read_charge_status(struct BQ24 *device) {

    uint8_t stat1; // Stat1 pin buffer
    uint8_t stat2; // Stat2 pin buffer
    uint8_t index; // Charger_status table index

    // Take a peek on the pins
    stat1 = !((*device->STAT1.peek)() ^ device->STAT1.logic);
    stat2 = !((*device->STAT2.peek)() ^ device->STAT2.logic);

    index = (uint8_t)(((stat1<<1) & 0x02) | (stat2 & 0x01));

    return CHARGER_STATUS[index];
}

/**
 * @func is_power_good
 * @brief Reads PG pin & returns the BQ2461x power state
 * @params[in] device: BQ24 handler pointer referencing to the desired BQ2461x
 *                     device
 * @return PG_status [ VALID_VCC | INVALID_VCC]
 */
enum BQ24_pg_status bq24_is_power_good(struct BQ24 *device){

    uint8_t read;

    // Take a peek on the pin
    read = (*device->PG.peek)();

    if(read == device->PG.logic)
	return BQ24_VALID_VCC;
    else
	return BQ24_INVALID_VCC;
}

/**
 * @func is_charge_enabled
 * @brief Reads CE pin & returns the BQ2461x charge enable state
 * @params[in] device: BQ24 handler pointer referencing to the desired BQ2461x
 *                     device
 * @return CE_status [ ce_ON | ce_OFF ]
 */
enum BQ24_ce_status bq24_is_charge_enabled(struct BQ24 *device){

    uint8_t read;

    // Take a peek on the pin
    read = (*device->CE.peek)();

    if(read == device->CE.logic)
	return BQ24_ce_ON;
    else
	return BQ24_ce_OFF;
}

/**
 * @func enable_charge
 * @brief Sets CE pin to ce_ON status
 * @params[in] device: BQ24 handler pointer referencing to the desired BQ2461x
 *                     device
 * @return CE_status [ ce_ON | ce_OFF ]
 */
enum BQ24_ce_status bq24_enable_charge(struct BQ24 *device) {

    if(bq24_is_charge_enabled(device) == BQ24_ce_ON)
	return BQ24_ce_ON;

    if(bq24_is_power_good(device) == BQ24_VALID_VCC)
	bq24_write_CE(device->PG.logic);

    return bq24_is_charge_enabled(device);
}

/**
 * @func disable_charge
 * @brief Sets CE pin to ce_OFF status
 * @params[in] device: BQ24 handler pointer referencing to the desired BQ2461x
 *                     device
 * @return CE_status [ ce_ON | ce_OFF ]
 */
enum BQ24_ce_status bq24_disable_charge(struct BQ24 *device) {

    if(bq24_is_charge_enabled(device) == BQ24_ce_OFF)
	return BQ24_ce_OFF;

    bq24_write_CE((uint8_t)(!device->PG.logic));

    return bq24_is_charge_enabled(device);
}

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
	void (*ce_peek), void (*ce_set), uint8_t ce_logic)
{
    device->PG.logic = pg_logic;
    device->PG.peek = pg_peek;
    device->PG.set  = NULL;

    device->STAT1.logic = stat1_logic;
    device->STAT1.peek = stat1_peek;
    device->STAT1.set  = NULL;

    device->STAT2.logic = stat2_logic;
    device->STAT2.peek = stat2_peek;
    device->STAT2.set  = NULL;

    device->CE.logic = ce_logic;
    device->CE.peek = ce_peek;
    device->CE.set  = ce_set;

    // charger starts disabled
    bq24_disable_charge(device);

    // return charger adapter status
    return bq24_is_power_good(device);

}
//
// End of file.
//

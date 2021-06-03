/******************************************************************************
 *
 * @file	bq2461x_defs.h
 * @brief 	Texas Instruments BQ2461x register and constants definitions
 * @version	v1.00f00
 * @date	29, may, 2021
 * @author	CECARELLI, Federico (fededc88@gmail.com),
 * 			MOYA, Martin		(moyamartin1@gmail.com),
 * 			SANTOS, Lucio		(lusho2206@gmail.com)
 * @copyright GPL license, all text here must be included in any redistribution
 *****************************************************************************/

#ifndef __BQ2461X_DEFS__
#define __BQ2461X_DEFS__

#include <stdint.h>

#ifndef __weak
#define __weak __attribute__((weak))
#endif

/**
* The open-drain PG (power-good) output indicates whether the VCC voltage is
* valid or not.
* BQ24 PG output open-drain FET turns on whenever the bq24618 has a valid VCC
* input.
 */
#define BQ24_PG_PIN_ON_VALUE 0

/**
* The open-drain STAT1 and STAT2 outputs indicate various charger operations as
* shown in Table 2.
* STATn OFF indicates that the open-drain FET transistor is turned off.
*/
#define BQ24_STATn_PIN_ON_VALUE 0

/**
 * The CE digital input is used to disable or enable the charge process.
 * A high-level signal on this pin enables charge, provided all the other
 * conditions for charge are met (see Enable and Disable Charging).
 * A high-to-low transition on this pin also resets all timers and fault
 * conditions.
*/
#define BQ24_CE_PIN_ON_VALUE 1

/**
 * @enum ce_status
 * @brief Represents the possible values for BQ2461x CE (charge enable) pin
 */
enum BQ24_ce_status {
    BQ24_ce_OFF = 0,
    BQ24_ce_ON
};

/**
 * @enum pq_status
 * @brief Represents the possible values for BQ2461x PG (power-good) pin
 */
enum BQ24_pg_status {
    BQ24_VALID_VCC = 0,
    BQ24_INVALID_VCC = -1,
};

/**
 * @enum charge_status
 * @brief Represents the possible values for BQ2461x STATn (charge status
 * output) pins
 */
enum BQ24_charge_status {
    BQ24_CHARGE_COMPLETE = 1,
    BQ24_CHARGE_IN_PROGRESS = 0,
    BQ24_FAULT = -1,
    BQ24_UNDEFINED_VALUE = -2
};

#endif

//
// End of file.
//

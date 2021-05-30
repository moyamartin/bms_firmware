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

#ifndef _weak
#define _weak __attribute__((weak))
#endif


/**
 * @enum bat_series_inputs
 * @brief Represents the possible values for cell series measurements, this is
 * related to bit fields CELL_SEL on adc_control
 * @see adc_control
 */
enum BQ24_ce_status {
    BQ24_ce_OFF = 0,
    BQ24_ce_ON
};

enum BQ24_pg_status {
    BQ24_VALID_VCC = 0,
    BQ24_INVALID_VCC = -1,
};

enum BQ24_charge_status {
    BQ24_CHARGE_COMPLETE = 1,
    BQ24_CHARGE_IN_PROGRESS = 0,
    BQ24_FAULT = -1,
    BQ24_INVALID_VALUE = -2
};

enum BQ24_pinState {
    ON = 0,
    OFF = 1
};

struct BQ24_pin {
    uint8_t logic;		// Pin ON Logic 
    uint8_t (*peek)(void);
    void (*set)(uint8_t);
};

/**
 * @struct io_control
 * @brief struct that represents I/O control register data
 */
struct BQ24_handler {
    struct BQ24_pin PG;
    struct BQ24_pin STAT1;
    struct BQ24_pin STAT2;
    struct BQ24_pin CE;
};
#endif

//
// End of file.
//

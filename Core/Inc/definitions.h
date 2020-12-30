/******************************************************************************
 *
 * @file	ina226_defs.h
 * @brief 	Texas Instruments INA226 register and constants definitions
 * @version	v1.00f00
 * @date	16, Dec. 2020
 * @author	CECARELLI, Federico (fededc88@gmail.com),
 * 			MOYA, Martin		(moyamartin1@gmail.com),
 * 			SANTOS, Lucio		(lusho2206@gmail.com)
 * @copyright GPL license, all text here must be included in any redistribution
 *****************************************************************************/

#ifndef	_DEFINITIONS_H_ 
#define _DEFINITIONS_H_ 

#include <stdint.h>

#ifndef TRUE
#define TRUE 1
#endif /* TRUE */

#ifndef FALSE
#define FALSE 0
#endif /* FALSE */


/**
 * @union 16b_buffer
 * @brief Union representing a buffer of 16 bits, that allows you to access to
 * 		  the low and high 8 bits of a uint16_t 
 */
union buffer_16b { 
	uint16_t all;
	struct {
		uint8_t b_low;
		uint8_t b_high;
	} separation;
};


/**
 * @func swap_16b
 * @brief swaps the high 8 bits with the 8 low bits of an unsigned data 
 * structure of 16 bits
 */
void swap_16b(uint16_t * source);

#endif /* INC_DEFINITIONS_H_ */

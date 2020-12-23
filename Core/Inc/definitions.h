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
 * @union 16b_buffer;
 * @brief Union representing a buffer of 16 bits, that allows you to access to
 * 		  the low and high 8 bits of a uint16_t 
 */
typedef union { 
	uint16_t all;
	struct {
		uint8_t b_low;
		uint8_t b_high;
	} separation;
} buffer_16b;

#endif /* INC_DEFINITIONS_H_ */

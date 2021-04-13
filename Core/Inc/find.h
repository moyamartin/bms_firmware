/******************************************************************************
 *
 * @file		find.h
 * @brief		Contains the definitions of helper function to find values of
 * 				different type in an array
 * @version		v1.00f00
 * @date		15. Nov. 2020
 * @author		CECCARELLI, Federico    (fededc88@gmail.com)
 *              MOYA, Martin            (moyamartin1@gmail.com)
 *              SANTOS, Lucio           (lusho2206@gmail.com)
 * @copyright	GPL license
 *
 *******************************************************************************/
#ifndef _FIND_H_
#define _FIND_H_

#include <stdint.h>
#include "arm_math.h"

/**
 *	@fn				find_closest_value_f32
 *	@brief  		Find the closest index of a needle inside a haysack 
 *					(array of values). This function is designed for an array 
 *					ordered desc. and data type of float32_t
 *	@params[in] 	needle value to find inside the haysack
 *	@params[in] 	haysack array of values
 *	@params[in] 	size_of_haysack the size of the haysack 
 *					(length of the array)
 *	@returns		returns the index where the value has been found	
 */
uint32_t find_closest_value_f32(float32_t needle, const float32_t * haysack, 
								size_t size_of_haysack);

/**
 *	@fn			  	get_closest_index_f32	
 *	@brief  		Gets the closest index of a value between two adyacents
 *					numbers	inside an array. 
 *	@params[in] 	target the value between index_a and index_b	
 *	@params[in] 	index_a the first index of the closest value to the target
 *	@params[in]   	index_b the second index of the closest value to the target	
 *	@params[in]		haysack the array of values where to look for
 *	@returns		returns the index of the closest value
 */
uint32_t get_closest_index_f32(float32_t target, uint32_t index_a, 
							   uint32_t index_b, const float32_t * haysack);

/**
 *	@fn			  	get_max_index_f32
 *	@brief  		Gets the index of the max value within an array
 *	@params[in]   	array of float32_t values where to look for the max value
 *	@params[in]		length of the array parameter
 *	@returns		returns the index of the max value
 */
uint32_t get_max_index_f32(const float32_t * array, size_t length);


/**
 *	@fn			  	get_min_index_f32
 *	@brief  		Gets the index of the min value within an array
 *	@params[in]   	array of float32_t values where to look for the max value
 *	@params[in]		length of the array parameter
 *	@returns		returns the index of the min value
 */
uint32_t get_min_index_f32(const float32_t * array, size_t lenght);

#endif

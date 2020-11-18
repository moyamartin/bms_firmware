#ifndef AUXILIARY_MATH_H_
#define AUXILIARY_MATH_H_

#include <stdint.h>
#include "arm_math.h"


/**
 *  @fn             arm_mat_eye_f32
 *  @brief          Initialize an Identity Arm float32_t Matrix
 *    
 *  @param[in, out] mat         Points to an instance of the floating-point 
 *                              matrix structure
 *  @param[in]      size		Identity matrices are squared, so it is only
 *  							necessary to specify one size of the matrix
 *  @returns        None
 */
void arm_mat_eye_f32(arm_matrix_instance_f32 * mat, uint16_t size, 
					 float32_t * data);

#endif

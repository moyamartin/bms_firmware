#include "auxiliary_mat.h"
#include <stdint.h>
#include <stdlib.h>

void arm_mat_eye_f32(arm_matrix_instance_f32 * mat, uint16_t size, 
	float32_t * data)
{
    /* Fill Data pointer */
	arm_mat_init_f32(mat, size, size, data);

	/* fill matrix axis with 1s */
	float32_t *pbuff = mat->pData;
	for(uint32_t i = 0; i < size; i++){
		*(pbuff) = 1.0f;
		pbuff += size + 1;
	}
}

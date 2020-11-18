#include "kalman.h"
#include "auxiliary_mat.h"
#include <stdlib.h>

/**
 *	@fn         state_extrapolation_predict
 *	@brief      Predicts the system state according to the actual state of the 
 *	            system and input. 
 *
 *	@param[in]  f_mat        State transition matrix
 *	@param[in]  x_mat_actual Current State Vector
 *	@param[in]  g_mat        Control matrix
 *	@param[in]  u            Input vector 
 *	@param[in]  x_mat_pred   State prediction vector
 *	@returns    arm_status
 *	@see        arm_status 
 */
static arm_status state_extrapolation_predict(
	const arm_matrix_instance_f32 *f_mat,
	const arm_matrix_instance_f32 *x_mat_actual, 
	const arm_matrix_instance_f32 *g_mat, const arm_matrix_instance_f32 *u, 
	arm_matrix_instance_f32 *x_mat_pred)
{
	arm_matrix_instance_f32 f_times_x;
	float32_t f_times_x_data[MATRIX_MAX_SIZE];
    arm_mat_init_f32(&f_times_x, f_mat->numRows, x_mat_actual->numCols, 
			f_times_x_data);
    arm_status current_state; 
    current_state = arm_mat_mult_f32(f_mat, x_mat_actual, &(f_times_x));
	if(current_state != ARM_MATH_SUCCESS){
        return current_state;
	}
	
	arm_matrix_instance_f32 g_times_u;
	float32_t g_times_u_data[MATRIX_MAX_SIZE];
    arm_mat_init_f32(&g_times_u, g_mat->numRows, u->numCols, g_times_u_data);
    current_state = arm_mat_mult_f32(g_mat, u, &g_times_u);
	if(current_state != ARM_MATH_SUCCESS) {
		return current_state;
	}

	current_state = arm_mat_add_f32(&f_times_x, &g_times_u, x_mat_pred);

	return current_state;
}

/**
 *	@fn         covariance_extrapolation 
 *	@brief      Calculates the covariance extrapolation for a system with dynamic
 *              model
 *
 *	@param[in]  f_mat           State transition matrix
 *	@param[in]  p_mat_actual    Actual Estimate Uncertainty Matrix
 *	@param[in]  f_mat_t         Transposed State Transition Matrix
 *	@param[in]  q_mat           Procees Noise Uncertainty
 *	@param[out] p_mat_pred      Predicted Estimate Uncertainty Matrix 
 *	@returns    arm_status
 *	@see        arm_status
 */
static arm_status covariance_extrapolation_uncertainty(
		const arm_matrix_instance_f32 *f_mat,
		const arm_matrix_instance_f32 *p_mat_actual,
		const arm_matrix_instance_f32 *f_mat_t,
		const arm_matrix_instance_f32 *q_mat,
		arm_matrix_instance_f32 *p_mat_pred)
{
	arm_matrix_instance_f32 f_times_p;
	float32_t f_times_p_data[MATRIX_MAX_SIZE];
    arm_mat_init_f32(&f_times_p, f_mat->numRows, p_mat_actual->numCols, 
			f_times_p_data);
    arm_status current_state = arm_mat_mult_f32(f_mat, p_mat_actual,
			&f_times_p);
	if(current_state != ARM_MATH_SUCCESS){
        return current_state;
	}

    arm_matrix_instance_f32 f_times_p_f_transpose;
	float32_t f_times_p_f_transpose_data[MATRIX_MAX_SIZE];
    arm_mat_init_f32(&f_times_p_f_transpose, f_times_p.numRows, 
            f_mat_t->numCols, f_times_p_f_transpose_data);
    current_state = arm_mat_mult_f32(&f_times_p, f_mat_t, 
			&f_times_p_f_transpose);
    if(current_state != ARM_MATH_SUCCESS){
        return current_state;
    }

    return arm_mat_add_f32(&f_times_p_f_transpose, q_mat, p_mat_pred);
}

/**
 *  @fn state_update
 *  @brief Updates the current state of the system taking into account the
 *  measurement and the previous prediction
 *
 *  @param[in]  h_mat           Observation matrix
 *  @param[in]  x_mat_pred      Predicted state matrix
 *  @param[in]  z               Measurement matrix
 *  @param[in]  k_mat           Kalman gain matrix
 *  @param[out] x_mat_actual    Actual state matrix
 *  @returns arm_status
 */
static arm_status state_update(const arm_matrix_instance_f32 *h_mat,
		const arm_matrix_instance_f32 *x_mat_pred,
		const arm_matrix_instance_f32 *measurement,
		const arm_matrix_instance_f32 *k_mat,
		arm_matrix_instance_f32 *x_mat_actual)
{
	arm_matrix_instance_f32 z;
	float32_t z_data[MATRIX_MAX_SIZE];
	arm_mat_init_f32(&z, x_mat_pred->numRows, x_mat_pred->numCols, z_data);
	arm_status current_state = arm_mat_mult_f32(h_mat, measurement, &z);
	if(current_state != ARM_MATH_SUCCESS){
		return current_state;
	}

    arm_matrix_instance_f32 h_times_x_prev;
	float32_t h_times_x_prev_data[MATRIX_MAX_SIZE];
    arm_mat_init_f32(&h_times_x_prev, h_mat->numRows, x_mat_pred->numCols, 
            h_times_x_prev_data);
    current_state = arm_mat_mult_f32(h_mat, x_mat_pred, 
            &h_times_x_prev);
    if(current_state != ARM_MATH_SUCCESS){
        return current_state;
    }

    arm_matrix_instance_f32 z_diff_h_times_x_prev;
	float32_t z_diff_h_times_x_prev_data[MATRIX_MAX_SIZE];
    arm_mat_init_f32(&z_diff_h_times_x_prev, z.numRows, z.numCols, 
			z_diff_h_times_x_prev_data);
    current_state = arm_mat_sub_f32(&z, &h_times_x_prev, &z_diff_h_times_x_prev);
    if(current_state != ARM_MATH_SUCCESS){
        return current_state;
    }
       
    arm_matrix_instance_f32 k_diff;
	float32_t k_diff_data[MATRIX_MAX_SIZE];
    arm_mat_init_f32(&k_diff, k_mat->numRows, z_diff_h_times_x_prev.numCols, 
			k_diff_data);
    current_state = arm_mat_mult_f32(k_mat, &z_diff_h_times_x_prev,
            &k_diff);
    if(current_state != ARM_MATH_SUCCESS){
        return current_state;
    }

    return arm_mat_add_f32(x_mat_pred, &k_diff, x_mat_actual);
}

/**
 *  @fn covariance_update 
 *  @brief Updates the current state of the system taking into account the
 *  measurement and the previous prediction
 *
 *  @param[in]  k_mat           Kalman gain matrix
 *  @param[in]  h_mat           Observation matrix
 *  @param[in]  id_mat          Identity matrix
 *  @param[in]  p_mat_pred      Predicted Uncertainty estimation matrix
 *  @param[in]  r_mat           Measurement uncertainty matrix 
 *  @param[out] p_mat_actual    Corrected Uncertainty estimation matrix
	arm_matrix_instance_f32 *r_mat;
 *  @returns arm_status
 */
static arm_status covariance_update(const arm_matrix_instance_f32 *k_mat, 
		const arm_matrix_instance_f32 *h_mat, 
		const arm_matrix_instance_f32 *id_mat,
		const arm_matrix_instance_f32 *p_mat_pred, 
		const arm_matrix_instance_f32 *r_mat, 
		arm_matrix_instance_f32 *p_mat_actual)
{
    arm_matrix_instance_f32 k_times_h;
	float32_t k_times_h_data[MATRIX_MAX_SIZE];
    arm_mat_init_f32(&k_times_h, k_mat->numRows, h_mat->numCols, 
			k_times_h_data);
    arm_status current_state = arm_mat_mult_f32(k_mat, h_mat, &k_times_h);
    if(current_state!= ARM_MATH_SUCCESS){
        return current_state;
    }

    arm_matrix_instance_f32 id_minus_kh;
	float32_t id_minus_kh_data[MATRIX_MAX_SIZE];
    arm_mat_init_f32(&id_minus_kh, id_mat->numRows, id_mat->numCols,
			id_minus_kh_data);
    current_state = arm_mat_sub_f32(id_mat, &k_times_h, &id_minus_kh);
    if(current_state != ARM_MATH_SUCCESS){
        return current_state;
    }

    arm_matrix_instance_f32 id_minus_kh_t;
	float32_t id_minus_kh_t_data[MATRIX_MAX_SIZE];
    arm_mat_init_f32(&id_minus_kh_t, id_mat->numRows, id_mat->numCols, 
			id_minus_kh_t_data);
    current_state = arm_mat_trans_f32(&id_minus_kh, &id_minus_kh_t);
    if(current_state != ARM_MATH_SUCCESS){
        return current_state;
    }

    arm_matrix_instance_f32 knh_times_p;
	float32_t knh_times_p_data[MATRIX_MAX_SIZE];
    arm_mat_init_f32(&knh_times_p, id_mat->numRows, id_mat->numCols, 
			knh_times_p_data);
    current_state = arm_mat_mult_f32(&id_minus_kh, p_mat_pred, &knh_times_p);
    if(current_state != ARM_MATH_SUCCESS){
        return current_state;
    }

    arm_matrix_instance_f32 first_term;
	float32_t first_term_data[MATRIX_MAX_SIZE];
    arm_mat_init_f32(&first_term, knh_times_p.numRows, id_minus_kh_t.numCols, 
            first_term_data);
    current_state = arm_mat_mult_f32(&knh_times_p, &id_minus_kh_t, &first_term);
    if(current_state != ARM_MATH_SUCCESS){
        return current_state;
    }

    arm_matrix_instance_f32 k_times_r;
	float32_t k_times_r_data[MATRIX_MAX_SIZE];
    arm_mat_init_f32(&k_times_r, k_mat->numRows, r_mat->numCols, 
			k_times_r_data);
    current_state = arm_mat_mult_f32(k_mat, r_mat, &k_times_r);
    if(current_state != ARM_MATH_SUCCESS){
        return current_state;
    }

    arm_matrix_instance_f32 k_mat_t;
	float32_t k_mat_t_data[MATRIX_MAX_SIZE];
    arm_mat_init_f32(&k_mat_t, k_mat->numRows, k_mat->numCols, k_mat_t_data);
    current_state = arm_mat_trans_f32(k_mat, &k_mat_t);

    arm_matrix_instance_f32 second_term;
	float32_t second_term_data[MATRIX_MAX_SIZE];
    arm_mat_init_f32(&second_term, k_times_r.numRows, k_mat_t.numCols, 
			second_term_data);
    current_state = arm_mat_mult_f32(&k_times_r, &k_mat_t, &second_term);
    if(current_state != ARM_MATH_SUCCESS){
        return current_state;
    }
    
    return arm_mat_add_f32(&first_term, &second_term, p_mat_actual);
}

/**
 *  @fn         kalman_gain_update
 *  @brief      Updates the Kalman Gain based on the current value of the 
 *              covariance. The Kalman Filter is an optimal filter. 
 *              Thus, the algorithm tends to seek for a Kalman gain that 
 *              minimizes the uncertainty of the estimation
 *
 *  @param[in]  h_mat       Observation matrix 
 *  @param[in]  h_mat_t     Transposed Observation matrix
 *  @param[in]  p_mat_pred  Predicted Uncertainty estimation matrix
 *  @param[in]  r_mat       Measurement Uncertainty matrix
 *  @param[out] k_mat       Kalman Gain Matrix
 *  @returns    arm_status
 */
static arm_status kalman_gain_update(const arm_matrix_instance_f32 *h_mat, 
		const arm_matrix_instance_f32 *h_mat_t,
		const arm_matrix_instance_f32 *p_mat_pred, 
		const arm_matrix_instance_f32 *r_mat,
		arm_matrix_instance_f32 *k_mat)
{
    arm_matrix_instance_f32 h_times_p;
	float32_t h_times_p_data[MATRIX_MAX_SIZE];
    arm_status current_state;
    arm_mat_init_f32(&h_times_p, h_mat->numRows, p_mat_pred->numCols, 
			h_times_p_data);
    current_state = arm_mat_mult_f32(h_mat, p_mat_pred, &h_times_p);
    if(current_state != ARM_MATH_SUCCESS){
        return current_state;
    }

    arm_matrix_instance_f32 h_times_p_times_h_t;
	float32_t h_times_p_times_h_t_data[MATRIX_MAX_SIZE];
    arm_mat_init_f32(&h_times_p_times_h_t, h_times_p.numRows, h_mat_t->numCols, 
            h_times_p_times_h_t_data);
    current_state = arm_mat_mult_f32(&h_times_p, h_mat_t, &h_times_p_times_h_t);
    if(current_state != ARM_MATH_SUCCESS){
        return current_state;
    }

    arm_matrix_instance_f32 prev_inverse_inner_sum;
	float32_t prev_inverse_inner_sum_data[MATRIX_MAX_SIZE];
    arm_mat_init_f32(&prev_inverse_inner_sum, r_mat->numRows, r_mat->numCols,
            prev_inverse_inner_sum_data);
    current_state = arm_mat_add_f32(&h_times_p_times_h_t, r_mat, 
			&prev_inverse_inner_sum);
    if(current_state != ARM_MATH_SUCCESS){
        return current_state;
    }

    arm_matrix_instance_f32 inverse;
	float32_t inverse_data[MATRIX_MAX_SIZE];
    arm_mat_init_f32(&inverse, prev_inverse_inner_sum.numRows, 
            prev_inverse_inner_sum.numCols, inverse_data);
    current_state = arm_mat_inverse_f32(&prev_inverse_inner_sum, &inverse);
    if(current_state != ARM_MATH_SUCCESS){
        return current_state;
    }
    
    arm_matrix_instance_f32 h_t_inverse;
	float32_t h_t_inverse_data[MATRIX_MAX_SIZE];
    arm_mat_init_f32(&h_t_inverse, h_mat_t->numRows, inverse.numCols, 
			h_t_inverse_data);
    current_state = arm_mat_mult_f32(h_mat_t, &inverse, &h_t_inverse);
    if(current_state != ARM_MATH_SUCCESS){
        return current_state;
    }

    return arm_mat_mult_f32(p_mat_pred, &h_t_inverse, k_mat);
}

uint8_t kalman_filter_init(struct KalmanFilter * kalman_filter, 
        uint16_t n_states, uint16_t n_input, uint16_t n_output)
{   

    /* ----- Initialize Vectors ----- */

    // Initialize State vector
	arm_mat_init_f32(&kalman_filter->x_act, n_states, 1, 
			kalman_filter->x_act_data);

	// Initialize State predicted vector with zeros
	arm_mat_init_f32(&kalman_filter->x_pred, n_states, 1, 
			kalman_filter->x_pred_data);

	// Initialize Input Vector
	arm_mat_init_f32(&kalman_filter->u, n_input, 1, kalman_filter->u_data);

    // Initialize State Transition Matrix
    arm_mat_init_f32(&kalman_filter->f_mat, n_states, n_states, 
			kalman_filter->f_mat_data);

	// Initialize Transposed State Transition Matrix
	arm_mat_init_f32(&kalman_filter->f_mat_t, n_states, n_states,
			kalman_filter->f_mat_t_data);
	arm_status status = arm_mat_trans_f32(&kalman_filter->f_mat, 
			&kalman_filter->f_mat_t);
    if(status != ARM_MATH_SUCCESS){
        return 0;
    }                                                

    // Initialize Estimate Uncertainty Matrix
    arm_mat_init_f32(&kalman_filter->p_mat_actual, n_states, n_states, 
            kalman_filter->p_mat_actual_data);

	// Initialize Estimate Predicted Uncertainty Matrix
	arm_mat_init_f32(&kalman_filter->p_mat_pred, n_states, n_states, 
			kalman_filter->p_mat_pred_data);

    // Initialize Process Noise Uncertainty Matrix
    arm_mat_init_f32(&kalman_filter->q_mat, n_states, n_states, 
			kalman_filter->q_mat_data);

    // Initialize Control Matrix
    arm_mat_init_f32(&kalman_filter->g_mat, n_states, n_input, 
			kalman_filter->g_mat_data);

    // Initialize Measurement Uncertainty
    arm_mat_init_f32(&kalman_filter->r_mat, n_output, n_output, 
			kalman_filter->r_mat_data);

	// Initialize Kalman Gain
	arm_mat_init_f32(&kalman_filter->k_mat, n_states, n_input, 
			kalman_filter->k_mat_data);

    // Initialize Observation Matrix 
    arm_mat_init_f32(&kalman_filter->h_mat, n_output, n_states, 
			kalman_filter->h_mat_data);

    // Initialize Observation transposed matrix
	arm_mat_init_f32(&kalman_filter->h_mat_t, n_states, n_output,
			kalman_filter->h_mat_t_data);
	status = arm_mat_trans_f32(&kalman_filter->h_mat, 
			&kalman_filter->h_mat_t);
    if(status != ARM_MATH_SUCCESS){
        return 0;
    }                                                

	// Initialize Identity matrix
	arm_mat_eye_f32(&kalman_filter->id_mat, n_states, 
			kalman_filter->id_mat_data);
    
    return 1;
}

kalman_filter_status kalman_filter_step(struct KalmanFilter *kalman_filter, 
		const float32_t * measurement)
{
    /* Time Update */
    // Extrapolate the state of the system based on the current state and 
    if(state_extrapolation_predict(&kalman_filter->f_mat, &kalman_filter->x_act, 
				&kalman_filter->g_mat, &kalman_filter->u, &kalman_filter->x_pred) 
            != ARM_MATH_SUCCESS){
        return KALMAN_FAILED_EXTRAPOLATE_STATE;
    }

    // Extrapolate uncertainty
    if(covariance_extrapolation_uncertainty(&kalman_filter->f_mat,
				&kalman_filter->p_mat_actual, &kalman_filter->f_mat_t,
				&kalman_filter->q_mat, &kalman_filter->p_mat_pred) 
            != ARM_MATH_SUCCESS){
        return KALMAN_FAILED_EXTRAPOLATE_UNCERTAINTY;
    }

    /* Measurement Update ('correct') */
    // Compute the kalman gain
    if(kalman_gain_update(&kalman_filter->h_mat, &kalman_filter->h_mat_t,
				&kalman_filter->p_mat_pred, &kalman_filter->r_mat,
				&kalman_filter->k_mat) != ARM_MATH_SUCCESS){
        return KALMAN_FAILED_GAIN_CALCULATION;
    }

	arm_matrix_instance_f32 measurement_mat;
	arm_mat_init_f32(&measurement_mat, kalman_filter->h_mat.numCols, 1, 
			measurement);
    // Update estimate with measurement
    if(state_update(&kalman_filter->h_mat, &kalman_filter->x_pred,
				&measurement_mat, &kalman_filter->k_mat, 
				&kalman_filter->x_act) != ARM_MATH_SUCCESS){
        return KALMAN_FAILED_GAIN_CALCULATION;
    }

    // Update the estimate uncertainty
    if(covariance_update(&kalman_filter->k_mat, &kalman_filter->h_mat,
                &kalman_filter->id_mat, &kalman_filter->p_mat_pred,
                &kalman_filter->r_mat, &kalman_filter->p_mat_actual) 
            != ARM_MATH_SUCCESS){
        return KALMAN_FAILED_UPDATE_UNCERTAINTY;
    }

    // Lastly return success
    return KALMAN_SUCCESS;
}

void kalman_filter_destruct(struct KalmanFilter *kalman_filter)
{
	free(kalman_filter);
}

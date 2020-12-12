#include "kalman.h"
#include "auxiliary_mat.h"
#include <stdlib.h>

/**
 *	@fn         state_extrapolation_predict
 *	@brief      Predicts the system state according to the actual state of the 
 *	            system and input following the next equation:
 *
 *	            x_{n+1,n} = F*x_{n,n} + G*u_{n,n}
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
	/* Calculates F*x_{n,n} */
	arm_matrix_instance_f32 f_times_x;
	float32_t f_times_x_data[MATRIX_MAX_SIZE];
    arm_mat_init_f32(&f_times_x, f_mat->numRows, x_mat_actual->numCols, 
					 f_times_x_data);
    arm_status current_state; 
    current_state = arm_mat_mult_f32(f_mat, x_mat_actual, &f_times_x);
	if(current_state != ARM_MATH_SUCCESS){
        return current_state;
	}
	
	/* Calculates G*u_{n,n} */
	arm_matrix_instance_f32 g_times_u;
	float32_t g_times_u_data[MATRIX_MAX_SIZE];
    arm_mat_init_f32(&g_times_u, g_mat->numRows, u->numCols, g_times_u_data);
    current_state = arm_mat_mult_f32(g_mat, u, &g_times_u);
	if(current_state != ARM_MATH_SUCCESS) {
		return current_state;
	}

	/* 
	 * Calculates F*x_{n,n} + G*u_{n,n} and returns if the operation is
	 * successful
	 */
	return arm_mat_add_f32(&f_times_x, &g_times_u, x_mat_pred);
}

/**
 *	@fn         covariance_extrapolation 
 *	@brief      Calculates the covariance extrapolation for a dynamic system
 *				following the next equation.
 *
 *				P_{n+1,n} = F*P_{n,n}*F' + Q
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
	/* Calculates F*P_{n,n} */
	arm_matrix_instance_f32 f_times_p;
	float32_t f_times_p_data[MATRIX_MAX_SIZE];
    arm_mat_init_f32(&f_times_p, f_mat->numRows, p_mat_actual->numCols, 
					 f_times_p_data);
    arm_status current_state = arm_mat_mult_f32(f_mat, p_mat_actual,
											    &f_times_p);
	if(current_state != ARM_MATH_SUCCESS){
        return current_state;
	}

	/* Calculates F*P_{n,n}*F' */
    arm_matrix_instance_f32 f_times_p_f_transpose;
	float32_t f_times_p_f_transpose_data[MATRIX_MAX_SIZE];
    arm_mat_init_f32(&f_times_p_f_transpose, f_times_p.numRows, 
					 f_mat_t->numCols, f_times_p_f_transpose_data);
    current_state = arm_mat_mult_f32(&f_times_p, f_mat_t, 
							         &f_times_p_f_transpose);
    if(current_state != ARM_MATH_SUCCESS){
        return current_state;
    }
	
	/* Calculates F*P_{n,n}*F' + Q */
    return arm_mat_add_f32(&f_times_p_f_transpose, q_mat, p_mat_pred);
}

/**
 *  @fn state_update
 *  @brief	Updates the current state of the system taking into account the
 * 			measurement and the previous prediction following the next equation
 *
 * 			x_{n,n} = x_{n,n-1} + K_n*(z_n - (H*x_{n,n-1} + D*u))
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
							   const arm_matrix_instance_f32 *observation,
							   const arm_matrix_instance_f32 *k_mat,
							   const arm_matrix_instance_f32 *j_mat,
							   const arm_matrix_instance_f32 *u_mat,
							   arm_matrix_instance_f32 *x_mat_actual)
{

	/* Calculates H*x_{n,n-1} */
    arm_matrix_instance_f32 h_times_x_prev;
	float32_t h_times_x_prev_data[MATRIX_MAX_SIZE];
    arm_mat_init_f32(&h_times_x_prev, h_mat->numRows, x_mat_pred->numCols, 
					 h_times_x_prev_data);
    arm_status current_state = arm_mat_mult_f32(h_mat, x_mat_pred, 
												&h_times_x_prev);
    if(current_state != ARM_MATH_SUCCESS){
        return current_state;
    }

	/* Calculates J*u */
	arm_matrix_instance_f32 j_times_u;
	float32_t j_times_u_data[MATRIX_MAX_SIZE];
	arm_mat_init_f32(&j_times_u, j_mat->numRows, u_mat->numCols, 
					 j_times_u_data);
	current_state = arm_mat_mult_f32(j_mat, u_mat, &j_times_u);
	if(current_state != ARM_MATH_SUCCESS){
		return current_state;
	}

	/* Calculates H*x_{n,n-1} + D*u */
	arm_matrix_instance_f32 model;
	float32_t model_data[MATRIX_MAX_SIZE];
	arm_mat_init_f32(&model, h_times_x_prev.numRows, h_times_x_prev.numCols,
					 model_data);
	current_state = arm_mat_add_f32(&h_times_x_prev, &j_times_u, &model);
	if(current_state != ARM_MATH_SUCCESS){
		return current_state;
	}

	
	/* Calculates z_n - (H*x_{n,n-1} + D*u) */
    arm_matrix_instance_f32 innovation;
	float32_t innovation_data[MATRIX_MAX_SIZE];
    arm_mat_init_f32(&innovation, model.numRows, model.numCols, 
					 innovation_data);
    current_state = arm_mat_sub_f32(observation, &model, &innovation);
    if(current_state != ARM_MATH_SUCCESS){
        return current_state;
    }
       
    arm_matrix_instance_f32 k_diff;
	float32_t k_diff_data[MATRIX_MAX_SIZE];
    arm_mat_init_f32(&k_diff, k_mat->numRows, innovation.numCols, k_diff_data);
    current_state = arm_mat_mult_f32(k_mat, &innovation, &k_diff);
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
	/* Calculte K*H */
    arm_matrix_instance_f32 k_times_h;
	float32_t k_times_h_data[MATRIX_MAX_SIZE];
    arm_mat_init_f32(&k_times_h, k_mat->numRows, h_mat->numCols, 
			k_times_h_data);
    arm_status current_state = arm_mat_mult_f32(k_mat, h_mat, &k_times_h);
    if(current_state!= ARM_MATH_SUCCESS){
        return current_state;
    }
	
	/* Calculate I - K*H */
    arm_matrix_instance_f32 id_minus_kh;
	float32_t id_minus_kh_data[MATRIX_MAX_SIZE];
    arm_mat_init_f32(&id_minus_kh, id_mat->numRows, id_mat->numCols,
					 id_minus_kh_data);
    current_state = arm_mat_sub_f32(id_mat, &k_times_h, &id_minus_kh);
    if(current_state != ARM_MATH_SUCCESS){
        return current_state;
    }
	
	/* Calculate (I - K*H)' */
    arm_matrix_instance_f32 id_minus_kh_t;
	float32_t id_minus_kh_t_data[MATRIX_MAX_SIZE];
    arm_mat_init_f32(&id_minus_kh_t, id_mat->numRows, id_mat->numCols, 
					 id_minus_kh_t_data);
    current_state = arm_mat_trans_f32(&id_minus_kh, &id_minus_kh_t);
    if(current_state != ARM_MATH_SUCCESS){
        return current_state;
    }

	/* Calculate (I - K*H)*P */
    arm_matrix_instance_f32 knh_times_p;
	float32_t knh_times_p_data[MATRIX_MAX_SIZE];
    arm_mat_init_f32(&knh_times_p, id_mat->numRows, id_mat->numCols, 
					 knh_times_p_data);
    current_state = arm_mat_mult_f32(&id_minus_kh, p_mat_pred, &knh_times_p);
    if(current_state != ARM_MATH_SUCCESS){
        return current_state;
    }

	
	/* Calculate (I - K*H)*P*(I - K*H)' */
    arm_matrix_instance_f32 first_term;
	float32_t first_term_data[MATRIX_MAX_SIZE];
    arm_mat_init_f32(&first_term, knh_times_p.numRows, id_minus_kh_t.numCols, 
					 first_term_data);
    current_state = arm_mat_mult_f32(&knh_times_p, &id_minus_kh_t, 
									 &first_term);
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
    arm_mat_init_f32(&k_mat_t, k_mat->numCols, k_mat->numRows, k_mat_t_data);
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
    current_state = arm_mat_mult_f32(&h_times_p, h_mat_t, 
									 &h_times_p_times_h_t);
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

kalman_filter_status kalman_filter_init(struct KalmanFilter * kalman_filter,
										const float32_t * x_act_data_param,
										const float32_t * f_mat_data_param,
										const float32_t * g_mat_data_param,
										const float32_t * h_mat_data_param,
										const float32_t * q_mat_data_param,
										const float32_t * p_mat_data_param,
										const float32_t * r_mat_data_param,
										const float32_t * u_mat_data_param,
										const float32_t * j_mat_data_param,
										uint16_t n_states, uint16_t n_inputs, 
										uint16_t n_outputs)
{
	// set kalman filter matrix dimensions
	kalman_filter->n_states = n_states;
	kalman_filter->n_inputs = n_inputs;
	kalman_filter->n_outputs = n_outputs;

	// Initialize system state
	if(x_act_data_param != NULL){
		memcpy(kalman_filter->x_act_data, x_act_data_param, 
			   n_states*sizeof(float32_t));
	}
	arm_mat_init_f32(&kalman_filter->x_act, n_states, 1, 
					 kalman_filter->x_act_data);

	// Initialize State predicted vector
	arm_mat_init_f32(&kalman_filter->x_pred, n_states, 1, 
					 kalman_filter->x_pred_data);

	// Initialize State Input Vector
	if(u_mat_data_param != NULL){
		kalman_filter_modify_u_data(kalman_filter, u_mat_data_param);
	}
	arm_mat_init_f32(&kalman_filter->u, n_inputs, 1, kalman_filter->u_data);

	// Initialize State Transition Matrix
	if(f_mat_data_param != NULL){
		memcpy(kalman_filter->f_mat_data, f_mat_data_param,
			   n_states*(unsigned long int)n_states*sizeof(float32_t));
	}
	arm_mat_init_f32(&kalman_filter->f_mat, n_states, n_states, 
					 kalman_filter->f_mat_data);

	// Initialize Transposed State Transition Matrix
	arm_mat_init_f32(&kalman_filter->f_mat_t, n_states, n_states,
					 kalman_filter->f_mat_t_data);
	arm_status status = arm_mat_trans_f32(&kalman_filter->f_mat,
										  &kalman_filter->f_mat_t);
	if(status != ARM_MATH_SUCCESS){
		return KALMAN_FAILED_INIT;
	}

		
	// Initialize Estimate Uncertainty Matrix
	if(p_mat_data_param != NULL){
		memcpy(kalman_filter->p_mat_actual_data, p_mat_data_param, 
			   n_states*(unsigned long int)n_states*sizeof(float32_t));
	}
	arm_mat_init_f32(&kalman_filter->p_mat_actual, n_states, n_states,
					 kalman_filter->p_mat_actual_data);

	// Initialize Estimated Predicted Uncertainty Matrix to zero
	arm_mat_init_f32(&kalman_filter->p_mat_pred, n_states, n_states,
					 kalman_filter->p_mat_pred_data);

	// Initialize Control Matrix
	if(g_mat_data_param != NULL){
		kalman_filter_modify_g_data(kalman_filter, g_mat_data_param);
	}
	arm_mat_init_f32(&kalman_filter->g_mat, n_states, n_inputs,
					 kalman_filter->g_mat_data);

	// Initialize Measurement Uncertainty
	if(r_mat_data_param != NULL){
		kalman_filter_modify_r_data(kalman_filter, r_mat_data_param);
	}
	arm_mat_init_f32(&kalman_filter->r_mat, n_outputs, n_outputs, 
					 kalman_filter->r_mat_data);

	// Initialize Kalman Gain Matrix
	arm_mat_init_f32(&kalman_filter->k_mat, n_states, n_outputs, 
				     kalman_filter->k_mat_data);

	// Initialize Observation matrix
	if(h_mat_data_param != NULL){
		memcpy(kalman_filter->h_mat_data, h_mat_data_param, 
			   kalman_filter->n_outputs*
			   (unsigned long int)kalman_filter->n_states*sizeof(float32_t));
	}
	arm_mat_init_f32(&kalman_filter->h_mat, n_outputs, n_states, 
					 kalman_filter->h_mat_data);

	// Initialize Observation transposed matrix
	arm_mat_init_f32(&kalman_filter->h_mat_t, n_states, n_outputs,
					 kalman_filter->h_mat_t_data);
	status = arm_mat_trans_f32(&kalman_filter->h_mat, &kalman_filter->h_mat_t);
	if(status != ARM_MATH_SUCCESS){
		return KALMAN_FAILED_INIT;
	}

	// initialize D matrix
	if(j_mat_data_param != NULL){
		kalman_filter_modify_j_data(kalman_filter, j_mat_data_param);
	}
	arm_mat_init_f32(&kalman_filter->j_mat, n_inputs, n_inputs, 
					 kalman_filter->j_mat_data);

	// Initialize Process Noise Uncertainty Matrix
	if(q_mat_data_param != NULL){
		memcpy(kalman_filter->q_mat_data, q_mat_data_param,
			   n_states*(unsigned long int)n_states*sizeof(float32_t));
	}
	arm_mat_init_f32(&kalman_filter->q_mat, n_states, n_states, 
				     kalman_filter->q_mat_data);
	
	// Initialize Identity matrix
	arm_mat_eye_f32(&kalman_filter->id_mat, n_states, 
					kalman_filter->id_mat_data);
	return KALMAN_SUCCESS;
}

kalman_filter_status kalman_filter_step(struct KalmanFilter *kalman_filter, 
										float32_t * measurement)
{
    /* Time Update */
    // Extrapolate the state of the system based on the current state and 
    if(state_extrapolation_predict(&kalman_filter->f_mat, &kalman_filter->x_act, 
								   &kalman_filter->g_mat, &kalman_filter->u, 
								   &kalman_filter->x_pred) 
			!= ARM_MATH_SUCCESS){
        return KALMAN_FAILED_EXTRAPOLATE_STATE;
    }

    // Extrapolate uncertainty
    if(covariance_extrapolation_uncertainty(&kalman_filter->f_mat,
											&kalman_filter->p_mat_actual, 
											&kalman_filter->f_mat_t,
											&kalman_filter->q_mat, 
											&kalman_filter->p_mat_pred) 
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

	/* calculate observation if necessary */
	arm_matrix_instance_f32 measurement_mat;
	arm_mat_init_f32(&measurement_mat, kalman_filter->n_outputs, 1, 
					 measurement);
    // Update estimate with measurement
    if(state_update(&kalman_filter->h_mat, &kalman_filter->x_pred,
					&measurement_mat, &kalman_filter->k_mat, 
					&kalman_filter->j_mat, &kalman_filter->u,
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

arm_status kalman_filter_modify_f_data(struct KalmanFilter * kalman_filter,
									   const float32_t * data)
{
	// Updates matrix f
	memcpy(kalman_filter->f_mat_data, data, kalman_filter->n_states*
		   (unsigned long int)kalman_filter->n_states*sizeof(float32_t));
	// Updates matrix f_transposed
	return arm_mat_trans_f32(&kalman_filter->f_mat, &kalman_filter->f_mat_t);
}

void * kalman_filter_modify_g_data(struct KalmanFilter * kalman_filter,
								   const float32_t * data)
{
	return memcpy(kalman_filter->g_mat_data, data, kalman_filter->n_states*
				  (unsigned long int)kalman_filter->n_inputs*
				  sizeof(float32_t));
}

arm_status kalman_filter_modify_h_data(struct KalmanFilter * kalman_filter,
									   const float32_t * data)
{
	memcpy(kalman_filter->h_mat_data, data, kalman_filter->n_outputs*
				  (unsigned long int)kalman_filter->n_states*
				  sizeof(float32_t));
	return arm_mat_trans_f32(&kalman_filter->h_mat, &kalman_filter->h_mat_t);
}

void * kalman_filter_modify_q_data(struct KalmanFilter * kalman_filter,
								   const float32_t * data)
{	
	return memcpy(kalman_filter->q_mat_data, data, kalman_filter->n_states*
				  (unsigned long int)kalman_filter->n_states*
				  sizeof(float32_t));
}

void * kalman_filter_modify_r_data(struct KalmanFilter * kalman_filter,
								   const float32_t * data)
{
	return memcpy(kalman_filter->r_mat_data, data,
				  kalman_filter->n_inputs*
				  (unsigned long int)kalman_filter->n_inputs*
				  sizeof(float32_t));
}

void * kalman_filter_modify_u_data(struct KalmanFilter * kalman_filter,
								   const float32_t * data)
{
	return memcpy(kalman_filter->u_data, data, 
				  kalman_filter->n_inputs*sizeof(float32_t));
}

void * kalman_filter_modify_j_data(struct KalmanFilter * kalman_filter,
								   const float32_t * data)
{
	return memcpy(kalman_filter->j_mat_data, data, 
				  kalman_filter->n_inputs*
				  (unsigned long int)kalman_filter->n_inputs*
				  sizeof(float32_t));
}

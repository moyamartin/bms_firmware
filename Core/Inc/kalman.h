/******************************************************************************
 *
 * @file			kalman.h
 * @brief		Contains the definition of a Kalman filter optimized for ARM
 *               devices, it is based on the following page (www.kalmanfilter.net)
 * @version		v1.00f00
 * @date			10. Aug. 2020
 * @author		CECCARELLI, Federico    (fededc88@gmail.com)
 *               MOYA, Martin            (moyamartin1@gmail.com)
 *               SANTOS, Lucio           (lusho2206@gmail.com)
 * @copyright	GPL license
 *
 *******************************************************************************/
#ifndef KALMAN_H
#define KALMAN_H

#include "stm32f4xx_hal.h"
#include "arm_math.h"
#include "definitions.h"
#include <stdint.h>

#define MATRIX_MAX_SIZE 64
#define VECTOR_MAX_SIZE 8

/**
 *	@typedef    kalman_filter_status
 *
 *	@brief      This typedef defines the state of the kalman filter. It states 
 *	            in which step of the process it has failed	
 */
typedef enum
{
	KALMAN_SUCCESS = 0,
	KALMAN_FAILED_EXTRAPOLATE_STATE = 1,
	KALMAN_FAILED_EXTRAPOLATE_UNCERTAINTY = 2,
	KALMAN_FAILED_GAIN_CALCULATION = 3,
	KALMAN_FAILED_UPDATE_ESTIMATE = 4,
	KALMAN_FAILED_UPDATE_UNCERTAINTY = 5,
	KALMAN_FAILED_UPDATE_MEASUREMENT = 6
} kalman_filter_status;

/**
 *	@struct	KalmanFilter
 * 	@brief	This structure defines the Kalman filter data structure
 *
 */
struct KalmanFilter
{
	/**
	 *	@brief  State vector
	 */
	float32_t x_act_data[VECTOR_MAX_SIZE], x_pred_data[VECTOR_MAX_SIZE];
	arm_matrix_instance_f32 x_act, x_pred;

	/**
	 *	@brief	Input vector
	 */
	float32_t u_data[VECTOR_MAX_SIZE];
	arm_matrix_instance_f32 u;

	/**
	 *	@brief  State Transition Matrix
	 *	@note   It includes the transposed version of the matrix
	 */
	float32_t f_mat_data[MATRIX_MAX_SIZE], f_mat_t_data[MATRIX_MAX_SIZE];
	arm_matrix_instance_f32 f_mat, f_mat_t;

	/**
	 * 	@brief  Control Matrix
	 */
	float32_t g_mat_data[MATRIX_MAX_SIZE];
	arm_matrix_instance_f32 g_mat;

	/**
	 * 	@brief  Estimate Uncertainty Matrix
	 */
	float32_t p_mat_pred_data[MATRIX_MAX_SIZE], 
			  p_mat_actual_data[MATRIX_MAX_SIZE];
	arm_matrix_instance_f32 p_mat_pred, p_mat_actual;

	/**
	 * 	@brief  Process Noise Uncertainty Matrix
	 */
	float32_t q_mat_data[MATRIX_MAX_SIZE];
	arm_matrix_instance_f32 q_mat;

	/**
	 * 	@brief  Measurement Uncertainty Matrix
	 */
	float32_t r_mat_data[MATRIX_MAX_SIZE];
	arm_matrix_instance_f32 r_mat;

	/**
	 *	@brief  Observation matrix
	 *	@note   It includes the transposed version of the Observation matrix 
	 */
	float32_t h_mat_data[MATRIX_MAX_SIZE], h_mat_t_data[MATRIX_MAX_SIZE];
	arm_matrix_instance_f32 h_mat, h_mat_t;

	/**
	 * 	@brief  Kalman gain
	 */
	float32_t k_mat_data[MATRIX_MAX_SIZE];
	arm_matrix_instance_f32 k_mat;

	/**
	 *  @brief  identity matrix
	 */
	float32_t id_mat_data[MATRIX_MAX_SIZE];
	arm_matrix_instance_f32 id_mat;
};

/**
 *  @fn         kalman_filter_init
 *  @brief      Initializes a kalman filter struture
 *
 *  @params[in] kalman_filter Kalman Filter data structure
 *  @params[in] n_states Number of states of the system
 *  @params[in] n_observables Number of observables
 *  @returns    kalman_filter_status
 */
uint8_t kalman_filter_init(struct KalmanFilter * kalman_filter, 
		uint16_t n_states, uint16_t n_input, uint16_t n_output);

/**
 *  @fn         kalman_filter_step
 *  @brief      Performs a step of the Kalman filter
 *
 *  @params[in] kalman_filter Kalman Filter data structure
 *  @returns    kalman_filter_status
 */
kalman_filter_status kalman_filter_step(struct KalmanFilter *kalman_filter, 
		float32_t *u);

#endif /* KALMAN_H */

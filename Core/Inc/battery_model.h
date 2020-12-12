#ifndef _BATTERY_MODEL_H
#define _BATTERY_MODEL_H

#include <stdint.h>
#include "kalman.h"
#include "battery_luts.h"

struct Cell {
	struct KalmanFilter filter;
	uint32_t current_soc_index;
	float32_t current_state[N_STATES];
};

/**
 *  @fn			calculate_battery_soc	
 *  @brief      Given the actual voltage and current circulating over the
 *  battery it calculates the soc. This function should be 
 *  run periodically with the period used to discretize the time domain model
 *  of the battery.
 *
 *  @params[in]	cell: Cell struct to calculate the current state of charge
 *  @params[in] voltage: Measured voltage of the battery
 *  @params[in] current: Measured circulating current of the cell
 *  @returns   	state of charge of the batter 
 */
float32_t calculate_battery_soc(struct Cell * cell, float32_t voltage, 
								float32_t current);

/**
 *  @fn			init_battery_model 
 *  @brief      Initializes the cell model with a given value of
 *  open_circuit_voltage, this open_circuit_voltage determines the initial
 *  SOC of the battery. 
 *  @note		the arg open_circuit_voltage is assumed to be the measured
 *  voltage of the cell in a state where there is no charge nor discharge of it
 *  during, at least, the last 15 minutes. In case this latter case cannot be
 *  determined, please update the q_data matrix accordingly using the
 *  battery_set_q_data function
 *
 *  @params[in] kalman_filter Kalman Filter data structure
 *  @returns    kalman_filter_status
 */
arm_status init_battery_model(struct Cell * cell, 
		  					  float32_t open_circuit_voltage);
							  

/**
 *	@fn			battery_model_set_q_data
 *	@brief		helper function to set the q_data matrix if calibration of the
 *	model is needed. This matrix represents the process uncertainty of the
 *	model, so with any external perturbance or so, this matrix should be
 *	modified.
 *
 *	@note The idea of this function is to give the user the posibility to update
 *	that matrix during runtime in order to calibrate the filter without the need
 *	to flash the MCU again.
 *	@params[in]	cell Cell data structure to modify
 *	@params[in] q_data float32_t array pointer to new data
 */
void battery_model_set_q_data(struct Cell * cell, const float32_t * q_data);

/**
 *	@fn			battery_model_set_r_data
 *	@brief		helper function to set the r_data matrix if calibration of the
 *	model is needed. This matrix represents the measurement uncertainty of the
 *	sensors, this should be calculated and measured with the current and voltage
 *	sensor noise data from their respective datasheet
 *
 *	@note The idea of this function is to give the user the posibility to update
 *	that matrix during runtime in order to calibrate the filter without the need
 *	to flash the MCU again.
 *	@params[in]	cell Cell data structure to modify
 *	@params[in] q_data float32_t array pointer to new data
 */
void battery_model_set_r_data(struct Cell * cell, const float32_t * r_data);

/**
 *	@fn			battery_model_get_soc
 *	@brief		helper function to get the current state of charge of the
 *	battery model
 *
 *	@params[in]	cell Cell data structure to modify
 *	@returns State of charge of the battery
 */
float32_t battery_model_get_soc(struct Cell * cell);

#endif 

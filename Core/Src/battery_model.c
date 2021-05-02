/**********************************************************************
*
* @file		battery_model.cpp
* @brief	Contains the definitions of the battery model
* @version	v1.00f00
* @date		12. Dec. 2020
* @author	CECARELLI, Federico (fededc88@gmail.com)
*			MOYA, Martin		(moyamartin1@gmail.com)
*			SANTOS, Lucio		(lusho2206@gmail.com)
* @copyright GPL license, all text here must be included in any redistribution.
**********************************************************************/
#include "battery_model.h"
#include "battery_luts.h"
#include "find.h"

static void update_filter_params(struct KalmanFilter * filter, 
								 uint32_t soc_index)
{
	kalman_filter_modify_f_data(filter, F_LUT[soc_index]);
	kalman_filter_modify_g_data(filter, G_LUT[soc_index]);
	kalman_filter_modify_h_data(filter, H_LUT[soc_index]);
	kalman_filter_modify_j_data(filter, &D_LUT[soc_index]);
}

float32_t cell_model_get_soc(struct Cell * cell)
{
    return SOC_LUT[cell->current_soc_index];
}

cell_status init_cell_model(struct Cell * cell, 
						    float32_t open_circuit_voltage)
{
	/* 
	 * At the battery model startup, we are considering we have the OCV (Open
	 * Circuit Voltage) of the battery, so we can determine the
	 * current_soc_index from it
	 */
	cell->current_soc_index = find_closest_value_f32(open_circuit_voltage, 
													 OCV_LUT, SAMPLE_SOCS);

	// Set initial state of the kalman filter
	float32_t initial_state[N_STATES] = {
		(float32_t) 0, (float32_t) 0, SOC_LUT[cell->current_soc_index]
	};

	// Initializes kalman filter with the battery data from the current index
	if(kalman_filter_init(&cell->filter, initial_state, 
                          F_LUT[cell->current_soc_index],
                          G_LUT[cell->current_soc_index], 
					      H_LUT[cell->current_soc_index], 
					      pre_calc_q, pre_calc_p, pre_calc_r, NULL, 
						  &D_LUT[cell->current_soc_index], N_STATES, N_INPUTS, 
                          N_OUTPUTS) == KALMAN_FAILED_INIT){
        return CELL_INIT_FAILED;
    }
    return CELL_INIT_SUCCESS;
}


void battery_cell_set_q_data(struct Cell * cell, const float32_t * q_data)
{
	kalman_filter_modify_q_data(&cell->filter, q_data);
}


void battery_cell_set_r_data(struct Cell * cell, const float32_t * r_data)
{
	kalman_filter_modify_r_data(&cell->filter, r_data);
}

float32_t calculate_cell_soc(struct Cell * cell, float32_t voltage, 
								float32_t current)
{
	kalman_filter_modify_u_data(&cell->filter, &current);
	kalman_filter_step(&cell->filter, &voltage);
	uint32_t new_soc_index = find_closest_value_f32(cell->filter.x_act_data[2], 
													SOC_LUT, SAMPLE_SOCS);
	if(cell->current_soc_index != new_soc_index){
		cell->current_soc_index = new_soc_index;
		update_filter_params(&cell->filter, cell->current_soc_index);
	}
							  
	// returns the current soc of the battery
	return cell->filter.x_act_data[2];
}

float32_t battery_cell_get_soc(struct Cell * cell)
{
	return cell->filter.x_act_data[2];
}

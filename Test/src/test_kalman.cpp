#include "CppUTest/TestHarness.h"
extern "C" {
#include "kalman.h"
#include "kalman_measurements.h"
}


TEST_GROUP(kalman_test)
{
	struct KalmanFilter soc_kalman_filter;
	void setup(){
		kalman_filter_init(&soc_kalman_filter, x_act_data_dummy, 
						   f_mat_data_dummy, g_mat_data_dummy, 
						   h_mat_data_dummy, q_mat_data_dummy,
						   p_mat_actual_data_dummy, r_mat_data_dummy,
						   u_data_dummy, NULL, N_STATES, N_INPUTS, 
						   N_OUTPUTS);
	}

	void teardown(){
	}
};


TEST(kalman_test, kalman_pass_me)
{	
	float32_t measurement_buffer[4];
	for(int i = 0; i < N_SAMPLES; i++){
		DOUBLES_EQUAL(filtered_data[i][0], 
					  soc_kalman_filter.x_act.pData[0], 0.01);
		DOUBLES_EQUAL(filtered_data[i][1],
					  soc_kalman_filter.x_act.pData[2], 0.01);
		memcpy(measurement_buffer, measurements[i], 4*sizeof(float32_t));
		kalman_filter_step(&soc_kalman_filter, measurement_buffer);
	}
}

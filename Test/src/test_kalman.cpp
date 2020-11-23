#include "CppUTest/TestHarness.h"
extern "C" {
#include "kalman.h"
#include "kalman_measurements.h"
}


TEST_GROUP(kalman_test)
{
	struct KalmanFilter soc_kalman_filter;
	void setup(){
		// Actually this should be initialized as in /Core/Src/main.c
		// but cpp-bullshit complains about initialization of structs due to
		// design constraints
		memcpy(soc_kalman_filter.x_act_data, &x_act_data_dummy, 
			   N_STATES*sizeof(float32_t));
		memcpy(soc_kalman_filter.f_mat_data, &f_mat_data_dummy, 
			   N_STATES*N_STATES*sizeof(float32_t));
		memcpy(soc_kalman_filter.g_mat_data, &g_mat_data_dummy, 
			   N_STATES*N_INPUTS*sizeof(float32_t));
		memcpy(soc_kalman_filter.h_mat_data, &h_mat_data_dummy,
			   N_INPUTS*N_STATES*sizeof(float32_t));
		memcpy(soc_kalman_filter.q_mat_data, &q_mat_data_dummy,
			   N_STATES*N_STATES*sizeof(float32_t));
		memcpy(soc_kalman_filter.p_mat_actual_data, &p_mat_actual_data_dummy,
			   N_STATES*N_STATES*sizeof(float32_t));
		memcpy(soc_kalman_filter.r_mat_data, &r_mat_data_dummy, 
			   N_INPUTS*N_INPUTS*sizeof(float32_t));
		memcpy(soc_kalman_filter.u_data, &u_data_dummy, 
			   N_INPUTS*sizeof(float32_t));
		kalman_filter_init(&soc_kalman_filter, 4, 4, 4);
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

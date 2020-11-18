#include "CppUTest/TestHarness.h"
extern "C" {
#include "kalman.h"
#include "kalman_measurements.h"
}

static struct KalmanFilter soc_kalman_filter {
	.x_act_data = 			{0.0, 70.71067812, 500.0, 70.71067812},
	.f_mat_data = 			{1.0, 0.1, 0.0, 0.0,
							  0.0, 1.0, 0.0, 0.0,
							  0.0, 0.0, 1.0, 0.1,
							  0.0, 0.0, 0.0, 1.0},
	.g_mat_data = 			{0.0, 0.0, 0.0, 0.0,
							 0.0, 0.0, 0.0, 0.0,
							 0.0, 0.0, 1.0, 0.0,
							 0.0, 0.0, 0.0, 1.0},
	.p_mat_actual_data = 	{1.0, 0.0, 0.0, 0.0,
							 0.0, 1.0, 0.0, 0.0,
							 0.0, 0.0, 1.0, 0.0,
							 0.0, 0.0, 0.0, 1.0},
	.q_mat_data =			{0.0, 0.0, 0.0, 0.0,		
							 0.0, 0.0, 0.0, 0.0,
							 0.0, 0.0, 0.0, 0.0,
							 0.0, 0.0, 0.0, 0.0},
	.r_mat_data =			{0.2, 0.0, 0.0, 0.0,
							 0.0, 0.2, 0.0, 0.0,
							 0.0, 0.0, 0.2, 0.0,
							 0.0, 0.0, 0.0, 0.2},
	.h_mat_data =			{1.0, 0.0, 0.0, 0.0,
							  0.0, 1.0, 0.0, 0.0,
							  0.0, 0.0, 1.0, 0.0,
							  0.0, 0.0, 0.0, 1.0},
	.u_data =				{0.0, 0.0, -.0495, -.981}
};

TEST_GROUP(kalman_test)
{
	void setUp(){
		uint8_t kalman_filter_is_init = kalman_filter_init(&soc_kalman_filter,
			   											   4, 4, 4);
		CHECK_EQUAL(1, kalman_filter_is_init);
	}

	void tearDown(){

	}
};


TEST (kalman_test, pass_me)
{	
	kalman_filter_status kf_status;
	for(int i = 0; i < NUM_SAMPLES; ++i){
		kf_status = kalman_filter_step(&soc_kalman_filter, 
									   measurements[i]);
		CHECK_EQUAL(filtered_data[i][0], 
					soc_kalman_filter.x_act.pData[0]);
		CHECK_EQUAL(filtered_data[i][1],
					soc_kalman_filter.x_act.pData[2]);
	}
}


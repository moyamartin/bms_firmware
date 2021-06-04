#include "CppUTest/TestHarness.h"
extern "C" {
#include "battery_model.h"
#include "soc_test_data.h"
}

TEST_GROUP(cell_test)
{
	struct Cell cell;
	void setup(){
		init_cell_model(&cell, 4.2f);
	}

	void teardown(){
	}
};


TEST(cell_test, cell_pass_me)
{	
	float32_t actual_soc = cell_model_get_soc(&cell);
	for(int i = 0; i < N_SAMPLES; i++){
		DOUBLES_EQUAL(expected_soc[i], actual_soc, 0.01);
		actual_soc = calculate_cell_soc(&cell, voltage_data[i], 
										current_data[i]);
	}
}

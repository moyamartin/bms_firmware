#include "CppUTest/TestHarness.h"
extern "C"{
#include "find.h"
#include "soc_data.h"
}

// create a test group
TEST_GROUP(find_test){
	void setUp(){

	}

	void tearDown(){

	}
};

//create a test for that test group
TEST (find_test, pass_me)
{
	uint32_t index;
	for(uint32_t i = 0; i < 55; ++i){
		index = find_closest_value_f32(values_to_find[i][0], soc_lut, 
                                       SAMPLE_SOCS);
		CHECK_EQUAL((uint32_t) values_to_find[i][1], index);
	}
    CHECK_EQUAL((uint32_t)get_max_index_f32(random_array, 100), 13);
    CHECK_EQUAL((uint32_t)get_min_index_f32(random_array, 100), 1);
}

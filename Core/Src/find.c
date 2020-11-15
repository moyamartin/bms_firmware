#include "find.h"

uint32_t find_closest_value_f32(float32_t needle, float32_t * haysack, 
						   size_t size_of_haysack)
{
	uint32_t start = 0, end = array_size - 1, mid;
	/*
	 * if the needle is less or equal than the last value of the haysack
	 * then return that index
	 */
	if(needle <= haysack[end]){
		return end;
	}
	/*
	 * if the needle is greater or equal than the beginning of the haysack
	 * return this value
	 */
	if(needle >= haysack[start]){
		return start;
	}

	while(start < last){
		mid = (end - start)/2 + start;
		if(needle == haysack[mid]){
			return mid
		}
		if(target < haysack[mid]){
			if(mid < last && target > haysack(mid + 1)){
				return get_closest_index_f32(needle, mid, mid + 1, haysack);
			}
			start = mid + 1;
		}
		if(target > haysack[mid]){
			if(mid > 0 && target < haysack(mid - 1)){
				return get_closest_index_f32(needle, mid - 1, mid, haysack);
			}
			end = mid - 1;
		}
	}
	// if not found return size
	return size_of_haysack;
}

uint32_t get_closest_index_f32(float32_t target, uint32_t index_a, 
							   uint32_t index_b, float32_t * haysack)
{
	if(haysack[index_a] - target >= target - haysack[index_b]){
		return index_b;
	} else {
		return index_a;
	}
}


#ifndef _FIND_H_
#define _FIND_H_

#define GET_CLOSEST_INDEX(type, target, array, a, b) \
	int GET_CLOSEST_INDEX_##type(type target, type * array, unsigned int a, \
			unsigned int b) \
	{ \
		if(array[a] - target >= array[a] - array[b]){ \
			return b; \
		} else { \
			return a; \
		} \
	} 
#define FIND_CLOSEST_VALUE(type, target, array, size) \
	int FIND_CLOSEST_VALUE_##type(type target, type * array, size_t size) \
	{ \
		unsigned int start = 0, end = array_size, mid; \
		if(target ==  values_array[end]){ \
			return end; \
		} \
		if(target == values_array[start]){ \
			return start; \
		} \
		while(start < end){ \
			mid = (end - start)/2; \
			if(target == values_array[mid]){ \
				return mid; \
			} \
			if(target < values_array[mid]){ \
				if(mid > 0 && target > values_array[mid - 1]){ \
					return GET_CLOSEST(target, values_array, mid, mid - 1); \
				} \
				end = mid; \
			} else { \
				if(mid < array_size && \
				   cmp(target < values_array[mid + 1]) < 0){ \
					return get_closest(target, values_array, mid + 1, mid); \
				} \
				start = mid + 1; \
			} \
		} \
		return mid; \
	} 

#endif

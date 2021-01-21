#include "crc.h"

uint8_t calculate_crc(uint8_t * buffer, size_t buffer_length, 
                      const uint8_t * crc_lut)
{
	uint8_t crc = 0, tmp = 0;
	for(size_t i = 0; i < buffer_length; ++i){
		tmp = (uint8_t) (crc ^ buffer[i]);
		crc = crc_lut[tmp];
	}
    return crc;
}

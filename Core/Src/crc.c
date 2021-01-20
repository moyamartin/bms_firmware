#include "crc.h"

uint8_t calculate_crc(uint8_t * buffer, size_t buffer_length, 
                      const uint8_t * crc_lut)
{
    uint8_t crc = 0;
    for(int i = 0; i < buffer_length; ++i){
       crc = crc_lut[crc ^ buffer[i]];
    }
    return crc;
}

#include "definitions.h"

void swap_16b(uint16_t * source)
{
	*source = ((*source << 8) & 0xFF00) | ((*source >> 8) & 0x00FF);
}

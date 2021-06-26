#include "battery_pack.h"

pack_status init_battery_pack(struct Pack * battery_pack, 
                              const float32_t * v_cells)
{
    for(int i = 0; i < SERIES_CELLS; ++i){
        if(init_cell_model(&(battery_pack->cells[i]), v_cells[i]/1000.0f) == 
                CELL_INIT_FAILED){
            return PACK_INIT_FAILED;
        }
    }
    battery_pack->initialized = 1;
    return PACK_INIT_SUCCESS;
}

void calc_battery_pack_soc(struct Pack * battery_pack, 
                           const float32_t * v_cells, const float32_t i_pack)
{
    for(int i = 0; i < SERIES_CELLS; ++i){
        calculate_cell_soc(&(battery_pack->cells[i]), v_cells[i]/1000.0f, 
                             i_pack/((float32_t)PARALLEL_CELLS));
    }
}

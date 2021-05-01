#ifndef BATTERY_PACK_H
#define BATTERY_PACK_H


#define SERIES_CELLS 6
#define PARALLEL_CELLS 3

#include "battery_model.h"
#include "arm_math.h"

typedef enum
{
    PACK_INIT_SUCCESS = 0,
    PACK_INIT_FAILED,
} pack_status;

struct Pack {
    struct Cell cells[SERIES_CELLS];
    int initialized;
};

/**
 *  @fn			init_battery_pack
 *  @brief      Given an array of voltages with a similar length to SERIES_CELLS
 *              this function will initialize all the individual cells of a
 *              battery pack.
 *  @params[in]	v_cells: float32_t array with the same length as SERIES_CELLS.
 *                       The library assumes that the developer must set an
 *                       array with the same length as SERIES_CELLS, otherwise
 *                       the BMS will crash
 *  that is interpreted to be the open circuit voltage of the battery.
 *  @returns    pack_status
 */
pack_status init_battery_pack(struct Pack  * pack, const float32_t * v_cells);

/**
 *  @fn			calc_battery_pack_sock
 *  @brief      Given an array of voltages with a similar length to SERIES_CELLS
 *              and the actuall circulating current value, this will calculate
 *              the State of charge of each cell
 *  @params[in]	v_cells: float32_t array with the same length as SERIES_CELLS
 *  @params[in] i_pack: float32_t value of the actual circulating current
 *              through the battery pack
 */
void calc_battery_pack_sock(struct Pack * battery_pack, 
                            const float32_t * v_cells, const float32_t i_pack);

#endif /* battery_pack.h */

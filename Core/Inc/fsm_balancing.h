/**
*****************************************************************************
 *
 * @file	fsm_balancing.h
 * @brief 	Finite State Machine of the battery equalization
 * @version	v1.00f00
 * @date	01, May. 2020
 * @author	CECARELLI, Federico (fededc88@gmail.com),
 * 			MOYA, Martin		(moyamartin1@gmail.com),
 * 			SANTOS, Lucio		(lusho2206@gmail.com)
 * @copyright GPL license, all text here must be included in any redistribution
 ****************************************************************************
 */
#ifndef _FSM_BALANCER_H
#define _FSM_BALANCER_H

#include "DataTypes.h"
#include "fsm.h"
#include "battery_pack.h"

// Balancer object structure
typedef struct
{
    struct Pack * pack;
    struct BQ76 * device;
} FSM_BALANC;

/**
 *  @brief      Given an Pack structure which contains the SoC of #SERIES_CELLS number of
 *              cells. Returns 1 if the diference between the SoC of the most and the least charged cell in the pack is greater than 2%. 
 *                        
 *  @param[in]	pack: struct Pack to detect unbalance.
 */
BOOL IsUnbalanced(struct Pack *pack);

/**
 *  @brief      Given an Pack structure which contains the SoC of #SERIES_CELLS number of
 *              cells, this will return an uint8_t whose internal bit correspond to the
 *              ByPass Transistors to be activated for balancing purposes              
 *  @param[in]	pack: struct Pack to set ByPass Transistors.
 */
uint8_t Balance_transistors(struct Pack *pack);



//state machine event functions
EVENT_DECLARE(BALANC_CV_Charging,NoEventData)
EVENT_DECLARE(BALANC_NOT_CV_Charging,NoEventData)
EVENT_DECLARE(BALANC_RUN_CURRENT_STATE, NoEventData)

#endif // _FSM_BALANCER_H

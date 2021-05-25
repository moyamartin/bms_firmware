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
    INT DummyInt;
} FSM_BALANC;

// Event data structure
typedef struct
{
    INT DummyInt;
} FSM_BALANC_Data;

/**
 *  @fn			IsDisbalanced
 *  @brief      Given an Pack structure which contains the SoC of #SERIES_CELLS number of
 *              cells, this will return 1 if the pack is balanced.              
 *  @params[in]	pack: struct Pack to detect disbalance.
 */
bool IsDisbalanced(Struct *Pack pack);

/**
 *  @fn			Balance_transistors
 *  @brief      Given an Pack structure which contains the SoC of #SERIES_CELLS number of
 *              cells, this will return an uint8_t whose internal bit correspond to the
 *              ByPass Transistors to be activated for balancing purposes              
 *  @params[in]	pack: struct Pack to set ByPass Transistors.
 */
uint8_t Balance_transistors(Struct *Pack pack);



//state machine event functions
EVENT_DECLARE(BALANC_CV_Charging,NoEventData)
EVENT_DECLARE(BALANC_NOT_CV_Charging,NoEventData)
EVENT_DECLARE(BALANC_BALANCED_PACK,NoEventData)
EVENT_DECLARE(BALANC_UNBALANCED,NoEventData)


#endif // _FSM_BALANCER_H

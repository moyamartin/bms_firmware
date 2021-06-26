#ifndef _FSM_CHARGER_H
#define _FSM_CHARGER_H

#include "fsm.h"
#include "arm_math.h"

#define CHRG_ENABLED 0x01
#define CHRG_STAT_1 0x02
#define CHRG_STAT_2 0x04
#define CHRG_PG     0x08
#define V_LOWV 20.0f
#define V_RECH 21.0f

enum ChrgStates
{
    CHRG_IDLE,
    CHRG_START,
    CHRG_PRECHARGE,
    CHRG_CC,
    CHRG_CV,
};

/**
 * @struct  Charger
 * @brief   This struct hold
 */
typedef struct
{
    uint8_t current_status_flags;   /*<- current_status_flags */
    float32_t current_pack_voltage; /*<- current pack voltage */
    float32_t current_pack_soc;     /*<- current pack soc */
    float32_t current_pack_cur;     /*<- current pack circulating current */
} Charger;

typedef struct
{
   uint8_t status_flags; 
   float32_t pack_voltage;
   float32_t pack_soc;
   float32_t pack_cur;
} ChargerData;

/**
 * Declare the private instance of Charger state machine
 */
EVENT_DECLARE(CHRG_start_charge, ChargerData)


#endif /* fsm_charger.h */

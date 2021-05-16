#include "fsm.h"
#include "fsm_charger.h"
#include "logging.h"
#include <assert.h>

enum ChrgStates
{
    CHRG_IDLE,
    CHRG_START,
    CHRG_PRECHARGE,
    CHRG_CC,
    CHRG_CV,
};

STATE_DECLARE(idle)
STATE_DECLARE(start)
STATE_DECLARE(precharge)
STATE_DECLARE(cc)
STATE_DECLARE(cv)

// Charger state map to define state function order
// This defines
// static const SM_StateStruct _smName_##StateMap[] = {
BEGIN_STATE_MAP(Charger)
    STATE_MAP_ENTRY(idle)      
    STATE_MAP_ENTRY(start)     
    STATE_MAP_ENTRY(precharge) 
    STATE_MAP_ENTRY(cc)        
    STATE_MAP_ENTRY(cv)
END_STATE_MAP(Charger)
// };
// static const SM_StateMachineConst _smName_##Const = { #_smName_,
// (sizeof(_smName_##StateMap)/sizeof(_smName_##StateMap[0])),
// _smName_##StateMap, Null};

EVENT_DEFINE(CHRG_start_charge, ChargerData)
{
    BEGIN_TRANSITION_MAP
        TRANSITION_MAP_ENTRY(CHRG_START)        // CHRG_IDLE
        TRANSITION_MAP_ENTRY(EVENT_IGNORED)      // CHRG_START
        TRANSITION_MAP_ENTRY(CANNOT_HAPPEN)     // CHRG_PRECHARGE
        TRANSITION_MAP_ENTRY(CANNOT_HAPPEN)     // CHRG_CC
        TRANSITION_MAP_ENTRY(CANNOT_HAPPEN)     // CHRG_CV
    END_TRANSITION_MAP(Charger, pEventData)
}

STATE_DEFINE(idle)
{
    _DEBUG("CHRG IDLE");
}

STATE_DEFINE(start)
{
    _DEBUG("CHRG START");
    _DEBUG("Evaluating charge phase");

    // Assert that the data structure pased to this state is valid
    assert(pEventData != NULL);
    
    Charger * pInstance = SM_GetInstance(Charger);
    ChargerData * charger_data = (ChargerData *) pEventData;
    pInstance->current_status_flags = charger_data->status_flags;

    if(charger_data->pack_soc == 1){
        _DEBUG("CHRG NOT READY");
        SM_InternalEvent(CHRG_IDLE, NULL);
    } else {
        // The next phase charge will depend on the pack voltage
        if(charger_data->pack_voltage < V_LOWV){
            // In this case goes to precharge mode
            SM_InternalEvent(CHRG_PRECHARGE, NULL);
        } else if(charger_data-> pack_voltage < V_RECH){
            // In this case goes to CC mode
            SM_InternalEvent(CHRG_CC, NULL);
        } else {
            // In this case goes to CV mode
            SM_InternalEvent(CHRG_CV, NULL);
        } 
    }
}

STATE_DEFINE(precharge)
{
    _DEBUG("CHRG PRECHARGING");
    
    ChargerData * charger_data = (ChargerData *) pEventData;
    Charger * pInstance = SM_GetInstance(Charger);
    pInstance->current_status_flags = ((ChargerData*) pEventData)->status_flags;
    if(!(charger_data->status_flags & CHRG_ENABLED) || 
            (!(charger_data->status_flags && CHRG_STAT_2) 
             && !(charger_data->status_flags && CHRG_STAT_1))){
        SM_InternalEvent(CHRG_IDLE, NULL);
    } else if(charger_data->pack_voltage > V_LOWV){
        SM_InternalEvent(CHRG_CC, NULL);
    }
}

STATE_DEFINE(cc)
{
    _DEBUG("CHRG CC");
    ChargerData * charger_data = (ChargerData *) pEventData;
    if(charger_data->pack_voltage > V_RECH){
        SM_InternalEvent(CHRG_CV, NULL);
    }
}

STATE_DEFINE(cv)
{
    _DEBUG("CHRG CV");
    ChargerData * charger_data = (ChargerData *) pEventData;
    if(charger_data->pack_soc == 1){
        SM_InternalEvent(CHRG_IDLE, NULL);
    }
}

/**
 ******************************************************************************
 * @file           : fsm_balancing.c
 * @brief          : Finite State Machine of Battery balancing
 ******************************************************************************

 */

#DEFINE UNBALANCED_THRESHOLD (0.02f)

#include "fsm_balancing.h"
#include "fsm.h"
#include <stdio.h>


bool IsUnbalanced(Struct *Pack pack){
    float32_t buffer[SERIES_CELLS];
    uint32_t max_index,min_index;
     for(int i = 0; i < SERIES_CELLS; ++i){
         buffer[i]=battery_model_get_soc(&(pack->cells[i]))
     }


    max_index=get_max_index_f32(buffer,SERIES_CELLS);
    min_index=get_if_index_f32(buffer,SERIES_CELLS);

    /*Comparison between the most and the least charged cells if this difference is 
    bigger than UNBALANCED_THRESHOLD the pack is declared unbalanced*/
    if ((buffer[max_index]-UNBALANCED_THRESHOLD)>buffer[min_index]){
        return 1;
    } 
	else{
        return 0;
    }
};


uint8_t Balance_transistors(Struct *Pack pack){
    float32_t buffer[SERIES_CELLS];
    uint32_t min_index;
    uint8_t transistors=0;
    
     for(int i = 0; i < SERIES_CELLS; ++i){
         buffer[i]=battery_model_get_soc(&(pack->cells[i]))
     }

    min_index=get_if_index_f32(buffer,SERIES_CELLS);
    
        for(int i = 0; i < SERIES_CELLS; ++i){
            if (buffer[min_index]+UNBALANCE_THRESHOLD<buffer[i]){
                		
			transistors = transistors | 1;
		    }

		    if (i < SERIES_CELLS-1) {
			transistors = transistors << 1;
		    }

        }
    }
return transistors

};

// State enumeration order must match the order of state
//method entries in the state map
enum States {
    BALANC_IDLE, /*!< Balancer is in idle state while waiting for CV charging process */
    BALANC_DETEC, /*!<Balancer is detecting whether or not the pack is balanced */
    BALANC_EQ, /*!< Balancer is setting Bypass transistors to the pack */
    BALANC_MAX_STATES/*!< Balancer MAX Number of States */
};

// State machine state functions
STATE_DECLARE(Idle,NoEventData)
STATE_DECLARE(Detecting,NoEventData)
STATE_DECLARE(Equalizing,NoEventData)


BEGIN_STATE_MAP(FSM_BALANC)
    STATE_MAP_ENTRY(ST_Idle)
    STATE_MAP_ENTRY(ST_Detecting)
    STATE_MAP_ENTRY(ST_Equalizing)
END_STATE_MAP(FSM_BALANC)

// CV Charging In Progress external event
EVENT_DEFINE(BALANC_CV_Charging, NoEventData)
{
    // Given the CV Charging In Progress event, transition to a new state based upon 
    // the current state of the state machine
    BEGIN_TRANSITION_MAP                        // - Current State -
        TRANSITION_MAP_ENTRY(BALANC_DETEC)      // ST_Idle       
        TRANSITION_MAP_ENTRY(EVENT_IGNORED)     // ST_Detecting       
        TRANSITION_MAP_ENTRY(EVENT_IGNORED)     // ST_Balancing     
    END_TRANSITION_MAP(FSM_BALANC, pEventData)
}

// CV Charging Off external event
EVENT_DEFINE(BALANC_NOT_CV_Charging, NoEventData)
{
    // Given the CV Charging Off event, transition to a new state based upon 
    // the current state of the state machine
    BEGIN_TRANSITION_MAP                          // - Current State -
        TRANSITION_MAP_ENTRY(BALANC_IDLE)         // ST_Idle       
        TRANSITION_MAP_ENTRY(BALANC_IDLE)         // ST_Detecting       
        TRANSITION_MAP_ENTRY(BALANC_IDLE)         // ST_Balancing     
    END_TRANSITION_MAP(FSM_BALANC, pEventData)
}

 //Event defined to run current state
EVENT_DEFINE(BALANC_RUN_CURRENT_STATE, NoEventData)
{
    // Given NoSpecific event, run the current event 
    // the current state of the state machine
    BEGIN_TRANSITION_MAP                          // - Current State -
        TRANSITION_MAP_ENTRY(BALANC_IDLE)       // ST_Idle       
        TRANSITION_MAP_ENTRY(BALANC_DETEC)        // ST_Detecting       
        TRANSITION_MAP_ENTRY(BALANC_EQ)        // ST_Balancing     
    END_TRANSITION_MAP(FSM_BALANC, pEventData)
}




// State machine sits here when battery is not charging
STATE_DEFINE(Idle, NoEventData)
{
    printf("%s ST_Idle\n", self->name);
}


// When the Charger is in CV charging process starts checking Unbalanced Cells
STATE_DEFINE(Detecting, NoEventData)
{
    // Get pointer to the instance data 
    FSM_BALANC* pInstance = SM_GetInstance(FSM_BALANC);
    
    
    printf("%s ST_Detecting: \n", self->name);

    if (IsUnbalanced(pEventData->pack)){
        SM_InternalEvent(BALANC_EQ, NULL);
    }

}

// Sets the Balance transistors output of the BQ76
STATE_DEFINE(Equalizing, NoEventData)
{
    // Get pointer to the instance data
    FSM_BALANC* pInstance = SM_GetInstance(FSM_BALANC);
    
    

    bq76_set_balancing_output(pEventData->device, 
                              Balance_transistors(pEventData->pack))
    
    printf("%s ST_Equalizing: \n", self->name);

        if (!IsUnbalanced(pEventData->pack)){
        SM_InternalEvent(BALANC_DETEC, NULL);
        }
}




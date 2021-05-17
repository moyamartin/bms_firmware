/**
 ******************************************************************************
 * @file           : fsm_balancing.c
 * @brief          : Finite State Machine of Battery balancing
 ******************************************************************************

 */

#include "fsm_balancing.h"
#include "fsm.h"
#include <stdio.h>

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

// Pack Balanced external event
EVENT_DEFINE(BALANC_BALANCED_PACK, NoEventData)
{
    // Given the Balanced Pack event, transition to a new state based upon 
    // the current state of the state machine
    BEGIN_TRANSITION_MAP                          // - Current State -
        TRANSITION_MAP_ENTRY(EVENT_IGNORED)       // ST_Idle       
        TRANSITION_MAP_ENTRY(BALANC_DETEC)        // ST_Detecting       
        TRANSITION_MAP_ENTRY(BALANC_DETEC)        // ST_Balancing     
    END_TRANSITION_MAP(FSM_BALANC, pEventData)
}


// Pack unbalanced external event
EVENT_DEFINE(BALANC_UNBALANCED, NoEventData)
{
    // Given the Balanced Pack event, transition to a new state based upon 
    // the current state of the state machine
    BEGIN_TRANSITION_MAP                          // - Current State -
        TRANSITION_MAP_ENTRY(EVENT_IGNORED)       // ST_Idle       
        TRANSITION_MAP_ENTRY(BALANC_EQ)           // ST_Detecting       
        TRANSITION_MAP_ENTRY(BALANC_EQ)           // ST_Balancing     
    END_TRANSITION_MAP(FSM_BALANC, pEventData)
}




// State machine sits here when motor is not running
STATE_DEFINE(Idle, NoEventData)
{
    printf("%s ST_Idle\n", self->name);
}


// Start the motor going
STATE_DEFINE(Detecting, NoEventData)
{
    // Get pointer to the instance data and update currentSpeed
    //FSM_BALANC* pInstance = SM_GetInstance(FSM_BALANC);
    
    // Set initial motor speed processing here
    printf("%s ST_Detecting: \n", self->name);
}

// Changes the motor speed once the motor is moving
STATE_DEFINE(Equalizing, NoEventData)
{
    // Get pointer to the instance data and update currentSpeed
    //FSM_BALANC* pInstance = SM_GetInstance(FSM_BALANC);

    // Perform the change motor speed here
    printf("%s ST_Equalizing: \n", self->name);
}


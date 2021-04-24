#include "fsm.h"
#include <assert.h>
#include <stddef.h>
#include <stdlib.h>

void _SM_ExternalEvent(SM_StateMachine* self, 
                       const SM_StateMachineConst* selfConst, uint8_t newState, 
                       void* pEventData)
{
    // Check if the new state is ignore
    if (newState == EVENT_IGNORED) 
    {
        // Just delete the event data, if any
        if (pEventData){
            free(pEventData);
        }
    } else {
        // TODO - capture software lock here for thread-safety if necessary

        // Generate the event 
        _SM_InternalEvent(self, newState, pEventData);

        // Execute state machine based on type of state map defined
        if (selfConst->stateMap)
            _SM_StateEngine(self, selfConst);
        else
            _SM_StateEngineEx(self, selfConst);

        // TODO - release software lock here 
    }
}

void _SM_InternalEvent(SM_StateMachine* self, uint8_t newState, 
                       void* pEventData)
{
    assert(self);

    self->pEventData = pEventData;
    self->eventGenerated = 1;
    self->newState = newState;
}

void _SM_StateEngine(SM_StateMachine* self, 
                     const SM_StateMachineConst* selfConst)
{
    void* pDataTemp = NULL;

    assert(self);
    assert(selfConst);

    // While events are being generated keep executing states
    while (self->eventGenerated)
    {
        // Error check that the new state is valid before proceeding
        assert(self->newState < selfConst->maxStates);

        // Get the pointers from the state map
        SM_StateFunc state = selfConst->stateMap[self->newState].pStateFunc;

        // Copy of event data pointer
        pDataTemp = self->pEventData;

        // Event data used up, reset the pointer
        self->pEventData = NULL;

        // Event used up, reset the flag
        self->eventGenerated = 0;

        // Switch to the new current state
        self->currentState = self->newState;

        // Execute the state action passing in event data
        assert(state != NULL);
        state(self, pDataTemp);

        // If event data was used, then delete it
        if (pDataTemp)
        {
            free(pDataTemp);
            pDataTemp = NULL;
        }
    }
}

void _SM_StateEngineEx(SM_StateMachine* self, const SM_StateMachineConst* selfConst)
{
    uint8_t guardResult = 1;
    void* pDataTemp = NULL;

    assert(self);
    assert(selfConst);

    // While events are being generated keep executing states
    while (self->eventGenerated)
    {
        // Error check that the new state is valid before proceeding
        assert(self->newState < selfConst->maxStates);

        // Get the pointers from the extended state map
        SM_StateFunc state = selfConst->stateMapEx[self->newState].pStateFunc;
        SM_GuardFunc guard = selfConst->stateMapEx[self->newState].pGuardFunc;
        SM_EntryFunc entry = selfConst->stateMapEx[self->newState].pEntryFunc;
        SM_ExitFunc exit = selfConst->stateMapEx[self->currentState].pExitFunc;

        // Copy of event data pointer
        pDataTemp = self->pEventData;

        // Event data used up, reset the pointer
        self->pEventData = NULL;

        // Event used up, reset the flag
        self->eventGenerated = 0;

        // Execute the guard condition
        if (guard != NULL){
            guardResult = guard(self, pDataTemp);
        }

        // If the guard condition succeeds
        if (guardResult){
            // Transitioning to a new state?
            if (self->newState != self->currentState){
                // Execute the state exit action on current state before 
                // switching to new state
                if (exit != NULL)
                    exit(self);

                // Execute the state entry action on the new state
                if (entry != NULL)
                    entry(self, pDataTemp);

                // Ensure exit/entry actions didn't call SM_InternalEvent by 
                // accident 
                assert(self->eventGenerated == 0);
            }

            // Switch to the new current state
            self->currentState = self->newState;

            // Execute the state action passing in event data
            assert(state != NULL);
            state(self, pDataTemp);
        }

        // If event data was used, then delete it
        if (pDataTemp)
        {
            free(pDataTemp);
            pDataTemp = NULL;
        }
    }
}

/******************************************************************************
 *
 * @file		fsm.h
 * @brief		Contains definitions of functions and data structures for 
 *              a finite-state machine (FSM) framework in C
 * @version		v1.00f00
 * @date		24. April 2020
 * @author		CECCARELLI, Federico    (fededc88@gmail.com)
 *              MOYA, Martin            (moyamartin1@gmail.com)
 *              SANTOS, Lucio           (lusho2206@gmail.com)
 * @copyright	GPL license
 *

 *******************************************************************************/

#ifndef FSM_H
#define FSM_H

#include <stdint.h>
#include <assert.h>



enum { EVENT_IGNORED = 0xFE, CANNOT_HAPPEN = 0xFF };

typedef void NoEventData;

/**
 * @struct  SM_StateMachineConst
 * @brief   This struct defines the const data for a FSM
 */
typedef struct
{
    const char* name;
    const uint8_t maxStates;
    const struct SM_StateStruct* stateMap;
    const struct SM_StateStructEx* stateMapEx;
} SM_StateMachineConst;

/**
 *  @struct SM_StateMachine
 *  @brief  defines a data structure for a FSM
 */
typedef struct
{
    const char* name;
    void* pInstance;
    uint8_t newState;
    uint8_t currentState;
    uint8_t eventGenerated;
    void* pEventData;
} SM_StateMachine;

// Generic state function signatures

/**
 *  @func SM_StateFunc 
 *  @brief Defines a signature function for a state
 */
typedef void (*SM_StateFunc)(SM_StateMachine* self, void* pEventData);

/**
 *  @func SM_GuardFunc
 *  @brief
 */
typedef uint8_t (*SM_GuardFunc)(SM_StateMachine* self, void* pEventData);

/**
 *  @func SM_EntryFunc
 *  @brief
 */
typedef void (*SM_EntryFunc)(SM_StateMachine* self, void* pEventData);

/**
 *  @func SM_ExitFunc
 *  @brief
 */
typedef void (*SM_ExitFunc)(SM_StateMachine* self);

typedef struct SM_StateStruct
{
    SM_StateFunc pStateFunc;
} SM_StateStruct;


typedef struct SM_StateStructEx
{
    SM_StateFunc pStateFunc;
    SM_GuardFunc pGuardFunc;
    SM_EntryFunc pEntryFunc;
    SM_ExitFunc pExitFunc;
} SM_StateStructEx;

// Public functions
#define SM_Event(_smName_, _eventFunc_, _eventData_) \
    _eventFunc_(&_smName_##Obj, _eventData_)

// Protected functions
#define SM_InternalEvent(_newState_, _eventData_) \
    _SM_InternalEvent(self, _newState_, _eventData_)
#define SM_GetInstance(_instance_) \
    (_instance_*)(self->pInstance);

/**
 *  @func   _SM_ExternalEvent
 *  @brief  private function that generates an external event. It has to be
 *          called once per external event to start executing the state machine
 */
void _SM_ExternalEvent(SM_StateMachine* self, 
                       const SM_StateMachineConst* selfConst, 
                       uint8_t newState, void* pEventData);
/**
 *  @func   _SM_InternalEvent
 *  @brief  private function that generates an internal event. It has to be
 *          called within a state function to transition to a new state
 */
void _SM_InternalEvent(SM_StateMachine* self, uint8_t newState, 
                       void* pEventData);

/**
 * @func    _SM_StateEngine
 * @brief   This functions makes the fsm engine to execute the state machin
 *          states
 */
void _SM_StateEngine(SM_StateMachine* self, 
                     const SM_StateMachineConst* selfConst);

/**
 * @func    _SM_StateEngineEx
 * @brief   The state engine executes the extended state machine states
 */
void _SM_StateEngineEx(SM_StateMachine* self, 
                       const SM_StateMachineConst* selfConst);

#define SM_DECLARE(_smName_) \
    extern SM_StateMachine _smName_##Obj; 

#define SM_DEFINE(_smName_, _instance_) \
    SM_StateMachine _smName_##Obj = { #_smName_, _instance_, \
        0, 0, 0, 0 }; 

#define SM_GET_CURRENT_STATE(_smName_) \
    _smName_##Obj.currentState

#define EVENT_DECLARE(_eventFunc_, _eventData_) \
    void _eventFunc_(SM_StateMachine* self, _eventData_* pEventData);

#define EVENT_DEFINE(_eventFunc_, _eventData_) \
    void _eventFunc_(SM_StateMachine* self, _eventData_* pEventData)

#define STATE_DECLARE(_stateFunc_) \
    static void ST_##_stateFunc_(SM_StateMachine* self, void* pEventData);

#define STATE_DEFINE(_stateFunc_) \
    static void ST_##_stateFunc_(SM_StateMachine* self, void* pEventData)

#define GUARD_DECLARE(_guardFunc_, _eventData_) \
    static uint8_t GD_##_guardFunc_(SM_StateMachine* self, _eventData_* pEventData);

#define GUARD_DEFINE(_guardFunc_, _eventData_) \
    static uint8_t GD_##_guardFunc_(SM_StateMachine* self, _eventData_* pEventData)

#define ENTRY_DECLARE(_entryFunc_, _eventData_) \
    static void EN_##_entryFunc_(SM_StateMachine* self, _eventData_* pEventData);

#define ENTRY_DEFINE(_entryFunc_, _eventData_) \
    static void EN_##_entryFunc_(SM_StateMachine* self, _eventData_* pEventData)

#define EXIT_DECLARE(_exitFunc_) \
    static void EX_##_exitFunc_(SM_StateMachine* self);

#define EXIT_DEFINE(_exitFunc_) \
    static void EX_##_exitFunc_(SM_StateMachine* self)

#define BEGIN_STATE_MAP(_smName_) \
    static const SM_StateStruct _smName_##StateMap[] = { 

#define STATE_MAP_ENTRY(_stateFunc_) \
    { ST_##_stateFunc_ },

#define END_STATE_MAP(_smName_) \
    }; \
    static const SM_StateMachineConst _smName_##Const = { #_smName_, \
        (sizeof(_smName_##StateMap)/sizeof(_smName_##StateMap[0])), \
        _smName_##StateMap, NULL };

#define BEGIN_STATE_MAP_EX(_smName_) \
    static const SM_StateStructEx _smName_##StateMap[] = { 

#define STATE_MAP_ENTRY_EX(_stateFunc_) \
    { _stateFunc_, NULL, NULL, NULL },

#define STATE_MAP_ENTRY_ALL_EX(_stateFunc_, _guardFunc_, _entryFunc_, _exitFunc_) \
    { _stateFunc_, _guardFunc_, _entryFunc_, _exitFunc_ },

#define END_STATE_MAP_EX(_smName_) \
    }; \
    static const SM_StateMachineConst _smName_##Const = { #_smName_, \
        (sizeof(_smName_##StateMap)/sizeof(_smName_##StateMap[0])), \
        NULL, _smName_##StateMap };

#define BEGIN_TRANSITION_MAP \
    static const uint8_t TRANSITIONS[] = { \

#define TRANSITION_MAP_ENTRY(_entry_) \
    _entry_,

#define END_TRANSITION_MAP(_smName_, _eventData_) \
    }; \
    _SM_ExternalEvent(self, &_smName_##Const, TRANSITIONS[self->currentState], _eventData_); \
    assert((sizeof(TRANSITIONS)/sizeof(uint8_t)) == (sizeof(_smName_##StateMap)/sizeof(_smName_##StateMap[0])));

#endif /* fms.h */

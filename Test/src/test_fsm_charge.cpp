#include "CppUTest/TestHarness.h"
extern "C"{
#include "fsm_charger.h"
#include "fsm.h"
}

// create a test group
TEST_GROUP(fsm_charger_start)
{
    void setUp()
    {

    }

    void tearDown()
    {

    }
};

TEST(fsm_charger_start, pass_me)
{
    Charger charger1;
    ChargerData data; 
    SM_DEFINE(Charger1SM, &charger1)
    data.pack_voltage = 19.0f;
    data.status_flags = CHRG_ENABLED;
    SM_Event(Charger1SM, CHRG_start_charge, &data);
    uint8_t current_state = SM_GET_CURRENT_STATE(Charger1SM);
    CHECK_EQUAL(current_state, CHRG_PRECHARGE);
}


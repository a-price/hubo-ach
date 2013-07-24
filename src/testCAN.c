#include "hubo-daemon.h"

int main(int argc, char **argv) 
{
    // Initialize Hubo Structs
    hubo_ref_t H_ref;
    hubo_ref_t H_ref_filt;
    hubo_board_cmd_t H_cmd;
    hubo_state_t H_state;
    hubo_param_t H_param;
    hubo_virtual_t H_virtual;
    memset( &H_ref,   0, sizeof(H_ref));
    memset( &H_ref_filt, 0, sizeof(H_ref_filt));
    memset( &H_cmd,  0, sizeof(H_cmd));
    memset( &H_state, 0, sizeof(H_state));
    memset( &H_param, 0, sizeof(H_param));
    memset( &H_virtual, 0, sizeof(H_virtual));

    // set joint parameters for Hubo
    setJointParams(&H_param, &H_state);
    setSensorDefaults(&H_param);

    // open hubo reference
    int r = ach_open(&chan_hubo_ref, HUBO_CHAN_REF_NAME, NULL);
    hubo_assert( ACH_OK == r, __LINE__ );

    // open hubo state
    r = ach_open(&chan_hubo_state, HUBO_CHAN_STATE_NAME, NULL);
    hubo_assert( ACH_OK == r, __LINE__ );

    // initilize control channel
    r = ach_open(&chan_hubo_board_cmd, HUBO_CHAN_BOARD_CMD_NAME, NULL);
    hubo_assert( ACH_OK == r, __LINE__ );

    // open to sim chan
    r = ach_open(&chan_hubo_to_sim, HUBO_CHAN_VIRTUAL_TO_SIM_NAME, NULL);
    hubo_assert( ACH_OK == r, __LINE__ );

    // open to sim chan
    r = ach_open(&chan_hubo_from_sim, HUBO_CHAN_VIRTUAL_FROM_SIM_NAME, NULL);
    hubo_assert( ACH_OK == r, __LINE__ );

    openAllCAN( HUBO_VIRTUAL_MODE_NONE );
    //ach_put(&chan_hubo_ref, &H_ref, sizeof(H_ref));
    //ach_put(&chan_hubo_board_cmd, &H_cmd, sizeof(H_cmd));
    //ach_put(&chan_hubo_state, &H_state, sizeof(H_state));
    //ach_put(&chan_hubo_to_sim, &H_virtual, sizeof(H_virtual));


    // Run our tests here
    struct can_frame frame;
    hGotoLimitAndGoOffset(RF1, &H_ref, &H_ref_filt, &H_param, &H_state, &frame, 1);
    
    return 0;
}

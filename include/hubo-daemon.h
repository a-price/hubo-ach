/*

	A simple header file to include in hubo-daemon.c
	and hubo-daemonizer.c

	Author: M.X. Grey ( mxgrey@gatech.edu )

*/

#ifndef HUBO_DAEMON_H
#define HUBO_DAEMON_H

#include <sys/types.h>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <net/if.h>

#include <linux/can.h>
#include <linux/can/raw.h>
#include <string.h>
#include <stdio.h>
#include <math.h>

// for timer
#include <time.h>
#include <sched.h>
#include <sys/io.h>
#include <unistd.h>

// for RT
#include <stdlib.h>
#include <sys/mman.h>

// for hubo
#include "hubo.h"
#include "hubo/canID.h"
#include "hubo-daemonID.h"
#include "hubo-jointparams.h"

// Check out which CAN API to use
#ifdef HUBO_CONFIG_ESD
#include "hubo/hubo-esdcan.h"
#else
#include "hubo/hubo-socketcan.h"
#endif

// for ach
#include <errno.h>
#include <fcntl.h>
#include <assert.h>
#include <unistd.h>
#include <pthread.h>
#include <ctype.h>
#include <stdbool.h>
#include <math.h>
#include <inttypes.h>
#include "ach.h"


/* At time of writing, these constants are not defined in the headers */
#ifndef PF_CAN
#define PF_CAN 29
#endif

#ifndef AF_CAN
#define AF_CAN PF_CAN
#endif

// Timing info
#define NSEC_PER_SEC    1000000000

#define slowLoopSplit   5		// slow loop is X times slower then

#define hubo_home_noRef_delay 3.0	// delay before trajectories can be sent while homeing in sec

#ifdef __cplusplus
extern "C" {
#endif

// ach channels
ach_channel_t chan_hubo_ref;      // hubo-ach
ach_channel_t chan_hubo_board_cmd; // hubo-ach-console
ach_channel_t chan_hubo_state;    // hubo-ach-state
ach_channel_t chan_hubo_to_sim;    // hubo-ach-to-sim
ach_channel_t chan_hubo_from_sim;    // hubo-ach-from-sim

int hubo_sig_quit;
int hubo_sig_usr1;
int hubo_sig_usr2;

void hubo_daemonize();
void hubo_daemon_close();
void hubo_assert( int result, int line ); // Instructs the program to quit gracefully if the result is not true

void stack_prefault(void);
static inline void tsnorm(struct timespec *ts);
void getMotorPosFrame(int motor, struct can_frame *frame);
void setEncRef(int jnt, hubo_ref_t *r, hubo_param_t *h);
void setEncRefAll( hubo_ref_t *r, hubo_param_t *h);
void fSetEncRef(int jnt, hubo_state_t *s, hubo_param_t *h, struct can_frame *f);
void fResetEncoderToZero(int jnt, hubo_param_t *h, struct can_frame *f);
void fGetCurrentValue(int jnt, hubo_param_t *h, struct can_frame *f);
void hSetBeep(int jnt, hubo_param_t *h, struct can_frame *f, double beepTime);
void fSetBeep(int jnt, hubo_param_t *h, struct can_frame *f, double beepTime);
void fGetBoardStatusAndErrorFlags(int jnt, hubo_param_t *h, struct can_frame *f);
void hGetBoardStatus(int jnt, hubo_state_t *s, hubo_param_t *h, struct can_frame *f);
void getBoardStatusAllSlow(hubo_state_t *s, hubo_param_t *h, struct can_frame *f);
void fInitializeBoard(int jnt, hubo_param_t *h, struct can_frame *f);
void fEnableMotorDriver(int jnt, hubo_param_t *h, struct can_frame *f);
void fDisableMotorDriver(int jnt, hubo_param_t *h, struct can_frame *f);
void fEnableFeedbackController(int jnt, hubo_param_t *h, struct can_frame *f);
void fDisableFeedbackController(int jnt, hubo_param_t *h, struct can_frame *f);
void fGotoLimitAndGoOffset(int jnt, hubo_param_t *h, struct can_frame *f);
void hInitilizeBoard(int jnt, hubo_ref_t *r, hubo_param_t *h, struct can_frame *f);
void hSetEncRef(int jnt, hubo_state_t *s, hubo_param_t *h, struct can_frame *f);
void hSetEncRefAll(hubo_ref_t *r, hubo_param_t *h, struct can_frame *f);
void hIniAll(hubo_ref_t *r, hubo_param_t *h, hubo_state_t *s, struct can_frame *f);
void huboLoop(hubo_param_t *H_param, int vflag);
void hMotorDriverOnOff(int jnt, hubo_param_t *h, struct can_frame *f, hubo_d_param_t onOff);
void hFeedbackControllerOnOff(int jnt, hubo_ref_t *r, hubo_state_t *s, hubo_param_t *h, struct can_frame *f, hubo_d_param_t onOff);
void hResetEncoderToZero(int jnt, hubo_ref_t *r, hubo_param_t *h, hubo_state_t *s, struct can_frame *f);
void huboMessage(hubo_ref_t *r, hubo_ref_t *r_filt, hubo_param_t *h,
        hubo_state_t *s, hubo_board_cmd_t *c, struct can_frame *f);
void hGotoLimitAndGoOffset(int jnt, hubo_ref_t *r, hubo_ref_t *r_filt, hubo_param_t *h,
    hubo_state_t *s, struct can_frame *f, int send);
int getEncRef(int jnt, hubo_state_t *s, hubo_param_t *h);
void hInitializeBoard(int jnt, hubo_param_t *h, struct can_frame *f);
int decodeFrame(hubo_state_t *s, hubo_param_t *h, struct can_frame *f);
double enc2rad(int jnt, int enc, hubo_param_t *h);
void hGetEncValue(int jnt, uint8_t encChoice, hubo_param_t *h, struct can_frame *f);
void fGetEncValue(int jnt, uint8_t encChoice, hubo_param_t *h, struct can_frame *f);
void getEncAllSlow(hubo_state_t *s, hubo_param_t *h, struct can_frame *f);
void getCurrentAllSlow(hubo_state_t *s, hubo_param_t *h, struct can_frame *f);
void hGetFT(int board, struct can_frame *f, int can);
void fGetFT(int board, struct can_frame *f);
void getFTAllSlow(hubo_state_t *s, hubo_param_t *h, struct can_frame *f);
void fGetAcc(int board, struct can_frame *f);
void hGetAcc(int board, struct can_frame *f);
void getAccAllSlow(hubo_state_t *s, hubo_param_t *h, struct can_frame *f);
void fGetIMU(int board, struct can_frame *f);
void hGetIMU(int board, struct can_frame *f);
void getIMUAllSlow(hubo_state_t *s, hubo_param_t *h, struct can_frame *f);
void hGetCurrentValue(int jnt, hubo_param_t *h, struct can_frame *f);
void setRefAll(hubo_ref_t *r, hubo_param_t *h, hubo_state_t *s, struct can_frame *f);
void hGotoLimitAndGoOffsetAll(hubo_ref_t *r, hubo_ref_t *r_filt, hubo_param_t *h,
    hubo_state_t *s, struct can_frame *f);
void hInitializeBoardAll(hubo_param_t *h, hubo_state_t *s, struct can_frame *f);
void fNullAccFTSensor(int bno, int nullType, struct can_frame *f);
void hNullFTSensor(hubo_d_param_t board, hubo_param_t *h, struct can_frame *f);
void hNullAccSensor(hubo_d_param_t board, hubo_param_t *h, struct can_frame *f);
void hNullAllFTSensors(hubo_param_t *h, struct can_frame *f);
void hNullAllAccSensors(hubo_param_t *h, struct can_frame *f);
void fNullIMUSensor( int bno, struct can_frame *f );
void hNullIMUSensor( hubo_d_param_t board, hubo_param_t *h, struct can_frame *f );
void hNullAllIMUSensors( hubo_param_t *h, struct can_frame *f );
void fInitAccFTSensor( int bno, struct can_frame *f );
void hInitAccFTSensor( hubo_d_param_t board, hubo_param_t *h, struct can_frame *f );
void hInitAllAccFTSensors( hubo_param_t *h, struct can_frame *f );
void hInitAllSensors( hubo_param_t *h, struct can_frame *f );
void hNullSensor( hubo_d_param_t board, hubo_param_t *h, struct can_frame *f );
void hNullAllSensors( hubo_param_t *h, struct can_frame *f );
double doubleFromBytePair(uint8_t data0, uint8_t data1);
uint8_t getFingerInt(double n);

void refFilterMode(hubo_ref_t *r, int L, hubo_param_t *h, hubo_state_t *s, hubo_ref_t *f);

/*   ~~~~   Added by M.X. Grey. Auxiliary CAN functions   ~~~~   */
void hSetPosGain(hubo_board_cmd_t *c, hubo_param_t *h, struct can_frame *f);
void fSetPosGain0(int jnt, hubo_param_t *h, struct can_frame *f, int Kp, int Ki, int Kd);
void fSetPosGain1(int jnt, hubo_param_t *h, struct can_frame *f, int Kp, int Ki, int Kd);
void hSetCurGain(hubo_board_cmd_t *c, hubo_param_t *h, struct can_frame *f);
void fSetCurGain0(int jnt, hubo_param_t *h, struct can_frame *f, int Kp, int Ki, int Kd);
void fSetCurGain1(int jnt, hubo_param_t *h, struct can_frame *f, int Kp, int Ki, int Kd);
void hOpenLoopPWM(hubo_board_cmd_t *c, hubo_param_t *h, struct can_frame *f);
void fOpenLoopPWM_2CH(int jnt, hubo_param_t *h, struct can_frame *f,
            int dir0, int duty0, int dir1, int duty1);
void fOpenLoopPWM_3CH(int jnt, hubo_param_t *h, struct can_frame *f,
            int dir0, int dt0, int dir1, int dt1, int dir2, int dt2);
void fOpenLoopPWM_5CH(int jnt, hubo_param_t *h, struct can_frame *f,
            int dir0, int dt0, int dir1, int dt1, int dir2, int dt2,
            int dir3, int dt3, int dir4, int dt4);
void hSetControlMode(int jnt, hubo_param_t *h, hubo_state_t *s, struct can_frame *f, hubo_d_param_t mode);
void fSetControlMode(int jnt, hubo_param_t *h, struct can_frame *f, int mode);
void hSetAlarm(int jnt, hubo_param_t *h, struct can_frame *f, hubo_d_param_t sound);
void fSetAlarm(int jnt, hubo_param_t *h, struct can_frame *f, int sound);
void hSetDeadZone(int jnt, hubo_param_t *h, struct can_frame *f, int deadzone);
void fSetDeadZone(int jnt, hubo_param_t *h, struct can_frame *f, int deadzone);
void hSetHomeSearchParams( hubo_board_cmd_t *c, hubo_param_t *h, struct can_frame *f );
void fSetHomeSearchParams(int jnt, hubo_param_t *h, struct can_frame *f, int limit,
                unsigned int dir, unsigned int offset);
void hSetEncoderResolution(hubo_board_cmd_t *c, hubo_param_t *h, struct can_frame *f);
void fSetEncoderResolution(int jnt, hubo_param_t *h, struct can_frame *f, int res);
void hSetMaxAccVel(int jnt, hubo_param_t *h, struct can_frame *f, int maxAcc, int maxVel);
void fSetMaxAccVel(int jnt, hubo_param_t *h, struct can_frame *f, int maxAcc, int maxVel);
void hSetLowerPosLimit(hubo_board_cmd_t *c, hubo_param_t *h, struct can_frame *f);
void fSetLowerPosLimit(int jnt, hubo_param_t *h, struct can_frame *f, int enable, int update, int limit);
void hSetUpperPosLimit(hubo_board_cmd_t *c, hubo_param_t *h, struct can_frame *f);
void fSetUpperPosLimit(int jnt, hubo_param_t *h, struct can_frame *f, int enable, int update, int limit);
void hSetHomeAccVel(hubo_board_cmd_t *c, hubo_param_t *h, struct can_frame *f);
void fSetHomeAccVel(int jnt, hubo_param_t *h, struct can_frame *f, float mAcc, int mVelS,
            int mVelP, int mode, int mDuty);
void hSetGainOverride(int jnt, hubo_param_t *h, struct can_frame *f, int gain0, int gain1, double dur);
void fSetGainOverride(int jnt, hubo_param_t *h, struct can_frame *f, int gain0, int gain1, int duration);
void hSetBoardNumber(int jnt, hubo_param_t *h, struct can_frame *f, int boardNum, int rate);
void fSetBoardNumber(int jnt, hubo_param_t *h, struct can_frame *f, int boardNum, int rate);
void fSetJamPwmLimits(int jnt, hubo_param_t *h, struct can_frame *f, int jamLimit, int pwmLimit,
            int lim_detection_duty, int jam_detection_duty );
void hSetErrorBound(int jnt, hubo_param_t *h, struct can_frame *f, int inputDiffErr, int maxError,
            int tempError);
void fSetErrorBound(int jnt, hubo_param_t *h, struct can_frame *f, int inputDiffErr, int maxError,
            int tempError);
void fGetBoardParamA( int jnt, int offset, hubo_param_t *h, struct can_frame *f );
void fGetBoardParamB( int jnt, int offset, hubo_param_t *h, struct can_frame *f );
void fGetBoardParamC( int jnt, int offset, hubo_param_t *h, struct can_frame *f );
void fGetBoardParamD( int jnt, int offset, hubo_param_t *h, struct can_frame *f );
void fGetBoardParamE( int jnt, int offset, hubo_param_t *h, struct can_frame *f );
void fGetBoardParamF( int jnt, int offset, hubo_param_t *h, struct can_frame *f );
void fGetBoardParamG( int jnt, hubo_param_t *h, struct can_frame *f );
void fGetBoardParamH( int jnt, hubo_param_t *h, struct can_frame *f );
void fGetBoardParamI( int jnt, hubo_param_t *h, struct can_frame *f );
void hGetBoardParams( int jnt, hubo_d_param_t param, hubo_param_t *h, hubo_state_t *s, struct can_frame *f );
void fSetComplementaryMode(int jnt, hubo_param_t *h, struct can_frame *f, int the_mode);
void hSetComplementaryMode(int jnt, hubo_param_t *h, struct can_frame *f, int the_mode);

void clearCanBuff(hubo_state_t *s, hubo_param_t *h, struct can_frame *f);
void getStatusIterate( hubo_state_t *s, hubo_param_t *h, struct can_frame *f);
int isError( int jnt, hubo_state_t *s);
uint8_t isHands(int jnt);
static uint8_t getJMC( hubo_param_t *h, int jnt ) { return (uint8_t)h->joint[jnt].jmc; }
static uint8_t getCAN( hubo_param_t *h, int jnt ) { return h->joint[jnt].can; }
static hubo_can_t getSocket( hubo_param_t *h, int jnt ) { return hubo_socket[h->joint[jnt].can]; }
static hubo_can_t sensorSocket( hubo_param_t *h, hubo_sensor_index_t board) {return hubo_socket[h->sensor[board].can];}

uint8_t int_to_bytes(int d, int index);
uint8_t duty_to_byte(int dir, int duty);
unsigned short DrcFingerSignConvention(short h_input,unsigned char h_type);
unsigned long DrcSignConvention(long h_input);


#ifdef __cplusplus
}
#endif

#endif // HUBO_DAEMON_H

/*
Copyright (c) 2012, Daniel M. Lofaro
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in the
      documentation and/or other materials provided with the distribution.
    * Neither the name of the author nor the names of its contributors may 
      be used to endorse or promote products derived from this software 
      without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT, INDIRECT, 
INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT 
LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR 
PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF 
LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE 
OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF 
ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
/* -*-	indent-tabs-mode:t; tab-width: 8; c-basic-offset: 8  -*- */

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
#include "hubo-daemon.h"
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
#include "hubo-canCmd.h"


/* At time of writing, these constants are not defined in the headers */
#ifndef PF_CAN
#define PF_CAN 29
#endif

#ifndef AF_CAN
#define AF_CAN PF_CAN
#endif

/* ... */

/* Somewhere in your app */



// Timing info
#define NSEC_PER_SEC    1000000000

#define hubo_home_noRef_delay 6.0	// delay before trajectories can be sent while homeing in sec


/* functions */
void stack_prefault(void);
static inline void tsnorm(struct timespec *ts);
void getMotorPosFrame(int motor, struct can_frame *frame);
void setEncRef(int jnt, struct hubo_ref *r, struct hubo_param *h);
void setEncRefAll( struct hubo_ref *r, struct hubo_param *h);
void fSetEncRef(int jnt, struct hubo_ref *r, struct hubo_param *h, struct can_frame *f);
void fResetEncoderToZero(int jnt, struct hubo_param *h, struct can_frame *f);
void fGetCurrentValue(int jnt, struct hubo_param *h, struct can_frame *f);
void hSetBeep(int jnt, struct hubo_param *h, struct can_frame *f, double beepTime);
void fSetBeep(int jnt, struct hubo_ref *r, struct hubo_param *h, struct can_frame *f, double beepTime);
void fGetBoardStatusAndErrorFlags(int jnt, struct hubo_ref *r, struct hubo_param *h, struct can_frame *f);
void fInitializeBoard(int jnt, struct hubo_param *h, struct can_frame *f);
void fEnableMotorDriver(int jnt, struct hubo_param *h, struct can_frame *f);
void fDisableMotorDriver(int jnt, struct hubo_param *h, struct can_frame *f);
void fEnableFeedbackController(int jnt, struct hubo_param *h, struct can_frame *f);
void fDisableFeedbackController(int jnt, struct hubo_param *h, struct can_frame *f);
void fGotoLimitAndGoOffset(int jnt, struct hubo_param *h, struct can_frame *f);
void hInitilizeBoard(int jnt, struct hubo_ref *r, struct hubo_param *h, struct can_frame *f);
void hSetEncRef(int jnt, struct hubo_ref *r, struct hubo_param *h, struct can_frame *f);
void hSetEncRefAll(struct hubo_ref *r, struct hubo_param *h, struct can_frame *f);
void hIniAll(struct hubo_ref *r, struct hubo_param *h, struct hubo_state *s, struct can_frame *f);
void huboLoop(struct hubo_param *H_param);
void hMotorDriverOnOff(int jnt, struct hubo_param *h, struct can_frame *f, int onOff);
void hFeedbackControllerOnOff(int jnt, struct hubo_param *h, struct can_frame *f, hubo_d_param_t onOff);
void hResetEncoderToZero(int jnt, struct hubo_param *h, struct hubo_state *s, struct can_frame *f);
void huboMessage(struct hubo_ref *r, struct hubo_param *h, struct hubo_state *s, struct hubo_board_cmd *c, struct can_frame *f);
void hGotoLimitAndGoOffset(int jnt, struct hubo_ref *r, struct hubo_param *h, struct hubo_state *s, struct can_frame *f);
uint32_t getEncRef(int jnt, struct hubo_ref *r , struct hubo_param *h);
void hInitializeBoard(int jnt, struct hubo_ref *r, struct hubo_param *h, struct can_frame *f);
int decodeFrame(struct hubo_state *s, struct hubo_param *h, struct can_frame *f);
double enc2rad(int jnt, int enc, struct hubo_param *h);
void hGetEncValue(int jnt, struct hubo_param *h, struct can_frame *f);
void fGetEncValue(int jnt, struct hubo_param *h, struct can_frame *f);
void getEncAll(struct hubo_state *s, struct hubo_param *h, struct can_frame *f);
void getEncAllSlow(struct hubo_state *s, struct hubo_param *h, struct can_frame *f);
void getCurrentAll(struct hubo_state *s, struct hubo_param *h, struct can_frame *f);
void getCurrentAllSlow(struct hubo_state *s, struct hubo_param *h, struct can_frame *f);
void hGetCurrentValue(int jnt, struct hubo_param *h, struct can_frame *f);
void setRefAll(struct hubo_ref *r, struct hubo_param *h, struct hubo_state *s, struct can_frame *f);
void hGotoLimitAndGoOffsetAll(struct hubo_ref *r, struct hubo_param *h, struct hubo_state *s, struct can_frame *f);
void hInitializeBoardAll(struct hubo_ref *r, struct hubo_param *h, struct hubo_state *s, struct can_frame *f);


/*   ~~~~   Added by M.X. Grey. Auxiliary CAN functions (frame fillers)   ~~~~   */
void hSetPosGain(struct hubo_board_cmd *c, struct hubo_param *h, struct can_frame *f);
void fSetPosGain0(int jnt, struct hubo_param *h, struct can_frame *f, int Kp, int Ki, int Kd);
void fSetPosGain1(int jnt, struct hubo_param *h, struct can_frame *f, int Kp, int Ki, int Kd);
void hSetCurGain(struct hubo_board_cmd *c, struct hubo_param *h, struct can_frame *f);
void fSetCurGain0(int jnt, struct hubo_ref *r, struct hubo_param *h, struct can_frame *f, int Kp, int Ki, int Kd);
void fSetCurGain1(int jnt, struct hubo_ref *r, struct hubo_param *h, struct can_frame *f, int Kp, int Ki, int Kd);
void hOpenLoopPWM(struct hubo_board_cmd *c, hubo_param *h, struct can_frame *f);
void fOpenLoopPWM_2CH(int jnt, struct hubo_param *h, struct can_frame *f,
			int dir0, int duty0, int dir1, int duty1);
void fOpenLoopPWM_3CH(int jnt, struct hubo_param *h, struct can_frame *f,
			int dir0, int dt0, int dir1, int dt1, int dir2, int dt2);
void fOpenLoopPWM_5CH(int jnt, struct hubo_param *h, struct can_frame *f,
			int dir0, int dt0, int dir1, int dt1, int dir2, int dt2,
			int dir3, int dt3, int dir4, int dt4);
void hSetControlMode(int jnt, struct hubo_param *h, struct can_frame *f, hubo_d_param_t mode);
void fSetControlMode(int jnt, struct hubo_param *h, struct can_frame *f, int mode);
void hSetAlarm(int jnt, struct hubo_param *h, struct can_frame *f, hubo_d_param_t sound);
void fSetAlarm(int jnt, struct hubo_param *h, struct can_frame *f, int sound);
void hSetDeadZone(int jnt, struct hubo_param *h, struct can_frame *f, int deadzone);
void fSetDeadZone(int jnt, struct hubo_param *h, struct can_frame *f, int deadzone);
void fGetBoardParameters(int jnt, struct hubo_param *h, struct can_frame *f, int parm);
void hSetHomeSearchParams( struct hubo_board_cmd *c, struct hubo_param *h, struct can_frame *f );
void fSetHomeSearchParams(int jnt, struct hubo_param *h, struct can_frame *f, int limit,
				unsigned int dir, unsigned int offset);
void hSetEncoderResolution(struct hubo_board_cmd *c, struct hubo_param *h, struct can_frame *f);
void fSetEncoderResolution(int jnt, struct hubo_param *h, struct can_frame *f, int res);
void hSetMaxAccVel(int jnt, struct hubo_param *h, struct can_frame *f, int maxAcc, int maxVel);
void fSetMaxAccVel(int jnt, struct hubo_param *h, struct can_frame *f, int maxAcc, int maxVel);
void hSetLowerPosLimit(struct hubo_board_cmd *c, struct hubo_param *h, struct can_frame *f);
void fSetLowerPosLimit(int jnt, struct hubo_param *h, struct can_frame *f, int enable, int update, int limit);
void hSetUpperPosLimit(struct hubo_board_cmd *c, struct hubo_param *h, struct can_frame *f);
void fSetUpperPosLimit(int jnt, struct hubo_param *h, struct can_frame *f, int enable, int update, int limit);
void hSetHomeAccVel(struct hubo_board_cmd *c, struct hubo_param *h, struct can_frame *f);
void fSetHomeAccVel(int jnt, struct hubo_param *h, struct can_frame *f, float mAcc, int mVelS,
			int mVelP, int mode, int mDuty);
void hSetGainOverride(int jnt, struct hubo_param *h, struct can_frame *f, int gain0, int gain1, double dur);
void fSetGainOverride(int jnt, struct hubo_param *h, struct can_frame *f, int gain0, int gain1, int duration);
void hSetBoardNumber(int jnt, struct hubo_param *h, struct can_frame *f, int boardNum, int rate);
void fSetBoardNumber(int jnt, struct hubo_param *h, struct can_frame *f, int boardNum, int rate);
void fSetJamPwmLimits(int jnt, struct hubo_param *h, struct can_frame *f, int jamLimit, int pwmLimit,
			int lim_detection_duty, int jam_detection_duty );
void hSetErrorBound(int jnt, struct hubo_param *h, struct can_frame *f, int inputDiffErr, int maxError,
			int tempError);
void fSetErrorBound(int jnt, struct hubo_param *h, struct can_frame *f, int inputDiffErr, int maxError,
			int tempError);







inline uint8_t getJMC( struct hubo_param *h, int jnt ) { return (uint8_t)h->joint[jnt].jmc; }
inline uint8_t getCAN( struct hubo_param *h, int jnt ) { return h->joint[jnt].can; }
inline hubo_can_t getSocket( struct hubo_param *h, int jnt ) { return hubo_socket[h->joint[jnt].can]; }


uint8_t num_to_bytes(double d, int index);
uint8_t num_to_bytes(int d, int index);
uint8_t duty_to_byte(int dir, int duty);

// ach message type
//typedef struct hubo h[1];

// ach channels
ach_channel_t chan_hubo_ref;	  // hubo-ach
ach_channel_t chan_hubo_board_cmd; // hubo-ach-console
ach_channel_t chan_hubo_state;    // hubo-ach-state

//int hubo_ver_can = 0;
/* time for the ref not to be sent while a joint is being moved */
//double hubo_noRefTime[HUBO_JOINT_NUM];
double hubo_noRefTimeAll = 0.0;

void huboLoop(struct hubo_param *H_param) {
	int i = 0;  // iterator
	// get initial values for hubo
	struct hubo_ref H_ref;
	struct hubo_board_cmd H_cmd;
	struct hubo_state H_state;
	memset( &H_ref,   0, sizeof(H_ref));
	memset( &H_cmd,  0, sizeof(H_cmd));
	memset( &H_state, 0, sizeof(H_state));
	//memset(&hubo_noRef, 0, sizeof(hubo_noRef));

	size_t fs;
	int r = ach_get( &chan_hubo_ref, &H_ref, sizeof(H_ref), &fs, NULL, ACH_O_LAST );
	if(ACH_OK != r) {fprintf(stderr, "Ref r = %s\n",ach_result_to_string(r));}
	hubo_assert( sizeof(H_ref) == fs, "Size of H_ref frame does not match the filesystem" );
	r = ach_get( &chan_hubo_board_cmd, &H_cmd, sizeof(H_cmd), &fs, NULL, ACH_O_LAST );
	if(ACH_OK != r) {fprintf(stderr, "CMD r = %s\n",ach_result_to_string(r));}
	hubo_assert( sizeof(H_cmd) == fs, "Size of the H_cmd frame does not match the filesystem" );
	r = ach_get( &chan_hubo_state, &H_state, sizeof(H_state), &fs, NULL, ACH_O_LAST );
	if(ACH_OK != r) {fprintf(stderr, "State r = %s\n",ach_result_to_string(r));}
	hubo_assert( sizeof(H_state) == fs, "Size of the H_state frame does not match the filesystem" );
	
	// put back on channels
	ach_put(&chan_hubo_ref, &H_ref, sizeof(H_ref));
	ach_put(&chan_hubo_board_cmd, &H_cmd, sizeof(H_cmd));
	ach_put(&chan_hubo_state, &H_state, sizeof(H_state));
//	ach_put( &chan_hubo_ref, &H, sizeof(H));

/* period */
//	int interval = 500000000; // 2hz (0.5 sec)
//	int interval = 20000000; // 50 hz (0.02 sec)
	int interval = 10000000; // 100 hz (0.01 sec)
//	int interval = 5000000; // 200 hz (0.005 sec)
//	int interval = 2000000; // 500 hz (0.002 sec)

	double T = (double)interval/(double)NSEC_PER_SEC; // 100 hz (0.01 sec)
	printf("T = %1.3f sec\n",T);

	/* Send a message to the CAN bus */
   	struct can_frame frame;

	// time info
	struct timespec t;

	// get current time
	//clock_gettime( CLOCK_MONOTONIC,&t);
	clock_gettime( 0,&t);

	sprintf( frame.data, "1234578" );
	frame.can_dlc = strlen( frame.data );

	int a = 0;

	fprintf(stdout, "Start Hubo Loop\n");
//	while(1) {
	while(!hubo_sig_quit) {
		fs = 0;
		// wait until next shot
		clock_nanosleep(0,TIMER_ABSTIME,&t, NULL);

		/* Get latest ACH message */
		/*
		r = ach_get( &chan_hubo_ref, &H_ref, sizeof(H_ref), &fs, NULL, ACH_O_LAST );
		hubo_assert( ACH_OK == r || ACH_MISSED_FRAME == r || ACH_STALE_FRAMES == r );
		hubo_assert( ACH_STALE_FRAMES == r || sizeof(H_ref) == fs );
		*/
		// hubo_assert( sizeof(H_param) == fs );

		r = ach_get( &chan_hubo_ref, &H_ref, sizeof(H_ref), &fs, NULL, ACH_O_LAST );
		if(ACH_OK != r) {
				if(debug) {
					fprintf(stderr, "Ref r = %s\n",ach_result_to_string(r));}
			}
		else{	hubo_assert( sizeof(H_ref) == fs ); }

/*
		r = ach_get( &chan_hubo_board_cmd, &H_cmd, sizeof(H_cmd), &fs, NULL, ACH_O_LAST );
		if(ACH_OK != r) {
				if(debug) {
					printf("CMD r = %s\n",ach_result_to_string(r));}
				}
		else{ hubo_assert( sizeof(H_cmd) == fs ); }
*/

		r = ach_get( &chan_hubo_state, &H_state, sizeof(H_state), &fs, NULL, ACH_O_LAST );
		if(ACH_OK != r) {
				if(debug) {
					fprintf(stderr, "State r = %s\n",ach_result_to_string(r));}
				}
		else{ hubo_assert( sizeof(H_state) == fs ); }

		/* read hubo console */
		huboMessage(&H_ref, H_param, &H_state, &H_cmd, &frame);
		/* set reference for zeroed joints only */
//		for(i = 0; i < HUBO_JOINT_COUNT; i++) {
//			if(H_param.joint[i].zeroed == true) {
//				hSetEncRef(H_param.joint[i].jntNo, &H_ref, H_param, &frame);
//			}
//		}

		/* Set all Ref */
		if(hubo_noRefTimeAll < T ) {
			setRefAll(&H_ref, H_param, &H_state, &frame);
		}
		else{
			hubo_noRefTimeAll = hubo_noRefTimeAll - T;
		}
		
		/* Get all Encoder data */
		getEncAllSlow(&H_state, H_param, &frame); 

		/* Get all Current data */
//		getCurrentAllSlow(&H_state, &H_param, &frame);
		
//		hGetCurrentValue(RSY, &H_param, &frame);
//		readCan(hubo_socket[H_param.joint[RSY].can], &frame, HUBO_CAN_TIMEOUT_DEFAULT);
//		decodeFrame(&H_state, &H_param, &frame);

		/* put data back in ACH channel */
		ach_put( &chan_hubo_state, &H_state, sizeof(H_state));
		t.tv_nsec+=interval;
		tsnorm(&t);

		fflush(stdout);
		fflush(stderr);
	}


}









static inline void tsnorm(struct timespec *ts){

//	clock_nanosleep( NSEC_PER_SEC, TIMER_ABSTIME, ts, NULL);
	// calculates the next shot
	while (ts->tv_nsec >= NSEC_PER_SEC) {
		//usleep(100);	// sleep for 100us (1us = 1/1,000,000 sec)
		ts->tv_nsec -= NSEC_PER_SEC;
		ts->tv_sec++;
	}
}


/*
uint32_t getEncRef(int jnt, struct hubo *h)
{
	//return (uint32_t)((double)h->joint[jnt].drive/(double)h->joint[jnt].driven/(double)h->joint[jnt].harmonic/(double)h->joint[jnt].enc*2.0*M_PI);
	return (uint32_t)((double)h->joint[jnt].drive/(double)h->joint[jnt].driven/(double)h->joint[jnt].harmonic/(double)h->joint[jnt].ref*2.0*M_PI);
}
*/
void setRefAll(struct hubo_ref *r, struct hubo_param *h, struct hubo_state *s, struct can_frame *f) {
	///> Requests all encoder and records to hubo_state
	int c[HUBO_JMC_COUNT];
	memset( &c, 0, sizeof(c));
	int jmc = 0;
	int i = 0;
	int canChan = 0;
	
	for( canChan = 0; canChan < HUBO_CAN_CHAN_NUM; canChan++) {
		for( i = 0; i < HUBO_JOINT_COUNT; i++ ) {
			jmc = h->joint[i].jmc+1;
			if((0 == c[jmc]) & (canChan == h->joint[i].can) & (s->joint[i].active == true)){	// check to see if already asked that motor controller
				hSetEncRef(i, r, h, f);
				c[jmc] = 1;
//				if(i == RHY){ printf(".%d %d %d %d",jmc,h->joint[RHY].can, canChan, c[jmc]); }
			}

		}
	}	
}




void getEncAll(struct hubo_state *s, struct hubo_param *h, struct can_frame *f) {
	///> Requests all encoder and records to hubo_state
	char c[HUBO_JMC_COUNT];
	memset( &c, 0, sizeof(c));
	//memset( &c, 1, sizeof(c));
	int jmc = 0;
	int i = 0;
//	c[h->joint[REB].jmc] = 0;
	int canChan = 0;
	for( canChan = 0; canChan < HUBO_CAN_CHAN_NUM; canChan++) {
	for( i = 0; i < HUBO_JOINT_COUNT; i++ ) {
		jmc = h->joint[i].jmc;
		if((0 == c[jmc]) & (canChan == h->joint[i].can)){	// check to see if already asked that motor controller
			hGetEncValue(i, h, f);
//			readCan(hubo_socket[h->joint[i].can], f, HUBO_CAN_TIMEOUT_DEFAULT);
//			decodeFrame(s, h, f);
			c[jmc] = 1;
		}
	}}	
	memset( &c, 0, sizeof(c));
	for( i = 0; i < HUBO_JOINT_COUNT; i++ ) {
		jmc = h->joint[i].jmc;
		if((0 == c[jmc]) & (canChan == h->joint[i].can)){	// check to see if already asked that motor controller
			readCan(getSocket(h,i), f, HUBO_CAN_TIMEOUT_DEFAULT);
			decodeFrame(s, h, f);
			c[jmc] = 1;
		}
	}

}


void getEncAllSlow(struct hubo_state *s, struct hubo_param *h, struct can_frame *f) {
	///> Requests all encoder and records to hubo_state
	char c[HUBO_JMC_COUNT];
	memset( &c, 0, sizeof(c));
	//memset( &c, 1, sizeof(c));
	int jmc = 0;
	int i = 0;
//	c[h->joint[REB].jmc] = 0;
	int canChan = 0;
	for( canChan = 0; canChan < HUBO_CAN_CHAN_NUM; canChan++) {
	for( i = 0; i < HUBO_JOINT_COUNT; i++ ) {
		jmc = h->joint[i].jmc;
		if((0 == c[jmc]) & (canChan == h->joint[i].can)){	// check to see if already asked that motor controller
			hGetEncValue(i, h, f);
			readCan(getSocket(h,i), f, HUBO_CAN_TIMEOUT_DEFAULT);
			decodeFrame(s, h, f);
			c[jmc] = 1;
		}
	}}
}




void getCurrentAll(struct hubo_state *s, struct hubo_param *h, struct can_frame *f) {
	///> Requests all motor currents and records to hubo_state
	char c[HUBO_JMC_COUNT];
	memset( &c, 0, sizeof(c));
	//memset( &c, 1, sizeof(c));
	int jmc = 0;
	int i = 0;
//	c[h->joint[REB].jmc] = 0;
	int canChan = 0;
	for( canChan = 0; canChan < HUBO_CAN_CHAN_NUM; canChan++) {
	for( i = 0; i < HUBO_JOINT_COUNT; i++ ) {
		jmc = h->joint[i].jmc;
		if(0 == c[jmc]){	// check to see if already asked that motor controller
			hGetCurrentValue(i, h, f);
//			readCan(hubo_socket[h->joint[i].can], f, HUBO_CAN_TIMEOUT_DEFAULT);
//			decodeFrame(s, h, f);
			c[jmc] = 1;
		}
	}}	
	memset( &c, 0, sizeof(c));
	for( i = 0; i < HUBO_JOINT_COUNT; i++ ) {
		jmc = h->joint[i].jmc;
		if(0 == c[jmc]){	// check to see if already asked that motor controller
			readCan(getSocket(h,i), f, HUBO_CAN_TIMEOUT_DEFAULT);
			decodeFrame(s, h, f);
			c[jmc] = 1;
		}
	}

}



void getCurrentAllSlow(struct hubo_state *s, struct hubo_param *h, struct can_frame *f) {
	///> Requests all motor currents and records to hubo_state
	char c[HUBO_JMC_COUNT];
	memset( &c, 0, sizeof(c));
	//memset( &c, 1, sizeof(c));
	int jmc = 0;
	int i = 0;
//	c[h->joint[REB].jmc] = 0;
	int canChan = 0;
	for( canChan = 0; canChan < HUBO_CAN_CHAN_NUM; canChan++) {
	for( i = 0; i < HUBO_JOINT_COUNT; i++ ) {
		jmc = h->joint[i].jmc;
		if(0 == c[jmc]){	// check to see if already asked that motor controller
			hGetCurrentValue(i, h, f);
			readCan(getSocket(h,i), f, HUBO_CAN_TIMEOUT_DEFAULT);
			decodeFrame(s, h, f);
			c[jmc] = 1;
		}
	}}	

}










uint32_t getEncRef(int jnt, struct hubo_ref *r , struct hubo_param *h) {
	// set encoder from reference
	struct hubo_joint_param *p = &h->joint[jnt];
	return (uint32_t)((double)p->driven/(double)p->drive*(double)p->harmonic*(double)p->enc*(double)r->ref[jnt]/2.0/M_PI);
}

unsigned long signConvention(long _input) {
	if (_input < 0) return (unsigned long)( ((-_input)&0x007FFFFF) | (1<<23) );
	else return (unsigned long)_input;
}
void fSetEncRef(int jnt, struct hubo_ref *r, struct hubo_param *h, struct can_frame *f) {
	// set ref
	f->can_id 	= REF_BASE_TXDF + h->joint[jnt].jmc;  //CMD_TXD;F// Set ID
	__u8 data[6];
	uint16_t jmc = h->joint[jnt].jmc;
	if(h->joint[jnt].numMot == 2) {
		__u8 m0 = h->driver[jmc].jmc[0];
		__u8 m1 = h->driver[jmc].jmc[1];
//		printf("m0 = %i, m1= %i \n",m0, m1);


		unsigned long pos0 = signConvention((int)getEncRef(m0, r, h));
		unsigned long pos1 = signConvention((int)getEncRef(m1, r, h));
		f->data[0] =    (uint8_t)(pos0		& 0x000000FF);
		f->data[1] = 	(uint8_t)((pos0>>8) 	& 0x000000FF);
		f->data[2] = 	(uint8_t)((pos0>>16)	& 0x000000FF);
		f->data[3] =    (uint8_t)(pos1 		& 0x000000FF);
		f->data[4] = 	(uint8_t)((pos1>>8) 	& 0x000000FF);
		f->data[5] = 	(uint8_t)((pos1>>16)	& 0x000000FF);

		if(r->ref[m0] < 0.0) {
			f->data[2] = f->data[2] | 0x80;
		}
		if(r->ref[m1] < 0.0) {
			f->data[5] = f->data[5] | 0x80;
		}
	}
	else if(h->joint[jnt].numMot == 1) {
		__u8 m0 = h->driver[jmc].jmc[0];
		__u8 m1 = h->driver[jmc].jmc[0];
//		printf("m0 = %i, m1= %i \n",m0, m1);


		unsigned long pos0 = signConvention((int)getEncRef(m0, r, h));
		unsigned long pos1 = signConvention((int)getEncRef(m1, r, h));
		f->data[0] =    (uint8_t)(pos0		& 0x000000FF);
		f->data[1] = 	(uint8_t)((pos0>>8) 	& 0x000000FF);
		f->data[2] = 	(uint8_t)((pos0>>16)	& 0x000000FF);
		f->data[3] =    (uint8_t)(pos1 		& 0x000000FF);
		f->data[4] = 	(uint8_t)((pos1>>8) 	& 0x000000FF);
		f->data[5] = 	(uint8_t)((pos1>>16)	& 0x000000FF);

		if(r->ref[m0] < 0.0) {
			f->data[2] = f->data[2] | 0x80;
		}
		if(r->ref[m1] < 0.0) {
			f->data[5] = f->data[5] | 0x80;
		}
	}
	f->can_dlc = 6; //= strlen( data );	// Set DLC
}


void hSetPosGain(struct hubo_board_cmd *c, struct hubo_param *h, struct can_frame *f)
{
	int jnt = c->joint;
	if(h->joint[jnt].motNo == 0 || h->joint[jnt].numMot > 2)
		fSetPosGain0(jnt, h, f, c->iValues[0], c->iValues[1], c->iValues[2]);
	else if(h->joint[jnt].motNo == 1)
		fSetPosGain1(jnt, h, f, c->iValues[0], c->iValues[1], c->iValues[2]);
	
	sendCan(getSocket(h,jnt), f);
	fprintf(stdout, "Joint number %d has changed position gain values to:\n\tKp:%d\tKi:%d\tKd:%d\n",
			jnt, c->iValues[0], c->iValues[1], c->iValues[2] );
}

void fSetPosGain0(int jnt, struct hubo_param *h, struct can_frame *f, int Kp, int Ki, int Kd){

	f->can_id	= CMD_TXDF;
	
	__u8 data[8];
	f->data[0]	= getJMC(h,jnt);
	f->data[1]	= H_SET_POS_GAIN_0;
	f->data[2]	= num_to_bytes(Kp,1);
	f->data[3]	= num_to_bytes(Kp,2);
	f->data[4]	= num_to_bytes(Ki,1);
	f->data[5]	= num_to_bytes(Ki,2);
	f->data[6]	= num_to_bytes(Kd,1);
	f->data[7]	= num_to_bytes(Kd,2);

	f->can_dlc 	= 8;
}
void fSetPosGain1(int jnt, struct hubo_param *h, struct can_frame *f, int Kp, int Ki, int Kd){

	f->can_id	= CMD_TXDF;

	__u8 data[8];
	f->data[0]	= getJMC(h,jnt);
	f->data[1]	= H_SET_POS_GAIN_1;
	f->data[2]	= num_to_bytes(Kp,1);
	f->data[3]	= num_to_bytes(Kp,2);
	f->data[4]	= num_to_bytes(Ki,1);
	f->data[5]	= num_to_bytes(Ki,2);
	f->data[6]	= num_to_bytes(Kd,1);
	f->data[7]	= num_to_bytes(Kd,2);

	f->can_dlc	= 8;
}

void hSetCurGain(struct hubo_board_cmd *c, struct hubo_ref *r, struct hubo_param *h, struct can_frame *f)
{
	int jnt = c->joint;
	if(h->joint[jnt].motNo == 0 || h->joint[jnt].numMot > 2)
		fSetCurGain0(jnt, r, h, f, c->iValues[0], c->iValues[1], c->iValues[2]);
	else if(h->joint[jnt].motNo == 1)
		fSetCurGain1(jnt, r, h, f, c->iValues[0], c->iValues[1], c->iValues[2]);
	
	sendCan(getSocket(h,jnt), f);
	fprintf(stdout, "Joint number %d has changed current gain values to:\n\tKp:%d\tKi:%d\tKd:%d\n",
			jnt, c->iValues[0], c->iValues[1], c->iValues[2] );
}

void fSetCurGain0(int jnt, struct hubo_ref *r, struct hubo_param *h, struct can_frame *f, int Kp, int Ki, int Kd){

	f->can_id	= CMD_TXDF;

	__u8 data[8];
	f->data[0]	= getJMC(h,jnt);
	f->data[1]	= H_SET_CUR_GAIN_0;
	f->data[2]	= num_to_bytes(Kp,1);
	f->data[3]	= num_to_bytes(Kp,2);
	f->data[4]	= num_to_bytes(Ki,1);
	f->data[5]	= num_to_bytes(Ki,2);
	f->data[6]	= num_to_bytes(Kd,1);
	f->data[7]	= num_to_bytes(Kd,2);

	f->can_dlc	= 8;
}

void fSetCurGain1(int jnt, struct hubo_ref *r, struct hubo_param *h, struct can_frame *f, int Kp, int Ki, int Kd){

	f->can_id	= CMD_TXDF;

	__u8 data[8];
	f->data[0]	= getJMC(h,jnt);
	f->data[1]	= H_SET_CUR_GAIN_1;
	f->data[2]	= num_to_bytes(Kp,1);
	f->data[3]	= num_to_bytes(Kp,2);
	f->data[4]	= num_to_bytes(Ki,1);
	f->data[5]	= num_to_bytes(Ki,2);
	f->data[6]	= num_to_bytes(Kd,1);
	f->data[7]	= num_to_bytes(Kd,2);

	f->can_dlc	= 8;
}





// 4
void fGetCurrentValue(int jnt, struct hubo_param *h, struct can_frame *f) {
	// get the value of the current in 10mA units A = V/100
	f->can_id 	= CMD_TXDF;	// Set ID

	__u8 data[2];
	f->data[0] 	= getJMC(h,jnt);
	f->data[1]	= H_GET_CURRENT;

	f->can_dlc 	= 2;
}

// 4

void hSetAlarm(int jnt, struct hubo_param *h, struct can_frame *f, hubo_d_param_t sound)
{
	uint8_t h_sound;
	switch (sound) {
		case D_ALARM_SOUND1:
			h_sound = H_ALARM_S1; break;
		case D_ALARM_SOUND2:
			h_sound = H_ALARM_S2; break;
		case D_ALARM_SOUND3:
			h_sound = H_ALARM_S3; break;
		case D_ALARM_SOUND4:
			h_sound = H_ALARM_S4; break;
		case D_ALARM_OFF:
			h_sound = H_ALARM_OFF; break;
		default:
			h_sound = H_ALARM_OFF;
			fprintf(stderr, "Invalid parameter given for Alarm Sound: %d\n", (int)sound);
			break;
	}
	fSetAlarm(jnt, h, f, h_sound);
	sendCan(getSocket(h,jnt), f);
}

void fSetAlarm(int jnt, struct hubo_param *h, struct can_frame *f, int h_sound)
{
	f->can_id 	= CMD_TXDF;	// Set ID

	__u8 data[3];
	f->data[0] 	= getJMC(h,jnt);	
	f->data[1]	= H_ALARM;
	f->data[2] 	= (uint8_t)sound; 	// Use H_ALARM_S1, S2, S3, S4, or OFF

	f->can_dlc = 3;
}


void hOpenLoopPWM(struct hubo_board_cmd *c, hubo_param *h, struct can_frame *f)
{
	int jnt = c->joint;
	
	if(h->joint[jnt].numMot == 2)
	{
		if(	(c->param[0]!=D_CLOCKWISE&&c->param[0]!=D_COUNTERCLOCKWISE) ||
			(c->param[1]!=D_CLOCKWISE&&c->param[1]!=D_COUNTERCLOCKWISE) ||
			(c->iValues[0]>100&&c->iValues[0]<-100) ||
			(c->iValues[1]>100&&c->iValues[1]<-100) )

		{
			fprintf(stderr, "Invalid PWM Values:\n\tDir:%d\tDuty:%d\tDir:%d\tDuty:%d\n\t"
					"Dir must be D_CLOCKWISE (%d) or D_COUNTERCLOCKWISE (%d)\n\t"
					"Duty range: [-100,100]\n",
					c->param[0], c->iValues[0], c->param[1], c->iValues[1],
					D_CLOCKWISE, D_COUNTERCLOCKWISE );
		}
		else
		{
			int dir1, dir2;
			if(c->param[0]==D_CLOCKWISE) dir1=0; else dir1=1;
			if(c->param[1]==D_CLOCKWISE) dir2=0; else dir2=1;

			fOpenLoopPWM_2CH(jnt, h, f, dir1, c->iValues[0], dir2, c->iValues[1]);
			
			sendCan(getSocket(h,jnt), f);
		}
	}
	else if(h->joint[jnt].numMot == 3)
	{
		if(	(c->param[0]!=D_CLOCKWISE&&c->param[0]!=D_COUNTERCLOCKWISE) ||
			(c->param[1]!=D_CLOCKWISE&&c->param[1]!=D_COUNTERCLOCKWISE) ||
			(c->param[2]!=D_CLOCKWISE&&c->param[2]!=D_COUNTERCLOCKWISE) ||
			(c->iValues[0]>100&&c->iValues[0]<-100) ||
			(c->iValues[1]>100&&c->iValues[1]<-100) ||
			(c->iValues[2]>100&&c->iValues[2]<-100) )
		{
			fprintf(stderr, "Invalid PWM Values:\n\tDir:%d\tDuty:%d\tDir:%d\tDuty:%d\t"
					"Dir:%d\tDuty:%d\n\t"
					"Dir must be D_CLOCKWISE (%d) or D_COUNTERCLOCKWISE (%d)\n\t"
					"Duty range: [-100,100]\n",
					(int)c->param[0], c->iValues[0], (int)c->param[1], c->iValues[1],
					(int)c->param[2], c->iValues[2],
					(int)D_CLOCKWISE, (int)D_COUNTERCLOCKWISE );
		}
		else
		{
			int dir1, dir2, dir3;
			if(c->param[0]==D_CLOCKWISE) dir1=0; else dir1=1;
			if(c->param[1]==D_CLOCKWISE) dir2=0; else dir2=1;
			if(c->param[2]==D_CLOCKWISE) dir3=0; else dir3=1;
			

			fOpenLoopPWM_3CH(jnt, h, f, dir1, c->iValues[0], dir2, c->iValues[1],
						 dir3, c->iValues[2] );
			
			sendCan(getSocket(h,jnt), f);
		}
	}
	else if(h->joint[jnt].numMot == 5)
	{
		if(	(c->param[0]!=D_CLOCKWISE&&c->param[0]!=D_COUNTERCLOCKWISE) ||
			(c->param[1]!=D_CLOCKWISE&&c->param[1]!=D_COUNTERCLOCKWISE) ||
			(c->param[2]!=D_CLOCKWISE&&c->param[2]!=D_COUNTERCLOCKWISE) ||
			(c->param[3]!=D_CLOCKWISE&&c->param[3]!=D_COUNTERCLOCKWISE) ||
			(c->param[4]!=D_CLOCKWISE&&c->param[4]!=D_COUNTERCLOCKWISE) ||
			(c->iValues[0]>100&&c->iValues[0]<-100) ||
			(c->iValues[1]>100&&c->iValues[1]<-100) ||
			(c->iValues[2]>100&&c->iValues[2]<-100) ||
			(c->iValues[3]>100&&c->iValues[3]<-100) ||
			(c->iValues[4]>100&&c->iValues[4]<-100) )
		{
			fprintf(stderr, "Invalid PWM Values:\n\tDir:%d\tDuty:%d\tDir:%d\tDuty:%d\t"
					"Dir:%d\tDuty:%d\tDir:%d\tDuty:%d\tDir:%d\tDuty:%d\n\t"
					"Dir must be D_CLOCKWISE (%d) or D_COUNTERCLOCKWISE (%d)\n\t"
					"Duty range: [-100,100]\n",
					(int)c->param[0], c->iValues[0], (int)c->param[1], c->iValues[1],
					(int)c->param[2], c->iValues[2], (int)c->param[3], c->iValues[3],
					(int)c->param[4], c->iValues[4],
					(int)D_CLOCKWISE, (int)D_COUNTERCLOCKWISE );	
		}
		else
		{
			int dir1, dir2, dir3, dir4, dir5;
			if(c->param[0]==D_CLOCKWISE) dir1=0; else dir1=1;
			if(c->param[1]==D_CLOCKWISE) dir2=0; else dir2=1;
			if(c->param[2]==D_CLOCKWISE) dir3=0; else dir3=1;
			if(c->param[3]==D_CLOCKWISE) dir4=0; else dir4=1;
			if(c->param[4]==D_CLOCKWISE) dir5=0; else dir5=1;

			fOpenLoopPWM_5CH(jnt, h, f, dir1, c->iValues[0], dir2, c->iValues[1],
						dir3, c->iValues[2], dir4, c->iValues[3],
						dir5, c->iValues[4] );

			sendCan(getSocket(h,jnt), f);
		}
	}
} 


void fOpenLoopPWM_2CH(int jnt, struct hubo_param *h, struct can_frame *f,
			int dir0, int duty0, int dir1, int duty1)
{
	f->can_id	= CMD_TXDF;
	
	__u8 data[8];
	f->data[0]	= getJMC(h,jnt);
	f->data[1]	= H_OPENLOOP_PWM;
	f->data[2]	= 0x01; // Pulse according to duty cycle
				//	Alternative value is 0x00 which enforces
				//	zero duty in order to stop the motor
	f->data[3]	= (uint8_t)dir0;
	f->data[4]	= (uint8_t)duty0;
	f->data[5]	= (uint8_t)dir1;
	f->data[6]	= (uint8_t)duty1;
	f->data[7]	= H_BLANK;

	f->can_dlc	= 8;
}

void fOpenLoopPWM_3CH(int jnt, struct hubo_param *h, struct can_frame *f,
			int dir0, int dt0, int dir1, int dt1, int dir2, int dt2)
{
	f->can_id	= CMD_TXDF;
	
	__u8 data[6];
	f->data[0]	= getJMC(h,jnt);
	f->data[1]	= H_OPENLOOP_PWM;
	f->data[2]	= 0x01; // Pulse according to duty cycle
				//	Alternative value is 0x00 which enforces
				//	zero duty in order to stop the motor
	f->data[3]	= duty_to_byte(dir0, dt0);
	f->data[4]	= duty_to_byte(dir1, dt1);
	f->data[5]	= duty_to_byte(dir2, dt2); 

	f->can_dlc	= 6;
}

void fOpenLoopPWM_5CH(int jnt, struct hubo_param *h, struct can_frame *f,
			int dir0, int dt0, int dir1, int dt1, int dir2, int dt2,
			int dir3, int dt3, int dir4, int dt4)
{
	f->can_id	= CMD_TXDF;
	
	__u8 data[8];
	f->data[0]	= getJMC(h,jnt);
	f->data[1]	= H_OPENLOOP_PWM;
	f->data[2]	= 0x01; // Pulse according to duty cycle
				//	Alternative value is 0x00 which enforces
				//	zero duty in order to stop the motor
	f->data[3]	= duty_to_byte(dir0, dt0);
	f->data[4]	= duty_to_byte(dir1, dt1);
	f->data[5]	= duty_to_byte(dir2, dt2); 
	f->data[6]	= duty_to_byte(dir3, dt3);
	f->data[7]	= duty_to_byte(dir4, dt4);

	f->can_dlc	= 8;
}

void hSetControlMode(int jnt, struct hubo_param *h, struct can_frame *f, hubo_d_param_t mode)
{
	switch (mode)
	{
		case D_POSITION:
			fSetControlMode(jnt, h, f, 0); // 0 > Position control
			sendCan(getSocket(h,jnt),f);
			break;
		case D_CURRENT:
			fSetControlMode(jnt, h, f, 1); // 1 > Current control
			sendCan(getSocket(h,jnt),f);
			break;
		default:
			fprintf(stderr,"Invalid Control Mode: %d\n\t"
					"Must use: D_POSITION (%d) or D_CURRENT (%d)\n",
					(int)mode, (int)D_POSITION, (int)D_CURRENT);
			break;
	}
}


void fSetControlMode(int jnt, struct hubo_param *h, struct can_frame *f, int mode)
{
	f->can_id	= CMD_TXDF;

	__u8 data[3];
	f->data[0]	= getJMC(h,jnt);
	f->data[1]	= H_SET_CTRL_MODE;
	f->data[2]	= (uint8_t)mode;

	f->can_dlc	= 3;
}


void hGetEncValue(int jnt, struct hubo_param *h, struct can_frame *f) { ///> make can frame for getting the value of the Encoder
	fGetEncValue( jnt, h, f);
	sendCan(getSocket(h,jnt), f);
}
void fGetEncValue(int jnt, struct hubo_param *h, struct can_frame *f) { ///> make can frame for getting the value of the Encoder
	f->can_id       = CMD_TXDF;     // Set ID
	__u8 data[3];
	f->data[0]      = h->joint[jnt].jmc;
	f->data[1]	= H_GET_ENCODER;
	f->data[2]	= H_BLANK;
	f->can_dlc = 3; //= strlen( data );     // Set DLC
}

void fGetBoardStatusAndErrorFlags(int jnt, struct hubo_ref *r, struct hubo_param *h, struct can_frame *f);
{
	f->can_id	= CMD_TXDF;
	__u8 data[3];
	f->data[0]	= h->joint[jnt].jmc;
	f->data[1]	= H_GET_STATUS;
	f->data[2]	= H_BLANK;
	f->can_dlc	= 3;
}

void hGetCurrentValue(int jnt, struct hubo_param *h, struct can_frame *f) { ///> make can frame for getting the motor current in amps (10mA resolution)
	fGetCurrentValue( jnt, h, f);
	sendCan(getSocket(h,jnt), f);
}


void hSetBeep(int jnt, struct hubo_param *h, struct can_frame *f, double beepTime) {
	fSetBeep(jnt, h, f, beepTime);
	sendCan(getSocket(h,jnt), f);
}

void fSetBeep(int jnt, struct hubo_param *h, struct can_frame *f, double beepTime) {
{
	f->can_id 	= CMD_TXDF;	// Set ID

	__u8 data[3];
	f->data[0] 	= getJMC(h,jnt);	// BNO
	f->data[1]	= H_BEEP;		// beep
	f->data[2]	= (uint8_t)floor(beepTime/0.1);
	
	f->can_dlc = 3;
}


// 16  home
void fGotoLimitAndGoOffset(int jnt, struct hubo_param *h, struct can_frame *f)
{
	f->can_id 	= CMD_TXDF;
	
	__u8 data[8];
	f->data[0] 	= getJMC(h,jnt);
	f->data[1] 	= H_HOME;
	f->data[2] 	= (((uint8_t)h->joint[jnt].motNo+1) << 4)|2; // set /DT high
	f->data[3]  	= H_BLANK;
	f->data[4]  	= H_BLANK;
	f->data[5]  	= H_BLANK;
	f->data[6]  	= H_BLANK;
	f->data[7]  	= H_BLANK;
	
	f->can_dlc	= 8;
}


void hSetDeadZone(int jnt, struct hubo_param *h, struct can_frame *f, int deadzone)
{
	if(deadzone>=0&&deadzone<=255)
	{
		fSetDeadZone(jnt, h, f, deadzone);
		sendCan(getSocket(h,jnt),f);
	}
	else
		fprintf(stderr,"Invalid value for deadzone: %d\n\t"
					"Range: [0,255]\n", deadzone);
}

void fSetDeadZone(int jnt, struct hubo_param *h, struct can_frame *f, int deadzone)
{
	f->can_id	= CMD_TDXF;

	__u8 data[3];
	f->data[0]	= getJMC(h,jnt);
	f->data[1]	= (uint8_t)(H_SET_DEADZONE + h->joint[jnt].motNo+1); // TODO: Find out if +1 is correct
	f->data[2]	= (uint8_t)deadzone;

	f->can_dlc	= 3;
}

void fGetBoardParameters(int jnt, struct hubo_param *h, struct can_frame *f, int parm)
{
	f->can_id	= CMD_TDXF;
	
	__u8 data[3];
	f->data[0]	= getJMC(h,jnt);
	f->data[1]	= H_GET_PARAMETERS;
	f->data[2]	= (uint8_t)parm;

	f->can_dlc	= 3;
}


void hSetHomeSearchParams( struct hubo_board_cmd *c, struct hubo_param *h, struct can_frame *f )
{
	unsigned int dir, offset;
	
	switch (c->param[0]){
		default:
			fprintf(stderr, "Invalid parameter for Limit switch search direction: %d\n\t"
						"Defaulting to D_CLOCKWISE (%d)\n",
						(int)c->param[0], (int)D_CLOCKWISE);
		case D_CLOCKWISE:
			dir = 0; break;
		case D_COUNTERCLOCKWISE:
			dir = 1; break;
	}

	offset = (unsigned int)c->iValues[1];

	fSetHomeSearchParams(c->joint, h, f, c->iValues[0], dir, offset);

	sendCan(getSocket(h,c->joint),f);
}

void fSetHomeSearchParams(int jnt, struct hubo_param *h, struct can_frame *f, int limit,
				unsigned int dir, unsigned int offset)
{
	f->can_id	= CMD_TDXF;
	
	__u8 data[8];
	f->data[0]	= getJMC(h,jnt);
	f->data[1]	= (uint8_t)(H_SET_HOME_PARAM + h->joint[jnt].motNo+1); // TODO: Find out if +1 is correct
	f->data[2]	= (uint8_t)limit;
	f->data[3]	= (uint8_t)dir;
	f->data[4]	= num_to_bytes(offset,1);
	f->data[5]	= num_to_bytes(offset,2);
	f->data[6]	= num_to_bytes(offset,3);
	f->data[7]	= num_to_bytes(offset,4);

	f->can_dlc	= 8;
}

void hSetEncoderResolution(struct hubo_board_cmd *c, struct hubo_param *h, struct can_frame *f)
{
	uint16_t res;
	
	switch(c->param[0]){
		case D_CLOCKWISE:
			res = 1 << 16; break;
		case default:
			fprintf(stderr, "Invalid parameter for Motor Direction: %d\n\t"
						"Defaulting to D_CLOCKWISE (%d)\n",
						(int)c->param[0], (int)D_CLOCKWISE);
		case D_CLOCKWISE:
			res = 0 << 16; break;
	}
	
	switch(c->param[1]){
		case default:
			fprintf(stderr, "Invalid Parameter for Auto-Scale: %d\n\t"
						"Defauling to D_ENABLE (%d)\n", (int)c->param[1], D_ENABLE);
		case D_ENABLE:
			res = res | (1<<15); break;
		case D_DISABLE:
			res = res | (0<<15); break;
	}

	if(c->iValues[0] >= 16384 || c->iValues[0]<0) // Cannot exceed 14 bits. 2^14 = 16384
	{
		fprintf(stderr, "Encoder resolution value out of range: %d\n\t"
					"Defaulting to max resolution (16383)\n", (int)c->iValues[0] );
		res = res | 16383;
	}
	else
		res = res | c->iValues[0];

	fSetEncoderResolution(c->joint, h, f, res);
	sendCan(getSocket(h,c->joint),f);
}

void fSetEncoderResolution(int jnt, struct hubo_param *h, struct can_frame *f, int res)
{
	f->can_id	= CMD_TDXF;
	
	__u8 data[4];
	f->data[0]	= getJMC(h,jnt);
	f->data[1]	= (uint8_t)(H_SET_ENC_RES + h->joint[jnt].motNo+1); // TODO: Find out if +1 is correct
	f->data[2]	= num_to_bytes(res,1); // TODO: Have handler construct res properly
	f->data[3]	= num_to_bytes(res,2);

	f->can_dlc	= 4;
}

void hSetMaxAccVel(int jnt, struct hubo_param *h, struct can_frame *f, int maxAcc, int maxVel)
{
	if( maxAcc >= 65536 || maxAcc <=0 )
		fprintf("Max Acceleration value out of bounds: %d\n\t"
				"Maximum value is 65535\n", maxAcc);
	else if( maxVel < 65536 && maxVel > 0 )
	{
		fSetMaxAccVel(jnt, h, f, maxAcc, maxVel);
		sendCan(getSocket(h,jnt),f);
	}
	else
		fprintf("Max Velocity value is out of bounds: %d\n\t"
				"Maximum value is 65535\n", maxVel);
}

void fSetMaxAccVel(int jnt, struct hubo_param *h, struct can_frame *f, int maxAcc, int maxVel)
{
	f->can_id	= CMD_TDXF;
	
	__u8 data[6];
	f->data[0]	= getJMC(h,jnt);
	f->data[1]	= (uint8_t)(H_SET_MAX_ACC_VEL + h->joint[jnt].motNo+1); // TODO: Find out if +1 is correct
	f->data[2]	= num_to_bytes(maxAcc,1);
	f->data[3]	= num_to_bytes(maxAcc,2);
	f->data[4]	= num_to_bytes(maxVel,1);
	f->data[5]	= num_to_bytes(maxVel,2);

	f->can_dlc	= 6;
}

void hSetLowerPosLimit(struct hubo_board_cmd *c, struct hubo_param *h, struct can_frame *f)
{
	uint8_t enable, update;
	
	switch(c->param[0]){
		case D_UPDATE:
			update = 1; break;
		default:
			fprintf(stderr, "Lower position limit update parameter invalid: %d\n\t"
						"Defaulting to D_IGNORE (%d)\n", (int)c->param[0], (int)D_IGNORE);
		case D_IGNORE:
			update = 0; break;
	}
	
	switch(c->param[1]){
		default:
			fprintf(stderr, "Lower position limit enabling parameter invalid: %d\n\t"
						"Defaulting to D_ENABLE (%d)\n", (int)c->param[1], (int)D_ENABLE);
		case D_ENABLE:
			enable = 1; break;
		case D_DISABLE:
			enable = 0; break;
	}

	fSetLowerPosLimit(c->joint, h, f, enable, update, c->iValues[0]);
	sendCan(getSocket(h,c->joint),f);
}

void fSetLowerPosLimit(int jnt, struct hubo_param *h, struct can_frame *f, int enable, int update, int limit)
{
	f->can_id	= CMD_TDXF;
	
	__u8 data[7];
	f->data[0]	= getJMC(h,jnt);
	f->data[1]	= (uint8_t)(H_SET_LOW_POS_LIM + h->joint[jnt].motNo+1); // TODO: Find out if +1 is correct
	f->data[2]	= (uint8_t)( (update << 1) | (enable) );
	f->data[3]	= num_to_bytes(limit,1);
	f->data[4]	= num_to_bytes(limit,2);
	f->data[5]	= num_to_bytes(limit,3);
	f->data[6]	= num_to_bytes(limit,4);

	f->can_dlc	= 7;
}



void hSetUpperPosLimit(struct hubo_board_cmd *c, struct hubo_param *h, struct can_frame *f)
{
	uint8_t enable, update;
	
	switch(c->param[0]){
		case D_UPDATE:
			update = 1; break;
		default:
			fprintf(stderr, "Upper position limit update parameter invalid: %d\n\t"
						"Defaulting to D_IGNORE (%d)\n", (int)c->param[0], (int)D_IGNORE);
		case D_IGNORE:
			update = 0; break;
	}
	
	switch(c->param[1]){
		default:
			fprintf(stderr, "Upper position limit enabling parameter invalid: %d\n\t"
						"Defaulting to D_ENABLE (%d)\n", (int)c->param[1], (int)D_ENABLE);
		case D_ENABLE:
			enable = 1; break;
		case D_DISABLE:
			enable = 0; break;
	}

	fSetUpperPosLimit(c->joint, h, f, enable, update, c->iValues[0]);
	sendCan(getSocket(h,c->joint),f);
}

void fSetUpperPosLimit(int jnt, struct hubo_param *h, struct can_frame *f, int enable, int update, int limit)
{
	f->can_id	= CMD_TDXF;
	
	__u8 data[7];
	f->data[0]	= getJMC(h,jnt);
	f->data[1]	= (uint8_t)(H_SET_UPP_POS_LIM + h->joint[jnt].motNo+1); // TODO: Find out if +1 is correct
	f->data[2]	= (uint8_t)( (update << 1) | (enable) );
	f->data[3]	= num_to_bytes(limit,1);
	f->data[4]	= num_to_bytes(limit,2);
	f->data[5]	= num_to_bytes(limit,3);
	f->data[6]	= num_to_bytes(limit,4);

	f->can_dlc	= 7;
}

void hSetHomeAccVel(struct hubo_board_cmd *c, struct hubo_param *h, struct can_frame *f)
{
	int mode;

	switch(c->param[0]){
		default:
			fprintf("Invalid homing mode parameter: %d\n\t"
					"Must use D_SWITCH_AND_INDEX (%d), D_SWITCH (%d),\n\t"
					"or D_JAM_LIMIT (%d)\n"
				"Defaulting to D_SWITCH_AND_INDEX\n", (int)c->param[0],
					(int)D_SWITCH_AND_INDEX, (int)D_SWITCH, (int)D_JAM_LIMIT);
		case D_SWITCH_AND_INDEX:
			mode = 0; break;
		case D_SWITCH:
			mode = 1; break;
		case D_JAM_LIMIT:
			mode = 2; break;
	}

	fSetHomeAccVel(c->joint, h, f, (float)c->dValues[0], c->iValues[0], c->iValues[1],
			mode, c->iValues[2]);

	sendCan(getSocket(h,c->joint),f);
}

void fSetHomeAccVel(int jnt, struct hubo_param *h, struct can_frame *f, float mAcc, int mVelS,
			int mVelP, int mode, int mDuty)
{
	f->can_id	= CMD_TDXF;

	__u8 data[7];
	f->data[0]	= getJMC(h,jnt);
	f->data[1]	= (uint8_t)(H_SET_HOME_VEL_ACC + h->joint[jnt].motNo+1); // TODO: Find out if +1 is correct
	f->data[2]	= (uint8_t)(mAcc*100);
	f->data[3]	= (uint8_t)(mVelS);
	f->data[4]	= (uint8_t)(mVelP);
	if( mode==0 || mode==1 || mode=2 )
		f->data[5]	= (uint8_t)(mode);
	else
		f->data[5]	= 0x00;
	f->data[6]	= (uint8_t)(mDuty);

	f->can_dlc	= 7;
}

void hSetGainOverride(int jnt, struct hubo_param *h, struct can_frame *f, int gain0, int gain1, double dur)
{
	fSetGainOverride(jnt, h, f, gain0, gain1, (int)(dur*1000);
	sendCan(getSocket(h,jnt),f);
}


void fSetGainOverride(int jnt, struct hubo_param *h, struct can_frame *f, int gain0, int gain1, int duration)
{
	f->can_id	= CMD_TDXF;

	__u8 data[6];
	f->data[0]	= getJMC(h,jnt);
	f->data[1]	= H_GAIN_OVERRIDE;
	f->data[2]	= (uint8_t)(gain0);
	f->data[3]	= (uint8_t)(gain1);
	f->data[4]	= num_to_bytes(duration,1);
	f->data[5]	= num_to_bytes(duration,2);

	f->can_dlc = 6;
}

void hSetBoardNumber(int jnt, struct hubo_param *h, struct can_frame *f, int boardNum, int rate)
{
	fprintf(stdout, "WARNING: Changing board number %d to %d with baud rate %d\n",
			getJMC(h,jnt), boardNum, rate);
	fSetBoardNumber(jnt, h, f, boardNum, rate);
	sendCan(getSocket(h,jnt),f);
}


void fSetBoardNumber(int jnt, struct hubo_param *h, struct can_frame *f, int boardNum, int rate)
{
	f->can_id	= CMD_TDXF;
	
	__u8 data[4];
	f->data[0]	= getJMC(h,jnt);
	f->data[1]	= H_SET_BOARD_NUM;
	f->data[2]	= (uint8_t)(boardNum);
	f->data[3]	= (uint8_t)(rate);

	f->can_dlc = 4;
}

void hSetJamPwmLimits(struct hubo_board_cmd *c, struct hubo_param *h, struct can_frame *f)
{
	fprintf( stdout, "Changing Jam and PWM Saturation limits:\n\t
				Jam Duty: %d \t Sat Duty: %d\n\t
				Jam Time: %d \t Sat time: %d\n",
			c->iValues[0], c->iValues[1], c->dValues[0], c->dValues[1] );
	fSetJamPwmLimits(c->joint, h, f, (int)(c->dValues[0]*1000), (int)(c->dValues[1]*1000),
				c->iValues[0], c->iValues[1] );

	sendCan(getSocket(h,c->joint),f);
}

void fSetJamPwmLimits(int jnt, struct hubo_param *h, struct can_frame *f, int jamLimit, int pwmLimit,
			int lim_detection_duty, int jam_detection_duty )
{
	f->can_id	= CMD_TDXF;

	__u8 data[8];
	f->data[0]	= getJMC(h,jnt);
	f->data[1]	= H_SET_JAM_SAT_LIM;
	f->data[2]	= num_to_bytes(jamLimit,1);
	f->data[3]	= num_to_bytes(jamLimit,2);
	f->data[4]	= num_to_bytes(pwmLimit,1);
	f->data[5]	= num_to_bytes(pwmLimit,2);
	f->data[6]	= (uint8_t)(lim_detection_duty);
	f->data[7]	= (uint8_t)(jam_detection_duty);
	
	f->can_dlc = 8;
}

void hSetErrorBound(int jnt, struct hubo_param *h, struct can_frame *f, int inputDiffErr, int maxError,
			int tempError)
{
	fprintf(stdout, "Changing error bounds on board %d:\n\t"
			"Input Difference error: %d\n\t"
			"Maximum error: %d\n\t"
			"Max temperature: %d\n", jnt, inputDiffErr, maxError, tempError);

	fSetErrorBound(jnt, h, f, inputDiffErr, maxError, tempError);
	sendCan(getSocket(h,c->joint),f);
}

void fSetErrorBound(int jnt, struct hubo_param *h, struct can_frame *f, int inputDiffErr, int maxError,
			int tempError)
{
	f->can_id	= CMD_TDXF;
	
	__u8 data[8];
	f->data[0]	= getJMC(h,jnt);
	f->data[1]	= H_SET_ERR_BOUND;
	f->data[2] 	= num_to_bytes(inputDiffErr,1);
	f->data[3]	= num_to_bytes(inputDiffErr,2);
	f->data[4]	= num_to_bytes(maxError,1);
	f->data[5]	= num_to_bytes(maxError,2);
	f->data[6]	= num_to_bytes(tempError,1);
	f->data[7]	= num_to_bytes(tempError,2);

	f->can_dlc = 8;
}




void hGotoLimitAndGoOffset(int jnt, struct hubo_ref *r, struct hubo_param *h, struct hubo_state *s,
				struct can_frame *f)
{
	fGotoLimitAndGoOffset(jnt, h, f);
	//sendCan(hubo_socket[getCAN(h,jnt)], f);
	sendCan( getSocket(h,jnt), f );
	r->ref[jnt] = 0;
	s->joint[jnt].zeroed = true;

	hubo_noRefTimeAll = hubo_home_noRef_delay;
}

void hGotoLimitAndGoOffsetAll(struct hubo_ref *r, struct hubo_param *h, struct hubo_state *s, struct can_frame *f) {
	int i = 0;
	for(i = 0; i < HUBO_JOINT_COUNT; i++) {
		if(s->joint[i].active == true) {
			hGotoLimitAndGoOffset(i, r, h, s, f);
		}
	}
}

void hInitializeBoard(int jnt, struct hubo_param *h, struct can_frame *f) {
	fInitializeBoard(jnt, h, f);
	sendCan(getSocket(h,jnt), f);
	//readCan(hubo_socket[h->joint[jnt].can], f, 4);	// 8 bytes to read and 4 sec timeout
	readCan(getSocket(h,jnt), f, HUBO_CAN_TIMEOUT_DEFAULT*100);	// 8 bytes to read and 4 sec timeout
}

void fInitializeBoard(int jnt, struct hubo_param *h, struct can_frame *f) {
	f->can_id 	= CMD_TXDF;

	__u8 data[2];
	f->data[0] 	= getJMC(h,jnt);
	f->data[1] 	= H_INIT_BOARD;

	f->can_dlc	= 2;
}

void hInitializeBoardAll( struct hubo_param *h, struct hubo_state *s, struct can_frame *f ) {
	///> Initilizes all boards
	int i = 0;
	for(i = 0; i < HUBO_JOINT_COUNT; i++) {
		if(s->joint[i].active == true) {
			hInitializeBoard(i, h, f);
		}
	}
}

void hSetEncRef(int jnt, struct hubo_ref *r, struct hubo_param *h, struct can_frame *f) {
//	setEncRef(jnt,r, h);
	fSetEncRef(jnt, r, h, f);
	sendCan(getSocket(h,jnt), f);
//	readCan(h->socket[h->joint[jnt].can], f, 4);	// 8 bytes to read and 4 sec timeout
}

void hIniAll(struct hubo_ref *r, struct hubo_param *h, struct hubo_state *s, struct can_frame *f) {
// --std=c99
		printf("2\n");
	int i = 0;
	for( i = 0; i < HUBO_JOINT_COUNT; i++ ) {
		if(s->joint[i].active) {
			hInitializeBoard(i, r, h, f);
			printf("%i\n",i);
		}
	}
}

void hMotorDriverOnOff(int jnt, struct hubo_param *h, struct can_frame *f, hubo_d_param_t onOff) {
	if(onOff == D_ENABLE) { // turn on FET
		fEnableMotorDriver(jnt, h, f);
		sendCan(getSocket(h,jnt), f); 
		
	}
	else if(onOff == D_DISABLE) { // turn off FET
		fDisableMotorDriver(jnt, h, f);
		sendCan(getSocket(h,jnt), f); }
	else
		fprintf(stderr,"FET Switch Error: Invalid param[0]\n\t
					Must be D_ENABLE (%d) or D_DISABLE (%d)",
				D_ENABLE, D_DISABLE);
}

void fEnableMotorDriver(int jnt, struct hubo_param *h, struct can_frame *f) {
	f->can_id 	= CMD_TXDF;

	__u8 data[3];
	f->data[0] 	= getJMC(h,jnt);
	f->data[1] 	= H_SWITCH_DRIVER;
	f->data[2] 	= 0x01; // Turn on
	
	f->can_dlc	= 3;
}

void fDisableMotorDriver(int jnt, struct hubo_param *h,  struct can_frame *f) {
	f->can_id 	= CMD_TXDF;
	
	__u8 data[3];
	f->data[0] 	= getJMC(h,jnt);
	f->data[1] 	= H_SWITCH_DRIVER;
	f->data[2] 	= 0x00; // Turn off
	
	f->can_dlc	= 3;
}

void hFeedbackControllerOnOff(int jnt, struct hubo_param *h, struct can_frame *f, hubo_d_param_t onOff) {
	if(onOff == D_ENABLE) { // turn on 
		fEnableFeedbackController(jnt, h, f);
		sendCan(getSocket(h,jnt), f); }
	else if(onOff == D_DISABLE) { // turn off FET
		fDisableFeedbackController(jnt, h, f);
		sendCan(getSocket(h,jnt), f); }
	else
		fprintf(stderr, "Controller Switch Error: Invalid param[0] (%d)\n\t
					Must be D_ENABLE (%d) or D_DISABLE (%d)", c->param[0],
					D_ENABLE, D_DISABLE);
}

void fEnableFeedbackController(int jnt, struct hubo_param *h, struct can_frame *f)
{
	f->can_id 	= CMD_TXDF;

	__u8 data[2];
	f->data[0] 	= getJMC(h,jnt);
	f->data[1] 	= H_MOTOR_CTRL_ON;
	
	f->can_dlc	= 2;
}

void fDisableFeedbackController(int jnt, struct hubo_param *h, struct can_frame *f)
{
	f->can_id 	= CMD_TXDF;

	__u8 data[2];
	f->data[0] 	= getJMC(h,jnt);
	f->data[1] 	= H_MOTOR_CTRL_OFF;
	
	f->can_dlc	= 2;
}

void hResetEncoderToZero(int jnt, struct hubo_param *h, struct hubo_state *s, struct can_frame *f) {
	fResetEncoderToZero(jnt, h, f);
	sendCan(getSocket(h,jnt), f);
	s->joint[jnt].zeroed == true;		// need to add a can read back to confirm it was zeroed
}

void fResetEncoderToZero(int jnt, struct hubo_param *h, struct can_frame *f) {
	/* Reset Encoder to Zero (REZ: 0x06)
	CH: Channel No.
	CH= 0 ~ 4 according to the board.
	CH= 0xF selects ALL Channel
	Action:
	1. Set encoder(s) to Zero.
	2. Initialize internal parameters.
	3. Reset Fault and Error Flags.
	*/

	f->can_id 	= CMD_TXDF;	// Set ID

	__u8 data[3];
	f->data[0] 	= getJMC(h,jnt);
	f->data[1]	= S_ENC_ZERO;
	f->data[2] 	= h->joint[jnt].motNo;
	
	f->can_dlc = 3; //= strlen( data );	// Set DLC
}

void huboMessage(struct hubo_ref *r, struct hubo_param *h, struct hubo_state *s, struct hubo_board_cmd *c, struct can_frame *f) {
	/* gui for controling basic features of the hubo  */
//	printf("hubo-ach - interface 2012-08-18\n");
	size_t fs;
	int status = 0;
	while ( status == 0 | status == ACH_OK | status == ACH_MISSED_FRAME ) {
		/* get oldest ach message */
		//status = ach_get( &chan_hubo_ref, &c, sizeof(c), &fs, NULL, 0 );
		//status = ach_get( &chan_hubo_board_cmd, c, sizeof(*c), &fs, NULL, ACH_O_LAST );
		status = ach_get( &chan_hubo_board_cmd, c, sizeof(*c), &fs, NULL, 0 );
	//printf("here2 h = %f status = %i c = %d v = %f\n", h->joint[0].ref, status,(uint16_t)c->cmd[0], c->val[0]);
//	printf("here2 h = %f status = %i c = %d v = %f\n", h->joint[0].ref, status,(uint16_t)c->cmd[0], c->val[0]);
		if( status == ACH_STALE_FRAMES) {
			break; }
		else {
		//hubo_assert( sizeof(c) == fs );

//		if ( status != 0 & status != ACH_OK & status != ACH_MISSED_FRAME ) {
//			break; }
			switch (c->type) {
				case D_JMC_INITIALIZE_ALL:
					hInitializeBoardAll( h, s, f );
					break;
				case D_JMC_INITIALIZE:
					hInitializeBoard( c->joint, h, f );
					break;
				case D_FET_SWITCH:
					hMotorDriverOnOff( c->joint, h, f, c->param[0] );
					break;
				case D_CTRL_SWITCH:
					hFeedbackControllerOnOff( c->joint, h, f, c->param[0] );
					break;
				case D_ZERO_ENCODER:
					hResetEncoderToZero( c->joint, h, s, f );
					break;
				case D_JMC_BEEP:
					hSetBeep( c->joint, h, f, c->dValue[0] );
					break;
				case D_GOTO_HOME_ALL:
					hGotoLimitAndGoOffsetAll( r, h, s, f );
					break;
				case D_GOTO_HOME:
					hGotoLimitAndGoOffset( c->joint, r, h, s, f );
					break;
				case D_JMC_ALARM:
					hSetAlarm( c->joint, h, f, c->param[0] );
					break;
				case D_SET_POS_GAIN:
					hSetPosGain( c, h, f );
					break;
				case D_SET_CUR_GAIN:
					hSetCurGain( c, h, f );
					break;
				case D_OPENLOOP_PWM:
					hOpenLoopPWM( c, h, f );
					break;
				case D_CTRL_ON:
					hFeedbackControllerOnOff( c->joint, h, f, D_ENABLE);
					break;
				case D_CTRL_OFF:
					hFeedbackControllerOnOff( c->joint, h, f, D_DISABLE);
					break;
				case D_CTRL_MODE:
					hSetControlMode( c->joint, h, f, c->param[0] );
					break;
				case D_SET_DEAD_ZONE:
					hSetDeadZone( c->joint, h, f, c->iValue[0] );
					break;
				case D_SET_HOME_PARAMS:
					hSetHomeSearchParams( c, h, f );
					break;
				case D_SET_ENC_RESOLUTION:
					hSetEncoderResolution( c, h, f );
					break;
				case D_SET_MAX_ACC_VEL:
					hSetMaxAccVel( c->joint, h, f, c->iValues[0], c->iValues[1] );
					break;
				case D_SET_LOW_POS_LIM:
					hSetLowerPosLimit( c, h, f );
					break;
				case D_SET_UPP_POS_LIM:
					hSetUpperPosLimit( c, h, f );
					break;
				case D_SET_HOME_VEL_ACC:
					hSetHomeAccVel( c, h, f);
					break;
				case D_SET_GAIN_SCALE:
					hSetGainOverride(c->joint, h, f, c->iValues[0],
							c->iValues[1], c->dValues[0]); break;
				case D_SET_BOARD_NUM:
					hSetBoardNumber(c->joint, h, f, c->iValues[0], c->iValues[1]);
					break;
				case D_SET_JAM_SAT_LIMIT:
					hSetJamPwmLimits( c, h, f );
					break;
				case D_SET_ERR_BOUND:
					hSetErrorBound( c->joint, h, f, c->iValues[0], c->iValues[1],
							c->iValues[2] ); break;
				case D_GET_BOARD_PARAMS:
					hGetBoardParams( c, h, f );
					break;
				default:
					fprintf(stderr,"Unrecognized command type: %d",c->type);
					break;
			}
		}
//		printf("c = %i\n",c->cmd[0]);
	}
}


double enc2rad(int jnt, int enc, struct hubo_param *h) {
	struct hubo_joint_param *p = &h->joint[jnt];
        return (double)(enc*(double)p->drive/(double)p->driven/(double)p->harmonic/(double)p->enc*2.0*M_PI);
}


int decodeFrame(struct hubo_state *s, struct hubo_param *h, struct can_frame *f) {
	int fs = (int)f->can_id;
	
	/* Current and Temp */
	if( (fs >= SETTING_BASE_RXDF) & (fs < (SETTING_BASE_RXDF+0x60))) {
		int jmc = fs-SETTING_BASE_RXDF;		// find the jmc value
		int i = 0;
		int jnt0 = h->driver[jmc].jmc[0];     // jmc number
		int motNo = h->joint[jnt0].numMot;     // motor number   
		double current = 0;				// motor current
		double temp = 0;			// temperature
		if(motNo == 2 | motNo == 1) {
			for( i = 0; i < motNo; i++) {
				current = f->data[0+i*1];
				current = current/100.0;
				temp = f->data[3];
				temp = temp/100.0;
				int jnt = h->driver[jmc].jmc[i];          // motor on the same drive
				s->joint[jnt].cur = current;
				s->joint[jnt].tmp = temp;

				
			}
		}


	}
	/* Return Motor Position */
	else if( (fs >= ENC_BASE_RXDF) & (fs < CUR_BASE_RXDF) ) {
		int jmc = fs-ENC_BASE_RXDF;
		int i = 0;
		int jnt0 = h->driver[jmc].jmc[0];     // jmc number
		int motNo = h->joint[jnt0].numMot;     // motor number   
		int32_t enc = 0;
		int16_t enc16 = 0;			// encoder value for neck and fingers
		if(motNo == 2 | motNo == 1){
			for( i = 0; i < motNo; i++ ) {
				enc = 0;
				enc = (enc << 8) + f->data[3 + i*4];
				enc = (enc << 8) + f->data[2 + i*4];
				enc = (enc << 8) + f->data[1 + i*4];
				enc = (enc << 8) + f->data[0 + i*4];
				int jnt = h->driver[jmc].jmc[i];          // motor on the same drive
				s->joint[jnt].pos =  enc2rad(jnt,enc, h);
			}
		}

		else if( motNo == 3 ) { 	// neck
			for( i = 0; i < motNo; i++ ) {
				enc16 = 0;
				enc16 = (enc << 8) + f->data[1 + i*4];
				enc16 = (enc << 8) + f->data[0 + i*4];
				int jnt = h->driver[jmc].jmc[i];          // motor on the same drive
				s->joint[jnt].pos =  enc2rad(jnt,enc16, h);
			}
		}
			
		else if( motNo == 5 & f->can_dlc == 6 ) {
			for( i = 0; i < 3 ; i++ ) {
				enc16 = 0;
				enc16 = (enc << 8) + f->data[1 + i*4];
				enc16 = (enc << 8) + f->data[0 + i*4];
				int jnt = h->driver[jmc].jmc[i];          // motor on the same drive
				s->joint[jnt].pos =  enc2rad(jnt,enc16, h);
			}
		}
			
		else if( motNo == 5 & f->can_dlc == 4 ) {
			for( i = 0; i < 2; i++ ) {
				enc16 = 0;
				enc16 = (enc << 8) + f->data[1 + i*4];
				enc16 = (enc << 8) + f->data[0 + i*4];
				int jnt = h->driver[jmc].jmc[i];          // motor on the same drive
				s->joint[jnt].pos =  enc2rad(jnt,enc16, h);
			}
		}
	
	}
	return 0;
}


int main(int argc, char **argv) {

	// Parse user input
	int vflag = 0;
	int c;

	int i = 1;
	while(argc > i) {
		if(strcmp(argv[i], "-d") == 0) {
			debug = 1;
		}
		if(strcmp(argv[i], "-v") == 0){
			vflag = 1;
		}
		if(strcmp(argv[i], "-p") == 0){ // p stands for print. We can probably come up with
			verbose = 1;		// a better letter.
		}
		i++;
	}

	// Daemonize
	hubo_daemonize();


/*
	// RT
	struct sched_param param;
	// Declare ourself as a real time task

	param.sched_priority = MY_PRIORITY;
	if(sched_setscheduler(0, SCHED_FIFO, &param) == -1) {
		perror("sched_setscheduler failed");
		exit(-1);
	}

	// Lock memory 

	if(mlockall(MCL_CURRENT|MCL_FUTURE) == -1) {
		perror("mlockall failed");
		exit(-2);
	}

	// Pre-fault our stack
	stack_prefault();
*/

	// Initialize Hubo Structs
	struct hubo_ref H_ref;
        struct hubo_board_cmd H_cmd;
        struct hubo_state H_state;
        struct hubo_param H_param;
        memset( &H_ref,   0, sizeof(H_ref));
        memset( &H_cmd,  0, sizeof(H_cmd));
        memset( &H_state, 0, sizeof(H_state));
        memset( &H_param, 0, sizeof(H_param));

	// set joint parameters for Hubo
	setJointParams(&H_param, &H_state);

	// open hubo reference
	int r = ach_open(&chan_hubo_ref, HUBO_CHAN_REF_NAME, NULL);
	hubo_assert( ACH_OK == r );

	// open hubo state
	r = ach_open(&chan_hubo_state, HUBO_CHAN_STATE_NAME, NULL);
	hubo_assert( ACH_OK == r );

	// initilize control channel
	r = ach_open(&chan_hubo_board_cmd, HUBO_CHAN_INIT_CMD_NAME, NULL);
	hubo_assert( ACH_OK == r );

	openAllCAN( vflag );
	ach_put(&chan_hubo_ref, &H_ref, sizeof(H_ref));
        ach_put(&chan_hubo_board_cmd, &H_cmd, sizeof(H_cmd));
        ach_put(&chan_hubo_state, &H_state, sizeof(H_state));

	// set default values for H_ref in ach	
//	setPosZeros();

	// set default values for H_cmd in ach
	// this is not working. Not sure if it's really needed
	// since the structs get initialized with zeros when instantiated
//	setConsoleFlags();	


	// run hubo main loop
	huboLoop(&H_param);

	hubo_daemon_close();
	
	return 0;
}

uint8_t num_to_bytes(double d, int index)
{
	return (uint8_t)(d%pow(256,index)/pow(256,i-1));
}

uint8_t num_to_bytes(int d, int index)
{
	return (uint8_t)(d%pow(256,index)/pow(256,i-1));
}

uint8_t duty_to_byte(int dir, int duty)
{
	return (uint8_t)(duty | dir<<8);
}

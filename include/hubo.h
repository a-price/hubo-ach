/* -*-	indent-tabs-mode:t; tab-width: 8; c-basic-offset: 8  -*- */
#ifndef HUBO_PRIMARY_H
#define HUBO_PRIMARY_H
#include "hubo/canID.h"
#include "hubo-daemonID.h"


//888888888888888888888888888888888888888888
//---------[Prerequisites for ACH]----------
#include <stdint.h>
#include <time.h>
#include <string.h>
#include <pthread.h>
#include <sys/stat.h>
#include <ach.h>
//888888888888888888888888888888888888888888






//888888888888888888888888888888888888888888
//-----[Static Definitions and Offsets]-----
//888888888888888888888888888888888888888888


/* Tx Message ID */

// Motor Command Message ID
#define txMoCmdId		0x01

// Sensor Command Message ID
#define txSeCmdId		0x02

// Reference Message ID 	(txReMsID + BNO)
#define txReMesId		0x10



/* Rx Message ID */

// FT sensor data Message ID 		(rxFtMsgId + SBNO)
#define rxFtMsgId		0x40

// IMU and Tilt Sensor data Message ID 	(rxImuMsgId + SBNO)
#define rxImuMsgId		0x50

// Encoder Value Message ID 		(rxEncValId + BNO)
#define rxEncValId 		0x60

// Status Message ID 			(rxStaMsgId + BNO)
#define rxStaMsgId		0x150

// Board Information Message ID 	(rxBrdInfoId + BNO + BOFF)
#define rxBrdInfoId		0x190

// Board para and Current Message ID 	(rxBrdCurMsgId + BNO)
#define rxBrdCurMsgId 		0x1C0

// BNO = Board number
// SBNO = sensor board number SBNO=BNO-0x2F
// BOFF=0 	for BNO < 0x30
// BOFF=0x80 	for BNO >= 0x30


/* Joint Numbers/Index values */
#define		RHY		26		//	Right Hip Yaw
#define		RHR		27		//	Right Hip Roll
#define		RHP		28		//	Right Hip Pitch
#define		RKN		29		//	Right Knee Pitch
#define		RAP		30		//	Right Ankle Pitch
#define		RAR		31		//	Right Ankle Roll

#define		LHY		19		//	Left Hip Yaw
#define		LHR		20		//	Left Hip Roll
#define		LHP		21		//	Left Hip Pitch
#define		LKN		22		//	Left Knee Pitch
#define		LAP		23		//	Left Ankle Pitch
#define		LAR		24		//	Left Ankle Roll

#define		RSP		11		//	Right Shoulder Pitch
#define		RSR		12		//	Right Shoulder Roll
#define		RSY		13		//	Right Shoulder Yaw
#define		REB		14		//	Right Elbow Pitch
#define		RWY		15		// right wrist yaw
#define		RWR		16		// right wrist roll
#define		RWP		17		// right wrist Pitch

#define		LSP		4		//	Left Shoulder Pitch
#define		LSR		5		//	Left Shoulder Yaw
#define		LSY		6		//	Left Shoulder Roll
#define		LEB		7		//	Left Elbow Pitch
#define		LWY		8		// left wrist yaw
#define		LWR		9		// left wrist roll
#define		LWP		10		// left wrist pitch

#define		NKY		1		// neck yaw
#define		NK1		2		// neck 1
#define		NK2		3		// neck 2

#define		WST		0		//	Trunk Yaw

#define		RF1		32		//	Right Finger
#define		RF2		33		//	Right Finger
#define		RF3		34		//	Right Finger
#define		RF4		35		//	Right Finger
#define		RF5		36		//	Right Finger
#define		LF1		37		//	Left Finger
#define		LF2		38		//	Left Finger
#define		LF3		39		//	Left Finger
#define		LF4		40		//	Left Finger
#define		LF5		41		//	Left Finger

#define 	HUBO_CAN_CHAN_NUM	4	///> Number of CAN channels avaliable

#define		HUBO_JOINT_COUNT	50	///< The size of the array
						///< For the joints
#define 	HUBO_JMC_COUNT		0X40	///< Numbher of jmc
#define 	HUBO_SENSOR_COUNT	10	///< Hubo Sensor Count


#define		HUBO_CHAN_REF_NAME       "hubo-ref"        ///> hubo ach channel
#define		HUBO_CHAN_BOARD_CMD_NAME "hubo-board-cmd"   ///> hubo console channel for ach
#define		HUBO_CHAN_STATE_NAME     "hubo-state"      ///> hubo state ach channel
#define		HUBO_CHAN_PARAM_NAME     "hubo-param"      ///> hubo param ach channel
#define 	HUBO_CHAN_REF_FILTER_NAME "hubo-ref-filter" ///> hubo reference with filter ach channel
#define		HUBO_CAN_TIMEOUT_DEFAULT 0.0005		///> Default time for CAN to time out
#define     HUBO_REF_FILTER_LENGTH   40


#define MAX_SAFE_STACK (1024*1024) /* The maximum stack size which is
				   guaranteed safe to access without
				   faulting */



typedef enum {
	HUBO_FT_R_HAND    = 0, ///< Index of right hand FT
	HUBO_FT_L_HAND    = 1, ///< Index of left hand FT
	HUBO_FT_R_FOOT    = 2, ///< Index of right foot FT
	HUBO_FT_L_FOOT    = 3, ///< Index of left foot FT
	HUBO_IMU0	  = 4, ///< Index of IMU0
	HUBO_IMU1	  = 5, ///< Index of IMU1
	HUBO_IMU2	  = 6  ///< Index of IMU2
} hubo_sensor_index_t;


#define HUBO_IMU_COUNT 3
typedef enum {
    TILT_R  = 0;
    TILT_L  = 1;
    IMU     = 2;
}

typedef enum {
	HUBO_REF_MODE_REF_FILTER    = 0, ///< Reference to reference filter
	HUBO_REF_MODE_REF           = 1, ///< Direct reference control
	HUBO_REF_MODE_COMPLIANT     = 2, ///< Compliant mode, sets ref to current encoder position. 
	HUBO_REF_MODE_ENC_FILTER    = 3  ///< Reference filter 
} hubo_mode_type_t;

#define RIGHT 0
#define LEFT 1


struct hubo_sensor_param {
	char name[5];		///< Name of sensor
	uint16_t sensNo;	///< Sensor number
	uint16_t can;		///< Can channel
	uint8_t active;		///< Active sensor
	uint16_t canID;		///< Can I.D. of the sensor
	uint16_t boardNo;	///< Sensor Board Nuber
};

struct hubo_joint_param {
	uint16_t motNo;		///< Onboard channel number
	uint16_t jntNo;		///< what overall number joint is it i.e. what RSP=23
	uint32_t refEnc; 	///< encoder reference
	uint16_t drive;		///< size of drive wheel
	uint16_t driven;	///< size of driven wheel
	uint16_t harmonic;	///< gear ratio of harmonic drive
	uint16_t enc;		///< encoder size
	uint8_t dir;		///< direction
	char name[4];		///< name
	uint16_t jmc;		///< motor controller number
	uint8_t can;		///< can channel
	uint8_t numMot;		///< number of motors
};

struct hubo_joint_state {
    double ref;
	double pos;     ///< actual position (rad)
	double cur;     ///< actual current (amps)
	double vel;     ///< actual velocity (rad/sec)
	double tmp;	///< temperature (dec C)
}

struct hubo_jmc_param {
	uint8_t joints[5]; // other motors on the same drive
};

struct hubo_param {
	struct hubo_joint_param joint[HUBO_JOINT_COUNT];
	struct hubo_jmc_param driver[HUBO_JMC_COUNT];	// motor driver conneciton info
};

struct hubo_imu {
	double angle_x;       ///< angular position around x (rad)
	double angle_y;       ///< angular position around y (rad)
	double w_x;    ///< rotational velocity in x (rad/s)
	double w_y;    ///< rotational velocity in y (rad/s)
	double w_z;    ///< rotational velocity in z (rad/s)
};

struct hubo_ft {
	double m_x;	///< Moment in X (Mx)
	double m_y;	///< Moment in Y (My)
	double f_z;	///< Force in Z (Fz)
};

struct hubo_joint_state {
	double pos;     	///< actual position (rad)
	double cur;     	///< actual current (amps)
	double vel;     	///< actual velocity (rad/sec)
	double heat;		///< Heat generated by motor in Joules >> NOTE: THIS IS NOT TEMPERATURE
				///	Temperature is stored in the "hubo_jmc_state driver[]"
	uint8_t active; 	///< checks if the joint is active or not
	uint8_t zeroed;		///< checks to see if the motor is zeroed
}hubo_joint_state_t;

struct hubo_board_msg {
	hubo_d_msg_t type;		// Type of message. Enumerated in hubo-daemonID.h
	int board;			// Board number which the message originates from
	hubo_d_param_t param[8];	// Parameters for the command. Enumerated in hubo-daemonID.h
	int values[8];			// Content of the message TODO: Figure out if 8 is sufficient
}hubo_board_msg_t;

struct hubo_jmc_state {
	double temp;	///< temperature (dec C)
//	hubo_d_param_t ctrlMode;
	// TODO: Add more things, such as whether an alarm is on
	//	 or whether motor control / FETs are on
}hubo_jmc_state_t;

struct hubo_ref {
	double ref[HUBO_JOINT_COUNT];	///< joint reference
	int status[HUBO_JOINT_COUNT];	///< 0:Good, 1:Frozen
	int mode[HUBO_JOINT_COUNT]; 	///< mode 0 = filter mode, 1 = direct reference mode
	int paused;
	struct timespec time;           ///< time message sent
}hubo_ref_t;

struct hubo_state {
	struct hubo_imu imu[HUBO_IMU_COUNT];	///< IMU
	struct hubo_ft ft[4];   ///< ft sensors
	struct hubo_joint_state joint[HUBO_JOINT_COUNT]; ///> Joint pos, velos, and current
	struct hubo_jmc_state driver[HUBO_JMC_COUNT];
    double time;
	int refWait;
}hubo_state_t;

struct jmcDriver{
	uint8_t jmc[5]; // other motors on the same drive
};
// Structure for sending board commands to the daemon
struct hubo_board_cmd {

	hubo_d_cmd_t type;		// Type of command. This value is REQUIRED. 
					// Enumerated in hubo-daemonID.h

	int joint;			// Target joint. If the message is meant for an entire board,
					// then fill this value with any joint number belonging to that
					// board. This value is REQUIRED (with a few exceptions).

	hubo_d_param_t param[8];	// Parameters for the command. Enumerated in hubo-daemonID.h
					// Note: This might or might not be used depending on the 
					// type of message. TODO: Figure out if 8 is sufficient (or excessive)

	int iValues[8];			// Integer values for the message. This may or may not be used
					// depending on the type of message. TODO: Figure out of 10 is sufficient
	
	double dValues[8];		// Double values for the message. This may or may not be used
					// depending on the type of message. TODO: Figure out of 8 is sufficient
};

struct hubo_param {
	struct hubo_joint_param joint[HUBO_JOINT_COUNT];	///< Joint param
	struct jmcDriver driver[HUBO_JMC_COUNT];		///< Motor driver param
	struct hubo_sensor_param sensor[HUBO_SENSOR_COUNT];	///< Sensor param
};

extern int hubo_debug;

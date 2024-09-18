#ifndef KRNX_H__
#define KRNX_H__

#ifdef WIN32
/* for windows */
#include <windows.h>
#else
/* for Linux */
typedef void*  HANDLE;
typedef char   BOOL;
typedef char   BOOLEAN;
typedef unsigned long   DWORD;
typedef unsigned int    UINT;
typedef unsigned long size_t;

#define WINAPI
#define TRUE 1
#define FALSE 0
#define NULL __null
#define CALLBACK
#ifdef DECLSPEC_IMPORT
#undef DECLSPEC_IMPORT
#endif
#define DECLSPEC_IMPORT
#endif

/************************************
*
*    Kawasaki Robot Controller Network Extension
*
*    Copyright (C) 2002 By
*    System Technology Development Center,
*    Kawasaki Heavy Industries, LTD.
*
*    This software is furnished under a license and may be used
*    and copied only in accordance with the terms of such license
*    and with the inclusion of the above copyright notice.
*
*    �t�@�C�����Fkrnx.h
*
*    $Header: /home/cvsroot/krnx_dll/krnx.h,v 1.33 2009/04/17 08:34:11 cvsuser Exp $
*
*    �@�\�^�ړI�F
*	Kawasaki Robot Network Extension API �֐��̃v���g�^�C�v�錾
*
*    �O���[�o���֐��F
*
*    �O���[�o���ϐ��F
*
*
*****************************************************
*/

#ifdef __cplusplus
extern "C"
{
#endif

#ifdef CAROTT3
#define KRNX_MAX_CONTROLLER	16	/* �ő�ڑ��R���g���[���� */
#else
#define KRNX_MAX_CONTROLLER	16	/* �ő�ڑ��R���g���[���� */
#endif
#define KRNX_MAX_ROBOT		8	/* 1�R���g���[�����T�|�[�g���郍�{�b�g�䐔 */
#define KRNX_MAX_PCPROG         3       /* 1�R���g���[�����T�|�[�g����PC�v���O������ */


/* �ő厲�� */
#define KRNX_MAXAXES	18      /* this should be same as the defined in AS system */

/* �ő�M���� */
#define KRNX_MAXSIGNAL  512

/* FX03496 a++ */
/* �ő�G���[�� */
#define KRNX_MAX_ERROR_LIST_SIZ 10
#define KRNX_MAX_ERROR_MSG_SIZ  160
/* FX03496 a-- */
	
#define INT_CBUF_SIZ	512

#define NON_COMPATIBLE
/*
 * !! �� �� !!
 *
 * krnx_dll�́A�M������512�_�܂ł����Ή����Ă��Ȃ�������
 * ��L #define NON_COMPATIBLE ���R�����g�A�E�g���邱�ƂŁA
 * 960�_�܂őΉ�����悤�ɂȂ�܂��B
 * �܂��A�C���^�v���^�p�̃o�b�t�@�T�C�Y��80�o�C�g�ł�����
 * �Ƃ��낪512�o�C�g�܂Ŋg������܂��B
 *
 * ���̃R�����g�́A�M��������уo�b�t�@�T�C�Y���g�������
 * ���Ȃ��]���̃o�[�W�����Ɗg�����ꂽ�V�o�[�W�����̌݊���
 * �Ƃꂽ���_�ō폜���Ă��������B
 * �܂��A������ NON_COMPATIBLE ���폜���Ă��������B
 *
 */

#define DI_MAX_SIGNAL		960
#define DO_MAX_SIGNAL		960
#define INTERNAL_MAX_SIGNAL	960

#define DBG_FVAL_NUM		16

/* �G���[�R�[�h */
#define KRNX_NOERROR            0
#define KRNX_E_BADARGS          (-0x1000)
#define KRNX_E_INTERNAL         (-0x1001)
#define KRNX_E_NOTSUPPORTED     (-0x1002)
#define KRNX_E_TIMEOUT          (-0x1003)
#define	KRNX_E_AUXNOTREADY      (-0x1004)
#define	KRNX_E_FOPENFAIL        (-0x1005)
#define	KRNX_E_FILENOTREADY     (-0x1006)
#define KRNX_E_MATRIX           (-0x1007)
#define	KRNX_E_OUTOFRANGE       (-0x1008)
#define	KRNX_E_CANNOTCAL        (-0x1009)
#define KRNX_E_COMPDATA         (-0x100a)
#define KRNX_E_BADUSRID         (-0x100c)
#define KRNX_E_NULLRESP         (-0x100d)
#define KRNX_E_LOSTPROMPT       (-0x100e)
#define KRNX_E_BUFSND           (-0x1010)
#define KRNX_E_BUFRCV           (-0x1011)
#define KRNX_E_BUFTMO           (-0x1012)
#define KRNX_E_AVOID_SING       (-0x1013)	/* EX3578 a++ */
#define KRNX_E_NOAVOID_SING     (-0x1014)	/* EX3578 a-- */
#define KRNX_WARN_SING          (-0x1015)       /* EX3578-6 a */

#define KRNX_E_ASERROR          (-0x1020)
#define KRNX_E_NOROBOT          (-0x1021)
#define KRNX_E_DISABLED         (-0x1022)	/* FX00436 a */

#define KRNX_E_CANTMOVECONFIG	(-0x1030)
#define KRNX_E_JT5NOT0DEG	(-0x1031)
#define KRNX_E_ILLCONFIG	(-0x1032)

#define KRNX_E_SOCK             (-0x2000)
#define KRNX_E_NOHOST           (-0x2001)
#define KRNX_E_IOCTLSOCK        (-0x2002)
#define KRNX_E_SOCKWRITE        (-0x2003)
#define KRNX_E_SOCKREAD         (-0x2004)
#define KRNX_E_NODATA           (-0x2005)
#define KRNX_E_INVALIDPORT      (-0x2006)
#define KRNX_E_CONNECT          (-0x2007)
#define KRNX_E_CANTLOGIN        (-0x2008)
#define KRNX_E_ALREADYOPENED    (-0x2009)
#define KRNX_E_UNEXPECTED       (-0x2010)
#define KRNX_E_KINENOTREADY     (-0x2011)
#define KRNX_E_ASDELAYED        (-0x2012)
#define KRNX_E_BUFEMPTY         (-0x2013)
#define KRNX_E_BUFNO            (-0x2014)
#define KRNX_E_BUFDATANUM       (-0x2015)
#define KRNX_E_SHMEM_OPEN       (-0x2016)

#define KRNX_E_RT_INTERNAL      (-0x2100)
#define KRNX_E_RT_CONNECT       (-0x2101)
#define KRNX_E_RT_TIMEOUT       (-0x2102)
#define KRNX_E_RT_NOTCONNECT    (-0x2103)
#define KRNX_E_RT_SEND          (-0x2104)
#define KRNX_E_RT_CYCLIC        (-0x2105)
#define KRNX_E_RT_SW_ON         (-0x2106)       /* FX03195 a */
#define KRNX_E_RT_NOTSUPPORTED  (-0x2107)
#define KRNX_E_RT_COMP_NOT_CLEARED (-0x2108)
#define KRNX_E_RT_SYNCIO_NEED_POSDATA (-0x2109)

#define KRNX_E_PCASALREADYRUNNING (-0x2200)     /* Dcon-plus6 */
#define KRNX_E_TOOMANYPROC      (-0x2201)       /* Dcon-plus6 */
#define KRNX_E_INVALIDFILENAME  (-0x2202)       /* Dcon-plus6 */
#define KRNX_E_ILLCONTNO        (-0x2203)       /* Dcon-plus16 */

#define KRNX_E_MAXNICNO         (-0x2300)

#define KRNX_E_UNDEF            (-0xFFFF)

/* RT�T�C�N���b�N�ʐM�f�[�^��� */
#define KRNX_CYC_KIND_ANGLE              (0x0001)   /* �e���ʒu(���ݒl) [rad or mm] */
#define KRNX_CYC_KIND_ANGLE_REF          (0x0002)   /* �e���ʒu(�w�ߒl) [rad or mm] */
#define KRNX_CYC_KIND_CURRENT            (0x0004)   /* �e���d���l(���ݒl) [A] */
#define KRNX_CYC_KIND_ENCORDER           (0x0008)   /* �e���G���R�[�_�[�l(���ݒl) [bit] */
#define KRNX_CYC_KIND_ERROR              (0x0010)   /* �G���[�����v/�R�[�h */
#define KRNX_CYC_KIND_CURRENT_REF        (0x0020)   /* �e���d���l(�w�ߒl) [A] */
#define KRNX_CYC_KIND_CURRENT_SAT        (0x0040)   /* �e���d���O�a��(���ݒl/�w�ߒl) */
#define KRNX_CYC_KIND_ENCORDER_REF       (0x0080)   /* �e���G���R�[�_�[�l(�w�ߒl) [bit] */
#define KRNX_CYC_KIND_ANGLE_VEL          (0x0100)   /* �e�����x�l(���ݒl/�w�ߒl) [rad/s or mm/s] */
#define KRNX_CYC_KIND_XYZOAT             (0x0200)   /* �ϊ��ʒu/���x(���ݒl/�w�ߒl) [rad, mm, rad/s or mm/s][rad/s or mm/s] */
#define KRNX_CYC_KIND_SIG_EXTERNAL       (0x0400)   /* �O���o��/���͐M�� */
#define KRNX_CYC_KIND_SIG_INTERNAL       (0x0800)   /* �����M�� */
#define KRNX_CYC_KIND_ROBOT_STATUS       (0x1000)   /* ���{�b�g��� */
#define KRNX_CYC_KIND_SIZE               (13)       /* ��ސ� */
#define KRNX_CYC_KIND_LEGACY             (KRNX_CYC_KIND_ANGLE|KRNX_CYC_KIND_ANGLE_REF|KRNX_CYC_KIND_CURRENT|KRNX_CYC_KIND_ERROR)
#define KRNX_CYC_KIND_SUPPORTED          ((1<<KRNX_CYC_KIND_SIZE)-1)

/* RT�T�C�N���b�N����f�[�^��� */
#define KRNX_CYC_KRNX2AS_KIND_ANGLE_RELATIVE      (0x0001)   /* �e���ʒu(���Βl) [rad or mm] */
#define KRNX_CYC_KRNX2AS_KIND_SIG_EXTERNAL_OUTPUT (0x0002)   /* �O���o�͐M�� */
#define KRNX_CYC_KRNX2AS_KIND_SIG_EXTERNAL_INPUT  (0x0004)   /* �O�����͐M�� */
#define KRNX_CYC_KRNX2AS_KIND_SIG_INTERNAL        (0x0008)   /* �����M�� */
#define KRNX_CYC_KRNX2AS_KIND_SIZE                (4)        /* ��ސ�*/
#define KRNX_CYC_KRNX2AS_KIND_LEGACY              (KRNX_CYC_KRNX2AS_KIND_ANGLE_RELATIVE)
#define KRNX_CYC_KRNX2AS_KIND_SUPPORTED           ((1<<KRNX_CYC_KRNX2AS_KIND_SIZE)-1 )

extern short KRNX_SYSLOG_FLAG; /* 1: output debug message to syslog */

typedef struct
{
    short error_lamp;
    short motor_lamp;
    short cycle_lamp;
    short repeat_lamp;
    short run_lamp;
    short trigger_lamp;
    short teach_lock_lamp;
    short emergency;
} TKrnxPanelInfo;

typedef struct
{
#ifdef NON_COMPATIBLE
    char    io_do[KRNX_MAXSIGNAL/8];
    char    io_di[KRNX_MAXSIGNAL/8];
    char    internal[KRNX_MAXSIGNAL/8];
#else
    char    io_do[DO_MAX_SIGNAL/8];
    char    io_di[DI_MAX_SIGNAL/8];
    char    internal[INTERNAL_MAX_SIGNAL/8];
#endif /* NON_COMPATIBLE */
} TKrnxIoInfo;

typedef struct
{
    char    io_do[DO_MAX_SIGNAL/8];
    char    io_di[DI_MAX_SIGNAL/8];
    char    internal[INTERNAL_MAX_SIGNAL/8];
} TKrnxIoInfoEx;

typedef struct
{
    short    robot_status;
    float    monitor_speed;
    float    always_speed;
    float    accuracy;
} TKrnxMonInfo;

typedef struct
{
    short    status;
    long     exec_count, remain_count;
    char     program_name[20];
    short    priority;
    short    step_number;
#ifdef NON_COMPATIBLE
    char     step_name[80];
#else
    char     step_name[INT_CBUF_SIZ];
#endif /* NON_COMPATIBLE */
} TKrnxStepperInfo;

typedef struct
{
#ifdef NON_COMPATIBLE
    /* �{���z��T�C�Y�́AKRNX_MAX_ROBOT�ł͂Ȃ��A   */
    /* 2(AS���̍ő�A�[�����^�R���g���[��)��        */
    /* �\���ł��邪�A�݊����m�ۂ̂��߂��̂܂܂Ƃ��� */
    TKrnxMonInfo      mon[KRNX_MAX_ROBOT];
    TKrnxStepperInfo  robot[KRNX_MAX_ROBOT];
#else
    TKrnxMonInfo      mon[2/*KRNX_MAX_ROBOT*/];
    TKrnxStepperInfo  robot[2/*KRNX_MAX_ROBOT*/];
#endif /* NON_COMPATIBLE */
    TKrnxStepperInfo  pc[KRNX_MAX_PCPROG];
} TKrnxProgramInfo;

typedef struct
{
    float    ang[KRNX_MAXAXES];
    long     enc[KRNX_MAXAXES];
    float    vel[KRNX_MAXAXES];
    float    ang_ref[KRNX_MAXAXES];
    long     vel_ref[KRNX_MAXAXES];
    float    cur_ref[KRNX_MAXAXES];
} TKrnxMotionInfo;

/* ZZ ++ */
typedef struct TSignalEx
{
    unsigned long usr_di[DI_MAX_SIGNAL/32];
    unsigned long usr_do[DO_MAX_SIGNAL/32];
    unsigned long usr_internal[INTERNAL_MAX_SIGNAL/32];
} TSignalEx;

/*** RTC��� ***/
typedef struct TKrnxRtcInfo
{
    short cyc;
    short buf;
    short interpolation;
} TKrnxRtcInfo;

/* �T�[�{�p�ϐ� */
typedef struct TDebugVariableInfo
{
    float val[KRNX_MAXAXES][DBG_FVAL_NUM];
}TDebugVariableInfo;

/* �M����� */
typedef struct TDebugSignalInfo
{
    TSignalEx 	sig;
}TDebugSignalInfo;

/* �V�X�e����� */
typedef struct TDebugSystemInfo
{
    int a;
    int b;
    int c;
}TDebugSystemInfo;

/* ���{�b�g������ */
typedef struct TDebugTrajInfo
{
    char	step_info[INT_CBUF_SIZ];		/* �X�e�b�v��� simu.h */
    float	sp;			/* ���x   */
    float 	accu;			/* �����x */
    int		ctl_axis;		/* ���� */
}TDebugTrajInfo;

typedef struct TDebugMotionInfo
{
    float    ang[KRNX_MAXAXES];
    float    xyzoat[KRNX_MAXAXES];
    long     enc[KRNX_MAXAXES];
    float    vel[KRNX_MAXAXES];
    float    ang_ref[KRNX_MAXAXES];
    long     vel_ref[KRNX_MAXAXES];
    float    cur_ref[KRNX_MAXAXES];
    float    tool[KRNX_MAXAXES];
    int         mode;			/* ���[�h  0:RPLAN, 1:RMOVE, 2:RWAIT, 3:RHOLD, 4:REND */
    int	    flg_stepup;
    float   end_pos_jtang[KRNX_MAXAXES];
    float   end_pos_trans[KRNX_MAXAXES];    
    unsigned short clamp_spot_flg;
}TDebugMotionInfo;

typedef struct TKrnxDebugInfoEnt
{
    TDebugSystemInfo	sys;		/* */
    TDebugMotionInfo	mtn;		/* */
    TDebugTrajInfo	trj;		/* */   
    TDebugVariableInfo	val;		/* */
    TDebugSignalInfo	io;		/* */
}TKrnxDebugInfoEnt;
/* ZZ -- */

/* FX03496 a++ */
typedef struct TKrnxErrorList
{
    int  error_num;
    long error_code[KRNX_MAX_ERROR_LIST_SIZ];
    char error_msg[KRNX_MAX_ERROR_LIST_SIZ][KRNX_MAX_ERROR_MSG_SIZ];
} TKrnxErrorList;
/* FX03496 a-- */

#define	RT_IOCTL_CMD_NONE      0
#define	RT_IOCTL_CMD_INS       1
#define	RT_IOCTL_CMD_SYNC      2
typedef struct TKrnxRtIoCtlDO
{
    unsigned short sync;
    unsigned short command;
    char    enable[DO_MAX_SIGNAL/8];
    char    data[DO_MAX_SIGNAL/8];
}TKrnxRtIoCtlDO;

typedef struct TKrnxRtIoCtlDI
{
    unsigned short sync;
    unsigned short command;
    char    enable[DI_MAX_SIGNAL/8];
    char    data[DI_MAX_SIGNAL/8];
}TKrnxRtIoCtlDI;

typedef struct TKrnxRtIoCtlInternal
{
    unsigned short sync;
    unsigned short command;
    char    enable[INTERNAL_MAX_SIGNAL/8];
    char    data[INTERNAL_MAX_SIGNAL/8];
}TKrnxRtIoCtlInternal;

// C#(PC-AS)�����΍�
// C#�ł̓f�t�H���g�����͎g�p�ł��Ȃ�
// �K�v����Εʂ�API��p�ӂ���
//#ifdef __cplusplus
//DECLSPEC_IMPORT int WINAPI krnx_Open( int cont_no, char *hostname = NULL, char *port_path = NULL );
//#else
DECLSPEC_IMPORT int WINAPI krnx_Open( int cont_no, char *hostname );
//#endif
DECLSPEC_IMPORT int WINAPI krnx_Close( int sd );

DECLSPEC_IMPORT int WINAPI krnx_SetAppParam( int type, char *param );	/* FX00436 a */

/**************************
 * AUXAPI(���j�^�R�}���h) *
 **************************
 */

/* element type for DELETE,LIST,SAVE */
#define	QUAL_PRG	0x0001	/* �v���O�����A�ϐ� */
#define	QUAL_LOC	0x0002
#define	QUAL_REAL	0x0004
#define	QUAL_STR	0x0008
#define	QUAL_INT	0x0010
#define	QUAL_SYS	0x0020	/* �f�[�^ */
#define	QUAL_ROB	0x0040
#define	QUAL_AUX	0x0080
#define	QUAL_ARC	0x0100
#define	QUAL_IFP	0x0200
#define	QUAL_ELOG	0x0400	/* ����f�[�^ */
#define	QUAL_FLT	0x0800

#ifdef __cplusplus
DECLSPEC_IMPORT int WINAPI krnx_Abort( int cont_no, int robot_no, int *as_err_code=NULL );
DECLSPEC_IMPORT int WINAPI krnx_Hold( int cont_no, int robot_no, int *as_err_code=NULL );
DECLSPEC_IMPORT int WINAPI krnx_Continue( int cont_no, int robot_no, int next, int *as_err_code=NULL );
DECLSPEC_IMPORT int WINAPI krnx_Execute( int cont_no, int robot_no, const char *program, int exec_num, int step_num, int *as_err_code=NULL );
DECLSPEC_IMPORT int WINAPI krnx_Kill( int cont_no, int robot_no, int *as_err_code=NULL );
DECLSPEC_IMPORT int WINAPI krnx_SetMonSpeed( int cont_no, int robot_no, float speed, int *as_err_code=NULL );
DECLSPEC_IMPORT int WINAPI krnx_PcAbort( int cont_no, int pcprogram_no, int *as_err_code=NULL );
DECLSPEC_IMPORT int WINAPI krnx_PcEnd( int cont_no, int pcprogram_no, int *as_err_code=NULL );
DECLSPEC_IMPORT int WINAPI krnx_PcContinue( int cont_no, int pcprogram_no, int next, int *as_err_code=NULL );
DECLSPEC_IMPORT int WINAPI krnx_PcExecute( int cont_no, int pcprogram_no, const char *program, int exec_num, int step_num, int *as_err_code=NULL );
DECLSPEC_IMPORT int WINAPI krnx_PcKill( int cont_no, int pcprogram_no, int *as_err_code=NULL );
DECLSPEC_IMPORT int WINAPI krnx_GetBaseMatrix( int cont_no, int robot_no, float *xyzoat, int *as_err_code=NULL );
DECLSPEC_IMPORT int WINAPI krnx_SetBaseMatrix( int cont_no, int robot_no, const float *xyzoat, int *as_err_code=NULL );
DECLSPEC_IMPORT int WINAPI krnx_GetToolMatrix( int cont_no, int robot_no, float *xyzoat, int *as_err_code=NULL );
DECLSPEC_IMPORT int WINAPI krnx_SetToolMatrix( int cont_no, int robot_no, const float *xyzoat, int *as_err_code=NULL );
DECLSPEC_IMPORT int WINAPI krnx_GetSignal( int cont_no, int signal_no, int *status, int *as_err_code=NULL );
DECLSPEC_IMPORT int WINAPI krnx_SetSignal( int cont_no, int signal_no, int status, int *as_err_code=NULL );
DECLSPEC_IMPORT int WINAPI krnx_Reset( int cont_no, int *as_err_code=NULL );
DECLSPEC_IMPORT int WINAPI krnx_Delete( int cont_no, const char *element_name, int element_type, int *as_err_code=NULL );
DECLSPEC_IMPORT int WINAPI krnx_List( int cont_no, const char *element_name, int element_type, char *buffer, int buffer_sz, int *as_err_code=NULL );
DECLSPEC_IMPORT int WINAPI krnx_Ereset( int cont_no, int robot_no, int *as_err_code=NULL );
DECLSPEC_IMPORT int WINAPI krnx_Do( int cont_no, int robot_no, const char *cmd, int *as_err_code=NULL );
DECLSPEC_IMPORT int WINAPI krnx_Prime( int cont_no, int robot_no, const char *program, int exec_num, int step_num, int create, int *as_err_code=NULL );
DECLSPEC_IMPORT int WINAPI krnx_RecOneStep( int cont_no, const char *program, int step_num, const char *step_data, int insert, int *as_err_code=NULL );
DECLSPEC_IMPORT int WINAPI krnx_ExecMon( int cont_no, const char *cmd, char *buffer, int buffer_sz, int *as_err_code=NULL );
#else
DECLSPEC_IMPORT int WINAPI krnx_Abort( int cont_no, int robot_no, int *as_err_code );
DECLSPEC_IMPORT int WINAPI krnx_Hold( int cont_no, int robot_no, int *as_err_code );
DECLSPEC_IMPORT int WINAPI krnx_Continue( int cont_no, int robot_no, int next, int *as_err_code );
DECLSPEC_IMPORT int WINAPI krnx_Execute( int cont_no, int robot_no, const char *program, int exec_num, int step_num, int *as_err_code );
DECLSPEC_IMPORT int WINAPI krnx_Kill( int cont_no, int robot_no, int *as_err_code );
DECLSPEC_IMPORT int WINAPI krnx_SetMonSpeed( int cont_no, int robot_no, float speed, int *as_err_code );
DECLSPEC_IMPORT int WINAPI krnx_PcAbort( int cont_no, int pcprogram_no, int *as_err_code );
DECLSPEC_IMPORT int WINAPI krnx_PcEnd( int cont_no, int pcprogram_no, int *as_err_code );
DECLSPEC_IMPORT int WINAPI krnx_PcContinue( int cont_no, int pcprogram_no, int next, int *as_err_code );
DECLSPEC_IMPORT int WINAPI krnx_PcExecute( int cont_no, int pcprogram_no, const char *program, int exec_num, int step_num, int *as_err_code );
DECLSPEC_IMPORT int WINAPI krnx_PcKill( int cont_no, int pcprogram_no, int *as_err_code );
DECLSPEC_IMPORT int WINAPI krnx_GetBaseMatrix( int cont_no, int robot_no, float *xyzoat, int *as_err_code );
DECLSPEC_IMPORT int WINAPI krnx_SetBaseMatrix( int cont_no, int robot_no, const float *xyzoat, int *as_err_code );
DECLSPEC_IMPORT int WINAPI krnx_GetToolMatrix( int cont_no, int robot_no, float *xyzoat, int *as_err_code );
DECLSPEC_IMPORT int WINAPI krnx_SetToolMatrix( int cont_no, int robot_no, const float *xyzoat, int *as_err_code );
DECLSPEC_IMPORT int WINAPI krnx_GetSignal( int cont_no, int signal_no, int *status, int *as_err_code );
DECLSPEC_IMPORT int WINAPI krnx_SetSignal( int cont_no, int signal_no, int status, int *as_err_code );
DECLSPEC_IMPORT int WINAPI krnx_Reset( int cont_no, int *as_err_code );
DECLSPEC_IMPORT int WINAPI krnx_Delete( int cont_no, const char *element_name, int element_type, int *as_err_code );
DECLSPEC_IMPORT int WINAPI krnx_List( int cont_no, const char *element_name, int element_type, char *buffer, int buffer_sz, int *as_err_code );
DECLSPEC_IMPORT int WINAPI krnx_Ereset( int cont_no, int robot_no, int *as_err_code );
DECLSPEC_IMPORT int WINAPI krnx_Do( int cont_no, int robot_no, const char *cmd, int *as_err_code );
DECLSPEC_IMPORT int WINAPI krnx_Prime( int cont_no, int robot_no, const char *program, int exec_num, int step_num, int create, int *as_err_code );
DECLSPEC_IMPORT int WINAPI krnx_RecOneStep( int cont_no, const char *program, int step_num, const char *step_data, int insert, int *as_err_code );
DECLSPEC_IMPORT int WINAPI krnx_ExecMon( int cont_no, const char *cmd, char *buffer, int buffer_sz, int *as_err_code );
#endif

DECLSPEC_IMPORT int WINAPI krnx_Save( int cont_no, const char *filename, const char *program_name, int option );
DECLSPEC_IMPORT int WINAPI krnx_Load( int cont_no, const char *filename );

typedef BOOL (*FLoadCallBack)( void *param, long byte, const char *msg );

#ifdef __cplusplus
DECLSPEC_IMPORT int WINAPI krnx_SaveEx( int cont_no, const char *filename, const char *program_name, int option, FLoadCallBack cbfp=NULL, void *cb_param=NULL );
DECLSPEC_IMPORT int WINAPI krnx_LoadEx( int cont_no, const char *filename, FLoadCallBack cbfp=NULL, void *cb_param=NULL );
#else
DECLSPEC_IMPORT int WINAPI krnx_SaveEx( int cont_no, const char *filename, const char *program_name, int option, FLoadCallBack cbfp, void *cb_param );
DECLSPEC_IMPORT int WINAPI krnx_LoadEx( int cont_no, const char *filename, FLoadCallBack cbfp, void *cb_param );
#endif

DECLSPEC_IMPORT int WINAPI krnx_ConvertErrorCode( int *error_code, char *error_level );
DECLSPEC_IMPORT int WINAPI krnx_GetRtcInfo( int cont_no, TKrnxRtcInfo *rtc_info );
DECLSPEC_IMPORT int WINAPI krnx_SetRtcInfo( int cont_no, TKrnxRtcInfo *rtc_info );

DECLSPEC_IMPORT int WINAPI krnx_SetAuxApiTimeoutPeriod( int cont_no, int period ); /* FX02360 a */

/**************************
 *        AS-API          *
 **************************
 */
DECLSPEC_IMPORT int WINAPI krnx_GetRobotName( int cont_no, int robot_no, char *robot_name );
DECLSPEC_IMPORT int WINAPI krnx_GetPanelInfo( int cont_no, int robot_no, TKrnxPanelInfo *panelinfo );
DECLSPEC_IMPORT int WINAPI krnx_SetPanelInfo( int cont_no, int robot_no, TKrnxPanelInfo *panelinfo );
DECLSPEC_IMPORT int WINAPI krnx_GetMotionInfo( int cont_no, int robot_no, TKrnxMotionInfo *mtninfo );
DECLSPEC_IMPORT int WINAPI krnx_GetCurMotionInfo( int cont_no, int robot_no, TKrnxMotionInfo *mtninfo );
DECLSPEC_IMPORT int WINAPI krnx_GetMotionInfoEx( int cont_no, int robot_no, TKrnxMotionInfo *mtninfo, int data_type );
DECLSPEC_IMPORT int WINAPI krnx_GetIoInfo( int cont_no, TKrnxIoInfo *ioinfo );
DECLSPEC_IMPORT int WINAPI krnx_GetCurIoInfo( int cont_no, TKrnxIoInfo *ioinfo );
DECLSPEC_IMPORT int WINAPI krnx_GetProgramInfo( int cont_no, int robot_no, TKrnxProgramInfo *proginfo );
DECLSPEC_IMPORT int WINAPI krnx_GetProgramInfo2( int cont_no, int robot_no, TKrnxProgramInfo *proginfo );
DECLSPEC_IMPORT int WINAPI krnx_GetCurErrorList( int cont_no, TKrnxErrorList *errorlist ); /* FX03496 a */

DECLSPEC_IMPORT int WINAPI krnx_BufferBusy( int cont_no, int buf_no );
DECLSPEC_IMPORT int WINAPI krnx_BufferEmpty( int cont_no, int buf_no );
DECLSPEC_IMPORT int WINAPI krnx_BufferSendF( int cont_no, int buf_no, short req_code, const float *p, int num,  int timeout );
DECLSPEC_IMPORT int WINAPI krnx_BufferSendW( int cont_no, int buf_no, short req_code, const short *p, int num,  int timeout );
DECLSPEC_IMPORT int WINAPI krnx_BufferSendB( int cont_no, int buf_no, short req_code, const char *p, int num,  int timeout );
DECLSPEC_IMPORT int WINAPI krnx_BufferRecvF( int cont_no, int buf_no, short *req_code, float *p, int *num,  int timeout );
DECLSPEC_IMPORT int WINAPI krnx_BufferPeekF( int cont_no, int buf_no, short *req_code, float *p, int *num,  int timeout );
DECLSPEC_IMPORT int WINAPI krnx_BufferRecvW( int cont_no, int buf_no, short *req_code, short *p, int *num,  int timeout );
DECLSPEC_IMPORT int WINAPI krnx_BufferPeekW( int cont_no, int buf_no, short *req_code, short *p, int *num,  int timeout );
DECLSPEC_IMPORT int WINAPI krnx_BufferRecvB( int cont_no, int buf_no, short *req_code, char *p, int *num,  int timeout );
DECLSPEC_IMPORT int WINAPI krnx_BufferPeekB( int cont_no, int buf_no, short *req_code, char *p, int *num,  int timeout );

DECLSPEC_IMPORT int WINAPI krnx_GetLimitM( int cont_no, int robot_no, float *ll );
DECLSPEC_IMPORT int WINAPI krnx_GetLimitP( int cont_no, int robot_no, float *ul );

DECLSPEC_IMPORT int WINAPI krnx_GetErrorInfo( int cont_no, int robot_no, int *error_code );
DECLSPEC_IMPORT int WINAPI krnx_IoSetDI( int cont_no, const char *in, const char *mask, size_t size );
DECLSPEC_IMPORT int WINAPI krnx_IoSetDO( int cont_no, const char *out, const char *mask, size_t size );

DECLSPEC_IMPORT int WINAPI krnx_SetAsApiTimeoutPeriod( int cont_no, int period ); /* FX02360 a */

/**************************
 *      ���t�ϊ�API       *
 **************************
 */

DECLSPEC_IMPORT int WINAPI krnx_JointToXyzoat( int cont_no, int robot_no, const float *joint,  float *xyzoat );
DECLSPEC_IMPORT int WINAPI krnx_JointToMatrix( int cont_no, int robot_no, const float *joint,  float *matrix );
DECLSPEC_IMPORT int WINAPI krnx_XyzoatToJoint( int cont_no, int robot_no, const float *xyzoat, float *joint, const float *old_joint );
DECLSPEC_IMPORT int WINAPI krnx_MatrixToJoint( int cont_no, int robot_no, const float *matrix, float *joint, const float *old_joint );
DECLSPEC_IMPORT int WINAPI krnx_MultiplyXyzoat( const float *xyzoat_a, const float *xyzoat_b, float *xyzoat_c );
DECLSPEC_IMPORT int WINAPI krnx_MultiplyMatrix( const float *matrix_a, const float *matrix_b, float *matrix_c );
DECLSPEC_IMPORT int WINAPI krnx_InverseXyzoat(  const float *xyzoat_a, float *xyzoat_b );
DECLSPEC_IMPORT int WINAPI krnx_InverseMatrix(  const float *matrix_a, float *matrix_b );
DECLSPEC_IMPORT int WINAPI krnx_MatrixToXyzoat( const float *matrix  , float *xyzoat  );
DECLSPEC_IMPORT int WINAPI krnx_XyzoatToMatrix( const float *xyzoat  , float *matrix  );
DECLSPEC_IMPORT int WINAPI krnx_FrameMatrix( const float *mat_a, const float *mat_b, const float *mat_c, const float *mat_d, float *mat_p );
DECLSPEC_IMPORT int WINAPI krnx_FrameXyzoat( const float *xyz_a, const float *xyz_b, const float *xyz_c, const float *xyz_d, float *xyz_p );
DECLSPEC_IMPORT int WINAPI krnx_JacobiMatrix( int cont_no, int robot_no, const float *joint, const float *tool_matrix , float *jacobi66 , float *matrix );
DECLSPEC_IMPORT int WINAPI krnx_JacobiXyzoat( int cont_no, int robot_no, const float *joint, const float *tool_xyzoat , float *jacobi66 , float *xyzoat );
DECLSPEC_IMPORT int WINAPI krnx_JaInvMatrix( int cont_no, int robot_no, const float *joint, const float *tool_matrix , float *ja_inv66 , float *matrix );
DECLSPEC_IMPORT int WINAPI krnx_JaInvXyzoat( int cont_no, int robot_no, const float *joint, const float *tool_xyzoat , float *ja_inv66 , float *xyzoat );

DECLSPEC_IMPORT int WINAPI krnx_XyzoatToJoint2( int cont_no, int robot_no, const float *xyzoat, float *joint, const float *old_joint, int conf );
DECLSPEC_IMPORT int WINAPI krnx_GetConfig( int cont_no, int robot_no, float *joint, int *conf );

DECLSPEC_IMPORT int WINAPI krnx_GetASCycle( int robot_no, int *cycle_time );

DECLSPEC_IMPORT int WINAPI krnx_SetJT1Mode( int mode );

  DECLSPEC_IMPORT int WINAPI krnx_Xyz456ToJoint( int cont_no, int robot_no, const float *xyzoat, float *joint, const float *old_joint, const float *tool );	/* EX3578-3 a */

/**************************
 *        RT-API          *
 **************************
 */
typedef struct
{
    float    ang[KRNX_MAXAXES];             /* �e���ʒu(���ݒl) [rad or mm] */
    float    ang_ref[KRNX_MAXAXES];         /* �e���ʒu(�w�ߒl) [rad or mm] */
    float    cur[KRNX_MAXAXES];             /* �e���d���l(���ݒl) [A] */
    int      enc[KRNX_MAXAXES];             /* �e���G���R�[�_�[�l(���ݒl) [bit] */
} TKrnxCurMotionData;

typedef struct
{
    float    ang[KRNX_MAXAXES];             /* �e���ʒu(���ݒl) [rad or mm] */
    float    ang_ref[KRNX_MAXAXES];         /* �e���ʒu(�w�ߒl) [rad or mm] */
    float    cur[KRNX_MAXAXES];             /* �e���d���l(���ݒl) [A] */
    int      enc[KRNX_MAXAXES];             /* �e���G���R�[�_�[�l(���ݒl) [bit] */
    float    cur_ref[KRNX_MAXAXES];         /* �e���d���l(�w�ߒl) [A] */
    float    cur_sat[KRNX_MAXAXES];         /* �e���d���O�a��(���ݒl�j */
    float    cur_sat_ref[KRNX_MAXAXES];     /* �e���d���O�a��(�w�ߒl) */
    int      enc_ref[KRNX_MAXAXES];         /* �e���G���R�[�_�[�l(�w�ߒl) [bit] */
    float    ang_vel[KRNX_MAXAXES];         /* �e�����x�l(���ݒl) [rad/s or mm/s] */
    float    ang_vel_ref[KRNX_MAXAXES];     /* �e�����x�l(�w�ߒl) [rad/s or mm/s] */
    float    xyzoat[6];                     /* �ϊ��ʒu(���ݒl) [rad or mm] */
    float    xyzoat_ref[6];                 /* �ϊ��ʒu(�w�ߒl) [rad or mm] */
    float    xyzoat_vel;                    /* �ϊ����x�l(���ݒl) [mm/s] */
    float    xyzoat_vel_ref;                /* �ϊ����x�l(�w�ߒl) [mm/s] */
} TKrnxCurMotionDataEx;

typedef struct
{
    short motor_lamp;                       /* ���[�^�����v */
    short cycle_lamp;                       /* �T�C�N�������v */
    short repeat_lamp;                      /* ���s�[�g�����v */
    short run_lamp;                         /* ���s�[�g�����v */
    short trigger_lamp;                     /* �g���K�[�����v */
    short teach_lock_lamp;                  /* �e�B�[�`���b�N�����v */
    short emergency;                        /* ����~�����v */
    short rtc_active;                       /* RTC�A�N�e�B�u��� */
    short rb_program_run;                   /* ���{�b�g�v���O�������s��ԃ`�F�b�N */
    short monitor_speed;                    /* ���j�^���x[%] */
    short check_speed;                      /* �`�F�b�N���x[mm/sec] */
    short enverr_warm;                      /* �΍��ُ���[���r�b�g] */
    char reserved[8];
} TKrnxCurRobotStatus;

DECLSPEC_IMPORT int WINAPI krnx_GetCurMotionData( int cont_no, int robot_no, TKrnxCurMotionData *md );
DECLSPEC_IMPORT int WINAPI krnx_GetCurMotionDataEx( int cont_no, int robot_no, TKrnxCurMotionDataEx *md );
DECLSPEC_IMPORT int WINAPI krnx_GetCurErrorLamp( int cont_no, int robot_no, int *error_lamp ); /* EX3390 a */
DECLSPEC_IMPORT int WINAPI krnx_GetCurErrorInfo( int cont_no, int robot_no, int *error_code ); /* EX3390 a */
DECLSPEC_IMPORT int WINAPI krnx_GetCurIoInfoEx( int cont_no, TKrnxIoInfoEx *ioinfo );
DECLSPEC_IMPORT int WINAPI krnx_GetCurRobotStatus( int cont_no, int robot_no, TKrnxCurRobotStatus *status );
DECLSPEC_IMPORT int WINAPI krnx_SetRtCyclicDataKind( int cont_no, unsigned short kind );
DECLSPEC_IMPORT int WINAPI krnx_GetRtCyclicDataKind( int cont_no, unsigned short *krnx_kind, unsigned short *as_kind );
DECLSPEC_IMPORT int WINAPI krnx_GetRtCyclicCtlKind( int cont_no, unsigned short *krnx_kind, unsigned short *as_kind );

/* RTC */
DECLSPEC_IMPORT int WINAPI krnx_SetRtcCompData( int cont_no, int robot_no, const float *comp, int *status, unsigned short seq_no );
DECLSPEC_IMPORT int WINAPI krnx_PrimeRtcCompData( int cont_no, int robot_no, const float *comp, int *status ); /* EX3391 a */
DECLSPEC_IMPORT int WINAPI krnx_PrimeRtIoData( int cont_no, TKrnxRtIoCtlDO *io_do, TKrnxRtIoCtlDI *io_di, TKrnxRtIoCtlInternal *internal ); /* EX3691 a */
DECLSPEC_IMPORT int WINAPI krnx_SendRtcCompData( int cont_no, unsigned short seq_no ); /* EX3391 a */
DECLSPEC_IMPORT int WINAPI krnx_SendRtCtlData( int cont_no, unsigned short seq_no, unsigned short kind ); /* EX3691 a */
DECLSPEC_IMPORT int WINAPI krnx_SetRtcCompDataEx( int cont_no, int robot_no, const float *comp, int *status, unsigned long *count_in, unsigned long *count_out, unsigned short seq_no );
DECLSPEC_IMPORT int WINAPI krnx_GetRtcCompData( int cont_no, int robot_no, float *comp );
DECLSPEC_IMPORT int WINAPI krnx_GetRtcCompLimit( int cont_no, int robot_no, float *comp_limit );
DECLSPEC_IMPORT int WINAPI krnx_SetRtcErrorFlag( int cont_no, int robot_no, int error_flag, unsigned short seq_no );
DECLSPEC_IMPORT int WINAPI krnx_GetRtcErrorFlag( int cont_no, int robot_no, int *error_flag );
DECLSPEC_IMPORT int WINAPI krnx_GetRtcSwitch( int cont_no, int robot_no, int *rtc_sw );
DECLSPEC_IMPORT int WINAPI krnx_SetRtcCompMask( int cont_no, int robot_no, int mask );
DECLSPEC_IMPORT int WINAPI krnx_OldCompClear( int cont_no, int robot_no );
DECLSPEC_IMPORT int WINAPI krnx_GetRtcBufferLength( int cont_no, int robot_no ); /* EX3390 a */
DECLSPEC_IMPORT void WINAPI krnx_timer_callback( void );

DECLSPEC_IMPORT int WINAPI krnx_RtcInit( int cont_no );

DECLSPEC_IMPORT int WINAPI krnx_SetConveyorSpeed( int cont_no, int robot_no, float spd, float *prev  );
DECLSPEC_IMPORT int WINAPI krnx_SetConveyorPos( int cont_no, int robot_no, int wk_no, float pos, float *prev  );
DECLSPEC_IMPORT int WINAPI krnx_SetConveyorSpeedEx( int cont_no, int robot_no, int axis_no, float spd, float *prev  );
DECLSPEC_IMPORT int WINAPI krnx_SetConveyorPosEx( int cont_no, int robot_no, int axis_no, int wk_no, float pos, float *prev  );

/*******************************
 * KRNX����API
 *******************************
 */
DECLSPEC_IMPORT int WINAPI krnx_GetKrnxVersion( char *ver_text, int ver_len ); /* FX00919 a */

/**************************
 *        ARC-API          *
 **************************
 */
#ifdef KRNX_ARC
DECLSPEC_IMPORT int  WINAPI krnx_ArcCmdWrite( int cont_no, int robot_no, int cmd, int i_cmd, int v_cmd );
DECLSPEC_IMPORT int  WINAPI krnx_ArcCmdRead( int cont_no, int robot_no, int *i_cmd, int *v_cmd );
DECLSPEC_IMPORT int  WINAPI krnx_ArcStatusRead( int cont_no, int robot_no, int *hw_status, int *rob_status, int *hw_cmd );
DECLSPEC_IMPORT int  WINAPI krnx_ArcModifyWrite( int cont_no, int robot_no, int and_cmd, int or_cmd, int and_status, int or_status );
DECLSPEC_IMPORT int  WINAPI krnx_ArcWeldChangeOk( int cont_no, int robot_no );
DECLSPEC_IMPORT int  WINAPI krnx_ArcWeldChangeMode( int cont_no, int robot_no, int on_off );
DECLSPEC_IMPORT int  WINAPI krnx_ArcWeldChange( int cont_no, int robot_no, float sp, float cur, float vlt, float width, float freq, int pn );
DECLSPEC_IMPORT int  WINAPI krnx_ArcXyzChangeOk( int cont_no, int robot_no );
DECLSPEC_IMPORT int  WINAPI krnx_ArcXyzChangeMode( int cont_no, int robot_no, int on_off );
DECLSPEC_IMPORT int  WINAPI krnx_ArcXyzChange( int cont_no, int robot_no, float x, float y, float z );
DECLSPEC_IMPORT int  WINAPI krnx_ArcXyzRead( int cont_no, int robot_no, float *x, float *y, float *z );
DECLSPEC_IMPORT int  WINAPI krnx_ArcToolXyzChange( int cont_no, int robot_no, float x, float y, float z );
DECLSPEC_IMPORT int  WINAPI krnx_ArcToolXyzRead( int cont_no, int robot_no, float *x, float *y, float *z );
/* SRV_TORCH ++ */
DECLSPEC_IMPORT int  WINAPI krnx_ArcGetSrvtMotorLoad( int cont_no, int robot_no, float *mload );
DECLSPEC_IMPORT int  WINAPI krnx_ArcGetSrvtRotation( int cont_no, int robot_no, float *delta_ang );
DECLSPEC_IMPORT int  WINAPI krnx_ArcGetCurSrvtRotation( int cont_no, int robot_no, float *ang );
/* SRV_TORCH -- */
#endif /* KRNX_ARC */

/* ETHER */
DECLSPEC_IMPORT int WINAPI krnx_GetCycleCount( int cont_no, int robot_no, int counter_no );
DECLSPEC_IMPORT	int WINAPI krnx_eth_init( char *hostname );
DECLSPEC_IMPORT	int WINAPI krnx_eth_open( int cont_no );
DECLSPEC_IMPORT	int WINAPI krnx_eth_close( int cont_no );
DECLSPEC_IMPORT	int WINAPI krnx_SetPriority( DWORD p );

/*******************************
 * ���o�b�R���Ƃ̌݊����̂���
 *******************************
 */
DECLSPEC_IMPORT int WINAPI krnx_NotSupport( void );
DECLSPEC_IMPORT int WINAPI krnx_PanelHw( int, int, char *);
DECLSPEC_IMPORT int WINAPI krnx_PanelToPC( int, int, char *);
DECLSPEC_IMPORT int WINAPI krnx_GetArmMode( int, int );

typedef struct
{
    short	no, num;	// �M���ԍ� 1�`1001�`�A�r�b�g��
    char 	str[80];	//
}TKrnxDDSig;

DECLSPEC_IMPORT	int WINAPI krnx_IoGetDDSig( int cont_no, char *dd_di, char *dd_do );
DECLSPEC_IMPORT	int WINAPI krnx_DDSigInfo( int cont_no, int io_no, const TKrnxDDSig** );
DECLSPEC_IMPORT	int WINAPI krnx_GetMotionInfoSync( int cont_no, int robot_no, TKrnxMotionInfo *md, int data_num );

/*******************************
 * PcAs�N��API
 *******************************
 */

/* ++ Dcon-plus6 ++ */
#ifdef __cplusplus
DECLSPEC_IMPORT	int WINAPI krnx_RunPcAs( char *ini_file_name=NULL );
#else
DECLSPEC_IMPORT	int WINAPI krnx_RunPcAs( char *ini_file_name );
#endif
DECLSPEC_IMPORT	int WINAPI krnx_StopPcAs( void );
/* -- Dcon-plus6 -- */
DECLSPEC_IMPORT int WINAPI krnx_StopPcAsEx( int entry );        /* Dcon-plus16 a */

/* New datasync api 090210 */
DECLSPEC_IMPORT int WINAPI krnx_GetDebugInfoSync( int cont_no, int robot_no, TKrnxDebugInfoEnt *md, int data_num );

DECLSPEC_IMPORT int WINAPI krnx_GetDebugInfo( int cont_no, int robot_no, TKrnxDebugInfoEnt *md, int data_num );

DECLSPEC_IMPORT int WINAPI krnx_SetJoint( int cont_no, int robot_no, float ang[KRNX_MAXAXES] );

#ifdef __cplusplus
}
#endif

#endif /* KRNX_H__ */

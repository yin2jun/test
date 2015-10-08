#ifndef __C103__
#define __C103__

#include "typedef.h"
#include "nucleus.h"
#include "inforec.h"
#include "system.h"
#include "dev_mng.h"
/********************************************************/
/*														*/
/*	�ⲿ�ӿ�����										*/
/*														*/
/********************************************************/
/* 103�豸����ӿ� */
BOOL c103_service(T_IED *p_ied, T_MESSAGE *p_msg);

/********************************************************/
/*														*/
/*	103�豸����ӿڼ��ڲ��ӿ�����						*/
/*														*/
/********************************************************/

typedef struct tagT_C103_CODE_TABLE
{
	T_EVENT_ENTRY  *p_event;	/* �¼���� */
	T_ALARM_ENTRY  *p_alarm;	/* �澯��� */
	T_SET_ENTRY    *p_setting;	/* ��ֵ��� */
	T_SFC_ENTRY	   *p_sfc;		/* ѹ����� */
	T_ANALOG_ENTRY *p_analog;	/* ģ������� */
	WORD			w_even;		/* �¼����� */
	WORD			w_alarm;	/* �澯���� */
	WORD			w_setting;	/* ��ֵ���� */
	WORD			w_sfc;		/* ѹ����� */
	WORD			w_analog;	/* ģ�������� */
}	T_C103_CODE_TABLE;

/* 103��Ϣ��� */
typedef struct tagT_C103_DATA_TYPE
{
	//ͨ���ࣨ�ߣ��ͣ�����ֵ����ѹ�壬��ֵ����ң�⣬�¼����澯��ң�ţ�ң�أ�
	BYTE	bySetGinh;
	BYTE	bySetGinl;
	BYTE	bySfcGinh;
	BYTE	bySfcGinl;
	BYTE	byZoneGinh;
	BYTE	byZoneGInl;
	BYTE	byMsGinh;
	BYTE	byMsGinl;
	BYTE	byEvnGinh;
	BYTE	byEvnGinl;
	BYTE	byAlmGinh;
	BYTE	byAlmGinl;
	BYTE	byDiGinh;
	BYTE	byDiGInl;
	BYTE	byCtrlGinh;
	BYTE	byCtrlGinl;
	
	//ASDU1����β�����¼����澯��ң�� 
	BYTE	byInfEvnStart;
	BYTE	byInfEvnEnd;
	BYTE	byInfAlmStart;
	BYTE	byInfAlmEnd;
	BYTE	byInfDiStart;
	BYTE	byInfDiEnd;
}	T_C103_DATA_TYPE;

/* 103�Ự��ʩ���� */
#define C103_ASDU_TIMEOUT	(2*SYS_SECOND_TICKS)
#define C103_ASDU_SIZE		0x300

/* ����ĸ������� */
#define BP2A	0
#define	BP2B	1

/* 103����֡��ʽ���� */
typedef struct tagT_C103_FRAME
{	
	BYTE	type;			/* ֡���� */
	BYTE	len;			/* ����   */
	BYTE	contral;		/* ������ */
	BYTE	address;		/* ��ַ�� */
	BYTE	crc;			/* ֡У��� */
	BYTE	endbyte;		/* �����ַ� */
}	T_C103_FRAME;

//֡��ʽ����
#define FRAME_T_VOL	0x68	/* �ɱ�֡�� */
#define FRAME_T_CON	0x10	/* �̶�֡�� */

/* 103Ӧ�÷������ݵ�Ԫ(ASDU)��ʽ */
typedef struct tagT_C103_ASDU
{
	BYTE	type;			/* ���ͱ�ʶ */
	BYTE	vsq;			/* �ɱ�ṹ�޶��� */
	BYTE	cot;			/* ����ԭ�� */
	BYTE	addr;			/* ������ַ */
	BYTE	fun;			/* �������� */
	BYTE	inf;			/* ��Ϣ��� */
	BYTE	data[C103_ASDU_SIZE-7];	/* ��Ϣ�屨�� */
}	T_C103_ASDU;

/* 103����(��ϢԪ)��ʽ */
typedef struct tagT_C103_DATA
{
	BYTE	ginh;			/* ͨ�÷����ʶ��Ÿ� */
	BYTE	ginl;			/* ͨ�÷����ʶ��ŵ� */
	BYTE	kod;			/* ��������� */
	BYTE	type;			/* �������� */
	BYTE	size;			/* ���ݿ�� */
	BYTE	num;			/* ��Ŀ */
	BYTE	data[16];		/* ����ֵ */
}	T_C103_DATA;

/* ϵͳ1ms�жϼ����� */
extern volatile DWORD	g_dTCounter;

/* ���ͱ�ʶ */
#define	ASDU1		1
#define ASDU5		5
#define ASDU8		8
#define ASDU10		10

/* �������� */
#define C103_FUN_DISTANCE	128		//���뱣��
#define C103_FUN_OVERCRT	160		//��������
#define C103_FUN_TDEFFER	176		//��ѹ�������

#define C103_FUN_CSL101B	178		//�ķ���·����
#define C103_FUN_CSL164B	179		//�ķ���·����
#define C103_FUN_CSL103C1	180		//�ķ���·������
#define C103_FUN_CSL103C2	181		//�ķ���·������
#define c103_FUN_CSI101C	188		//�ķ���·������
#define c103_FUN_CSI101A	225		//�ķ��ۺ��غ�բ
#define C103_FUN_CSL103B	220		//�ķ���·����
#define C103_FUN_CSC161A	242		//�ķ���·����

#define C103_FUN_LDEFFER	192		//��·�����
#define C103_FUN_BP			210		//��������ĸ�߱���
#define C103_FUN_GEN		254		//ͨ�÷��๦������
#define C103_FUN_GLB		255		//ȫ�ֹ�������

/* ͨ�÷����ʶ��(GINH:���) */
#define C103_GIN_SYS0	0	//ϵͳ��0
#define C103_GIN_SYS1	1	//ϵͳ��1
#define C103_GIN_SET0	2	//��ֵ��0
#define C103_GIN_SET1	3	//��ֵ��1
#define C103_GIN_EVENT	4	//����������
#define C103_GIN_ALARM	5	//�����澯��
#define C103_GIN_CHN	6	//����������
#define C103_GIN_MS		7	//ң����
#define C103_GIN_DI		8	//ң����
#define C103_GIN_PS		10	//ң����
#define C103_GIN_DO		11	//ң����
#define C103_GIN_TP		12	//��ͷ��(tap position)
#define C103_GIN_YT		13	//ң����
#define C103_GIN_SFC	14	//��ѹ����
#define C103_GIN_SOE	24	//ң��SOE

/* kind of description(KOD) */
#define C103_KOD_VAL	1	//value:	ʵ��ֵ
#define C103_KOD_DEF	2	//default:	ȱʡֵ
#define C103_KOD_RAN	3	//range:	���̣���Сֵ�����ֵ��������
#define C103_KOD_PRE	5	//precision:���ȣ�n��m��
#define C103_KOD_FAC	6	//factor:	����
#define C103_KOD_UNI	9	//unit:		��λ(����)
#define C103_KOD_NAM	10	//name:		����(����)

/* type of data(TOD) */
#define C103_TOD_NIL	0	//������
#define C103_TOD_ASC	1	//ASCII�ַ�
#define C103_TOD_UINT	3	//�޷�������
#define C103_TOD_SINT	4	//�з�������
#define C103_TOD_FLOAT	6	//������
#define C103_TOD_R3223	7	//IEEE��׼754��ʵ��
#define C103_TOD_R6453	8	//IEEE��׼754ʵ��
#define C103_TOD_DPI	9	//˫����Ϣ
#define C103_TOD_MSQ	12	//��Ʒ�������Ĳ���ֵ
#define C103_TOD_SOE	18	//��ʱ��ı���

/* ϵͳ��0�µ���Ŀ���� */
#define C103_CUR_ZONE	2	//��ǰ��ֵ��
#define C103_RUN_ZONE	3	//���ж�ֵ��
#define C103_PLS_STS	5	//����״̬(����/�ⶳ)
#define C103_SIG_STS	6	//�ź�״̬(����/δ����)

/* 103Э���ʼ�� */
BOOL c103_initialize();

/* 103�豸��ʼ�� */
BOOL c103_open_ied(T_IED *p_ied);

/* 103�豸�ܲ�ѯ���� */
VOID task_c103(UNSIGNED argc, VOID *argv);

/* ��ȡ�������� */
BOOL c103_get_pulse(T_IED *p_ied, T_PULSE *p_ps);

/* ��������ͨ��ֵ */
BOOL c103_get_channel(T_IED *p_ied, T_CHANNEL *p_chn);

/* ��ֵ����� */
BOOL c103_get_setting(T_IED *p_ied, T_SET *p_set);

BOOL c103_chk_setting(T_IED *p_ied, const T_SET *p_set);

BOOL c103_set_setting(T_IED *p_ied, const T_SET *p_set);

BOOL c103_get_zone(T_IED *p_ied, WORD *p_set_no);

BOOL c103_chk_zone(T_IED *p_ied, WORD set_no);

BOOL c103_set_zone(T_IED *p_ied, WORD set_no);

/* ��ѹ������� */
BOOL c103_get_sfc(T_IED *p_ied);

BOOL c103_chk_sfc(T_IED *p_ied, WORD sfc_no, WORD sfc_state);

BOOL c103_set_sfc(T_IED *p_ied, WORD sfc_no, WORD sfc_state);

/* ң�������[��Сң�ص�� =1] */
BOOL c103_check_control(T_IED *p_ied, WORD ctrl_no);

BOOL c103_remote_control(T_IED *p_ied, WORD ctrl_no);

/* �źŸ��� */
BOOL c103_reset_signal(T_IED *p_ied);

/* ʱ����� */
static BOOL c103_set_clock(T_IED *p_ied, const T_DATE *p_date);

/* ��������� */
BOOL c103_get_list(T_IED *p_ied, T_LIST *p_list);

/* ȡ103��Ŀ�� */
WORD C103_Get_Code(T_IED* p_ied, T_LIST* p_list, WORD code);

static BOOL c103_device_initialize(T_IED* p_ied);

#endif

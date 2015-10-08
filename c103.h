#ifndef __C103__
#define __C103__

#include "typedef.h"
#include "nucleus.h"
#include "inforec.h"
#include "system.h"
#include "dev_mng.h"
/********************************************************/
/*														*/
/*	外部接口描述										*/
/*														*/
/********************************************************/
/* 103设备服务接口 */
BOOL c103_service(T_IED *p_ied, T_MESSAGE *p_msg);

/********************************************************/
/*														*/
/*	103设备服务接口及内部接口描述						*/
/*														*/
/********************************************************/

typedef struct tagT_C103_CODE_TABLE
{
	T_EVENT_ENTRY  *p_event;	/* 事件码表 */
	T_ALARM_ENTRY  *p_alarm;	/* 告警码表 */
	T_SET_ENTRY    *p_setting;	/* 定值码表 */
	T_SFC_ENTRY	   *p_sfc;		/* 压板码表 */
	T_ANALOG_ENTRY *p_analog;	/* 模拟量码表 */
	WORD			w_even;		/* 事件个数 */
	WORD			w_alarm;	/* 告警个数 */
	WORD			w_setting;	/* 定值个数 */
	WORD			w_sfc;		/* 压板个数 */
	WORD			w_analog;	/* 模拟量个数 */
}	T_C103_CODE_TABLE;

/* 103信息序号 */
typedef struct tagT_C103_DATA_TYPE
{
	//通用类（高，低）：定值，软压板，定值区，遥测，事件，告警，遥信，遥控，
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
	
	//ASDU1（起，尾）：事件，告警，遥信 
	BYTE	byInfEvnStart;
	BYTE	byInfEvnEnd;
	BYTE	byInfAlmStart;
	BYTE	byInfAlmEnd;
	BYTE	byInfDiStart;
	BYTE	byInfDiEnd;
}	T_C103_DATA_TYPE;

/* 103会话设施描述 */
#define C103_ASDU_TIMEOUT	(2*SYS_SECOND_TICKS)
#define C103_ASDU_SIZE		0x300

/* 南瑞母差保护类型 */
#define BP2A	0
#define	BP2B	1

/* 103报文帧格式描述 */
typedef struct tagT_C103_FRAME
{	
	BYTE	type;			/* 帧类型 */
	BYTE	len;			/* 长度   */
	BYTE	contral;		/* 控制域 */
	BYTE	address;		/* 地址域 */
	BYTE	crc;			/* 帧校验和 */
	BYTE	endbyte;		/* 结束字符 */
}	T_C103_FRAME;

//帧格式类型
#define FRAME_T_VOL	0x68	/* 可变帧长 */
#define FRAME_T_CON	0x10	/* 固定帧长 */

/* 103应用服务数据单元(ASDU)格式 */
typedef struct tagT_C103_ASDU
{
	BYTE	type;			/* 类型标识 */
	BYTE	vsq;			/* 可变结构限定词 */
	BYTE	cot;			/* 传送原因 */
	BYTE	addr;			/* 公共地址 */
	BYTE	fun;			/* 功能类型 */
	BYTE	inf;			/* 信息序号 */
	BYTE	data[C103_ASDU_SIZE-7];	/* 信息体报文 */
}	T_C103_ASDU;

/* 103数据(信息元)格式 */
typedef struct tagT_C103_DATA
{
	BYTE	ginh;			/* 通用分类标识序号高 */
	BYTE	ginl;			/* 通用分类标识序号低 */
	BYTE	kod;			/* 描述的类别 */
	BYTE	type;			/* 数据类型 */
	BYTE	size;			/* 数据宽度 */
	BYTE	num;			/* 数目 */
	BYTE	data[16];		/* 数据值 */
}	T_C103_DATA;

/* 系统1ms中断计数器 */
extern volatile DWORD	g_dTCounter;

/* 类型标识 */
#define	ASDU1		1
#define ASDU5		5
#define ASDU8		8
#define ASDU10		10

/* 功能类型 */
#define C103_FUN_DISTANCE	128		//距离保护
#define C103_FUN_OVERCRT	160		//过流保护
#define C103_FUN_TDEFFER	176		//变压器差动保护

#define C103_FUN_CSL101B	178		//四方线路保护
#define C103_FUN_CSL164B	179		//四方线路保护
#define C103_FUN_CSL103C1	180		//四方线路保护主
#define C103_FUN_CSL103C2	181		//四方线路保护后备
#define c103_FUN_CSI101C	188		//四方断路器保护
#define c103_FUN_CSI101A	225		//四方综合重合闸
#define C103_FUN_CSL103B	220		//四方线路保护
#define C103_FUN_CSC161A	242		//四方线路保护

#define C103_FUN_LDEFFER	192		//线路差动保护
#define C103_FUN_BP			210		//深圳南瑞母线保护
#define C103_FUN_GEN		254		//通用分类功能类型
#define C103_FUN_GLB		255		//全局功能类型

/* 通用分类标识码(GINH:组号) */
#define C103_GIN_SYS0	0	//系统组0
#define C103_GIN_SYS1	1	//系统组1
#define C103_GIN_SET0	2	//定值组0
#define C103_GIN_SET1	3	//定值组1
#define C103_GIN_EVENT	4	//保护动作组
#define C103_GIN_ALARM	5	//保护告警组
#define C103_GIN_CHN	6	//保护测量组
#define C103_GIN_MS		7	//遥测组
#define C103_GIN_DI		8	//遥信组
#define C103_GIN_PS		10	//遥脉组
#define C103_GIN_DO		11	//遥控组
#define C103_GIN_TP		12	//分头组(tap position)
#define C103_GIN_YT		13	//遥调组
#define C103_GIN_SFC	14	//软压板组
#define C103_GIN_SOE	24	//遥信SOE

/* kind of description(KOD) */
#define C103_KOD_VAL	1	//value:	实际值
#define C103_KOD_DEF	2	//default:	缺省值
#define C103_KOD_RAN	3	//range:	量程（最小值、最大值、步长）
#define C103_KOD_PRE	5	//precision:精度（n，m）
#define C103_KOD_FAC	6	//factor:	因子
#define C103_KOD_UNI	9	//unit:		单位(量纲)
#define C103_KOD_NAM	10	//name:		名称(描述)

/* type of data(TOD) */
#define C103_TOD_NIL	0	//无数据
#define C103_TOD_ASC	1	//ASCII字符
#define C103_TOD_UINT	3	//无符号整数
#define C103_TOD_SINT	4	//有符号整数
#define C103_TOD_FLOAT	6	//浮点数
#define C103_TOD_R3223	7	//IEEE标准754短实数
#define C103_TOD_R6453	8	//IEEE标准754实数
#define C103_TOD_DPI	9	//双点信息
#define C103_TOD_MSQ	12	//带品质描述的测量值
#define C103_TOD_SOE	18	//带时标的报文

/* 系统组0下的条目定义 */
#define C103_CUR_ZONE	2	//当前定值区
#define C103_RUN_ZONE	3	//运行定值区
#define C103_PLS_STS	5	//脉冲状态(冻结/解冻)
#define C103_SIG_STS	6	//信号状态(复归/未复归)

/* 103协议初始化 */
BOOL c103_initialize();

/* 103设备初始化 */
BOOL c103_open_ied(T_IED *p_ied);

/* 103设备总查询任务 */
VOID task_c103(UNSIGNED argc, VOID *argv);

/* 读取脉冲电度量 */
BOOL c103_get_pulse(T_IED *p_ied, T_PULSE *p_ps);

/* 保护采样通道值 */
BOOL c103_get_channel(T_IED *p_ied, T_CHANNEL *p_chn);

/* 定值类服务 */
BOOL c103_get_setting(T_IED *p_ied, T_SET *p_set);

BOOL c103_chk_setting(T_IED *p_ied, const T_SET *p_set);

BOOL c103_set_setting(T_IED *p_ied, const T_SET *p_set);

BOOL c103_get_zone(T_IED *p_ied, WORD *p_set_no);

BOOL c103_chk_zone(T_IED *p_ied, WORD set_no);

BOOL c103_set_zone(T_IED *p_ied, WORD set_no);

/* 软压板类服务 */
BOOL c103_get_sfc(T_IED *p_ied);

BOOL c103_chk_sfc(T_IED *p_ied, WORD sfc_no, WORD sfc_state);

BOOL c103_set_sfc(T_IED *p_ied, WORD sfc_no, WORD sfc_state);

/* 遥控类服务[最小遥控点号 =1] */
BOOL c103_check_control(T_IED *p_ied, WORD ctrl_no);

BOOL c103_remote_control(T_IED *p_ied, WORD ctrl_no);

/* 信号复归 */
BOOL c103_reset_signal(T_IED *p_ied);

/* 时间服务 */
static BOOL c103_set_clock(T_IED *p_ied, const T_DATE *p_date);

/* 描述表服务 */
BOOL c103_get_list(T_IED *p_ied, T_LIST *p_list);

/* 取103条目号 */
WORD C103_Get_Code(T_IED* p_ied, T_LIST* p_list, WORD code);

static BOOL c103_device_initialize(T_IED* p_ied);

#endif

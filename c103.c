/*****************************************************************************/
/*																			 */
/* 				 版权 (c) 2000-2010 国电南京自动化股份有限公司				 */
/*																			 */
/*****************************************************************************/
/*****************************************************************************/
/*	FILE NAME							VERSION								 */
/*                                                                           */
/*  c103.c	                              1.0								 */
/*                                                                           */
/*  DESCRIPTION                                                              */
/*                                                                           */
/*		深圳南瑞串口103规约接入(DL/T 667-1999)                               */
/*                                                                           */
/*  HISTORY                                                                  */
/*                                                                           */
/*		NAME				DATE				    REMARKS                  */
/*                                                                           */
/*      林俊                2001.10.08              初始版本                 */
/*		yinjun				2015.10.08				git v1.0				 */	
/*                                                                           */
/*****************************************************************************/
/*****************************************************************************/

/******************************串口 103规约***********************************/
/***上行报文：                                                               */
/*                                                                           */
/*  事件，遥信				                      ASDU1                      */
/*  遥测量，定值，软压板                          ASDU10                     */ 
/*                                                                           */
/***下行报文：                                                               */
/*                                                                           */
/*  对时			                              ASDU6                      */
/*  总查询					                      ASDU7					     */
/*  复归保护信号			                      ASDU20                     */
/*  读取（定值、压板状态）						  ASDU21                     */
/*****************************************************************************/
/*****************************************************************************
	苏州慈云变电站
	盐城万盛（盐都）变

	[通讯设置]
	RS232 9600 偶校验	(BP2A只支持9600波特率)
	串口方式：3-收，6-发，7-地
	172.20.100.4

	[说明]
	初始化过程为复位通讯单元，复位正帧记数位。
	平时轮讯过程为请求二级用户数据，有信息时召唤一级用户数据。
	保护采样通道值，定值，压板采用通用分类数据，但在BP-2A装置中压板状态不上送，
	定值上送的顺序也不一定。
	事件，遥信变位采用ASDU1上送，每次送一条信息。
	在通讯过程中，每两条下行命令的时间间隔不超过5秒，否则判为超时，连接将中断

******************************************************************************/
#include "typedef.h"
#include "nucleus.h"
#include "generic.h"
#include "system.h"
#include "inforec.h"
#include "dev_mng.h"
#include "uart.h"
#include "c103.h"

/* 通讯端口 */
static  WORD	m_dev_port=UART_PORT_A;

/* 轮询任务 */
static  NU_TASK m_c103_task;
static  BYTE    m_c103_stack[0x2000];
static  VOID task_c103(UNSIGNED argc, VOID *argv);

/* 通讯资源 */
static  BYTE		 m_tx_buf[UART_PACKET_SIZE];
static  BYTE		 m_rx_buf[UART_PACKET_SIZE];
static  NU_SEMAPHORE m_com_lock;

#define polling_delay_time 2*SYS_SECOND_TICKS

/* 锁定通讯资源 */
#define DEV_LOCK_COM()		NU_Obtain_Semaphore(&m_com_lock, NU_SUSPEND)

/* 开放通讯资源 */
#define DEV_UNLOCK_COM()	NU_Release_Semaphore(&m_com_lock)

/* 103功能类型 */
static	BYTE  m_c103_fun =C103_FUN_BP;			

/* 103扫描序号sn */
volatile BYTE m_c103_sn =0;

/* 103返回信息标识符rii */
volatile BYTE m_c103_rii =0;

/* 103帧记数位fbc */
volatile BYTE m_c103_fcb =0;

extern const T_C103_CODE_TABLE g_c103_code[];
extern const WORD	g_c103_code_size;

/* 接收中断回调函数，用以判断数据帧的结束 */
static WORD c103_check_packet(const BYTE *p_data, WORD data_len)
{
	if ((data_len == 1) && ((p_data[0] == 0x68) || (p_data[0] == 0x10)))
	{
		return UART_PKT_CON;
	}
	if (data_len < 5)
	{
		return UART_PKT_CON;
	}
	if (p_data[0] == 0x10 && data_len == 5 && p_data[4] == 0x16)
	{
		return UART_PKT_FIN;
	}
	else if (p_data[0] == 0x68 && data_len == p_data[1]+6 && p_data[data_len-1] == 0x16)
	{
		return UART_PKT_FIN;
	}
	return UART_PKT_CON;
}

/* 校验和 */
BYTE  calc_check_sum(const BYTE *p_data, WORD data_len)
{
	WORD  i;
	BYTE  sum =0;

	for (i =0; i < data_len; i++)
	{
		sum += p_data[i];
	}
	return sum;
}

/* 串口管理 */
WORD c103_serial_com(T_IED* p_ied,T_C103_FRAME* p_frame,BOOL brx)
{
	WORD wSize,size,i;

	m_c103_fcb ^=0x20;	//帧记数位取反

	switch(p_frame->type)
	{
	case FRAME_T_CON:
		/* 固定帧长报文 */
		m_tx_buf[0] =p_frame->type;
		m_tx_buf[1] =p_frame->contral | m_c103_fcb;
		m_tx_buf[2] =p_frame->address;
		m_tx_buf[3] =calc_check_sum(&m_tx_buf[1], 2);
		m_tx_buf[4] =p_frame->endbyte;

		wSize =5;
		break;
	case FRAME_T_VOL:
		/* 可变帧长报文 */
		m_tx_buf[0] =p_frame->type;
		m_tx_buf[1] =p_frame->len;
		m_tx_buf[2] =p_frame->len;
		m_tx_buf[3] =p_frame->type;
		m_tx_buf[4] =p_frame->contral | m_c103_fcb;
		m_tx_buf[5] =p_frame->address;
		m_tx_buf[4 +p_frame->len] =calc_check_sum(&m_tx_buf[4], p_frame->len);
		m_tx_buf[5 +p_frame->len] =p_frame->endbyte;

		wSize =p_frame->len +6;
		break;
	default:
		break;
	}

	/* 出现差错重传三次 */
	for(i =0; i <3; i++)
	{
		UART_Write(m_dev_port, m_tx_buf, wSize);
		if (!brx) break;

		size = UART_Read(m_dev_port,m_rx_buf,1000);
		if (size !=0) break;
	}

	/* 接收报文分类 */
	if (size)
	{
		switch (m_rx_buf[0])
		{
		case FRAME_T_CON:	//固定帧长
			p_frame->type =FRAME_T_CON;
			p_frame->contral =m_rx_buf[1];
			p_frame->address =m_rx_buf[2];
			p_frame->crc =m_rx_buf[3];

			if (p_frame->crc ==calc_check_sum(&m_rx_buf[1], 2) 
					&& p_ied->dev_id == p_frame->address);		//校验
			{
				memset(&m_rx_buf[5], 0, (UART_PACKET_SIZE-5));
				return size; 
			}
			break;
		case FRAME_T_VOL:	//可变帧长
			p_frame->type =FRAME_T_VOL;
			p_frame->len =m_rx_buf[1];
			p_frame->contral =m_rx_buf[4];
			p_frame->address =m_rx_buf[5];
			p_frame->crc =m_rx_buf[p_frame->len +4];

			if (p_frame->crc ==calc_check_sum(&m_rx_buf[4], p_frame->len)
					&& p_ied->dev_id == p_frame->address);		//校验
			{
				return size;	
			}
			break;
		}
	}

	if (brx && !size && p_frame->contral != 0x40)	//超时无应答发送复位(复位无应答返回)
	{
		c103_device_initialize(p_ied);
	}

	return 0;
}

/********************************************************/
/*														*/
/*	103 IED侧协议解释器									*/
/*														*/
/********************************************************/
/* 请求一级用户数据 */
BOOL c103_request_first(T_IED* p_ied)
{
	T_DATE	 t_date;
	T_EVENT  t_event;
	T_ALARM  t_alarm;
	T_LIST	 t_list; 
	T_DI	 t_di ={48,{0}};
	T_C103_FRAME t_frame;
	T_C103_ASDU* p_asdu;
//	int		i;

	t_frame.type = FRAME_T_CON;
	t_frame.contral = 0x5a;
	t_frame.address = (BYTE)p_ied->dev_id;
	t_frame.len =3;
	t_frame.endbyte =0x16;

	if ( !c103_serial_com(p_ied, &t_frame, 1) )
	{
		return false;
	}

	p_asdu = (T_C103_ASDU *) &m_rx_buf[6];
	switch (p_asdu->type)
	{
	case ASDU5:
		if (p_asdu->cot ==4 || p_asdu->cot ==5 || p_asdu->cot ==3)	//复位通信单元
		break;
		return	false;
	case ASDU8:
		if (p_asdu->cot !=9 || p_asdu->inf !=0)	//总查询结束
		break;
		return	false;
	case ASDU1:		//有保护事件/自检信息
		if (p_asdu->cot != 1)	return false;

/*		if (p_asdu->inf >29 && p_asdu->inf <78)		//遥信量写库，没有做SOE处理
		{
			i =p_asdu->inf -30;
			t_di.di_val[i/32] |= (p_asdu->data[0] << (i % 32));
			break;
		}
*/
		SYS_Get_Clock(&t_date);

		t_event.dev_id =p_ied->dev_id;
		t_event.e_state =p_asdu->data[0] -1;
		if (t_event.e_state == 0)		//林俊，2002,6,8,为与94规约配合，不送保护动作返回信号
		{
			break;				
		}
		t_event.e_date.msec =MAKEWORD(p_asdu->data[1],p_asdu->data[2]);
		t_event.e_date.minute =p_asdu->data[3];
		t_event.e_date.hour =p_asdu->data[4];
		t_event.e_date.day =t_date.day;
		t_event.e_date.month =t_date.month;
		t_event.e_date.year =t_date.year;
		
		if (p_asdu->inf < 30 || p_asdu->inf >=255)	//告警事件
		{
			t_list.l_type =LIST_T_ALARM;
			t_event.e_code =C103_Get_Code(p_ied, &t_list, p_asdu->inf);
//			T_event =t_alarm;
			INF_Record_Alarm(&t_alarm);
		}
		else if (p_asdu->inf < 255 && p_asdu->inf > 30)		//保护动作
		{
			t_list.l_type =LIST_T_EVENT;
			t_event.e_code =C103_Get_Code(p_ied, &t_list, p_asdu->inf);
			INF_Record_Event(&t_event);
		}
		break;
	case ASDU10:
		if (p_asdu->cot !=42 || p_asdu->fun !=254 || p_asdu->inf !=241)	 return false;
		return true;	
		break;
	}

	if ( (t_frame.contral &0x20) !=0 )		//检查是否有一级用户数据
	{
		return c103_request_first( p_ied);
	}
	if (t_frame.contral == 0x08 || t_frame.contral ==0x09)
	{
		return true;
	}

	return false;
}

/* 请求二级用户数据 */
BOOL c103_request_second(T_IED* p_ied)
{
	T_C103_FRAME t_frame;

	t_frame.type = FRAME_T_CON;
	t_frame.contral = 0x5b;
	t_frame.address = (BYTE)p_ied->dev_id;
	t_frame.len =3;
	t_frame.endbyte =0x16;

	if ( !c103_serial_com(p_ied, &t_frame, 1) )
	{
		return false;
	}

	if ( t_frame.type ==FRAME_T_CON )
	{
		if ( (t_frame.contral &0x20) !=0 )	//有一级用户数据
		{
			return c103_request_first(p_ied);
		}
		if ( (t_frame.contral &0x20) ==0)	//无所响应的数据
			return true;
	}
	return false;
}

/* 主站ASDU21读一个(组/条目)的属性/值 */
/*	p_ied:	设备标识
	ginh:	组号
	ginl:	条目号(0:全部条目)
	kod:	数据类型
*/
static BOOL c103_generic_read(T_IED *p_ied, BYTE ginh, BYTE ginl, BYTE kod)
{
	T_C103_FRAME t_frame;
	T_C103_ASDU* p_asdu;

	/* 构建ASDU21通用分类读命令 */
	p_asdu =(T_C103_ASDU *)&m_tx_buf[6];
	p_asdu->type =21;
	p_asdu->vsq =0x81;
	p_asdu->cot =42;
	p_asdu->addr = (BYTE)p_ied->dev_id;
	p_asdu->fun =254;
	p_asdu->inf =241;
	p_asdu->data[0] =m_c103_rii++;
	p_asdu->data[1] =1;
	p_asdu->data[2] =ginh;
	p_asdu->data[3] =ginl;
	p_asdu->data[4] =kod;
	
	t_frame.type = FRAME_T_VOL;
	t_frame.contral = 0x53;
	t_frame.address = (BYTE)p_ied->dev_id;
	t_frame.len =13;
	t_frame.endbyte =0x16;

	if (c103_serial_com(p_ied, &t_frame, 1))
	{
		if (t_frame.type ==0x10 && (t_frame.contral &0x20)!=0 
			&& t_frame.address ==p_ied->dev_id)	
		{
			if (c103_request_first(p_ied)) 
			{
				return true;
			}
		}
	}

	return false;
}

/* 主站ASDU10带确认写一个(组/条目)的值 */
/*	p_ied:	设备标识
	ngd:	通用数据数目
*/
static BOOL c103_generic_ackwrite(T_IED *p_ied, BYTE ngd, BYTE len)
{
	T_C103_FRAME t_frame;
	T_C103_ASDU* p_asdu;

	/* 加入ASDU10通用分类数据写报文头 */
	p_asdu =(T_C103_ASDU *)&m_tx_buf[6];
	p_asdu->type =10;
	p_asdu->vsq =0x81;
	p_asdu->cot =40;
	p_asdu->addr = (BYTE)p_ied->dev_id;
	p_asdu->fun =254;
	p_asdu->inf =249;
	p_asdu->data[0] =m_c103_rii++;
	p_asdu->data[1] =ngd;

	/* 帧格式 */
	t_frame.type = FRAME_T_VOL;
	t_frame.contral = 0x53;
	t_frame.address = (BYTE)p_ied->dev_id;
	t_frame.len = len;
	t_frame.endbyte =0x16;

	if (c103_serial_com(p_ied, &t_frame, 1))
	{
		if (t_frame.type ==0x10 && t_frame.contral ==0x21 
			&& t_frame.address ==p_ied->dev_id)		//接收数据识别
		{
			c103_request_first(p_ied);
		}
		if (t_frame.type ==0x68 && t_frame.contral ==0 
			&& t_frame.address ==p_ied->dev_id)
		{
			p_asdu =(T_C103_ASDU *) &m_rx_buf[6];
			/* 校验ASDU信息 */
			if ((p_asdu->cot != 44) || (p_asdu->fun != 254) || (p_asdu->inf != 249))
			{
				return false;
			}
			return true;
		}
	}

	return false;
}

/* 主站ASDU10带执行写一个(组/条目)的值 */
/*	p_ied:	设备标识
	ngd:	通用数据数目
*/
static BOOL c103_generic_exewrite(T_IED *p_ied, BYTE ngd, BYTE len)
{
	T_C103_FRAME t_frame;
	T_C103_ASDU* p_asdu;

	/* 加入ASDU10通用分类数据写报文头 */
	p_asdu =(T_C103_ASDU *) &m_tx_buf[6];
	p_asdu->type=10;
	p_asdu->vsq =0x81;
	p_asdu->cot =40;
	p_asdu->addr= (BYTE)p_ied->dev_id;
	p_asdu->fun =254;
	p_asdu->inf =250;
	p_asdu->data[0] =m_c103_rii++;
	p_asdu->data[1] =ngd;


	/* 帧格式 */
	t_frame.type = FRAME_T_VOL;
	t_frame.contral = 0x53;
	t_frame.address = (BYTE)p_ied->dev_id;
	t_frame.len = len;
	t_frame.endbyte =0x16;

	if (c103_serial_com(p_ied, &t_frame, 1))
	{
		if (t_frame.type ==0x10 && t_frame.contral ==0x21 
			&& t_frame.address ==p_ied->dev_id)
		{
			c103_request_first(p_ied);
		}
		if (t_frame.type ==0x68 && t_frame.contral ==0 
			&& t_frame.address ==p_ied->dev_id)
		{
			p_asdu =(T_C103_ASDU *)m_rx_buf[6];
			/* 校验ASDU信息 */
			if ((p_asdu->cot != 40) || (p_asdu->fun != 254) || (p_asdu->inf != 250))
			{
				return false;
			}
			return true;
		}
	}

	return false;
}

/********************************************************/
/*														*/
/*	103设备服务接口及内部接口定义						*/
/*														*/
/********************************************************/
/* 读取脉冲电度量 */
BOOL c103_get_pulse(T_IED *p_ied, T_PULSE *p_ps)
{
	return TRUE;
}

/* 保护采样通道值 */
BOOL c103_get_analog(T_IED *p_ied, T_CHANNEL *p_chn)
{
	T_LIST	 t_list; 
	T_C103_ASDU *p_asdu;
	T_C103_DATA *p_rec;
	BYTE		i,ginh,ginl,kod;

	// 解析数据 
	switch (p_ied->user_defined_1)
	{
	case BP2A:
		ginh = 0x02;
		ginl = 0x00;
		kod  = 0x01;
		break;
	case BP2B:
		ginh = 0x03;
		ginl = 0x00;
		kod  = 0x01;
		break;
	default:
		break;
	}

	p_asdu =(T_C103_ASDU *)&m_tx_buf[6];

	if ( !c103_generic_read(p_ied, ginh, ginl, kod) )
	{
		return FALSE;
	}

	p_asdu =(T_C103_ASDU *) &m_rx_buf[6];

	p_chn->chn_num =p_asdu->data[1];
	if (p_chn->chn_num > SYS_MAX_CHANNEL)
	{
		p_chn->chn_num =SYS_MAX_CHANNEL;
	}
	p_rec =(T_C103_DATA *)&p_asdu->data[2];

	for (i =0; i < p_chn->chn_num; i++)
	{
		WORD j;
		if ((p_rec->ginh != ginh) || (p_rec->kod != kod) || (p_rec->num != 1))
		{
			return FALSE;
		}

		if (p_rec->type == 0x40)
		{
			float f_val;
			f_val = MAKEWORD(p_rec->data[0],p_rec->data[1]) /100.0;
			t_list.l_type = LIST_T_ANALOG;
			j =C103_Get_Code(p_ied, &t_list, p_rec->ginl);
			p_chn->chn_val[i] =f_val;
		}
		else if (p_rec->type == 0x07)
		{
			float f_val;
			gen_scan_float(&p_rec->data[0], &f_val);	//intel浮点数转motorola浮点数
			t_list.l_type = LIST_T_ANALOG;
			j =C103_Get_Code(p_ied, &t_list, p_rec->ginl);
			p_chn->chn_val[i] =f_val;
		}
		else
		{
			p_chn->chn_val[i] =(float)p_rec->data[0];
		}
		p_rec =(T_C103_DATA *)((BYTE *)p_rec+6+p_rec->num*p_rec->size);
	}
	return TRUE;
}

/* 选择当前操作定值区 */
static BOOL c103_select_zone(T_IED *p_ied, BYTE set_no)
{
	BYTE len;
	T_C103_ASDU *p_asdu;
	T_C103_DATA *p_rec;
	BYTE		ngd =1;

	p_asdu =(T_C103_ASDU *)&m_tx_buf[6];

	p_rec =(T_C103_DATA *)&p_asdu->data[2];
	p_rec->ginh	=0;
	p_rec->ginl	=2;
	p_rec->kod	=1;
	p_rec->type	=3;
	p_rec->size	=1;
	p_rec->num	=1;
	p_rec->data[0] =set_no;

	len =17;

	if ( !c103_generic_exewrite(p_ied, ngd, len) )
	{
		return false;
	}

	p_asdu =(T_C103_ASDU *) &m_rx_buf[6];
	p_rec  =(T_C103_DATA *)&p_asdu->data[2];

	if ((p_rec->ginh != 0) || (p_rec->ginl != 2) || (p_rec->kod != 1))
	{
		return false;
	}
	return (p_rec->data[0] == set_no);
}

/* 定值类服务 */
BOOL c103_get_setting(T_IED *p_ied, T_SET *p_set)
{
	T_LIST	 t_list; 
	T_C103_ASDU* p_asdu;
	T_C103_DATA* p_rec;
	BYTE		i,ginh,ginl,kod;

	/* 选择当前操作定值区 */
//	if ( !c103_select_zone(p_ied, (BYTE)(p_set->set_no)) )
//	{
//		return FALSE;
//	}

	switch (p_ied->user_defined_1)
	{
	case BP2A:
		ginh = 0x01;
		ginl = 0x00;
		kod  = 0x01;
		break;
	case BP2B:
		ginh = (BYTE) p_set->set_no;
		ginl = 0x00;
		kod  = 0x01;
		break;
	default:
		break;
	}

	p_asdu =(T_C103_ASDU *)&m_tx_buf[6];

	if ( !c103_generic_read(p_ied, ginh, ginl, kod) )
	{
		return FALSE;
	}

	/* 解析数据 */
	p_asdu =(T_C103_ASDU *)&m_rx_buf[6];
	p_set->set_num =p_asdu->data[1];
	if (p_set->set_num > SYS_MAX_SET)
	{
		p_set->set_num =SYS_MAX_SET;
	}
	p_rec =(T_C103_DATA *)&p_asdu->data[2];

	for (i =0; i < p_set->set_num; i++)
	{
		WORD j;
		if ( (p_rec->ginh != ginh) && (p_rec->kod != kod)  || (p_rec->num != 1))
		{
			return false;
		}

//		j =p_rec->ginl;
		if (p_rec->type == 0x40)
		{
			float f_val;		//定值格式转换
			f_val = MAKEWORD(p_rec->data[0], p_rec->data[1]) /100.0;
			t_list.l_type =LIST_T_SET;
			j =C103_Get_Code(p_ied, &t_list, p_rec->ginl) -1;
			p_set->set_val[j].type =SET_T_FLOAT;
			p_set->set_val[j].un_val.f_val =f_val;
		}
		else if (p_rec->type == 0x07)
		{
			float f_val;
			gen_scan_float(&p_rec->data[0], &f_val);	//intel浮点数转motorola浮点数
//			t_list.l_type =LIST_T_SET;
//			j =C103_Get_Code(p_ied, &t_list, p_rec->ginl);
			p_set->set_val[i].type =SET_T_FLOAT;
			p_set->set_val[i].un_val.f_val =f_val;
		}
		else
		{
			p_set->set_val[i].type =SET_T_UINT;
			p_set->set_val[i].un_val.u_val =p_rec->data[0];
		}
		p_rec =(T_C103_DATA *)((BYTE *)p_rec+6+p_rec->num*p_rec->size);
	}

	return true;
}

/* 将定值转存为103格式 */
static WORD c103_print_setting(BYTE *p_data, const T_SET *p_set)
{
	WORD		i;
	WORD		data_len =0;
	T_C103_DATA *p_rec =(T_C103_DATA *)p_data;
	
	for (i =0; i < p_set->set_num; i++)
	{
		p_rec->ginh =0x01;
		p_rec->ginl =i+1;
		p_rec->kod  =1;
		p_rec->num  =1;
		if (p_set->set_val[i].type == SET_T_FLOAT)
		{
			p_rec->type =0x40;		//用户自定义的类型
			p_rec->size =2;
			p_rec->data[0] =LOBYTE( LOWORD((DWORD)p_set->set_val[i].un_val.f_val) );
			p_rec->data[1] =HIBYTE( LOWORD((DWORD)p_set->set_val[i].un_val.f_val) );
		}
		else
		{
			p_rec->type =3;
			p_rec->size =2;
			p_rec->data[0] =LOBYTE(p_set->set_val[i].un_val.u_val);
			p_rec->data[1] =HIBYTE(p_set->set_val[i].un_val.u_val);
		}
		data_len +=6+p_rec->num*p_rec->size;
		p_rec =(T_C103_DATA *)(p_data + data_len);
	}
	return data_len;
}

BOOL c103_chk_setting(T_IED *p_ied, const T_SET *p_set)
{
	BYTE len;
	T_C103_ASDU* p_asdu;

	/* 选择当前操作定值区 */
//	if ( !c103_select_zone(p_ied, (BYTE)p_set->set_no) )
//	{
//		return false;
//	}

	p_asdu =(T_C103_ASDU *)&m_tx_buf[6];

	len =c103_print_setting(&p_asdu->data[2], p_set) + 10;

	if ( !c103_generic_ackwrite(p_ied, (BYTE)p_set->set_num, len) )
	{
		return false;
	}

	p_asdu =(T_C103_ASDU *)&m_rx_buf[6];
	//此处忽略定值校验
	if (p_asdu->data[1] != p_set->set_num)
	{
		return false;
	}

	return true;
}

BOOL c103_set_setting(T_IED *p_ied, const T_SET *p_set)
{
	BYTE len;
	T_C103_ASDU* p_asdu;

	/* 选择当前操作定值区 */
//	if ( !c103_select_zone(p_ied, (BYTE)p_set->set_no) )
//	{
//		return false;
//	}

	p_asdu =(T_C103_ASDU *)&m_tx_buf[6];

	len =c103_print_setting(&p_asdu->data[2], p_set) + 10;

	if ( !c103_generic_exewrite(p_ied, (BYTE)p_set->set_num, len) )
	{
		return false;
	}

	p_asdu =(T_C103_ASDU *)&m_rx_buf[6];
	//此处忽略定值校验
	if (p_asdu->data[1] != p_set->set_num)
	{
		return false;
	}
	return true;
}

BOOL c103_get_zone(T_IED *p_ied, WORD *p_set_no)
{
	T_C103_ASDU *p_asdu;
	T_C103_DATA *p_rec;

	p_asdu =(T_C103_ASDU *)&m_tx_buf[6];

	if ( !c103_generic_read(p_ied, 0x00, 0x03, 0x01))
	{
		return false;
	}

	p_asdu =(T_C103_ASDU *)&m_rx_buf[6];
	p_rec  =(T_C103_DATA *)&p_asdu->data[2];

	if ((p_rec->ginh != 0x00) || (p_rec->ginl != 0x03) || (p_rec->kod != 0x01))
	{
		return false;
	}

	*p_set_no =p_rec->data[0];

	return true;
}

BOOL c103_chk_zone(T_IED *p_ied, WORD set_no)
{
	BYTE len;
	T_C103_ASDU *p_asdu;
	T_C103_DATA *p_rec;
	BYTE		ngd =1;

	p_asdu =(T_C103_ASDU *)&m_tx_buf[6];

	p_rec  =(T_C103_DATA *)&p_asdu->data[2];
	p_rec->ginh	=0x00;
	p_rec->ginl	=0x03;
	p_rec->kod	=0x01;
	p_rec->type	=3;
	p_rec->size	=1;
	p_rec->num	=1;
	p_rec->data[0] =(BYTE)set_no;

	len =17;

	if ( !c103_generic_ackwrite(p_ied, ngd, len) )
	{
		return false;
	}
	
	p_asdu =(T_C103_ASDU *)&m_rx_buf[6];
	p_rec  =(T_C103_DATA *)&p_asdu->data[2];

	if ((p_rec->ginh != 0x00) || (p_rec->ginl != 0x03) || (p_rec->kod != 0x01) ||
		(p_rec->data[0] != set_no))
	{
		return false;
	}

	return true;
}

BOOL c103_set_zone(T_IED *p_ied, WORD set_no)
{
	BYTE len;
	T_C103_ASDU *p_asdu;
	T_C103_DATA *p_rec;
	BYTE		ngd =1;

	p_asdu =(T_C103_ASDU *)&m_tx_buf[6];

	p_rec  =(T_C103_DATA *)&p_asdu->data[2];
	p_rec->ginh	=0x00;
	p_rec->ginl	=0x03;
	p_rec->kod	=0x01;
	p_rec->type	=3;
	p_rec->size	=1;
	p_rec->num	=1;
	p_rec->data[0] =(BYTE)set_no;

	len =17;

	if ( !c103_generic_exewrite(p_ied, ngd, len) )
	{
		return false;
	}

	p_asdu =(T_C103_ASDU *)&m_rx_buf[6];
	p_rec  =(T_C103_DATA *)&p_asdu->data[2];

	if ((p_rec->ginh != 0x00) || (p_rec->ginl != 0x03) || (p_rec->kod != 0x01) ||
		(p_rec->data[0] != set_no))
	{
		return false;
	}

	return true;
}

/* 软压板类服务 */
BOOL c103_get_sfc(T_IED *p_ied)
{
	T_C103_ASDU* p_asdu;
	T_C103_DATA* p_rec;
	T_SFC t_sfc;
	BYTE		i,ginh,ginl,kod;

	switch (p_ied->user_defined_1)
	{
	case BP2A:
		ginh = 0x03;
		ginl = 0x00;
		kod  = 0x01;
		break;
	case BP2B:
		ginh = 0x02;
		ginl = 0x00;
		kod  = 0x01;
		break;
	default:
		break;
	}

	p_asdu =(T_C103_ASDU *)&m_tx_buf[6];

	if ( !c103_generic_read(p_ied, ginh, ginl, kod) )
	{
		return FALSE;
	}

	p_asdu =(T_C103_ASDU *)&m_rx_buf[6];
	/* 解析数据 */
	t_sfc.sfc_num =p_asdu->data[1];
	if (t_sfc.sfc_num > SYS_MAX_SFC)
	{
		t_sfc.sfc_num =SYS_MAX_SFC;
	}
	p_rec =(T_C103_DATA *)&p_asdu->data[2];

	for (i =1; i <= t_sfc.sfc_num; i++)
	{
		if ((p_rec->ginh != ginh) || (p_rec->kod != kod) || (p_rec->num != 1))
		{
			return FALSE;
		}
		if ( (p_rec->data[0] /0x32) == 0x01 )
		{
			INF_Preset_SFC(&t_sfc, i, INF_S_ON);
		}
		else if  ( (p_rec->data[0] /0x32) == 0x02 )
		{
			INF_Preset_SFC(&t_sfc, i, INF_S_OFF);
		}
		p_rec =(T_C103_DATA *)((BYTE *)p_rec+6+p_rec->num*p_rec->size);
	}

	INF_Set_SFC(p_ied->dev_id, &t_sfc);
	return TRUE;
}

BOOL c103_chk_sfc(T_IED *p_ied, WORD sfc_no, WORD sfc_state)
{
	BYTE len;
	T_C103_ASDU *p_asdu;
	T_C103_DATA *p_rec;
	BYTE		dpi =(sfc_state == 0)?0x64:0x32;
	BYTE		ngd  =1; // 通用分类数据集数目

	p_asdu =(T_C103_ASDU *)&m_tx_buf[6];

	p_rec =(T_C103_DATA*)&p_asdu->data[2];
	p_rec->ginh	=0x03;
	p_rec->ginl	= (BYTE)sfc_no;
	p_rec->kod	=0x01;
	p_rec->type	=0x01;
	p_rec->size	=1;
	p_rec->num	=1;
	p_rec->data[0] = dpi;

	len =17;

	if ( !c103_generic_ackwrite(p_ied, ngd, len) )
	{
		return false;
	}

	p_asdu =(T_C103_ASDU *)&m_rx_buf[6];
	p_rec  =(T_C103_DATA *)&p_asdu->data[2];
	if ((p_rec->ginh != 0x03) || (p_rec->ginl != sfc_no) || (p_rec->kod != 0x01) ||
		(p_rec->data[0] != dpi))
	{
		return false;
	}

	return true;
}

BOOL c103_set_sfc(T_IED *p_ied, WORD sfc_no, WORD sfc_state)
{
	BYTE len;
	T_C103_ASDU *p_asdu;
	T_C103_DATA *p_rec;
	BYTE		dpi =(sfc_state == 0)?0x64:0x32;
	BYTE		ngd  =1; // 通用分类数据集数目

	p_asdu =(T_C103_ASDU *)&m_tx_buf[6];

	p_rec =(T_C103_DATA*)&p_asdu->data[2];
	p_rec->ginh	=0x0E;
	p_rec->ginl	= (BYTE)sfc_no;
	p_rec->kod	=0x01;
	p_rec->type	=0x09;
	p_rec->size	=1;
	p_rec->num	=1;
	p_rec->data[0] =dpi;

	len =17;

	if ( !c103_generic_exewrite(p_ied, ngd, len) )
	{
		return false;
	}

	p_asdu =(T_C103_ASDU *)&m_rx_buf[6];
	p_rec  =(T_C103_DATA *)&p_asdu->data[2];
	if ((p_rec->ginh != 0x0E) || (p_rec->ginl != sfc_no) || (p_rec->kod != 0x01) ||
		(p_rec->data[0] != dpi))
	{
		return false;
	}

	return true;
}

/* 遥控类服务[最小遥控点号 =1] */
BOOL c103_check_control(T_IED *p_ied, WORD ctrl_no)
{
	BYTE len;
	T_C103_ASDU* p_asdu;
	T_C103_DATA* p_rec;
	BYTE		ginl =(ctrl_no+1)/2;
	BYTE		dpi  =((ctrl_no % 2) != 0)?0x01:0x02;
	BYTE		ngd  =1; // 通用分类数据集数目

	p_asdu =(T_C103_ASDU *)&m_tx_buf[6];

	p_rec =(T_C103_DATA *)&p_asdu->data[2];
	p_rec->ginh	=0x0B;
	p_rec->ginl	=ginl;
	p_rec->kod	=0x01;
	p_rec->type	=9;
	p_rec->size	=1;
	p_rec->num	=1;
	p_rec->data[0] =dpi;

	len =17;

	if ( !c103_generic_ackwrite(p_ied, ngd, len) )
	{
		return false;
	}

	p_asdu =(T_C103_ASDU *)&m_rx_buf[6];
	p_rec  =(T_C103_DATA *)&p_asdu->data[2];
	if ((p_rec->ginh != 0x0B) || (p_rec->ginl != ginl) || (p_rec->kod != 0x01) ||
		(p_rec->data[0] != dpi))
	{
		return false;
	}

	return true;
}

BOOL c103_remote_control(T_IED *p_ied, WORD ctrl_no)
{
	T_C103_FRAME  t_frame;
	T_C103_ASDU* p_asdu;
	BYTE		inf = (BYTE)(ctrl_no+1)/2;
	BYTE		dpi  = (BYTE)((ctrl_no % 2) != 0)?0x01:0x02;
	BYTE		ngd  =1; // 通用分类数据集数目

	p_asdu =(T_C103_ASDU *)&m_tx_buf[6];
	p_asdu->type =20;
	p_asdu->vsq	=0x81;
	p_asdu->cot	=20;
	p_asdu->addr = (BYTE)p_ied->dev_id;
	p_asdu->fun	= m_c103_fun;			
	p_asdu->inf	= inf;
	p_asdu->data[0] = dpi;
	p_asdu->data[1] = m_c103_rii++;

	t_frame.type = FRAME_T_VOL;
	t_frame.contral = 0x53;		//发送/确认
	t_frame.address = (BYTE)p_ied->dev_id;
	t_frame.len =10;
	t_frame.endbyte =0x16;

	if (c103_serial_com(p_ied, &t_frame, 1))
	{
		p_asdu =(T_C103_ASDU *)&m_rx_buf[6];
		if ( (t_frame.contral &0x0F) == 0 && p_asdu->cot == 20 && p_asdu->inf == inf)
		{
			return true;
		}
	}

	return true;
}

/* 信号复归 */
BOOL c103_reset_signal(T_IED *p_ied)
{
	T_C103_ASDU* p_asdu;
	T_C103_FRAME t_frame;

	/* 执行信号复归 */
	p_asdu =(T_C103_ASDU *)&m_tx_buf[6];
	p_asdu->type =20;
	p_asdu->vsq	=0x81;
	p_asdu->cot	=20;
	p_asdu->addr = (BYTE)p_ied->dev_id;
	p_asdu->fun	=m_c103_fun;			
	p_asdu->inf	=19;
	p_asdu->data[0] =0x01;
	p_asdu->data[1] =m_c103_rii++;

	t_frame.type = FRAME_T_VOL;
	t_frame.contral = 0x53;		//发送/确认
	t_frame.address = (BYTE)p_ied->dev_id;
	t_frame.len =10;
	t_frame.endbyte =0x16;

	if (c103_serial_com(p_ied, &t_frame, 1))
	{
		if (t_frame.type ==0x10 && (t_frame.contral &0xF0) ==0x20 && t_frame.address ==p_ied->dev_id)
		{
			c103_request_first(p_ied);
		}
		p_asdu =(T_C103_ASDU *)&m_rx_buf[6];
		if ( (t_frame.contral &0x0F) == 0 && p_asdu->cot == 20 && p_asdu->inf == 19)
		{
			return true;
		}
	}

	return false;
}


/* 时间服务 */
BOOL c103_set_clock(T_IED *p_ied, const T_DATE *p_date)
{
	T_C103_ASDU* p_asdu;
	T_C103_FRAME t_frame;

	/* 时间同步报文 */
	p_asdu =(T_C103_ASDU *)&m_tx_buf[6];
	p_asdu->type =6;
	p_asdu->vsq  =0x81;
	p_asdu->cot  =8;
	p_asdu->addr = 0xFF;
	p_asdu->fun  =255;
	p_asdu->inf  =0;
	p_asdu->data[0] =LOBYTE(p_date->msec);
	p_asdu->data[1] =HIBYTE(p_date->msec);
	p_asdu->data[2] =p_date->minute;
	p_asdu->data[3] =p_date->hour;
	p_asdu->data[4] =((p_date->week << 5) & 0xE0) | (p_date->day & 0x1F);
	p_asdu->data[5] =p_date->month;
	p_asdu->data[6] =p_date->year - 2000;

	t_frame.type = FRAME_T_VOL;
	t_frame.contral = 0x44;			//发送/无回答
	t_frame.address = 0xFF;
	t_frame.len =15;
	t_frame.endbyte =0x16;

	m_c103_fcb =0x20;

	c103_serial_com(p_ied, &t_frame, 0);
	
	SYS_Set_Clock(p_date);
	
	return TRUE;
}

/* 协议初始化 */
static BOOL c103_initialize()
{
	T_UART_CONFIG  config;
	STATUS	status;

	if (m_dev_port == UART_PORT_A)
	{
		config =g_sys_config.uart_a;
	}
	else if (m_dev_port == UART_PORT_B)
	{
		config =g_sys_config.uart_b;
	}
	else
	{
		return FALSE;
	}
	config.pcb =c103_check_packet;

	if ( !UART_Open(m_dev_port, &config) )
	{
		return FALSE;
	}
	NU_Create_Semaphore(&m_com_lock, "COM_LOCK", 1, NU_FIFO);
	
	status =NU_Create_Task(&m_c103_task, "TASK_C103", task_c103, 0, NU_NULL,
					        m_c103_stack, 0x2000, 10, 0, NU_PREEMPT, NU_START);

	return (status == NU_SUCCESS);
}

/* 设备初始化 */
static BOOL c103_open_ied(T_IED *p_ied)
{
	return true;
}

/* 设备描述表 */
BOOL c103_get_List(T_IED *p_ied, T_LIST* p_list)
{
	switch(p_list->l_type)
	{				
	case LIST_T_SET:
		{
			p_list->l_ptr = g_c103_code[0].p_setting;
			p_list->l_size = g_c103_code[0].w_setting;
			return true;
		}
	case LIST_T_EVENT:
		{
			p_list->l_ptr = g_c103_code[0].p_event;
			p_list->l_size = g_c103_code[0].w_even;
			return true;
		}
	case LIST_T_ALARM:
		{
			p_list->l_ptr = g_c103_code[0].p_alarm;
			p_list->l_size = g_c103_code[0].w_alarm;
			return true;
		}
	case LIST_T_SFC:
		{
			p_list->l_ptr = g_c103_code[0].p_sfc;
			p_list->l_size = g_c103_code[0].w_sfc;
			return true;
		}
	case LIST_T_ANALOG:
		{
			p_list->l_ptr = g_c103_code[0].p_analog;
			p_list->l_size = g_c103_code[0].w_analog;
			return true;
		}
	}
	return false;
}

/* 取103条目号 */
static WORD C103_Get_Code(T_IED* p_ied, T_LIST* p_list, WORD code)
{
	if(c103_get_List(p_ied,p_list))
	{	
		switch(p_list->l_type)
		{
			case  LIST_T_EVENT:
			{
				int i;
				T_EVENT_ENTRY* p_event = (T_EVENT_ENTRY*) p_list->l_ptr;
				for(i = 0; i< p_list->l_size; i++)
				{
					if(p_event[i].e_key == code)
					return i+1;
				}		
			}
			case LIST_T_ALARM:
			{
				int i;
				T_ALARM_ENTRY* p_alarm = (T_ALARM_ENTRY*) p_list->l_ptr;
				for(i = 0; i< p_list->l_size; i++)
				{
					if(p_alarm[i].e_key  == code)
					return i+1;
				}
			}
			case LIST_T_SET:
			{
				int i;
				T_SET_ENTRY* p_set = (T_SET_ENTRY*) p_list->l_ptr;
				for(i = 0; i< p_list->l_size; i++)
				{
					if(p_set[i].e_key  == code)
					return i+1;
				}	
			}
			case LIST_T_ANALOG:
			{
				int i;
				T_ANALOG_ENTRY* p_analog = (T_ANALOG_ENTRY*) p_list->l_ptr;
				for(i = 0; i< p_list->l_size; i++)
				{
					if(p_analog[i].e_key  == code)
					return i+1;
				}
			}
		}
	}
	return 0;
}
/* 103设备服务接口 */
BOOL c103_service(T_IED *p_ied, T_MESSAGE *p_msg)
{
	BOOL  ret_code =FALSE;
	/* [初始化部分] */
	switch(p_msg->m_type)
	{
	case MSG_T_INIT:
		/* 初始化协议 */
		return c103_initialize();
	case MSG_T_OPEN:
		/* 初始化设备 */
		return c103_open_ied((T_IED *)(p_msg->m_data));
	default:
		break;
	}

	/*	[设备操作接口部分] */

	/* 锁定通讯资源 */
	DEV_LOCK_COM();

	/* 映射到内部操作接口 */
	switch(p_msg->m_type)
	{
	case MSG_T_LIST:
		if(p_msg->m_flag == MSG_F_READ)
		{
			ret_code = c103_get_List(p_ied, (T_LIST*) p_msg->m_data);
		}
		break;
	case MSG_T_SET:
		if (p_msg->m_flag == MSG_F_READ)
		{
			ret_code = c103_get_setting(p_ied, (T_SET *)p_msg->m_data);
		}
		
		if (p_msg->m_flag == MSG_F_CHECK)
		{
			ret_code = c103_chk_setting(p_ied, (T_SET *)p_msg->m_data);
		}
		if (p_msg->m_flag == MSG_F_WRITE)
		{
			ret_code = c103_set_setting(p_ied, (T_SET *)p_msg->m_data);
		}
		break;
	case MSG_T_ZONE:
		if (p_msg->m_flag == MSG_F_READ)
		{
			ret_code = c103_get_zone(p_ied, (WORD *)p_msg->m_data);
		}
		if (p_msg->m_flag == MSG_F_CHECK)
		{
			ret_code = c103_chk_zone(p_ied, (WORD)p_msg->m_data);
		}
		if (p_msg->m_flag == MSG_F_WRITE)
		{
			ret_code = c103_set_zone(p_ied, (WORD)p_msg->m_data);
		}
		break;
	case MSG_T_SFC:
		if (p_msg->m_flag == MSG_F_READ)
		{
			ret_code = c103_get_sfc(p_ied);
		}
		if (p_msg->m_flag == MSG_F_CHECK)
		{
			ret_code = c103_chk_sfc(p_ied, LOWORD(p_msg->m_data), HIWORD(p_msg->m_data));
		}
		if (p_msg->m_flag == MSG_F_WRITE)
		{
			ret_code = c103_set_sfc(p_ied, LOWORD(p_msg->m_data), HIWORD(p_msg->m_data));
		}
		break;
	case MSG_T_CTRL:
		if (p_msg->m_flag == MSG_F_READ)
		{
			ret_code = true;
		}
		if (p_msg->m_flag == MSG_F_CHECK)
		{
			ret_code = c103_check_control(p_ied, (WORD) p_msg->m_data);
		}
		if (p_msg->m_flag == MSG_F_WRITE)
		{
			ret_code = c103_remote_control(p_ied, (WORD) p_msg->m_data);
		}
		break;
	case MSG_T_SIGNAL:
		/* 信号复归 */
		if (p_msg->m_flag == MSG_F_WRITE)
		{
			ret_code = c103_reset_signal(p_ied);
		}
		break;
	case MSG_T_ANALOG:
		if(p_msg->m_flag == MSG_F_READ)
		{
			ret_code = c103_get_analog(p_ied, (T_CHANNEL *)p_msg->m_data);
		}
		break;
	case MSG_T_CLOCK:
		/* 广播对时 */
		if (p_msg->m_flag == MSG_F_WRITE)
		{
			ret_code = c103_set_clock(p_ied, (T_DATE *)p_msg->m_data);
		}
		break;
	default:
		break;
	}
	/* 开放通讯资源 */
	DEV_UNLOCK_COM();

	/* 返回操作结果 */
	return ret_code;
}

/* 复位通信单元 */
BOOL c103_reset_cu(T_IED* p_ied)
{
	T_C103_FRAME t_frame;

	t_frame.type =FRAME_T_CON;
	t_frame.contral =0x40;			//复位通信单元
	t_frame.address = (BYTE)p_ied->dev_id;
	t_frame.endbyte =0x16;

	m_c103_fcb =0x20;		//使帧计数位常为0

	if ( c103_serial_com(p_ied, &t_frame, 1) )
	{
		if ( (t_frame.contral &0x0F) == 0 )	//确认响应
		{

			if ( (t_frame.contral &0x20) != 0)		//判别响应帧
			{
				return c103_request_first(p_ied);
			}
			else
			{
				return c103_request_second(p_ied);	//请求二级用户数据
			}
		}
	}

	return false;
}

/* 复位帧计数位 */
BOOL c103_reset_fcb(T_IED* p_ied)
{
	T_C103_FRAME t_frame;

	t_frame.type =FRAME_T_CON;
	t_frame.contral =0x47;			//复位帧计数位
	t_frame.address = (BYTE)p_ied->dev_id;
	t_frame.endbyte =0x16;

	m_c103_fcb =0x20;		//使帧计数位常为0

	if (c103_serial_com(p_ied, &t_frame, 1))
	{
		if ( (t_frame.contral &0x0F) == 0 )	//确认响应
		{

			if ( (t_frame.contral &0x20) != 0)		//判别响应帧
			{
				return c103_request_first(p_ied);
			}
			else
			{
				return c103_request_second(p_ied);	//请求二级用户数据
			}
		}
	}

	return false;
}

/* 总查询 */
BOOL c103_request_gi(T_IED* p_ied)
{
	T_C103_FRAME t_frame;
	T_C103_ASDU* p_asdu;

	p_asdu =(T_C103_ASDU *)&m_tx_buf[6];
	p_asdu->type =7;
	p_asdu->vsq =0x81;
	p_asdu->cot =9;
	p_asdu->addr = (BYTE)p_ied->dev_id;
	p_asdu->fun =C103_FUN_GLB;
	p_asdu->inf =0;
	p_asdu->data[0] =m_c103_sn++;

	t_frame.type =FRAME_T_VOL;
	t_frame.len =9;
	t_frame.contral =0x53;	//发送/确认
	t_frame.address = (BYTE)p_ied->dev_id;
	t_frame.endbyte =0x16;

	if (c103_serial_com(p_ied, &t_frame, 1))
	{
		if ( (t_frame.contral &0x0F) == 0 )	//确认响应
		{

			if ( (t_frame.contral &0x20) != 0)		//判别响应帧
			{
				return c103_request_first(p_ied);
			}
			else
			{
				return c103_request_second(p_ied);	//请求二级用户数据
			}
		}
	}

	return false;
}
 
/* 上电初始化 */
BOOL c103_device_initialize(T_IED* p_ied)
{	
	/* 复位通信单元 */
	if ( !c103_reset_cu(p_ied) )
	{
		return false;
	}

	/* 复位帧计数位 */
	if ( !c103_reset_fcb(p_ied) )
	{
		return false;
	}

	/* 总查询 */
	if ( !c103_request_gi(p_ied) )
	{
		return false;
	}
	return true;
}

/* module's task */
VOID task_c103(UNSIGNED argc, VOID *argv)
{	
	T_IED *p_ied;

	//上电初始化开入
	p_ied = DEV_First_IED();
	while (p_ied != 0)
	{
		if(	p_ied->dev_if == c103_service)	
		{
			DEV_LOCK_COM();

			if ( c103_device_initialize(p_ied) )
			{
				p_ied->dev_flag |= DEV_F_ONLINE;	//连接正常
			}
			else
			{
				p_ied->dev_flag &=~DEV_F_ONLINE;	//连接异常
			}

			DEV_UNLOCK_COM();
		}
		p_ied =DEV_Next_IED(p_ied->dev_id);
	}
loop:	
	p_ied =DEV_First_IED();
	while (p_ied != 0)
	{
		if(	p_ied->dev_if == c103_service )	
		{
			DEV_LOCK_COM();
		
			if ( c103_request_second(p_ied) /*&& c103_get_sfc(p_ied)*/ ) //请求二级用户数据,召唤软压板
			{
				p_ied->dev_flag |= DEV_F_ONLINE;	//连接正常
			}
			else
			{
				p_ied->dev_flag &=~DEV_F_ONLINE;	//连接异常
			}

			DEV_UNLOCK_COM();
		}
		p_ied =DEV_Next_IED(p_ied->dev_id);
	}
	NU_Sleep(polling_delay_time);

	goto loop;
}
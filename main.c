/*
///////////////////////////////////////////
本文用于芯力波通自研KJ1012-K矿用本安型定位卡
本文仅适用于KJ1012-K矿用本安型定位卡
文件编写者：杨照然
文件编写日期：2019年4月13号
文件版本：v2.0
版权所有：郑州芯力波通信息技术有限公司

版本修改：
v1.0 定时加随机延时广播编号，低电量报警，主动求救，一键升级功能
v2.0 添加收发功能，能够被动呼叫
//////////////////////////////////////////
*/
#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <stdio.h>
#include "radio_test.h"
#include "boards.h"
#include "nrf_drv_timer.h"
#include "nordic_common.h"
#include "nrf_error.h"
#include "app_error.h"
#include "app_timer.h"
#include "nrf_delay.h"
#include "nrf_drv_uart.h"
#include "nrf_uarte.h"
#include "nrf_gpio.h"

//#include "SEGGER_RTT.h"
//#include "nrf_log.h"
//#include "nrf_log_ctrl.h"
//#include "nrf_log_default_backends.h"


//区间接收器的参数配置
#define MAX_TADA_NUM     					(100U)								//定义一个周期内最大的接入定位卡数量
#define Uart_data_num    					(350U)  							//串口传输最大的字符串,设计100个终端数
#define Sleep_enter								0x01									//定义定位卡休眠的命令
#define Passive_callvalue         0x08                  //定义定位卡被动呼叫的命令
#define M_TIMER_INTERVAL    			APP_TIMER_TICKS(15000)   //周期发送数据的间隔，单位：ms
#define M_TIMEOUT_UART_RECEIVE   	APP_TIMER_TICKS(9000)  	 //定义超时重发时间，单位：ms
#define	M_TIMEOUT_CAN_UART				APP_TIMER_TICKS(3000)		 //定义距离上一次数据发送的间隔大于3s
#define M_DELAY_INQUIRE						APP_TIMER_TICKS(3000)
#define List_Array_Num		        (64U)                 //定义数组链表的数组大小
#define Uart_TX_Pin								8
#define Uart_RX_Pin								6

//#define Uart_TX_Pin								RX_PIN_NUMBER
//#define Uart_RX_Pin								TX_PIN_NUMBER

#define Uart_BAUDRATE             NRF_UART_BAUDRATE_9600 //定义串口波特率9600
#define UartR_Position_Type				      0x11                	//定义定位数据接收回应类型
#define UartR_Passive_Type							0x02
#define UartR_Open_Passive_Type					0x06
#define UartR_Close_Passive_Type				0x07
		
#define Cancel_Passive_Card						 0x10                   //测试无线取消定位卡的被动响铃功能
//定义main.c文件中的结构体和全局变量
APP_TIMER_DEF(m_receiver);                               //定义周期发送定位卡数据的定时器
APP_TIMER_DEF(m_OT_Send); 															 //定义超时重传定时器	
APP_TIMER_DEF(m_Serial_Transmit);												 //定义超时清除禁止串口发送标志位
APP_TIMER_DEF(m_Delay_Inquire);													 //若串口不可以发送信息，则进行延时查询

static volatile uint16_t Link_list_present_num=0;  			 //链表中保存的定位卡个数
static volatile uint16_t Traver_num=0;   						     //遍历链表采集的数据个数
static volatile uint16_t m_Present_uart;                 //需要串口发送的字节数
static volatile uint16_t m_PassiveCall_Card;						 //单呼叫的定位卡编号
static volatile uint16_t m_Paging_num=0x00;									 //寻呼序号
static  radio_packet_t    m_receive_packet;               //定义的radio数据结构
static  radio_packet_t    m_send_packet={
																					.Card_stat=0x00
};									 
static bool is_true=true;
static bool is_false=false;
static nrf_drv_uart_t Uart_Inst=NRF_DRV_UART_INSTANCE(0); //定义串口服务
static    uint8_t Uart_data[Uart_data_num]={0}; 						//定义串口传输的数组
static    uint8_t Uart_data_receive[8]={0};                 //接收到的串口回复信息 
static  volatile bool Is_Stop_Uart=false;                  //当没有接收到任何定位卡信息时，停止接收信息标志位
static  volatile bool Is_Port_Transmit=true;               //表示是否可以进行串口发送
static    uint8_t Uart_Passive_Reply_date[4]={0x55,0x22,0x00,0xff};      //回复被动呼叫信息,第一次默认校验成功

typedef enum {
		All_Passive_state,																		//优先级最高
		Single_Passive_state,
		No_Passive_state
}Internal_state;
static volatile Internal_state  m_Internal_state=No_Passive_state;        //初始化当前状态，定义状态切换的优先级
typedef struct Ne_Dimon  																	//保存信息的链表结构体
{
    uint8_t     Card_State;  															//当前状态
	//	uint8_t     Frequency;
    uint16_t    Card_ID;
    struct Ne_Dimon *next;
}Ne_Dimon,*pNe_Dimon;
static pNe_Dimon pHead[List_Array_Num];   								//定义链表数组
struct Information{                                       //定义定位卡信息结构体
		uint8_t State;																				//定位卡状态
		uint8_t Identification_1;															//定位卡编号的高八位
		uint8_t Identification_2;															//定位卡编号的低八位
}Inform_Array[MAX_TADA_NUM];


static void Clock_init(void)															//时钟初始化
{
		NRF_CLOCK->EVENTS_HFCLKSTARTED=0;                      //开启32M时钟
		NRF_CLOCK->TASKS_HFCLKSTART=1;
	  while(NRF_CLOCK->EVENTS_HFCLKSTARTED==0)
		{
		}
		NRF_CLOCK->LFCLKSRC=(CLOCK_LFCLKSRC_SRC_Xtal<<CLOCK_LFCLKSRC_SRC_Pos);  //开启外部32.768时钟
		NRF_CLOCK->EVENTS_LFCLKSTARTED =0;
		NRF_CLOCK->TASKS_LFCLKSTART     =1;
		while(NRF_CLOCK->EVENTS_LFCLKSTARTED==0)
		{		
		}
}

static void Init_Node(pNe_Dimon *node_t,uint8_t card_state,uint16_t card_id)   //初始化链表节点
{
    (*node_t)->Card_State=card_state;
	//	(*node_t)->Frequency=0;
    (*node_t)->Card_ID=card_id;
    (*node_t)->next=NULL;
}

void Conserve_Unique(pNe_Dimon *node_t,const radio_packet_t *packet)   					//去除重复数据
{
		Ne_Dimon *preNode=NULL;
    Ne_Dimon *pNode=NULL;
    Ne_Dimon *New_Node=NULL;
		uint16_t temp_id     =packet->ID;
		uint8_t  temp_state  =packet->Card_stat;
    for(pNode=*node_t;pNode;preNode=pNode,pNode=pNode->next)
		{
			if(temp_id==pNode->Card_ID)
        {
				//		pNode->Frequency+=1;                                                //扫描出现的次数自加一
						if(temp_state==pNode->Card_State)
						{
							return;
						}else{
							 pNode->Card_State|=temp_state;                                   //更新定位卡最后的状态
							return;
						}
				}
				 __DSB();
		}
//		 if(pNode==NULL&&Link_list_present_num<MAX_TADA_NUM)
		 if(pNode==NULL)
      {
				  if(Link_list_present_num>=MAX_TADA_NUM)                               //限制数组链表大小
							return;
						New_Node=(Ne_Dimon*)malloc(sizeof(Ne_Dimon));
						Init_Node(&New_Node,temp_state,temp_id);
						if(preNode==NULL)
            {
                New_Node->next=*node_t;
                *node_t=New_Node;
            }else{
                New_Node->next=preNode->next;
                preNode->next=New_Node;
            }
						Link_list_present_num++;
			}
}
/*  遍历链表结构*/
static void Traver_List(pNe_Dimon *Head_t)
{
		 pNe_Dimon *pIter=Head_t;
     pNe_Dimon pWork=NULL;
    while((*pIter)!=NULL)
    {
			//	if((*pIter)->Frequency>0){                                              //将链表信息保存到数组中
				Inform_Array[Traver_num].Identification_1=((*pIter)->Card_ID)>>8;
				Inform_Array[Traver_num].Identification_2=((*pIter)->Card_ID);
				Inform_Array[Traver_num].State=((*pIter)->Card_State);
				Traver_num++;
			//	}
        pWork=*pIter;
        *pIter=(*pIter)->next;
        free(pWork);
        pWork=NULL;
			  __DSB();
    }
}
static void Traver_List_Array()
{
			for(uint8_t i=0;i<List_Array_Num;i++)
				{
//						if(pHead[i]==NULL)
//								continue;
						Traver_List(&(pHead[i]));
				}
}
void radio_evt_handler(radio_evt_t * evt)																				//radio处理函数
{
			if(evt->type==PACKET_RECEIVED){                                            //接收到信息
				m_receive_packet=evt->packet;
				if((m_receive_packet.Crc)==((m_receive_packet.Random)^(m_receive_packet.Card_stat)^( \
																			(uint8_t)m_receive_packet.ID)^(m_receive_packet.ID>>8)))         //对接收信息进行字节校验
				{
						if(m_receive_packet.Random==0||m_receive_packet.Random==1){         //进行信息的验证

							radio_frequency_set(0);                                          //更改频率到2360MHz
							switch(m_Internal_state){
								case No_Passive_state:{
										m_send_packet.Random=Cancel_Passive_Card;
										m_send_packet.ID=m_receive_packet.ID;
//									m_send_packet.Random=Sleep_enter;                            										 //更改回复信息
//									m_send_packet.ID=m_receive_packet.ID;
//									m_send_packet.Crc=Sleep_enter^(m_send_packet.Card_stat)^( \
//																								(uint8_t)m_receive_packet.ID)^(m_receive_packet.ID>>8);//重新计算校验
								}break;
								case All_Passive_state:{							
									m_send_packet.Random=Passive_callvalue;                            										 //更改回复信息
									m_send_packet.ID=m_receive_packet.ID;
									m_send_packet.Card_stat=(uint8_t)m_Paging_num;
									m_send_packet.Crc=m_Paging_num>>8;	 
								}break;
								case Single_Passive_state:{
									if(m_receive_packet.ID==m_PassiveCall_Card){                                           //匹配定位卡编号
									m_send_packet.Random=Passive_callvalue;                            										 //更改回复信息
									m_send_packet.ID=m_receive_packet.ID;
									m_send_packet.Card_stat=(uint8_t)m_Paging_num;
									m_send_packet.Crc=m_Paging_num>>8;		
									}else{                                                                                //匹配失败则发送休眠信息
									m_send_packet.Random=Cancel_Passive_Card;
									m_send_packet.ID=m_receive_packet.ID;
//									m_send_packet.Random=Sleep_enter;                            												 //更改回复信息
//									m_send_packet.ID=m_receive_packet.ID;
//									m_send_packet.Crc=Sleep_enter^(m_send_packet.Card_stat)^( \
//																								(uint8_t)m_receive_packet.ID)^(m_receive_packet.ID>>8);//重新计算校验
									}
								}break;
								default:
									break;
							}
							radio_send(&m_send_packet);
							return ;
						}		
				}				
			}else if(evt->type==PACKET_SENT){
							Conserve_Unique(&(pHead[(m_receive_packet.ID)&0x3f]),&m_receive_packet);	
			}
			radio_frequency_set(20);                                                //更改频率到2380MHz
			radio_receive();																									 //进入接收状态
}
static uint8_t Crc8_Compute(uint8_t const *p_data,uint16_t size)               //计算字节校验值
{
		uint8_t crc=0x00;
		for(uint16_t i=1;i<size;i++)
		{
				crc^=*(p_data+i);
		}
		return crc;
}
static bool CRC8_Command_Frame()                                 //对全局数组Uart_data_receive做校验
{
		uint8_t temp_crc=0x00;
		for(uint8_t i=1;i<7;i++)
			temp_crc^=(Uart_data_receive[i]);
		if(temp_crc==Uart_data_receive[7])
				return true;                                             //校验通过
		else	
				return false;
		
}
static void Timeout_handler_transmit(void *p_context)                                  //周期通过串口发送扫描到的定位卡信息
{
     if(Link_list_present_num==0)
		 {
				Is_Stop_Uart=true;
				return;
		 }else{      
			 Is_Stop_Uart=false;
			 CRITICAL_REGION_ENTER();
			 Traver_num=0;
			 Traver_List_Array();  																										//遍历并删除链表
			  Uart_data[0]=0x55;
				Uart_data[1]=0x01;
				Uart_data[2]=Traver_num;
			 memcpy(Uart_data+3,Inform_Array,Traver_num*sizeof(struct Information));
			 m_Present_uart=3+Traver_num*3;
			 Uart_data[m_Present_uart]=Crc8_Compute(Uart_data,m_Present_uart);				//添加一字节的crc校验
			 m_Present_uart++;
			 memset(Inform_Array,0,MAX_TADA_NUM*sizeof(struct Information));
			 Link_list_present_num=0;
			 
			 if(Is_Port_Transmit)
			   nrf_drv_uart_tx(&Uart_Inst,Uart_data,m_Present_uart);
			 else{
				app_timer_stop(m_Delay_Inquire);
				 app_timer_start(m_Delay_Inquire,M_DELAY_INQUIRE,&is_true);
			 }
		   CRITICAL_REGION_EXIT();
			 app_timer_start(m_OT_Send,M_TIMEOUT_UART_RECEIVE,&is_true);             //开启超时定时器
	}
}
static void Timeout_handler_receive(void *p_context)  																  //串口接收数据处理函数
{
		bool m_temp= *(bool *)p_context;                                            //根据传进来的参数值进入不同的服务
	  if(m_temp)                                                                  //超时重传操作
		{
				if(Is_Port_Transmit){
					 nrf_drv_uart_tx(&Uart_Inst,Uart_data,m_Present_uart);
				}else{
					 app_timer_stop(m_Delay_Inquire);
					 app_timer_start(m_Delay_Inquire,M_DELAY_INQUIRE,&is_true);  //重新延时查询
				}
		}else{                                                                      //对串口收到的信息做处理
				CRITICAL_REGION_ENTER();
				//if((0x56==Uart_data_receive[0])&&(CRC8_Command_Frame())){                //校验接收数组的第一个数
				if((0x56==Uart_data_receive[0])){                //校验接收数组的第一个数，方便测试，没有对接收的数据进行CRC校验
						if(UartR_Position_Type==Uart_data_receive[1]){  																	                                    
											app_timer_stop(m_OT_Send);                                //如果收到的信息正确，则取消超时重传操作，否侧等待一段时间后再次重传一次
									memset(Uart_data_receive,0,sizeof(Uart_data_receive));                  //清空接收数组内容
									nrf_drv_uart_rx_abort(&Uart_Inst);
									nrf_drv_uart_rx(&Uart_Inst,Uart_data_receive,sizeof(Uart_data_receive));//等待串口信息服务，即等待其它串口发来的指令
						}else if(UartR_Passive_Type==Uart_data_receive[1]) {                 //接收控制命令
							    m_PassiveCall_Card=Uart_data_receive[2]<<8|Uart_data_receive[3];
									m_Paging_num=Uart_data_receive[5]<<8|Uart_data_receive[6];
							    if(UartR_Open_Passive_Type==Uart_data_receive[4]){
										    if(m_PassiveCall_Card){
													m_Internal_state=(m_Internal_state==All_Passive_state)?All_Passive_state:Single_Passive_state;
												}else{
													 m_Internal_state=All_Passive_state;
												}
									}else if(UartR_Close_Passive_Type==Uart_data_receive[4]){
												if(m_PassiveCall_Card){
													m_Internal_state=(m_Internal_state==All_Passive_state)?All_Passive_state:No_Passive_state;
												}else{
													 m_Internal_state=No_Passive_state;
												}
									}else{
											 CRITICAL_REGION_EXIT();
											 return ;
									}
									Uart_Passive_Reply_date[2]=Uart_data_receive[4];
									Uart_Passive_Reply_date[3]=0x00^Uart_Passive_Reply_date[1]^Uart_Passive_Reply_date[2];      //一字节校验值
									if(Is_Port_Transmit){
											 nrf_drv_uart_tx(&Uart_Inst,Uart_Passive_Reply_date,4);        //回复信息
									}else{
											 app_timer_stop(m_Delay_Inquire);
											 app_timer_start(m_Delay_Inquire,M_DELAY_INQUIRE,&is_false);  //重新延时查询
									}
						}
					}
				CRITICAL_REGION_EXIT();
		}
}
static void Timeout_handler_restore_uart(void *context)
{
		Is_Port_Transmit=true;
}
static void Timeout_handler_inquire(void *context)
{
		bool m_temp= *(bool *)context;
		if(m_temp){
			  if(Is_Port_Transmit){
					 nrf_drv_uart_tx(&Uart_Inst,Uart_data,m_Present_uart);
				}else{
					 app_timer_stop(m_Delay_Inquire);
					 app_timer_start(m_Delay_Inquire,M_DELAY_INQUIRE,&is_true);  //重新延时查询
				}
		}else{
				if(Is_Port_Transmit){
					 nrf_drv_uart_tx(&Uart_Inst,Uart_Passive_Reply_date,4);        //回复信息
				}else{
					 app_timer_stop(m_Delay_Inquire);
					 app_timer_start(m_Delay_Inquire,M_DELAY_INQUIRE,&is_true);  //重新延时查询
				}
		}
	
}
static void Times_init()                                                        			//初始化RTC定时器
{
		app_timer_init();
	  app_timer_create(&m_receiver,APP_TIMER_MODE_REPEATED,Timeout_handler_transmit);   //循环定时器
		app_timer_create(&m_OT_Send,APP_TIMER_MODE_SINGLE_SHOT,Timeout_handler_receive);  //只能生效一次
		app_timer_create(&m_Serial_Transmit,APP_TIMER_MODE_SINGLE_SHOT,Timeout_handler_restore_uart);
		app_timer_create(&m_Delay_Inquire,APP_TIMER_MODE_SINGLE_SHOT,Timeout_handler_inquire);
}
static void APP_Start()
{
		app_timer_start(m_receiver,M_TIMER_INTERVAL,NULL);
}
void uart_event_handler(nrf_drv_uart_event_t *p_event,void* p_context)           //串口中断处理函数
{
	  switch(p_event->type)
		{
			 case NRF_DRV_UART_EVT_TX_DONE:
			 {
					Is_Port_Transmit=false;
				  app_timer_start(m_Serial_Transmit,M_TIMEOUT_CAN_UART,NULL);
					nrf_drv_uart_rx_abort(&Uart_Inst);
					nrf_drv_uart_rx(&Uart_Inst,Uart_data_receive,sizeof(Uart_data_receive));//等待串口信息服务
			 }break;
			 case NRF_DRV_UART_EVT_RX_DONE:
			 {
					Timeout_handler_receive(&is_false);																			//处理接收到的信息
			 }break;
			 default:
				 break;
		}
}
static void Uart_init(void)																												//串口初始化
{
		nrf_drv_uart_config_t config_uart=NRF_DRV_UART_DEFAULT_CONFIG;
	  config_uart.baudrate = Uart_BAUDRATE;
		config_uart.hwfc = NRF_UART_HWFC_DISABLED;
    config_uart.interrupt_priority = 0;
    config_uart.parity = NRF_UART_PARITY_EXCLUDED;
    config_uart.pselrxd = Uart_RX_Pin;
    config_uart.pseltxd = Uart_TX_Pin ;
		nrf_drv_uart_init(&Uart_Inst, &config_uart,  uart_event_handler);
}
//static void log_init()
//{
//		uint32_t err_code;
//    err_code = NRF_LOG_INIT(NULL);
//    APP_ERROR_CHECK(err_code);
//    NRF_LOG_DEFAULT_BACKENDS_INIT();
//}
int main()
{
		Clock_init();
//		log_init();
//		NRF_LOG_INFO("-----------");
//		NRF_LOG_FLUSH();
		radio_init(&radio_evt_handler);
		Uart_init();
		Times_init();
		APP_Start();
		radio_receive();
		while(1)
		{
				__WFI();
		}
		
}


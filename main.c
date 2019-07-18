/*
///////////////////////////////////////////
��������о����ͨ����KJ1012-K���ñ����Ͷ�λ��
���Ľ�������KJ1012-K���ñ����Ͷ�λ��
�ļ���д�ߣ�����Ȼ
�ļ���д���ڣ�2019��4��13��
�ļ��汾��v2.0
��Ȩ���У�֣��о����ͨ��Ϣ�������޹�˾

�汾�޸ģ�
v1.0 ��ʱ�������ʱ�㲥��ţ��͵���������������ȣ�һ����������
v2.0 ����շ����ܣ��ܹ���������
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


//����������Ĳ�������
#define MAX_TADA_NUM     					(100U)								//����һ�����������Ľ��붨λ������
#define Uart_data_num    					(350U)  							//���ڴ��������ַ���,���100���ն���
#define Sleep_enter								0x01									//���嶨λ�����ߵ�����
#define Passive_callvalue         0x08                  //���嶨λ���������е�����
#define M_TIMER_INTERVAL    			APP_TIMER_TICKS(15000)   //���ڷ������ݵļ������λ��ms
#define M_TIMEOUT_UART_RECEIVE   	APP_TIMER_TICKS(9000)  	 //���峬ʱ�ط�ʱ�䣬��λ��ms

#define List_Array_Num		        (64U)                 //������������������С
#define Uart_TX_Pin								8
#define Uart_RX_Pin								6

//#define Uart_TX_Pin								RX_PIN_NUMBER
//#define Uart_RX_Pin								TX_PIN_NUMBER

#define Uart_BAUDRATE             NRF_UART_BAUDRATE_9600 //���崮�ڲ�����9600
#define UartR_Position_Type				      0x11                	//���嶨λ���ݽ��ջ�Ӧ����
#define UartR_Passive_Type							0x02
#define UartR_Open_Passive_Type					0x06
#define UartR_Close_Passive_Type				0x07
		
#define Cancel_Passive_Card						 0x10                   //��������ȡ����λ���ı������幦��
//����main.c�ļ��еĽṹ���ȫ�ֱ���
APP_TIMER_DEF(m_receiver);                               //�������ڷ��Ͷ�λ�����ݵĶ�ʱ��
APP_TIMER_DEF(m_OT_Send); 															 //���峬ʱ�ش���ʱ��	

static volatile uint16_t Link_list_present_num=0;  			 //�����б���Ķ�λ������
static volatile uint16_t Traver_num=0;   						     //��������ɼ������ݸ���
static volatile uint16_t m_Present_uart;                 //��Ҫ���ڷ��͵��ֽ���


static  radio_packet_t    m_receive_packet;               //�����radio���ݽṹ
static  radio_packet_t    m_send_packet={
																					.Card_stat=0x00
};									 
static bool is_true=true;
static bool is_false=false;
static nrf_drv_uart_t Uart_Inst=NRF_DRV_UART_INSTANCE(0); //���崮�ڷ���
static    uint8_t Uart_data[Uart_data_num]={0}; 						//���崮�ڴ��������

static  uint8_t Contrast_uart_receive[6]={0x55,0x11,0xff,0x00,0x0d,0x0a};   //0xff����ɱ�����
static  volatile bool Is_Stop_Uart=false;                  //��û�н��յ��κζ�λ����Ϣʱ��ֹͣ������Ϣ��־λ
static  uint8_t Uart_data_receive[6]={0};

typedef struct Ne_Dimon  																	//������Ϣ������ṹ��
{
    uint8_t     Card_State;  															//��ǰ״̬
	//	uint8_t     Frequency;
    uint16_t    Card_ID;
    struct Ne_Dimon *next;
}Ne_Dimon,*pNe_Dimon;
static pNe_Dimon pHead[List_Array_Num];   								//������������
struct Information{                                       //���嶨λ����Ϣ�ṹ��
		uint8_t State;																				//��λ��״̬
		uint8_t Identification_1;															//��λ����ŵĸ߰�λ
		uint8_t Identification_2;															//��λ����ŵĵͰ�λ
}Inform_Array[MAX_TADA_NUM];

static void Clock_init(void)															//ʱ�ӳ�ʼ��
{
		NRF_CLOCK->EVENTS_HFCLKSTARTED=0;                      //����32Mʱ��
		NRF_CLOCK->TASKS_HFCLKSTART=1;
	  while(NRF_CLOCK->EVENTS_HFCLKSTARTED==0)
		{
		}
		NRF_CLOCK->LFCLKSRC=(CLOCK_LFCLKSRC_SRC_Xtal<<CLOCK_LFCLKSRC_SRC_Pos);  //�����ⲿ32.768ʱ��
		NRF_CLOCK->EVENTS_LFCLKSTARTED =0;
		NRF_CLOCK->TASKS_LFCLKSTART     =1;
		while(NRF_CLOCK->EVENTS_LFCLKSTARTED==0)
		{		
		}
}

static void Init_Node(pNe_Dimon *node_t,uint8_t card_state,uint16_t card_id)   //��ʼ������ڵ�
{
    (*node_t)->Card_State=card_state;
	//	(*node_t)->Frequency=0;
    (*node_t)->Card_ID=card_id;
    (*node_t)->next=NULL;
}

void Conserve_Unique(pNe_Dimon *node_t,const radio_packet_t *packet)   					//ȥ���ظ�����
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
				//		pNode->Frequency+=1;                                                //ɨ����ֵĴ����Լ�һ
						if(temp_state==pNode->Card_State)
						{
							return;
						}else{
							 pNode->Card_State|=temp_state;                                   //���¶�λ������״̬
							return;
						}
				}
				 __DSB();
		}
//		 if(pNode==NULL&&Link_list_present_num<MAX_TADA_NUM)
		 if(pNode==NULL)
      {
				    if(Link_list_present_num>=MAX_TADA_NUM)                               //�������������С
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
/*  ��������ṹ*/
static void Traver_List(pNe_Dimon *Head_t)
{
		 pNe_Dimon *pIter=Head_t;
     pNe_Dimon pWork=NULL;
    while((*pIter)!=NULL)
    {
			//	if((*pIter)->Frequency>0){                                              //��������Ϣ���浽������
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
void radio_evt_handler(radio_evt_t * evt)																				//radio������
{
			if(evt->type==PACKET_RECEIVED){                                            //���յ���Ϣ
				m_receive_packet=evt->packet;
				if((m_receive_packet.Crc)==((m_receive_packet.Random)^(m_receive_packet.Card_stat)^( \
																			(uint8_t)m_receive_packet.ID)^(m_receive_packet.ID>>8)))         //�Խ�����Ϣ�����ֽ�У��
				{
						if(m_receive_packet.Random==0||m_receive_packet.Random==1){         //������Ϣ����֤

							radio_frequency_set(0);                                          //����Ƶ�ʵ�2360MHz
							
										m_send_packet.Random=Sleep_enter;
										m_send_packet.ID=m_receive_packet.ID;
							radio_send(&m_send_packet);
							return ;
						}		
				}				
			}else if(evt->type==PACKET_SENT){
							Conserve_Unique(&(pHead[(m_receive_packet.ID)&0x3f]),&m_receive_packet);	
			}
			radio_frequency_set(20);                                                //����Ƶ�ʵ�2380MHz
			radio_receive();																									 //�������״̬
}
static uint8_t Crc8_Compute(uint8_t const *p_data,uint16_t size)               //�����ֽ�У��ֵ
{
		uint8_t crc=0x00;
		for(uint16_t i=0;i<size;i++)
		{
				crc^=*(p_data+i);
		}
		return crc;
}

static void Timeout_handler_transmit(void *p_context)                                  //����ͨ�����ڷ���ɨ�赽�Ķ�λ����Ϣ
{
     if(Link_list_present_num==0)
		 {
				Is_Stop_Uart=true;
				return;
		 }else{      
			 Is_Stop_Uart=false;
			 CRITICAL_REGION_ENTER();
			 Traver_num=0;
			 Traver_List_Array();  																										//������ɾ������
			  Uart_data[0]=0x55;
				Uart_data[1]=0x01;
				Uart_data[2]=Traver_num;
			 Contrast_uart_receive[2]=Traver_num;
			 memcpy(Uart_data+3,Inform_Array,Traver_num*sizeof(struct Information));
			 m_Present_uart=3+Traver_num*3;
			 Uart_data[m_Present_uart]=Crc8_Compute(Uart_data,m_Present_uart);				//���һ�ֽڵ�crcУ��
			 Uart_data[++m_Present_uart]=0x0d;
			 Uart_data[++m_Present_uart]=0x0a;
			 m_Present_uart++;
			 memset(Inform_Array,0,MAX_TADA_NUM*sizeof(struct Information));
			 Link_list_present_num=0;
		   CRITICAL_REGION_EXIT();
			 nrf_drv_uart_tx(&Uart_Inst,Uart_data,m_Present_uart);
	}
}
static void Timeout_handler_receive(void *p_context)  																  //���ڽ������ݴ�����
{
		bool m_temp= *(bool *)p_context;                                            //���ݴ������Ĳ���ֵ���벻ͬ�ķ���
	  if(m_temp)                                                                  //��ʱ�ش�����
		{
			  nrf_drv_uart_tx(&Uart_Inst,Uart_data,m_Present_uart);
		}else{                                                                      //�Դ����յ�����Ϣ������
				CRITICAL_REGION_ENTER();
			  if(strcmp((char*)Uart_data_receive,(char*)Contrast_uart_receive)==0)
					{
						  uint32_t err_code=app_timer_stop(m_OT_Send);
							m_Present_uart=0;
							memset(Uart_data,0,sizeof(Uart_data));
					}
				memset(Uart_data_receive,0,sizeof(Uart_data_receive));
				CRITICAL_REGION_EXIT();
		}
}

static void Times_init()                                                        			//��ʼ��RTC��ʱ��
{
		app_timer_init();
	  app_timer_create(&m_receiver,APP_TIMER_MODE_REPEATED,Timeout_handler_transmit);   //ѭ����ʱ��
		app_timer_create(&m_OT_Send,APP_TIMER_MODE_SINGLE_SHOT,Timeout_handler_receive);  //ֻ����Чһ��
}
static void APP_Start()
{
		app_timer_start(m_receiver,M_TIMER_INTERVAL,NULL);
}
void uart_event_handler(nrf_drv_uart_event_t *p_event,void* p_context)           //�����жϴ�����
{
	  switch(p_event->type)
		{
			 case NRF_DRV_UART_EVT_TX_DONE:
			 {
				  if(Is_Stop_Uart)
								return;
					nrf_drv_uart_rx_abort(&Uart_Inst);
					nrf_drv_uart_rx(&Uart_Inst,Uart_data_receive,sizeof(Uart_data_receive));//�ȴ�������Ϣ����
					app_timer_stop(m_OT_Send);
					app_timer_start(m_OT_Send,M_TIMEOUT_UART_RECEIVE,&is_true);
			 }break;
			 case NRF_DRV_UART_EVT_RX_DONE:
			 {
					Timeout_handler_receive(&is_false);																			//������յ�����Ϣ
			 }break;
			 default:
				 break;
		}
}
static void Uart_init(void)																												//���ڳ�ʼ��
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

int main()
{
		Clock_init();
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


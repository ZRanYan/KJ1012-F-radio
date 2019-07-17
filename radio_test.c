
#include "radio_test.h"
#include <stdint.h>
#include <stdbool.h>
#include "nrf.h"
#include "app_util_platform.h"
#include <string.h>

static uint32_t swap_bits(uint32_t inp)
{
    uint32_t i;
    uint32_t retval = 0;
    inp = (inp & 0x000000FFUL);
    for (i = 0; i < 8; i++)
    {
        retval |= ((inp >> i) & 0x01) << (7 - i);
    }
    return retval;
}
typedef enum{
		IDLE,
		RX_PACKET_RECEIVE,
		TX_PACKET_SEND,
}radio_state_t;
volatile static radio_state_t m_state;
volatile static radio_packet_t m_packet;
static radio_evt_handler_t * m_evt_handler;

#define PREPARE_DISABLE() do \
    { \
        NRF_RADIO->SHORTS = RADIO_SHORTS_READY_START_Enabled << RADIO_SHORTS_READY_START_Pos | \
        RADIO_SHORTS_END_DISABLE_Enabled << RADIO_SHORTS_END_DISABLE_Pos; \
    } while(0)
static uint32_t bytewise_bitswap(uint32_t inp)
{
      return (swap_bits(inp >> 24) << 24)
           | (swap_bits(inp >> 16) << 16)
           | (swap_bits(inp >> 8) << 8)
           | (swap_bits(inp));
}
void RADIO_IRQHandler(void)
{
		radio_evt_t evt;
		if((NRF_RADIO->EVENTS_DISABLED==1)&&(NRF_RADIO->INTENSET&RADIO_INTENSET_DISABLED_Msk))
			{
						NRF_RADIO->EVENTS_DISABLED=0;
						switch(m_state)
						{
							case RX_PACKET_RECEIVE:
							{
  							if(NRF_RADIO->CRCSTATUS==1U)    //crc校验
								{
										evt.type=PACKET_RECEIVED;
										evt.packet=m_packet;
								}else{
										evt.type=PACKET_ERROR;
								}
								m_state=IDLE;                   //接收数据完毕，指示空闲模式
							}break;
							case TX_PACKET_SEND:
							{
									evt.type=PACKET_SENT;
									m_state=IDLE;
							}break;
							default:
								break;
						}
						(*m_evt_handler)(&evt);
			}
}
void radio_init(radio_evt_handler_t * evt_handler) //发射参数设置
{
			m_state=IDLE;
			m_evt_handler=evt_handler;
			PREPARE_DISABLE();
			NRF_RADIO->TXPOWER   = (RADIO_TXPOWER_TXPOWER_Pos4dBm << RADIO_TXPOWER_TXPOWER_Pos);
			NRF_RADIO->FREQUENCY = 20UL<<RADIO_FREQUENCY_FREQUENCY_Pos|RADIO_FREQUENCY_MAP_Low<<RADIO_FREQUENCY_MAP_Pos;  
			NRF_RADIO->MODE      =(RADIO_MODE_MODE_Ble_1Mbit<< RADIO_MODE_MODE_Pos);  //
		  NRF_RADIO->PREFIX0 =((uint32_t)swap_bits(0xC0) << 0); 
			NRF_RADIO->BASE0 = bytewise_bitswap(0x98854567UL);  

			NRF_RADIO->RXADDRESSES = 0x01UL;   //选择地址4
			NRF_RADIO->TXADDRESS=0x04UL;    //选择发射地址
	    NRF_RADIO->PCNF0 = (PACKET_S1_FIELD_SIZE     << RADIO_PCNF0_S1LEN_Pos) |  //
                         (PACKET_S0_FIELD_SIZE     << RADIO_PCNF0_S0LEN_Pos) |  //
                         (PACKET_LENGTH_FIELD_SIZE << RADIO_PCNF0_LFLEN_Pos)|   //
												 (RADIO_PCNF0_PLEN_16bit   << RADIO_PCNF0_PLEN_Pos);     //16bit前缀		
			NRF_RADIO->PCNF1=(RADIO_PCNF1_WHITEEN_Enabled << RADIO_PCNF1_WHITEEN_Pos)|
												(RADIO_PCNF1_ENDIAN_Little       << RADIO_PCNF1_ENDIAN_Pos)|
												(4UL        << RADIO_PCNF1_BALEN_Pos)|             //5 bytes total address
												(RADIO_PACKET_LENGTH << RADIO_PCNF1_STATLEN_Pos)|   
												(RADIO_PACKET_LENGTH<< RADIO_PCNF1_MAXLEN_Pos);    
			NRF_RADIO->CRCCNF =(RADIO_CRCCNF_LEN_Three << RADIO_CRCCNF_LEN_Pos);
			if ((NRF_RADIO->CRCCNF & RADIO_CRCCNF_LEN_Msk) == (RADIO_CRCCNF_LEN_Three << RADIO_CRCCNF_LEN_Pos))
			{
					NRF_RADIO->CRCINIT = 0xFFFFUL;   // Initial value
					NRF_RADIO->CRCPOLY = 0xD2DUL;  // CRC poly: x^16 + x^12+x^5 + 1
			}
	  	NRF_RADIO->PACKETPTR=(uint32_t)&m_packet;
			
			NRF_RADIO->INTENSET= RADIO_INTENSET_DISABLED_Set<<RADIO_INTENSET_DISABLED_Pos;			
			NVIC_SetPriority(RADIO_IRQn,1);
		  NVIC_EnableIRQ(RADIO_IRQn);
}
void  radio_send(const radio_packet_t  *packet)
{
			if(m_state==IDLE){
					m_state=TX_PACKET_SEND;
			}else{ 																//如果是其它状态，先停止后开启
					NRF_RADIO->TASKS_STOP   =1U;
						__DSB();
					while(NRF_RADIO->STATE!=0);
					m_state=TX_PACKET_SEND;
			}
			m_packet=*packet;
			NRF_RADIO->EVENTS_READY=0U;
		  NRF_RADIO->TASKS_TXEN  =1;
}
void radio_receive()
{
			if(m_state==IDLE){
					m_state=RX_PACKET_RECEIVE;
			}else{
					NRF_RADIO->TASKS_STOP   =1U;	
					__DSB();
				  while(NRF_RADIO->STATE!=0);
					__DSB();
					m_state=RX_PACKET_RECEIVE;
			}
			NRF_RADIO->EVENTS_READY=0U;
		  NRF_RADIO->TASKS_RXEN  =1;
}

void radio_stop(void)
{
			m_state=IDLE;
			NRF_RADIO->TASKS_STOP   =1U;	
			__DSB();
			while(NRF_RADIO->STATE!=0);
			__DSB();
}
void	radio_frequency_set(uint8_t Fre_value)
{
			NRF_RADIO->FREQUENCY=Fre_value<<RADIO_FREQUENCY_FREQUENCY_Pos|RADIO_FREQUENCY_MAP_Low<<RADIO_FREQUENCY_MAP_Pos;
			__DSB();
}

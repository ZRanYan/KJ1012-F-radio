

#ifndef  _RADIO_TEST_H_
#define  _RADIO_TEST_H_
#include <stdint.h>

#define PACKET_S1_FIELD_SIZE      (8UL)
#define PACKET_S0_FIELD_SIZE      (8UL)
#define PACKET_LENGTH_FIELD_SIZE  (8UL)

#define RADIO_PACKET_LENGTH       (5UL)

typedef enum
{
    PACKET_RECEIVED,
    PACKET_SENT,
    PACKET_ERROR,
} radio_evt_type_t;
typedef struct __attribute__((packed))
{
    uint8_t Crc;                    //命令格式
    uint8_t Random; 								//设备类型
		uint16_t ID;
		uint8_t Card_stat;        			//数据
} radio_packet_t;
typedef struct
{
   radio_evt_type_t type;
   radio_packet_t packet;
} radio_evt_t;

typedef void (radio_evt_handler_t)(radio_evt_t * evt);
void 		 radio_init(radio_evt_handler_t * evt_handler);
void     radio_send(const radio_packet_t  *packet);
void     radio_receive(void);
void		 radio_stop(void);
void		radio_frequency_set(uint8_t Fre_value);

#endif


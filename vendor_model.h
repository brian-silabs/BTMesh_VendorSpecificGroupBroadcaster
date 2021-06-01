#ifndef _VENDOR_MODEL_H_
#define _VENDOR_MODEL_H_

#include <stdint.h>

#define PRIMARY_ELEMENT                 0
#define MY_VENDOR_ID                    0x02FF

#define MY_MODEL_SERVER_ID              0xABCD
#define MY_MODEL_CLIENT_ID              0xABCD

#define CUSTOM_DATA_LENGTH              4

#define NUMBER_OF_OPCODES               2

#define ACK_REQ										(0x1)
#define STATUS_UPDATE_REQ							(0x2)

#define INDEX_OF(x)									((x) - 1)

typedef enum {
	custom_get = 0x1,
	custom_set,
} my_msg_t;

typedef struct {
  uint16_t elem_index;
  uint16_t vendor_id;
  uint16_t model_id;
  uint8_t publish; // publish - 1, not - 0
  uint8_t opcodes_len;
  uint8_t opcodes_data[NUMBER_OF_OPCODES];
} my_model_t;
#endif //_VENDOR_MODEL_H_
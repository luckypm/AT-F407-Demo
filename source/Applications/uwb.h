#ifndef _uwb_h
#define _uwb_h

#include "arm_math.h"
#include "digital.h"



#define UWB_STACK_SIZE	    200
#define UWB_PRIORITY	    10

#define ANCHOR_ID1	2192
#define ANCHOR_ID2	2296
#define ANCHOR_ID3	2246
#define ANCHOR_ID4	2282
#define TAG_ID		3001

////////////////// Pozyx Prams //////////////////////////////

#define NUM_ANCHORS 4

#define MSG_HEADER          0x01
#define MSGID_BEACON_CONFIG 0x02
#define MSGID_BEACON_DIST   0x03
#define MSGID_POSITION      0x04

// structure for messages uploaded to ardupilot
union beacon_config_msg {
    struct {
        uint8_t beacon_id;
        uint8_t beacon_count;
        int32_t x;
        int32_t y;
        int32_t z;
    } __attribute__((packed))info;
    uint8_t buf[14];
};
union beacon_distance_msg {
    struct {
        uint8_t beacon_id;
        uint32_t distance;
    } __attribute__((packed))info;
    uint8_t buf[5];
};
union vehicle_position_msg {
    struct {
        int32_t x;
        int32_t y;
        int32_t z;
        int16_t position_error;
    } __attribute__((packed))info;
    uint8_t buf[14];
};
////////////////////////////////////////////////
/**
* The coordinates type defines the coordinates of position result or anchor location
*/
typedef struct __attribute__((packed))_coordinates {
    /** The x-coordinate in mm */
    int32_t x;
    /** The y-coordinate in mm */
    int32_t y;
    /** The z-coordinate in mm */
    int32_t z;
}coordinates_t;



enum uwbStates {
    UWB_WAIT_HEADER1 = 0,
    UWB_WAIT_HEADER2,
    UWB_WAIT_CNT,
    UWB_WAIT_PAYLOAD,
    UWB_WAIT_TAIL
};


typedef struct {
	digitalPin *rs485TXEnable;		/*控制485收发引脚 */
	arm_matrix_instance_f32 A;      /* Matrix A Instance */
    arm_matrix_instance_f32 AT;     /* Matrix AT(A transpose) instance */
    arm_matrix_instance_f32 ATMA;   /* Matrix ATMA( AT multiply with A) instance */
    arm_matrix_instance_f32 ATMAI;  /* Matrix ATMAI(Inverse of ATMA) instance */
    arm_matrix_instance_f32 B;      /* Matrix B instance */
    arm_matrix_instance_f32 X;      /* Matrix X(Unknown Matrix) instance */
	uint8_t solveflag;
	uint32_t timer[2];
	uint32_t deltaTime;
	unsigned char state;
	char measureCnt[6];
	uint8_t mcnt;
	char payload[64];
	uint8_t pcnt;
	char tail[4];
	uint8_t tcnt;
	float distance[4];
	unsigned long loops;
} uwbStruct_t;

extern uwbStruct_t uwbData;

extern void uwbInit(void);
extern void uwbPostionShow(void);
extern void uwbSendNotice(const char *s);
void send_message(uint8_t msg_id, uint8_t data_len, uint8_t data_buf[]);
void uwbSendPacket(const uint8_t *buf, uint16_t len);
void send_vehicle_position(void);
void send_beacon_distance(uint8_t beacon_id, uint32_t distance_mm);
void send_beacon_config();
void get_ranges();




#endif

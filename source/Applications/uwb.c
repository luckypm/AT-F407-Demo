
#include "uwb.h"
#include "aq.h"
#include "config.h"
#include "comm.h"
#include "supervisor.h"
#include "util.h"
#include "d_imu.h"
#include "lcd.h"
#include <stdlib.h>
#include <string.h>

uwbStruct_t uwbData __attribute__((section(".ccm")));

TaskHandle_t uwbTaskHandle;
arm_status status;

// the network id of the anchors: change these to the network ids of your anchors.
uint16_t anchor_id[4] = { 0x601C, // (0,0)
                          0x6020, // x-axis
                          0x6057, // y-axis
                          0x605E};     
// only required for manual anchor calibration. 
// Please change this to the coordinates measured for the anchors
/*int32_t anchors_x[NUM_ANCHORS] = {0,     1000, 0,     1000};    // anchor x-coorindates in mm (horizontal)
int32_t anchors_y[NUM_ANCHORS] = {0,     0,     1000, 1000};    // anchor y-coordinates in mm (vertical)
int32_t heights[NUM_ANCHORS] =   {-1200, -1200, -1200, -1200}; */   // anchor z-coordinates in mm (1.2m above vehicle's starting altitude)
int32_t anchors_x[NUM_ANCHORS] = {300,	 0, 0,	  0};	// anchor x-coorindates in mm (horizontal)
int32_t anchors_y[NUM_ANCHORS] = {0,	 0, 300, 0};	// anchor y-coordinates in mm (vertical)
int32_t heights[NUM_ANCHORS] =	 {0, 	 0, 0,  -300};


//float32_t A[9] = {1,2,3,2,4,5,3,5,6};//错,顺序主子式为0，也不能求解
//float32_t A[9] = {1,2,3,2,2,1,3,4,3};//对,注意不能加const，加了也会计算错误
//float32_t A[9] = {5,0,0,0,3,1,0,2,1};//对
//float32_t A[9] = { 0, -2,  1,
//				   3,  0, -2,
//				   -2, 3,  0};//错
//float32_t A[9] = {2,1,-3,1,2,-2,-1,3,2};//对
//float32_t AI[9];
float p1[3] = {30,0,0};
float p2[3] = {0,0,0};
float p3[3] = {0,30,0};
float p4[3] = {0,0,30};
//float R[4] = {30,42.426,30,51.962};
//float R[4] = {42.426,51.962,42.426,42.426};
//float R[4] = {30,42.426,51.962,30};




float32_t B_f32[3];
float32_t A_f32[9];
/* ----------------------------------------------------------------------
* Temporary buffers  for storing intermediate values
* ------------------------------------------------------------------- */
/* Transpose of A Buffer */
float32_t AT_f32[9];
/* (Transpose of A * A) Buffer */
float32_t ATMA_f32[9];
/* Inverse(Transpose of A * A)	Buffer */
float32_t ATMAI_f32[9];
/* Test Output Buffer */
float32_t X_f32[3];

static void setArrayA_f32(void){
	A_f32[0] = 2*(p1[0] - p2[0]);
	A_f32[1] = 2*(p1[1] - p2[1]);
	A_f32[2] = 2*(p1[2] - p2[2]);
	A_f32[3] = 2*(p1[0] - p3[0]);
	A_f32[4] = 2*(p1[1] - p3[1]);
	A_f32[5] = 2*(p1[2] - p3[2]);
	A_f32[6] = 2*(p1[0] - p4[0]);
	A_f32[7] = 2*(p1[1] - p4[1]);
	A_f32[8] = 2*(p1[2] - p4[2]);
}
static void setArrayB_f32(void){
    float d1,d2,d3,d4;
	float *R=uwbData.distance;
	
	d1 = p1[0]*p1[0] + p1[1]*p1[1] + p1[2]*p1[2];
	d2 = p2[0]*p2[0] + p2[1]*p2[1] + p2[2]*p2[2];
	d3 = p3[0]*p3[0] + p3[1]*p3[1] + p3[2]*p3[2];
	d4 = p4[0]*p4[0] + p4[1]*p4[1] + p4[2]*p4[2];
	B_f32[0] = R[1]*R[1] - R[0]*R[0] + d1 - d2;
	B_f32[1] = R[2]*R[2] - R[0]*R[0] + d1 - d3;
	B_f32[2] = R[3]*R[3] - R[0]*R[0] + d1 - d4;
}
static void uwbMatrixInit(void){
  uint16_t srcRows, srcColumns;
  srcRows = 3;
  srcColumns = 3;
  arm_mat_init_f32(&uwbData.A, srcRows, srcColumns, (float32_t *)A_f32);
  arm_mat_init_f32(&uwbData.AT, srcRows, srcColumns, AT_f32);
  arm_mat_init_f32(&uwbData.ATMA, srcRows, srcColumns, ATMA_f32);
  arm_mat_init_f32(&uwbData.ATMAI, srcRows, srcColumns, ATMAI_f32);
  srcRows = 3;
  srcColumns = 1;
  arm_mat_init_f32(&uwbData.B, srcRows, srcColumns, (float32_t *)B_f32);
  arm_mat_init_f32(&uwbData.X, srcRows, srcColumns, X_f32);
}
static void uwbPositionSolution(void) {
  arm_status status;
  uint32_t startTime = timerMicros();
  
  setArrayB_f32();
  /* calculation of A transpose */
  status = arm_mat_trans_f32(&uwbData.A, &uwbData.AT);
  /* calculation of AT Multiply with A */
  status = arm_mat_mult_f32(&uwbData.AT, &uwbData.A, &uwbData.ATMA);
  /* calculation of Inverse((Transpose(A) * A) */
  status = arm_mat_inverse_f32(&uwbData.ATMA, &uwbData.ATMAI);

  /* calculation of (Inverse((Transpose(A) * A)) *  Transpose(A)) */
  status = arm_mat_mult_f32(&uwbData.ATMAI, &uwbData.AT, &uwbData.ATMA);
  /* calculation ((Inverse((Transpose(A) * A)) *  Transpose(A)) * B) */
  status = arm_mat_mult_f32(&uwbData.ATMA, &uwbData.B, &uwbData.X);
  uwbData.deltaTime = timerMicros() - startTime;

}

void uwbTaskCode(void *pvParameters){
	portTickType xLastWakeTime;
	xLastWakeTime = xTaskGetTickCount();
	for ( ;; )
	{	
		if(uwbData.solveflag){
			uwbPositionSolution();
			uwbPostionShow();
		}
		utilTaskPeriodTime(uwbData.timer);
		/* 执行周期10Hz */
		vTaskDelayUntil(&xLastWakeTime, (100 / portTICK_RATE_MS));
	}
}


// send all beacon config to ardupilot
void send_beacon_config()
{
    union beacon_config_msg msg;
    msg.info.beacon_count = NUM_ANCHORS;
    for (uint8_t i=0; i<NUM_ANCHORS; i++) {
        msg.info.beacon_id = i;
        msg.info.x = anchors_x[i];
        msg.info.y = anchors_y[i];
        msg.info.z = heights[i];
        send_message(MSGID_BEACON_CONFIG, sizeof(msg.buf), msg.buf);
    }
}
// get ranges for each anchor
void get_ranges()
{
    // get range for each anchor
    for (uint8_t i=0; i<NUM_ANCHORS; i++) {
            // send info to ardupilot
            //send_beacon_distance(i, uwbData.distance[i]*10);
            send_beacon_distance(i, 1000+i*100);
        }

}

// send a beacon's distance to ardupilot
void send_beacon_distance(uint8_t beacon_id, uint32_t distance_mm)
{
    union beacon_distance_msg msg;
    msg.info.beacon_id = beacon_id;
    msg.info.distance = distance_mm;
    send_message(MSGID_BEACON_DIST, sizeof(msg.buf), msg.buf);
}

// send vehicle's position to ardupilot
void send_vehicle_position(void)
{
    union vehicle_position_msg msg;

    // sanity check position
    if (X_f32[0] == 0 || X_f32[1] == 0) {
        return;
    }

    /*msg.info.x = X_f32[0]*10;
    msg.info.y = X_f32[1]*10;
    msg.info.z = X_f32[2]*10;*/
    msg.info.x = 100*10;
    msg.info.y = 200*10;
    msg.info.z = 100*10;

    send_message(MSGID_POSITION, sizeof(msg.buf), msg.buf);
}
void uwbSendPacket(const uint8_t *buf, uint16_t len) {
    commTxBuf_t *txBuf;
    uint8_t *ptr;
    int i;

    txBuf = commGetTxBuf(COMM_STREAM_TYPE_POZYX, len);
    // cannot block, must fail
    if (txBuf != 0) {
	ptr = &txBuf->buf;

	for (i = 0; i < len; i++)
	    *ptr++ = *buf++;

	commSendTxBuf(txBuf, len);
    }
}

void send_message(uint8_t msg_id, uint8_t data_len, uint8_t data_buf[])
{
    // sanity check
    if (data_len == 0) {
        return;
    }

    // message is buffer length + 1 (for checksum)
    uint8_t msg_len = data_len+1;

    // calculate checksum and place in last element of array
    uint8_t checksum = 0;
    checksum ^= msg_id;
    checksum ^= msg_len;
    for (uint8_t i=0; i<data_len; i++) {
        checksum = checksum ^ data_buf[i];
    }

    // send message
    uint8_t buf[3];
	buf[0] = MSG_HEADER;
	buf[1] = msg_id;
	buf[2] = msg_len;
	uwbSendPacket((const uint8_t *)buf,3);
	uwbSendPacket((const uint8_t *)data_buf,data_len);
	uwbSendPacket(&checksum,1);
}

void uwbPozyxDo(void) {
    uwbData.loops++;
	if (!(uwbData.loops % 100)) {
		send_beacon_config();
		get_ranges();
		send_vehicle_position();
	}
	
}

void uwbParseString(void){
	int anchorID,tagID;
	float distance;
	int n;
	n = sscanf(uwbData.payload,"T:[%d],A:[%d],R:%f",&anchorID,&tagID,&distance);
	switch(anchorID){
		case ANCHOR_ID1:
			uwbData.distance[0] = distance;
		break;
		case ANCHOR_ID2:
			uwbData.distance[1] = distance;
		break;
		case ANCHOR_ID3:
			uwbData.distance[2] = distance;
		break;
		case ANCHOR_ID4:
			uwbData.distance[3] = distance;
		break;
		default:
		break;
	}

}
unsigned char uwbCharIn(unsigned char c) {
	uint8_t ret = 0;
    switch (uwbData.state) {
    case UWB_WAIT_HEADER1:
        if (c == 'S')
            uwbData.state = UWB_WAIT_HEADER2;
     	break;

    case UWB_WAIT_HEADER2:
        if (c == ':'){
            uwbData.state = UWB_WAIT_CNT;
			uwbData.mcnt = 0;
        }
        else
            uwbData.state = UWB_WAIT_HEADER1;
    	break;

    case UWB_WAIT_CNT:
		if(c == ','){
			uwbData.state = UWB_WAIT_PAYLOAD;
			uwbData.pcnt = 0;
		}
		else{
        	uwbData.measureCnt[uwbData.mcnt++] = c;
			if(uwbData.mcnt >6)
				uwbData.state = UWB_WAIT_HEADER1;	
		}
   
        break;

    case UWB_WAIT_PAYLOAD:
        if(c == 'c'){
			uwbData.state = UWB_WAIT_TAIL;
			uwbData.tcnt = 0;
        	}
		else{
        	uwbData.payload[uwbData.pcnt++] = c;
			if(uwbData.pcnt > 64)
				uwbData.state = UWB_WAIT_HEADER1;	
		}
        break;

    case UWB_WAIT_TAIL:
        if(c == '\n'){
			uwbData.state = UWB_WAIT_HEADER1;
			ret = 1;
        }
		else{
			uwbData.tail[uwbData.tcnt++] = c;
			if(uwbData.tcnt > 4)
				uwbData.state = UWB_WAIT_HEADER1;
		}
        break;
    default:
        break;
    }

    return ret;
}
void uwbPostionShow(void){
	char dtbuf[50];	 	  
	POINT_COLOR=RED;
	sprintf(dtbuf,"ID2192-d1:%.2f cm",uwbData.distance[0]);
	LCD_ShowString(30,120,200,16,16,(u8*)dtbuf);
	sprintf(dtbuf,"ID2296-d2:%.2f cm",uwbData.distance[1]);
	LCD_ShowString(30,140,200,16,16,(u8*)dtbuf);
	sprintf(dtbuf,"ID2246-d3:%.2f cm",uwbData.distance[2]);
	LCD_ShowString(30,160,200,16,16,(u8*)dtbuf);
	sprintf(dtbuf,"ID2282-d4:%.2f cm",uwbData.distance[3]);
	LCD_ShowString(30,180,200,16,16,(u8*)dtbuf);
	LCD_ShowString(30,200,200,16,16,"Position:");
	LCD_ShowString(30,220,200,16,16,"x =");
	sprintf(dtbuf,"                                ");
	LCD_ShowString(30,240,200,16,16,(u8*)dtbuf);
	sprintf(dtbuf,"[%.2f;%.2f;%.2f]",X_f32[0],X_f32[1],X_f32[2]);
 	LCD_ShowString(30,240,200,16,16,(u8*)dtbuf);	 	
	LCD_ShowString(30,260,200,16,16,"b =");
	sprintf(dtbuf,"                                ");	
	LCD_ShowString(30,280,200,16,16,(u8*)dtbuf);
	sprintf(dtbuf,"[%.0f;%.0f;%.0f]",B_f32[0],B_f32[1],B_f32[2]);	
	LCD_ShowString(30,280,200,16,16,(u8*)dtbuf);
}
void uwbSendNotice(const char *s) {
    commTxBuf_t *txBuf;
    uint8_t *ptr;
    txBuf = commGetTxBuf(COMM_STREAM_TYPE_TELEMETRY, 64);//长度固定

    if (txBuf > 0) {
	ptr = &txBuf->buf;
	do {
	    *ptr++ = *s;
	} while (*(s++));

	//最后ptr还自增了一次，指向了字符串末尾的空字符，故减1
	commSendTxBuf(txBuf, ptr - &txBuf->buf-1);
    }

}

void uwbRecvTaskCode(commRcvrStruct_t *r) {
	char c;
	unsigned int ret = 0;
    while (commAvailable(r)){
		c = commReadChar(r);
		ret = uwbCharIn(c);
		if(ret == 1)
			uwbParseString();
    }
}

void uwbInit(void) {
    memset((void *)&uwbData, 0, sizeof(uwbData));
	setArrayA_f32();
	uwbMatrixInit();
	LCD_Init();
	POINT_COLOR=BLUE;
	LCD_ShowString(30,20,200,16,16,"--------------------");
	LCD_ShowString(30,40,200,16,16,"UWBLOC TEST");	
	LCD_ShowString(30,60,200,16,16,"KEY0:start/stop");	
	LCD_ShowString(30,80,200,16,16,"--------------------");
	POINT_COLOR=RED;
	LCD_ShowString(30,100,200,16,16,"Distance:");

	commRegisterTelemFunc(uwbPozyxDo);
	commRegisterRcvrFunc(COMM_STREAM_TYPE_TELEMETRY, uwbRecvTaskCode);
	uwbData.rs485TXEnable = digitalInit(UWB_RS485_TXEN_PORT, UWB_RS485_TXEN_PIN, 0);//设置为0，为485接收模式
	xTaskCreate((TaskFunction_t )uwbTaskCode,     	
                (const char*    )"uwb_task",   	
                (uint16_t       )UWB_STACK_SIZE, 
                (void*          )NULL,				
                (UBaseType_t    )UWB_PRIORITY,	
                (TaskHandle_t*  )&uwbTaskHandle);  
	

}


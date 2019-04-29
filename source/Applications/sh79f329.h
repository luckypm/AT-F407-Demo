#ifndef __SH79F329_H
#define __SH79F329_H

#include "aq.h"

#define SH79F329_STACK_SIZE	    200
#define SH79F329_PRIORITY	    10



typedef struct
{
	uint16_t readValue;
	uint32_t cnt;
	uint8_t  flag;
	uint8_t start;
	uint8_t stop;
	uint8_t status;
	float lat;
	float lon;
	double lat_double;
	double lon_double;
}  sh79f329Struct_t;

extern sh79f329Struct_t sh79f329Data;
extern void sh79f329Init(void);





#endif



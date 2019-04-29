#ifndef __PWM_H
#define __PWM_H

#include "aq.h"

//#define OLD_FOOT
#define NEW_FOOT
#define PWM_STACK_SIZE	    200
#define PWM_PRIORITY	    10


typedef enum{
   FOOT_TAKE_OFF = 0,
   FOOT_TAKE_ON = 1
}FOOT_Types_t;
typedef enum{
	  PWM_ENABLE = 0,
	  PWM_DISABLE = 1
}PWM_Status_t;

typedef struct
{
	uint32_t pwm;
	uint8_t  flag;
	uint32_t downLoop;
	uint8_t status;
}  pwmStruct_t;

extern pwmStruct_t pwmData;
extern void pwmInit(void);





#endif


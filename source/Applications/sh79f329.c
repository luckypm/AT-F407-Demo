
#include "supervisor.h"
#include "util.h"
#include "lcd.h"
#include "hmc5983.h"
#include "sh79f329.h"
#include <stdlib.h>
#include <string.h>

float lat = 110.123456789;//测试float精度
float lon = 54.987654321;
double lat_double = 110.123456789;
double lon_double = 54.123456789;

sh79f329Struct_t sh79f329Data __attribute__((section(".ccm")));

TaskHandle_t sh79f329TaskHandle;


void sh79f329TaskCode(void *pvParameters){
	portTickType xLastWakeTime;
	xLastWakeTime = xTaskGetTickCount();
	for ( ;; )
	{	
//		char dtbuf[50];	
//		if(sh79f329Data.start == 1)
//			sh79f329Data.readValue =  sh79f329GetReg(0x02);
//		if(sh79f329Data.readValue > 0){
//			LCD_ShowString(30,120,200,16,16,"readValue");
//			sprintf(dtbuf,"value = %d",sh79f329Data.readValue);
//			LCD_ShowString(30,140,200,16,16,(u8*)dtbuf);
//		}
		sh79f329Data.cnt++;
		sh79f329Data.lat = lat;
		sh79f329Data.lon = lon;
		sh79f329Data.lat_double = lat_double;
		sh79f329Data.lon_double = lon_double;
		/* 执行周期10Hz */
		vTaskDelayUntil(&xLastWakeTime, (100 / portTICK_RATE_MS));
	}
}

void sh79f329Init(void) {
    memset((void *)&sh79f329Data, 0, sizeof(sh79f329Data));
	LCD_Init();
	POINT_COLOR=BLUE;
	LCD_ShowString(30,20,200,16,16,"--------------------");
	LCD_ShowString(30,40,200,16,16,"sh79f329 test");	
	LCD_ShowString(30,60,200,16,16,"KEY_UP:UP;KEY_1:DOWN");	
	LCD_ShowString(30,80,200,16,16,"--------------------");
	POINT_COLOR=RED;
	hmc5983PreInit();

	xTaskCreate((TaskFunction_t )sh79f329TaskCode,     	
                (const char*    )"sh79f329_task",   	
                (uint16_t       )SH79F329_STACK_SIZE, 
                (void*          )NULL,				
                (UBaseType_t    )SH79F329_PRIORITY,	
                (TaskHandle_t*  )&sh79f329TaskHandle);  
	

}




#include "aq.h"
#include "config.h"
#include "comm.h"
#include "supervisor.h"
#include "digital.h"
#include "util.h"
#include "d_imu.h"
#include "key.h"
#include "uwb.h"
#include "pwm.h"
#include "sh79f329.h"
#include "lcd.h"

#include <stdlib.h>
#include <string.h>

supervisorStruct_t supervisorData __attribute__((section(".ccm")));

TaskHandle_t supervisoTaskHandle;

void supervisorLEDsOn(void) {
    digitalLo(supervisorData.readyLed);
}

void supervisorLEDsOff(void) {
    digitalHi(supervisorData.readyLed);
}
void supervisorLEDsTogg(void) {
    digitalTogg(supervisorData.readyLed);
}


void supervisorTare(void) {
    supervisorLEDsOn();
    dIMUTare();
    AQ_NOTICE("Level calibration complete.\n");
    supervisorLEDsOff();
}

void supervisorTaskCode(void *pvParameters){
	portTickType xLastWakeTime;
	xLastWakeTime = xTaskGetTickCount();
	for ( ;; )
	{
		/************************可替换部分*********************************/
		char dtbuf[50];	
		LCD_ShowString(30,120,200,16,16,"mag_value:");
		sprintf(dtbuf,"mag_x = %f",hmc5983Data.mag[0]);
		LCD_ShowString(30,140,200,16,16,(u8*)dtbuf);
		sprintf(dtbuf,"mag_y = %f",hmc5983Data.mag[1]);
		LCD_ShowString(30,160,200,16,16,(u8*)dtbuf);
		sprintf(dtbuf,"mag_z = %f",hmc5983Data.mag[2]);
		LCD_ShowString(30,180,200,16,16,(u8*)dtbuf);
		/*********************************************************/
		supervisorLEDsTogg();
		supervisorData.key = KEY_Scan(0);
		if(supervisorData.key == KEY0_PRES){
			uwbData.solveflag = !uwbData.solveflag;
		}
		else if(supervisorData.key == WKUP_PRES){
			pwmData.flag = FOOT_TAKE_ON;
		}
		else if(supervisorData.key == KEY1_PRES){
			pwmData.flag = FOOT_TAKE_OFF;
		}

		if(supervisorData.tareSwitch)
		{
			supervisorData.tareSwitch = 0;
			supervisorTare();
#ifdef DIMU_HAVE_EEPROM
                dIMURequestCalibWrite();
#endif
			
		}
		utilTaskPeriodTime(supervisorData.timer);
		/* 执行周期1Hz */
		//vTaskDelayUntil(&xLastWakeTime, (100 / portTICK_RATE_MS));
		vTaskDelay(100);
	}
}

void supervisorInit(void) {
    memset((void *)&supervisorData, 0, sizeof(supervisorData));
	LCD_Init();
	/************************可替换部分*********************************/
	POINT_COLOR=BLUE;
	LCD_ShowString(30,20,200,16,16,"--------------------");
	LCD_ShowString(30,40,200,16,16,"mag test");	
	LCD_ShowString(30,60,200,16,16,"");	
	LCD_ShowString(30,80,200,16,16,"--------------------");
	POINT_COLOR=RED;
	/**************************************************************/
    supervisorData.readyLed = digitalInit(SUPERVISOR_READY_PORT, SUPERVISOR_READY_PIN, 1);
	xTaskCreate((TaskFunction_t )supervisorTaskCode,     	
                (const char*    )"supervisor_task",   	
                (uint16_t       )SUPERVISOR_STACK_SIZE, 
                (void*          )NULL,				
                (UBaseType_t    )SUPERVISOR_PRIORITY,	
                (TaskHandle_t*  )&supervisoTaskHandle);  

}

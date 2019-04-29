

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
		supervisorLEDsTogg();
		supervisorData.key = KEY_Scan(0);
		if(supervisorData.key == KEY0_PRES){
			uwbData.solveflag = !uwbData.solveflag;
		}
		else if(supervisorData.key == WKUP_PRES){
			pwmData.flag = FOOT_TAKE_ON;
			sh79f329Data.start = 1;
		}
		else if(supervisorData.key == KEY1_PRES){
			pwmData.flag = FOOT_TAKE_OFF;
			sh79f329Data.start = 0;
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
		/* Ö´ÐÐÖÜÆÚ1Hz */
		//vTaskDelayUntil(&xLastWakeTime, (100 / portTICK_RATE_MS));
		vTaskDelay(100);
	}
}

void supervisorInit(void) {
    memset((void *)&supervisorData, 0, sizeof(supervisorData));
    supervisorData.readyLed = digitalInit(SUPERVISOR_READY_PORT, SUPERVISOR_READY_PIN, 1);
	xTaskCreate((TaskFunction_t )supervisorTaskCode,     	
                (const char*    )"supervisor_task",   	
                (uint16_t       )SUPERVISOR_STACK_SIZE, 
                (void*          )NULL,				
                (UBaseType_t    )SUPERVISOR_PRIORITY,	
                (TaskHandle_t*  )&supervisoTaskHandle);  

}

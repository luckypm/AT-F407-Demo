#include "pwm.h"
#include "supervisor.h"
#include "util.h"
#include "lcd.h"
#include <stdlib.h>
#include <string.h>


pwmStruct_t pwmData __attribute__((section(".ccm")));

TaskHandle_t pwmTaskHandle;

/*
 *@brief	PWM1模式，输出极性高，则在CNT<CCR时为高电平，故设置的CCR值大小就相当于PWM的高电平脉宽，即PWM值
 			用来控制起落架
 *@param 	resolution: PWM值的分辨率，1PWM值表示1/resolution 秒
 *@param 	freq: PWM的频率
 *@param 	inititalValue: PWM的初始值
 *
 */
void pwmInitOut(uint32_t resolution, uint32_t freq, uint32_t inititalValue)
{
	uint32_t period = resolution / freq;
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
	TIM_OCInitTypeDef TIM_OCInitStructure;
	GPIO_InitTypeDef GPIO_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);

	TIM_DeInit(TIM3);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7 | GPIO_Pin_6;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;
	//GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_DOWN;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	GPIO_PinAFConfig(GPIOA, GPIO_PinSource7, GPIO_AF_TIM3);
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource6, GPIO_AF_TIM3);

	TIM_TimeBaseStructure.TIM_Period = period - 1 ;//AAR的值
	TIM_TimeBaseStructure.TIM_Prescaler = 84000000/resolution - 1;//定时器分频值
	TIM_TimeBaseStructure.TIM_ClockDivision = 0;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;//向上计数

	TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);

	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;//PWM1为CNT<CCR时为有效状态，否则为无效状态；PWM2为CNT<CCR时为无效，否则为有效
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_Pulse = inititalValue;
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;//输出极性高，高电平有效，LOW为低电平有效
	TIM_OC1Init(TIM3, &TIM_OCInitStructure);
	TIM_OC1PreloadConfig(TIM3, TIM_OCPreload_Enable);
	TIM_OC2Init(TIM3, &TIM_OCInitStructure);
	TIM_OC2PreloadConfig(TIM3, TIM_OCPreload_Enable);


	TIM_ARRPreloadConfig(TIM3, ENABLE);
//	TIM_ITConfig(TIM3,TIM_IT_Update,DISABLE); 
//
//	NVIC_InitStructure.NVIC_IRQChannel=TIM3_IRQn; 
//	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=0x01; 
//	NVIC_InitStructure.NVIC_IRQChannelSubPriority=0x00; 
//	NVIC_InitStructure.NVIC_IRQChannelCmd=ENABLE;
//	NVIC_Init(&NVIC_InitStructure);

	TIM_Cmd(TIM3, ENABLE);
}



void enableFootPWM(void){
	if(pwmData.downLoop != 0)
	{
		pwmData.downLoop = 0;
		pwmData.status = PWM_ENABLE;
	}
}


void disableFootPWM(void){
	if(++pwmData.downLoop == 20)//20*10=200ms
	{
		TIM_SetCompare1(TIM3,0);
		TIM_SetCompare2(TIM3,0);
		pwmData.status = PWM_DISABLE;
	}
}
//收起落架
void takeOnFoot(void){	
	enableFootPWM();
	if(pwmData.pwm > 1400){
		pwmData.pwm -= 6;
		pwmData.pwm = constrain(pwmData.pwm, 1300, 2000);
		TIM_SetCompare1(TIM3,pwmData.pwm);
		TIM_SetCompare2(TIM3,pwmData.pwm);
	}

}
//放起落架
void takeOffFoot(void){
	if(pwmData.pwm < 2000){
		pwmData.pwm += 6;
		pwmData.pwm = constrain(pwmData.pwm, 1300, 2000);
		TIM_SetCompare1(TIM3,pwmData.pwm);
		TIM_SetCompare2(TIM3,pwmData.pwm);
	}
	else
	{
		disableFootPWM();
	}

}



void pwmTaskCode(void *pvParameters){
	portTickType xLastWakeTime;
	xLastWakeTime = xTaskGetTickCount();
	for ( ;; )
	{	char dtbuf[50];	
		if(pwmData.flag == FOOT_TAKE_ON)
			takeOnFoot();
		else if(pwmData.flag == FOOT_TAKE_OFF)
			takeOffFoot();
		if(pwmData.status == PWM_ENABLE){
			LCD_ShowString(30,120,200,16,16,"PWM  enable");
			sprintf(dtbuf,"PWM value = %d",pwmData.pwm);
			LCD_ShowString(30,140,200,16,16,(u8*)dtbuf);
		}
		else{
			LCD_ShowString(30,120,200,16,16,"PWM disable");
			sprintf(dtbuf,"PWM value =                " );
			LCD_ShowString(30,140,200,16,16,(u8*)dtbuf);
			sprintf(dtbuf,"PWM value =  0" );
			LCD_ShowString(30,140,200,16,16,(u8*)dtbuf);
		}
		/* 执行周期20Hz */
		vTaskDelayUntil(&xLastWakeTime, (10 / portTICK_RATE_MS));
	}
}

void pwmInit(void) {
    memset((void *)&pwmData, 0, sizeof(pwmData));
	pwmInitOut(1000000, 50, 2000);
	pwmData.pwm = 2000;
	LCD_Init();
	POINT_COLOR=BLUE;
	LCD_ShowString(30,20,200,16,16,"--------------------");
	LCD_ShowString(30,40,200,16,16,"PWM TEST--50HZ");	
	LCD_ShowString(30,60,200,16,16,"KEY_UP:UP;KEY_1:DOWN");	
	LCD_ShowString(30,80,200,16,16,"--------------------");
	POINT_COLOR=RED;


	xTaskCreate((TaskFunction_t )pwmTaskCode,     	
                (const char*    )"pwm_task",   	
                (uint16_t       )PWM_STACK_SIZE, 
                (void*          )NULL,				
                (UBaseType_t    )PWM_PRIORITY,	
                (TaskHandle_t*  )&pwmTaskHandle);  
	

}

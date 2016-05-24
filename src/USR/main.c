#include "gpio.h"
#include "uart.h"
#include "pit.h"
#include "adc.h"
#include "stdio.h"
#include "TPM.h"
#include "user.h"
#include "app.h"
#include "counter.h"

static uint32_t time = 0;

static int32_t balance = 0, speed = 0, turn = 0;

int main(void){
  //������ȷ������ⲿ�����Ƿ��Ӧ��8M���������ClockSource_EX8M��
	//50M���������ClockSource_EX50M����ƵƵ����ʹ�� go to�鿴��������
  SystemClockSetup(ClockSource_EX50M,CoreClock_120M);
	UART_PortInit(UART0_RX_PD06_TX_PD07,128000);
	UART_PortInit(UART1_RX_PE01_TX_PE00,96000);
  DisableInterrupts();
	
	PWM_userInit();
	IMU_userInit();
	PIT_userInit();
	GPIO_userInit();
	Counter0_Init();
	Counter1_Init();
	inductance_userInit();

	while(1){
		if(PIT_GetITStatus(PIT0, PIT_IT_TIF) == SET){
			PIT_ClearITPendingBit(PIT0, PIT_IT_TIF);
			
			if(time<300) { ++time; }
			
			balance = balanceCtrl();
			speed = speedCtrl();
			turn = directionCtrl();
		}
		if(time < 300) {
			balance = speed = turn = 0;
		}
		
		motorControl(balance, speed, turn);
	}
}

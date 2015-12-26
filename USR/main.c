
#include "gpio.h"
#include "uart.h"
#include "delay.h"
#include "led.h"
#include "dma.h"
#include "spi.h"
#include "pit.h"
#include "adc.h"
#include "i2c.h"
#include "isr.h"
#include "accel.h"
#include "crc.h"
#include "pwm.h"
#include "stdio.h"
#include "TPM.h"
#include "user.h"
#include "app.h"

int main(void){
  //������ȷ������ⲿ�����Ƿ��Ӧ��8M���������ClockSource_EX8M��
	//50M���������ClockSource_EX50M����ƵƵ����ʹ�� go to�鿴��������
  SystemClockSetup(ClockSource_EX50M,CoreClock_120M);

  DelayInit();
	ledInit(PTB,0);
	
  DisableInterrupts();
	
	PWMInit(PTA4,DIV1,65535);
	PWMInit(PTA12,DIV1,65535);
	//ADC_userInit();
	//GPIO_userInit();
	//PIT_userInit();
	
	while(1){
		/*
		balanceDataTypeDef tmp_balance;
		spdTypeDef spd;
		uint8_t Tim = timer();
		switch(Tim){
			case 1:
				getBalanceData(&tmp_balance);
				break;
			case 2:
				spd.m_spd_balance = balanceControl(&tmp_balance);
				break;
			case 5:
				motorControl(&spd);
				break;
			default:
				break;
		}*/
		PWMOutput(PTA4, 0x8000);
		PWMOutput(PTA12, 0x7000);
	}
}

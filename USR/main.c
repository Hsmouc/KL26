
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

#define duty_to_value(x) (uint16_t)(65535*x)

int main(void){
  //������ȷ������ⲿ�����Ƿ��Ӧ��8M���������ClockSource_EX8M��
	//50M���������ClockSource_EX50M����ƵƵ����ʹ�� go to�鿴��������
  SystemClockSetup(ClockSource_EX50M,CoreClock_120M);

  DelayInit();
	ledInit(PTB,0);
	
  EnableInterrupts();
	
	PWMInit(PTA4,DIV1,65535);
	PWMInit(PTA12,DIV1,65535);	
	ADC_userInit();
	GPIO_userInit();
	
	while(1){
    DelayMs(500);
    twinkleLed(PTB,0);
	}
}





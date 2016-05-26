#include "adc.h"

static uint8_t ADC_Cal(ADC_Type *ADCx);

void ADC_Init(ADC_InitTypeDef* ADC_InitStruct)
{
	uint8_t i;
	
	ADC_Type *ADCx = ADC0;
	
	PeripheralMapTypeDef *pADC_Map = (PeripheralMapTypeDef*)&(ADC_InitStruct->ADCxMap);
	
	SIM->SCGC6 |= SIM_SCGC6_ADC0_MASK;  //打开时钟门
	
	ADCx->CFG1 &= ~(ADC_CFG1_MODE_MASK);   //设置AD精度
	
	ADCx->CFG1 |= ADC_CFG1_MODE(ADC_InitStruct->ADC_Precision);
	
	ADCx->CFG1 &= ~ADC_CFG1_ADICLK_MASK;  //总线时钟为ad时钟源
	
	ADCx->CFG1 |=  ADC_CFG1_ADICLK(0); 
	
	ADCx->CFG1 &= ~ADC_CFG1_ADLSMP_MASK;  //抽样时间

	ADCx->CFG1 &= ~ADC_CFG1_ADIV_MASK;    //抽样时钟分频
	
	ADCx->CFG1 |= ADC_CFG1_ADIV(3); 

	if(pADC_Map->m_PinCntIndex > 1)     	// 单通道、差分通道
	{
		ADCx->SC1[pADC_Map->m_SpecDefine1] |= ADC_SC1_DIFF_MASK; 
	}
	else
	{
		ADCx->SC1[pADC_Map->m_SpecDefine1] &= ~ADC_SC1_DIFF_MASK; 
	}
	//Calieration
	i = ADC_Cal(ADCx);
	//触发源
	(ADC_TRIGGER_HW == ADC_InitStruct->ADC_TriggerSelect)?(ADCx->SC2 |= ADC_SC2_ADTRG_MASK):(ADCx->SC2 &= ~ADC_SC2_ADTRG_MASK);
	
	for(i=0;i<pADC_Map->m_PinCntIndex;i++)
	{
		PinMuxConfig(pADC_Map->m_PortIndex,pADC_Map->m_PinBaseIndex+i,pADC_Map->m_MuxIndex);
	}
}

void ADC_ITConfig(ADC_Type* ADCx,uint8_t ADC_Mux, uint16_t ADC_IT, FunctionalState NewState)
{
	switch(ADC_IT)
	{
		case ADC_IT_AI:
			(ENABLE == NewState)?(ADCx->SC1[ADC_Mux] |= ADC_SC1_AIEN_MASK):(ADCx->SC1[ADC_Mux] &= ~ADC_SC1_AIEN_MASK);
			break;
		default:break;
	}
}

void ADC_DMACmd(ADC_Type* ADCx, uint16_t ADC_DMAReq, FunctionalState NewState)
{
	switch(ADC_DMAReq)
	{
		case ADC_DMAReq_COCO:
			(NewState == ENABLE)?(ADCx->SC2 |= ADC_SC2_DMAEN_MASK):(ADCx->SC2 &= ~ADC_SC2_DMAEN_MASK);
			break;
			default:break;
	}
}

uint32_t ADC_GetConversionValue(uint32_t ADCxMap)
{
	ADC_Type *ADCx = ADC0;
	PeripheralMapTypeDef *pADC_Map = (PeripheralMapTypeDef*)&ADCxMap;
	

	ADCx->SC1[pADC_Map->m_SpecDefine1] &= ~(ADC_SC1_ADCH_MASK);	
	ADCx->SC1[pADC_Map->m_SpecDefine1] |= pADC_Map->m_ChlIndex;

	while(!(ADCx->SC1[pADC_Map->m_SpecDefine1] & ADC_SC1_COCO_MASK));  //等待转换完毕
	//返回结果
	return ADCx->R[pADC_Map->m_SpecDefine1];
}


ITStatus ADC_GetITStatus(ADC_Type* ADCx, uint8_t ADC_Mux, uint16_t ADC_IT)
{
	ITStatus retval;
	switch(ADC_IT)
	{
		case ADC_IT_AI:
			(ADCx->SC1[ADC_Mux] & ADC_SC1_COCO_MASK)?(retval = SET):(retval = RESET);
			break;
		default:break;
	}
	return retval;
}


//======================================================================
//清除中断标志
//入口：端口：ADCx 通道：ADCx:ADC_Mux 
//返回：无
//
//======================================================================
void ADC_ClearITPendingBit(ADC_Type* ADCx, uint8_t ADC_Mux, uint16_t ADC_IT)
{
	uint8_t i = 0;
	i = i;
	switch(ADC_IT)
	{
		case ADC_IT_AI:
		  i = ADCx->R[ADC_Mux];
			break;
		default:break;
	}
}


//ADC 校准
static uint8_t ADC_Cal(ADC_Type *ADCx)
{
  uint16_t cal_var;
  ADCx->SC2 &= ~ADC_SC2_ADTRG_MASK; //软件触发
	ADCx->SC3 &= ~ADC_SC3_ADCO_MASK;  
	//32样本平均
	ADCx->SC3 &= ~ADC_SC3_AVGS_MASK;
	ADCx->SC3 |=  ADC_SC3_AVGS(3);
	//使能硬件平均
	ADCx->SC3 |= ADC_SC3_AVGE_MASK;
	//开始校准
  ADCx->SC3 |= ADC_SC3_CAL_MASK ; 
	while((ADCx->SC1[A] & ADC_SC1_COCO_MASK) == 0);
	if((ADCx->SC3 & ADC_SC3_CALF_MASK) == ADC_SC3_CALF_MASK)
	{
		return 1;
	}
  // 正则校准
  cal_var = 0x00;
  cal_var =  ADCx->CLP0; 
  cal_var += ADCx->CLP1; 
  cal_var += ADCx->CLP2; 
  cal_var += ADCx->CLP3; 
  cal_var += ADCx->CLP4; 
  cal_var += ADCx->CLPS; 
  cal_var = cal_var/2;
  cal_var |= 0x8000; //设置MSB
	ADCx->PG =  ADC_PG_PG(cal_var);
  // 负面校准
  cal_var = 0x00;
  cal_var =  ADCx->CLM0; 
  cal_var += ADCx->CLM1; 
  cal_var += ADCx->CLM2; 
  cal_var += ADCx->CLM3; 
  cal_var += ADCx->CLM4; 
  cal_var += ADCx->CLMS; 
  cal_var = cal_var/2;
  cal_var |= 0x8000; //设置msb
	ADCx->MG = ADC_MG_MG(cal_var); 
  ADCx->SC3 &= ~ADC_SC3_CAL_MASK;
  return 0;
}
/*
static const PeripheralMapTypeDef ADC_Check_Maps[] = 
{ 
	{0, 4, 1,20, 2, 0, 0},  //ADC0_DP0_PE20_DM0_PE21
	{0, 4, 1,16, 2, 1, 0},  //ADC0_DP1_PE16_DM1_PE17
	{0, 4, 1,18, 2, 2, 0},  //ADC0_DP2_PE18_DM2_PE19
	{0, 4, 1,22, 2, 3, 0},  //ADC0_DP3_PE22_DM3_PE23
	{0, 4, 1,16, 1, 1, 0},  //ADC0_SE1A_PE16
	{0, 4, 1,17, 1, 5, 0},  //ADC0_SE5A_PE17
	{0, 4, 1,18, 1, 0, 0},  //ADC0_SE2A_PE18
	{0, 4, 1,19, 1, 6, 0},  //ADC0_SE6A_PE19
	{0, 4, 1,20, 1, 0, 0},  //ADC0_SE0A_PE20
	{0, 4, 1,21, 1, 4, 0},  //ADC0_SE4A_PE21
	{0, 4, 1,22, 1, 3, 0},  //ADC0_SE3A_PE22
	{0, 4, 1,23, 1, 7, 0},  //ADC0_SE7A_PE23
	{0, 4, 1,29, 1, 4, 1},  //ADC0_SE4B_PE29
	{0, 4, 1,30, 1,23, 0},  //ADC0_SE23A_PE30
	{0, 1, 1, 0, 1, 8, 0},  //ADC0_SE8A_PB0
	{0, 1, 1, 1, 1, 9, 0},  //ADC0_SE9A_PB1
	{0, 1, 1, 2, 1,12, 0},  //ADC0_SE12A_PB2
	{0, 1, 1, 3, 1,13, 0},  //ADC0_SE13A_PB3
	{0, 2, 1, 0, 1,14, 0},  //ADC0_SE14A_PC0
	{0, 2, 1, 1, 1,15, 0},  //ADC0_SE15A_PC1
	{0, 2, 1, 2, 1,11, 0},  //ADC0_SE11A_PC2
	{0, 3, 1, 1, 1, 5, 1},  //ADC0_SE5B_PD1
	{0, 3, 1, 5, 1, 6, 1},  //ADC0_SE6B_PD5
	{0, 3, 1, 6, 1, 7, 1},  //ADC0_SE7B_PD6
};
void ADC_CalConstValue(void)
{
	uint8_t i =0;
	uint32_t value = 0;
	for(i=0;i<sizeof(ADC_Check_Maps)/sizeof(PeripheralMapTypeDef);i++)
	{
		value =   ADC_Check_Maps[i].m_ModuleIndex<<0;
		value |=  ADC_Check_Maps[i].m_PortIndex <<3;
		value |=  ADC_Check_Maps[i].m_MuxIndex<<6;
		value |=  ADC_Check_Maps[i].m_PinBaseIndex<<9;
		value |=  ADC_Check_Maps[i].m_PinCntIndex<<14;
		value |=  ADC_Check_Maps[i].m_ChlIndex<<17;
		UART_printf("(0x%xU)\r\n",value);
	}
}
*/

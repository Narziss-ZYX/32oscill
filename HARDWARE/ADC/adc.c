#include "adc.h"
#include "delay.h"
#include "malloc.h"
#include "usart.h"

/******ADC寄存器地址声明*******/	
#define ADC1_DR_ADDRESS    ((uint32_t)0x4001204C)   //ADC通道1输出寄存器地址
u16 *Array_ADC1Covert=0;  //指向ADC1转化后的波形数组
u8  DMA2_TransferCompleteFlag = 0; //DMA2传输完成标志位

 //初始化ADC															   
void  Adc_Init(void)
{    
  GPIO_InitTypeDef  GPIO_InitStructure;
	ADC_CommonInitTypeDef ADC_CommonInitStructure;
	ADC_InitTypeDef       ADC_InitStructure;
	
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);//使能GPIOA时钟
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE); //使能ADC1时钟

  //先初始化ADC1通道5 IO口
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5;//PA5 通道5
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;//模拟输入
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL ;//不带上下拉
  GPIO_Init(GPIOA, &GPIO_InitStructure);//初始化  
 
	RCC_APB2PeriphResetCmd(RCC_APB2Periph_ADC1,ENABLE);	  //ADC1复位
	RCC_APB2PeriphResetCmd(RCC_APB2Periph_ADC1,DISABLE);	//复位结束	 
 
	
  ADC_CommonInitStructure.ADC_Mode = ADC_Mode_Independent;//独立模式
  ADC_CommonInitStructure.ADC_TwoSamplingDelay = ADC_TwoSamplingDelay_5Cycles;//两个采样阶段之间的延迟5个时钟
  ADC_CommonInitStructure.ADC_DMAAccessMode = ADC_DMAAccessMode_Disabled; //DMA失能
  ADC_CommonInitStructure.ADC_Prescaler = ADC_Prescaler_Div4;//预分频4分频。ADCCLK=PCLK2/4=84/4=21Mhz,ADC时钟最好不要超过36Mhz 
  ADC_CommonInit(&ADC_CommonInitStructure);//初始化
	
  ADC_InitStructure.ADC_Resolution = ADC_Resolution_12b;//12位模式
  ADC_InitStructure.ADC_ScanConvMode = DISABLE;//非扫描模式	
  ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;//开启连续转换
  ADC_InitStructure.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_None;//禁止触发检测，使用软件触发
  ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;//右对齐	
  ADC_InitStructure.ADC_NbrOfConversion = 1;//1个转换在规则序列中 也就是只转换规则序列1 
  ADC_Init(ADC1, &ADC_InitStructure);//ADC初始化
	
	ADC_RegularChannelConfig(ADC1, ADC_Channel_5, 1, ADC_SampleTime_84Cycles );//ADC1,ADC通道,480个周期,提高采样时间可以提高精确度	
	
	ADC_DMARequestAfterLastTransferCmd(ADC1, ENABLE);//使能DMA请求(单ADC模式)
	ADC_DMACmd(ADC1, ENABLE); //使能DMA
	
  ADC_Cmd(ADC1, ENABLE);//开启AD转换器	
	ADC_SoftwareStartConv(ADC1);   //软件转换
	

}

/*************DMA初始化************
	par:外设地址
	mar:内存地址
	ndtr:数据传输量
*/
void DMA2_Int_Init(u32 par,u16 *mar,u16 ndtr)
{
	DMA_InitTypeDef            DMA_InitStructure;
	NVIC_InitTypeDef      NVIC_InitStructure;
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2,ENABLE);//DMA2时钟使能
	
	DMA_ITConfig(DMA2_Stream0, DMA_IT_TC, ENABLE);   //开启DMA2中断
  
	DMA_InitStructure.DMA_Channel = DMA_Channel_0; //通道选择ADC1	
	
	DMA_StructInit( &DMA_InitStructure);		//DMA结构初始化
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;//外设到内存
	DMA_InitStructure.DMA_BufferSize = ndtr;//缓存大小，一般为256点
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;//外设地址不递增
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;	//内存地址递增
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;//宽度为半字16位
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;//宽度为半字
	DMA_InitStructure.DMA_Priority = DMA_Priority_High;//高优先级
	DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;//循环发送模式
	DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;         
  DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_Full;
	DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;//存储器突发单次传输
  DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;//外设突发单次传输

	DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)par;//外设地址为ADC数据寄存器
	DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)mar;//内存地址为输出波形数据数组
	DMA_Init(DMA2_Stream0,&DMA_InitStructure);  //第0流
  DMA_Cmd(DMA2_Stream0,ENABLE);    //启动DMA2的通道0

  /*DMA2中断配置*/
	NVIC_InitStructure.NVIC_IRQChannel = DMA2_Stream0_IRQn;//外部中断0
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;//抢占优先级3
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;//子优先级3
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;//使能外部中断通道
  NVIC_Init(&NVIC_InitStructure);//配置


     
}

//void DMA2_Stream0_IRQHandler(void)
//{
//	static int i=0;
//	printf("counter:%d  ",i);
//	printf("DMA2_TransferCompleteFlag:%d\r\n",DMA2_TransferCompleteFlag);
//	i++;
//	ADC_DMACmd(ADC1, DISABLE);
//	if(DMA_GetITStatus(DMA2_Stream0,DMA_IT_TCIF0))
//	{
//		DMA2_TransferCompleteFlag=1;
////		printf("DMA2_TransferCompleteFlag:%d\r\n",DMA2_TransferCompleteFlag);
//	}
//	DMA_ClearITPendingBit(DMA2_Stream0,DMA_IT_TCIF0);
//}


//获得ADC值
//ch: @ref ADC_channels 
//通道值 0~16取值范围为：ADC_Channel_0~ADC_Channel_16
//返回值:转换结果
u16 Get_Adc(u8 ch)   
{
	  	//设置指定ADC的规则组通道，一个序列，采样时间
	ADC_RegularChannelConfig(ADC1, ch, 1, ADC_SampleTime_480Cycles );	//ADC1,ADC通道,480个周期,提高采样时间可以提高精确度			    
  
	ADC_SoftwareStartConv(ADC1);		//使能指定的ADC1的软件转换启动功能	
	 
	while(!ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC ));//等待转换结束

	return ADC_GetConversionValue(ADC1);	//返回最近一次ADC1规则组的转换结果
}
//获取通道ch的转换值，取times次,然后平均 
//ch:通道编号
//times:获取次数
//返回值:通道ch的times次转换结果平均值
u16 Get_Adc_Average(u8 ch,u8 times)
{
	u32 temp_val=0;
	u8 t;
	for(t=0;t<times;t++)
	{
		temp_val+=Get_Adc(ch);
		delay_ms(5);
	}
	return temp_val/times;
}


//通用定时器3中断初始化
//arr：自动重装值。
//psc：时钟预分频数
//定时器溢出时间计算方法:Tout=((arr+1)*(psc+1))/Ft us.
//Ft=定时器工作频率,单位:Mhz
//这里使用的是定时器3!
//void TIM3_Int_Init(u16 arr,u16 psc)
//{
//	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
//	NVIC_InitTypeDef NVIC_InitStructure;
	
//	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3,ENABLE);  ///使能TIM3时钟
//	
//  TIM_TimeBaseInitStructure.TIM_Period = arr; 	//自动重装载值
//	TIM_TimeBaseInitStructure.TIM_Prescaler=psc;  //定时器分频
//	TIM_TimeBaseInitStructure.TIM_CounterMode=TIM_CounterMode_Up; //向上计数模式
//	TIM_TimeBaseInitStructure.TIM_ClockDivision=TIM_CKD_DIV1; 
//	
//	TIM_TimeBaseInit(TIM3,&TIM_TimeBaseInitStructure);//初始化TIM3
//	
//	TIM_ITConfig(TIM3,TIM_IT_Update,ENABLE); //允许定时器3更新中断
//	TIM_Cmd(TIM3,ENABLE); //使能定时器3
	
//	NVIC_InitStructure.NVIC_IRQChannel=TIM3_IRQn; //定时器3中断
//	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=0x01; //抢占优先级1
//	NVIC_InitStructure.NVIC_IRQChannelSubPriority=0x03; //子优先级3
//	NVIC_InitStructure.NVIC_IRQChannelCmd=ENABLE;
//	NVIC_Init(&NVIC_InitStructure);
//	
//}

////定时器3中断服务函数
//void TIM3_IRQHandler(void)
//{
//	if(TIM_GetITStatus(TIM3,TIM_IT_Update)==SET) //溢出中断
//	{
//		
//	}
//	TIM_ClearITPendingBit(TIM3,TIM_IT_Update);  //清除中断标志位
//}
void Adc_Covert_Init(void)
{

	u8 sramx=1; //外部sram
	Array_ADC1Covert=mymalloc(sramx,8192);//申请2K字节
	Adc_Init();
	DMA2_Int_Init(ADC1_DR_ADDRESS,Array_ADC1Covert,4000);  ////DMA初始化：外设地址：ADC1_DR_ADDRESS 内存地址：Array_ADC1Covert 数据传输量500个

	
}	 
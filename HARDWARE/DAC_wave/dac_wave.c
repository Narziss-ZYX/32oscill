#include "dac_wave.h"
#include "math.h"
#include "malloc.h" 
#include "key.h"
#include "usart.h"

u16 *Array_Wavedata=0; //全局变量，指向波形数组
u32 Fre_Wave=1000;          //输出波形频率
/*
  cycle:一个周期内的点数
  D:储存正弦波数据的数组
*/
/********生成正弦波形输出表***********/
void SineWave_Data( u16 cycle ,u16 *D)
{
	int i;
	for( i=0;i<cycle;i++)
	{
		D[i]=(u16)((Um*sin(( 1.0*i/(cycle-1))*2*PI)+Um)*4095/3.3);
//		printf("D[%d]:%d",i,D[i]);
	}
}
/********生成锯齿波形输出表***********/
void SawTooth_Data( u16 cycle ,u16 *D)
{
	int i;
	for( i=0;i<cycle;i++)
	{
		D[i]= (u16)(Vref*1.0*i/(cycle-1)*4095/3.3);
	}
}
/********生成方波形输出表***********/
void SquareWave_Data(u16 cycle ,u16 *D)
{
	int i;
	for(i=0;i<cycle;i++)
	{
		if(i<cycle/2)
			D[i]=(u16)(Vref*4095/3.3);
		else
			D[i]=(u16)0;
	}
}
/********生成三角波输出表***********/
void TriWave_Data(u16 cycle ,u16 *D)
{
		int i;
	for(i=0;i<cycle;i++)
		{
			 if(i<cycle/2)
				D[i]= (u16)(2.0*i/(cycle-1)*4095.0/3.3);
			 else
				D[i]= (u16)(2.0*((cycle-1)-i)/(cycle-1)*4095.0/3.3);
		}
}

///******************正弦波形表***********************/
//#ifdef  Sine_WaveOutput_Enable 	
//     u16 SineWave_Value[N];		//已用函数代替
//#endif
///******************锯齿波形表***********************/
//#ifdef  SawTooth_WaveOutput_Enable
//     u16 SawToothWave_Value[N];  //已用函数代替
//#endif	
///*****************方波******************/
//#ifdef  Square_WaveOutput_Enable
//     u16 SquareWave_Value[N];  //已用函数代替
//#endif	
///******************三角波形表***********************/
//#ifdef  Tri_WaveOutput_Enable
//     u16 TriWave_Value[N];  //已用函数代替
//#endif	

/******DAC寄存器地址声明*******/	
#define DAC_DHR12R1    (u32)&(DAC->DHR12R1)   //DAC通道1输出寄存器地址
//#define DAC_DHR12R2    (u32)&(DAC->DHR12R2)   //DAC通道2输出寄存器地址



/**********DAC通道1输出初始化**********/
void Dac1_Init(void)
{  
  GPIO_InitTypeDef  GPIO_InitStructure;
	DAC_InitTypeDef DAC_InitType;
	
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);//使能GPIOA时钟
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_DAC, ENABLE);//使能DAC时钟
	   
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;//输出
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;//下拉
//  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
//  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;       //推挽输出
//  GPIO_SetBits(GPIOA,GPIO_Pin_4)	;	//拉高输出
  GPIO_Init(GPIOA, &GPIO_InitStructure);//初始化


	DAC_InitType.DAC_Trigger=DAC_Trigger_T2_TRGO;	//选择DAC触发源为TIM2
	DAC_InitType.DAC_WaveGeneration=DAC_WaveGeneration_None;//不使用波形发生
	DAC_InitType.DAC_LFSRUnmask_TriangleAmplitude=DAC_LFSRUnmask_Bit0;//屏蔽、幅值设置
	DAC_InitType.DAC_OutputBuffer=DAC_OutputBuffer_Disable ;	//DAC1输出缓存关闭 BOFF1=1
  DAC_Init(DAC_Channel_1,&DAC_InitType);	 //初始化DAC通道1

	DAC_Cmd(DAC_Channel_1, ENABLE);  //使能DAC通道1
	DAC_DMACmd(DAC_Channel_1, ENABLE); //使能DAC通道1的DMA
  
  DAC_SetChannel1Data(DAC_Align_12b_R, 0);  //12位右对齐数据格式设置DAC值
}


/*********定时器2初始化************/
void TIM2_Int_Init(u16 arr,u16 psc)
{
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
//	NVIC_InitTypeDef NVIC_InitStructure;
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2,ENABLE);  ///使能TIM2时钟
	
  TIM_TimeBaseInitStructure.TIM_Period = arr; 	//自动重装载值
	TIM_TimeBaseInitStructure.TIM_Prescaler=psc;  //定时器分频
	TIM_TimeBaseInitStructure.TIM_CounterMode=TIM_CounterMode_Up; //向上计数模式
	TIM_TimeBaseInitStructure.TIM_ClockDivision=TIM_CKD_DIV1; 
	
	TIM_TimeBaseInit(TIM2,&TIM_TimeBaseInitStructure);//初始化TIM2
	TIM_SelectOutputTrigger(TIM2, TIM_TRGOSource_Update);//设置TIM2输出触发为更新模式
	TIM_Cmd(TIM2,ENABLE); //使能定时器2
	

	
//	TIM_ITConfig(TIM2,TIM_IT_Update,ENABLE); //允许定时器2更新中断
//	NVIC_InitStructure.NVIC_IRQChannel=TIM2_IRQn; //定时器2中断
//	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=0x01; //抢占优先级1
//	NVIC_InitStructure.NVIC_IRQChannelSubPriority=0x03; //子优先级3
//	NVIC_InitStructure.NVIC_IRQChannelCmd=ENABLE;
//	NVIC_Init(&NVIC_InitStructure);
	
}


/*************DMA初始化************
	par:外设地址
	mar:内存地址
	ndtr:数据传输量
*/
void DMA1_Int_Init(u32 par,u16 *mar,u16 ndtr)
{					
	DMA_InitTypeDef            DMA_InitStructure;
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1,ENABLE);//DMA1时钟使能
  
	DMA_InitStructure.DMA_Channel = DMA_Channel_7; //通道选择DAC1	
	
//	DMA_StructInit( &DMA_InitStructure);		//DMA结构初始化
	DMA_InitStructure.DMA_DIR = DMA_DIR_MemoryToPeripheral;//存储器到外设模式
	DMA_InitStructure.DMA_BufferSize = ndtr;//缓存大小，一般为256点
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;//外设地址不递增
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;	//内存地址递增
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;//宽度为半字16位
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;//宽度为半字
	DMA_InitStructure.DMA_Priority = DMA_Priority_Medium;//中等优先级
	DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;//循环发送模式
	DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;         
  DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_Full;
	DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;//存储器突发单次传输
  DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;//外设突发单次传输

	DMA_InitStructure.DMA_PeripheralBaseAddr = par;//外设地址为DAC通道1数据寄存器
	DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)mar;//内存地址为输出波形数据数组
	DMA_Init(DMA1_Stream5,&DMA_InitStructure);  //第5流
  DMA_Cmd(DMA1_Stream5,ENABLE);    //启动DMA1的通道5    


     
}

//设置通道1输出电压
//vol:0~3300,代表0~3.3V
void Dac1_Set_Vol(u16 vol)
{
	double temp=vol;
	temp/=1000;
	temp=temp*4096/3.3;
	DAC_SetChannel1Data(DAC_Align_12b_R,temp);//12位右对齐数据格式设置DAC值
}

/*******波形初始化********
  Freq:频率
	WaveType：波形类型
*/
void Wave_Init(u32 Freq,u8 WaveType)
{
	u16 fre=(u16)(72000000/(N*2)/Freq);
	u8 sramx=1; //外部sram
	Array_Wavedata=mymalloc(sramx,2048);//申请2K字节
	switch(WaveType)           
	{
		case SineWave:
			SineWave_Data(N,Array_Wavedata);     //生成正弦波表
			break;
		case SawToothWave:        
			SawTooth_Data(N,Array_Wavedata);     //生成锯齿波表
			break;
		case SquareWave:          
			SquareWave_Data(N,Array_Wavedata);   //生成方波表
			break;
		case TriWave:            
			TriWave_Data(N,Array_Wavedata);      //生成三角波表 
		  break;
	}
	TIM2_Int_Init(fre,0);     //定时器2初始化：不分频
	Dac1_Init();    //DAC1初始化
	DMA1_Int_Init(DAC_DHR12R1,Array_Wavedata,N);   //DMA初始化：外设地址：DAC_DHR12R1 内存地址：Array_Wavedata
}


//频率步进为100HZ
/*******************频率+*******************
*/
void WaveFre_ADD()
{
  u16 reload;
	Fre_Wave+=100;
	reload=(u16)(72000000/(N*2)/Fre_Wave);
	TIM_SetAutoreload( TIM2 ,reload);
}

/*******************频率-*******************
*/
void WaveFre_SUB()
{
  u16 reload;
	Fre_Wave-=100;
	reload=(u16)(72000000/(N*2)/Fre_Wave);
	TIM_SetAutoreload( TIM2 ,reload);
}



/***************按键控制函数***************
  KEY0:改变波形
	KEY1：频率+100HZ
	KEY2：频率-100HZ
*/
void Key_Control()
{
	static int i=0;
	static u8 key;
	key=KEY_Scan(0);
	switch(key)
	{
		case 0://没有按键按下	
			break;
		case KEY0_PRES://KEY0按下
			i++;
			switch(i)
			{	
				case SineWave:
					SineWave_Data(N,Array_Wavedata);     //生成正弦波表
					break;
				case SawToothWave:        
					SawTooth_Data(N,Array_Wavedata);     //生成锯齿波表
					break;
				case SquareWave:          
					SquareWave_Data(N,Array_Wavedata);   //生成方波表
					break;
				case TriWave:            
					TriWave_Data(N,Array_Wavedata);      //生成三角波表
					i=-1;
					break;
			}
			break;
		case KEY1_PRES:    //KEY1按下	
			WaveFre_ADD();   //+100
		  break;
		case KEY2_PRES:	//KEY2按下	 
			WaveFre_SUB();   //-100
		  break;
		}	

}

void Change_Wave(u8 WaveType)
{
	DMA_Cmd(DMA1_Stream5,DISABLE);
	switch(WaveType)           
	{
		case SineWave:
			SineWave_Data(N,Array_Wavedata);     //生成正弦波表
			break;
		case SawToothWave:        
			SawTooth_Data(N,Array_Wavedata);     //生成锯齿波表
			break;
		case SquareWave:          
			SquareWave_Data(N,Array_Wavedata);   //生成方波表
			break;
		case TriWave:            
			TriWave_Data(N,Array_Wavedata);      //生成三角波表 
		  break;
	}
  DMA1_Int_Init(DAC_DHR12R1,Array_Wavedata,N);
	DMA_Cmd(DMA1_Stream5, ENABLE);  //使能DMA      
}


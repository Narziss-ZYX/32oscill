#include "dac_wave.h"
#include "math.h"
#include "malloc.h" 
#include "key.h"
#include "usart.h"

u16 *Array_Wavedata=0; //ȫ�ֱ�����ָ��������
u32 Fre_Wave=1000;          //�������Ƶ��
/*
  cycle:һ�������ڵĵ���
  D:�������Ҳ����ݵ�����
*/
/********�������Ҳ��������***********/
void SineWave_Data( u16 cycle ,u16 *D)
{
	int i;
	for( i=0;i<cycle;i++)
	{
		D[i]=(u16)((Um*sin(( 1.0*i/(cycle-1))*2*PI)+Um)*4095/3.3);
//		printf("D[%d]:%d",i,D[i]);
	}
}
/********���ɾ�ݲ��������***********/
void SawTooth_Data( u16 cycle ,u16 *D)
{
	int i;
	for( i=0;i<cycle;i++)
	{
		D[i]= (u16)(Vref*1.0*i/(cycle-1)*4095/3.3);
	}
}
/********���ɷ����������***********/
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
/********�������ǲ������***********/
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

///******************���Ҳ��α�***********************/
//#ifdef  Sine_WaveOutput_Enable 	
//     u16 SineWave_Value[N];		//���ú�������
//#endif
///******************��ݲ��α�***********************/
//#ifdef  SawTooth_WaveOutput_Enable
//     u16 SawToothWave_Value[N];  //���ú�������
//#endif	
///*****************����******************/
//#ifdef  Square_WaveOutput_Enable
//     u16 SquareWave_Value[N];  //���ú�������
//#endif	
///******************���ǲ��α�***********************/
//#ifdef  Tri_WaveOutput_Enable
//     u16 TriWave_Value[N];  //���ú�������
//#endif	

/******DAC�Ĵ�����ַ����*******/	
#define DAC_DHR12R1    (u32)&(DAC->DHR12R1)   //DACͨ��1����Ĵ�����ַ
//#define DAC_DHR12R2    (u32)&(DAC->DHR12R2)   //DACͨ��2����Ĵ�����ַ



/**********DACͨ��1�����ʼ��**********/
void Dac1_Init(void)
{  
  GPIO_InitTypeDef  GPIO_InitStructure;
	DAC_InitTypeDef DAC_InitType;
	
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);//ʹ��GPIOAʱ��
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_DAC, ENABLE);//ʹ��DACʱ��
	   
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;//���
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;//����
//  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
//  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;       //�������
//  GPIO_SetBits(GPIOA,GPIO_Pin_4)	;	//�������
  GPIO_Init(GPIOA, &GPIO_InitStructure);//��ʼ��


	DAC_InitType.DAC_Trigger=DAC_Trigger_T2_TRGO;	//ѡ��DAC����ԴΪTIM2
	DAC_InitType.DAC_WaveGeneration=DAC_WaveGeneration_None;//��ʹ�ò��η���
	DAC_InitType.DAC_LFSRUnmask_TriangleAmplitude=DAC_LFSRUnmask_Bit0;//���Ρ���ֵ����
	DAC_InitType.DAC_OutputBuffer=DAC_OutputBuffer_Disable ;	//DAC1�������ر� BOFF1=1
  DAC_Init(DAC_Channel_1,&DAC_InitType);	 //��ʼ��DACͨ��1

	DAC_Cmd(DAC_Channel_1, ENABLE);  //ʹ��DACͨ��1
	DAC_DMACmd(DAC_Channel_1, ENABLE); //ʹ��DACͨ��1��DMA
  
  DAC_SetChannel1Data(DAC_Align_12b_R, 0);  //12λ�Ҷ������ݸ�ʽ����DACֵ
}


/*********��ʱ��2��ʼ��************/
void TIM2_Int_Init(u16 arr,u16 psc)
{
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
//	NVIC_InitTypeDef NVIC_InitStructure;
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2,ENABLE);  ///ʹ��TIM2ʱ��
	
  TIM_TimeBaseInitStructure.TIM_Period = arr; 	//�Զ���װ��ֵ
	TIM_TimeBaseInitStructure.TIM_Prescaler=psc;  //��ʱ����Ƶ
	TIM_TimeBaseInitStructure.TIM_CounterMode=TIM_CounterMode_Up; //���ϼ���ģʽ
	TIM_TimeBaseInitStructure.TIM_ClockDivision=TIM_CKD_DIV1; 
	
	TIM_TimeBaseInit(TIM2,&TIM_TimeBaseInitStructure);//��ʼ��TIM2
	TIM_SelectOutputTrigger(TIM2, TIM_TRGOSource_Update);//����TIM2�������Ϊ����ģʽ
	TIM_Cmd(TIM2,ENABLE); //ʹ�ܶ�ʱ��2
	

	
//	TIM_ITConfig(TIM2,TIM_IT_Update,ENABLE); //����ʱ��2�����ж�
//	NVIC_InitStructure.NVIC_IRQChannel=TIM2_IRQn; //��ʱ��2�ж�
//	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=0x01; //��ռ���ȼ�1
//	NVIC_InitStructure.NVIC_IRQChannelSubPriority=0x03; //�����ȼ�3
//	NVIC_InitStructure.NVIC_IRQChannelCmd=ENABLE;
//	NVIC_Init(&NVIC_InitStructure);
	
}


/*************DMA��ʼ��************
	par:�����ַ
	mar:�ڴ��ַ
	ndtr:���ݴ�����
*/
void DMA1_Int_Init(u32 par,u16 *mar,u16 ndtr)
{					
	DMA_InitTypeDef            DMA_InitStructure;
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1,ENABLE);//DMA1ʱ��ʹ��
  
	DMA_InitStructure.DMA_Channel = DMA_Channel_7; //ͨ��ѡ��DAC1	
	
//	DMA_StructInit( &DMA_InitStructure);		//DMA�ṹ��ʼ��
	DMA_InitStructure.DMA_DIR = DMA_DIR_MemoryToPeripheral;//�洢��������ģʽ
	DMA_InitStructure.DMA_BufferSize = ndtr;//�����С��һ��Ϊ256��
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;//�����ַ������
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;	//�ڴ��ַ����
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;//���Ϊ����16λ
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;//���Ϊ����
	DMA_InitStructure.DMA_Priority = DMA_Priority_Medium;//�е����ȼ�
	DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;//ѭ������ģʽ
	DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;         
  DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_Full;
	DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;//�洢��ͻ�����δ���
  DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;//����ͻ�����δ���

	DMA_InitStructure.DMA_PeripheralBaseAddr = par;//�����ַΪDACͨ��1���ݼĴ���
	DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)mar;//�ڴ��ַΪ���������������
	DMA_Init(DMA1_Stream5,&DMA_InitStructure);  //��5��
  DMA_Cmd(DMA1_Stream5,ENABLE);    //����DMA1��ͨ��5    


     
}

//����ͨ��1�����ѹ
//vol:0~3300,����0~3.3V
void Dac1_Set_Vol(u16 vol)
{
	double temp=vol;
	temp/=1000;
	temp=temp*4096/3.3;
	DAC_SetChannel1Data(DAC_Align_12b_R,temp);//12λ�Ҷ������ݸ�ʽ����DACֵ
}

/*******���γ�ʼ��********
  Freq:Ƶ��
	WaveType����������
*/
void Wave_Init(u32 Freq,u8 WaveType)
{
	u16 fre=(u16)(72000000/(N*2)/Freq);
	u8 sramx=1; //�ⲿsram
	Array_Wavedata=mymalloc(sramx,2048);//����2K�ֽ�
	switch(WaveType)           
	{
		case SineWave:
			SineWave_Data(N,Array_Wavedata);     //�������Ҳ���
			break;
		case SawToothWave:        
			SawTooth_Data(N,Array_Wavedata);     //���ɾ�ݲ���
			break;
		case SquareWave:          
			SquareWave_Data(N,Array_Wavedata);   //���ɷ�����
			break;
		case TriWave:            
			TriWave_Data(N,Array_Wavedata);      //�������ǲ��� 
		  break;
	}
	TIM2_Int_Init(fre,0);     //��ʱ��2��ʼ��������Ƶ
	Dac1_Init();    //DAC1��ʼ��
	DMA1_Int_Init(DAC_DHR12R1,Array_Wavedata,N);   //DMA��ʼ���������ַ��DAC_DHR12R1 �ڴ��ַ��Array_Wavedata
}


//Ƶ�ʲ���Ϊ100HZ
/*******************Ƶ��+*******************
*/
void WaveFre_ADD()
{
  u16 reload;
	Fre_Wave+=100;
	reload=(u16)(72000000/(N*2)/Fre_Wave);
	TIM_SetAutoreload( TIM2 ,reload);
}

/*******************Ƶ��-*******************
*/
void WaveFre_SUB()
{
  u16 reload;
	Fre_Wave-=100;
	reload=(u16)(72000000/(N*2)/Fre_Wave);
	TIM_SetAutoreload( TIM2 ,reload);
}



/***************�������ƺ���***************
  KEY0:�ı䲨��
	KEY1��Ƶ��+100HZ
	KEY2��Ƶ��-100HZ
*/
void Key_Control()
{
	static int i=0;
	static u8 key;
	key=KEY_Scan(0);
	switch(key)
	{
		case 0://û�а�������	
			break;
		case KEY0_PRES://KEY0����
			i++;
			switch(i)
			{	
				case SineWave:
					SineWave_Data(N,Array_Wavedata);     //�������Ҳ���
					break;
				case SawToothWave:        
					SawTooth_Data(N,Array_Wavedata);     //���ɾ�ݲ���
					break;
				case SquareWave:          
					SquareWave_Data(N,Array_Wavedata);   //���ɷ�����
					break;
				case TriWave:            
					TriWave_Data(N,Array_Wavedata);      //�������ǲ���
					i=-1;
					break;
			}
			break;
		case KEY1_PRES:    //KEY1����	
			WaveFre_ADD();   //+100
		  break;
		case KEY2_PRES:	//KEY2����	 
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
			SineWave_Data(N,Array_Wavedata);     //�������Ҳ���
			break;
		case SawToothWave:        
			SawTooth_Data(N,Array_Wavedata);     //���ɾ�ݲ���
			break;
		case SquareWave:          
			SquareWave_Data(N,Array_Wavedata);   //���ɷ�����
			break;
		case TriWave:            
			TriWave_Data(N,Array_Wavedata);      //�������ǲ��� 
		  break;
	}
  DMA1_Int_Init(DAC_DHR12R1,Array_Wavedata,N);
	DMA_Cmd(DMA1_Stream5, ENABLE);  //ʹ��DMA      
}


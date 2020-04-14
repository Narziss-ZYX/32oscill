#ifndef __DAC_wave_H
#define __DAC_wave_H
#include "sys.h"



#define PI  3.1415926
#define Vref 2		//0.1~3.3V可调
#define Um  (Vref/2)
#define N 1000

/********不需要的波形注释掉即可**********/
//#define  Sine_WaveOutput_Enable
#define SineWave         0          
#define SawToothWave     1  
#define SquareWave       2
#define TriWave          3

void Wave_Init(u32 Freq,u8 WaveType);
void Key_Control(void);

#endif
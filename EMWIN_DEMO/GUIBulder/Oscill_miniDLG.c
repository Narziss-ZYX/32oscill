/*********************************************************************
*                                                                    *
*                SEGGER Microcontroller GmbH & Co. KG                *
*        Solutions for real time microcontroller applications        *
*                                                                    *
**********************************************************************
*                                                                    *
* C-file generated by:                                               *
*                                                                    *
*        GUI_Builder for emWin version 5.28                          *
*        Compiled Jan 30 2015, 16:41:06                              *
*        (c) 2015 Segger Microcontroller GmbH & Co. KG               *
*                                                                    *
**********************************************************************
*                                                                    *
*        Internet: www.segger.com  Support: support@segger.com       *
*                                                                    *
**********************************************************************
*/

// USER START (Optionally insert additional includes)
// USER END

#include "DIALOG.h"
#include "Oscill_miniDLG.h"
#include "malloc.h"
#include "usart.h"
#include "includes.h"

/*********************************************************************
*
*       Defines
*
**********************************************************************
*/
#define ID_FRAMEWIN_0        (GUI_ID_USER + 0x00)
#define ID_GRAPH_0        (GUI_ID_USER + 0x01)
#define ID_BUTTON_0        (GUI_ID_USER + 0x02)
#define ID_BUTTON_1        (GUI_ID_USER + 0x03)
#define ID_BUTTON_2        (GUI_ID_USER + 0x04)
#define ID_BUTTON_3        (GUI_ID_USER + 0x05)
#define ID_TEXT_0        (GUI_ID_USER + 0x06)
#define ID_BUTTON_4        (GUI_ID_USER + 0x07)
#define ID_BUTTON_5        (GUI_ID_USER + 0x08)
#define ID_TEXT_1        (GUI_ID_USER + 0x09)
#define ID_TEXT_2        (GUI_ID_USER + 0x0A)
#define ID_BUTTON_6        (GUI_ID_USER + 0x0B)
#define ID_BUTTON_7        (GUI_ID_USER + 0x0C)
#define ID_TEXT_3        (GUI_ID_USER + 0x0D)
#define ID_BUTTON_8        (GUI_ID_USER + 0x0E)
#define ID_BUTTON_9        (GUI_ID_USER + 0x0F)
#define ID_TEXT_4        (GUI_ID_USER + 0x16)
#define ID_TEXT_5        (GUI_ID_USER + 0x17)
#define ID_BUTTON_10        (GUI_ID_USER + 0x18)
#define ID_BUTTON_11        (GUI_ID_USER + 0x19)
#define ID_BUTTON_12        (GUI_ID_USER + 0x1A)
#define ID_BUTTON_13        (GUI_ID_USER + 0x1B)
#define ID_TEXT_6        (GUI_ID_USER + 0x1C)


//u8  Trig_Flag = 1;          //是否触发标志位
u16 Trig_Value = 1241;      //触发值  中间
u16 Trig_Positin = 0;         //触发后数据位置
extern u16 *Array_ADC1Covert;  //指向ADC1转化后的波形数组
extern u8  DMA2_TransferCompleteFlag; //DMA2传输完成标志位
static u16 ShowWave_Buffer[529]={0};     //绘图数据指针
static u16 ShowWave_BufTemp[1024]={0};  //临时绘图数组
static short int data_yt;
static	GRAPH_SCALE_Handle hScaleV;
//static	GRAPH_SCALE_Handle hScaleH;
static	GRAPH_DATA_Handle hData;
u16 usCurPos;    //ADC1读取的位置

/*********************************************************************
*
*       Static data
*
**********************************************************************
*/

// USER START (Optionally insert additional static data)
// USER END

/*********************************************************************
*
*       _aDialogCreate
*/
static const GUI_WIDGET_CREATE_INFO _aDialogCreate[] = {
  { FRAMEWIN_CreateIndirect, "Oscilloscope", ID_FRAMEWIN_0, 0, 0, 800, 480, 0, 0x0, 0 },
  { GRAPH_CreateIndirect, "Graph", ID_GRAPH_0, 0, 19, 529, 360, 0, 0x0, 0 },
  { BUTTON_CreateIndirect, "Run", ID_BUTTON_0, 585, 26, 80, 35, 0, 0x0, 0 },
  { BUTTON_CreateIndirect, "Stop", ID_BUTTON_1, 689, 26, 80, 35, 0, 0x0, 0 },
  { BUTTON_CreateIndirect, "Auto", ID_BUTTON_2, 585, 70, 80, 35, 0, 0x0, 0 },
  { BUTTON_CreateIndirect, "Single", ID_BUTTON_3, 689, 70, 80, 35, 0, 0x0, 0 },
  { TEXT_CreateIndirect, "Triggle:", ID_TEXT_0, 545, 135, 50, 20, 0, 0x0, 0 },
  { BUTTON_CreateIndirect, "+", ID_BUTTON_4, 698, 116, 55, 25, 0, 0x0, 0 },
  { BUTTON_CreateIndirect, "-", ID_BUTTON_5, 698, 149, 55, 25, 0, 0x0, 0 },
  { TEXT_CreateIndirect, "V", ID_TEXT_1, 655, 136, 13, 20, 0, 0x0, 0 },
  { TEXT_CreateIndirect, "Volts/Div:", ID_TEXT_2, 544, 196, 66, 20, 0, 0x0, 0 },
  { BUTTON_CreateIndirect, "+", ID_BUTTON_6, 659, 191, 50, 25, 0, 0x0, 0 },
  { BUTTON_CreateIndirect, "-", ID_BUTTON_7, 723, 191, 50, 25, 0, 0x0, 0 },
  { TEXT_CreateIndirect, "Time/Div:", ID_TEXT_3, 544, 228, 66, 20, 0, 0x0, 0 },
  { BUTTON_CreateIndirect, "+", ID_BUTTON_8, 659, 225, 50, 25, 0, 0x0, 0 },
  { BUTTON_CreateIndirect, "-", ID_BUTTON_9, 723, 225, 50, 25, 0, 0x0, 0 },
  { TEXT_CreateIndirect, "Freq:", ID_TEXT_4, 593, 304, 43, 20, 0, 0x0, 0 },
  { TEXT_CreateIndirect, "Vpp:", ID_TEXT_5, 593, 340, 43, 20, 0, 0x0, 0 },
  { BUTTON_CreateIndirect, "Button1", ID_BUTTON_10, 30, 414, 110, 45, 0, 0x0, 0 },
  { BUTTON_CreateIndirect, "Button2", ID_BUTTON_11, 199, 414, 110, 45, 0, 0x0, 0 },
  { BUTTON_CreateIndirect, "Button3", ID_BUTTON_12, 390, 418, 110, 40, 0, 0x0, 0 },
  { BUTTON_CreateIndirect, "Button4", ID_BUTTON_13, 555, 419, 110, 40, 0, 0x0, 0 },
  { TEXT_CreateIndirect, "Type:", ID_TEXT_6, 593, 266, 80, 20, 0, 0x0, 0 },
  // USER START (Optionally insert additional widgets)
  // USER END
};

/*********************************************************************
*
* 初始化窗口
*
**********************************************************************
*/

void InitDialog(WM_MESSAGE * pMsg)
{
		WM_HWIN hItem;
		//初始化'Oscilloscope'
		hItem = pMsg->hWin;
		FRAMEWIN_SetTitleVis(hItem, 0);  //隐藏标记
		// Initialization of 'Triggle:'
		hItem = WM_GetDialogItem(pMsg->hWin, ID_TEXT_0);
		TEXT_SetFont(hItem, GUI_FONT_16_1);
		// Initialization of '-'
    hItem = WM_GetDialogItem(pMsg->hWin, ID_BUTTON_5);
    BUTTON_SetFont(hItem, GUI_FONT_20_1);
    // Initialization of 'V'
    hItem = WM_GetDialogItem(pMsg->hWin, ID_TEXT_1);
    TEXT_SetFont(hItem, GUI_FONT_16_1);
    // Initialization of 'Volts/Div:'
    hItem = WM_GetDialogItem(pMsg->hWin, ID_TEXT_2);
    TEXT_SetFont(hItem, GUI_FONT_16B_ASCII);
    // Initialization of '-'
    hItem = WM_GetDialogItem(pMsg->hWin, ID_BUTTON_7);
    BUTTON_SetFont(hItem, GUI_FONT_20_1);
    // Initialization of 'Time/Div:'
    hItem = WM_GetDialogItem(pMsg->hWin, ID_TEXT_3);
    TEXT_SetFont(hItem, GUI_FONT_16B_1);
    // Initialization of '-'
    hItem = WM_GetDialogItem(pMsg->hWin, ID_BUTTON_9);
    BUTTON_SetFont(hItem, GUI_FONT_20_1);
    // Initialization of 'Freq:'
    hItem = WM_GetDialogItem(pMsg->hWin, ID_TEXT_4);
    TEXT_SetFont(hItem, GUI_FONT_20_ASCII);
    TEXT_SetTextColor(hItem, 0x00804000);
    // Initialization of 'Vpp:'
    hItem = WM_GetDialogItem(pMsg->hWin, ID_TEXT_5);
    TEXT_SetFont(hItem, GUI_FONT_20_ASCII);
    TEXT_SetTextColor(hItem, 0x00804000);
    // Initialization of 'Type:'
    hItem = WM_GetDialogItem(pMsg->hWin, ID_TEXT_6);
    TEXT_SetFont(hItem, GUI_FONT_20_ASCII);
    TEXT_SetTextColor(hItem, 0x00804000);
		//创建Graph
		hItem=WM_GetDialogItem(pMsg->hWin, ID_GRAPH_0);
		GRAPH_SetBorder(hItem,30, 0, 0, 0);   //增加左边框
		GRAPH_SetColor(hItem,GUI_WHITE,GRAPH_CI_BORDER);  //设置边框颜色为白色
		GRAPH_SetGridVis(hItem,1);  //启用网格绘制
		GRAPH_SetGridFixedX(hItem,1);  //固定x轴方向网格
		GRAPH_SetGridDistY(hItem,60);  //网格高度50
		GRAPH_SetGridDistX(hItem,100); //宽度100
		hScaleV=GRAPH_SCALE_Create(30, GUI_TA_RIGHT, GRAPH_SCALE_CF_VERTICAL, 60);//垂直方向刻度
		GRAPH_SCALE_SetTextColor(hScaleV,GUI_BLUE);  //刻度颜色
		GRAPH_AttachScale(hItem,hScaleV);   //刻度对象添加到控件
		GRAPH_SCALE_SetNumDecs(hScaleV,1);  //刻度小数点位数
		GRAPH_SCALE_SetOff(hScaleV,180);    //刻度上移180
		GRAPH_SCALE_SetFactor(hScaleV, 0.008f); //刻度换算
		hData=GRAPH_DATA_YT_Create(GUI_RED, 529, 0, 0); //创建数据对象
		GRAPH_AttachData(hItem,hData);      //添加到控件
		GRAPH_DATA_YT_SetOffY(hData,100);    //数据对象偏移80
}

/*********************************************************************
*
*       _cbDialog
*/
static void _cbDialog(WM_MESSAGE * pMsg) {
  int     NCode;
  int     Id;
	
  switch (pMsg->MsgId) {
	case WM_PAINT:
			GUI_SetBkColor(GUI_WHITE);  //设置背景色
			GUI_Clear();
	break;
  case WM_INIT_DIALOG:
			InitDialog(pMsg);     //初始化窗口
    break;
  case WM_NOTIFY_PARENT:
    Id    = WM_GetId(pMsg->hWinSrc);
    NCode = pMsg->Data.v;
    switch(Id) {
    case ID_BUTTON_0: // Notifications sent by 'Run'
      switch(NCode) {
      case WM_NOTIFICATION_CLICKED:
        // USER START (Optionally insert code for reacting on notification message)
        // USER END
        break;
      case WM_NOTIFICATION_RELEASED:
        // USER START (Optionally insert code for reacting on notification message)
        // USER END
        break;
      // USER START (Optionally insert additional code for further notification handling)
      // USER END
      }
      break;
    case ID_BUTTON_1: // Notifications sent by 'Stop'
      switch(NCode) {
      case WM_NOTIFICATION_CLICKED:
        // USER START (Optionally insert code for reacting on notification message)
        // USER END
        break;
      case WM_NOTIFICATION_RELEASED:
        // USER START (Optionally insert code for reacting on notification message)
        // USER END
        break;
      // USER START (Optionally insert additional code for further notification handling)
      // USER END
      }
      break;
    case ID_BUTTON_2: // Notifications sent by 'Auto'
      switch(NCode) {
      case WM_NOTIFICATION_CLICKED:
        // USER START (Optionally insert code for reacting on notification message)
        // USER END
        break;
      case WM_NOTIFICATION_RELEASED:
        // USER START (Optionally insert code for reacting on notification message)
        // USER END
        break;
      // USER START (Optionally insert additional code for further notification handling)
      // USER END
      }
      break;
    case ID_BUTTON_3: // Notifications sent by 'Single'
      switch(NCode) {
      case WM_NOTIFICATION_CLICKED:
        // USER START (Optionally insert code for reacting on notification message)
        // USER END
        break;
      case WM_NOTIFICATION_RELEASED:
        // USER START (Optionally insert code for reacting on notification message)
        // USER END
        break;
      // USER START (Optionally insert additional code for further notification handling)
      // USER END
      }
      break;
    case ID_BUTTON_4: // Notifications sent by '+'
      switch(NCode) {
      case WM_NOTIFICATION_CLICKED:
        // USER START (Optionally insert code for reacting on notification message)
        // USER END
        break;
      case WM_NOTIFICATION_RELEASED:
        // USER START (Optionally insert code for reacting on notification message)
        // USER END
        break;
      // USER START (Optionally insert additional code for further notification handling)
      // USER END
      }
      break;
    case ID_BUTTON_5: // Notifications sent by '-'
      switch(NCode) {
      case WM_NOTIFICATION_CLICKED:
        // USER START (Optionally insert code for reacting on notification message)
        // USER END
        break;
      case WM_NOTIFICATION_RELEASED:
        // USER START (Optionally insert code for reacting on notification message)
        // USER END
        break;
      // USER START (Optionally insert additional code for further notification handling)
      // USER END
      }
      break;
    case ID_BUTTON_6: // Notifications sent by '+'
      switch(NCode) {
      case WM_NOTIFICATION_CLICKED:
        // USER START (Optionally insert code for reacting on notification message)
        // USER END
        break;
      case WM_NOTIFICATION_RELEASED:
        // USER START (Optionally insert code for reacting on notification message)
        // USER END
        break;
      // USER START (Optionally insert additional code for further notification handling)
      // USER END
      }
      break;
    case ID_BUTTON_7: // Notifications sent by '-'
      switch(NCode) {
      case WM_NOTIFICATION_CLICKED:
        // USER START (Optionally insert code for reacting on notification message)
        // USER END
        break;
      case WM_NOTIFICATION_RELEASED:
        // USER START (Optionally insert code for reacting on notification message)
        // USER END
        break;
      // USER START (Optionally insert additional code for further notification handling)
      // USER END
      }
      break;
    case ID_BUTTON_8: // Notifications sent by '+'
      switch(NCode) {
      case WM_NOTIFICATION_CLICKED:
        // USER START (Optionally insert code for reacting on notification message)
        // USER END
        break;
      case WM_NOTIFICATION_RELEASED:
        // USER START (Optionally insert code for reacting on notification message)
        // USER END
        break;
      // USER START (Optionally insert additional code for further notification handling)
      // USER END
      }
      break;
    case ID_BUTTON_9: // Notifications sent by '-'
      switch(NCode) {
      case WM_NOTIFICATION_CLICKED:
        // USER START (Optionally insert code for reacting on notification message)
        // USER END
        break;
      case WM_NOTIFICATION_RELEASED:
        // USER START (Optionally insert code for reacting on notification message)
        // USER END
        break;
      // USER START (Optionally insert additional code for further notification handling)
      // USER END
      }
      break;
    case ID_BUTTON_10: // Notifications sent by 'Button1'
      switch(NCode) {
      case WM_NOTIFICATION_CLICKED:
        // USER START (Optionally insert code for reacting on notification message)
        // USER END
        break;
      case WM_NOTIFICATION_RELEASED:
        // USER START (Optionally insert code for reacting on notification message)
        // USER END
        break;
      // USER START (Optionally insert additional code for further notification handling)
      // USER END
      }
      break;
    case ID_BUTTON_11: // Notifications sent by 'Button2'
      switch(NCode) {
      case WM_NOTIFICATION_CLICKED:
        // USER START (Optionally insert code for reacting on notification message)
        // USER END
        break;
      case WM_NOTIFICATION_RELEASED:
        // USER START (Optionally insert code for reacting on notification message)
        // USER END
        break;
      // USER START (Optionally insert additional code for further notification handling)
      // USER END
      }
      break;
    case ID_BUTTON_12: // Notifications sent by 'Button3'
      switch(NCode) {
      case WM_NOTIFICATION_CLICKED:
        // USER START (Optionally insert code for reacting on notification message)
        // USER END
        break;
      case WM_NOTIFICATION_RELEASED:
        // USER START (Optionally insert code for reacting on notification message)
        // USER END
        break;
      // USER START (Optionally insert additional code for further notification handling)
      // USER END
      }
      break;
    case ID_BUTTON_13: // Notifications sent by 'Button4'
      switch(NCode) {
      case WM_NOTIFICATION_CLICKED:
        // USER START (Optionally insert code for reacting on notification message)
        // USER END
        break;
      case WM_NOTIFICATION_RELEASED:
        // USER START (Optionally insert code for reacting on notification message)
        // USER END
        break;
      // USER START (Optionally insert additional code for further notification handling)
      // USER END
      }
      break;
    // USER START (Optionally insert additional code for further Ids)
    // USER END
    }
    break;
  // USER START (Optionally insert additional message handling)
  // USER END
  default:
    WM_DefaultProc(pMsg);
    break;
  }
}


/*********************************************************************
*
*       Public code
*
**********************************************************************
*/
/*********************************************************************
*
*       CreateOscilloscope
*/

WM_HWIN CreateOscilloscope(void) {
  WM_HWIN hWin;

  hWin = GUI_CreateDialogBox(_aDialogCreate, GUI_COUNTOF(_aDialogCreate), _cbDialog, WM_HBKWIN, 0, 0);
  return hWin;
}

void DMA2_Stream0_IRQHandler(void)
{
	OSIntEnter();
//	static int i=0;
	
//	printf("counter:%d  ",i);
//	printf("DMA2_TransferCompleteFlag:%d\r\n",DMA2_TransferCompleteFlag);
//	i++;
	printf("DMA2_TransferCompleteFlag:%d\r\n",DMA2_TransferCompleteFlag);
	if(DMA_GetITStatus(DMA2_Stream0,DMA_IT_TCIF0))
	{	
//		ADC_DMACmd(ADC1, DISABLE);
		DMA_ClearITPendingBit(DMA2_Stream0,DMA_IT_TCIF0);
		DMA2_TransferCompleteFlag=1;
//		printf("DMA2_TransferCompleteFlag:%d\r\n",DMA2_TransferCompleteFlag);
	}

	OSIntExit();
}

void Show_Task(void)
{
	
	u16 i,j;

	WM_HWIN hWin;
  hWin=CreateOscilloscope();         //创建窗体	
	while(1)
	{
		
		if(DMA2_TransferCompleteFlag)
		{
			DMA2_TransferCompleteFlag = 0;
			usCurPos = 4000 - DMA2_Stream0->NDTR;
			if(usCurPos < 1024)
			{
				//j = 2048 - usCurPos;
				//j = 10240 - j;
				j = 2976 + usCurPos;
				
				/* 获取1k数据的前部分  */
				for(i = j; i < 4000; i++)
				{
					ShowWave_BufTemp[i-j] = *(Array_ADC1Covert+i);
				}
				
				j = 1024 - usCurPos;
				
				/* 获取1K数据的后部分 */
				for(i = 0; i < usCurPos; i++)
				{
					ShowWave_BufTemp[j+i] = *(Array_ADC1Covert+i);
				}		
			}
			else
			{	
				usCurPos = usCurPos - 1024;
					for(i=0;i<1024;i++)
				{
					ShowWave_BufTemp[i] = *(Array_ADC1Covert+i+usCurPos);
				}
			}
			
//			for(i=0;i<1000;i++)
//			{
//				ShowWave_Buffer[i] = *(Array_ADC1Covert+i);
//			}
//			 ADC_DMACmd(ADC1, ENABLE);      //使能DMA
			for(i=0; i<495; i++)            //找到满足触发条件的坐标
			{
				 if(ShowWave_BufTemp[i] < Trig_Value && ShowWave_BufTemp[i+1] > Trig_Value) //满足触发条件
				 {
					 Trig_Positin = i;
					 break;
				 }
			}
			for(i=0;i<529;i++)
			{
				ShowWave_Buffer[i] = ShowWave_BufTemp[i+Trig_Positin];
				data_yt=(I16)(ShowWave_Buffer[i]*3.3*80/4095);
				GRAPH_DATA_YT_AddValue(hData,data_yt);
			}
//    for(i=0;i<100;i++) printf("Array_ADC1Covert:%d  ",*(Array_ADC1Covert+i));

//		printf("malloc_flag:%d\r\n",malloc_flag);
		}
	
   GUI_Delay(10);
		
	}	
}


/*************************** End of file ****************************/

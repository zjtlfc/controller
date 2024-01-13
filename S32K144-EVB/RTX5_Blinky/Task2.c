#ifndef _TASK2_H_
#define _TASK2_H_


#include <stdio.h>
#include "cmsis_os2.h"                  /* ::CMSIS:RTOS2 */
#include "RTE_Components.h"             /* Component selection */
#include "Task.h"
#include "S32K144.h"
#include "S32K144_API.h"
#include "SystemConfig.h"
#include "function.h"
#include "s32_core_cm4.h"
#include "Bms-Input-Output-Port.h"
#include "string.h"
#include "arm_math.h"



#define NUM_STAGES 4
 int FILTER_ORDER = NUM_STAGES * 2;
 int split_size = 48;
 float32_t BUFFER1[50];
 float32_t BUFFER2[50];

float32_t scaleValue = 0.000061288658092324002700339302673882003f * 0.000044561811460938234303978677797175578f * 0.000020977170916845972928723232375780583f * 0.000004322274220224779011994501592974771f * 0.891250938133745562730325673328479751945f;
static float32_t state1[NUM_STAGES * 4] = {0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f};
static arm_biquad_casd_df1_inst_f32 S1;
	
static int music_threshold = 0;

void Task2(void *argument) 
{
unsigned	int	TempSize;
	(void)argument;

	int i,j,split,TempSoundValue;

		float32_t biquadCoeffs[5 * NUM_STAGES] = {
		1.0f,  2.0f,  1.0f, 1.998655678528192147780373488785699009895f,  -0.998900833160561441026459306158358231187f,
		1.0f,  2.0f,  1.0f, 1.996694713757903860695819275861140340567f, -0.996872961003747670716279571934137493372f,
		1.0f,  2.0f,  1.0f, 1.995239667538846628502824387396685779095f, -0.995323576222513994871121667529223486781f,
		1.0f,  2.0f,  1.0f, 1.994468716431189392324085929431021213531f, -0.994486005528070426962017336336430162191f,
	};  // lowpass of 120Hz
	
	arm_biquad_cascade_df1_init_f32(&S1, NUM_STAGES, biquadCoeffs, state1);
	
	for (;;) 
	{

		DISABLE_FS0_GR2;		
		osDelay(TICK_16mS);	
		ENABLE_FS0_GR2;		
   	//TempSize=InputLRData(IisLeftSoundData,SOUND_DATA_BUFF_SIZE,IisRightSoundData,SOUND_DATA_BUFF_SIZE);    //SOUND_DATA_BUFF_SIZE = 1000
		//直接从buffer获取数据,不做拷贝操作
		
		struct  CAN_MESSAGE_BUFFER TempCan0;
		ReadCanData(CanChannel0,&TempCan0);
		if (TempCan0.Data[0] != 0) {
			music_threshold = 1;
		} else {
			music_threshold = 0;
		}
		
		if (music_threshold == 1 && TempSize > 0 && IisLeftSoundData[0] != 0.0f  && IisLeftSoundData[1] != 0.0f && IisLeftSoundData[2] != 0.0f && IisLeftSoundData[3] != 0.0f) {
		//if (TempSize > 0 ) {
		#if	SOUND_DIRECT_OUTPUT == 1								// when == 0, into this if;
			memcpy(SoundLeftOutData,IisLeftSoundData,2*TempSize);
			memcpy(SoundRightOutData,IisRightSoundData,2*TempSize);
			memcpy(SoundLeftOutData1,IisLeftSoundData,2*TempSize);
			memcpy(SoundRightOutData1,IisRightSoundData,2*TempSize);
		#else	
		TempSize=768;    //16ms
		split = TempSize/split_size;
		for (j = 0; j < split; j++) {		// j from 0 to 15
			
			for (i = 0; i < split_size; i++) {		// i from 0 to 48
				BUFFER1[i] = ((int)IisLeftSoundData[i+j*split_size])-32768;    // 1.why transfer datatype to int?    2.why minus 32768?
				if (BUFFER1[i] < 0) BUFFER1[i] += 65536;					   // why?
			}
			
			arm_biquad_cascade_df1_f32(&S1, BUFFER1, BUFFER2, split_size);
			for (i = 0; i < split_size; i++) {
				BUFFER2[i] *= scaleValue;
				BUFFER2[i] += 32768;
				if (BUFFER2[i] > 65536) BUFFER2[i] -= 65536;
								
				TempSoundValue=((unsigned short)(32768+BUFFER2[i]* music_threshold))/44+88;
			  SoundOutputCH1[(SoundTopPoint + i+j*split_size) % SOUND_DRIVE_BUFF_SIZE]=TempSoundValue;
			}
		}
		
		memcpy(SoundOutputCH2,SoundOutputCH1, sizeof(unsigned short)*(SOUND_DRIVE_BUFF_SIZE+5));
		memcpy(SoundOutputCH3,SoundOutputCH1, sizeof(unsigned short)*(SOUND_DRIVE_BUFF_SIZE+5));
		memcpy(SoundOutputCH4,SoundOutputCH1, sizeof(unsigned short)*(SOUND_DRIVE_BUFF_SIZE+5));
		(void)OutputLRData(TempSize);
		#endif
		} 
	}
}

#endif


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
#include "RTX_config.h"


#define NUM_STAGES 4
 int FILTER_ORDER = NUM_STAGES * 2;
 int split_size = 48;
 float32_t BUFFER1[50];
 float32_t BUFFER2[50];
 unsigned short BUFFER3[480];
 unsigned short BUFFER4[480];

unsigned short alert_audio[] = {32767, 33196, 33625, 34053, 34482, 34910, 35338, 35765, 36192, 36618, 37044, 37469, 37893, 38316, 38738, 39160, 
	39580, 39999, 40416, 40833, 41248, 41661, 42073, 42484, 42893, 43300, 43705, 44108, 44510, 44909, 45307, 45702, 46095, 46485, 46874, 
	47260, 47643, 48024, 48402, 48778, 49151, 49521, 49888, 50252, 50613, 50972, 51327, 51679, 52027, 52373, 52715, 53053, 53388, 53720, 
	54048, 54372, 54693, 55010, 55323, 55632, 55937, 56238, 56536, 56829, 57118, 57403, 57684, 57960, 58232, 58500, 58763, 59022, 59276, 
	59526, 59772, 60012, 60248, 60479, 60706, 60928, 61144, 61357, 61564, 61766, 61963, 62155, 62342, 62525, 62702, 62873, 63040, 63202, 
	63358, 63509, 63655, 63796, 63931, 64061, 64185, 64304, 64418, 64526, 64629, 64727, 64818, 64905, 64986, 65061, 65131, 65195, 65254, 
	65307, 65355, 65397, 65433, 65464, 65490, 65509, 65523, 65532, 65535, 65532, 65523, 65509, 65490, 65464, 65433, 65397, 65355, 65307, 
	65254, 65195, 65131, 65061, 64986, 64905, 64818, 64727, 64629, 64526, 64418, 64304, 64185, 64061, 63931, 63796, 63655, 63509, 63358, 
	63202, 63040, 62873, 62702, 62525, 62342, 62155, 61963, 61766, 61564, 61357, 61144, 60928, 60706, 60479, 60248, 60012, 59772, 59526, 
	59276, 59022, 58763, 58500, 58232, 57960, 57684, 57403, 57118, 56829, 56536, 56238, 55937, 55632, 55323, 55010, 54693, 54372, 54048, 
	53720, 53388, 53053, 52715, 52373, 52027, 51679, 51327, 50972, 50613, 50252, 49888, 49521, 49151, 48778, 48402, 48024, 47643, 47260, 
	46874, 46485, 46095, 45702, 45307, 44909, 44510, 44108, 43705, 43300, 42893, 42484, 42073, 41661, 41248, 40833, 40416, 39999, 39580, 
	39160, 38738, 38316, 37893, 37469, 37044, 36618, 36192, 35765, 35338, 34910, 34482, 34053, 33625, 33196, 32767, 32338, 31909, 31481, 
	31052, 30624, 30196, 29769, 29342, 28916, 28490, 28065, 27641, 27218, 26796, 26374, 25954, 25535, 25118, 24701, 24286, 23873, 23461, 
	23050, 22641, 22234, 21829, 21426, 21024, 20625, 20227, 19832, 19439, 19049, 18660, 18274, 17891, 17510, 17132, 16756, 16383, 16013, 
	15646, 15282, 14921, 14562, 14207, 13855, 13507, 13161, 12819, 12481, 12146, 11814, 11486, 11162, 10841, 10524, 10211, 9902, 9597, 9296, 
	8998, 8705, 8416, 8131, 7850, 7574, 7302, 7034, 6771, 6512, 6258, 6008, 5762, 5522, 5286, 5055, 4828, 4606, 4390, 4177, 3970, 3768, 3571, 
	3379, 3192, 3009, 2832, 2661, 2494, 2332, 2176, 2025, 1879, 1738, 1603, 1473, 1349, 1230, 1116, 1008, 905, 807, 716, 629, 548, 473, 403, 
	339, 280, 227, 179, 137, 101, 70, 44, 25, 11, 2, 0, 2, 11, 25, 44, 70, 101, 137, 179, 227, 280, 339, 403, 473, 548, 629, 716, 807, 905, 
	1008, 1116, 1230, 1349, 1473, 1603, 1738, 1879, 2025, 2176, 2332, 2494, 2661, 2832, 3009, 3192, 3379, 3571, 3768, 3970, 4177, 4390, 4606, 
	4828, 5055, 5286, 5522, 5762, 6008, 6258, 6512, 6771, 7034, 7302, 7574, 7850, 8131, 8416, 8705, 8998, 9296, 9597, 9902, 10211, 10524, 10841, 
	11162, 11486, 11814, 12146, 12481, 12819, 13161, 13507, 13855, 14207, 14562, 14921, 15282, 15646, 16013, 16383, 16756, 17132, 17510, 17891, 
	18274, 18660, 19049, 19439, 19832, 20227, 20625, 21024, 21426, 21829, 22234, 22641, 23050, 23461, 23873, 24286, 24701, 25118, 25535, 25954, 
26374, 26796, 27218, 27641, 28065, 28490, 28916, 29342, 29769, 30196, 30624, 31052, 31481, 31909, 32338};

float32_t scaleValue = 0.000061288658092324002700339302673882003f * 0.000044561811460938234303978677797175578f * 0.000020977170916845972928723232375780583f * 0.000004322274220224779011994501592974771f * 0.891250938133745562730325673328479751945f;
static float32_t state1[NUM_STAGES * 4] = {0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f};
static arm_biquad_casd_df1_inst_f32 S1;

	
static int music_threshold = 0;
static int vibra_threshold = 0;
static int alert_threshold = 0;
	
void Task2(void *argument) 
{
unsigned	int	TempSize;
	(void)argument;

	int i,j,split;

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
   	TempSize=InputLRData(IisLeftSoundData,SOUND_DATA_BUFF_SIZE,IisRightSoundData,SOUND_DATA_BUFF_SIZE);    //SOUND_DATA_BUFF_SIZE = 1000
		
		struct  CAN_MESSAGE_BUFFER TempCan0;
		ReadCanData(CanChannel0,&TempCan0);
		if (TempCan0.Data[0] != 0) {
			music_threshold = 1;
		} else {
			music_threshold = 0;
		}
		
		if (TempCan0.Data[1] == 1) {
			vibra_threshold = 1;
		} else if(TempCan0.Data[1] == 2){
			vibra_threshold = 2;
		} else {
			vibra_threshold = 0;
		}
		
		if (TempCan0.Data[2] != 0) {
			alert_threshold = 1;
		} else {
			alert_threshold = 0;
		}
		
		SysTickTime=osKernelGetTickCount() / ( OS_TICK_FREQ / 100) ;      /*ÏµÍ³¼ì²â½ÚÅÄÊ±¼ä,¾«¶È10mS*/  
    unsigned int SystemTimes1 = SysTickTime / 30;
		unsigned int SystemTimes2 = SysTickTime / 20;
		
		// ÒôÀÖÂÉ¶¯Âß¼­
		if (music_threshold == 1 && TempSize > 0 && IisLeftSoundData[0] != 0.0f  && IisLeftSoundData[1] != 0.0f && IisLeftSoundData[2] != 0.0f && IisLeftSoundData[3] != 0.0f) {
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
				
				if (BUFFER2[i] > (65536.0f - 8000.0f) || BUFFER2[i] < 8000.0f) {
					BUFFER2[i] = 0.0f;
				} 
				SoundLeftOutData[i+j*split_size] = BUFFER2[i] * music_threshold;
				SoundRightOutData[i+j*split_size] = BUFFER2[i] * music_threshold;
			}	
		}	
		(void)OutputLRData(SoundLeftOutData,TempSize,SoundRightOutData,TempSize);
		}
		
		
		// ×ùÒÎ°´Ä¦Âß¼­1
		if (music_threshold == 0 && vibra_threshold == 1) {
			int coef = 0;
			int coef1 = 0;
			if (SystemTimes1 % 12 == 0 || SystemTimes1 % 12 == 2 || SystemTimes1 % 12 == 4) {
				coef = 1;
				coef1 = 0;
			}
			if (SystemTimes1 % 12 == 6 || SystemTimes1 % 12 == 8 || SystemTimes1 % 12 == 10) {
				coef = 0;
				coef1 = 1;
			}
			TempSize=480;
			for (int index = 0; index < TempSize; index++) {
				BUFFER3[index] = alert_audio[index] * coef;
				BUFFER4[index] = alert_audio[index] * coef1;
			}
			memcpy(SoundLeftOutData,BUFFER3,2*TempSize);
			memcpy(SoundRightOutData,BUFFER4,2*TempSize);
			(void)OutputLRData(SoundLeftOutData,TempSize,SoundRightOutData,TempSize);
		}
		
		
		
		// ×ùÒÎ°´Ä¦Âß¼­2
		if (music_threshold == 0 && vibra_threshold == 2) {
			int coef = 1;
			if (SystemTimes2 % 55 == 5 || SystemTimes2 % 55 == 11 || SystemTimes2 % 55 == 17 || SystemTimes2 % 55 == 23 || SystemTimes2 % 55 == 54) {
				coef = 0;
			}
			TempSize=480;
			for (int index = 0; index < TempSize; index++) {
				BUFFER3[index] = alert_audio[index] * coef;
			}
			memcpy(SoundLeftOutData,BUFFER3,2*TempSize);
			memcpy(SoundRightOutData,BUFFER3,2*TempSize);
			(void)OutputLRData(SoundLeftOutData,TempSize,SoundRightOutData,TempSize);
		}
		
		
		// ¾¯Ê¾ÒôÂß¼­
		if (music_threshold == 0 && vibra_threshold == 0 && alert_threshold == 1) {
			TempSize=480;
			memcpy(SoundLeftOutData,alert_audio,2*TempSize);
			memcpy(SoundRightOutData,alert_audio,2*TempSize);
			(void)OutputLRData(SoundLeftOutData,TempSize,SoundRightOutData,TempSize);
			
		}
	}
}


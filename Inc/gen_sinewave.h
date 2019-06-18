/*
 * gen_sinewave.h
 *
 *  Created on: Apr 12, 2019
 *      Author: bum
 */
//
#ifndef GEN_SINEWAVE_H_
#define GEN_SINEWAVE_H_



#include <math.h>
#include "stm32l4xx.h"
#include <stdint.h>
#include "arm_math.h"

#include "main.h"


#define AUDIO_FREQ 8000
#define LOOKUP_MAXSIZE 8000
#define MAX12BIT 4095
#define MAX12BIT_2 (MAX12BIT>>1)
#define MAX8BIT 255
#define MAX8BIT_2 (MAX8BIT>>1)
#define MAX16BIT  65535
#define MAX16BIT_2 (MAX16BIT>>1)


extern int16_t lookup[2*LOOKUP_MAXSIZE];
extern int16_t prepare[2*LOOKUP_MAXSIZE];
extern int16_t sampleShow;

typedef struct
{
	float32_t amp;
	uint16_t freq;
	uint16_t* data;
	uint32_t sampleNum;
}SineWave;

typedef enum
{
	RangeValid 					= 	0,
	SigmaFail 					= 	1,
	SignalFail 					= 	2,
	RangeValidMinRangeClipped	= 	3,
	OutOfBoundsFail 			= 	4,
    HardwareFail              	=   5,
    RangeValidNoWrapCheckFail 	=   6,
    WrapTargetFail            	=   7,
	ProcessingFail            	=   8,
    XtalkSignalFail           	=   9,
    SynchronizationInt          =  10,
    MinRangeFail              	=  13,
    None                      	= 255,
} RangeStatus;
typedef struct
{
  uint16_t range_mm;
  RangeStatus range_status;
  float peak_signal_count_rate_MCPS;
  float ambient_count_rate_MCPS;
} RangingData;

typedef struct
{
    uint8_t range_status;
    uint16_t dss_actual_effective_spads_sd0;
    uint16_t ambient_count_rate_mcps_sd0;
    uint16_t final_crosstalk_corrected_range_mm_sd0;
    uint16_t peak_signal_count_rate_crosstalk_corrected_mcps_sd0;
} ResultBuffer;


typedef SineWave *SineWaveHandler;

//SineWave sin;
//SineWaveHandler hsin = &sin;

void SineWave_init(SineWaveHandler hsin);
void SineWave_generate(SineWaveHandler hsin, RangingData *dataFreq, RangingData *dataAmp);
//void SineWave_adjustFreq(SineWaveHandler hsin,  TIM_HandleTypeDef *htim);


#endif

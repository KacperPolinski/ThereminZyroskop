

#include "gen_sinewave.h"

#define RANGE_MAX(R, L, H) R<L ? L : R<H ? R : H
int16_t lookup[2*LOOKUP_MAXSIZE];
int16_t prepare[2*LOOKUP_MAXSIZE]={0};

void SineWave_init(SineWaveHandler hsin)
{
	hsin->amp = 1.0;
	hsin->freq = 1000;
}

void SineWave_generate(SineWaveHandler hsin, float freq, float amp)
{
	hsin->freq = freq;
	hsin->amp = amp;
	float32_t step = 2.0*PI/12000;
	hsin->sampleNum = 12000;
	float32_t pos = 0;
	float32_t sample;
	for(uint32_t i=0; i<hsin->sampleNum; i+=2)
	{
		sample = hsin->amp*((arm_sin_f32(hsin->freq*pos))*(INT16_MAX));
		prepare[i]= (uint16_t)sample;
		prepare[i+1] = prepare[i];
		pos+=step;
	}
	hsin->data = prepare;
}




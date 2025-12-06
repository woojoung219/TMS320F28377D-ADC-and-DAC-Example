//	파일이름		:	SGEN_1CH_F32_TMU.c
//	대상장치		:	TMS320F28x
//	파일버전		:	0.90
//	갱신이력		:	2024-07-23,		Version 0.90


#include "SGEN_1CH_F32_TMU.h"		// Sine 신호 생성 모듈 헤더파일
#include <math.h>					// C표준 수학 함수용 헤더파일


// Sine 값 계산 함수
void sine_1ch_f32_tmu_calc(SGEN_1CH_F32_TMU *v)
{
	float tmp;

	// 산출 함수 호출 주기와 설정된 주파수에 따라 스탭 값 설정 후, Sine 값 계산
#ifdef __TMS320C28XX_TMU__
	v->SineStep = __divf32(TWOPI, __divf32(v->IsrFreq, v->SineFreq));
	tmp = (__sin(v->SineIndex) * v->SineGain) + v->SineOffset;
#else
	v->SineStep = TWOPI / (v->IsrFreq / v->SineFreq);
	tmp = (sin(v->SineIndex) * v->SineGain) + v->SineOffset;
#endif

	// Sine 산출 값에 대한 최대, 최소 값 제한 설정
	v->SineOut = (tmp > v->SineOutMax) ? v->SineOutMax : tmp;
	v->SineOut = (tmp < v->SineOutMin) ? v->SineOutMin : tmp;
	
	// 다음 호출 시, 연속된 값을 계산할 수 있도록 인덱스 값을 스탭 값 만큼 증가시킴 (2*PI 범위 내에서 제한)
	tmp = v->SineIndex + v->SineStep;
	v->SineIndex = (tmp >= TWOPI) ? (tmp - TWOPI) : tmp;
}

// Sine 값 계산용 인덱스 초기화 함수
void sine_1ch_f32_tmu_sync(SGEN_1CH_F32_TMU *v)
{
	v->SineIndex = 0.0;
}


// 파일 끝.


//	파일이름		:	SGEN_1CH_F32_TMU.h
//	대상장치		:	TMS320F28x
//	파일버전		:	0.90
//	갱신이력		:	2024-07-23,		Version 0.90

#ifndef __SGEN_1CH_F32_TMU_H__
#define __SGEN_1CH_F32_TMU_H__


#include <math.h>	// C표준 수학함수 헤더파일 (RTS 라이브러리에 포함)


// FLASH 메모리에서 코드 실행 시, Sine 값 산출 함수를 램 영역으로 복사해서 고속 실행하도록 함
#ifdef _FLASH
    #ifdef __TI_COMPILER_VERSION__
        #if __TI_COMPILER_VERSION__ >= 15009000
            #pragma CODE_SECTION(sine_1ch_f32_tmu_calc,	".TI.ramfunc");
			#pragma CODE_SECTION(sine_1ch_f32_tmu_sync,	".TI.ramfunc");
        #else
            #pragma CODE_SECTION(sine_1ch_f32_tmu_calc,	"ramfuncs");
			#pragma CODE_SECTION(sine_1ch_f32_tmu_sync,	"ramfuncs");
        #endif
    #endif
#endif

#ifndef NULL
#define NULL		0	// NULL 값 정의
#endif

#define	PI			3.1415926	// PI 값 정의
#define	TWOPI		6.2831853	// 2*PI 값 정의


// Sine 신호 생성 모듈 구조 정의 - Sgen_1Ch_f32_TMU
typedef struct {  float			SineFreq;
				  float			IsrFreq;
				  float			SineStep;
				  float			SineOut;
				  float			SineOutMax;
				  float			SineOutMin;
				  float			SineIndex;
				  float			SineGain;
				  float			SineOffset;
				  void			(*calc)();	// Pointer to sine out calculate function
				 } SGEN_1CH_F32_TMU;	            


typedef SGEN_1CH_F32_TMU	*SGEN_1CH_F32_TMU_handle;


// Sine 신호 생성 모듈 기본 값 정의
#define SGEN_1CH_F_TMU_DEFAULTS	{	60.0, \
									100E3, \
									0.0, \
									0.0, \
									0.99, \
									0.0, \
									0.0, \
									0.49, \
									0.5, \
									(void (*)(Uint32))sine_1ch_f32_tmu_calc }


// Sine 값 산출 함수 원형 선언 - SGEN_1CH_FLOAT_TMU.c
void sine_1ch_f32_tmu_calc(SGEN_1CH_F32_TMU_handle);
void sine_1ch_f32_tmu_sync(SGEN_1CH_F32_TMU_handle);


#endif	// __SGEN_1CH_F32_TMU_H__


// 파일 끝.


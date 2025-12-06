// 파일이름	:	LAB2_ADC_CPU1.c
// 대상장치	:	TR2837xD 트레이닝 키트
// 파일버전	:	1.00
// 갱신이력	:	2024-07-24, 버전 1.00
// 예제설명	:

//************************************************************************************************************************************************************************
//
// 본 예제는 TMS320F2837xD PTP 칩이 실장된 TR2837xD 트레이닝 키트를 대상으로 하고 있으며,
// F2837x 시스템 설계 강좌 2장 ADC, DAC 실습 예제용 메인 소스파일로 사용됩니다.
//
// a). Dual-Core로 구성된 TMS320F2837xD 칩의 CPU1 코어용 프로젝트 입니다.
// b). CPU2가 DAC-A 모듈로 아날로그 정현파 신호를 출력할 수 있도록, DAC-A 모듈의 제어권을 CPU2로 이양합니다.
//     (DAC 운용 - CPU2 / DAC 출력 신호 주파수 변경 - CPU1)
// c). ADC-A 모듈을 초기화하고, 아래 채널에 대한 변환 결과를 각각의 버퍼 공간(Array)에 저장합니다.
//     - Sampling Frequency : 1kHz (EPWM2 SOCA)
//     - ADCINA2	: TR2837xD 가변 전압출력 회로(Potentiometer)와 연결된 ADC 입력 채널
//     - ADCINA12	: DAC-A 모듈의 출력이 F2837x 칩 내부적으로 연결된 ADC 입력 채널 (DACOUTA --> ADCINA12 / Internal Connection)
// d). ADC-A 모듈의 PPB 1번을 초기화하여, SOC0에 의한 ADCINA2 채널의 변환 결과가 미리 설정된 최대/최소 값을 넘을 경우, CPU에 인터럽트를 요청하도록 합니다.
// e). ADC-A 모듈의 PPB 4번을 초기화하여, SOC1에 의한 ADCINA2 채널의 변환 결과에 Offset을 보정합니다.
// f). TR2837xD 트레이닝 키트, 로터리 엔코더 노브 스위치의 Active High 신호를 GPIO 23번 포트로 입력 받아 외부 인터럽트 1 회로에 전달하고,
//     외부 인터럽트 1 회로를 통해 신호의 Rising(Positive) Edge 검출 시, CPU에 인터럽트를 요청하여 CPU1이 CPU2에 전달하는 정현파 주파수 지령 값에 대한
//     Frequency Sweeping을 시작합니다. (주파수 지령 값 변수 : SineOutFrequency)
//
// 예제 폴더에는 Expressions 탭에 관찰할 변수들을 등록하고, Graph 창을 열기 위해 미리 저장된 설정파일이 포함되어 있습니다.
// --> Expressions 탭 변수 등록용 파일 : LAB2_ADC_Expressions.txt
// --> Buffer_A2[ ] 배열 데이터 관찰용 시간축 Graph 창 설정 : Buffer_A2_Array_Graph_Properties.graphProp
// --> Buffer_A12[ ] 배열 데이터에 대한 FFT Magnitude 관찰용 Graph 창 설정 : Buffer_A12_Array_FFTmagGraph_Properties.graphProp
//
// 본 예제는 28X 칩 MMR(Memory Mapped Register)을 직접 조작하는 TI의 Bit-Field Approach 기반 코드들과
// API Driver Library 기반 코드들을 모두 사용할 수 있도록 제작되었습니다. (Driverlib + Bit-Field Example)
//
//************************************************************************************************************************************************************************


// 헤더 파일들
#include "F28x_Project.h"		// TI 제공 칩-지원 헤더 통합 Include 용 헤더파일
#include "driverlib.h"			// TI 제공 Driver API Library 헤더파일 (driverlib)
#include "device.h"
#include "SGEN_1CH_F32_TMU.h"   // Sine 값 산출 모듈의 헤더파일


//  전처리 정의
#define DLOG_SIZE			1024	// 데이터 저장용 배열 크기
#define	SINE_FREQ_MAX		5E3		// 5kHz, DAC 출력에 실어낼 Sine 파형의 최대 주파수 (DAC 운용 - CPU2 / DAC 출력 신호 주파수 변경 - CPU1)
#define SINE_FREQ_MIN		1E2		// 100Hz, DAC 출력에 실어낼 Sine 파형의 최소 주파수
#define FREQ_SWEEPING_STEP	1E2		// 주파수 스위핑 단위

#define CPUFREQ_MHZ     200E6   // 200MHz, 28X Core(CPU)의 동작 클럭 주파수
#define SAMPLINGFREQ    200E3   // 200kHz, DAC 출력용 Sine 파형 샘플링 주파수
#define SINEFREQMAX     5E3     // 5kHz, DAC 출력에 실어낼 Sine 파형의 최대 주파수
#define SINEFREQMIN     1E2     // 100Hz, DAC 출력에 실어낼 Sine 파형의 최소 주파수
#define SWEEPINGSTEP    1E2     // 주파수 스위핑 단위


//  함수 선언

void InitAdcaModule(void);			// ADC-A 모듈 초기화 함수
__interrupt void AdcaIsr(void);		// ADC-A 모듈 인터럽트 서비스 루틴
void InitDacaModule(void);              // DAC-A 모듈 초기화 함수
__interrupt void CpuTimer0_ISR(void);   // CpuTimer 0번 인터럽트 서비스 루틴(ISR)



//  변수 선언
Uint32	BackTickerCpu1;				// Idle-loop 실행횟수 계수용 SW 카운터
Uint32	Xint1IsrTicker;				// XINT1 인터럽트 서비스 루틴 호출횟수 계수용 S/W 카운터

float32	SineOutFrequency;			// Sine 생성 주파수 값 (CPU1이 값 변경, CPU2가 참조)

Uint32	AdcIsrTicker;				// ADC 인터럽트 서비스 루틴 실행횟수 계수용 SW 카운터
Uint32	AdcPpbIsrTicker;			// ADC PPB 인터럽트 서비스 루틴 실행횟수 계수용 SW 카운터
Uint32	AdcPpbTripHigh;				// ADC PPB Trip High 인터럽트 발생 횟수 계수용 SW 카운터
Uint32	AdcPpbTripLow;				// ADC PPB Trip Low 인터럽트 발생 횟수 계수용 SW 카운터

int16	AdcResultA0;				// ADC-A 모듈 ADCINA0 채널 변환결과 저장용 변수 (ADC-A SOC0)
int16   AdcResultA1;                // ADC-A 모듈 ADCINA1 채널 변환결과 저장용 변수 (ADC-A SOC0)

int16	Buffer_A0[DLOG_SIZE];		// CCS 그래프 창 관찰용 데이터 저장 공간
int16	Buffer_A1[DLOG_SIZE];		// CCS 그래프 창 관찰용 데이터 저장 공간
Uint16	BufferPointer;

// Sine 파형 생성 모듈 정의
SGEN_1CH_F32_TMU sgen1chf = SGEN_1CH_F_TMU_DEFAULTS;
// 메인 함수
void main(void)
{
	Uint16 i;

//	1. 전역 인터럽트 스위치 OFF, CPU 인터럽트 벡터 비-활성화 및 플래그(Flag) 비트 클리어
	DINT;			// 전역 인터럽트 스위치 OFF (/INTM OFF)
	IER = 0x0000;	// CPU 인터럽트 벡터 비-활성화
	IFR = 0x0000;	// CPU 인터럽트 플래그 클리어


//	2. 시스템 초기화 - InitSysCtrl( ) 함수 호출 (F2837xD_SysCtrl.c)
//	* 왓치독 타이머 비-활성화
//	* CPU 클럭 주파수 설정 (PLL)
//	* 주변회로 클럭 공급 설정
	InitSysCtrl();



//	4. 주변회로 인터럽트 확장회로 초기화 - InitPieCtrl( ) 함수 호출 (F2837xD_PieCtrl.c)
	InitPieCtrl();


//	5. 주변회로 인터럽트 벡터 확장 및 복사 실행 - InitPieVectTable( ) 함수 호출 (F2837xD_PieVect.c)
	InitPieVectTable();


//	6. 인터럽트 벡터와 인터럽트 서비스 루틴 재-연결, 인터럽트 벡터 활성화
	EALLOW;
	PieVectTable.ADCA1_INT = &AdcaIsr;			// ADCA1_INT 인터럽트 벡터 재-설정 (Re-mapping)
    PieVectTable.TIMER0_INT = &CpuTimer0_ISR;   // CpuTimer0 인터럽트 벡터 재-설정 (Re-mapping)

	EDIS;

	PieCtrlRegs.PIEIER1.bit.INTx1 = 1;	// ADCA1_INT 인터럽트 Enable : PIE 그룹 1.1
    PieCtrlRegs.PIEIER1.bit.INTx7 = 1;  // TIMER0_INT 인터럽트 Enable : PIE 그룹 1.7

	IER |= (M_INT1);			// CPU 인터럽트 벡터 1번, 10번 Enable : PIE 그룹 1.x & 10.x
	//Q. CPU 인터럽트 벡터 1번을 ADCA1_INT와 CpuTimer0용으로 같이 써도 될까?


//	7. 주변회로 초기화

	// DAC-A 모듈 초기화
    InitDacaModule();

	// ADC-A 모듈 초기화 함수 실행
	InitAdcaModule();

	// CpuTimer 0번 계수 시작
    InitCpuTimers();
    //0.5Hz 주기로 Timer Interrupt 걸어줌
    //ConfigCpuTimer(&CpuTimer0, CPU 클럭주파수(MHz), timer주기(microsec)) (1000000.0 / SAMPLINGFREQ)
    ConfigCpuTimer(&CpuTimer0, (CPUFREQ_MHZ / 1000000), (1000000.0 / SAMPLINGFREQ));
    StartCpuTimer0();

    // Sine 값 산출 모듈 초기화
    sine_1ch_f32_tmu_sync(&sgen1chf);
    sgen1chf.SineFreq = SINEFREQMIN;                            // Sine 주파수 초기 값 설정
    sgen1chf.IsrFreq = SAMPLINGFREQ;                            // Sine 값 산출 모듈이 호출되는 주기 설정
    sgen1chf.SineStep = TWOPI / (SAMPLINGFREQ / SINEFREQMIN);   // Sine 값 산출용 인덱스 값 설정
    sgen1chf.SineGain = 0.49;   // Sine 신호의 이득(Gain) 값 설정
    sgen1chf.SineOffset = 0.50; // Sine 신호의 옵셋(Offset) 값 설정
    sgen1chf.SineOutMax = 0.99; // Sine 신호 출력의 최대 값 제한
    sgen1chf.SineOutMin = 0.00; // Sine 신호 출력의 최소 값 제한


//	8. 전역 변수 및 S/W 모듈 초기화
	BackTickerCpu1 = 0;
	Xint1IsrTicker = 0;

	SineOutFrequency = SINE_FREQ_MIN;

	AdcIsrTicker = 0;
	AdcPpbIsrTicker = 0;
	AdcPpbTripHigh = 0;
	AdcPpbTripLow = 0;
	AdcResultA0 = 0;
    AdcResultA1 = 0;



	for(i = 0 ; i < DLOG_SIZE ; i++)
	{
		Buffer_A0[i] = 0;
		Buffer_A1[i] = 0;
	}

	BufferPointer = 0;


//	9. 실시간 디버깅 활성화, 전역 인터럽트 스위치 ON
	ERTM;	// Debug Enable Mask 비트 설정 (실시간 디버깅이 가능하도록 ST1 레지스터의 /DBGM 비트를 0으로 클리어)
	EINT;	// 전역 인터럽트 스위치 ON (/INTM ON)


//	10. Background(Idle)-Loop
	for(;;)
	{
		// Background(Idle)-loop 호출횟수 확인용 카운터
	    BackTickerCpu1++;
	}
}


//	11. 인터럽트 서비스 루틴 및 기타 함수들

//	ADC-A 모듈 인터럽트 서비스 루틴
__interrupt void AdcaIsr(void)
{
	// ADC 인터럽트 호출횟수 계수용 SW 카운터
	AdcIsrTicker++;

	// ADC 변환 결과 이력을 CCS 그래프 창으로 관찰하기 위해, 데이터 저장
	AdcResultA0 = AdcaResultRegs.ADCRESULT0;
	AdcResultA1 = AdcaResultRegs.ADCRESULT1;

	Buffer_A0[BufferPointer] = AdcResultA0;
	Buffer_A1[BufferPointer++] = AdcResultA1;

	// 결과 저장용 배열의 끝까지 저장이 완료되었다면, BufferPointer 변수를 다시 0으로 초기화
	if(BufferPointer >= DLOG_SIZE)
	{
		BufferPointer = 0;
	}

	AdcaRegs.ADCINTFLGCLR.bit.ADCINT1 = 1;	// ADC-A 모듈의 ADCINT1 Flag Clear
	PieCtrlRegs.PIEACK.bit.ACK1 = 1;		// 주변회로 인터럽트 확장회로(PIE)의 그룹 1 Acknowledge Bit Clear
}


//	ADC-A 모듈 초기화 함수
void InitAdcaModule(void)
{
	// ADC-A 모듈 클럭 및 분해능 설정 후 기동(Power-up)
	EALLOW;
	AdcaRegs.ADCCTL2.bit.PRESCALE = 6;		// ADC-A CLK = 50MHz @ 200MHz SYSCLK (/4)
	AdcSetMode(ADC_ADCA, ADC_RESOLUTION_12BIT, ADC_SIGNALMODE_SINGLE); // ADC-A 모듈 초기화, 12비트 분해능, Single-ended 모드
	AdcaRegs.ADCCTL1.bit.INTPULSEPOS = 1;	// ADC-A 모듈 인터럽트 요청 시점 설정 : 변환 완료 후 CPU에 인터럽트 요청 (EOC)
	AdcaRegs.ADCCTL1.bit.ADCPWDNZ = 1;		// ADC-A 모듈 Power-up
	DELAY_US(1000);							// ADC 모듈의 정상 기동을 위한 지연시간 : 약 1msec
	EDIS;

	// ADC-A 모듈 변환 관련 설정 (SOC0, SOC1, SOC2 사용 / PPB4, PPB1 사용)
	EALLOW;

	// SOC0 : ADCINA0 채널 변환(DAC-A) / Acquisition Window Size : 75nsec / Trigger Source : EPWM2 SOCA
	// -------------------------------------------------------------------------------------------
	AdcaRegs.ADCSOC0CTL.bit.CHSEL = 0;
	AdcaRegs.ADCSOC0CTL.bit.ACQPS = 14;//ADC-A 모듈 SOC0 S/H 시간(신호 샘플링 시간): (14 + 1) SYSCLK(=20MHz) = 75nsec
	AdcaRegs.ADCSOC0CTL.bit.TRIGSEL = 7;
	// -------------------------------------------------------------------------------------------

	// SOC1 : ADCINA1 채널 변환(DAC-B) / Acquisition Window Size : 75nsec / Trigger Source : EPWM2 SOCA
	// -------------------------------------------------------------------------------------------
	AdcaRegs.ADCSOC1CTL.bit.CHSEL = 1;
	AdcaRegs.ADCSOC1CTL.bit.ACQPS = 14;
	AdcaRegs.ADCSOC1CTL.bit.TRIGSEL = 7;
	// -------------------------------------------------------------------------------------------

	AdcaRegs.ADCINTSEL1N2.bit.INT1SEL = 1;	// SOC1 변환 완료 후 ADCINT1 인터럽트 요청 (EOC2 is trigger for ADCINT1)
	AdcaRegs.ADCINTSEL1N2.bit.INT1E = 1;	// ADCINT1 인터럽트 Enable
	AdcaRegs.ADCINTFLGCLR.bit.ADCINT1 = 1;	// ADCINT1 Flag Clear

	EDIS;

	// ADC-A 모듈에 변환시작 신호(Start-Of-Conversion)를 전달하기 위한 EPWM2 모듈 설정 (Time-Base, Event-Trigger 서브모듈)
	EPwm2Regs.TBCTL.bit.CTRMODE = 0;	// 상승계수 모드 (Count Up / Asymmetric)

	// EPWM2 모듈 16비트 타이머 카운터가 사용할 클럭 주파수와 주기 값 설정 : 1kHz
	// -------------------------------------------------------------------------------------------
	EPwm2Regs.TBCTL.bit.HSPCLKDIV = 1;
	EPwm2Regs.TBCTL.bit.CLKDIV = 0;//100MHz->50MHz
	EPwm2Regs.TBPRD = 49999;
	// -------------------------------------------------------------------------------------------

	//Time base submodule에서 설정한 counter가 0이 될 때마다 SOC신호 생성
	EPwm2Regs.TBCTR = 0;				// 타이머 카운터 레지스터를 0으로 초기화
	EPwm2Regs.ETPS.bit.SOCPSSEL = 0;	// EPWMxSOC A/B에 대한 이벤트 분주 레지스터 선택 (0: ETPS / 1: ETSOCPS)
	EPwm2Regs.ETSEL.bit.SOCAEN = 1;		// EPWM2SOCA 활성화
	EPwm2Regs.ETSEL.bit.SOCASEL = 1;	// 타이머 카운터가 0과 일치할 때, EPWM2SOCA 신호 생성
	EPwm2Regs.ETPS.bit.SOCAPRD = 1;		// SOCASEL에서 선택된 매 이벤트 마다 SOC 신호 생성 (/1)
	EPwm2Regs.ETCLR.bit.SOCA = 1;		// EPWM2SOCA Flag Clear
}

// CpuTimer 0번 인터럽트 서비스 루틴(CpuTimer 0 ISR) - 200kHz (5usec)
__interrupt void CpuTimer0_ISR(void)
{
    CpuTimer0.InterruptCount++;

    // DAC-A 출력에 실어낼 Sine 값 계산 함수 호출
    sgen1chf.calc(&sgen1chf);

    // 계산된 Sine 값을 DAC-A 모듈 출력에 반영
    DacaRegs.DACVALS.bit.DACVALS = 4096 * sgen1chf.SineOut;
    DacbRegs.DACVALS.bit.DACVALS = 4096 * sgen1chf.SineOut;

    // TIMER0_INT 인터럽트 벡터가 포함된 CPU 인터럽트 확장그룹 1번의 Acknowledge 비트 클리어
    PieCtrlRegs.PIEACK.bit.ACK1 = 1;
}


// DAC-A 모듈 초기화 함수
void InitDacaModule(void)
{
    EALLOW;
    DacaRegs.DACCTL.bit.DACREFSEL = 1;  // ADC-A 모듈의 VREFHI 전압을 DAC-A 모듈의 참조전압으로 사용
    DacaRegs.DACOUTEN.bit.DACOUTEN = 1; // DAC-A 출력 활성화
    DacaRegs.DACVALS.all = 0;           // DAC-A 출력 전압 초기화 (0.0V)
    DELAY_US(10L);                      // DAC 활성화 후, 필요한 지연 (10usec)

    DacbRegs.DACCTL.bit.DACREFSEL = 1;  // ADC-B 모듈의 VREFHI 전압을 DAC-B 모듈의 참조전압으로 사용
    DacbRegs.DACOUTEN.bit.DACOUTEN = 1; // DAC-B 출력 활성화
    DacbRegs.DACVALS.all = 0;           // DAC-B 출력 전압 초기화 (0.0V)
    DELAY_US(10L);                      // DAC 활성화 후, 필요한 지연 (10usec)
    EDIS;
}



// 파일 끝.

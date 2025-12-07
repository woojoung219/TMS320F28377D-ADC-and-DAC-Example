# TMS320F28377D-ADC-and-DAC-Example
This is TMS320F28377D ADC and DAC Example.

1) DAC-A, B 모듈로 아날로그 정현파 신호를 출력합니다. (200kHz마다 반복호출 되는 CpuTimer0 인터럽트 이용)
2) ADC-A SOC0로 ADCINA0채널로 들어오는 DAC-A출력을 입력받고, ADC-A SOC1으로 ADCINA1채널로 들어오는 DAC-B출력을 입력받습니다. 
3) ADC-A SOC1 변환 완료 후 ADCINT1 인터럽트를 요청하고 인터럽트가 실행되면 AdcaIsr 함수가 실행됩니다. 
4) AdcaIsr함수에서는 ADC-A SOC0, ADC-A SOC1 변환 결과를 저장합니다. 
5) ADC-A 모듈에 변환시작 신호(Start-Of-Conversion)를 전달하기 위한 EPWM2을 사용합니다.
*싱크웍스 LAB2_ADC_CPU1 프로젝트를 변형하였습니다. 

2) 추가설명
SOC(Start Of Conversion): SOC별로 어떤 채널(ADCINA0, A1 등)을 측정할지, 어떤 트리거로 변환을 시작할지 (예: 타이머, ePWM, 소프트웨어), 어떤 샘플링 윈도우를 사용할지 정해줍니다.
TMS320F28377D datasheet에 따르면 ADCINA0채널은 DACOUTA와 ADCINA1채널은 DACOUTB와 연결되어 있습니다.

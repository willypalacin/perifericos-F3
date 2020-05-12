#include "stm32f4xx.h"

//CONSTANTS
#define TRUE 1
#define USER_BUTTON GPIO_Pin_0
#include "stm32f4xx.h"

//CONSTANTS
#define TRUE 1
#define USER_BUTTON GPIO_Pin_0
#define INJECTOR_GPIO_A GPIO_Pin_2
#define INJECTOR_GPIO_B GPIO_Pin_4
#define INJECTOR_GPIO_C GPIO_Pin_6
#define SALIDA_TINJ GPIO_Pin_13
#define MIDA 32

#define ADC3_DR_ADDRESS     ((uint32_t)0x4001224C)



//VARIABLES
static uint16_t tics = 0;
uint16_t btn_pressed = 0;
uint16_t rev_min = 0;
uint16_t array_cargar[16] = {479,239,159,119,95,79,67,59,52,47,42,39,35,33,31};
uint16_t flag_decrementa = 1;
static uint16_t milis_rebots = 0;
static uint16_t cont_10micros = 0;
static uint16_t velocidad = 0;
uint16_t t_inj = 25;
uint16_t interrupcio = 0;
uint16_t count_tinj = 0;
uint16_t Tim6_RSI;
uint16_t Tim2_RSI;
uint16_t EXTI1_RSI;
uint16_t aux = 0;
uint16_t auxDMA = 0;



uint16_t ADC3ConvertedValue[MIDA] = {};

uint16_t FinalValue[MIDA] = {};

uint16_t transferencia_completada;

uint16_t mostres32 = 0;

uint16_t DMA_option = 0;

uint16_t Cabalimetre_mostra_0 = 0;
uint16_t Sensor_temp_liquid_refrigerant_mostra_0 = 0;
uint16_t Sensor_temp_aire_mostra_0 = 0;
uint16_t Tensio_bateria_mostra_0 = 0;
uint16_t entra = 0;


void configuracionDMAyADC(void){


	ADC_InitTypeDef       ADC_InitStructure;
	ADC_CommonInitTypeDef ADC_CommonInitStructure;
	DMA_InitTypeDef       DMA_InitStructure;


	/* Enable ADC3, DMA2 and GPIO clocks ****************************************/
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2 | RCC_AHB1Periph_GPIOC|RCC_AHB1Periph_GPIOF, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC3, ENABLE);

	//Configurem el channel2 del canal 0 de la DMA2
	DMA_InitStructure.DMA_Channel = DMA_Channel_2;

	//ADC que usem (registre d'on llegira les dades la DMA, on escriu l'ADC
	DMA_InitStructure.DMA_PeripheralBaseAddr = ADC3_DR_ADDRESS;
	DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)&ADC3ConvertedValue;
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;
	DMA_InitStructure.DMA_BufferSize = (uint32_t)MIDA;
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord; //Datos de 16 bits
	DMA_InitStructure.DMA_Mode = DMA_Mode_Normal; //DMA_Mode_Circular
	DMA_InitStructure.DMA_Priority = DMA_Priority_High;
	DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;
	DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_HalfFull;
	DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
	DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
	DMA_Init(DMA2_Stream0, &DMA_InitStructure);

	DMA_Cmd(DMA2_Stream0, ENABLE);


	gpioConfigADC();

	ADC_CommonInitStructure.ADC_Mode = ADC_Mode_Independent;
	ADC_CommonInitStructure.ADC_Prescaler = ADC_Prescaler_Div2;
	ADC_CommonInitStructure.ADC_DMAAccessMode = ADC_DMAAccessMode_Disabled;
	ADC_CommonInitStructure.ADC_TwoSamplingDelay = ADC_TwoSamplingDelay_5Cycles;
	ADC_CommonInit(&ADC_CommonInitStructure);


	ADC_InitStructure.ADC_Resolution = ADC_Resolution_12b;
	ADC_InitStructure.ADC_ScanConvMode = ENABLE; //
	ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;
	ADC_InitStructure.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_Rising;
	ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigInjecConv_T5_TRGO; //
	ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
	ADC_InitStructure.ADC_NbrOfConversion = 4;
	ADC_Init(ADC3, &ADC_InitStructure);


	ADC_RegularChannelConfig(ADC3, ADC_Channel_13 , 1, ADC_SampleTime_15Cycles);
	ADC_RegularChannelConfig(ADC3, ADC_Channel_4, 2, ADC_SampleTime_15Cycles);
	ADC_RegularChannelConfig(ADC3, ADC_Channel_5, 3, ADC_SampleTime_15Cycles);
	ADC_RegularChannelConfig(ADC3, ADC_Channel_6, 4, ADC_SampleTime_15Cycles);


	//DEspues del ADC DMAREQ
	ADC_DMARequestAfterLastTransferCmd(ADC3, ENABLE);

	// Activamos ADC3 DMA
	ADC_DMACmd(ADC3, ENABLE);

	//Activamos ADC3
	ADC_Cmd(ADC3, ENABLE);

	NVIC_InitTypeDef NVIC_InitStructure;

	NVIC_InitStructure.NVIC_IRQChannel = DMA2_Stream0_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	DMA_ClearITPendingBit(DMA2_Stream0, DMA_IT_TCIF0);

	DMA_ITConfig(DMA2_Stream0, DMA_IT_TC | DMA_IT_HT, ENABLE);



}

void DMA_MemToMem_Config(void){

	//ADC_InitTypeDef       ADC_InitStructure;
	//ADC_CommonInitTypeDef ADC_CommonInitStructure;
	DMA_InitTypeDef       DMA_InitStructure;
	//GPIO_InitTypeDef      GPIO_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
		/* Enable the DMA Stream IRQ Channel */
		NVIC_InitStructure.NVIC_IRQChannel = DMA2_Stream0_IRQn;
		NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x01;
		NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x01;
		NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
		NVIC_Init(&NVIC_InitStructure);
		//DMA_ClearITPendingBit(DMA2_Stream0, DMA_IT_TCIF0);

		DMA_ITConfig(DMA2_Stream0, DMA_IT_TC|DMA_IT_TCIF0, ENABLE);

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2, ENABLE);

	DMA_InitStructure.DMA_Channel = DMA_Channel_2;
	DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&ADC3ConvertedValue;
	DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)&FinalValue;
	DMA_InitStructure.DMA_DIR = DMA_DIR_MemoryToMemory;
	DMA_InitStructure.DMA_BufferSize = (uint32_t)MIDA;
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Enable;
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord; //16 bits
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord; //16 bits
	DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
	DMA_InitStructure.DMA_Priority = DMA_Priority_High;
	DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;
	DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_HalfFull;
	DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
	DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
	DMA_Init(DMA2_Stream0, &DMA_InitStructure);
	DMA_Cmd(DMA2_Stream0, ENABLE);



}


void enableADCparameters() {
	configuracionDMAyADC();
	//ADC_DMARequestAfterLastTransferCmd(ADC3, ENABLE);
	// Activamos ADC3 DMA
	//Activamos ADC3
	//ADC_Cmd(ADC3, ENABLE);
	//ADC_SoftwareStartConv(ADC3);


}


//Configuració simuladors switch
void init_switch(void){
                GPIO_InitTypeDef gpio_a;

                //El configurem en el GPIOD
                RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);

                GPIO_StructInit(&gpio_a);
                gpio_a.GPIO_Pin = INJECTOR_GPIO_A;
                //Input mode
                gpio_a.GPIO_Mode = GPIO_Mode_IN;
                //Activem pull-up (pq simulem, no soldem switch)
                gpio_a.GPIO_PuPd = GPIO_PuPd_UP;
                //Carreguem configuració pin
                GPIO_Init(GPIOA, &gpio_a);
                GPIO_InitTypeDef gpio_b;
                //El configurem en el GPIOD
                RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
                GPIO_StructInit(&gpio_b);
                gpio_b.GPIO_Pin = INJECTOR_GPIO_B;
                //Input mode
                gpio_b.GPIO_Mode = GPIO_Mode_IN;
                //Activem pull-up (pq simulem, no soldem switch)
                gpio_b.GPIO_PuPd = GPIO_PuPd_UP;
                //Carreguem configuració pin
                GPIO_Init(GPIOB, &gpio_b);
                GPIO_InitTypeDef gpio_c;
                //El configurem en el GPIOD
                RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
                GPIO_StructInit(&gpio_c);
                gpio_c.GPIO_Pin = INJECTOR_GPIO_C;
                //Input mode
                gpio_c.GPIO_Mode = GPIO_Mode_IN;
                //Activem pull-up (pq simulem, no soldem switch)
                gpio_c.GPIO_PuPd = GPIO_PuPd_UP;
                //Carreguem configuració pin
                GPIO_Init(GPIOC, &gpio_c);
}



//Interrupcio de finalitzacio de la transferencia per DMA
void DMA2_Stream0_IRQHandler(void){
	 auxDMA = 200;
	//Mirem si el flag de fi de transferencia esta actiu
	if(DMA_GetITStatus(DMA2_Stream0,DMA_IT_TCIF0) != RESET) {

		//Netejem flag
		//DMA_ClearITPendingBit(DMA2_Stream0,DMA_IT_TCIF0);
		DMA_ClearITPendingBit(DMA2_Stream0, DMA_IT_HTIF0 | DMA_IT_TCIF0);

		if(!DMA_option){

			//Desactivem ADC
			ADC_Cmd(ADC3, DISABLE);


				//Activamos ADC3


			DMA_option = 1;
			//Configurem la DMA perque passi de memoria a memoria
			DMA_MemToMem_Config();



		}else {
			DMA_option = 0;

			enableADCparameters();

			//transferencia_completada = 1;
		}
	}
}


void EXTI1_IRQHandler(void) {
	//EXTI1_RSI =	TIM_GetCounter(TIM3);

    if (EXTI_GetITStatus(EXTI_Line1) != RESET) {
    	// comprobamos flanco de bajada
    	if(!GPIO_ReadInputDataBit(GPIOD, GPIO_Pin_1)) {
    		//configuracionDMAyADC();
    		ADC_SoftwareStartConv(ADC3);
    		aux = 200;
    		//Flanco de subida
    	}else {
    		interrupcio = 1;
    		velocidad= (6000000/(cont_10micros*2)); //xq cuenta cada 10 micros segun los tics

    		cont_10micros = 0;
    	}
    	//limpiamos int
    	EXTI_ClearITPendingBit(EXTI_Line1);
    }
   // EXTI1_RSI = TIM_GetCounter(TIM3) - EXTI1_RSI;
}

void ConfiguraPD1(void) {
    GPIO_InitTypeDef GPIO_InitStruct;
    EXTI_InitTypeDef EXTI_InitStruct;
    NVIC_InitTypeDef NVIC_InitStruct;
   //Activa clock
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);
    /* Enable clock for SYSCFG */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);
    //Lectura Input
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IN;
    GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStruct.GPIO_Pin = GPIO_Pin_1;
    GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_Init(GPIOD, &GPIO_InitStruct);

    //Utilizremos la EXIT_line para el PD1
    SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOD, EXTI_PinSource1);

    //PD1, esta conectado a EXIT1
    EXTI_InitStruct.EXTI_Line = EXTI_Line1;
    EXTI_InitStruct.EXTI_LineCmd = ENABLE;

    EXTI_InitStruct.EXTI_Mode = EXTI_Mode_Interrupt;
    //La int sera por flanco subida
    EXTI_InitStruct.EXTI_Trigger = EXTI_Trigger_Rising_Falling;
    //anadimos a EXTI
    EXTI_Init(&EXTI_InitStruct);

    // PD1 = EXTI1_IRQn
    NVIC_InitStruct.NVIC_IRQChannel = EXTI1_IRQn;
    //No prioridad
    NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 0x02;
    //No subprioridad
    NVIC_InitStruct.NVIC_IRQChannelSubPriority = 0x02;
    NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStruct);
}

void calcula_temps_injeccio(void){
	if (GPIO_ReadInputDataBit(GPIOC, INJECTOR_GPIO_C) != TRUE){
		if (GPIO_ReadInputDataBit(GPIOB, INJECTOR_GPIO_B) != TRUE){
			if (GPIO_ReadInputDataBit(GPIOA, INJECTOR_GPIO_A) != TRUE){
				t_inj = 25;
			} else{
				t_inj = 50;
			}
		}else {
			if (GPIO_ReadInputDataBit(GPIOA, INJECTOR_GPIO_A) != TRUE){
				t_inj = 100;
			} else{
				t_inj = 200;
			}
		}
	}else {
		if (GPIO_ReadInputDataBit(GPIOB, INJECTOR_GPIO_B) != TRUE){
			if (GPIO_ReadInputDataBit(GPIOA, INJECTOR_GPIO_A) != TRUE){
				t_inj = 400;
			} else{
				t_inj = 800;
			}
		}else {
			if (GPIO_ReadInputDataBit(GPIOA, INJECTOR_GPIO_A) != TRUE){
				t_inj = 1600;
			} else{
				t_inj = 3200;
			}
		}
	}

}

void TIM5_IRQHandler(void) {

	if (TIM_GetITStatus(TIM5, TIM_IT_Update) != RESET){
		if(entra==0) {
				  GPIO_SetBits(GPIOE, GPIO_Pin_4);
				  entra = 1;
			  } else {
				  GPIO_ResetBits(GPIOE, GPIO_Pin_4);
				  	  entra = 0;
			  }
		TIM_ClearITPendingBit(TIM5, TIM_IT_Update);

	}

}


void configuraTimer2(uint16_t numOfMilleseconds){
  NVIC_InitTypeDef NVIC_InitStructure;
  /* global interrupt */
  NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
  TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
  //Salta cada milisegundo * numOfMilleseconds.
  TIM_TimeBaseStructure.TIM_Period = (7*numOfMilleseconds);
  TIM_TimeBaseStructure.TIM_Prescaler = 10499;

  TIM_TimeBaseStructure.TIM_ClockDivision = 0;
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
  TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);
  TIM_ClearITPendingBit(TIM2, TIM_IT_Update);

  TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE);
  TIM_Cmd(TIM2, ENABLE);
}

void configuraTimer5(){
  NVIC_InitTypeDef NVIC_InitStructure;
  /* global interrupt */
  NVIC_InitStructure.NVIC_IRQChannel = TIM5_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM5, ENABLE);
  TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
  //Salta cada milisegundo * numOfMilleseconds.
  TIM_TimeBaseStructure.TIM_Period = 799;
  TIM_TimeBaseStructure.TIM_Prescaler = 20;

  TIM_TimeBaseStructure.TIM_ClockDivision = 0;
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
  TIM_TimeBaseInit(TIM5, &TIM_TimeBaseStructure);
  TIM_ClearITPendingBit(TIM5, TIM_IT_Update);

  TIM_ITConfig(TIM5, TIM_IT_Update, ENABLE);
  TIM_SelectOutputTrigger(TIM5, TIM_TRGOSource_Update); //ACtivamos el trigger para que convierta cada 200 micros
  TIM_Cmd(TIM5, ENABLE);
}



void configuraGPIOG13() {
	GPIO_InitTypeDef gpio;
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOG, ENABLE);
	GPIO_StructInit(&gpio);
	gpio.GPIO_Pin = GPIO_Pin_13 | GPIO_Pin_14;
	gpio.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_Init(GPIOG, &gpio);
	//GPIO_SetBits(GPIOG, GPIO_Pin_13);
}

void configuraGPIOE() {
	GPIO_InitTypeDef gpio;
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);
	GPIO_StructInit(&gpio);
	gpio.GPIO_Pin = GPIO_Pin_2 | GPIO_Pin_3|GPIO_Pin_4;
	gpio.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_Init(GPIOE, &gpio);
}


void TIM6_DAC_IRQHandler(void) {
  Tim6_RSI =	TIM_GetCounter(TIM3);
  if (TIM_GetITStatus(TIM6, TIM_IT_Update) != RESET) {
    TIM_ClearITPendingBit(TIM6, TIM_IT_Update);
    tics++;
    if (tics>=5){
    	cont_10micros++; //cuenta cada diez micros
    	tics=0;
    	if(interrupcio==1) {
    		count_tinj++;
    	}
    }
    if (cont_10micros>60000) cont_10micros=60000;
  }

  Tim6_RSI = TIM_GetCounter(TIM3) - Tim6_RSI;
}

void TIM2_IRQHandler(void){
	Tim2_RSI =	TIM_GetCounter(TIM3);
  if (TIM_GetITStatus(TIM2, TIM_IT_Update) != RESET){
	  /*if(entra==0) {
		  GPIO_SetBits(GPIOG, GPIO_Pin_13);
		  entra = 1;
	  } else {
		  GPIO_ResetBits(GPIOG, GPIO_Pin_13);
		  	  entra = 0;
	  }*/

	  TIM_ClearITPendingBit(TIM2, TIM_IT_Update);
	  if (GPIO_ReadInputDataBit(GPIOA, USER_BUTTON) == TRUE){
		  //Boto pulsat!!!
		  btn_pressed = 1;
		  milis_rebots++;
		  if (milis_rebots == 100){
			  milis_rebots = 0;
			  if (rev_min == 15)  flag_decrementa = -1;
			  if (rev_min == 0) flag_decrementa = 1;
			  rev_min = (rev_min + flag_decrementa);
			  inicia_Timer4();
			  configuraDutyPWM();
			  inicializaLedPWM();
			  //GPIO_SetBits(GPIOG, GPIO_Pin_13);
		  }
	  } else{
		  milis_rebots = 0;
		  btn_pressed=0;
		 // GPIO_ResetBits(GPIOG, GPIO_Pin_13);
	  }
	  //configuraTimer2(100);*/
  }
  Tim2_RSI = TIM_GetCounter(TIM3)- Tim2_RSI;
}

void inicia_Timer3() {
	NVIC_InitTypeDef NVIC_InitStructure;
	//NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	///NVIC_EnableIRQ(TIM3_IRQn);
	NVIC_Init(&NVIC_InitStructure);
	TIM_TimeBaseInitTypeDef TIM_BaseStruct;
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE); //Activamos el clock para el timer3
	TIM_BaseStruct.TIM_Prescaler = 1; //frec a 8M
    TIM_BaseStruct.TIM_CounterMode = TIM_CounterMode_Up;
    /* Tenemos este array preconfigurado, de forma que aumentamos su valor, si se pulsa el pulsador */
    TIM_BaseStruct.TIM_Period = 0xFFFFFFFF;
    TIM_BaseStruct.TIM_ClockDivision = 1;
    TIM_BaseStruct.TIM_RepetitionCounter = 0;
    TIM_ITConfig(TIM3, TIM_IT_Update, ENABLE);
    TIM_ClearITPendingBit(TIM3, TIM_IT_Update);
    TIM_TimeBaseInit(TIM3, &TIM_BaseStruct);
    TIM_Cmd(TIM3, ENABLE);
}

void inicia_Timer4(void) {
	TIM_TimeBaseInitTypeDef TIM_BaseStruct;
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE); //Activamos el clock para el timer4
	TIM_BaseStruct.TIM_Prescaler = 10499; //Reducimos la frecuencia aprox 16Mhz
    TIM_BaseStruct.TIM_CounterMode = TIM_CounterMode_Up;
    /* Tenemos este array preconfigurado, de forma que aumentamos su valor, si se pulsa el pulsador */
    TIM_BaseStruct.TIM_Period = array_cargar[rev_min];
    TIM_BaseStruct.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_BaseStruct.TIM_RepetitionCounter = 0;
    TIM_TimeBaseInit(TIM4, &TIM_BaseStruct);
    TIM_Cmd(TIM4, ENABLE);
}

void inicializaTimer6(void) {
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM6, ENABLE);
	TIM_TimeBaseStructure.TIM_Prescaler = 1;
	//Salta cada 2 micros
	TIM_TimeBaseStructure.TIM_Period = 83;
	TIM_TimeBaseStructure.TIM_ClockDivision = 1;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;
	TIM_TimeBaseInit(TIM6, &TIM_TimeBaseStructure);
	//Nombre de la IRQ
	NVIC_InitStructure.NVIC_IRQChannel = TIM6_DAC_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 9;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	//USART_SendTextNL(USART1, ''TIMER SET UP'');
	TIM_ITConfig(TIM6, TIM_IT_Update, ENABLE);
	TIM_ClearITPendingBit(TIM6, TIM_IT_Update);
	TIM_Cmd(TIM6, ENABLE);
}

void configuraDutyPWM(void) {
    TIM_OCInitTypeDef TIM_OCStruct;
    TIM_OCStruct.TIM_OCMode = TIM_OCMode_PWM2;
    TIM_OCStruct.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCStruct.TIM_OCPolarity = TIM_OCPolarity_Low;
    //Declaramos los tics segun las rpm a las que estemos
    uint16_t max_tics = array_cargar[rev_min];
    //Generamos uestro pwm con duty del 54
    uint16_t  pwm = ((max_tics + 1) * 25) / 100 - 1;
    pwm = ((max_tics + 1) * 54) / 100 - 1;
    TIM_OCStruct.TIM_Pulse = pwm;
    TIM_OC2Init(TIM4, &TIM_OCStruct);
    //Asociamos con TIM4
    TIM_OC2PreloadConfig(TIM4, TIM_OCPreload_Enable);

}

void inicializaLedPWM(void) {
	GPIO_InitTypeDef GPIO_InitStruct;

	//Ponemos clock para GPIOD
	    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);

	    //Asoc con Timer4
	    GPIO_PinAFConfig(GPIOD, GPIO_PinSource12, GPIO_AF_TIM4);
	    GPIO_PinAFConfig(GPIOD, GPIO_PinSource13, GPIO_AF_TIM4);

	    //Utilizamos estos dos pins. Solo Pin 13 es el que saca 54 por ciento de duty
	    GPIO_InitStruct.GPIO_Pin = GPIO_Pin_12 | GPIO_Pin_13;
	    GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
	    GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;
	    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;
	    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_100MHz;
	    GPIO_Init(GPIOD, &GPIO_InitStruct);
}

void init_clock(void) {
	RCC_DeInit();
	//Activem cristall extern HSE
	RCC_HSEConfig(RCC_HSE_ON);
	//Esperem a que s'activi
	RCC_WaitForHSEStartUp();
	//Configurem el PLL per tenir sysclock sigui de 168 MHz
	RCC_PLLConfig(RCC_PLLSource_HSE, 8, 336, 2, 7);
	//Activem el PLL
	RCC_PLLCmd(ENABLE);
	//Esperem a que el PLL estigui actiu
	while (RCC_GetFlagStatus(RCC_FLAG_PLLRDY) == RESET);
	//AHB a 168MHz
	RCC_HCLKConfig(RCC_SYSCLK_Div1);
	//APB1 a 42MHz
	RCC_PCLK1Config(RCC_HCLK_Div4);
	//APB2 a 84MHz
	RCC_PCLK2Config(RCC_HCLK_Div2);
	//Posem el PLL al clock
	RCC_SYSCLKConfig(RCC_SYSCLKSource_PLLCLK);
}

void init_button(void){
	GPIO_InitTypeDef gpio;
	//Activem el clock del GPIOA
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
	GPIO_StructInit(&gpio);
	gpio.GPIO_Pin = USER_BUTTON;
	gpio.GPIO_Mode = GPIO_Mode_IN;
	gpio.GPIO_PuPd = GPIO_PuPd_DOWN;
	GPIO_Init(GPIOA, &gpio);
}

void gpioConfigADC(){
	GPIO_InitTypeDef      GPIO_InitStructure;
	GPIO_InitTypeDef      GPIO_InitStructure2;
	GPIO_InitTypeDef      GPIO_InitStructure3;
	GPIO_InitTypeDef      GPIO_InitStructure4;

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL ;
	GPIO_Init(GPIOC, &GPIO_InitStructure);

	GPIO_InitStructure2.GPIO_Pin = GPIO_Pin_6;
	GPIO_InitStructure2.GPIO_Mode = GPIO_Mode_AN;
	GPIO_InitStructure2.GPIO_PuPd = GPIO_PuPd_NOPULL ;
	GPIO_Init(GPIOF, &GPIO_InitStructure2);

	GPIO_InitStructure3.GPIO_Pin = GPIO_Pin_7 ;
	GPIO_InitStructure3.GPIO_Mode = GPIO_Mode_AN;
	GPIO_InitStructure3.GPIO_PuPd = GPIO_PuPd_NOPULL ;
	GPIO_Init(GPIOF, &GPIO_InitStructure3);

	GPIO_InitStructure4.GPIO_Pin = GPIO_Pin_8 ;
	GPIO_InitStructure4.GPIO_Mode = GPIO_Mode_AN;
	GPIO_InitStructure4.GPIO_PuPd = GPIO_PuPd_NOPULL ;
	GPIO_Init(GPIOF, &GPIO_InitStructure4);

}

void inicialitza_sistema(void){
	init_clock();
	init_button();
	inicia_Timer4();
	configuraDutyPWM();
	inicializaLedPWM();
	inicializaTimer6();
	configuraTimer2(1);
	configuraTimer5();
	inicia_Timer3();
	configuraGPIOG13();
	ConfiguraPD1();
	init_switch();
	configuraGPIOE();
	//configGpioAdc();

	configuracionDMAyADC();
	//entra = ADC_GetConversionValue(ADC3);


}

void espera_interrupcio(void){
	while(!interrupcio){
		continue;
	}
	GPIO_SetBits(GPIOG, GPIO_Pin_13);
	//Contamos hasta llegar al valor max del tiempo de inyeccion
	if(count_tinj>t_inj) {
		count_tinj=0;
	    GPIO_ResetBits(GPIOG, GPIO_Pin_13);
	    interrupcio=0;
	 }
}

int main(void) {
	inicialitza_sistema();
    while(1) {
    	calcula_temps_injeccio();
    	espera_interrupcio();
    }
}

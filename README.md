# Project-FreeRtos-STM32F407
Explication : 
Mots clé:Un reservoir intelligent /Une vanne equipé par un bras metal q'on l'utulise pour la fermeture et  l'ouverture du vanne/Servo Moteur SG90/Capteur niveau
d'eau/Carte STM32F407/Un tuyeau pour le vidage du reservoir:
deux phases :
phase 1 :
On remplit le reservoir avec du l'eau a travers la vanne jusqu'a le capteur de niveau d'eau prend la valeur x = 4095 ( On obeservant les valeurs 
avec Tera term comme un terminal serie on observant 3 etats "Le niveau  d eau n'a pas encore en contact avec le capteur :" , "Le niveau  d eau est en contact avec le capteur :"
"Tourner Servo Moteur :" dans notre cas "Tourner Servo Moteur :
On peut voir le changement l'etat des leds dans la carte d'ou le clignotement de tout les leds , et l'orientation du notre Servo Moteur avec des angles 
qui permet de lever la bras pour la fermeture du vanne
Phase 2: 
On essaye de vider le reservoir a travers le tuyeau jusqu'a le niveau d'eau soit different du 4095  (Affichage :"Le niveau  d eau n'a pas encore en contact avec le capteur :" ou "Le niveau  d eau est en contact avec le capteur :":
Il provoque le changement l'etat des leds Led rouge reste allumé et le recule du moteur servo pour laisser la vanne sortir du l'eau
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
Code(partie configuration et main) pour chaque partie responsable :
Configuration :
Pour le capteur  pour la lecture des valeurs analogique et la convertir en digital on ajoute comme configuration :
void ADC_Config ()
{
	/* Configure ADC Channel 12 pin (ADC_IN1 = PC.2) as analog input */ 
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL ;
  GPIO_Init(GPIOC, &GPIO_InitStructure);	
	
		/* ADC1 regular channel 12 configuration ************************************/
  ADC_InitStructure.ADC_Resolution = ADC_Resolution_12b;
  ADC_InitStructure.ADC_ScanConvMode = DISABLE;
  ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;
  ADC_InitStructure.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_None;
  ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
  ADC_InitStructure.ADC_NbrOfConversion = 1;
  ADC_Init(ADC1, &ADC_InitStructure);

  ADC_RegularChannelConfig(ADC1, ADC_Channel_10, 1, ADC_SampleTime_3Cycles);
	
	/* Enable ADC1 **************************************************************/
  ADC_Cmd(ADC1, ENABLE);
	
	/* Start ADC1 Software Conversion */ 
  ADC_SoftwareStartConv(ADC1);
}
	
void DAC_Config ()
{
	/* DAC channel 1 (DAC_OUT1 = PA.4) configuration */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	/* DAC channel1 Configuration */
	DAC_DeInit();
  DAC_InitStructure.DAC_Trigger = DAC_Trigger_None;
  DAC_InitStructure.DAC_WaveGeneration = DAC_WaveGeneration_None;
  DAC_InitStructure.DAC_OutputBuffer = DAC_OutputBuffer_Enable;
  DAC_Init(DAC_Channel_1, &DAC_InitStructure);
  
  /* Enable DAC Channel1 */
  DAC_Cmd(DAC_Channel_1, ENABLE);
}
Main :
while(ADC_GetFlagStatus(ADC1,ADC_FLAG_EOC)==RESET); // wait till the and of the conversion
x=ADC_GetConversionValue(ADC1);
DAC_SetChannel1Data(DAC_Align_12b_R, ADC_GetConversionValue(ADC1));
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
***Pour L'affichage des valeurs il faut ajouter l'usb type b pour la serial communication 
Configuration :
void USB_Config ()
{
USBD_Init(&USB_OTG_dev,
#ifdef USE_USB_OTG_HS 
  USB_OTG_HS_CORE_ID,
#else            
  USB_OTG_FS_CORE_ID,
#endif  
  &USR_desc, 
  &USBD_CDC_cb,
  &USR_cb);
}
Main:
sprintf(test,"%i",x); // Convert int to string
		strcat(test, "\n");		// \r --> CR  13
		if(synchronisation==1) {
			if ( x < 2000 ){
				printf("Le niveau  d eau n'a pas encore en contact avec le capteur : ");
				printf(test, "\n");
			}
			else if  ( x < 4095 || x > 2000)
			{
				 
				printf("Le niveau  d eau est en contact avec le capteur : ");
				printf(test, "\n");
				
			}
			else 
				{
				if ( x == 4095 ) {
				printf("Tourner Servo Moteur : ");
				printf(test, "\n");
				}
			}
				
 //   VCP_send_str((uint8_t *) test); // Send string through USB_VCP
		synchronisation=0;
		}		
	    if(VCP_get_char((uint8_t *)buf)) {
		car=buf[0];
 //   VCP_send_str((uint8_t *) buf);					 //pour echo uniquement
	    synchronisation=1;
		}

	{
}
  }
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
***Pour Les Leds :
Config : 
void LED()
{
	
/*gpio_2GE.GPIO_Speed=GPIO_Speed_2MHz;
gpio_2GE.GPIO_Pin=GPIO_Pin_0;
gpio_2GE.GPIO_Mode=GPIO_Mode_IN;
GPIO_Init(GPIOA,&gpio_2GE);*/   // PA0 en entree "boutton poussoir"
gpio_2GE.GPIO_Speed=GPIO_Speed_2MHz;
gpio_2GE.GPIO_Pin=GPIO_Pin_12|GPIO_Pin_13|GPIO_Pin_14|GPIO_Pin_15;
gpio_2GE.GPIO_Mode=GPIO_Mode_OUT;
GPIO_Init(GPIOD,&gpio_2GE);
	
//void GPIO_Init(GPIO_TypeDef* GPIOx, GPIO_InitTypeDef* GPIO_InitStruct);

}

ou
 
void LED_Config ()
{
	STM32F4_Discovery_LEDInit(LED3);
  STM32F4_Discovery_LEDInit(LED4);
  STM32F4_Discovery_LEDInit(LED5);
  STM32F4_Discovery_LEDInit(LED6);
  STM32F4_Discovery_PBInit(BUTTON_USER, BUTTON_MODE_EXTI);
}
Main :
 if ( x == 4095 ){
			GPIO_ToggleBits(GPIOD,GPIO_Pin_13);
			GPIO_ToggleBits(GPIOD,GPIO_Pin_15);
			GPIO_ToggleBits( GPIOD,GPIO_Pin_12);
			GPIO_ToggleBits(GPIOD,GPIO_Pin_14);}
 else { 
		GPIO_ToggleBits(GPIOD,GPIO_Pin_13);

}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
***Pour Le  servo moteur  :
Configuration : 
void GPIO_AInit(void);
void TIM2_Init(void);
void TIM4_ms_Delay(uint32_t delay);

void GPIO_AInit(){
	//RCC->AHB1ENR |= (1<<0); //Enable GPIOA clock
	//GPIOA->AFR[0] |= 0x00100000; // Select the PA5 pin in alternate function mode
	//GPIOA->MODER |= (2<<14); //Set the PA5 pin alternate function
	//GPIOA->OTYPER = 0;
	//GPIOA->OSPEEDR = 0;
	RCC->AHB1ENR |= 1; //Enable GPIOA clock
	
   //GPIOA->AFR[1] |= 0x10000000; // Select the PB13 pin in alternate function mode
   //GPIOA->MODER |=  0x80000000; //Set the PB13 pin alternate function
	GPIOA->AFR[0] |= 0x00100000; // Select the PA5 pin in alternate function mode
  GPIOA->MODER |= 0x00000800; //Set the PA5 pin alternate function
	
}

void TIM2_Init(){
	//RCC->APB1ENR |= (1<<0);
	//TIM2->PSC = 16-1; //Setting the clock frequency to 1MHz.
	//TIM2->ARR = 20000; // Total period of the timer
	//TIM2->CR1 |= (1<<0);
	//TIM2->CNT = 0;
	//TIM2->CCMR1 = 0x0060; //PWM mode for the timer
	//TIM2->CCER |= 1; //Enable channel 1 as output
	//TIM2->CCR1 = 500 ; // Pulse width for PWM
	//while(!(TIM2->SR & (1<<0)));//Polling the update interrupt flag
    //Reset the update interrupt flag
	  RCC->APB1ENR |=1;
    TIM2->PSC = 16-1; //Setting the clock frequency to 1MHz.
    TIM2->ARR = 20000; // Total period of the timer
    TIM2->CNT = 0;
    TIM2->CCMR1 = 0x0060; //PWM mode for the timer
    TIM2->CCER |= 1; //Enable channel 1 as output
    TIM2->CCR1 = 500; // Pulse width for PWM
}

void TIM4_ms_Delay(uint32_t delay){
	//RCC->APB1ENR |= (1<<2) ; //Start the clock for the timer peripheral
	//TIM4->PSC = 16000-1; //Setting the clock frequency to 1kHz.
	//TIM4->ARR = (delay); // Total period of the timer
	//TIM4->CNT = 0;
	//TIM4->CR1 |= 1; //Start the Timer
	//while(!(TIM4->SR & TIM_SR_UIF)){} //Polling the update interrupt flag
	//TIM4->SR &= ~(0x0001); //Reset the update interrupt flag
		RCC->APB1ENR |= 1<<2; //Start the clock for the timer peripheral
    TIM4->PSC = 16000-1; //Setting the clock frequency to 1kHz.
    TIM4->ARR = (delay); // Total period of the timer
    TIM4->CNT = 0;
    TIM4->CR1 |= 1; //Start the Timer
    while(!(TIM4->SR & TIM_SR_UIF)){} //Polling the update interrupt flag
    TIM4->SR &= ~(0x0001); //Reset the update interrupt flag
}
Main :
RCC->CR |= RCC_CR_HSEON;
	while(!(RCC->CR & RCC_CR_HSERDY));
	RCC->APB1ENR |= RCC_APB1ENR_PWREN ;
	RCC->CR |= PWR_CR_VOS;
	FLASH->ACR = FLASH_ACR_ICEN | FLASH_ACR_DCEN | FLASH_ACR_PRFTEN | FLASH_ACR_LATENCY_5WS;
	RCC->CFGR |= 0<<10; // set APB1 = 16 MHz
	GPIO_AInit();
	TIM2_Init();
	TIM2->CR1 |= 1;
if ( x == 4095 ){
			
			
			TIM2->CCR1 = TIM2->CCR1 + 2500;
      TIM4_ms_Delay(2500);
			TIM2->CCR1 = TIM2->CCR1 + 2500;
      TIM4_ms_Delay(2500);
			TIM2->CCR1 = TIM2->CCR1 + 2500;
      TIM4_ms_Delay(2500);
			TIM2->CCR1 = TIM2->CCR1 + 2500;
      TIM4_ms_Delay(2500);
			TIM2->CCR1 = TIM2->CCR1 + 2500;
      TIM4_ms_Delay(2500);
			TIM2->CCR1 = TIM2->CCR1 + 2500;
      TIM4_ms_Delay(2500);
			
			
			}
			else{
        if(TIM2->CCR1 < 2500){
            TIM2->CCR1 = TIM2->CCR1 + 50;
            TIM4_ms_Delay(50);
        }
        else{
            TIM2->CCR1 = 2500;
            TIM4_ms_Delay(2500);
        }
    
			}
	}

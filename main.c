/* Includes ------------------------------------------------------------------*/
#include "stm32f4_discovery.h"
#include "Config.h"
#include <stdio.h> // to use sprintf
#include "usbd_cdc_core.h"
#include <stdlib.h>
#include "usbd_cdc_vcp.h"
#include "usbd_usr.h"
#include "usbd_desc.h"
#include <time.h>
#define ARM_MATH_CM4

void GPIO_AInit(void);
void TIM2_Init(void);
void TIM4_ms_Delay(uint32_t delay);

void GPIO_AInit(){
	RCC->AHB1ENR |= 1; //Enable GPIOA clock
	
   //GPIOA->AFR[1] |= 0x10000000; // Select the PB13 pin in alternate function mode
   //GPIOA->MODER |=  0x80000000; //Set the PB13 pin alternate function
	GPIOA->AFR[0] |= 0x00100000; // Select the PA5 pin in alternate function mode
  GPIOA->MODER |= 0x00000800; //Set the PA5 pin alternate function
	
}

void TIM2_Init(){
	  RCC->APB1ENR |=1;
    TIM2->PSC = 16-1; //Setting the clock frequency to 1MHz.
    TIM2->ARR = 20000; // Total period of the timer
    TIM2->CNT = 0;
    TIM2->CCMR1 = 0x0060; //PWM mode for the timer
    TIM2->CCER |= 1; //Enable channel 1 as output
    TIM2->CCR1 = 500; // Pulse width for PWM
}

void TIM4_ms_Delay(uint32_t delay){
		RCC->APB1ENR |= 1<<2; //Start the clock for the timer peripheral
    TIM4->PSC = 16000-1; //Setting the clock frequency to 1kHz.
    TIM4->ARR = (delay); // Total period of the timer
    TIM4->CNT = 0;
    TIM4->CR1 |= 1; //Start the Timer
    while(!(TIM4->SR & TIM_SR_UIF)){} //Polling the update interrupt flag
    TIM4->SR &= ~(0x0001); //Reset the update interrupt flag
}

int x;
int Len1;
uint32_t Len;
char test_car,car;
uint8_t synchronisation=1;
char * lec_cr;
uint32_t position;
char buf[1];
char test[] = "test  ";
char test_receive[10];
struct __FILE{
	int dummy;
};
FILE __stdout;
int fputc (int ch, FILE *f) {
VCP_put_char(ch);
	return ch;
}
void Delay(__IO uint32_t nCount){
	while(nCount--){
	}
}

int main(void)
{ 
 	int xnn =  2 ;
	RCC->CR |= RCC_CR_HSEON;
	while(!(RCC->CR & RCC_CR_HSERDY));
	RCC->APB1ENR |= RCC_APB1ENR_PWREN ;
	RCC->CR |= PWR_CR_VOS;
	FLASH->ACR = FLASH_ACR_ICEN | FLASH_ACR_DCEN | FLASH_ACR_PRFTEN | FLASH_ACR_LATENCY_5WS;
	RCC->CFGR |= 0<<10; // set APB1 = 16 MHz
	GPIO_AInit();
	TIM2_Init();
	TIM2->CR1 |= 1;
	void LED();
 	RCC_Config ();
	ADC_Config ();	  // PC2 canal 12
	LED_Config ();
	USB_Config ();
  while (1)
  {

		while(ADC_GetFlagStatus(ADC1,ADC_FLAG_EOC)==RESET); // wait till the and of the conversion
		x=ADC_GetConversionValue(ADC1);
		//code servo moteur zied
    if ( x == 4095 ){
			GPIO_ToggleBits(GPIOD,GPIO_Pin_13);
			GPIO_ToggleBits(GPIOD,GPIO_Pin_15);
			GPIO_ToggleBits( GPIOD,GPIO_Pin_12);
			GPIO_ToggleBits(GPIOD,GPIO_Pin_14);
			GPIO_SetBits(GPIOD,GPIO_Pin_5);}
		
		// fin code servo moteur
		//code servo moteur github
		if (xnn == 2 ){
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

			
		//DAC_SetChannel1Data(DAC_Align_12b_R, ADC_GetConversionValue(ADC1)); // sent the ADC converted value through DAC
		sprintf(test,"%i",x); // Convert int to string
		strcat(test, "\n");		// \r --> CR  13
		if(synchronisation==1) {
			if ( x < 2000 ){
				printf("Le niveau  d eau n'a pas encore en contact avec le capteur : ");
				printf(test, "\n");
			}
			else if  (x > 2000)
			{
				 
				printf("Le niveau  d eau est en contact avec le capteur : ");
				printf(test, "\n");
				
			}
			else if( x == 4095 )
				{
				
				printf("Tourner Servo Moteur : ");
				printf(test, "\n");
				
			}
				
		synchronisation=0;
		}		
	    if(VCP_get_char((uint8_t *)buf)) {
		car=buf[0];
	    synchronisation=1;
		}

	{
}
  }
}



#include "stm32f4xx_tim.h"
#include "usbd_cdc_core.h"
#include "usbd_cdc_vcp.h"
#include "usbd_usr.h"
#include "usbd_desc.h"

GPIO_InitTypeDef 			GPIO_InitStructure; // periph initialisation strucrures
ADC_InitTypeDef       ADC_InitStructure;
DAC_InitTypeDef				DAC_InitStructure;
GPIO_InitTypeDef  gpio_2GE;
//TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
//TIM_TimeBaseInitTypeDef TIM_BaseStruct;
//uint16_t PrescalerValue;//Valeur du prédiviseur
//uint16_t TimPeriod;

__ALIGN_BEGIN USB_OTG_CORE_HANDLE    USB_OTG_dev __ALIGN_END ;

void RCC_Config ()
{
	/* ADC Periph clock enable */
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);  
	
	/* GPIOA clock enable (to be used for DAC, PA4) */
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
	
	/* GPIOC clock enable (to be used for ADC, PC2) */
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
  
	/* DAC Periph clock enable */
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_DAC, ENABLE);
	/* LED */
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);
}
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

void LED_Config ()
{
	STM32F4_Discovery_LEDInit(LED3);
  STM32F4_Discovery_LEDInit(LED4);
  STM32F4_Discovery_LEDInit(LED5);
  STM32F4_Discovery_LEDInit(LED6);
  STM32F4_Discovery_PBInit(BUTTON_USER, BUTTON_MODE_EXTI);
}

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

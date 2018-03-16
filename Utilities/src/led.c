#include "stm32f10x.h"
#include "led.h"

void Led_init(void)	//��ʼ�� LED
{
		GPIO_InitTypeDef gpio_init_struct;//�ṹ��
		
    RCC_APB2PeriphClockCmd(LED1_RCC_PERIPH, ENABLE);//��ʼ�� LED ʱ�� 
	
	    
    /* ���� LED1 GPIO Ϊ����������ٶ�Ϊ 50MHz */
    gpio_init_struct.GPIO_Pin = LED1_GPIO_PIN;			
    gpio_init_struct.GPIO_Mode = GPIO_Mode_Out_PP;
    gpio_init_struct.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(LED1_GPIO_PORT, &gpio_init_struct);

}


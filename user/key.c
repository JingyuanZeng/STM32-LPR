#include "stm32f10x.h"
#include "key.h"

void Key_init(void)	//��ʼ�� KEY1
{	
	GPIO_InitTypeDef gpio_init_struct;//�ṹ��

	RCC_APB2PeriphClockCmd(KEY1_RCC, ENABLE);//��ʼ��ʱ�� 

	gpio_init_struct.GPIO_Mode = GPIO_Mode_IPU;//��������
	gpio_init_struct.GPIO_Speed = GPIO_Speed_50MHz;
	gpio_init_struct.GPIO_Pin = KEY1_PIN;

	GPIO_Init(KEY1_PORT, &gpio_init_struct);
}


#include "stm32f10x.h"
#include "key.h"

void Key_init(void)	//��ʼ�� KEY
{
    GPIO_InitTypeDef gpio_init_struct;//�ṹ��
	
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE); //��ʼ�� KEY ʱ��   ������������������������������
		RCC_APB2PeriphClockCmd(KEY0_RCC_PERIPH, ENABLE);
		RCC_APB2PeriphClockCmd(KEY1_RCC_PERIPH, ENABLE);
	
    /* ���ð��� (key0) GPIO Ϊ�������룬�ٶ�Ϊ 50MHz */
    gpio_init_struct.GPIO_Pin = KEY0_GPIO_PIN;
    gpio_init_struct.GPIO_Mode = GPIO_Mode_IPU;//GPIO_Mode_IN_FLOATING;//GPIO_Mode_IPU;��������----���պ���������,���ڰ�����˵�������������ã����������������
    gpio_init_struct.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(KEY0_GPIO_PORT, &gpio_init_struct);
    
		/* ���ð��� (key1) GPIO Ϊ�������룬�ٶ�Ϊ 50MHz */
		gpio_init_struct.GPIO_Pin = KEY1_GPIO_PIN;
		gpio_init_struct.GPIO_Mode = GPIO_Mode_IPU;//��������-
		GPIO_Init(KEY1_GPIO_PORT, &gpio_init_struct);
	
//     /* ���ð��� (key2) GPIO Ϊ�������룬�ٶ�Ϊ 50MHz */
//     gpio_init_struct.GPIO_Pin = KEY2_GPIO_PIN;
// 		gpio_init_struct.GPIO_Mode = GPIO_Mode_IPU;//��������----���պ���������,���ڰ�����˵�������������ã����������������
//     GPIO_Init(KEY2_GPIO_PORT, &gpio_init_struct);
}


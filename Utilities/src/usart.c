#include "stm32f10x.h"
#include "usart.h"

void USART1_init(void)//��ʼ������
{
		GPIO_InitTypeDef   gpio_init_struct;
		USART_InitTypeDef  usart_init_struct;//�ṹ��
	
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_AFIO, ENABLE);//��ʼ������ʱ��
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE); 
    //���ô���(UART)�õ��� GPIO
    /* ���ð������ڵ� TX ��Ϊ����ʽ���ù��ܣ��ٶ�Ϊ 50MHz */
    gpio_init_struct.GPIO_Mode = GPIO_Mode_AF_PP;
    gpio_init_struct.GPIO_Pin = GPIO_Pin_9;
    gpio_init_struct.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &gpio_init_struct);

    /* ���ð������ڵ� RX ��Ϊ�������룬�ٶ�Ϊ 50MHz */
    gpio_init_struct.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    gpio_init_struct.GPIO_Pin = GPIO_Pin_10;
    GPIO_Init(GPIOA, &gpio_init_struct);
	
    /* ������������:
       - ������     = 115200  
       - ����λ���� = 8 Bits
       - ֹͣλ���� = 1 Bit
       - ��żУ��λ:  ��
       - Ӳ������:    �� (RTS �� CTS �ź���)
       - ���ͺͽ���:  ʹ��
    */
    usart_init_struct.USART_BaudRate = 115200;
    usart_init_struct.USART_WordLength = USART_WordLength_8b;
    usart_init_struct.USART_StopBits = USART_StopBits_1;
    usart_init_struct.USART_Parity = USART_Parity_No;
    usart_init_struct.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    usart_init_struct.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;

    /* USART ���� */
    USART_Init(USART1, &usart_init_struct);
		USART_Cmd(USART1, ENABLE);	//ʹ�� USART1
}

void USART1_SendChar(int32_t ch)
{
    /* �ȴ�������� */
    while (USART_GetFlagStatus(USART1, USART_FLAG_TC) == RESET)
    {
    }
    USART_SendData(USART1, (uint8_t) ch);
}
u16 USART1_ReceiveChar(void)
{
    /* �ȴ������� */
    while (USART_GetFlagStatus(USART1, USART_FLAG_RXNE) == RESET)
    {
    }
    return USART_ReceiveData(USART1);
}

void USART1_SendNum(u32 num)//������λ������
{
		u8 ge,shi,bai,qian;
	
		ge=num%10;
		shi=num%100/10;
		bai=num%1000/100;
		qian=num/1000;
	
		USART1_SendChar(qian+0x30);
		USART1_SendChar(bai+0x30);
		USART1_SendChar(shi+0x30);
		USART1_SendChar(ge+0x30);
	
		USART1_SendChar(0x0d);//�س���������
		USART1_SendChar(0x0a);
}
void USART1_SendNum_0d0a(u32 num)//������λ������
{
		u8 ge,shi,bai,qian;
	
		ge=num%10;
		shi=num%100/10;
		bai=num%1000/100;
		qian=num/1000;
	
		USART1_SendChar(qian+0x30);
		USART1_SendChar(bai+0x30);
		USART1_SendChar(shi+0x30);
		USART1_SendChar(ge+0x30);
	
		USART1_SendChar(' ');//�س���������

}


/**
  ******************************************************************************
  * �ļ�: ltk_usart.h
  * ����: LeiTek (leitek.taobao.com)
  * �汾: V1.0.0
  * ����: �����м��ͷ�ļ�
  ******************************************************************************
  *
  *                  ��Ȩ���� (C): LeiTek (leitek.taobao.com)
  *                                www.leitek.com
  *
  ******************************************************************************
  */
  
/* ��ֹ�ݹ������ͷ�ļ� ------------------------------------------------------*/
#ifndef __USART_H
#define __USART_H

#ifdef __cplusplus
 extern "C" {
#endif 

/* Includes ------------------------------------------------------------------*/

void USART1_init(void);//��ʼ������
void USART1_SendChar(int32_t ch);
void USART1_SendNum(u32 num);
u16  USART1_ReceiveChar(void);


#ifdef USART1_IRQ
void ltk_usart_nvic_init(void);
#endif

#ifdef __cplusplus
}
#endif


#endif /* __LTK_USART_H */

/****************************** leitek.taobao.com *****************************/

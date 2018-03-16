#include "stm32f10x.h"
#include "stm32f10x_it.h"
#include "led.h"
#include "key.h"
#include "usart.h"
#include "delay.h"
#include "lcd.h"
#include "ov7670.h"
#include "string.h"
TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
u8 cur_status;
u8 LED_flag;
u32 a;
u32 b;
u16 AA,BB;
u16 color;
u16 color_save;//����һ�����ص�ֵ
u8 R,G,B;//��ɫ����
u8 TableChangePoint_240[240];//�����240��
u8 Max_ChangePoint_240,Min_ChangePoint_240,Max_bChangePoint,Min_bChangePoint;//���������ʼ��ĩ����,��������ʼ��ĩ����
u8 a_Continue,b_Continue;//��¼�ݡ�����ͻ����������
u8 flag_aMax;//ĩֵ���±�־
u8 Max_aChangePoint_reset,Min_aChangePoint_reset;//�������������
u16 Length_card,Width_card;//���Ƶĳ��Ϳ�
u8 Max_aChangePoint_reset_1,Min_aChangePoint_reset_1;//�����ϴε�����
u8 flag_MaxMinCompare;//Max_aChangePoint_reset_1��Max_aChangePoint_reset�ı�־
u8 TableChangePoint_320[320];//���������320��
float V,S,H;//����HSVֵ
u16 Min_blue;
u16 Max_blue;//���峵����ɫ����ĺ������ֵ����Сֵ
u16 k1,kk1,k2,kk2,k3,kk3,k4,kk4,k5,kk5,k6,kk6,k7,kk7,k8,kk8;//�˸��ַ��߽�
extern u8 Table[6300];//�����ַ��� ��10+26��*150 = 5400 �ֽ�
extern u8 talble_0[150];//�ַ�3,������
extern u8 table_yu[32];//����
extern u8 table_min[32];//����
extern u8 table_lu[32];//³��
extern u8 table_zhe[32];//����
extern u8 table_shan[32];//����
extern u8 table_cuan[32];//����
u8 R_a,G_a,B_a;//��ֵ

u8 table_picture[150];//���屣��ͼƬ������
u8 table_char[36]={0,1,2,3,4,5,6,7,8,9,'A','B','C','D','E','F','G','H','I','J','K','L','M','N','O','P','Q','R','S','T','U','V','W','X','Y','Z',};
u8 table_card[5][8]={	//����5�����ƵĶ�ά����
{0,0,0,0,0,0,0,0},		//���һλ����ʱ��
{0,0,0,0,0,0,0,0},
{0,0,0,0,0,0,0,0},
{0,0,0,0,0,0,0,0},
{0,0,0,0,0,0,0,0},
};
u8 tim3_num;//TIM3���Ӽ�ʱ

u8 table_cardMeasure[7];//�����ĳ��ƽ��
void Show_Card(u8 i);//��ʾ�ڼ��鳵��
void Show_Title();//��ʾ����

void MYRCC_DeInit(void)//��λ������������
{										 
	NVIC_InitTypeDef NVIC_InitStructure;	
	RCC->APB1RSTR = 0x00000000;//��λ����			 
	RCC->APB2RSTR = 0x00000000; 
	  
	RCC->AHBENR = 0x00000014;  //˯��ģʽ�����SRAMʱ��ʹ��.�����ر�.	  
	RCC->APB2ENR = 0x00000000; //����ʱ�ӹر�.			   
	RCC->APB1ENR = 0x00000000;   
	RCC->CR |= 0x00000001;     //ʹ���ڲ�����ʱ��HSION	 															 
	RCC->CFGR &= 0xF8FF0000;   //��λSW[1:0],HPRE[3:0],PPRE1[2:0],PPRE2[2:0],ADCPRE[1:0],MCO[2:0]					 
	RCC->CR &= 0xFEF6FFFF;     //��λHSEON,CSSON,PLLON
	RCC->CR &= 0xFFFBFFFF;     //��λHSEBYP	   	  
	RCC->CFGR &= 0xFF80FFFF;   //��λPLLSRC, PLLXTPRE, PLLMUL[3:0] and USBPRE 
	RCC->CIR = 0x00000000;     //�ر������ж�
	/* Enable the TIM3 global Interrupt */
	NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQn;  //TIM3�ж�
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;  //��ռ���ȼ�0��
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;  //�����ȼ�3��
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; //IRQͨ����ʹ��
	NVIC_Init(&NVIC_InitStructure);  //����NVIC_InitStruct��ָ���Ĳ�����ʼ������NVIC�Ĵ���
}
void Stm32_Clock_Init(u8 PLL)//ϵͳʱ�ӳ�ʼ������  pll:ѡ��ı�Ƶ������2��ʼ�����ֵΪ16	
{
	unsigned char temp=0;   
	MYRCC_DeInit();		  //��λ������������
	RCC->CR|=0x00010000;  //�ⲿ����ʱ��ʹ��HSEON
	while(!(RCC->CR>>17));//�ȴ��ⲿʱ�Ӿ���
	RCC->CFGR=0X00000400; //APB1=DIV2;APB2=DIV1;AHB=DIV1;
	PLL-=2;//����2����λ
	RCC->CFGR|=PLL<<18;   //����PLLֵ 2~16
	RCC->CFGR|=1<<16;	  //PLLSRC ON 
	FLASH->ACR|=0x32;	  //FLASH 2����ʱ����

	RCC->CR|=0x01000000;  //PLLON
	while(!(RCC->CR>>25));//�ȴ�PLL����
	RCC->CFGR|=0x00000002;//PLL��Ϊϵͳʱ��	 
	while(temp!=0x02)     //�ȴ�PLL��Ϊϵͳʱ�����óɹ�
	{   
		temp=RCC->CFGR>>2;
		temp&=0x03;
	}    
}

void LED()//LEDָʾ��-��ʾ
{
	GPIO_WriteBit(LED1_GPIO_PORT, LED1_GPIO_PIN,LED_flag>>7);
	LED_flag=~LED_flag;
}
void Data_LCD_Display()//������ʾ
{
	LCD_SetWindows(0,0,320,240);//������ʾ����
	GPIO_WriteBit(LCD_RS_PORT, LCD_RS_PIN,1);//��־������д��

	while(GPIO_ReadInputDataBit(GPIOC,GPIO_Pin_0)==1);
	GPIO_WriteBit(FIFO_WRST_PORT, FIFO_WRST_PIN, 0);
	GPIO_WriteBit(FIFO_WRST_PORT, FIFO_WRST_PIN, 1);
	GPIO_WriteBit(FIFO_WR_PORT, FIFO_WR_PIN, 1);
	while(GPIO_ReadInputDataBit(GPIOC,GPIO_Pin_0)==0);
	while(GPIO_ReadInputDataBit(GPIOC,GPIO_Pin_0)==1);
	GPIO_WriteBit(FIFO_WR_PORT, FIFO_WR_PIN, 0);
	
	FIFO_Reset_Read_Addr(); 
//	LED();
	
	for (a=0;a<240;a++)
	{
		for(b=0;b<320;b++)
		{
			GPIOC->BRR =1<<4;
			AA=GPIOA->IDR;						
			GPIOC->BSRR =1<<4;
			
			GPIOC->BRR =1<<4;
			BB=GPIOA->IDR&0x00ff;								
			GPIOC->BSRR =1<<4;	
			
			color=(AA<<8)|BB;
			
			LCD_DATA_PORT->ODR = color;

			GPIOC->BRR =1<<11;
			GPIOC->BSRR =1<<11;
		}							
	}	 
}
void RGB_HSV(u16 num)//RGB565תHSV
{
	float max,min;
	u8 r,g,b;
	r=(num>>11)*255/31;g=((num>>5)&0x3f)*255/63;b=(num&0x1f)*255/31;
	
	max=r;min=r;
	if(g>=min)max=g;
	if(b>=max)max=b;
	if(g<=max)min=g;
	if(b<=min)min=b;
	
	V=100*max/255;//ת��Ϊ�ٷֱ�
	S=100*(max-min)/max;//����100����ʾ
	if(max==r) H=(g-b)/(max-min)*60;
	if(max==g) H=120+(b-r)/(max-min)*60;
	if(max==b) H=240+(r-g)/(max-min)*60;
	if(H<0) H=H-360;
}
void Data_LCD_ColorDisplay()//��ɫ������ʾ������
{
	LCD_SetWindows(0,0,320,240);//������ʾ����
	GPIO_WriteBit(LCD_RS_PORT, LCD_RS_PIN,1);//��־������д��

	while(GPIO_ReadInputDataBit(GPIOC,GPIO_Pin_0)==1);
	GPIO_WriteBit(FIFO_WRST_PORT, FIFO_WRST_PIN, 0);
	GPIO_WriteBit(FIFO_WRST_PORT, FIFO_WRST_PIN, 1);
	GPIO_WriteBit(FIFO_WR_PORT, FIFO_WR_PIN, 1);
	while(GPIO_ReadInputDataBit(GPIOC,GPIO_Pin_0)==0);
	while(GPIO_ReadInputDataBit(GPIOC,GPIO_Pin_0)==1);
	GPIO_WriteBit(FIFO_WR_PORT, FIFO_WR_PIN, 0);
	
	FIFO_Reset_Read_Addr(); 
//	LED();
	
	for (a=0;a<320;a++)
	{
		for(b=0;b<240;b++)
		{
			GPIOC->BRR =1<<4;
			AA=GPIOA->IDR;						
			GPIOC->BSRR =1<<4;
			
			GPIOC->BRR =1<<4;
			BB=GPIOA->IDR&0x00ff;								
			GPIOC->BSRR =1<<4;	
			
			color=(AA<<8)|BB;
			

// 			if(a==100&&b==100)
// 			{
// 				R=color>>11;
// 				G=(color>>5)&0x3f;
// 				B=color&0x1f;	
// 				
// 				color_save=color;
// 			}
// //���ص���ԣ�λ��ָʾ
// 			if(a==99&&b==100)
// 			{
// 				color=0xf800;
// 			}
// 			if(a==101&&b==100)
// 			{
// 				color=0xf800;
// 			}
// 			if(a==100&&b==99)
// 			{
// 				color=0xf800;
// 			}
// 			if(a==100&&b==101)
// 			{
// 				color=0xf800;
// 			}
			if(a==100&&b==250)
			{
				R=color>>11;
				G=(color>>5)&0x3f;
				B=color&0x1f;	
				
				color_save=color;
			}
//���ص���ԣ�λ��ָʾ
			if(a==100&&b==249)
			{
				color=0xf800;
			}
			if(a==100&&b==251)
			{
				color=0xf800;
			}
			if(a==99&&b==250)
			{
				color=0xf800;
			}
			if(a==101&&b==250)
			{
				color=0xf800;
			}
			LCD_DATA_PORT->ODR = color;

			GPIOC->BRR =1<<11;
			GPIOC->BSRR =1<<11;
		}							
	}

	RGB_HSV(color_save);//RGB565תHSV
	LCD_ShowChar(30,200,'H',0);//X���꣬Y���꣬�ַ������ģʽ
	LCD_ShowNum(30,220,H,3);//X���꣬Y���꣬���֣���λ��

	LCD_ShowChar(60,200,'S',0);//X���꣬Y���꣬�ַ������ģʽ
	LCD_ShowNum(60,220,S,3);//X���꣬Y���꣬���֣���λ��

	LCD_ShowChar(90,200,'V',0);//X���꣬Y���꣬�ַ������ģʽ
	LCD_ShowNum(90,220,V,3);//X���꣬Y���꣬���֣���λ��	
	//��ʾ�ⶨ������
	LCD_ShowChar(160,200,'R',0);//X���꣬Y���꣬�ַ������ģʽ
	LCD_ShowNum(160,220,R,2);//X���꣬Y���꣬���֣���λ��

	LCD_ShowChar(190,200,'G',0);//X���꣬Y���꣬�ַ������ģʽ
	LCD_ShowNum(190,220,G,2);//X���꣬Y���꣬���֣���λ��

	LCD_ShowChar(220,200,'B',0);//X���꣬Y���꣬�ַ������ģʽ
	LCD_ShowNum(220,220,B,2);//X���꣬Y���꣬���֣���λ��	
	for(a=0;a<30;a++)//����
	{
		for(b=0;b<30;b++)//����
		{
			LCD_DrawPoint(a+300,b+200,(R<<11)|(G<<5)|B);
		}
	}
}
void Picture_String()//ͼƬ->����table_picture      �и�BUG���ȷ��Ű�
{
	u16 a,b,num1;
	for(a=0;a<150;a++)//����
	{
		table_picture[a]=0x00;	
	}	
	for(a=0;a<50;a++)//50��
	{
		for(b=0;b<24;b++)//24��
		{
			num1=LCD_ReadPoint(b+296,a+191);
			if(num1==0xffff)
			{
				table_picture[b/8+a*3]|=(1<<(7-b%8));
			}
		}				
	}
}
void String_Picture_All()//������->ͼƬ
{
	u16 a,b,e,i,num1;

	for(a=0;a<42;a++)//36
	{
		for(e=0;e<50;e++)//50��
		{
			for(i=0;i<24;i++)//24��
			{
				if(Table[150*a+i/8+e*3]&(1<<(7-i%8)))
				{
					num1=0xffff;
				}
				else
				{
					num1=0x0000;
				}
				LCD_DrawPoint(i+296,e+191,num1);//����
			}				
		}
		delay_ms(500);
	}
}
u8 MoShiShiBie_All(u8 begin,u8 end)//�ַ�ƥ�䣬ģʽʶ��,ѡ����ƥ��begin-end
{
	u16 Compare_num,num_save;
	u8 a,b,e,a_save,st1,st2,s1,s2;
	int num1;
	for(a=begin;a<end;a--)					//36
	{
		num1=0;
		for(b=0;b<110;b++)
		{
			st1=table_picture[b];
			st2=Table[150*a+b];
			for(e=0;e<8;e++)
			{
				s1=st1&(1<<e);
				s2=st2&(1<<e);
				if(s1==s2) num1++;
				if(s1!=s2) num1--;
			}
		}
		if(num_save<num1)
		{
			num_save=num1;
			a_save=a;
		}
		LCD_ShowNum(50,220,a,2);//��ʾƥ����ַ���"a"			<������>
		LCD_ShowNum(70,220,num1,4);//��ʾƥ�����ȷ������
		LCD_ShowNum(120,220,num_save,4);//ƥ������ֵ��ʾ					
	}
	return a_save;
}
void String_Picture()//����->ͼƬ
{
	u16 a,b,e,num1;
	
	for(a=0;a<150;a++)//150��
	{
		for(b=0;b<24;b++)//24��
		{
			if(talble_0[b/8+a*3]&(1<<(7-b%8)))
			{
				num1=0xffff;
			}
			else
			{
				num1=0x0000;
			}
			LCD_DrawPoint(b+296,a+191,num1);//����
		}				
	}	
}
void WordShow(u8 num,u16 x,u16 y)//��ʾ����16*16
{
	u16 a,b,e,num1;
	u8 table1[32];
	if(num==1)
	{
		for(a=0;a<32;a++)
		{
			table1[a]=table_yu[a];	
		}		
	}
	if(num==2)
	{
		for(a=0;a<32;a++)
		{
			table1[a]=table_min[a];	
		}		
	}
	if(num==3)
	{
		for(a=0;a<3;a++)
		{
			table1[a]=table_lu[a];	
		}		
	}
	if(num==4)
	{
		for(a=0;a<32;a++)
		{
			table1[a]=table_zhe[a];	
		}		
	}
	if(num==5)
	{
		for(a=0;a<32;a++)
		{
			table1[a]=table_shan[a];	
		}		
	}
	if(num==6)
	{
		for(a=0;a<32;a++)
		{
			table1[a]=table_cuan[a];	
		}		
	}
	for(a=0;a<16;a++)
	{
		for(b=0;b<16;b++)
		{
			if(table1[b/8+a*2]&(1<<(7-b%8)))
			{
				num1=0xffff;
			}
			else
			{
				num1=0x0000;
			}
			LCD_DrawPoint(b+x,a+y,num1);//����
		}				
	}	
}
u8 ZhiFuFenGe()//�ַ��ָ�,���طָ���ַ������������жϺϷ���
{
	u16 a,b;
	u8 i;//ͳ�Ʒָ���ַ���������Ϊ9˵���ָ�����
	i=0;//������0һ�£���Ȼ���д�
	for(b=Max_blue;b>Min_blue;b--)
	{
		if(TableChangePoint_320[b]==0)//��϶�ָ�
		{
			for(a=Min_ChangePoint_240;a<Max_ChangePoint_240;a++)//����--������
			{
 				LCD_DrawPoint(b,a+1,0x001f);
			}
			i--;b++;
			while(TableChangePoint_320[b]==0)
			{
				b--;
				if(b<=Min_blue) break;
			}
		}
	}
	i--;
	LCD_ShowNum(30,220,i,2);//��ʾ�ָ���ַ�����+1��8������ֵ
	return i;
}
void GuiYi(u16 k,u16 kk)//��һ�� 25*50
{
	u16 a,b,e;
	u16 num;//�����ȡ����
	u8 Mo,Yu;//ȡ����ȡģ
	u8 num1,num2,num3;
	u8 Mo_1;//
	u8 Min_240,Max_240;//����ַ����������
	
	if((k-kk)<25)
	{
		//����ַ�
		Min_240=Min_ChangePoint_240+1;
		Max_240=Max_ChangePoint_240-1;
		while(Min_240++)//����󣬵õ�: Min_240
		{
			for(b=kk-10;b<k;b++)//kk1��k1                                
			{
				num=LCD_ReadPoint(b,Min_240);
				if(num) break;
			}
			if(num) break;
		}
		while(Max_240--)//����󣬵õ�: Max_240
		{
			for(b=kk+1;b<k;b++)//kk1��k1                                
			{
				num=LCD_ReadPoint(b,Max_240);
				if(num) break;
			}
			if(num) break;
		}
		Min_240-=1;
		Max_240+=2;
		LCD_DrawPoint(kk,Min_240,0xffff);//
		LCD_DrawPoint( k,Max_240,0xffff);//
		//��ʾ���Ƶ�ͼƬ
		num3=0;
		for(a=Min_240+1;a<Max_240;a++)
		{
			num2=10;
			for(b=kk+1;b<k;b++)//kk1��k1                                   +1
			{
				num=LCD_ReadPoint(b,a);
				LCD_DrawPoint(271-(k-kk-1)+num2,191+num3,num);//��������ֵ
				num2++;
			}
			num3++;
		}
		num3=0;
		Mo=(24-(k-kk-1))/(k-kk-1);//ȡģ
		Yu=(24-(k-kk-1))%(k-kk-1);//ȡ��
		if(Yu!=0) 
		{
			Mo_1=24/Yu;//ƽ��Mo_1�����أ�����һ�����أ���7+1
		}
// 		LCD_ShowNum(30,20,Mo,3);//��ʾģ		<������>
// 		LCD_ShowNum(70,20,Yu,3);//��ʾ��
// 		LCD_ShowNum(100,20,(k1-kk1),3);//��ʾ��ֵ

		for(a=Min_240+1;a<Max_240;a++)//��Ŵ�Ϊ25����  =??
		{//��
			num2=10;
			Yu=(24-(k-kk-1))%(k-kk-1);//ȡ��
			
			for(b=kk+1;b<k;b++)//kk1��k1                                   +1
			{
				num=LCD_ReadPoint(b,a);
				LCD_DrawPoint(271+num2,191+num3,num);
				num2++;
				Mo=(24-(k-kk-1))/(k-kk-1);//ȡģ
				while(Mo)
				{
					LCD_DrawPoint(271+num2,191+num3,num);
					Mo--;
					num2++;
				}
				if(Yu!=0)//��������
				{	
					if(((num2+1)%Mo_1==0) && (num2!=1))//�Ĳ���ĵط�7+1
					{
						LCD_DrawPoint(271+num2,191+num3,num);
						Yu--;
						num2++;
					}
				}
			}
			num3--;
		}
		LCD_DrawPoint(271,191,0x07E0);//��ǵ㣬4������
		LCD_DrawPoint(271,240,0x07E0);
		LCD_DrawPoint(295,191,0x07E0);
		LCD_DrawPoint(295,240,0x07E0);
//��������
		if((Max_240-Min_240)<50)
		{
			Mo=(50-(Max_240-Min_240+1))/(Max_240-Min_240+1);//ȡģ
			Yu=(50-(Max_240-Min_240+1))%(Max_240-Min_240+1);//ȡ��
			Mo_1=50/Yu;
			
// 			LCD_ShowNum(30,170,Mo,3);//					<������>
// 			LCD_ShowNum(70,170,Yu,3);//
// 			LCD_ShowNum(100,170,Max_ChangePoint_240-Min_ChangePoint_240,3);//
			num2=0;
			for(a=0;a<(Max_240-Min_240);a++)//����ͼ��,���Ƿ�Χ�Ƿ���Ҫ����������
			{//��
				for(b=271;b<=295;b++)//271��ʼ���ƣ�295�Ž���
				{
					num=LCD_ReadPoint(b,a+191);
					LCD_DrawPoint(b+25,191+num2,num);//��������ֵ
				}
				num2++;
				while(Mo)
				{
					for(b=271;b<=295;b++)//271��ʼ���ƣ�295�Ž���
					{
						num=LCD_ReadPoint(b,a+191);
						LCD_DrawPoint(b+25,191+num2+a,num);//��������ֵ
					}
					Mo--;
					num2++;
				}
				if(Yu!=0)
				{
					if((((num2+1) % Mo_1)==0)&& (num2!=1))
					{
						for(b=271;b<=295;b++)//271��ʼ���ƣ�295�Ž���
						{
							num=LCD_ReadPoint(b,a+191);
							LCD_DrawPoint(b+25,191+num2,num);//��������ֵ
						}
						Yu--;
						num2++;
					}
				}					
			}
		}
		LCD_DrawPoint(320,191,0xf800);//��ǵ㣬1������
	}
}
void ZhiFuShiBie()//�ַ�ʶ��
{
	u16 a,b,e;
	u16 i,u;
	u8 Result;//ʶ����

	for(b=Max_blue-1;b>Min_blue;b--)//��������ʶ�𣬻�ȡ�����ַ���k,kKֵ
	{
		while(TableChangePoint_320[b]==0) b--;//ȡ��1���ַ�
		k1=b+1;//+1
		while(TableChangePoint_320[b]>0) b--;
		kk1=b;
		if((k1-kk1)<4)//ʡ�Ե����������ص�λ��
		{
			while(TableChangePoint_320[b]==0) b--;//
			k1=b+1;//+1
			while(TableChangePoint_320[b]>0) b--;
			kk1=b;
		}
		while(TableChangePoint_320[b]==0) b--;//ȡ��2���ַ�
		k2=b+1;
		while(TableChangePoint_320[b]>0) b--;
		kk2=b;
		if((k2-kk2)<4)//ʡ�Ե���3�����ص�λ��
		{
			while(TableChangePoint_320[b]==0) b--;//
			k2=b+1;//+1
			while(TableChangePoint_320[b]>0) b--;
			kk2=b;
		}
		while(TableChangePoint_320[b]==0) b--;//ȡ��3���ַ�
		k3=b+1;//+1
		while(TableChangePoint_320[b]>0) b--;
		kk3=b;
		if((k3-kk3)<4)//ʡ�Ե���3�����ص�λ��
		{
			while(TableChangePoint_320[b]==0) b--;//
			k3=b+1;//+1
			while(TableChangePoint_320[b]>0) b--;
			kk3=b;
		}
		while(TableChangePoint_320[b]==0) b--;//ȡ��4���ַ�
		k4=b+1;
		while(TableChangePoint_320[b]>0) b--;
		kk4=b;
		if((k4-kk4)<4)//ʡ�Ե���3�����ص�λ��
		{
			while(TableChangePoint_320[b]==0) b--;//
			k4=b+1;//+1
			while(TableChangePoint_320[b]>0) b--;
			kk4=b;
		}
		while(TableChangePoint_320[b]==0) b--;//ȡ��5���ַ�
		k5=b+1;//+1
		while(TableChangePoint_320[b]>0) b--;
		kk5=b;
		if((k5-kk5)<4)//ʡ�Ե���3�����ص�λ��
		{
			while(TableChangePoint_320[b]==0) b--;//
			k5=b+1;//+1
			while(TableChangePoint_320[b]>0) b--;
			kk5=b;
		}
		while(TableChangePoint_320[b]==0) b--;//ȡ��6���ַ�
		k6=b+1;
		while(TableChangePoint_320[b]>0) b--;
		kk6=b;
		while(TableChangePoint_320[b]==0) b--;//ȡ��7���ַ�
		k7=b+1;//+1
		while(TableChangePoint_320[b]>0) b--;
		kk7=b;
		if((k7-kk7)<4)//ʡ�Ե���3�����ص�λ��
		{
			while(TableChangePoint_320[b]==0) b--;//
			k7=b+1;//+1
			while(TableChangePoint_320[b]>0) b--;
			kk7=b;
		}
		while(TableChangePoint_320[b]==0) b--;//ȡ��8���ַ�
		k8=b+1;
 		while(TableChangePoint_320[b]>0) 
		{
			if(b<=Min_blue)
			{
				break;
			}
			b--;
		}
		kk8=b;
		b=Min_blue;//�Է���һ��������forѭ������
	}
	for(a=Min_ChangePoint_240;a<Max_ChangePoint_240;a++)//����
	{
		LCD_DrawPoint(k1,a+1,0x001f);
		LCD_DrawPoint(kk1,a+1,0x001f);
		LCD_DrawPoint(k2,a+1,0x001f);
		LCD_DrawPoint(kk2,a+1,0x001f);
		LCD_DrawPoint(k3,a+1,0x001f);
		LCD_DrawPoint(kk3,a+1,0x001f);
		LCD_DrawPoint(k4,a+1,0x001f);
		LCD_DrawPoint(kk4,a+1,0x001f);
		LCD_DrawPoint(k5,a+1,0x001f);
		LCD_DrawPoint(kk5,a+1,0x001f);
		LCD_DrawPoint(k6,a+1,0x001f);
		LCD_DrawPoint(kk6,a+1,0x001f);
		LCD_DrawPoint(k7,a+1,0x001f);
		LCD_DrawPoint(kk7,a+1,0x001f);
		LCD_DrawPoint(k8,a+1,0x001f);
		LCD_DrawPoint(kk8,a+1,0x001f);
	}
//��һ��������СΪ25*50
	
//��1���ַ���
	GuiYi(k1,kk1);//��һ�� 24*24
	Picture_String();//ͼƬ->����
	Result=MoShiShiBie_All(0,36);//�ַ�ƥ�䣬ģʽʶ��,����a,0<= a <36
	if(Result<10)
	{
		LCD_ShowNum(240,220,table_char[Result],1);
	}
	else
	{
		LCD_ShowChar(240,220,table_char[Result],0);
	}
	table_cardMeasure[6]=Result;//����ʶ��ĳ��ƽ��
	
//��2���ַ���
	GuiYi(k2,kk2);//��һ�� 25*50
	Picture_String();//ͼƬ->����
	Result=MoShiShiBie_All(0,36);//�ַ�ƥ�䣬ģʽʶ��
	if(Result<10)
	{
		LCD_ShowNum(230,220,table_char[Result],1);
	}
	else
	{
		LCD_ShowChar(230,220,table_char[Result],0);
	}
	table_cardMeasure[5]=Result;//����ʶ��ĳ��ƽ��
	
	GuiYi(k3,kk3);//��һ�� 25*50
	Picture_String();//ͼƬ->����
	Result=MoShiShiBie_All(0,36);//�ַ�ƥ�䣬ģʽʶ��
	if(Result<10)
	{
		LCD_ShowNum(220,220,table_char[Result],1);
	}
	else
	{
		LCD_ShowChar(220,220,table_char[Result],0);
	}
	table_cardMeasure[4]=Result;//����ʶ��ĳ��ƽ��
	
	GuiYi(k4,kk4);//��һ�� 25*50
	Picture_String();//ͼƬ->����
	Result=MoShiShiBie_All(0,36);//�ַ�ƥ�䣬ģʽʶ��
	if(Result<10)
	{
		LCD_ShowNum(210,220,table_char[Result],1);
	}
	else
	{
		LCD_ShowChar(210,220,table_char[Result],0);
	}
	table_cardMeasure[3]=Result;//����ʶ��ĳ��ƽ��
	
	GuiYi(k5,kk5);//��һ�� 25*50
	Picture_String();//ͼƬ->����
	Result=MoShiShiBie_All(0,36);//�ַ�ƥ�䣬ģʽʶ��
	if(Result<10)
	{
		LCD_ShowNum(200,220,table_char[Result],1);
	}
	else
	{
		LCD_ShowChar(200,220,table_char[Result],0);
	}
	table_cardMeasure[2]=Result;//����ʶ��ĳ��ƽ��
	LCD_ShowChar(190,220,'.',0);

	GuiYi(k7,kk7);//��һ�� 25*50
	Picture_String();//ͼƬ->����
	Result=MoShiShiBie_All(10,36);//�ַ�ƥ�䣬ģʽʶ��ֻƥ����ĸ
	if(Result<10)
	{
		LCD_ShowNum(180,220,table_char[Result],1);
	}
	else
	{
		LCD_ShowChar(180,220,table_char[Result],0);
	}
	table_cardMeasure[1]=Result;//����ʶ��ĳ��ƽ��
	
	GuiYi(k8,kk8);//��һ�� 25*50					���һ�����֣�����ʶ��
	Picture_String();//ͼƬ->����
	Result=MoShiShiBie_All(36,42);//�ַ�ƥ�䣬ƥ�人��
	WordShow(Result-35,160,220);//��ʾ����
	table_cardMeasure[0]=Result-35;//����ʶ��ĳ��ƽ��
	//ʶ�����
	//	while(1);
	//��ƥ���ѱ���ĳ��ƺ�
//	for(u=0;u<5;u++)
//	{
//		for(i=0;i<7;i++)
//		{
//			if(table_card[u][i]!=table_cardMeasure[i]) i=8;//�˳�forѭ��
//			
//		}	
//		if(i==7)//ƥ��ɹ�
//		{
//			LCD_Fill(0x00);//����
//			Show_Title();//��ʾ����
//			Show_Card(u);//��ʾ�ڼ��鳵��			
//			delay_ms(15000);
//			u=5;
//		}
//	}
//	if(i==9)//��ƥ�䳵�ƣ��򱣴泵��
//	{
//		i=0;
//		while(1)
//		{
//			if(GPIO_ReadInputDataBit(KEY1_PORT,KEY1_PIN)==0) break;
//			LCD_ShowNum(30,220,i/100,2);
//			if(i==300)														//��������
//			{
//				for(u=0;u<5;u++)
//				{
//					if(table_card[u][0]==0)
//					{
//						for(i=0;i<7;i++)
//						{
//							table_card[u][i]=table_cardMeasure[i];
//						}					
//						u=5;//�˳�ѭ��
//					}
//				}
//				LCD_Fill(0x00);//����
//				Show_Title();//��ʾ����
//				Show_Card(0);//��ʾ�ڼ��鳵��
//				Show_Card(1);
//				Show_Card(2);
//				Show_Card(3);
//				Show_Card(4);
//				delay_ms(10000);
//				break;
//			}
//			delay_ms(1);
//			i++;
//		}
//	}
	
}
void Copy_Card()//���Ƴ���Max_aChangePoint_reset_1,Min_blue
{
	u16 a,b,num;
	for(a=Min_aChangePoint_reset;a<=Max_aChangePoint_reset;a++)
	{	
		for(b=Min_blue;b<Max_blue;b++)
		{
			num=LCD_ReadPoint(b,a);//��ȡ����ֵ
			LCD_DrawPoint(b,50+a,num);
		}
		
	}
}
void ChangePoint_Show_240()//240�����������ʾ
{
	for(a=0;a<240;a++)//�����ο���10��20��30
	{
		LCD_DrawPoint(10,a,0x63<<5);//10
		LCD_DrawPoint(20,a,0x63<<5);//20
		LCD_DrawPoint(30,a,0x63<<5);//30
	}
	
	for(a=0;a<240;a++)//��ʾ��Ӧ�ĺ��������								
	{
		LCD_DrawPoint(TableChangePoint_240[a],a,0xf800);//�������ʾ����ɫ���
		if(TableChangePoint_240[a]>=15)					//������������ֵ���趨       ��ֵ����3-��1��
		{
			for(b=35;b<40;b++)						//��ʾ�ﵽ��ֵ��׼�ĵ�
			{
				LCD_DrawPoint(b,a,0x63<<5);//Green			
			}
		}
	}
}
void ChangePoint_Analysis_240()//240������������������������Ƶ����±߽�͸߶�ֵ
{
	Min_ChangePoint_240=240;Max_ChangePoint_240=0;
	
	for(a=0;a<240;a++)//240ɨ��	����ȡ������ֵ	��Min_ChangePoint_240��Max_ChangePoint_240				
	{
		while(TableChangePoint_240[a]<=15)									//��ֵ����3-��2��
		{
			a++;
		}
		Min_ChangePoint_240=a;
		while(TableChangePoint_240[a]>15)									//��ֵ����3-��3��
		{
			a++;
		}
		Max_ChangePoint_240=a;
		if(Max_ChangePoint_240-Min_ChangePoint_240>=15) a=240;//��������ֵ   	//��ֵ����2-��1��
	}
	Min_ChangePoint_240=Min_ChangePoint_240-3;//����΢��3����
	Max_ChangePoint_240=Max_ChangePoint_240+2;//����΢��2����
	for(a=30;a<280;a++)//��ʾ�Ͻ���				
	{
		LCD_DrawPoint(a,Max_ChangePoint_240,0x001f);
	}
	for(a=30;a<280;a++)//��ʾ�½���						
	{
		LCD_DrawPoint(a,Min_ChangePoint_240,0x001f);
	}
	for(a=30;a<280;a++)//��ʾ50,�ο�50����λ�ô�������λ�ò�Ҫ��������ߣ���ò����ַ��Ĺ�һ������						
	{
		LCD_DrawPoint(a,Min_ChangePoint_240+50,0xf800);
	}
	flag_MaxMinCompare=1;
	if(Min_ChangePoint_240>Max_ChangePoint_240)//�жϺϷ���1����Сֵ>���ֵ
	{
		flag_MaxMinCompare=0;
	}
	if(Min_ChangePoint_240==240||Max_ChangePoint_240==0)//�жϺϷ���2��ֵû�����¸�ֵ
	{
		flag_MaxMinCompare=0;
	}
	if(Max_ChangePoint_240-Min_ChangePoint_240<15)		//�жϺϷ���3��			//��ֵ����2-��2��
	{
		flag_MaxMinCompare=0;
	}
}
void ChangePoint_Analysis_Blue()//320��ɫ�������,���ö�ȡ���أ��ý��Min_blue,Max_blue
{																//���������Ƶ����ұ߽磬�����Ƶĺ�����
	u16 a,b,num_color;
	u16 min_320,max_320;//���е���С�����ֵ
	
	Min_blue=0;Max_blue=320;
	min_320=320;max_320=0;
	
	for(a=Min_ChangePoint_240;a<Max_ChangePoint_240;a++)								
	{
		for(b=30;b<290;b++)//���õ�320    for(b=30;b<320;b++)
		{
			num_color=LCD_ReadPoint(b,a);//��ȡ���أ������Ż��ٶ��д����� ��ɨ�跽��Ҳ���Ż����������ٶ�
			RGB_HSV(num_color);//RGB565תHSV
			if( 250>H && H>190 && 60>S && S>15 && 100>V && V>45)//HSV ��ֵ
			{
				if(b<min_320)//��ú����Min��Maxֵ������ɫ���Ƶ����ұ߽�
				{
					min_320=b;
				}
				if(b>max_320)
				{
					max_320=b;
				}
			}
		}
	}
	Min_blue=min_320;//��ȡ���е����ֵ//����һ��
	Max_blue=max_320-50;//��ȡ���е���Сֵ//����һ��
	
	for(a=Min_ChangePoint_240;a<Max_ChangePoint_240;a++)//��ʾ�����				
	{
		LCD_DrawPoint(Min_blue,a,0xf8);//LCD_DrawPoint(Min_blue,a,0xf800);
	}
	for(a=Min_ChangePoint_240;a<Max_ChangePoint_240;a++)//��ʾ�ҽ���					
	{
		LCD_DrawPoint(Max_blue,a,0xf800);
	}
// 	delay_ms(6000);
}
void Card_101Show()//���������ֵ����ʾ
{
	u16 a,b,num_color;
	u8 R1,G1,B1;
	u8 Mid_ChangePoint_240;
	u8 max_R,max_G,max_B,min_R,min_G,min_B;
	u8 mid_R,mid_G,mid_B;
	
	max_R=0;max_G=0;max_B=0;
	min_R=30;min_G=60;min_B=30;
	
	Mid_ChangePoint_240=(Min_ChangePoint_240+Max_ChangePoint_240)/2;
	for(b=Min_blue;b<Max_blue;b++)
	{
		num_color=LCD_ReadPoint(b,Mid_ChangePoint_240);//��ȡ���أ������Ż��ٶ��д����� ��ɨ�跽��Ҳ���Ż����������ٶ�
		R1=num_color>>11;
		G1=(num_color>>5)&0x3F;
		B1=num_color&0x1F;
		if( (R1>10) && (G1>25) && (B1>15) && (R1<=30) && (G1<=60) && (B1<=30) )//��ֵ��,����ֵ��25.55.25���Ϻ�����ֵ��21,47,21��
		{
			if(max_R<R1) max_R=R1;//������ֵ����Сֵ
			if(max_G<G1) max_G=G1;
			if(max_B<B1) max_B=B1;
			
			if(min_R>R1) min_R=R1;
			if(min_G>G1) min_G=G1;
			if(min_B>B1) min_B=B1;		
		}
	}
	mid_R=(max_R+min_R)/2;
	mid_G=(max_G+min_G)/2;
	mid_B=(max_B+	min_B)/2;
// 	LCD_ShowNum(30,200,max_R,2);//X���꣬Y���꣬���֣���λ��             <������>
// 	LCD_ShowNum(70,200,max_G,2);//X���꣬Y���꣬���֣���λ��
// 	LCD_ShowNum(100,200,max_B,2);//X���꣬Y���꣬���֣���λ��
// 	
// 	LCD_ShowNum(30,220,min_R,2);//X���꣬Y���꣬���֣���λ��
// 	LCD_ShowNum(70,220,min_G,2);//X���꣬Y���꣬���֣���λ��
// 	LCD_ShowNum(100,220,min_B,2);//X���꣬Y���꣬���֣���λ��
	
	for(a=Min_ChangePoint_240;a<Max_ChangePoint_240;a++)								
	{
		for(b=Min_blue;b<Max_blue;b++)
		{
			num_color=LCD_ReadPoint(b,a);//��ȡ���أ������Ż��ٶ��д����� ��ɨ�跽��Ҳ���Ż����������ٶ�
			R1=num_color>>11;
			G1=(num_color>>5)&0x3F;
			B1=num_color&0x1F;
			if((R1>=mid_R) && (G1>=mid_G) && (B1>=mid_B))//��ֵ��,����ֵ��25.55.25���Ϻ�����ֵ��21,47,21��
			{
				LCD_DrawPoint(b,a,0xffff);
			}
			else
			{
				LCD_DrawPoint(b,a,0x0000);
			}
		}
	}
	delay_ms(20000);
}
void ChangePoint_Show_320()//320�����������ʾ
{
	for(a=0;a<320;a++)//��ʾ��Ӧ�ĺ��������								
	{
		if(TableChangePoint_320[a]==0)
		{
			LCD_DrawPoint(a,0,0x001F);//�������ʾ����ɫ���
		}
		else
		{
			LCD_DrawPoint(a,TableChangePoint_320[a],0xf800);//�������ʾ����ɫ���
		}
		

	}
}
void ChangePoint_Analysis_320()//��ɫ�����У�320��������,���TableChangePoint_320[b]���
{								//(�ȶ�ֵ�������жϰ׵������=0���Ƿָ��ߣ�
	u16 a,b,num_color;
	u8 R1,G1,B1;
	u8 Mid_ChangePoint_240;
	u8 max_R,max_G,max_B,min_R,min_G,min_B;
	u8 mid_R,mid_G,mid_B;
	
	max_R=0;max_G=0;max_B=0;
	min_R=30;min_G=60;min_B=30;
	
	Mid_ChangePoint_240=(Min_ChangePoint_240+Max_ChangePoint_240)/2;
	for(b=Min_blue;b<Max_blue;b++)
	{
		num_color=LCD_ReadPoint(b,Mid_ChangePoint_240);//��ȡ���أ������Ż��ٶ��д����� ��ɨ�跽��Ҳ���Ż����������ٶ�
		R1=num_color>>11;
		G1=(num_color>>5)&0x3F;
		B1=num_color&0x1F;
		if( (R1>10) && (G1>25) && (B1>15) && (R1<=30) && (G1<=60) && (B1<=30) )//��ֵ��,����ֵ��25.55.25���Ϻ�����ֵ��21,47,21��
		{
			if(max_R<R1) max_R=R1;//������ֵ����Сֵ
			if(max_G<G1) max_G=G1;
			if(max_B<B1) max_B=B1;
			
			if(min_R>R1) min_R=R1;
			if(min_G>G1) min_G=G1;
			if(min_B>B1) min_B=B1;		
		}
	}
	mid_R=(max_R+min_R)/2;
	mid_G=(max_G+min_G)/2;
	mid_B=(max_B+	min_B)/2;
// 	LCD_ShowNum(30,200,max_R,2);//X���꣬Y���꣬���֣���λ��		<������>
// 	LCD_ShowNum(70,200,max_G,2);//X���꣬Y���꣬���֣���λ��
// 	LCD_ShowNum(100,200,max_B,2);//X���꣬Y���꣬���֣���λ��
// 	
// 	LCD_ShowNum(30,220,min_R,2);//X���꣬Y���꣬���֣���λ��
// 	LCD_ShowNum(70,220,min_G,2);//X���꣬Y���꣬���֣���λ��
// 	LCD_ShowNum(100,220,min_B,2);//X���꣬Y���꣬���֣���λ��
	for(b=0;b<320;b++)//����������������������
	{
		TableChangePoint_320[b]=0;
	}
	for(a=Min_ChangePoint_240;a<Max_ChangePoint_240;a++)								
	{
		for(b=Min_blue+1;b<Max_blue;b++)
		{
			num_color=LCD_ReadPoint(b,a);//��ȡ���أ������Ż��ٶ��д����� ��ɨ�跽��Ҳ���Ż����������ٶ�
			R1=num_color>>11;
			G1=(num_color>>5)&0x3F;
			B1=num_color&0x1F;
			if((R1>=mid_R) && (G1>=mid_G) && (B1>=mid_B))//��ֵ��,����ֵ��25.55.25���Ϻ�����ֵ��21,47,21��
			{
				LCD_DrawPoint(b,a,0xffff);
				TableChangePoint_320[b]++;//��ɫ�������+1
			}
			else
			{
				LCD_DrawPoint(b,a,0x0000);
			}
		}
	}
}

void Data_LCD_ColorChange()//����ͷɨ��
{
	u8 i;
	for(a=0;a<240;a++)//����������������������
	{
		TableChangePoint_240[a]=0;
	}
	Min_blue=320;//��ʼ����¼��ɫ���������ֵ
	Max_blue=0;
	
	LCD_SetWindows(0,0,320,240);//������ʾ����
	GPIO_WriteBit(LCD_RS_PORT, LCD_RS_PIN,1);//��־������д��

	while(GPIO_ReadInputDataBit(GPIOC,GPIO_Pin_0)==1);
	GPIO_WriteBit(FIFO_WRST_PORT, FIFO_WRST_PIN, 0);
	GPIO_WriteBit(FIFO_WRST_PORT, FIFO_WRST_PIN, 1);
	GPIO_WriteBit(FIFO_WR_PORT, FIFO_WR_PIN, 1);
	while(GPIO_ReadInputDataBit(GPIOC,GPIO_Pin_0)==0);
	while(GPIO_ReadInputDataBit(GPIOC,GPIO_Pin_0)==1);
	GPIO_WriteBit(FIFO_WR_PORT, FIFO_WR_PIN, 0);
	
	FIFO_Reset_Read_Addr(); 
//	LED();
	
	for (a=0;a<240;a++)
	{
		for(b=0;b<320;b++)
		{
			GPIOC->BRR =1<<4;
			AA=GPIOA->IDR;						
			GPIOC->BSRR =1<<4;
			
			GPIOC->BRR =1<<4;
			BB=GPIOA->IDR&0x00ff;								
			GPIOC->BSRR =1<<4;	
			
			color=(AA<<8)|BB;

			R=color>>11;
			G=(color>>5)&0x3f;
			B=color&0x1f;
			
			if((R>R_a) && (G>=G_a) && (B>=B_a))//��ֵ��,����ֵ��25.55.25���Ϻ�����ֵ��21,47,21��
			{
				color=0xffff;
			}
			else
			{
				color=0x0000;
			}
			
			if(color!=color_save)//�����
			{
				TableChangePoint_240[a]++;		//������������+1
			}
			color_save=color;//��������ֵ������һ���жϺͱȽ�
			
			color=(AA<<8)|BB;//��ԭɫ��
					
			LCD_DATA_PORT->ODR = color;
			GPIOC->BRR =1<<11;
			GPIOC->BSRR =1<<11;
		}
	}
	
	ChangePoint_Show_240();//240�����������ʾ
	ChangePoint_Analysis_240();	//��������
							//����flag_MaxMinCompare��Min_ChangePoint_240��Max_ChangePoint_240
	if(flag_MaxMinCompare==1)//�����ɸѡ�ɹ�
	{
		ChangePoint_Analysis_Blue();//320��ɫ�������,���ö�ȡ���أ��ý��Min_blue,Max_blue
		if(Min_blue>Max_blue) flag_MaxMinCompare=0;//���к������ж�1
		if((Min_blue>290)||(Max_blue>290)) flag_MaxMinCompare=0;//���к������ж�2
	}
	if(flag_MaxMinCompare==1)//�����ɸѡ�ɹ�
	{
//		ChangePoint_Analysis_Blue();//320��ɫ�������,���ö�ȡ���أ��ý��Min_blue,Max_blue
// 		Card_101Show();//���������ֵ����ʾ 
		ChangePoint_Analysis_320();//��ɫ�����У�320��������,��ã�TableChangePoint_320[b]���
		ChangePoint_Show_320();//320�����������ʾ
		i=ZhiFuFenGe();
		
		if(i==8)//�ַ��ָ�,���طָ���ַ������������жϺϷ���
		{
			ZhiFuShiBie();//�ַ�ʶ��	
		}
		else
		{
			LCD_Fill(0x6666);//��������ʾMeasure Faill
			LCD_ShowChar(8*1,200,'M',0);
			LCD_ShowChar(8*2,200,'e',0);
			LCD_ShowChar(8*3,200,'a',0);
			LCD_ShowChar(8*4,200,'s',0);
			LCD_ShowChar(8*5,200,'u',0);
			LCD_ShowChar(8*6,200,'r',0);
			LCD_ShowChar(8*7,200,'e',0);
			
			LCD_ShowChar(8*9,200,'F',0);
			LCD_ShowChar(8*10,200,'a',0);
			LCD_ShowChar(8*11,200,'i',0);
			LCD_ShowChar(8*12,200,'l',0);
			LCD_ShowChar(8*13,200,'l',0);

			delay_ms(800);
		}
	}
}
void Data_LCD_ColorChange_Test()//����ͷɨ�����
{
	for(a=0;a<240;a++)//����������������������
	{
		TableChangePoint_240[a]=0;
	}
	Min_blue=320;//��ʼ����¼��ɫ���������ֵ
	Max_blue=0;
	
	LCD_SetWindows(0,0,320,240);//������ʾ����
	GPIO_WriteBit(LCD_RS_PORT, LCD_RS_PIN,1);//��־������д��

	while(GPIO_ReadInputDataBit(GPIOC,GPIO_Pin_0)==1);
	GPIO_WriteBit(FIFO_WRST_PORT, FIFO_WRST_PIN, 0);
	GPIO_WriteBit(FIFO_WRST_PORT, FIFO_WRST_PIN, 1);
	GPIO_WriteBit(FIFO_WR_PORT, FIFO_WR_PIN, 1);
	while(GPIO_ReadInputDataBit(GPIOC,GPIO_Pin_0)==0);
	while(GPIO_ReadInputDataBit(GPIOC,GPIO_Pin_0)==1);
	GPIO_WriteBit(FIFO_WR_PORT, FIFO_WR_PIN, 0);
	
	FIFO_Reset_Read_Addr(); 
//	LED();
	
	for (a=0;a<240;a++)
	{
		for(b=0;b<320;b++)
		{
			GPIOC->BRR =1<<4;
			AA=GPIOA->IDR;						
			GPIOC->BSRR =1<<4;
			
			GPIOC->BRR =1<<4;
			BB=GPIOA->IDR&0x00ff;								
			GPIOC->BSRR =1<<4;	
			
			color=(AA<<8)|BB;

			R=color>>11;
			G=(color>>5)&0x3f;
			B=color&0x1f;
			
			if((R>R_a) && (G>=G_a) && (B>=B_a))//��ֵ��,����ֵ��25.55.25���Ϻ�����ֵ��21,47,21��
			{
				color=0xffff;
			}
			else
			{
				color=0x0000;
			}
			
			if(color!=color_save)//�����
			{
				TableChangePoint_240[a]++;		//������������+1
			}
			color_save=color;//��������ֵ������һ���жϺͱȽ�
			
			color=(AA<<8)|BB;//��ԭɫ��
					
			LCD_DATA_PORT->ODR = color;
			GPIOC->BRR =1<<11;
			GPIOC->BSRR =1<<11;
		}
	}
	ChangePoint_Show_240();//240�����������ʾ
	ChangePoint_Analysis_240();	//��������
}
void Data_LCD_ColorPut()//��ɫ����
{
	LCD_SetWindows(0,0,320,240);//������ʾ����
	GPIO_WriteBit(LCD_RS_PORT, LCD_RS_PIN,1);//��־������д��

	while(GPIO_ReadInputDataBit(GPIOC,GPIO_Pin_0)==1);
	GPIO_WriteBit(FIFO_WRST_PORT, FIFO_WRST_PIN, 0);
	GPIO_WriteBit(FIFO_WRST_PORT, FIFO_WRST_PIN, 1);
	GPIO_WriteBit(FIFO_WR_PORT, FIFO_WR_PIN, 1);
	while(GPIO_ReadInputDataBit(GPIOC,GPIO_Pin_0)==0);
	while(GPIO_ReadInputDataBit(GPIOC,GPIO_Pin_0)==1);
	GPIO_WriteBit(FIFO_WR_PORT, FIFO_WR_PIN, 0);
	
	FIFO_Reset_Read_Addr(); 
//	LED();
	
	for (a=0;a<240;a++)
	{
		for(b=0;b<320;b++)
		{
			GPIOC->BRR =1<<4;
			AA=GPIOA->IDR;						
			GPIOC->BSRR =1<<4;
			
			GPIOC->BRR =1<<4;
			BB=GPIOA->IDR&0x00ff;								
			GPIOC->BSRR =1<<4;	
			
			color=(AA<<8)|BB;
			
			R=color>>11;
			G=(color>>5)&0x3f;
			B=color&0x1f;
			
// 			if(R<25&&G<50&&B<25)
// 			{
// 				color=0xf800;
// 			}
			if(R<29&&G<59&&B<29)
			{
				color=0x0000;
			}
			LCD_DATA_PORT->ODR = color;

			GPIOC->BRR =1<<11;
			GPIOC->BSRR =1<<11;
		}							
	}
}
void PC0_in()//PC0����Ϊ����
{
	GPIO_InitTypeDef gpio_init_struct;//�ṹ��
	gpio_init_struct.GPIO_Mode = GPIO_Mode_IPU;//��������
	gpio_init_struct.GPIO_Speed = GPIO_Speed_50MHz;
	gpio_init_struct.GPIO_Pin = GPIO_Pin_0;
	GPIO_Init(GPIOC, &gpio_init_struct);
}
void Show_Card(u8 i)//��ʾ�ڼ��鳵��
{
	u16 t0;
//��ʾ����
	if(table_card[i][0]!=0)
	{
		WordShow(table_card[i][0],9,i*16+50);//
	}
	
	if(table_card[i][1]<10)
	{
		LCD_ShowNum(25,i*16+50,table_char[table_card[i][1]],1);
	}
	else
	{
		LCD_ShowChar(25,i*16+50,table_char[table_card[i][1]],0);
	}
	LCD_ShowChar(33,i*16+50,'.',0);														//��
	if(table_card[i][2]<10)
	{
		LCD_ShowNum(41,i*16+50,table_char[table_card[i][2]],1);
	}
	else
	{
		LCD_ShowChar(41,i*16+50,table_char[table_card[i][2]],0);
	}
	if(table_card[i][3]<10)
	{
		LCD_ShowNum(49,i*16+50,table_char[table_card[i][3]],1);
	}
	else
	{
		LCD_ShowChar(49,i*16+50,table_char[table_card[i][3]],0);
	}
	if(table_card[i][4]<10)
	{
		LCD_ShowNum(57,i*16+50,table_char[table_card[i][4]],1);
	}
	else
	{
		LCD_ShowChar(57,i*16+50,table_char[table_card[i][4]],0);
	}
	if(table_card[i][5]<10)
	{
		LCD_ShowNum(65,i*16+50,table_char[table_card[i][5]],1);
	}
	else
	{
		LCD_ShowChar(65,i*16+50,table_char[table_card[i][5]],0);
	}
	if(table_card[i][6]<10)
	{
		LCD_ShowNum(73,i*16+50,table_char[table_card[i][6]],1);
	}
	else
	{
		LCD_ShowChar(73,i*16+50,table_char[table_card[i][6]],0);
	}
	t0=table_card[i][7];
	LCD_ShowNum(100,i*16+50,t0,6);//��ʾʱ��

	if(t0<60)
	{
		LCD_ShowNumPoint(168,i*16+50,t0*8);
	}
	else
	{
		LCD_ShowNumPoint(168,i*16+50,t0/60*500+t0%60*8);
	}
	
}
//��ʱ��3�жϷ������	 
void TIM3_IRQHandler(void)
{ 		    		  			    
	if(TIM3->SR&0X0001)//����ж�
	{
		LED();
		if(tim3_num==60)
		{
			if(table_card[0][0]!=0)//��1���ʱ
			{
				table_card[0][7]++;
			}		
			if(table_card[1][0]!=0)//��2���ʱ
			{
				table_card[1][7]++;
			}
			if(table_card[2][0]!=0)//��3���ʱ
			{
				table_card[2][7]++;
			}	
			if(table_card[3][0]!=0)//��4���ʱ
			{
				table_card[3][7]++;
			}	
			if(table_card[4][0]!=0)//��5���ʱ
			{
				table_card[4][7]++;
			}
			tim3_num=0;
		}		
	}			
	tim3_num++;
	TIM3->SR&=~(1<<0);//����жϱ�־λ 	    
}
void TIM3_Configuration(void)
	{
	/* TIM3 clock enable */
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
	/* ---------------------------------------------------------------
	TIM3CLK ��PCLK1=36MHz
	TIM3CLK = 36 MHz, Prescaler = 7200, TIM3 counter clock = 5K,���ı�һ��Ϊ5K,���ھ�Ϊ10K
	--------------------------------------------------------------- */
	/* Time base configuration */
	TIM_TimeBaseStructure.TIM_Period = 10000; //��������һ�������¼�װ�����Զ���װ�ؼĴ������ڵ�ֵ	 ������10000Ϊ1000ms
	TIM_TimeBaseStructure.TIM_Prescaler =(12800-1); //����������ΪTIMxʱ��Ƶ�ʳ�����Ԥ��Ƶֵ  10Khz�ļ���Ƶ��  
	TIM_TimeBaseStructure.TIM_ClockDivision = 0; //����ʱ�ӷָ�:TDTS = Tck_tim
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  //TIM���ϼ���ģʽ
	TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure); //����TIM_TimeBaseInitStruct��ָ���Ĳ�����ʼ��TIMx��ʱ�������λ
	
	/* Enables the Update event for TIM3 */
	//TIM_UpdateDisableConfig(TIM3,ENABLE); 	//ʹ�� TIM3 �����¼� 
	
	/* TIM IT enable */
	TIM_ITConfig(  //ʹ�ܻ���ʧ��ָ����TIM�ж�
		TIM3, //TIM2
		TIM_IT_Update  |  //TIM �ж�Դ
		TIM_IT_Trigger,   //TIM �����ж�Դ 
		ENABLE  //ʹ��
		);
	
	/* TIM3 enable counter */
	TIM_Cmd(TIM3, ENABLE);  //ʹ��TIMx����
}
void Show_Title()//��ʾ����
{
	LCD_ShowChar(35+8*0,10,'N',0);
	LCD_ShowChar(35+8*1,10,'u',0);
	LCD_ShowChar(35+8*2,10,'m',0);
	LCD_ShowChar(35+8*3,10,'b',0);
	LCD_ShowChar(35+8*4,10,'e',0);
	LCD_ShowChar(35+8*5,10,'r',0);
	
	LCD_ShowChar(35+8*9,10,'T',0);
	LCD_ShowChar(35+8*10,10,'i',0);
	LCD_ShowChar(35+8*11,10,'m',0);
	LCD_ShowChar(35+8*12,10,'e',0);

	
	LCD_ShowChar(35+8*17,10,'P',0);
	LCD_ShowChar(35+8*18,10,'r',0);
	LCD_ShowChar(35+8*19,10,'i',0);
	LCD_ShowChar(35+8*20,10,'c',0);
	LCD_ShowChar(35+8*21,10,'e',0);
}

void Uart1_PutChar(u8 ch)
{
	USART_SendData(USART1, (u8) ch);
	while(USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET);
}

void Uart_SendStr (u8 pucStr[], u8 ulNum)
{ 
	u8 m; 
	for(m = 0;m<ulNum;m++) 
	{ 
		Uart1_PutChar(pucStr[m]);
	}      
}

void main(void)
{  
	unsigned int num;
	u16 i;
	Stm32_Clock_Init(16);//��ʼ��ʱ��

	Led_init();			//��ʼ�� LED	
	Lcd_Gpio_Init();
	LCD_Init();	
	
 	Key_init();	//��ʼ�� KEY1
	PC0_in();//PC0����Ϊ����
	
	OV7670_Gpio_Init();//OV7670���ų�ʼ�������ڴ��ڳ�ʼ��ǰ��
	GPIO_WriteBit(FIFO_OE_PORT, FIFO_OE_PIN, 0);
	USART1_init();//��ʼ������	
// 	PC0_in();//PC0����Ϊ����
	TIM3_Configuration();//10Khz�ļ���Ƶ�ʣ�������5000Ϊ500ms  
	LCD_Fill(0x6666);		
	
	while(!Sensor_init());
	
	LCD_Fill(0xF800);
	delayms(100);
	num=2;
	               
	R_a=24;
	G_a=53;
	B_a=24;

	while(1)
	{
		if(num<=1)
		{
			Data_LCD_ColorChange();//���Ʋⶨ
		}
		if(num>1)
		{
			Data_LCD_ColorChange_Test();//����ͷɨ�����
			Uart_SendStr(table_cardMeasure,7);
			LCD_ShowNum(30,220,21-num,2);//
		}
		if(num==20)
		{
			num=0;
		}                                                                  
		num++;                                        
	}	
}



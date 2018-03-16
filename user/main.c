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
u16 color_save;//保存一个像素的值
u8 R,G,B;//颜色分量
u8 TableChangePoint_240[240];//跳变点240个
u8 Max_ChangePoint_240,Min_ChangePoint_240,Max_bChangePoint,Min_bChangePoint;//跳变点纵轴始、末坐标,跳变点横轴始、末坐标
u8 a_Continue,b_Continue;//记录纵、横轴突变点的连续性
u8 flag_aMax;//末值更新标志
u8 Max_aChangePoint_reset,Min_aChangePoint_reset;//修正后的上下限
u16 Length_card,Width_card;//车牌的长和宽
u8 Max_aChangePoint_reset_1,Min_aChangePoint_reset_1;//保存上次的数据
u8 flag_MaxMinCompare;//Max_aChangePoint_reset_1和Max_aChangePoint_reset的标志
u8 TableChangePoint_320[320];//纵向跳变点320个
float V,S,H;//定义HSV值
u16 Min_blue;
u16 Max_blue;//定义车牌蓝色区域的横向最大值和最小值
u16 k1,kk1,k2,kk2,k3,kk3,k4,kk4,k5,kk5,k6,kk6,k7,kk7,k8,kk8;//八个字符边界
extern u8 Table[6300];//所有字符集 （10+26）*150 = 5400 字节
extern u8 talble_0[150];//字符3,测试用
extern u8 table_yu[32];//渝字
extern u8 table_min[32];//闽字
extern u8 table_lu[32];//鲁字
extern u8 table_zhe[32];//浙字
extern u8 table_shan[32];//陕字
extern u8 table_cuan[32];//川字
u8 R_a,G_a,B_a;//阈值

u8 table_picture[150];//定义保存图片的数组
u8 table_char[36]={0,1,2,3,4,5,6,7,8,9,'A','B','C','D','E','F','G','H','I','J','K','L','M','N','O','P','Q','R','S','T','U','V','W','X','Y','Z',};
u8 table_card[5][8]={	//保存5个车牌的二维数组
{0,0,0,0,0,0,0,0},		//最后一位保存时间
{0,0,0,0,0,0,0,0},
{0,0,0,0,0,0,0,0},
{0,0,0,0,0,0,0,0},
{0,0,0,0,0,0,0,0},
};
u8 tim3_num;//TIM3分钟计时

u8 table_cardMeasure[7];//测量的车牌结果
void Show_Card(u8 i);//显示第几组车牌
void Show_Title();//显示标题

void MYRCC_DeInit(void)//复位并配置向量表
{										 
	NVIC_InitTypeDef NVIC_InitStructure;	
	RCC->APB1RSTR = 0x00000000;//复位结束			 
	RCC->APB2RSTR = 0x00000000; 
	  
	RCC->AHBENR = 0x00000014;  //睡眠模式闪存和SRAM时钟使能.其他关闭.	  
	RCC->APB2ENR = 0x00000000; //外设时钟关闭.			   
	RCC->APB1ENR = 0x00000000;   
	RCC->CR |= 0x00000001;     //使能内部高速时钟HSION	 															 
	RCC->CFGR &= 0xF8FF0000;   //复位SW[1:0],HPRE[3:0],PPRE1[2:0],PPRE2[2:0],ADCPRE[1:0],MCO[2:0]					 
	RCC->CR &= 0xFEF6FFFF;     //复位HSEON,CSSON,PLLON
	RCC->CR &= 0xFFFBFFFF;     //复位HSEBYP	   	  
	RCC->CFGR &= 0xFF80FFFF;   //复位PLLSRC, PLLXTPRE, PLLMUL[3:0] and USBPRE 
	RCC->CIR = 0x00000000;     //关闭所有中断
	/* Enable the TIM3 global Interrupt */
	NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQn;  //TIM3中断
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;  //先占优先级0级
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;  //从优先级3级
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; //IRQ通道被使能
	NVIC_Init(&NVIC_InitStructure);  //根据NVIC_InitStruct中指定的参数初始化外设NVIC寄存器
}
void Stm32_Clock_Init(u8 PLL)//系统时钟初始化函数  pll:选择的倍频数，从2开始，最大值为16	
{
	unsigned char temp=0;   
	MYRCC_DeInit();		  //复位并配置向量表
	RCC->CR|=0x00010000;  //外部高速时钟使能HSEON
	while(!(RCC->CR>>17));//等待外部时钟就绪
	RCC->CFGR=0X00000400; //APB1=DIV2;APB2=DIV1;AHB=DIV1;
	PLL-=2;//抵消2个单位
	RCC->CFGR|=PLL<<18;   //设置PLL值 2~16
	RCC->CFGR|=1<<16;	  //PLLSRC ON 
	FLASH->ACR|=0x32;	  //FLASH 2个延时周期

	RCC->CR|=0x01000000;  //PLLON
	while(!(RCC->CR>>25));//等待PLL锁定
	RCC->CFGR|=0x00000002;//PLL作为系统时钟	 
	while(temp!=0x02)     //等待PLL作为系统时钟设置成功
	{   
		temp=RCC->CFGR>>2;
		temp&=0x03;
	}    
}

void LED()//LED指示灯-提示
{
	GPIO_WriteBit(LED1_GPIO_PORT, LED1_GPIO_PIN,LED_flag>>7);
	LED_flag=~LED_flag;
}
void Data_LCD_Display()//常规显示
{
	LCD_SetWindows(0,0,320,240);//设置显示窗口
	GPIO_WriteBit(LCD_RS_PORT, LCD_RS_PIN,1);//标志：数据写入

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
void RGB_HSV(u16 num)//RGB565转HSV
{
	float max,min;
	u8 r,g,b;
	r=(num>>11)*255/31;g=((num>>5)&0x3f)*255/63;b=(num&0x1f)*255/31;
	
	max=r;min=r;
	if(g>=min)max=g;
	if(b>=max)max=b;
	if(g<=max)min=g;
	if(b<=min)min=b;
	
	V=100*max/255;//转换为百分比
	S=100*(max-min)/max;//扩大100倍显示
	if(max==r) H=(g-b)/(max-min)*60;
	if(max==g) H=120+(b-r)/(max-min)*60;
	if(max==b) H=240+(r-g)/(max-min)*60;
	if(H<0) H=H-360;
}
void Data_LCD_ColorDisplay()//单色像素显示、测量
{
	LCD_SetWindows(0,0,320,240);//设置显示窗口
	GPIO_WriteBit(LCD_RS_PORT, LCD_RS_PIN,1);//标志：数据写入

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
// //像素点测试，位置指示
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
//像素点测试，位置指示
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

	RGB_HSV(color_save);//RGB565转HSV
	LCD_ShowChar(30,200,'H',0);//X坐标，Y坐标，字符，填充模式
	LCD_ShowNum(30,220,H,3);//X坐标，Y坐标，数字，几位数

	LCD_ShowChar(60,200,'S',0);//X坐标，Y坐标，字符，填充模式
	LCD_ShowNum(60,220,S,3);//X坐标，Y坐标，数字，几位数

	LCD_ShowChar(90,200,'V',0);//X坐标，Y坐标，字符，填充模式
	LCD_ShowNum(90,220,V,3);//X坐标，Y坐标，数字，几位数	
	//显示测定的像素
	LCD_ShowChar(160,200,'R',0);//X坐标，Y坐标，字符，填充模式
	LCD_ShowNum(160,220,R,2);//X坐标，Y坐标，数字，几位数

	LCD_ShowChar(190,200,'G',0);//X坐标，Y坐标，字符，填充模式
	LCD_ShowNum(190,220,G,2);//X坐标，Y坐标，数字，几位数

	LCD_ShowChar(220,200,'B',0);//X坐标，Y坐标，字符，填充模式
	LCD_ShowNum(220,220,B,2);//X坐标，Y坐标，数字，几位数	
	for(a=0;a<30;a++)//划线
	{
		for(b=0;b<30;b++)//划线
		{
			LCD_DrawPoint(a+300,b+200,(R<<11)|(G<<5)|B);
		}
	}
}
void Picture_String()//图片->数组table_picture      有个BUG，先放着吧
{
	u16 a,b,num1;
	for(a=0;a<150;a++)//归零
	{
		table_picture[a]=0x00;	
	}	
	for(a=0;a<50;a++)//50排
	{
		for(b=0;b<24;b++)//24行
		{
			num1=LCD_ReadPoint(b+296,a+191);
			if(num1==0xffff)
			{
				table_picture[b/8+a*3]|=(1<<(7-b%8));
			}
		}				
	}
}
void String_Picture_All()//总数组->图片
{
	u16 a,b,e,i,num1;

	for(a=0;a<42;a++)//36
	{
		for(e=0;e<50;e++)//50排
		{
			for(i=0;i<24;i++)//24行
			{
				if(Table[150*a+i/8+e*3]&(1<<(7-i%8)))
				{
					num1=0xffff;
				}
				else
				{
					num1=0x0000;
				}
				LCD_DrawPoint(i+296,e+191,num1);//画点
			}				
		}
		delay_ms(500);
	}
}
u8 MoShiShiBie_All(u8 begin,u8 end)//字符匹配，模式识别,选择性匹配begin-end
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
		LCD_ShowNum(50,220,a,2);//显示匹配的字符是"a"			<调试用>
		LCD_ShowNum(70,220,num1,4);//显示匹配的正确像素数
		LCD_ShowNum(120,220,num_save,4);//匹配的最大值显示					
	}
	return a_save;
}
void String_Picture()//数组->图片
{
	u16 a,b,e,num1;
	
	for(a=0;a<150;a++)//150排
	{
		for(b=0;b<24;b++)//24行
		{
			if(talble_0[b/8+a*3]&(1<<(7-b%8)))
			{
				num1=0xffff;
			}
			else
			{
				num1=0x0000;
			}
			LCD_DrawPoint(b+296,a+191,num1);//画点
		}				
	}	
}
void WordShow(u8 num,u16 x,u16 y)//显示汉字16*16
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
			LCD_DrawPoint(b+x,a+y,num1);//画点
		}				
	}	
}
u8 ZhiFuFenGe()//字符分割,返回分割的字符个数，用于判断合法性
{
	u16 a,b;
	u8 i;//统计分割的字符个数，不为9说明分割有误
	i=0;//必须置0一下，不然会有错
	for(b=Max_blue;b>Min_blue;b--)
	{
		if(TableChangePoint_320[b]==0)//间隙分割
		{
			for(a=Min_ChangePoint_240;a<Max_ChangePoint_240;a++)//划线--调试用
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
	LCD_ShowNum(30,220,i,2);//显示分割的字符个数+1，8是正常值
	return i;
}
void GuiYi(u16 k,u16 kk)//归一化 25*50
{
	u16 a,b,e;
	u16 num;//保存读取像素
	u8 Mo,Yu;//取整和取模
	u8 num1,num2,num3;
	u8 Mo_1;//
	u8 Min_240,Max_240;//框紧字符后的上下限
	
	if((k-kk)<25)
	{
		//框紧字符
		Min_240=Min_ChangePoint_240+1;
		Max_240=Max_ChangePoint_240-1;
		while(Min_240++)//框紧后，得到: Min_240
		{
			for(b=kk-10;b<k;b++)//kk1→k1                                
			{
				num=LCD_ReadPoint(b,Min_240);
				if(num) break;
			}
			if(num) break;
		}
		while(Max_240--)//框紧后，得到: Max_240
		{
			for(b=kk+1;b<k;b++)//kk1→k1                                
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
		//显示复制的图片
		num3=0;
		for(a=Min_240+1;a<Max_240;a++)
		{
			num2=10;
			for(b=kk+1;b<k;b++)//kk1→k1                                   +1
			{
				num=LCD_ReadPoint(b,a);
				LCD_DrawPoint(271-(k-kk-1)+num2,191+num3,num);//复制像素值
				num2++;
			}
			num3++;
		}
		num3=0;
		Mo=(24-(k-kk-1))/(k-kk-1);//取模
		Yu=(24-(k-kk-1))%(k-kk-1);//取余
		if(Yu!=0) 
		{
			Mo_1=24/Yu;//平均Mo_1个像素，插有一个像素，机7+1
		}
// 		LCD_ShowNum(30,20,Mo,3);//显示模		<调试用>
// 		LCD_ShowNum(70,20,Yu,3);//显示余
// 		LCD_ShowNum(100,20,(k1-kk1),3);//显示差值

		for(a=Min_240+1;a<Max_240;a++)//宽放大为25像素  =??
		{//↓
			num2=10;
			Yu=(24-(k-kk-1))%(k-kk-1);//取余
			
			for(b=kk+1;b<k;b++)//kk1→k1                                   +1
			{
				num=LCD_ReadPoint(b,a);
				LCD_DrawPoint(271+num2,191+num3,num);
				num2++;
				Mo=(24-(k-kk-1))/(k-kk-1);//取模
				while(Mo)
				{
					LCD_DrawPoint(271+num2,191+num3,num);
					Mo--;
					num2++;
				}
				if(Yu!=0)//横轴拉长
				{	
					if(((num2+1)%Mo_1==0) && (num2!=1))//改插入的地方7+1
					{
						LCD_DrawPoint(271+num2,191+num3,num);
						Yu--;
						num2++;
					}
				}
			}
			num3--;
		}
		LCD_DrawPoint(271,191,0x07E0);//标记点，4个顶角
		LCD_DrawPoint(271,240,0x07E0);
		LCD_DrawPoint(295,191,0x07E0);
		LCD_DrawPoint(295,240,0x07E0);
//纵轴拉长
		if((Max_240-Min_240)<50)
		{
			Mo=(50-(Max_240-Min_240+1))/(Max_240-Min_240+1);//取模
			Yu=(50-(Max_240-Min_240+1))%(Max_240-Min_240+1);//取余
			Mo_1=50/Yu;
			
// 			LCD_ShowNum(30,170,Mo,3);//					<调试用>
// 			LCD_ShowNum(70,170,Yu,3);//
// 			LCD_ShowNum(100,170,Max_ChangePoint_240-Min_ChangePoint_240,3);//
			num2=0;
			for(a=0;a<(Max_240-Min_240);a++)//复制图像,考虑范围是否需要进行修正？
			{//↓
				for(b=271;b<=295;b++)//271开始复制，295才结束
				{
					num=LCD_ReadPoint(b,a+191);
					LCD_DrawPoint(b+25,191+num2,num);//复制像素值
				}
				num2++;
				while(Mo)
				{
					for(b=271;b<=295;b++)//271开始复制，295才结束
					{
						num=LCD_ReadPoint(b,a+191);
						LCD_DrawPoint(b+25,191+num2+a,num);//复制像素值
					}
					Mo--;
					num2++;
				}
				if(Yu!=0)
				{
					if((((num2+1) % Mo_1)==0)&& (num2!=1))
					{
						for(b=271;b<=295;b++)//271开始复制，295才结束
						{
							num=LCD_ReadPoint(b,a+191);
							LCD_DrawPoint(b+25,191+num2,num);//复制像素值
						}
						Yu--;
						num2++;
					}
				}					
			}
		}
		LCD_DrawPoint(320,191,0xf800);//标记点，1个顶角
	}
}
void ZhiFuShiBie()//字符识别
{
	u16 a,b,e;
	u16 i,u;
	u8 Result;//识别结果

	for(b=Max_blue-1;b>Min_blue;b--)//由右至左识别，获取各个字符的k,kK值
	{
		while(TableChangePoint_320[b]==0) b--;//取第1个字符
		k1=b+1;//+1
		while(TableChangePoint_320[b]>0) b--;
		kk1=b;
		if((k1-kk1)<4)//省略低于三个像素的位置
		{
			while(TableChangePoint_320[b]==0) b--;//
			k1=b+1;//+1
			while(TableChangePoint_320[b]>0) b--;
			kk1=b;
		}
		while(TableChangePoint_320[b]==0) b--;//取第2个字符
		k2=b+1;
		while(TableChangePoint_320[b]>0) b--;
		kk2=b;
		if((k2-kk2)<4)//省略低于3个像素的位置
		{
			while(TableChangePoint_320[b]==0) b--;//
			k2=b+1;//+1
			while(TableChangePoint_320[b]>0) b--;
			kk2=b;
		}
		while(TableChangePoint_320[b]==0) b--;//取第3个字符
		k3=b+1;//+1
		while(TableChangePoint_320[b]>0) b--;
		kk3=b;
		if((k3-kk3)<4)//省略低于3个像素的位置
		{
			while(TableChangePoint_320[b]==0) b--;//
			k3=b+1;//+1
			while(TableChangePoint_320[b]>0) b--;
			kk3=b;
		}
		while(TableChangePoint_320[b]==0) b--;//取第4个字符
		k4=b+1;
		while(TableChangePoint_320[b]>0) b--;
		kk4=b;
		if((k4-kk4)<4)//省略低于3个像素的位置
		{
			while(TableChangePoint_320[b]==0) b--;//
			k4=b+1;//+1
			while(TableChangePoint_320[b]>0) b--;
			kk4=b;
		}
		while(TableChangePoint_320[b]==0) b--;//取第5个字符
		k5=b+1;//+1
		while(TableChangePoint_320[b]>0) b--;
		kk5=b;
		if((k5-kk5)<4)//省略低于3个像素的位置
		{
			while(TableChangePoint_320[b]==0) b--;//
			k5=b+1;//+1
			while(TableChangePoint_320[b]>0) b--;
			kk5=b;
		}
		while(TableChangePoint_320[b]==0) b--;//取第6个字符
		k6=b+1;
		while(TableChangePoint_320[b]>0) b--;
		kk6=b;
		while(TableChangePoint_320[b]==0) b--;//取第7个字符
		k7=b+1;//+1
		while(TableChangePoint_320[b]>0) b--;
		kk7=b;
		if((k7-kk7)<4)//省略低于3个像素的位置
		{
			while(TableChangePoint_320[b]==0) b--;//
			k7=b+1;//+1
			while(TableChangePoint_320[b]>0) b--;
			kk7=b;
		}
		while(TableChangePoint_320[b]==0) b--;//取第8个字符
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
		b=Min_blue;//以防万一，还满足for循环条件
	}
	for(a=Min_ChangePoint_240;a<Max_ChangePoint_240;a++)//划线
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
//归一化处理：大小为25*50
	
//第1个字符：
	GuiYi(k1,kk1);//归一化 24*24
	Picture_String();//图片->数组
	Result=MoShiShiBie_All(0,36);//字符匹配，模式识别,返回a,0<= a <36
	if(Result<10)
	{
		LCD_ShowNum(240,220,table_char[Result],1);
	}
	else
	{
		LCD_ShowChar(240,220,table_char[Result],0);
	}
	table_cardMeasure[6]=Result;//保存识别的车牌结果
	
//第2个字符：
	GuiYi(k2,kk2);//归一化 25*50
	Picture_String();//图片->数组
	Result=MoShiShiBie_All(0,36);//字符匹配，模式识别
	if(Result<10)
	{
		LCD_ShowNum(230,220,table_char[Result],1);
	}
	else
	{
		LCD_ShowChar(230,220,table_char[Result],0);
	}
	table_cardMeasure[5]=Result;//保存识别的车牌结果
	
	GuiYi(k3,kk3);//归一化 25*50
	Picture_String();//图片->数组
	Result=MoShiShiBie_All(0,36);//字符匹配，模式识别
	if(Result<10)
	{
		LCD_ShowNum(220,220,table_char[Result],1);
	}
	else
	{
		LCD_ShowChar(220,220,table_char[Result],0);
	}
	table_cardMeasure[4]=Result;//保存识别的车牌结果
	
	GuiYi(k4,kk4);//归一化 25*50
	Picture_String();//图片->数组
	Result=MoShiShiBie_All(0,36);//字符匹配，模式识别
	if(Result<10)
	{
		LCD_ShowNum(210,220,table_char[Result],1);
	}
	else
	{
		LCD_ShowChar(210,220,table_char[Result],0);
	}
	table_cardMeasure[3]=Result;//保存识别的车牌结果
	
	GuiYi(k5,kk5);//归一化 25*50
	Picture_String();//图片->数组
	Result=MoShiShiBie_All(0,36);//字符匹配，模式识别
	if(Result<10)
	{
		LCD_ShowNum(200,220,table_char[Result],1);
	}
	else
	{
		LCD_ShowChar(200,220,table_char[Result],0);
	}
	table_cardMeasure[2]=Result;//保存识别的车牌结果
	LCD_ShowChar(190,220,'.',0);

	GuiYi(k7,kk7);//归一化 25*50
	Picture_String();//图片->数组
	Result=MoShiShiBie_All(10,36);//字符匹配，模式识别，只匹配字母
	if(Result<10)
	{
		LCD_ShowNum(180,220,table_char[Result],1);
	}
	else
	{
		LCD_ShowChar(180,220,table_char[Result],0);
	}
	table_cardMeasure[1]=Result;//保存识别的车牌结果
	
	GuiYi(k8,kk8);//归一化 25*50					最后一个汉字，不做识别
	Picture_String();//图片->数组
	Result=MoShiShiBie_All(36,42);//字符匹配，匹配汉字
	WordShow(Result-35,160,220);//显示汉字
	table_cardMeasure[0]=Result-35;//保存识别的车牌结果
	//识别结束
	//	while(1);
	//先匹配已保存的车牌号
//	for(u=0;u<5;u++)
//	{
//		for(i=0;i<7;i++)
//		{
//			if(table_card[u][i]!=table_cardMeasure[i]) i=8;//退出for循环
//			
//		}	
//		if(i==7)//匹配成功
//		{
//			LCD_Fill(0x00);//黑屏
//			Show_Title();//显示标题
//			Show_Card(u);//显示第几组车牌			
//			delay_ms(15000);
//			u=5;
//		}
//	}
//	if(i==9)//无匹配车牌，则保存车牌
//	{
//		i=0;
//		while(1)
//		{
//			if(GPIO_ReadInputDataBit(KEY1_PORT,KEY1_PIN)==0) break;
//			LCD_ShowNum(30,220,i/100,2);
//			if(i==300)														//保存数据
//			{
//				for(u=0;u<5;u++)
//				{
//					if(table_card[u][0]==0)
//					{
//						for(i=0;i<7;i++)
//						{
//							table_card[u][i]=table_cardMeasure[i];
//						}					
//						u=5;//退出循环
//					}
//				}
//				LCD_Fill(0x00);//黑屏
//				Show_Title();//显示标题
//				Show_Card(0);//显示第几组车牌
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
void Copy_Card()//复制车牌Max_aChangePoint_reset_1,Min_blue
{
	u16 a,b,num;
	for(a=Min_aChangePoint_reset;a<=Max_aChangePoint_reset;a++)
	{	
		for(b=Min_blue;b<Max_blue;b++)
		{
			num=LCD_ReadPoint(b,a);//读取像素值
			LCD_DrawPoint(b,50+a,num);
		}
		
	}
}
void ChangePoint_Show_240()//240方向跳变点显示
{
	for(a=0;a<240;a++)//建立参考线10、20、30
	{
		LCD_DrawPoint(10,a,0x63<<5);//10
		LCD_DrawPoint(20,a,0x63<<5);//20
		LCD_DrawPoint(30,a,0x63<<5);//30
	}
	
	for(a=0;a<240;a++)//显示对应的横向跳变点								
	{
		LCD_DrawPoint(TableChangePoint_240[a],a,0xf800);//跳变点显示，红色标记
		if(TableChangePoint_240[a]>=15)					//跳变点个数（阈值）设定       阈值调节3-（1）
		{
			for(b=35;b<40;b++)						//显示达到阈值标准的点
			{
				LCD_DrawPoint(b,a,0x63<<5);//Green			
			}
		}
	}
}
void ChangePoint_Analysis_240()//240跳变点分析，即纵向分析出车牌的上下边界和高度值
{
	Min_ChangePoint_240=240;Max_ChangePoint_240=0;
	
	for(a=0;a<240;a++)//240扫描	，获取上下限值	：Min_ChangePoint_240，Max_ChangePoint_240				
	{
		while(TableChangePoint_240[a]<=15)									//阈值调节3-（2）
		{
			a++;
		}
		Min_ChangePoint_240=a;
		while(TableChangePoint_240[a]>15)									//阈值调节3-（3）
		{
			a++;
		}
		Max_ChangePoint_240=a;
		if(Max_ChangePoint_240-Min_ChangePoint_240>=15) a=240;//连续性阈值   	//阈值调节2-（1）
	}
	Min_ChangePoint_240=Min_ChangePoint_240-3;//向上微调3像素
	Max_ChangePoint_240=Max_ChangePoint_240+2;//向下微调2像素
	for(a=30;a<280;a++)//显示上界限				
	{
		LCD_DrawPoint(a,Max_ChangePoint_240,0x001f);
	}
	for(a=30;a<280;a++)//显示下界限						
	{
		LCD_DrawPoint(a,Min_ChangePoint_240,0x001f);
	}
	for(a=30;a<280;a++)//显示50,参考50像素位置处，车牌位置不要超过这根线，免得不能字符的归一化处理						
	{
		LCD_DrawPoint(a,Min_ChangePoint_240+50,0xf800);
	}
	flag_MaxMinCompare=1;
	if(Min_ChangePoint_240>Max_ChangePoint_240)//判断合法性1：最小值>最大值
	{
		flag_MaxMinCompare=0;
	}
	if(Min_ChangePoint_240==240||Max_ChangePoint_240==0)//判断合法性2：值没有重新赋值
	{
		flag_MaxMinCompare=0;
	}
	if(Max_ChangePoint_240-Min_ChangePoint_240<15)		//判断合法性3：			//阈值调节2-（2）
	{
		flag_MaxMinCompare=0;
	}
}
void ChangePoint_Analysis_Blue()//320蓝色区域分析,采用读取像素，得结果Min_blue,Max_blue
{																//分析出车牌的左右边界，即车牌的横向宽度
	u16 a,b,num_color;
	u16 min_320,max_320;//各行的最小、最大值
	
	Min_blue=0;Max_blue=320;
	min_320=320;max_320=0;
	
	for(a=Min_ChangePoint_240;a<Max_ChangePoint_240;a++)								
	{
		for(b=30;b<290;b++)//不用到320    for(b=30;b<320;b++)
		{
			num_color=LCD_ReadPoint(b,a);//读取像素，代码优化速度有待提升 ？扫描方法也可优化，以提升速度
			RGB_HSV(num_color);//RGB565转HSV
			if( 250>H && H>190 && 60>S && S>15 && 100>V && V>45)//HSV 阈值
			{
				if(b<min_320)//获得横轴的Min和Max值，即蓝色车牌的左右边界
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
	Min_blue=min_320;//获取各行的最大值//修正一点
	Max_blue=max_320-50;//获取各行的最小值//修正一点
	
	for(a=Min_ChangePoint_240;a<Max_ChangePoint_240;a++)//显示左界限				
	{
		LCD_DrawPoint(Min_blue,a,0xf8);//LCD_DrawPoint(Min_blue,a,0xf800);
	}
	for(a=Min_ChangePoint_240;a<Max_ChangePoint_240;a++)//显示右界限					
	{
		LCD_DrawPoint(Max_blue,a,0xf800);
	}
// 	delay_ms(6000);
}
void Card_101Show()//车牌区域二值化显示
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
		num_color=LCD_ReadPoint(b,Mid_ChangePoint_240);//读取像素，代码优化速度有待提升 ？扫描方法也可优化，以提升速度
		R1=num_color>>11;
		G1=(num_color>>5)&0x3F;
		B1=num_color&0x1F;
		if( (R1>10) && (G1>25) && (B1>15) && (R1<=30) && (G1<=60) && (B1<=30) )//二值化,高阈值：25.55.25，较合适阈值（21,47,21）
		{
			if(max_R<R1) max_R=R1;//获得最大值和最小值
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
// 	LCD_ShowNum(30,200,max_R,2);//X坐标，Y坐标，数字，几位数             <调试用>
// 	LCD_ShowNum(70,200,max_G,2);//X坐标，Y坐标，数字，几位数
// 	LCD_ShowNum(100,200,max_B,2);//X坐标，Y坐标，数字，几位数
// 	
// 	LCD_ShowNum(30,220,min_R,2);//X坐标，Y坐标，数字，几位数
// 	LCD_ShowNum(70,220,min_G,2);//X坐标，Y坐标，数字，几位数
// 	LCD_ShowNum(100,220,min_B,2);//X坐标，Y坐标，数字，几位数
	
	for(a=Min_ChangePoint_240;a<Max_ChangePoint_240;a++)								
	{
		for(b=Min_blue;b<Max_blue;b++)
		{
			num_color=LCD_ReadPoint(b,a);//读取像素，代码优化速度有待提升 ？扫描方法也可优化，以提升速度
			R1=num_color>>11;
			G1=(num_color>>5)&0x3F;
			B1=num_color&0x1F;
			if((R1>=mid_R) && (G1>=mid_G) && (B1>=mid_B))//二值化,高阈值：25.55.25，较合适阈值（21,47,21）
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
void ChangePoint_Show_320()//320方向跳变点显示
{
	for(a=0;a<320;a++)//显示对应的横向跳变点								
	{
		if(TableChangePoint_320[a]==0)
		{
			LCD_DrawPoint(a,0,0x001F);//跳变点显示，红色标记
		}
		else
		{
			LCD_DrawPoint(a,TableChangePoint_320[a],0xf800);//跳变点显示，红色标记
		}
		

	}
}
void ChangePoint_Analysis_320()//蓝色区域中，320跳变点分析,获得TableChangePoint_320[b]结果
{								//(先二值化，再判断白点个数，=0则是分割线）
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
		num_color=LCD_ReadPoint(b,Mid_ChangePoint_240);//读取像素，代码优化速度有待提升 ？扫描方法也可优化，以提升速度
		R1=num_color>>11;
		G1=(num_color>>5)&0x3F;
		B1=num_color&0x1F;
		if( (R1>10) && (G1>25) && (B1>15) && (R1<=30) && (G1<=60) && (B1<=30) )//二值化,高阈值：25.55.25，较合适阈值（21,47,21）
		{
			if(max_R<R1) max_R=R1;//获得最大值和最小值
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
// 	LCD_ShowNum(30,200,max_R,2);//X坐标，Y坐标，数字，几位数		<调试用>
// 	LCD_ShowNum(70,200,max_G,2);//X坐标，Y坐标，数字，几位数
// 	LCD_ShowNum(100,200,max_B,2);//X坐标，Y坐标，数字，几位数
// 	
// 	LCD_ShowNum(30,220,min_R,2);//X坐标，Y坐标，数字，几位数
// 	LCD_ShowNum(70,220,min_G,2);//X坐标，Y坐标，数字，几位数
// 	LCD_ShowNum(100,220,min_B,2);//X坐标，Y坐标，数字，几位数
	for(b=0;b<320;b++)//各行跳变点计数，数组清零
	{
		TableChangePoint_320[b]=0;
	}
	for(a=Min_ChangePoint_240;a<Max_ChangePoint_240;a++)								
	{
		for(b=Min_blue+1;b<Max_blue;b++)
		{
			num_color=LCD_ReadPoint(b,a);//读取像素，代码优化速度有待提升 ？扫描方法也可优化，以提升速度
			R1=num_color>>11;
			G1=(num_color>>5)&0x3F;
			B1=num_color&0x1F;
			if((R1>=mid_R) && (G1>=mid_G) && (B1>=mid_B))//二值化,高阈值：25.55.25，较合适阈值（21,47,21）
			{
				LCD_DrawPoint(b,a,0xffff);
				TableChangePoint_320[b]++;//白色，跳变点+1
			}
			else
			{
				LCD_DrawPoint(b,a,0x0000);
			}
		}
	}
}

void Data_LCD_ColorChange()//摄像头扫描
{
	u8 i;
	for(a=0;a<240;a++)//各行跳变点计数，数组清零
	{
		TableChangePoint_240[a]=0;
	}
	Min_blue=320;//初始化记录蓝色车牌区域的值
	Max_blue=0;
	
	LCD_SetWindows(0,0,320,240);//设置显示窗口
	GPIO_WriteBit(LCD_RS_PORT, LCD_RS_PIN,1);//标志：数据写入

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
			
			if((R>R_a) && (G>=G_a) && (B>=B_a))//二值化,高阈值：25.55.25，较合适阈值（21,47,21）
			{
				color=0xffff;
			}
			else
			{
				color=0x0000;
			}
			
			if(color!=color_save)//跳变点
			{
				TableChangePoint_240[a]++;		//该行跳变点计数+1
			}
			color_save=color;//保存像素值，供下一次判断和比较
			
			color=(AA<<8)|BB;//还原色彩
					
			LCD_DATA_PORT->ODR = color;
			GPIOC->BRR =1<<11;
			GPIOC->BSRR =1<<11;
		}
	}
	
	ChangePoint_Show_240();//240方向跳变点显示
	ChangePoint_Analysis_240();	//跳变点分析
							//返回flag_MaxMinCompare，Min_ChangePoint_240，Max_ChangePoint_240
	if(flag_MaxMinCompare==1)//跳变点筛选成功
	{
		ChangePoint_Analysis_Blue();//320蓝色区域分析,采用读取像素，得结果Min_blue,Max_blue
		if(Min_blue>Max_blue) flag_MaxMinCompare=0;//进行合理性判断1
		if((Min_blue>290)||(Max_blue>290)) flag_MaxMinCompare=0;//进行合理性判断2
	}
	if(flag_MaxMinCompare==1)//跳变点筛选成功
	{
//		ChangePoint_Analysis_Blue();//320蓝色区域分析,采用读取像素，得结果Min_blue,Max_blue
// 		Card_101Show();//车牌区域二值化显示 
		ChangePoint_Analysis_320();//蓝色区域中，320跳变点分析,获得：TableChangePoint_320[b]结果
		ChangePoint_Show_320();//320方向跳变点显示
		i=ZhiFuFenGe();
		
		if(i==8)//字符分割,返回分割的字符个数，用于判断合法性
		{
			ZhiFuShiBie();//字符识别	
		}
		else
		{
			LCD_Fill(0x6666);//黑屏，显示Measure Faill
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
void Data_LCD_ColorChange_Test()//摄像头扫描测试
{
	for(a=0;a<240;a++)//各行跳变点计数，数组清零
	{
		TableChangePoint_240[a]=0;
	}
	Min_blue=320;//初始化记录蓝色车牌区域的值
	Max_blue=0;
	
	LCD_SetWindows(0,0,320,240);//设置显示窗口
	GPIO_WriteBit(LCD_RS_PORT, LCD_RS_PIN,1);//标志：数据写入

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
			
			if((R>R_a) && (G>=G_a) && (B>=B_a))//二值化,高阈值：25.55.25，较合适阈值（21,47,21）
			{
				color=0xffff;
			}
			else
			{
				color=0x0000;
			}
			
			if(color!=color_save)//跳变点
			{
				TableChangePoint_240[a]++;		//该行跳变点计数+1
			}
			color_save=color;//保存像素值，供下一次判断和比较
			
			color=(AA<<8)|BB;//还原色彩
					
			LCD_DATA_PORT->ODR = color;
			GPIOC->BRR =1<<11;
			GPIOC->BSRR =1<<11;
		}
	}
	ChangePoint_Show_240();//240方向跳变点显示
	ChangePoint_Analysis_240();	//跳变点分析
}
void Data_LCD_ColorPut()//颜色处理
{
	LCD_SetWindows(0,0,320,240);//设置显示窗口
	GPIO_WriteBit(LCD_RS_PORT, LCD_RS_PIN,1);//标志：数据写入

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
void PC0_in()//PC0引脚为输入
{
	GPIO_InitTypeDef gpio_init_struct;//结构体
	gpio_init_struct.GPIO_Mode = GPIO_Mode_IPU;//输入上拉
	gpio_init_struct.GPIO_Speed = GPIO_Speed_50MHz;
	gpio_init_struct.GPIO_Pin = GPIO_Pin_0;
	GPIO_Init(GPIOC, &gpio_init_struct);
}
void Show_Card(u8 i)//显示第几组车牌
{
	u16 t0;
//显示汉字
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
	LCD_ShowChar(33,i*16+50,'.',0);														//点
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
	LCD_ShowNum(100,i*16+50,t0,6);//显示时间

	if(t0<60)
	{
		LCD_ShowNumPoint(168,i*16+50,t0*8);
	}
	else
	{
		LCD_ShowNumPoint(168,i*16+50,t0/60*500+t0%60*8);
	}
	
}
//定时器3中断服务程序	 
void TIM3_IRQHandler(void)
{ 		    		  			    
	if(TIM3->SR&0X0001)//溢出中断
	{
		LED();
		if(tim3_num==60)
		{
			if(table_card[0][0]!=0)//第1组计时
			{
				table_card[0][7]++;
			}		
			if(table_card[1][0]!=0)//第2组计时
			{
				table_card[1][7]++;
			}
			if(table_card[2][0]!=0)//第3组计时
			{
				table_card[2][7]++;
			}	
			if(table_card[3][0]!=0)//第4组计时
			{
				table_card[3][7]++;
			}	
			if(table_card[4][0]!=0)//第5组计时
			{
				table_card[4][7]++;
			}
			tim3_num=0;
		}		
	}			
	tim3_num++;
	TIM3->SR&=~(1<<0);//清除中断标志位 	    
}
void TIM3_Configuration(void)
	{
	/* TIM3 clock enable */
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
	/* ---------------------------------------------------------------
	TIM3CLK 即PCLK1=36MHz
	TIM3CLK = 36 MHz, Prescaler = 7200, TIM3 counter clock = 5K,即改变一次为5K,周期就为10K
	--------------------------------------------------------------- */
	/* Time base configuration */
	TIM_TimeBaseStructure.TIM_Period = 10000; //设置在下一个更新事件装入活动的自动重装载寄存器周期的值	 计数到10000为1000ms
	TIM_TimeBaseStructure.TIM_Prescaler =(12800-1); //设置用来作为TIMx时钟频率除数的预分频值  10Khz的计数频率  
	TIM_TimeBaseStructure.TIM_ClockDivision = 0; //设置时钟分割:TDTS = Tck_tim
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  //TIM向上计数模式
	TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure); //根据TIM_TimeBaseInitStruct中指定的参数初始化TIMx的时间基数单位
	
	/* Enables the Update event for TIM3 */
	//TIM_UpdateDisableConfig(TIM3,ENABLE); 	//使能 TIM3 更新事件 
	
	/* TIM IT enable */
	TIM_ITConfig(  //使能或者失能指定的TIM中断
		TIM3, //TIM2
		TIM_IT_Update  |  //TIM 中断源
		TIM_IT_Trigger,   //TIM 触发中断源 
		ENABLE  //使能
		);
	
	/* TIM3 enable counter */
	TIM_Cmd(TIM3, ENABLE);  //使能TIMx外设
}
void Show_Title()//显示标题
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
	Stm32_Clock_Init(16);//初始化时钟

	Led_init();			//初始化 LED	
	Lcd_Gpio_Init();
	LCD_Init();	
	
 	Key_init();	//初始化 KEY1
	PC0_in();//PC0引脚为输入
	
	OV7670_Gpio_Init();//OV7670引脚初始化，放在串口初始化前面
	GPIO_WriteBit(FIFO_OE_PORT, FIFO_OE_PIN, 0);
	USART1_init();//初始化串口	
// 	PC0_in();//PC0引脚为输入
	TIM3_Configuration();//10Khz的计数频率，计数到5000为500ms  
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
			Data_LCD_ColorChange();//车牌测定
		}
		if(num>1)
		{
			Data_LCD_ColorChange_Test();//摄像头扫描测试
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



#include "stm32f10x.h"
#include "board.h"
#include "ov7670.h"

#define OV7670_REG_NUM  184

#define DELAYTIME 2//9��Ƶʱ��ʱ��1����16��Ƶʱ��ʱ��2��

void OV7670_Gpio_Init()
{
    GPIO_InitTypeDef gpio_init_struct;//�ṹ��
		
	RCC_APB2PeriphClockCmd(FIFO_WR_RCC, ENABLE);//��ʼ��ʱ��
	RCC_APB2PeriphClockCmd(FIFO_RRST_RCC, ENABLE);//��ʼ��ʱ��
	RCC_APB2PeriphClockCmd(FIFO_OE_RCC, ENABLE);//��ʼ��ʱ��
	RCC_APB2PeriphClockCmd(FIFO_RCLK_RCC, ENABLE);//��ʼ��ʱ��
	RCC_APB2PeriphClockCmd(FIFO_WRST_RCC, ENABLE);//��ʼ��ʱ��
	RCC_APB2PeriphClockCmd(SCCB_SIC_RCC, ENABLE);//��ʼ��ʱ��
	RCC_APB2PeriphClockCmd(SCCB_SID_RCC, ENABLE);//��ʼ��ʱ��
	RCC_APB2PeriphClockCmd(OV7670_RRST_RCC, ENABLE);//��ʼ��ʱ��
	
	gpio_init_struct.GPIO_Mode = GPIO_Mode_Out_PP;
	gpio_init_struct.GPIO_Speed = GPIO_Speed_50MHz;

	gpio_init_struct.GPIO_Pin = FIFO_WR_PIN;
	GPIO_Init(FIFO_WR_PORT, &gpio_init_struct);

	gpio_init_struct.GPIO_Pin = FIFO_RRST_PIN;
	GPIO_Init(FIFO_RRST_PORT, &gpio_init_struct);

	gpio_init_struct.GPIO_Pin = FIFO_OE_PIN;
	GPIO_Init(FIFO_OE_PORT, &gpio_init_struct);

	gpio_init_struct.GPIO_Pin = FIFO_RCLK_PIN;
	GPIO_Init(FIFO_RCLK_PORT, &gpio_init_struct);

	gpio_init_struct.GPIO_Pin = FIFO_WRST_PIN;
	GPIO_Init(FIFO_WRST_PORT, &gpio_init_struct);

	gpio_init_struct.GPIO_Pin = SCCB_SIC_PIN;
	GPIO_Init(SCCB_SIC_PORT, &gpio_init_struct);

	gpio_init_struct.GPIO_Pin = SCCB_SID_PIN;
	GPIO_Init(SCCB_SID_PORT, &gpio_init_struct);

	gpio_init_struct.GPIO_Pin = OV7670_RRST_PIN;
	GPIO_Init(OV7670_RRST_PORT, &gpio_init_struct);
	
	RCC_APB2PeriphClockCmd(OV7670_DATA_RCC, ENABLE);//��ʼ��ʱ��
	
	gpio_init_struct.GPIO_Mode = GPIO_Mode_IPU;//��������
	
	gpio_init_struct.GPIO_Pin = OV7670_DATA_PIN;//OV7670���ݿ����ų�ʼ��
	GPIO_Init(OV7670_DATA_PORT, &gpio_init_struct);
}
void SCCB_SID_change_in()//�����л�Ϊ����
{
    GPIO_InitTypeDef gpio_init_struct;//�ṹ��

    gpio_init_struct.GPIO_Mode = GPIO_Mode_IPU;//��������
    gpio_init_struct.GPIO_Speed = GPIO_Speed_50MHz;
	
	gpio_init_struct.GPIO_Pin = SCCB_SID_PIN;
    GPIO_Init(SCCB_SID_PORT, &gpio_init_struct);
}
void SCCB_SID_change_out()//�����л�Ϊ���
{
    GPIO_InitTypeDef gpio_init_struct;//�ṹ��
		
    gpio_init_struct.GPIO_Mode = GPIO_Mode_Out_PP;//�������
    gpio_init_struct.GPIO_Speed = GPIO_Speed_50MHz;
	gpio_init_struct.GPIO_Pin = SCCB_SID_PIN;
    GPIO_Init(SCCB_SID_PORT, &gpio_init_struct);
}
void FIFO_Reset_Read_Addr(void)//FIFO �����ݸ�λ��ͨ��ֱ�Ӳ����Ĵ���������ٶ�
{				
	GPIOC->BRR =1<<2;	//	RRST=0
	GPIOC->BRR =1<<4;	//	RCLK= 0
	GPIOC->BSRR =1<<4;	//  RCLK=1
	GPIOC->BRR =1<<4;	//	RCLK=0
	GPIOC->BSRR =1<<2;	//  RRST=1
	GPIOC->BSRR =1<<4;	//	RCLK=1
}
void startSCCB()//SCCB���ã�����I2C
{
	GPIO_WriteBit(SCCB_SID_PORT, SCCB_SID_PIN, 1);
	delay(DELAYTIME);
	GPIO_WriteBit(SCCB_SIC_PORT, SCCB_SIC_PIN, 1);
	delay(DELAYTIME);
	GPIO_WriteBit(SCCB_SID_PORT, SCCB_SID_PIN, 0);
	delay(DELAYTIME);
	GPIO_WriteBit(SCCB_SIC_PORT, SCCB_SIC_PIN, 0);
	delay(DELAYTIME);
}

void stopSCCB()//stop����,SCCB��ֹͣ�ź�
{
	GPIO_WriteBit(SCCB_SID_PORT, SCCB_SID_PIN, 0);
	delay(DELAYTIME);
	GPIO_WriteBit(SCCB_SIC_PORT, SCCB_SIC_PIN, 1);
	delay(DELAYTIME);
	GPIO_WriteBit(SCCB_SID_PORT, SCCB_SID_PIN, 1);
	delay(DELAYTIME);  
}
void noAck(void)//noAck,����������ȡ�е����һ����������
{
	GPIO_WriteBit(SCCB_SID_PORT, SCCB_SID_PIN, 1);
	delay(DELAYTIME);
	GPIO_WriteBit(SCCB_SIC_PORT, SCCB_SIC_PIN, 1);
	delay(DELAYTIME);
	GPIO_WriteBit(SCCB_SIC_PORT, SCCB_SIC_PIN, 0);
	delay(DELAYTIME);
	GPIO_WriteBit(SCCB_SID_PORT, SCCB_SID_PIN, 0);
	delay(DELAYTIME);
}
u8 getAck() 
{
	u8 Error;

	SCCB_SID_change_in();																			
	GPIO_WriteBit(SCCB_SID_PORT, SCCB_SID_PIN, 1);
	delay(DELAYTIME);

	GPIO_WriteBit(SCCB_SIC_PORT, SCCB_SIC_PIN, 1);
	delay(DELAYTIME);

	Error= GPIO_ReadInputDataBit(SCCB_SID_PORT, SCCB_SID_PIN);
	delay(DELAYTIME);

	GPIO_WriteBit(SCCB_SIC_PORT, SCCB_SIC_PIN, 0);
	delay(DELAYTIME);

	SCCB_SID_change_out();//���
	GPIO_WriteBit(SCCB_SID_PORT, SCCB_SID_PIN, 0);

	return !Error;
}

u8 SCCBwriteByte(u8 dat)//д��һ���ֽڵ����ݵ�SCCB
{
	u8 i;
	for(i=0;i<8;i++)
	{
		GPIO_WriteBit(SCCB_SID_PORT, SCCB_SID_PIN, (((dat<<i)&0x80))>>7);
		delay(DELAYTIME);

		GPIO_WriteBit(SCCB_SIC_PORT, SCCB_SIC_PIN, 1);
		delay(DELAYTIME);

		GPIO_WriteBit(SCCB_SIC_PORT, SCCB_SIC_PIN, 0);	 
		delay(DELAYTIME);
	}
	GPIO_WriteBit(SCCB_SID_PORT, SCCB_SID_PIN, 0);

	return getAck();
}
u8 SCCBreadByte(void)//һ���ֽ����ݶ�ȡ���ҷ���
{
	u8 i,rbyte=0;

	SCCB_SID_change_in();
	for(i=0;i<8;i++)
	{
		GPIO_WriteBit(SCCB_SIC_PORT, SCCB_SIC_PIN, 1);
		delay(DELAYTIME);

		if(GPIO_ReadInputDataBit(SCCB_SID_PORT, SCCB_SID_PIN)) rbyte|=(0x80>>i);
		delay(DELAYTIME);

		GPIO_WriteBit(SCCB_SIC_PORT, SCCB_SIC_PIN, 0);	 
		delay(DELAYTIME);
	} 

	SCCB_SID_change_out();//���
	GPIO_WriteBit(SCCB_SID_PORT, SCCB_SID_PIN, 0);
	return rbyte;
}

uc8 OV7670_reg[OV7670_REG_NUM][2]=//OV7670 �������Ĵ�������ʼ���������
{	 
// 		/*����ΪOV7670 QVGA RGB565����  */
// 		{0x3a, 0x04},//dummy
// 		{0x40, 0x10},//565
// 		{0x12, 0x14},//@QVGA=0x14;VGA=0x04?
// 		{0x32, 0x80},//HREF control	bit[2:0] HREF start 3 LSB	 bit[5:3] HSTOP HREF end 3LSB
// 		{0x17, 0x16},//              HSTART start high 8-bit MSB
// 		{0x18, 0x04},//5	 HSTOP end high 8-bit
// 		{0x19, 0x02},
// 		{0x1a, 0x7b},//0x7a,
// 		{0x03, 0x06},//0x0a,
// 		{0x0c, 0x0c},
// 		{0x15, 0x02},
// 		{0x3e, 0x00},//10
// 		{0x70, 0x00},
// 		{0x71, 0x01},
// 		{0x72, 0x11},
// 		{0x73, 0x09},//
// 		{0xa2, 0x02},//15
// 		{0x11, 0x00}, //fenpin--03����ͨ��--00(�Ͽ�)--80�����ˣ�---��Ƶ(me)
// 		{0x7a, 0x20},
// 		{0x7b, 0x1c},
// 		{0x7c, 0x28},
// 		{0x7d, 0x3c},//20
// 		{0x7e, 0x55},
// 		{0x7f, 0x68},
// 		{0x80, 0x76},
// 		{0x81, 0x80},
// 		{0x82, 0x88},
// 		{0x83, 0x8f},
// 		{0x84, 0x96},
// 		{0x85, 0xa3},
// 		{0x86, 0xaf},
// 		{0x87, 0xc4},//30
// 		{0x88, 0xd7},
// 		{0x89, 0xe8},
// 		{0x13, 0xe0},
// 		{0x00, 0x00},//AGC
// 		{0x10, 0x00},
// 		{0x0d, 0x00},
// 		{0x14, 0x20},//0x38, limit the max gain
// 		{0xa5, 0x05},
// 		{0xab, 0x07},
// 		{0x24, 0x75},//40
// 		{0x25, 0x63},
// 		{0x26, 0xA5},
// 		{0x9f, 0x78},
// 		{0xa0, 0x68},
// 		{0xa1, 0x03},//0x0b,
// 		{0xa6, 0xdf},//0xd8,
// 		{0xa7, 0xdf},//0xd8,
// 		{0xa8, 0xf0},
// 		{0xa9, 0x90},
// 		{0xaa, 0x94},//50
// 		{0x13, 0xe5},
// 		{0x0e, 0x61},
// 		{0x0f, 0x4b},
// 		{0x16, 0x02},
// 		{0x1e, 0x37},//0x07,
// 		{0x21, 0x02},
// 		{0x22, 0x91},
// 		{0x29, 0x07},
// 		{0x33, 0x0b},
// 		{0x35, 0x0b},//60
// 		{0x37, 0x1d},
// 		{0x38, 0x71},
// 		{0x39, 0x2a},
// 		{0x3c, 0x78},
// 		{0x4d, 0x40},
// 		{0x4e, 0x20},
// 		{0x69, 0x5d},
// 		{0x6b, 0x40},//PLL
// 		{0x74, 0x19},
// 		{0x8d, 0x4f},
// 		{0x8e, 0x00},//70
// 		{0x8f, 0x00},
// 		{0x90, 0x00},
// 		{0x91, 0x00},
// 		{0x92, 0x00},//0x19,//0x66
// 		{0x96, 0x00},
// 		{0x9a, 0x80},
// 		{0xb0, 0x84},
// 		{0xb1, 0x0c},
// 		{0xb2, 0x0e},
// 		{0xb3, 0x82},//80
// 		{0xb8, 0x0a},
// 		{0x43, 0x14},
// 		{0x44, 0xf0},
// 		{0x45, 0x34},
// 		{0x46, 0x58},
// 		{0x47, 0x28},
// 		{0x48, 0x3a},
// 		{0x59, 0x88},
// 		{0x5a, 0x88},
// 		{0x5b, 0x44},//90
// 		{0x5c, 0x67},
// 		{0x5d, 0x49},
// 		{0x5e, 0x0e},
// 		{0x64, 0x04},
// 		{0x65, 0x20},
// 		{0x66, 0x05},
// 		{0x94, 0x04},
// 		{0x95, 0x08},
// 		{0x6c, 0x0a},
// 		{0x6d, 0x55},
// 		{0x4f, 0x80},
// 		{0x50, 0x80},
// 		{0x51, 0x00},
// 		{0x52, 0x22},
// 		{0x53, 0x5e},
// 		{0x54, 0x80},
// 		//{0x54, 0x40},//110
// 		{0x6e, 0x11},//100
// 		//{0x13, 0xe7},																	//�Լ��ӵ�--me//AWB��AGC��AGC Enable and ...
// 		{0x6f, 0x9f},//0x9e for advance AWB												9f
// 		{0x55, 0x00},//����																0x00
// 		{0x56, 0x50},//�Աȶ�													��ʼֵ��0x20
// 		{0x57, 0x40},//0x40,  change according to Jim's request	      
{0x3a, 0x04},//dummy
{0x40, 0x10},//565   
{0x12, 0x14},//QVGA,RGB

{0x32, 0x80},//HREF control	bit[2:0] HREF start 3 LSB	bit[5:3] HSTOP HREF end 3LSB
{0x17, 0x16},//HSTART start high 8-bit MSB         
{0x18, 0x04},//5 HSTOP end high 8-bit
{0x19, 0x02},
{0x1a, 0x7b},//0x7a,
 	{0x03, 0x06},//0x0a,???��?����??��????

{0x0c, 0x0c},
{0x15, 0x02},//0x00
{0x3e, 0x00},//10
{0x70, 0x00},
{0x71, 0x01},
{0x72, 0x11},
{0x73, 0x09},//
        
{0xa2, 0x02},//15
{0x11, 0x00},//
{0x7a, 0x20},
{0x7b, 0x1c},
{0x7c, 0x28},
        
{0x7d, 0x3c},//20
{0x7e, 0x55},
{0x7f, 0x68},
{0x80, 0x76},
{0x81, 0x80},
        
{0x82, 0x88},
{0x83, 0x8f},
{0x84, 0x96},
{0x85, 0xa3},
{0x86, 0xaf},
        
{0x87, 0xc4},//30
{0x88, 0xd7},
{0x89, 0xe8},
{0x13, 0xe0},
{0x00, 0x00},//AGC
        
{0x10, 0x00},
{0x0d, 0x00},//
{0x14, 0x20},//0x38, limit the max gain
{0xa5, 0x05},
{0xab, 0x07},
        
{0x24, 0x75},//40
{0x25, 0x63},
{0x26, 0xA5},
{0x9f, 0x78},
{0xa0, 0x68},
        
{0xa1, 0x03},//0x0b,
{0xa6, 0xdf},//0xd8,
{0xa7, 0xdf},//0xd8,
{0xa8, 0xf0},
{0xa9, 0x90},
        
{0xaa, 0x94},//50
{0x13, 0xe5},
{0x0e, 0x61},
{0x0f, 0x4b},
{0x16, 0x02},
        
{0x1e, 0x37},//
{0x21, 0x02},
{0x22, 0x91},
{0x29, 0x07},
{0x33, 0x0b},
        
{0x35, 0x0b},//60
{0x37, 0x1d},
{0x38, 0x71},
{0x39, 0x2a},
{0x3c, 0x78},
        
{0x4d, 0x40},
{0x4e, 0x20},
{0x69, 0x5d},
{0x6b, 0x40},//PLL*4=48Mhz
{0x74, 0x19},
{0x8d, 0x4f},
        
{0x8e, 0x00},//70
{0x8f, 0x00},
{0x90, 0x00},
{0x91, 0x00},
{0x92, 0x00},//0x19,//0x66
        
{0x96, 0x00},
{0x9a, 0x80},
{0xb0, 0x84},
{0xb1, 0x0c},
{0xb2, 0x0e},
        
{0xb3, 0x82},//80
{0xb8, 0x0a},
{0x43, 0x14},
{0x44, 0xf0},
{0x45, 0x34},
        
{0x46, 0x58},
{0x47, 0x28},
{0x48, 0x3a},
{0x59, 0x88},
{0x5a, 0x88},
        
{0x5b, 0x44},//90
{0x5c, 0x67},
{0x5d, 0x49},
{0x5e, 0x0e},
{0x64, 0x04},
{0x65, 0x20},
        
{0x66, 0x05},
{0x94, 0x04},
{0x95, 0x08},
{0x6c, 0x0a},
{0x6d, 0x55},
        
        
{0x4f, 0x80},
{0x50, 0x80},
{0x51, 0x00},
{0x52, 0x22},
{0x53, 0x5e},
{0x54, 0x80},//ƫɫ���ã�ƫ�̣�ƫ����������ʼֵ0x80,������0xA0(160)

{0x09, 0x03},

{0x6e, 0x11},//100
{0x6f, 0x9e},//0x9e for advance AWB
  {0x55, 0x20},//����
  {0x56, 0x50},//�Աȶ�
  {0x57, 0x40},//0x40,  change according to Jim's request 

{0x6a, 0x40},//--���沿�ּĴ��������ã����Զ���ƽ��Ĺ���--me
{0x01, 0x40},
{0x02, 0x40},
{0x13, 0xe7},
{0x15, 0x02},  


{0x58, 0x9e},

{0x41, 0x08},
{0x3f, 0x00},
{0x75, 0x05},
{0x76, 0xe1},
{0x4c, 0x00},
{0x77, 0x01},
{0x3d, 0xc2}, 
{0x4b, 0x09},
{0xc9, 0x60},
{0x41, 0x38},

{0x34, 0x11},
{0x3b, 0x02}, 

{0xa4, 0x89},
{0x96, 0x00},
{0x97, 0x30},
{0x98, 0x20},
{0x99, 0x30},
{0x9a, 0x84},
{0x9b, 0x29},
{0x9c, 0x03},
{0x9d, 0x4c},
{0x9e, 0x3f},
{0x78, 0x04},

{0x79, 0x01},
{0xc8, 0xf0},
{0x79, 0x0f},
{0xc8, 0x00},
{0x79, 0x10},
{0xc8, 0x7e},
{0x79, 0x0a},
{0xc8, 0x80},
{0x79, 0x0b},
{0xc8, 0x01},
{0x79, 0x0c},
{0xc8, 0x0f},
{0x79, 0x0d},
{0xc8, 0x20},
{0x79, 0x09},
{0xc8, 0x80},
{0x79, 0x02},
{0xc8, 0xc0},
{0x79, 0x03},
{0xc8, 0x40},
{0x79, 0x05},
{0xc8, 0x30},
{0x79, 0x26}, 
{0x09, 0x00}, 
};

u8 wr_Sensor_Reg(u8 regID, u8 regDat)//дOV7670�Ĵ���
{
		startSCCB(); //����SCCB ���߿�ʼ��������
		if(0==SCCBwriteByte(0x42))//д��ַ
		{	
			stopSCCB();//����SCCB ����ֹͣ��������
			return(0);//���󷵻�
		}
		// UART_Put_Inf("wr_Sensor_Reg:",1);
		if(0==SCCBwriteByte(regID))//������ID
		{
			stopSCCB();//����SCCB ����ֹͣ��������
			return(0); //���󷵻�
		}
		// UART_Put_Inf("wr_Sensor_Reg:",2);
		if(0==SCCBwriteByte(regDat))//д���ݵ�������
		{
			stopSCCB();//����SCCB ����ֹͣ��������
			return(0);//���󷵻�
		}
		stopSCCB();//����SCCB ����ֹͣ��������
		// UART_Put_Inf("wr_Sensor_Reg:",3);
		return(1);//�ɹ�����
}

u8 rd_Sensor_Reg(u8 regID, u8 *regDat)//��OV7670�Ĵ���
{
		//ͨ��д�������üĴ�����ַ
		startSCCB();
		if(0==SCCBwriteByte(0x42))//д��ַ
		{
			stopSCCB();//����SCCB ����ֹͣ��������
			return(0);//���󷵻�
		}
		if(0==SCCBwriteByte(regID))//������ID
		{
			stopSCCB();//����SCCB ����ֹͣ��������
			return(0);//���󷵻�
		}
		stopSCCB();//����SCCB ����ֹͣ��������

		//���üĴ�����ַ�󣬲��Ƕ�
		startSCCB();
		if(0==SCCBwriteByte(0x43))//����ַ
		{
			stopSCCB();//����SCCB ����ֹͣ��������
			return(0);//���󷵻�
		}
		*regDat=SCCBreadByte();//���ض�����ֵ
		noAck();//����NACK����
		stopSCCB();//����SCCB ����ֹͣ��������
		return(1);//�ɹ�����
}
u8 Sensor_init(void)//����ͷоƬ��ʼ��
{
		u8 temp;	
		u8 i=0;

		temp=0x80;
		if(0==wr_Sensor_Reg(0x12,temp)) //Reset SCCB
		{
			return 0 ;//���󷵻�
		}							
		//printf("Sensor_init:%d\n\r",1);
		if(0==rd_Sensor_Reg(0x0b, &temp))//��ID
		{
			return 0 ;//���󷵻�
		}									
		//printf("Sensor_init:%d\n\r",temp);
		if(temp==0x73)//OV7670
		{																	
			for(i=0;i<OV7670_REG_NUM;i++)
			{
				//printf("Sensor_init:cycle%d\n\r",i);
				if(0==wr_Sensor_Reg(OV7670_reg[i][0],OV7670_reg[i][1]))
				{																															
					return 0;//���󷵻�
				}
			}
		}
		//printf("Sensor_init OK! %d\n\r",0);
		return 1; //ok
} 


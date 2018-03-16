#ifndef __OV7670_H
#define __OV7670_H
	 
#ifdef __cplusplus
 extern "C" {
#endif 
	 
void OV7670_Gpio_Init();
void SCCB_SID_change_in();//�����л�Ϊ����
void SCCB_SID_change_out();//�����л�Ϊ���

void FIFO_Reset_Read_Addr(void);//FIFO�����ݸ�λ
	 
void startSCCB(void);
void stopSCCB(void);
void noAck(void);
unsigned char getAck();
unsigned char SCCBwriteByte(unsigned char dat);
unsigned char SCCBreadByte(void);
	 
void FIFO_Reset_Read_Addr(void);

unsigned char wr_Sensor_Reg(unsigned char regID, unsigned char regDat);
unsigned char rd_Sensor_Reg(unsigned char regID, unsigned char *regDat);
unsigned char Sensor_init(void);
	 
#ifdef __cplusplus
}
#endif


#endif 

#include "nrf24l01.h"
 
const u8 TX_ADDRESS_S[TX_ADR_WIDTH]={0x34,0x43,0x10,0x10,0x01}; //���͵�ַ
const u8 RX_ADDRESS_S[RX_ADR_WIDTH]={0x34,0x43,0x10,0x10,0x01}; //���յ�ַ					    
const u8 RX_ADDRESS_M[RX_ADR_WIDTH]={0x34,0x43,0x10,0x10,0x01}; //���յ�ַ	
 
void Delay(vu32 nCount)
{
  for(; nCount != 0; nCount--);
}
 
//��ʼ��NRF24L01IO��
 
//CE->PA4,CSN->PA3,SCK->PA5,MOSI->PA7,MISO->PA6,IRQ->PB0
void Init_NRF24L01(void)
{
	//CSN->PA3,SCK->PA5,MOSI->PA7��CE->PA4
	GPIO_InitTypeDef GPIO_InitStructure;			
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA,ENABLE);	//ʹ��GPIO ��ʱ��
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB,ENABLE);	
	
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3|GPIO_Pin_4|GPIO_Pin_5|GPIO_Pin_7;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;    	//�������
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	
	CE_H;           //��ʼ��ʱ������
  CSN_H;					//��ʼ��ʱ������
	
	//MISO->PA6,
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;     //��������
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	  //IRQ->PB0,
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;    	
	GPIO_Init(GPIOB, &GPIO_InitStructure);
		
	IRQ_H;			 							//IRQ�ø�
	CE_L; 	                  //ʹ��NRF24L01
	CSN_H;                    //SPIƬѡȡ��
}
 
//ģ��SPI��д���ݺ���
u8 SPI_ReadWriteByte(u8 TxData)                                        
{		
	u16 bit_ctr;
   	for(bit_ctr=0;bit_ctr<8;bit_ctr++) 
   	{
		if(TxData & 0x80)
		MOSI_H;         
		else
		MOSI_L;
		TxData = (TxData << 1);           
		SCK_H; 
		Delay(0xff);
		if(READ_MISO)                     
		TxData |= 0x01;       		  
		SCK_L; 
		Delay(0xff);           		 
   	}
    return(TxData);           		  		    
}
 
//�ϵ���NRF24L01�Ƿ���λ
//д5������Ȼ���ٶ��������бȽϣ�
//��ͬʱ����ֵ0����ʾ��λ;
//���򷵻�1����ʾ����λ.	
u8 NRF24L01_Check(void)
{
	u8 buf[5]={0XA5,0XA5,0XA5,0XA5,0XA5};
	u8 buf1[5];
	u8 i;   	 
	NRF24L01_Write_Buf(SPI_WRITE_REG+TX_ADDR,buf,5);//д��5���ֽڵĵ�ַ.	
	NRF24L01_Read_Buf(TX_ADDR,buf1,5);              //����д��ĵ�ַ  	
	for(i=0;i<5;i++)
		if(buf1[i]!=0XA5) break;					   
	if(i!=5) return 1;                               //NRF24L01����λ
		
	return 0;		                                //NRF24L01��λ
}	 	 
//ͨ��SPIд�Ĵ���
u8 NRF24L01_Write_Reg(u8 reg_addr,u8 data)
{
	u8 status;	
    CSN_L;                    //ʹ��SPI����
  	status =SPI_ReadWriteByte(reg_addr); //���ͼĴ����� 
  	SPI_ReadWriteByte(data);            //д��Ĵ�����ֵ
  	CSN_H;                    //��ֹSPI����	   
  	return(status);       		         //����״ֵ̬
}
//��ȡSPI�Ĵ���ֵ ��regaddr:Ҫ���ļĴ���
u8 NRF24L01_Read_Reg(u8 reg_addr)
{
	u8 reg_val;	    
 	CSN_L;                //ʹ��SPI����		
  	SPI_ReadWriteByte(reg_addr);     //���ͼĴ�����
  	reg_val=SPI_ReadWriteByte(reg_addr);//��ȡ�Ĵ�������
 
  	CSN_H;                //��ֹSPI����		    
  	return(reg_val);                 //����״ֵ̬
}	
//��ָ��λ�ö���ָ�����ȵ�����
//*pBuf:����ָ��
//����ֵ,�˴ζ�����״̬�Ĵ���ֵ 
u8 NRF24L01_Read_Buf(u8 reg_addr,u8 *pBuf,u8 data_len)
{
	u8 status,i;	       
  	CSN_L;                     //ʹ��SPI����
  	status=SPI_ReadWriteByte(reg_addr);   //���ͼĴ���ֵ(λ��),����ȡ״ֵ̬   	   
 	for(i=0;i<data_len;i++)
		pBuf[i]=SPI_ReadWriteByte(0);//��������
 
  	CSN_H;                     //�ر�SPI����
  	return status;                        //���ض�����״ֵ̬
}
//��ָ��λ��дָ�����ȵ�����
//*pBuf:����ָ��
//����ֵ,�˴ζ�����״̬�Ĵ���ֵ
u8 NRF24L01_Write_Buf(u8 reg_addr, u8 *pBuf, u8 data_len)
{
	u8 status,i;	    
 	CSN_L;                                    //ʹ��SPI����
  	status = SPI_ReadWriteByte(reg_addr);                //���ͼĴ���ֵ(λ��),����ȡ״ֵ̬
  	for(i=0; i<data_len; i++)
		SPI_ReadWriteByte(*pBuf++); //д������	 
  	CSN_H;                                    //�ر�SPI����
  	return status;                                       //���ض�����״ֵ̬
}				   
//����NRF24L01����һ������
//txbuf:�����������׵�ַ
//����ֵ:�������״��
u8 NRF24L01_TxPacket(u8 *tx_buf)
{
	u8 state;   
	CE_L;
  	NRF24L01_Write_Buf(WR_TX_PLOAD,tx_buf,TX_PLOAD_WIDTH);//д���ݵ�TX BUF  32���ֽ�
 	CE_H;                                     //��������	   
	while(READ_IRQ != 0);                         //�ȴ��������
	state=NRF24L01_Read_Reg(STATUS);                     //��ȡ״̬�Ĵ�����ֵ	   
	NRF24L01_Write_Reg(SPI_WRITE_REG+STATUS,state);      //���TX_DS��MAX_RT�жϱ�־
	if(state&MAX_TX)                                     //�ﵽ����ط�����
	{
		NRF24L01_Write_Reg(FLUSH_TX,0xff);               //���TX FIFO�Ĵ��� 
		return MAX_TX; 
	}
	if(state&TX_OK)                                      //�������
	{
		return TX_OK;
	}
	return 0xff;                                         //����ԭ����ʧ��
}
 
//����NRF24L01����һ������
//txbuf:�����������׵�ַ
//����ֵ:0��������ɣ��������������
u8 NRF24L01_RxPacket(u8 *rx_buf)
{
	u8 state;		    							      
	state=NRF24L01_Read_Reg(STATUS);                //��ȡ״̬�Ĵ�����ֵ    	 
	NRF24L01_Write_Reg(SPI_WRITE_REG+STATUS,state); //���TX_DS��MAX_RT�жϱ�־
	if(state&RX_OK)                                 //���յ�����
	{
		NRF24L01_Read_Buf(RD_RX_PLOAD,rx_buf,RX_PLOAD_WIDTH);//��ȡ����
		NRF24L01_Write_Reg(FLUSH_RX,0xff);          //���RX FIFO�Ĵ��� 
		return 0; 
	}	   
	return 1;                                      //û�յ��κ�����
}
 
//�ú�����ʼ��NRF24L01��RXģʽ
//����RX��ַ,дRX���ݿ��,ѡ��RFƵ��,�����ʺ�LNA HCURR
//��CE��ߺ�,������RXģʽ,�����Խ���������		   
void RX_Mode(void)
{
	CE_L;	  
    //дRX�ڵ��ַ
  	NRF24L01_Write_Buf(SPI_WRITE_REG+RX_ADDR_P0,(u8*)RX_ADDRESS_S,RX_ADR_WIDTH);
 
    //ʹ��ͨ��0���Զ�Ӧ��    
  	NRF24L01_Write_Reg(SPI_WRITE_REG+EN_AA,0x01);    
    //ʹ��ͨ��0�Ľ��յ�ַ  	 
  	NRF24L01_Write_Reg(SPI_WRITE_REG+EN_RXADDR,0x01);
    //����RFͨ��Ƶ��		  
  	NRF24L01_Write_Reg(SPI_WRITE_REG+RF_CH,40);	     
    //ѡ��ͨ��0����Ч���ݿ�� 	    
  	NRF24L01_Write_Reg(SPI_WRITE_REG+RX_PW_P0,RX_PLOAD_WIDTH);
    //����TX�������,0db����,2Mbps,���������濪��   
  	NRF24L01_Write_Reg(SPI_WRITE_REG+RF_SETUP,0x0f);
    //���û�������ģʽ�Ĳ���;PWR_UP,EN_CRC,16BIT_CRC,PRIM_RX����ģʽ 
  	NRF24L01_Write_Reg(SPI_WRITE_REG+CONFIG, 0x0f); 
    //CEΪ��,�������ģʽ 
  	CE_H;                                
}			
 
//�ú�����ʼ��NRF24L01��TXģʽ
//����TX��ַ,дTX���ݿ��,����RX�Զ�Ӧ��ĵ�ַ,���TX��������,
//ѡ��RFƵ��,�����ʺ�LNA HCURR PWR_UP,CRCʹ��
//��CE��ߺ�,������RXģʽ,�����Խ���������		   
//CEΪ�ߴ���10us,����������.	 
void TX_Mode(void)
{														 
	CE_L;	    
    //дTX�ڵ��ַ 
  	NRF24L01_Write_Buf(SPI_WRITE_REG+TX_ADDR,(u8*)TX_ADDRESS_S,TX_ADR_WIDTH);    
    //����TX�ڵ��ַ,��ҪΪ��ʹ��ACK	  
  	NRF24L01_Write_Buf(SPI_WRITE_REG+RX_ADDR_P0,(u8*)RX_ADDRESS_S,RX_ADR_WIDTH); 
 
    //ʹ��ͨ��0���Զ�Ӧ��    
  	NRF24L01_Write_Reg(SPI_WRITE_REG+EN_AA,0x01);     
    //ʹ��ͨ��0�Ľ��յ�ַ  
  	NRF24L01_Write_Reg(SPI_WRITE_REG+EN_RXADDR,0x01); 
    //�����Զ��ط����ʱ��:500us + 86us;����Զ��ط�����:10��
  	NRF24L01_Write_Reg(SPI_WRITE_REG+SETUP_RETR,0x1a);
    //����RFͨ��Ϊ40
  	NRF24L01_Write_Reg(SPI_WRITE_REG+RF_CH,40);       
    //����TX�������,0db����,2Mbps,���������濪��   
  	NRF24L01_Write_Reg(SPI_WRITE_REG+RF_SETUP,0x0f);  
    //���û�������ģʽ�Ĳ���;PWR_UP,EN_CRC,16BIT_CRC,PRIM_RX����ģʽ,���������ж�
  	NRF24L01_Write_Reg(SPI_WRITE_REG+CONFIG,0x0e);    
    // CEΪ��,10us����������
	CE_H;                                  
}		  
 

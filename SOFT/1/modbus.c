
#include <stm32f10x_lib.h>
#include <stm32f10x_conf.h>
#include "stm32_Reg.h"
#include "modbus.h"
#include "main.h"
#include "beep.h"
#include "uart1.h"
#include "cmd.h"
#include "control.h"
#include <string.h>
#include <stdio.h>
#include "eeprom_map.h"
#include "rtl.h"
//#include "modbus_tcp.h"
#include "25lc640.h"
//#include "sc16is7xx.h"
//#include "uart0.h"
#include "avar_hndl.h"

#define can1_out mcp2515_transmit

#define MODBUS_RTU_PROT	0
#define MODBUS_TCP_PROT	1

//extern int  mem_copy (void *dp, void *sp, int len);

short modbus_error_cntr;

unsigned char modbus_buf[20];
short modbus_crc16;
char modbus_timeout_cnt;
char bMODBUS_TIMEOUT;
unsigned char modbus_rx_buffer_d1[500];
unsigned char modbus_rx_buffer[100];		//�����, ���� ���������� ����������� ������� ���������� ���������� �� ������ ����� 
unsigned char modbus_rx_buffer_d2[500];
unsigned char modbus_an_buffer[100];    	//�����, ���� ��� ����� ���������� ��� �������
unsigned char modbus_rx_buffer_ptr;		//��������� �� ������� ������� ������������ ������
unsigned char modbus_rx_counter;		//���������� �������� ����, ������������ ��� ������� ����������� ������� � ��� �����������

short modbus_plazma;				//�������
short modbus_plazma1;				//�������
short modbus_plazma2;				//�������
short modbus_plazma3;				//�������
short modbus_plazma_p;				//�������
short modbus_plazma_pp;				//�������

unsigned short modbus_rx_arg0;		//���������� � ������� ������ ��������
unsigned short modbus_rx_arg1;		//���������� � ������� ������ ��������
unsigned short modbus_rx_arg2;		//���������� � ������� ������ ��������
unsigned short modbus_rx_arg3;		//���������� � ������� ��������� ��������

char modbus_tx_buff[500];

char* modbus_tcp_out_ptr;

short modbus_register_offset;
short modbus_register_offset_ui;
short modbus_register_offset_un;
short modbus_register_offset_i;
short modbus_register_offset_t;
short modbus_register_995, modbus_register_996, modbus_register_997;
short modbus_register_998;
short modbus_register_999;
short modbus_register_1000, modbus_register_1001, modbus_register_1002, modbus_register_1003;
short modbus_register_1004, modbus_register_1005, modbus_register_1006, modbus_register_1007;
short modbus_register_1008, modbus_register_1009, modbus_register_1010, modbus_register_1011;
short modbus_register_1012, modbus_register_1013, modbus_register_1014, modbus_register_1015;
short modbus_register_1016, modbus_register_1017, modbus_register_1018, modbus_register_1019;
short modbus_register_1020, modbus_register_1021;
short modbus_register_1022, modbus_register_1023, modbus_register_1024, modbus_register_1025;
short modbus_register_1027, modbus_register_1028, modbus_register_1029, modbus_register_1030;
//char modbus_registers[200];

//static const char foo[] = "I wish I'd read K&R, and other tomes more diligently";



/*modbus_registers[3]=(char)(bps_I%256);
modbus_registers[4]=(char)(net_U/256);					//���3   	���������� ���� �������, 1�
modbus_registers[5]=(char)(net_U%256);
modbus_registers[6]=(char)(net_F/256);					//���4   	������� ���� �������, 0.1��
modbus_registers[7]=(char)(net_F%256);
modbus_registers[8]=(char)(net_Ua/256);					//���5	���������� ���� ������� ���� A, 1�	
modbus_registers[9]=(char)(net_Ua%256);		 	
modbus_registers[10]=(char)(net_Ub/256);				//���6	���������� ���� ������� ���� B, 1�
modbus_registers[11]=(char)(net_Ub%256);
modbus_registers[12]=(char)(net_Uc/256);				//���7	���������� ���� ������� ���� C, 1�
modbus_registers[13]=(char)(net_Uc%256);
modbus_registers[14]=(char)(bat[0]._Ub/256);				//���8	���������� ������� �1, 0.1�
modbus_registers[15]=(char)(bat[0]._Ub%256);
modbus_registers[16]=(char)(bat[0]._Ib/256);				//���9   	��� ������� �1, 0.01�
modbus_registers[17]=(char)(bat[0]._Ib%256);
modbus_registers[18]=(char)(bat[0]._Tb/256);				//���10	����������� ������� �1, 1��
modbus_registers[19]=(char)(bat[0]._Tb%256);
modbus_registers[20]=(char)(bat[0]._zar/256);			//���11	����� ������� �1, %
modbus_registers[21]=(char)(bat[0]._zar%256);
modbus_registers[22]=(char)(bat[0]._Ubm/256);			//���12	���������� ������� ����� ������� �1, 0.1�
modbus_registers[23]=(char)(bat[0]._Ubm%256);
modbus_registers[24]=(char)(bat[0]._dUbm/256);			//���13	������ ������� ����� ������� �1, %
modbus_registers[25]=(char)(bat[0]._dUbm%256);
modbus_registers[26]=(char)(BAT_C_REAL[0]/256);			//���14	�������� ������� ������� �1, 0.1�*�, ���� 0x5555 �� �� ����������
modbus_registers[27]=(char)(BAT_C_REAL[0]%256);
modbus_registers[28]=(char)(bat[1]._Ub/256);				//���15	���������� ������� �1, 0.1�
modbus_registers[29]=(char)(bat[1]._Ub%256);
modbus_registers[30]=(char)(bat[1]._Ib/256);				//���16   	��� ������� �1, 0.01�
modbus_registers[31]=(char)(bat[1]._Ib%256);
modbus_registers[32]=(char)(bat[1]._Tb/256);				//���17	����������� ������� �1, 1��
modbus_registers[33]=(char)(bat[1]._Tb%256);
modbus_registers[34]=(char)(bat[1]._zar/256);			//���18	����� ������� �1, %
modbus_registers[35]=(char)(bat[1]._zar%256);
modbus_registers[36]=(char)(bat[1]._Ubm/256);			//���19	���������� ������� ����� ������� �1, 0.1�
modbus_registers[37]=(char)(bat[1]._Ubm%256);
modbus_registers[38]=(char)(bat[1]._dUbm/256);			//���20	������ ������� ����� ������� �1, %
modbus_registers[39]=(char)(bat[1]._dUbm%256);
modbus_registers[40]=(char)(BAT_C_REAL[1]/256);			//���21	�������� ������� ������� �1, 0.1�*�, ���� 0x5555 �� �� ����������
modbus_registers[41]=(char)(BAT_C_REAL[1]%256);
modbus_registers[42]=(char)(bps[0]._Uii/256);			//���22	�������� ���������� ����������� �1, 0.1�
modbus_registers[43]=(char)(bps[0]._Uii%256);
modbus_registers[44]=(char)(bps[0]._Ii/256);				//���23	�������� ��� ����������� �1, 0.1�
modbus_registers[45]=(char)(bps[0]._Ii%256);
modbus_registers[46]=(char)(bps[0]._Ti/256);				//���24	����������� ��������� ����������� �1, 1��
modbus_registers[47]=(char)(bps[0]._Ti%256);
modbus_registers[48]=(char)(bps[0]._av/256);				//���25	���� ������ ����������� �1, 0x01 - ��������, 0x02 �������� U���, 0x04 �������� U���, 0x08 - ����������� ����� � ������������
modbus_registers[49]=(char)(bps[0]._av%256);
modbus_registers[50]=(char)(bps[1]._Uii/256);			//���26	�������� ���������� ����������� �2, 0.1�
modbus_registers[51]=(char)(bps[1]._Uii%256);
modbus_registers[52]=(char)(bps[1]._Ii/256);				//���27	�������� ��� ����������� �2, 0.1�
modbus_registers[53]=(char)(bps[1]._Ii%256);
modbus_registers[54]=(char)(bps[1]._Ti/256);				//���28	����������� ��������� ����������� �2, 1��
modbus_registers[55]=(char)(bps[1]._Ti%256);
modbus_registers[56]=(char)(bps[1]._av/256);				//���29	���� ������ ����������� �2, 0x01 - ��������, 0x02 �������� U���, 0x04 �������� U���, 0x08 - ����������� ����� � ������������
modbus_registers[57]=(char)(bps[1]._av%256);
modbus_registers[58]=(char)(bps[2]._Uii/256);			//���30	�������� ���������� ����������� �3, 0.1�
modbus_registers[59]=(char)(bps[2]._Uii%256);
modbus_registers[60]=(char)(bps[2]._Ii/256);				//���31	�������� ��� ����������� �3, 0.1�
modbus_registers[61]=(char)(bps[2]._Ii%256);
modbus_registers[62]=(char)(bps[2]._Ti/256);				//���32	����������� ��������� ����������� �3, 1��
modbus_registers[63]=(char)(bps[2]._Ti%256);
modbus_registers[64]=(char)(bps[2]._av/256);				//���33	���� ������ ����������� �3, 0x01 - ��������, 0x02 �������� U���, 0x04 �������� U���, 0x08 - ����������� ����� � ������������
modbus_registers[65]=(char)(bps[2]._av%256);
modbus_registers[66]=(char)(bps[3]._Uii/256);			//���34	�������� ���������� ����������� �4, 0.1�
modbus_registers[67]=(char)(bps[3]._Uii%256);
modbus_registers[68]=(char)(bps[3]._Ii/256);				//���35	�������� ��� ����������� �4, 0.1�
modbus_registers[69]=(char)(bps[3]._Ii%256);
modbus_registers[70]=(char)(bps[3]._Ti/256);				//���36	����������� ��������� ����������� �4, 1��
modbus_registers[71]=(char)(bps[3]._Ti%256);
modbus_registers[72]=(char)(bps[3]._av/256);				//���37	���� ������ ����������� �4, 0x01 - ��������, 0x02 �������� U���, 0x04 �������� U���, 0x08 - ����������� ����� � ������������
modbus_registers[73]=(char)(bps[3]._av%256);
modbus_registers[74]=(char)(bps[4]._Uii/256);			//���38	�������� ���������� ����������� �5, 0.1�
modbus_registers[75]=(char)(bps[4]._Uii%256);
modbus_registers[76]=(char)(bps[4]._Ii/256);				//���39	�������� ��� ����������� �5, 0.1�
modbus_registers[77]=(char)(bps[4]._Ii%256);
modbus_registers[78]=(char)(bps[4]._Ti/256);				//���40	����������� ��������� ����������� �5, 1��
modbus_registers[79]=(char)(bps[4]._Ti%256);
modbus_registers[80]=(char)(bps[4]._av/256);				//���41	���� ������ ����������� �5, 0x01 - ��������, 0x02 �������� U���, 0x04 �������� U���, 0x08 - ����������� ����� � ������������
modbus_registers[81]=(char)(bps[4]._av%256);
modbus_registers[82]=(char)(bps[5]._Uii/256);			//���42	�������� ���������� ����������� �6, 0.1�
modbus_registers[83]=(char)(bps[5]._Uii%256);
modbus_registers[84]=(char)(bps[5]._Ii/256);				//���43	�������� ��� ����������� �6, 0.1�
modbus_registers[85]=(char)(bps[5]._Ii%256);
modbus_registers[86]=(char)(bps[5]._Ti/256);				//���44	����������� ��������� ����������� �6, 1��
modbus_registers[87]=(char)(bps[5]._Ti%256);
modbus_registers[88]=(char)(bps[5]._av/256);				//���45	���� ������ ����������� �6, 0x01 - ��������, 0x02 �������� U���, 0x04 �������� U���, 0x08 - ����������� ����� � ������������
modbus_registers[89]=(char)(bps[5]._av%256);
modbus_registers[90]=(char)(bps[6]._Uii/256);			//���46	�������� ���������� ����������� �7, 0.1�
modbus_registers[91]=(char)(bps[6]._Uii%256);
modbus_registers[92]=(char)(bps[6]._Ii/256);				//���47	�������� ��� ����������� �7, 0.1�
modbus_registers[93]=(char)(bps[6]._Ii%256);
modbus_registers[94]=(char)(bps[6]._Ti/256);				//���48	����������� ��������� ����������� �7, 1��
modbus_registers[95]=(char)(bps[6]._Ti%256);
modbus_registers[96]=(char)(bps[6]._av/256);				//���49	���� ������ ����������� �7, 0x01 - ��������, 0x02 �������� U���, 0x04 �������� U���, 0x08 - ����������� ����� � ������������
modbus_registers[97]=(char)(bps[6]._av%256);
modbus_registers[98]=(char)(bps[7]._Uii/256);			//���50	�������� ���������� ����������� �8, 0.1�
modbus_registers[99]=(char)(bps[7]._Uii%256);
modbus_registers[100]=(char)(bps[7]._Ii/256);			//���51	�������� ��� ����������� �8, 0.1�
modbus_registers[101]=(char)(bps[7]._Ii%256);
modbus_registers[102]=(char)(bps[7]._Ti/256);			//���52	����������� ��������� ����������� �8, 1��
modbus_registers[103]=(char)(bps[7]._Ti%256);
modbus_registers[104]=(char)(bps[7]._av/256);			//���53	���� ������ ����������� �8, 0x01 - ��������, 0x02 �������� U���, 0x04 �������� U���, 0x08 - ����������� ����� � ������������
modbus_registers[105]=(char)(bps[7]._av%256);
modbus_registers[106]=(char)(bps_U/256);					//���54   	���������� ������������, 0.1�
modbus_registers[107]=(char)(bps_U%256);
tempS=0;
if(speedChIsOn) tempS=1;
modbus_registers[108]=(char)(tempS/256);					//���55   	���������� ����� ������������, (1 - ���, 0 - ����)
modbus_registers[109]=(char)(tempS%256);
tempS=0;
if(spc_stat==spcVZ) tempS=1;
modbus_registers[110]=(char)(tempS/256);					//���56   	������������� ����� ������������, (1 - ���, 0 - ����)
modbus_registers[111]=(char)(tempS%256);
modbus_registers[112]=(char)(uout_av/256);					//���57   �������� ��������� ����������, (0 - �����, 1 - ��������, 2 - ��������)
modbus_registers[113]=(char)(uout_av%256);

tempS=t_ext[0];
if(ND_EXT[0])tempS=-1000;
modbus_registers[400]=(char)(tempS/256);				//���201	������� ������ ����������� �1
modbus_registers[401]=(char)(tempS%256);
tempS=t_ext[1];
if(ND_EXT[1])tempS=-1000;
modbus_registers[402]=(char)(tempS/256);				//���202	������� ������ ����������� �2
modbus_registers[403]=(char)(tempS%256);
tempS=t_ext[2];
if(ND_EXT[2])tempS=-1000;
modbus_registers[404]=(char)(tempS/256);				//���203	������� ������ ����������� �3
modbus_registers[405]=(char)(tempS%256);
tempS=t_ext[3];
if(ND_EXT[3])tempS=-1000;
modbus_registers[406]=(char)(tempS/256);				//���204	������� ������ ����������� �4
modbus_registers[407]=(char)(tempS%256);

tempS=0;
if(sk_stat[0]==ssON) tempS|=0x0001;
if(sk_av_stat[0]==sasON) tempS|=0x0002;
modbus_registers[420]=(char)(tempS/256);				//���211	���������  ������ �������� �1, (������� ��� - ���������� ���������, 1 - �������, 0 - ���������, ������ ��� - �����������, 1 - ������, 0 - �����)
modbus_registers[421]=(char)(tempS%256);
tempS=0;
if(sk_stat[1]==ssON) tempS|=0x0001;
if(sk_av_stat[1]==sasON) tempS|=0x0002;
modbus_registers[422]=(char)(tempS/256);				//���212	���������  ������ �������� �1, (������� ��� - ���������� ���������, 1 - �������, 0 - ���������, ������ ��� - �����������, 1 - ������, 0 - �����)
modbus_registers[423]=(char)(tempS%256);
tempS=0;
if(sk_stat[2]==ssON) tempS|=0x0001;
if(sk_av_stat[2]==sasON) tempS|=0x0002;
modbus_registers[424]=(char)(tempS/256);				//���213	���������  ������ �������� �1, (������� ��� - ���������� ���������, 1 - �������, 0 - ���������, ������ ��� - �����������, 1 - ������, 0 - �����)
modbus_registers[425]=(char)(tempS%256);
tempS=0;
if(sk_stat[3]==ssON) tempS|=0x0001;
if(sk_av_stat[3]==sasON) tempS|=0x0002;
modbus_registers[426]=(char)(tempS/256);				//���214	���������  ������ �������� �1, (������� ��� - ���������� ���������, 1 - �������, 0 - ���������, ������ ��� - �����������, 1 - ������, 0 - �����)
modbus_registers[427]=(char)(tempS%256);

if(prot==MODBUS_RTU_PROT)
	{
	modbus_tx_buff[0]=adr;
	modbus_tx_buff[1]=func;

	modbus_tx_buff[2]=(char)(reg_quantity*2);

	mem_copy((char*)&modbus_tx_buff[3],(char*)&modbus_registers[(reg_adr-1)*2],reg_quantity*2);

	crc_temp=CRC16_2(modbus_tx_buff,(reg_quantity*2)+3);

	modbus_tx_buff[3+(reg_quantity*2)]=crc_temp%256;
	modbus_tx_buff[4+(reg_quantity*2)]=crc_temp/256;

	for (i=0;i<(5+(reg_quantity*2));i++)
		{
		putchar0(modbus_tx_buff[i]);
		}
	for (i=0;i<(5+(reg_quantity*2));i++)
		{
		putchar_sc16is700(modbus_tx_buff[i]);
		}
	}
else if(prot==MODBUS_TCP_PROT)
	{
	modbus_tcp_out_ptr=(char*)&modbus_registers[(reg_adr-1)*2];
	} */

	//   	};


//-----------------------------------------------
unsigned short CRC16_2(char* buf, short len)
{
unsigned short crc = 0xFFFF;
short pos;
short i;

for (pos = 0; pos < len; pos++)
  	{
    	crc ^= (unsigned short)buf[pos];          // XOR byte into least sig. byte of crc

    	for ( i = 8; i != 0; i--) 
		{    // Loop over each bit
      	if ((crc & 0x0001) != 0) 
			{      // If the LSB is set
        		crc >>= 1;                    // Shift right and XOR 0xA001
        		crc ^= 0xA001;
      		}
      	else  crc >>= 1;                    // Just shift right
    		}
  	}
  // Note, this number has low and high bytes swapped, so use it accordingly (or swap bytes)
return crc;
}

/*
//-----------------------------------------------
void modbus_zapros_ENMV (void){	 //o_2_s
unsigned short crc_temp;
unsigned char i_cnt;
	modbus_tx_buff[0]=MODBUS_ADRESS;
	modbus_tx_buff[1]=1;
	modbus_tx_buff[2]=0;
	modbus_tx_buff[3]=0;
	modbus_tx_buff[4]=0;
	modbus_tx_buff[5]=64;
	crc_temp=CRC16_2(modbus_tx_buff,6);
	modbus_tx_buff[6]=(char)crc_temp;
	modbus_tx_buff[7]=crc_temp>>8;
	for (i_cnt=0;i_cnt<8;i_cnt++)	putchar_sc16is700(modbus_tx_buff[i_cnt]);
	if(enmv_on<5)enmv_on++;
   	else {for (i_cnt=0;i_cnt<64;i_cnt++) snmp_enmv_data[i_cnt]=0xFF;}
} //o_2_e*/

//-----------------------------------------------
void modbus_in(void)
{
char i;
short crc16_calculated;		//����������� �� �������� ������ CRC
short crc16_incapsulated;	//����������� � ������� CRC
signed short modbus_rx_arg0;		//���������� � ������� ������ ��������
signed short modbus_rx_arg1;		//���������� � ������� ������ ��������
//unsigned short modbus_rx_arg2;		//���������� � ������� ������ ��������
//unsigned short modbus_rx_arg3;		//���������� � ������� ��������� ��������
unsigned char modbus_func;			//���������� � ������� ��� �������
char i_cnt, j_cnt; //o_2

plazma_uart1[1]++;

memcpy(modbus_an_buffer,modbus_rx_buffer,modbus_rx_buffer_ptr);
modbus_rx_counter=modbus_rx_buffer_ptr;
modbus_rx_buffer_ptr=0;
bMODBUS_TIMEOUT=0;
	
crc16_calculated  = CRC16_2((char*)modbus_an_buffer, modbus_rx_counter-2);
crc16_incapsulated = *((short*)&modbus_an_buffer[modbus_rx_counter-2]);

//modbus_plazma1=modbus_rx_counter;
//modbus_plazma2=crc16_calculated;
//modbus_plazma3=crc16_incapsulated;

modbus_func=modbus_an_buffer[1];
modbus_rx_arg0=(((unsigned short)modbus_an_buffer[2])*((unsigned short)256))+((unsigned short)modbus_an_buffer[3]);
modbus_rx_arg1=(((unsigned short)modbus_an_buffer[4])*((unsigned short)256))+((unsigned short)modbus_an_buffer[5]);
//modbus_rx_arg2=(((unsigned short)modbus_an_buffer[6])*((unsigned short)256))+((unsigned short)modbus_an_buffer[7]);
//modbus_rx_arg3=(((unsigned short)modbus_an_buffer[8])*((unsigned short)256))+((unsigned short)modbus_an_buffer[9]);

//#define IPS_CURR_AVG_MODBUS_ADRESS	1

if(modbus_an_buffer[0]=='r')
	{
	//pvlk=1;
	if(modbus_an_buffer[1]=='e')
		{
		//pvlk=2;
		if(modbus_an_buffer[2]=='a')
			{
			//pvlk=3;
			if(modbus_an_buffer[3]=='d')
				{
				//pvlk=4;
				if(modbus_an_buffer[6]==crc_87((char*)modbus_an_buffer,6))
					{
					//pvlk=5;
					if(modbus_an_buffer[7]==crc_95((char*)modbus_an_buffer,6))
						{
						//pvlk=6;	

							{
							unsigned short ptr;
							unsigned long data1,data2;
							char temp_out[20];
							//pvlk++;
							ptr=modbus_an_buffer[4]+(modbus_an_buffer[5]*256U);
							data1=lc640_read_long(ptr);
							data2=lc640_read_long(ptr+4);
							temp_out[0]='r';
							temp_out[1]='e';
							temp_out[2]='a';
							temp_out[3]='d';
							temp_out[4]=*((char*)&ptr);
							temp_out[5]=*(((char*)&ptr)+1);	
							temp_out[6]=*((char*)&data1);
							temp_out[7]=*(((char*)&data1)+1);		
							temp_out[8]=*(((char*)&data1)+2);	
							temp_out[9]=*(((char*)&data1)+3);		
							temp_out[10]=*((char*)&data2);
							temp_out[11]=*(((char*)&data2)+1);		
							temp_out[12]=*(((char*)&data2)+2);	
							temp_out[13]=*(((char*)&data2)+3);	
							temp_out[14]=crc_87(temp_out,14);	
							temp_out[15]=crc_95(temp_out,14);			
							
							temp_out[17]=0;
							for (i=0;i<16;i++)
								{
								//*/putchar_sc16is700(temp_out[i]);
								temp_out[17]^=temp_out[i];
								}
							//*/putchar_sc16is700(16);
							//*/putchar_sc16is700(temp_out[17]^16);
							//*/putchar_sc16is700(0x0a);
							}
						}
					}
				}
			} 
		}	 
	} 

if(modbus_an_buffer[0]=='w')
	{
//	pvlk=1;
	if(modbus_an_buffer[1]=='r')
		{
//		pvlk=2;
		if(modbus_an_buffer[2]=='i')
			{
//			pvlk=3;
			if(modbus_an_buffer[3]=='t')
				{
//				pvlk=4;
				if(modbus_an_buffer[4]=='e')
					{
//					pvlk=5;
					if(modbus_an_buffer[15]==crc_87((char*)modbus_an_buffer,15))
						{
//						pvlk=6;
						if(modbus_an_buffer[16]==crc_95((char*)modbus_an_buffer,15))

							{
							unsigned short ptr;
							unsigned long data1,data2;
							char temp_out[20];
//							pvlk=7;
							ptr=modbus_an_buffer[5]+(modbus_an_buffer[6]*256U);
							*((char*)&data1)=modbus_an_buffer[7];
							*(((char*)&data1)+1)=modbus_an_buffer[8];
							*(((char*)&data1)+2)=modbus_an_buffer[9];
							*(((char*)&data1)+3)=modbus_an_buffer[10];
							*((char*)&data2)=modbus_an_buffer[11];
							*(((char*)&data2)+1)=modbus_an_buffer[12];
							*(((char*)&data2)+2)=modbus_an_buffer[13];
							*(((char*)&data2)+3)=modbus_an_buffer[14];	
							lc640_write_long(ptr,data1);
							lc640_write_long(ptr+4,data2);
							
							//data1=lc640_read_long(ptr);
							//data2=lc640_read_long(ptr+4);
							temp_out[0]='w';
							temp_out[1]='r';
							temp_out[2]='i';
							temp_out[3]='t';
							temp_out[4]='e';
							temp_out[5]=*((char*)&ptr);
							temp_out[6]=*(((char*)&ptr)+1);	
						
							temp_out[7]=crc_87(temp_out,7);	
							temp_out[8]=crc_95(temp_out,7);			
							
							temp_out[10]=0;
							for (i=0;i<9;i++)
								{
								//*/putchar_sc16is700(temp_out[i]);
								temp_out[10]^=temp_out[i];
								}
							//*/putchar_sc16is700(9);
							//*/putchar_sc16is700(temp_out[10]^9);
							//*/putchar_sc16is700(0x0a);
							}
						}
					}
				}
		   	}
		}
	}

if(crc16_calculated==crc16_incapsulated)
	{
	
	
 	if(modbus_an_buffer[0]==MODBUS_ADRESS)
		{
		modbus_error_cntr=0;
		modbus_plazma++;
		//modbus_modbus_adress_eq++;
		if(modbus_func==3)		//������ ������������� ���-�� ��������� ��������
			{
			
			 modbus_plazma1++;
			modbus_plazma2++;
			modbus_hold_registers_transmit(MODBUS_ADRESS,modbus_func,modbus_rx_arg0,modbus_rx_arg1,MODBUS_RTU_PROT);
			}

		if(modbus_func==4)		//������ ������������� ���-�� ���������	������
			{
			modbus_input_registers_transmit(MODBUS_ADRESS,modbus_func,modbus_rx_arg0,modbus_rx_arg1,MODBUS_RTU_PROT);
			//modbus_modbus4f_cnt++;
			plazma_uart1[3]++;
			modbus_plazma1++;
			
			}

		else if(modbus_func==6) 	//������ ��������� ��������
			{
			if(modbus_rx_arg0==1)		//���������� �� HMI
				{
				hmi_cntrl_reg=modbus_rx_arg1;
				hmi_notconnect_cnt=0;
				}
			if(modbus_rx_arg0==2)		//������������ ����� �� HMI
				{
				hmi_avg_reg=modbus_rx_arg1;
				hmi_notconnect_cnt=0;
				}
			if(modbus_rx_arg0==3)		//���������� ����������� ����������� �� HMI
				{
				hmi_unecc_reg=modbus_rx_arg1;
				hmi_notconnect_cnt=0;
				}
			if(modbus_rx_arg0==4)		//���������� ������������ ����� ������ �� HMI
				{
				hmi_izmax_reg=modbus_rx_arg1;
				hmi_notconnect_cnt=0;
				}
			if(modbus_rx_arg0==11)		//��������� �������, ���
				{
				PWR->CR      |= PWR_CR_DBP;
				BKP->DR1=modbus_rx_arg1;
				PWR->CR   &= ~PWR_CR_DBP;
				printf("Reg 11 writed   \r\n");
				}
			if(modbus_rx_arg0==12)		//��������� �������, ����� 
				{
				gran(&modbus_rx_arg1,1,12);
				PWR->CR      |= PWR_CR_DBP;
				BKP->DR2=modbus_rx_arg1;
				PWR->CR   &= ~PWR_CR_DBP;
				printf("Reg 12 writed   \r\n");
				}
			if(modbus_rx_arg0==13)		//��������� �������, ���� 
				{
				short temp_t;
				if(((BKP->DR2)==1)||((BKP->DR2)==3)||((BKP->DR2)==5)||((BKP->DR2)==7)||((BKP->DR2)==8)||((BKP->DR2)==10)||((BKP->DR2)==12))temp_t=31;
				else if(((BKP->DR2)==4)||((BKP->DR2)==6)||((BKP->DR2)==9)||((BKP->DR2)==11))temp_t=30;
				else if(((BKP->DR2)==2)&&((BKP->DR1)%4==0)) temp_t=29;
				else temp_t=28;

				gran(&modbus_rx_arg1,1,temp_t);
				PWR->CR      |= PWR_CR_DBP;
				BKP->DR3=modbus_rx_arg1;
				PWR->CR   &= ~PWR_CR_DBP;
				//LPC_RTC->DOM=(uint16_t)modbus_rx_arg1;
				printf("Reg 13 writed   \r\n");
				}
//#define KOSYAK0
#ifdef KOSYAK0			if(modbus_rx_arg0==1400)		//��������� �������, ���
				{
				long temp_H, temp_M, temp_S, temp_L;
				gran(&modbus_rx_arg1,0,23);
				temp_L=(((long)(RTC->CNTH))<<16)+((long)(RTC->CNTL));
				temp_H=modbus_rx_arg1;			
				temp_M=((temp_L)%3600)/60;		
				temp_S=((temp_L)%3600)%60;		
				temp_L=((temp_H*3600L)+(temp_M*60L)+temp_S);
  				RCC->APB1ENR |= RCC_APB1ENR_PWREN;                            
  				RCC->APB1ENR |= RCC_APB1ENR_BKPEN;
				PWR->CR   |= PWR_CR_DBP;
				RTC->CRL  |=  RTC_CRL_CNF;
				RTC->CNTH=(short)(temp_L>>16);
				RTC->CNTL=(short)temp_L;
				RTC->CRL  &= ~RTC_CRL_CNF;
				while (!((RTC->CRL)&RTC_CRL_RTOFF)){};
				PWR->CR   &= ~PWR_CR_DBP;
				printf("Reg 14 writed   \r\n");
				printf("%d",temp_L);  
				}

			if(modbus_rx_arg0==15)		//��������� �������, ������ 
				{
				long temp_H, temp_M, temp_S, temp_L;
				gran(&modbus_rx_arg1,0,59);
				temp_L=(((long)(RTC->CNTH))<<16)+((long)(RTC->CNTL));
				temp_H=((temp_L)/3600);			
				temp_M=modbus_rx_arg1;		
				temp_S=((temp_L)%3600)%60;		
				temp_L=((temp_H*3600L)+(temp_M*60L)+temp_S);
  				RCC->APB1ENR |= RCC_APB1ENR_PWREN;                            
  				RCC->APB1ENR |= RCC_APB1ENR_BKPEN;
				PWR->CR   |= PWR_CR_DBP;
				RTC->CRL  |=  RTC_CRL_CNF;
				RTC->CNTH=(short)(temp_L>>16);
				RTC->CNTL=(short)temp_L;
				RTC->CRL  &= ~RTC_CRL_CNF;
				while (!((RTC->CRL)&RTC_CRL_RTOFF)){};
				PWR->CR   &= ~PWR_CR_DBP;*/
				printf("Reg 15 writed   \r\n");
				printf("%d",temp_L);  
				}
			if(modbus_rx_arg0==16)		//��������� �������, ������� 
				{
				long temp_H, temp_M, temp_S, temp_L;
				gran(&modbus_rx_arg1,0,59);
				temp_L=(((long)(RTC->CNTH))<<16)+((long)(RTC->CNTL));
				temp_H=((temp_L)/3600);			
				temp_M=((temp_L)%3600)/60;		
				temp_S=modbus_rx_arg1;		
				temp_L=((temp_H*3600L)+(temp_M*60L)+temp_S);
				RCC->APB1ENR |= RCC_APB1ENR_PWREN;                            
  				RCC->APB1ENR |= RCC_APB1ENR_BKPEN;
				PWR->CR   |= PWR_CR_DBP;
				RTC->CRL  |=  RTC_CRL_CNF;
				RTC->CNTH=(short)(temp_L>>16);
				RTC->CNTL=(short)temp_L;
				RTC->CRL  &= ~RTC_CRL_CNF;
				while (!((RTC->CRL)&RTC_CRL_RTOFF)){};
				PWR->CR   &= ~PWR_CR_DBP;
				printf("Reg 16 writed   \r\n");
				printf("%d",temp_L);  

				}  
	#endif
			if(modbus_rx_arg0==20)		//��� ������������ ��� ������ ������������ ����
				{
				if((modbus_rx_arg1>=0)&&(modbus_rx_arg1<=18))
				lc640_write_int(EE_NUMIST,modbus_rx_arg1);  
				}
			if(modbus_rx_arg0==21)		//��� ������������ ��� ������ ������������ ����
				{
				if((modbus_rx_arg1==0)||(modbus_rx_arg1==1))
				lc640_write_int(EE_PAR,modbus_rx_arg1);  
				}
			if(modbus_rx_arg0==22)		//��� ������������ ��� ������ ������������ ����
				{
				if((modbus_rx_arg1==0)||(modbus_rx_arg1==1))
				lc640_write_int(EE_ZV_ON,modbus_rx_arg1);  
				}
			if(modbus_rx_arg0==23)		//��� ������������ ��� ������ ������������ ����
				{
				///*/if((modbus_rx_arg1==0)||(modbus_rx_arg1==1))
				///*/lc640_write_int(EE_TERMOKOMP,modbus_rx_arg1);  
				}
			if(modbus_rx_arg0==24)		//��� ������������ ��� ������ ������������ ����
				{
				///*/if(/*(modbus_rx_arg1>=0)||*/(modbus_rx_arg1<=20))
				///*/lc640_write_int(EE_UBM_AV,modbus_rx_arg1);  
				}


			if(modbus_rx_arg0==30)		//���������� ������������ ��� ������ ������������ ����������
				{
				if((modbus_rx_arg1>0)&&(modbus_rx_arg1<5))modbus_rx_arg1=0;
				else if(modbus_rx_arg1>=60)TBAT=60;
				else TBAT=modbus_rx_arg1;
				lc640_write_int(EE_TBAT,TBAT);

				main_kb_cnt=(TBAT*60)-20;
	     		}
			if(modbus_rx_arg0==31)		//
				{
				 lc640_write_int(EE_UMAX,modbus_rx_arg1);
	     		}
			if(modbus_rx_arg0==32)		//
				{
				 lc640_write_int(EE_DU,UB20-modbus_rx_arg1);
	     		}
			if(modbus_rx_arg0==33)		//
				{
				 lc640_write_int(EE_UDCDC,modbus_rx_arg1);
	     		}
			if(modbus_rx_arg0==34)		//
				{
				 lc640_write_int(EE_UB20,modbus_rx_arg1);
	     		}
			if(modbus_rx_arg0==35)		//
				{
				 lc640_write_int(EE_USIGN,modbus_rx_arg1);
	     		}
			if(modbus_rx_arg0==36)		//
				{
				 lc640_write_int(EE_UMN,modbus_rx_arg1);
	     		}
			if(modbus_rx_arg0==37)		//
				{
				 lc640_write_int(EE_U0B,modbus_rx_arg1);
	     		}
			if(modbus_rx_arg0==38)		//
				{
				 lc640_write_int(EE_IKB,modbus_rx_arg1);
	     		}
			if(modbus_rx_arg0==39)		//
				{
				 lc640_write_int(EE_IZMAX,modbus_rx_arg1);
	     		}
			if(modbus_rx_arg0==40)		//
				{
				 lc640_write_int(EE_IMAX,modbus_rx_arg1);
	     		}
			if(modbus_rx_arg0==41)		//
				{
				 lc640_write_int(EE_IMIN,modbus_rx_arg1);
	     		}
			if(modbus_rx_arg0==42)		//
				{
				 lc640_write_int(EE_UVZ,modbus_rx_arg1);
	     		}
			if(modbus_rx_arg0==43)		//
				{
				if((modbus_rx_arg1>=0)&&(modbus_rx_arg1<=3))lc640_write_int(EE_TZAS,modbus_rx_arg1);
	     		}
			if(modbus_rx_arg0==44)		//
				{
				lc640_write_int(EE_TMAX,modbus_rx_arg1);
	     		}
			if(modbus_rx_arg0==45)		//
				{
				lc640_write_int(EE_TSIGN,modbus_rx_arg1);
	     		}
			if(modbus_rx_arg0==46)		//
				{
				lc640_write_int(EE_TBATMAX,modbus_rx_arg1);
	     		}
			if(modbus_rx_arg0==47)		//
				{
				lc640_write_int(EE_TBATSIGN,modbus_rx_arg1);
	     		}
			if(modbus_rx_arg0==48)		//
				{
				 lc640_write_int(EE_SPEED_CHRG_CURR,modbus_rx_arg1);
	     		}
			if(modbus_rx_arg0==49)		//
				{
				 lc640_write_int(EE_SPEED_CHRG_VOLT,modbus_rx_arg1);
	     		}
			if(modbus_rx_arg0==50)		//
				{
				 lc640_write_int(EE_SPEED_CHRG_TIME,modbus_rx_arg1);
	     		}
			if(modbus_rx_arg0==51)		//
				{
				 lc640_write_int(EE_U_OUT_KONTR_MAX,modbus_rx_arg1);
	     		}
			if(modbus_rx_arg0==52)		//
				{
				 lc640_write_int(EE_U_OUT_KONTR_MIN,modbus_rx_arg1);
	     		}
			if(modbus_rx_arg0==53)		//
				{
				 if((modbus_rx_arg1>=5)&&(modbus_rx_arg1<=100))lc640_write_int(EE_U_OUT_KONTR_DELAY,modbus_rx_arg1);
	     		}
			if(modbus_rx_arg0==54)		//
				{
				 lc640_write_int(EE_UDCDC,modbus_rx_arg1);
				 //lc640_write_int(EE_UB20,modbus_rx_arg1);
	     		}
			if(modbus_rx_arg0==55)		//������������ (���������) ���������� �������� ����, 1�
				{
				lc640_write_int(EE_UMAXN,modbus_rx_arg1);
	     		}
			if(modbus_rx_arg0==70)		//
				{
				gran(&modbus_rx_arg1,1,254);
				lc640_write_int(EE_MODBUS_ADRESS,modbus_rx_arg1);
	     		}
			if(modbus_rx_arg0==71)		//
				{
				if(	(modbus_rx_arg1==240)||(modbus_rx_arg1==480)||
					(modbus_rx_arg1==960)||(modbus_rx_arg1==1920)||
					(modbus_rx_arg1==3840)||(modbus_rx_arg1==5760)||
					(modbus_rx_arg1==11520)||(modbus_rx_arg1==23040)
					) 
					{
					lc640_write_int(EE_MODBUS_BAUDRATE,modbus_rx_arg1);
					stm32_Usart1Setup(modbus_rx_arg1*10UL);
					}
	     		}

			if(modbus_rx_arg0==72)		//���72	 �������������� ���������� �����, ���(1) ����(0)
				{
				gran(&modbus_rx_arg1,0,1);
				lc640_write_int(EE_SPEED_CHRG_AVT_EN,modbus_rx_arg1);
	     		}

			if(modbus_rx_arg0==73)		//���73	 �������������� ���������� ����� ������ (���������� �������� ���������), 1�
				{
				gran(&modbus_rx_arg1,0,100);
				lc640_write_int(EE_SPEED_CHRG_D_U,modbus_rx_arg1);
	     		}

			if(modbus_rx_arg0==74)		//���74	 �������� ���������� ����������� ������, ����(0) ��1(1) ��2(2)
				{
				gran(&modbus_rx_arg1,0,2);
				lc640_write_int(EE_SPEED_CHRG_BLOCK_SRC,modbus_rx_arg1);
	     		}

			if(modbus_rx_arg0==75)		//���75	 ������ ���������� ����������� ������, ����������(0) ��������(1)
				{
				printf("Reg 75 writed   \r\n");
				gran(&modbus_rx_arg1,0,2);
				lc640_write_int(EE_SPEED_CHRG_BLOCK_LOG,modbus_rx_arg1);
	     		}

			if(modbus_rx_arg0==76)		//���76	 ������������ ����������� ������ �����������, ���(1) ����(0) 
				{
				printf("Reg 76 writed   \r\n");
				gran(&modbus_rx_arg1,0,2);
				lc640_write_int(EE_SP_CH_VENT_BLOK,modbus_rx_arg1);
	     		}

			if(modbus_rx_arg0==77)		//���77	 ���������� ��� �������� ����, 1 - (1)  3 - (3)
				{
				printf("Reg 77 writed   \r\n");
				gran(&modbus_rx_arg1,0,1);
				if(modbus_rx_arg1==1)lc640_write_int(EE_NUMPHASE,1);
				else lc640_write_int(EE_NUMPHASE,3);
	     		}

			if(modbus_rx_arg0==78)		//���78	 //��������� ������������ ����1
				{
				printf("Reg 78 writed   \r\n");
				lc640_write_int(EE_RELE1SET,modbus_rx_arg1);
	     		}

			if(modbus_rx_arg0==79)		//���79	 //��������� ������������ ����2
				{
				printf("Reg 79 writed   \r\n");
				lc640_write_int(EE_RELE2SET,modbus_rx_arg1);
	     		}

			if(modbus_rx_arg0==80)		//���80	 //��������� ������������ ����1
				{
				printf("Reg 80 writed   \r\n");
				lc640_write_int(EE_RELE3SET,modbus_rx_arg1);
	     		}

			if(modbus_rx_arg0==81)		//���81	 ������� ������� ��� 20-�� ������� �������
				{
				printf("Reg 81 writed   \r\n");
				lc640_write_int(EE_BAT_C_POINT_20,modbus_rx_arg1);
	     		}

			if(modbus_rx_arg0==82)		//���82	 ������� ������� ��� 10-�� ������� �������
				{
				printf("Reg 82 writed   \r\n");
				lc640_write_int(EE_BAT_C_POINT_10,modbus_rx_arg1);
	     		}

			if(modbus_rx_arg0==83)		//���83	 ������� ������� ��� 5-�� ������� �������
				{
				printf("Reg 83 writed   \r\n");
				lc640_write_int(EE_BAT_C_POINT_5,modbus_rx_arg1);
	     		}

			if(modbus_rx_arg0==84)		//���84	 ������� ������� ��� 3-� ������� �������
				{
				printf("Reg 84 writed   \r\n");
				lc640_write_int(EE_BAT_C_POINT_3,modbus_rx_arg1);
	     		}

			if(modbus_rx_arg0==85)		//���85	 ������� ������� ��� 1-� ������� �������
				{
				printf("Reg 85 writed   \r\n");
				lc640_write_int(EE_BAT_C_POINT_1,modbus_rx_arg1);
	     		}

			if(modbus_rx_arg0==86)		//���86	 ������� ������� ��� 1/2 ������� �������
				{
				printf("Reg 86 writed   \r\n");
				lc640_write_int(EE_BAT_C_POINT_1_2,modbus_rx_arg1);
	     		}

			if(modbus_rx_arg0==87)		//���87	 ������� ������� ��� 1/6 ������� �������
				{
				printf("Reg 87 writed   \r\n");
				lc640_write_int(EE_BAT_C_POINT_1_6,modbus_rx_arg1);
	     		}

			if(modbus_rx_arg0==88)		//���88	 ���������� ������������ ��������� ��� 20-�� ������� �������
				{
				printf("Reg 88 writed   \r\n");
				lc640_write_int(EE_BAT_U_END_20,modbus_rx_arg1);
	     		}

			if(modbus_rx_arg0==89)		//���89	 ���������� ������������ ��������� ��� 10-�� ������� �������
				{
				printf("Reg 89 writed   \r\n");
				lc640_write_int(EE_BAT_U_END_10,modbus_rx_arg1);
	     		}

			if(modbus_rx_arg0==90)		//���90	 ���������� ������������ ��������� ��� 5-�� ������� �������
				{
				printf("Reg 90 writed   \r\n");
				lc640_write_int(EE_BAT_U_END_5,modbus_rx_arg1);
	     		}

			if(modbus_rx_arg0==91)		//���91	 ���������� ������������ ��������� ��� 3-� ������� �������
				{
				printf("Reg 91 writed   \r\n");
				lc640_write_int(EE_BAT_U_END_3,modbus_rx_arg1);
	     		}

			if(modbus_rx_arg0==92)		//���92	 ���������� ������������ ��������� ��� 1-� ������� �������
				{
				printf("Reg 92 writed   \r\n");
				lc640_write_int(EE_BAT_U_END_1,modbus_rx_arg1);
	     		}

			if(modbus_rx_arg0==93)		//���93	 ���������� ������������ ��������� ��� 1/2 ������� �������
				{
				printf("Reg 93 writed   \r\n");
				lc640_write_int(EE_BAT_U_END_1_2,modbus_rx_arg1);
	     		}

			if(modbus_rx_arg0==94)		//���94	 ���������� ������������ ��������� ��� 1/6 ������� �������
				{
				printf("Reg 94 writed   \r\n");
				lc640_write_int(EE_BAT_U_END_1_6,modbus_rx_arg1);
	     		}

 			if(modbus_rx_arg0==95)		//���95	 ���������� ������������� ��������� � �������
				{
				printf("Reg 95 writed   \r\n");
				lc640_write_int(EE_BAT_C_POINT_NUM_ELEM,modbus_rx_arg1);
	     		}

			if(modbus_rx_arg0==96)		//���96	 ����������� �������� �������
				{
				printf("Reg 96 writed   \r\n");
				lc640_write_int(EE_BAT_K_OLD,modbus_rx_arg1);
	     		}

			if(modbus_rx_arg0==97)		//���97	 ����������� �����, ���������� ���� 1, 0.1� 
				{
				printf("Reg 97 writed   \r\n");
				lc640_write_int(EE_FZ_U1,modbus_rx_arg1);
	     		}
			if(modbus_rx_arg0==98)		//���98	 ����������� �����, ������������ ��� ���� 1, 0.1�
				{
				printf("Reg 98 writed   \r\n");
				lc640_write_int(EE_FZ_IMAX1,modbus_rx_arg1);
	     		}
			if(modbus_rx_arg0==99)		//���99	 ����������� �����, ������������ ���� 1, 1�
				{
				printf("Reg 99 writed   \r\n");
				lc640_write_int(EE_FZ_T1,modbus_rx_arg1);
	     		}
			if(modbus_rx_arg0==100)		//���100  ����������� �����, ��� ������������ � ���� 1 �� ���� 2, 0.1�
				{
				printf("Reg 100 writed   \r\n");
				lc640_write_int(EE_FZ_ISW12,modbus_rx_arg1);
	     		}
			if(modbus_rx_arg0==101)		//���101  ����������� �����, ���������� ���� 2, 0.1�
				{
				printf("Reg 101 writed   \r\n");
				lc640_write_int(EE_FZ_U2,modbus_rx_arg1);
	     		}
			if(modbus_rx_arg0==102)		//���102  ����������� �����, ������������ ��� ���� 2, 0.1�
				{
				printf("Reg 102 writed   \r\n");
				lc640_write_int(EE_FZ_IMAX2,modbus_rx_arg1);
	     		}
			if(modbus_rx_arg0==103)		//���103  ����������� �����, ������������ ���� 2, 1� 
				{
				printf("Reg 103 writed   \r\n");
				lc640_write_int(EE_FZ_T2,modbus_rx_arg1);
	     		}

			if(modbus_rx_arg0==940)		// ������� �������� �������� ������� �������
				{
				main_kb_cnt=modbus_rx_arg1;
				
				}
				
			if(modbus_rx_arg0==968)		// ����� ������ ����
				{
				can1_out(modbus_rx_arg1,modbus_rx_arg1,CMND,ALRM_RES,0,0,0,0);
				}

			if(modbus_rx_arg0==969)		// ���������� ��������
				{
				log_cmd_mb=modbus_rx_arg1;
				log_hndl();
				}				
															
			if(modbus_rx_arg0==970)		// ���������� ����1 � �����
				{
				test_hndl_rele1_cntrl=modbus_rx_arg1;
				if(modbus_rx_arg1) test_hndl_rele1_cnt=100;
				}
			if(modbus_rx_arg0==971)		// ���������� ����2 � �����
				{
				test_hndl_rele2_cntrl=modbus_rx_arg1;
				if(modbus_rx_arg1) test_hndl_rele2_cnt=100;
				}
			if(modbus_rx_arg0==972)		// ���������� ����3 � �����
				{
				test_hndl_rele3_cntrl=modbus_rx_arg1;
				if(modbus_rx_arg1) test_hndl_rele3_cnt=100;
				}
			if(modbus_rx_arg0==973)		// ���������� ����HV � �����
				{
				test_hndl_releHV_cntrl=modbus_rx_arg1;
				if(modbus_rx_arg1) test_hndl_releHV_cnt=100;
				}
			if(modbus_rx_arg0==974)		// ���������� ������ � �����, ����� ��� (0-�� ��������� �������,1-3 - ����� ������������ ���, 255 - ��������� �����)
				{
				test_hndl_bps_number=modbus_rx_arg1;
				if(modbus_rx_arg1==255) 
					{
					test_hndl_bps_cnt=600;
					test_hndl_bps_state=3;
					}
				else if(modbus_rx_arg1==0) 
					{
					test_hndl_bps_cnt=0;
					test_hndl_bps_state=0;
					}
				else
					{
					test_hndl_bps_cnt=600;
					test_hndl_bps_state=0;
					}
				}
			if(modbus_rx_arg0==975)		// ���������� ������ � �����, 0 - ���������, 1 - �������� ������������ ���, 2 - �������� ����������� ���, 3 - �������� ���������������, 4 - �������� ���������)
				{
				test_hndl_bps_cnt=600;
				test_hndl_bps_state=modbus_rx_arg1;
				}
			if(modbus_rx_arg0==976)		// ���� ������, 0 - ���������, 1 - ��������)
				{
				if(modbus_rx_arg1) zv_test_cnt=20;
				else zv_test_cnt=0;
				}
			if(modbus_rx_arg0==977)		// ���� ����������� 0 - ��������, 1 - �������(U�������), 2 - ������(�������), 3 - �������(������), 4 - �������(���), 5 - ��� ��������� 
				{
				if(modbus_rx_arg1) test_led_cnt=50;
				test_led_stat=modbus_rx_arg1;
				}

			if(modbus_rx_arg0==982)		// ���������-���������� ����������� ������
				{
				speedChargeStartStop();
				}

			if(modbus_rx_arg0==994)		//
				{
/*				  RCC->APB1ENR |= RCC_APB1ENR_PWREN;                            // enable clock for Power interface
  RCC->APB1ENR |= RCC_APB1ENR_BKPEN;
  PWR->CR      |= PWR_CR_DBP;                                   // enable access to RTC, BDC registers


 
//  RTC->CRL   &= ~(1<<3);                                        // reset Registers Synchronized Flag
//  while ((RTC->CRL & (1<<3)) == 0);                             // wait until registers are synchronized

  RTC->CRL  |=  RTC_CRL_CNF;                                    // set configuration mode
  RTC->PRLH  = ((__RTC_PERIOD*__RTCCLK/1000-1)>>16) & 0x00FF;   // set prescaler load register high
  RTC->PRLL  = ((__RTC_PERIOD*__RTCCLK/1000-1)    ) & 0xFFFF;   // set prescaler load register low
//RTC->PRLL  = 5;
	//BKP->DR5=5;
  RTC->CNTH  = ((__RTC_CNT)>>16) & 0xFFFF;                      // set counter high
  RTC->CNTL  = ((__RTC_CNT)    ) & 0xFFFF;                      // set counter low
  RTC->ALRH  = ((__RTC_ALR)>>16) & 0xFFFF;                      // set alarm high
  RTC->ALRL  = ((__RTC_ALR)    ) & 0xFFFF;                      // set alarm low
  if (__RTC_INTERRUPTS) {                                       // RTC interrupts used
    RTC->CRH = __RTC_CRH;                                       // enable RTC interrupts
    NVIC->ISER[0]  = (1 << (RTC_IRQChannel & 0x1F));            // enable interrupt
  }
  RTC->CRL  &= ~RTC_CRL_CNF;                                    // reset configuration mode
  while ((RTC->CRL & RTC_CRL_RTOFF) == 0);                      // wait until write is finished

  PWR->CR   &= ~PWR_CR_DBP; */    
	     		}
			if(modbus_rx_arg0==995)		//
				{
				modbus_register_995=modbus_rx_arg1;
	     		}

			if(modbus_rx_arg0==996)		//
				{
				if(modbus_rx_arg1==1)
					{
					printf("Zero current for IPS#%d executed   \r\n", modbus_register_998);
					can1_out(modbus_register_998-1,modbus_register_998-1,KLBR,(2*16)+1,(2*16)+1,0,0,0);
					//modbus_register_998=modbus_rx_arg1;
					//if(modbus_register_998)can1_out(modbus_register_998-1,modbus_register_998-1,CMND,ALRM_RES,0,0,0,0);
					}
	     		}


			if(modbus_rx_arg0==997)		//
				{
				if(modbus_rx_arg1==1)
					{
					can1_out(modbus_register_998-1,modbus_register_998-1,KLBR,(2*16)+1,(2*16)+1,0,0,0);
					//modbus_register_998=modbus_rx_arg1;
					//if(modbus_register_998)can1_out(modbus_register_998-1,modbus_register_998-1,CMND,ALRM_RES,0,0,0,0);
					}
	     		}

			if(modbus_rx_arg0==998)		//������ ������ ������������ ��������� 
				{
				if((modbus_rx_arg1>=0)&&(modbus_rx_arg1<=NUMIST))
					{
					modbus_register_998=modbus_rx_arg1;
					if(modbus_register_998)can1_out(modbus_register_998-1,modbus_register_998-1,CMND,ALRM_RES,0,0,0,0);
					}
	     		}

			if(modbus_rx_arg0==999)		//
				{
				//if(modbus_rx_arg1!=0)
					{
					modbus_register_999=modbus_rx_arg1;
					gran(&modbus_register_999,0,1023);
					}
	     		}

			if(modbus_rx_arg0==1000)		//
				{
				//if(modbus_rx_arg1==1)
					{
					modbus_register_1000=modbus_rx_arg1;
					}
	     		}
			if(modbus_rx_arg0==1001)		//
				{
				short tempS;
				long tempL;
				
				tempL=(long)modbus_rx_arg1;
				tempL*=500L;
				tempL/=(signed long)adc_buff_[5];
				Kuout=(signed short)tempL;
				lc640_write_int(EE_KUOUT,Kuout);

	     		}
			if(modbus_rx_arg0==1002)		//
				{
					{
					modbus_register_1002=modbus_rx_arg1;
					}

	     		}
			if(modbus_rx_arg0==1003)		//
				{

	     		}
			if(modbus_rx_arg0==1005)		//
				{
				short tempS;
				long tempL;
				
				tempL=(long)modbus_rx_arg1;
				tempL*=6000L;
				tempL/=(signed long)adc_buff_[0];
				KunetA=(signed short)tempL;
				lc640_write_int(EE_KUNETA,KunetA);
	     		}

			if(modbus_rx_arg0==1006)		//
				{
				short tempS;
				long tempL;
				
				printf("Reg 1006 writed   \r\n");

				tempL=(long)modbus_rx_arg1;
				tempL*=6000L;
				tempL/=(signed long)adc_buff_[1];
				KunetB=(signed short)tempL;
				lc640_write_int(EE_KUNETB,KunetB);
	     		}

			if(modbus_rx_arg0==1007)		//
				{
				short tempS;
				long tempL;
				
				printf("Reg 1007 writed   \r\n");

				tempL=(long)modbus_rx_arg1;
				tempL*=6000L;
				tempL/=(signed long)adc_buff_[2];
				KunetC=(signed short)tempL;
				lc640_write_int(EE_KUNETC,KunetC);
	     		}
 			if(modbus_rx_arg0==1008)		//
				{
				short tempS;
				long tempL;
				
				printf("Reg 1008 writed   \r\n");

				tempL=(long)modbus_rx_arg1;
				tempL*=6000L;
				tempL/=(signed long)adc_buff_[2];
				KunetC=(signed short)tempL;
				lc640_write_int(EE_KUNETC,KunetC);
	     		}
			if(modbus_rx_arg0==1009)		//
				{
				printf("Reg 1009 writed   \r\n");
					{
					short tempS;
					signed long tempL,tempL1;
				/*	tempL=(long)modbus_rx_arg1;
					tempL*=1000L;
					Kuout=(short)(tempL/(long)adc_buff_virt_0);	*/

					tempL=(signed long)modbus_rx_arg1;
					//if(modbus_rx_arg1<0)tempL=(signed long)(-modbus_rx_arg1);
					tempL*=2000L;
					//tempL/=((signed long)ibat_metr_buff_[0]-(signed long)ibat_metr_buff_[1]);
					tempL1=(signed long)(ibat_metr_buff_[0]-ibat_metr_buff_[1]);
					//tempL1=-500L;
					tempL/=tempL1;
					
					Kibat1[0]=(signed short)tempL;
					//if(modbus_rx_arg1<0)Kibat1[0]=(signed short)(-tempL);
					lc640_write_int(EE_KI1BAT1,Kibat1[0]);
					}
	     		}

			if(modbus_rx_arg0==1010)		//
				{
				printf("Reg 1010 writed   \r\n");
					{
					short tempS;
					long tempL;
				/*	tempL=(long)modbus_rx_arg1;
					tempL*=1000L;
					Kuout=(short)(tempL/(long)adc_buff_virt_0);	*/

					tempL=(long)modbus_rx_arg1;
					tempL+=273;
					tempL*=20000L;
					tempL/=(signed long)adc_buff_[3];
					Ktbat[0]=(signed short)tempL;
					lc640_write_int(EE_KTBAT1,Ktbat[0]);
					}
	     		}

			if(modbus_rx_arg0==1011)		//
				{
				printf("Reg 1011 writed   \r\n");

	     		}
						
			if(modbus_rx_arg0==1022)		//
				{
				printf("Reg 1022 writed   \r\n");

				if(modbus_rx_arg1)
					{
					avt_klbr_mode_ui=1;
					avt_klbr_num_ui=modbus_register_998;
					avt_klbr_phase_ui=1;
					avt_klbr_necc_value_ui=modbus_rx_arg1;
					avt_klbr_err_cnt_ui=300;
					modbus_register_1022=modbus_rx_arg1;
					modbus_register_offset_ui=22;
					avt_klbr_start_ui();
					}
				else
					{
					avt_klbr_mode_ui=0;
					//avt_klbr_num=0;
					//avt_klbr_phase=0;
					modbus_register_1022=0;
					} 
	     		}
			if(modbus_rx_arg0==1023)		//
				{
				printf("Reg 1023 writed   \r\n");

				if(modbus_rx_arg1)
					{
					avt_klbr_mode_un=1;
					avt_klbr_num_un=modbus_register_998;
					avt_klbr_phase_un=1;
					avt_klbr_necc_value_un=modbus_rx_arg1;
					avt_klbr_err_cnt_un=300;
					modbus_register_1023=modbus_rx_arg1;
					modbus_register_offset_un=23;
					avt_klbr_start_un();
					}
				else
					{
					avt_klbr_mode_un=0;
					//avt_klbr_num=0;
					//avt_klbr_phase=0;
					modbus_register_1023=0;
					} 
	     		}
			if(modbus_rx_arg0==1024)		//
				{
				printf("Reg 1024 writed   \r\n");

				if(modbus_rx_arg1)
					{
					avt_klbr_mode_i=1;
					avt_klbr_num_i=modbus_register_998;
					avt_klbr_phase_i=1;
					avt_klbr_necc_value_i=modbus_rx_arg1;
					avt_klbr_err_cnt_i=300;
					modbus_register_1024=modbus_rx_arg1;
					modbus_register_offset_i=24;
					avt_klbr_start_i();
					}
				else
					{
					avt_klbr_mode_i=0;
					//avt_klbr_num=0;
					//avt_klbr_phase=0;
					modbus_register_1024=0;
					} 
	     		}
			if(modbus_rx_arg0==1025)		//
				{
				printf("Reg 1025 writed   \r\n");

				if(modbus_rx_arg1)
					{
					avt_klbr_mode_t=1;
					avt_klbr_num_t=modbus_register_998;
					avt_klbr_phase_t=1;
					avt_klbr_necc_value_t=modbus_rx_arg1;
					avt_klbr_err_cnt_t=300;
					modbus_register_1025=modbus_rx_arg1;
					modbus_register_offset_t=25;
					avt_klbr_start_t();
					}
				else
					{
					avt_klbr_mode_t=0;
					//avt_klbr_num=0;
					//avt_klbr_phase=0;
					modbus_register_1025=0;
					} 
	     		}
			if(modbus_rx_arg0==1027)		//
				{
				printf("Reg 1027 writed   \r\n");

				if(modbus_rx_arg1)
					{
					avt_klbr_mode=akmUI;
					avt_klbr_num=2;
					avt_klbr_phase=1;
					avt_klbr_necc_value=modbus_rx_arg1;
					avt_klbr_err_cnt=300;
					modbus_register_1027=modbus_rx_arg1;
					modbus_register_offset=27;
					}
				else
					{
					avt_klbr_mode=akmOFF;
					//avt_klbr_num=0;
					//avt_klbr_phase=0;
					modbus_register_1027=0;
					} 
	     		}

			if(modbus_rx_arg0==1070)		//
				{
				short tempS;
				long tempL;
				
				tempL=(long)modbus_rx_arg1;
				tempL*=500L;
				tempL/=(signed long)adc_buff_[4];
				Kubps=(signed short)tempL;
				lc640_write_int(EE_KUBPS,Kubps);

	     		}
			if(modbus_rx_arg0==19)		//���/���� ��������� ����.
				{
	/*			if(modbus_rx_arg1==1)
					{
					if(work_stat!=wsPS)
						{
						work_stat=wsPS;
						time_proc=0;
						time_proc_remain=T_PROC_PS;
						restart_on_PS();
						lc640_write_int(EE_MAIN_MENU_MODE,mmmIN);
						}
					}
				if(modbus_rx_arg1==0)
					{
					if(work_stat==wsPS)
						{
						work_stat=wsOFF;
						restart_off();
						}
					} */
				}
			if(modbus_rx_arg0==20)		//���/���� ��������� ����
				{
/*				if(modbus_rx_arg1==1)
					{
					if(work_stat!=wsGS)
						{
						work_stat=wsGS;
						time_proc=0;
						time_proc_remain=T_PROC_GS;
						lc640_write_int(EE_MAIN_MENU_MODE,mmmIT);
						}
					}
				if(modbus_rx_arg1==0)
					{
					if(work_stat==wsGS)
						{
						work_stat=wsOFF;
						restart_off();
						}
					}*/
				}
			//modbus_hold_register_transmit(MODBUS_ADRESS,modbus_func,modbus_rx_arg0);

			if(modbus_rx_arg0==100)		//�������� ���� ��� ���������� �� �������� ���
				{
				//plazma1000[2]=modbus_rx_arg1;
				if(modbus_rx_arg1&0x4000)
					{
					short tempSSSS;
					
					tempSSSS=modbus_rx_arg1&0x3fff;
					//plazma1000[3]=tempSSSS;
					if((tempSSSS>0)&&(tempSSSS<5))tempSSSS=0;
					else if(tempSSSS>=60)tempSSSS=60;
				//	else tempSSSS=modbus_rx_arg1;
				///*/	if(TBAT!=tempSSSS)lc640_write_int(EE_TBAT,tempSSSS);

				///*/	main_kb_cnt=(tempSSSS*60)-20;
					}
				///*/else ica_cntrl_hndl=modbus_rx_arg1;
				///*/ica_cntrl_hndl_cnt=200;

				//plazma1000[1]++;
				}
			if(modbus_rx_arg0==101)		//�������� ���� � ������� ���� (����������� �������� � ������� � ���������� ��������)
				{
				///*/ica_your_current==modbus_rx_arg1;
				///*/ica_cntrl_hndl_cnt=200;
				//plazma1000[2]++;
				}
			if(modbus_rx_arg0==300)		//���������� ���������� ���������� �������
				{
				kalibrate_func(modbus_rx_arg1);
				}

			//printf("Register %d writed, parametr %d   \r\n",modbus_rx_arg0,modbus_rx_arg1);

			memcpy(modbus_tx_buff,modbus_rx_buffer,8);
	
			for (i=0;i<(8);i++)
				{
				putchar1(modbus_tx_buff[i]);
				}
			}
		} 
	else if(modbus_an_buffer[0]==ICA_MODBUS_ADDRESS)
		{
//		ica_plazma[3]++;
		if(modbus_func==4)		//������ ������������� ���-�� ���������	������
			{
//			ica_plazma[2]++;
			if(modbus_an_buffer[2]==2)
				{
				///*/ica_your_current=(((unsigned short)modbus_an_buffer[3])*((unsigned short)256))+((unsigned short)modbus_an_buffer[4]);
				}
			}
		}
	
	}


}
/*
//-----------------------------------------------
void modbus_register_transmit(unsigned char adr,unsigned char func,unsigned short reg_adr)
{
char modbus_registers[150];
//char modbus_tx_buff[50];
unsigned short crc_temp;
char i;


modbus_registers[0]=(char)(load_U/256);					//���1
modbus_registers[1]=(char)(load_U%256);
modbus_registers[2]=(char)(load_I/256);					//���2
modbus_registers[3]=(char)(load_I%256);
modbus_registers[4]=(char)(Ib_ips_termokompensat/256);		//���3
modbus_registers[5]=(char)(Ib_ips_termokompensat%256);
modbus_registers[6]=(char)(t_ext[0]/256);				//���4
modbus_registers[7]=(char)(t_ext[0]%256);
modbus_registers[8]=(char)(net_Ua/256);					//���5
modbus_registers[9]=(char)(net_Ua%256);		 	
modbus_registers[10]=(char)(net_Ub/256);				//���6
modbus_registers[11]=(char)(net_Ub%256);
modbus_registers[12]=(char)(net_Uc/256);				//���7
modbus_registers[13]=(char)(net_Uc%256);
modbus_registers[14]=(char)(net_F3/256);				//���8
modbus_registers[15]=(char)(net_F3%256);
modbus_registers[16]=(char)(load_I/256);				//���9
modbus_registers[17]=(char)(load_I%256);
modbus_registers[18]=(char)(load_I/256);				//���10
modbus_registers[19]=(char)(load_I%256);
modbus_registers[20]=(char)((LPC_RTC->YEAR)/256);			//���11
modbus_registers[21]=(char)((LPC_RTC->YEAR)%256);
modbus_registers[22]=(char)((LPC_RTC->MONTH)/256);		//���12
modbus_registers[23]=(char)((LPC_RTC->MONTH)%256);
modbus_registers[24]=(char)((LPC_RTC->DOM)/256);			//���13
modbus_registers[25]=(char)((LPC_RTC->DOM)%256);
modbus_registers[26]=(char)((LPC_RTC->HOUR)/256);			//���14
modbus_registers[27]=(char)((LPC_RTC->HOUR)%256);
modbus_registers[28]=(char)((LPC_RTC->MIN)/256);			//���15
modbus_registers[29]=(char)((LPC_RTC->MIN)%256);
modbus_registers[30]=(char)((LPC_RTC->SEC)/256);			//���16
modbus_registers[31]=(char)((LPC_RTC->SEC)%256);
modbus_registers[32]=(char)(load_I/256);				//���17
modbus_registers[33]=(char)(load_I%256);
modbus_registers[34]=(char)(load_I/256);				//���18
modbus_registers[35]=(char)(load_I%256);
modbus_registers[36]=(char)(load_I/256);		//���19
modbus_registers[37]=(char)(load_I%256);
modbus_registers[38]=(char)(NUMIST/256);		//���20
modbus_registers[39]=(char)(NUMIST%256);
modbus_registers[40]=(char)(PAR/256);		//���21
modbus_registers[41]=(char)(PAR%256);
modbus_registers[42]=(char)(ZV_ON/256);		//���22
modbus_registers[43]=(char)(ZV_ON%256);
modbus_registers[44]=(char)(TERMOKOMPENS/256);		//���23
modbus_registers[45]=(char)(TERMOKOMPENS%256);
modbus_registers[46]=(char)(UBM_AV/256);		//���24
modbus_registers[47]=(char)(UBM_AV%256);
modbus_registers[48]=(char)(load_I/256);		//���25
modbus_registers[49]=(char)(load_I%256);
modbus_registers[50]=(char)(load_I/256);		//���26
modbus_registers[51]=(char)(load_I%256);
modbus_registers[52]=(char)(load_I/256);		//���27
modbus_registers[53]=(char)(load_I%256);
modbus_registers[54]=(char)(load_I/256);		//���28
modbus_registers[55]=(char)(load_I%256);
modbus_registers[56]=(char)(load_I/256);		//���29
modbus_registers[57]=(char)(load_I%256);
modbus_registers[58]=(char)(TBAT/256);			//���30
modbus_registers[59]=(char)(TBAT%256);
modbus_registers[60]=(char)(UMAX/256);			//���31
modbus_registers[61]=(char)(UMAX%256);
modbus_registers[62]=(char)((UB20-DU)/256);		//���32
modbus_registers[63]=(char)((UB20-DU)%256);
modbus_registers[64]=(char)(UB0/256);			//���33
modbus_registers[65]=(char)(UB0%256);
modbus_registers[66]=(char)(UB20/256);			//���34
modbus_registers[67]=(char)(UB20%256);
modbus_registers[68]=(char)(USIGN/256);		//���35
modbus_registers[69]=(char)(USIGN%256);
modbus_registers[70]=(char)(UMN/256);		//���36
modbus_registers[71]=(char)(UMN%256);
modbus_registers[72]=(char)(U0B/256);		//���37
modbus_registers[73]=(char)(U0B%256);
modbus_registers[74]=(char)(IKB/256);		//���38
modbus_registers[75]=(char)(IKB%256);
modbus_registers[76]=(char)(IZMAX/256);		//���39
modbus_registers[77]=(char)(IZMAX%256);
modbus_registers[78]=(char)(IMAX/256);		//���40
modbus_registers[79]=(char)(IMAX%256);
modbus_registers[80]=(char)(IMIN/256);		//���41
modbus_registers[81]=(char)(IMIN%256);
modbus_registers[82]=(char)(UVZ/256);		//���42
modbus_registers[83]=(char)(UVZ%256);
modbus_registers[84]=(char)(TZAS/256);		//���43
modbus_registers[85]=(char)(TZAS%256);
modbus_registers[86]=(char)(TMAX/256);		//���44
modbus_registers[87]=(char)(TMAX%256);
modbus_registers[88]=(char)(TSIGN/256);		//���45
modbus_registers[89]=(char)(TSIGN%256);
modbus_registers[90]=(char)(TBATMAX/256);		//���46
modbus_registers[91]=(char)(TBATMAX%256);
modbus_registers[92]=(char)(TBATSIGN/256);		//���47
modbus_registers[93]=(char)(TBATSIGN%256);
modbus_registers[94]=(char)(load_I/256);		//���48
modbus_registers[95]=(char)(load_I%256);
modbus_registers[96]=(char)(load_I/256);		//���49
modbus_registers[97]=(char)(load_I%256);
modbus_registers[98]=(char)(load_I/256);		//���50
modbus_registers[99]=(char)(load_I%256);

modbus_tx_buff[0]=adr;
modbus_tx_buff[1]=func;
modbus_tx_buff[2]=(char)(reg_adr/256);
modbus_tx_buff[3]=(char)(reg_adr%256);
//modbus_tx_buff[4]=(char)(reg_quantity/256);
//modbus_tx_buff[5]=(char)(reg_quantity%256);


mem_copy((char*)&modbus_tx_buff[4],(char*)&modbus_registers[(reg_adr-1)*2],2);

crc_temp=CRC16_2(modbus_tx_buff,6);

modbus_tx_buff[6]=crc_temp%256;
modbus_tx_buff[7]=crc_temp/256;

for (i=0;i<8;i++)
	{
	putchar0(modbus_tx_buff[i]);
	}
for (i=0;i<8;i++)
	{
	putchar_sc16is700(modbus_tx_buff[i]);
	}
}*/
/*//-----------------------------------------------
void modbus_registers_transmit(unsigned char adr,unsigned char func,unsigned short reg_adr,unsigned short reg_quantity)
{
char modbus_registers[100];
//char modbus_tx_buff[100];
unsigned short crc_temp;
char i;


modbus_registers[0]=(char)(load_U/256);					//���1
modbus_registers[1]=(char)(load_U%256);
modbus_registers[2]=(char)(load_I/256);					//���2
modbus_registers[3]=(char)(load_I%256);
modbus_registers[4]=(char)(Ib_ips_termokompensat/256);		//���3
modbus_registers[5]=(char)(Ib_ips_termokompensat%256);
modbus_registers[6]=(char)(t_ext[0]/256);				//���4
modbus_registers[7]=(char)(t_ext[0]%256);
modbus_registers[8]=(char)(net_Ua/256);					//���5
modbus_registers[9]=(char)(net_Ua%256);		 	
modbus_registers[10]=(char)(net_Ub/256);				//���6
modbus_registers[11]=(char)(net_Ub%256);
modbus_registers[12]=(char)(net_Uc/256);				//���7
modbus_registers[13]=(char)(net_Uc%256);
modbus_registers[14]=(char)(net_F3/256);				//���8
modbus_registers[15]=(char)(net_F3%256);
modbus_registers[16]=(char)(load_I/256);				//���9
modbus_registers[17]=(char)(load_I%256);
modbus_registers[18]=(char)(load_I/256);				//���10
modbus_registers[19]=(char)(load_I%256);
modbus_registers[20]=(char)((LPC_RTC->YEAR)/256);			//���11
modbus_registers[21]=(char)((LPC_RTC->YEAR)%256);
modbus_registers[22]=(char)((LPC_RTC->MONTH)/256);		//���12
modbus_registers[23]=(char)((LPC_RTC->MONTH)%256);
modbus_registers[24]=(char)((LPC_RTC->DOM)/256);			//���13
modbus_registers[25]=(char)((LPC_RTC->DOM)%256);
modbus_registers[26]=(char)((LPC_RTC->HOUR)/256);			//���14
modbus_registers[27]=(char)((LPC_RTC->HOUR)%256);
modbus_registers[28]=(char)((LPC_RTC->MIN)/256);			//���15
modbus_registers[29]=(char)((LPC_RTC->MIN)%256);
modbus_registers[30]=(char)((LPC_RTC->SEC)/256);			//���16
modbus_registers[31]=(char)((LPC_RTC->SEC)%256);
modbus_registers[32]=(char)(load_I/256);				//���17
modbus_registers[33]=(char)(load_I%256);
modbus_registers[34]=(char)(load_I/256);				//���18
modbus_registers[35]=(char)(load_I%256);
modbus_registers[36]=(char)(load_I/256);		//���19
modbus_registers[37]=(char)(load_I%256);
modbus_registers[38]=(char)(NUMIST/256);		//���20
modbus_registers[39]=(char)(NUMIST%256);
modbus_registers[40]=(char)(PAR/256);		//���21
modbus_registers[41]=(char)(PAR%256);
modbus_registers[42]=(char)(ZV_ON/256);		//���22
modbus_registers[43]=(char)(ZV_ON%256);
modbus_registers[44]=(char)(TERMOKOMPENS/256);		//���23
modbus_registers[45]=(char)(TERMOKOMPENS%256);
modbus_registers[46]=(char)(UBM_AV/256);		//���24
modbus_registers[47]=(char)(UBM_AV%256);
modbus_registers[48]=(char)(load_I/256);		//���25
modbus_registers[49]=(char)(load_I%256);
modbus_registers[50]=(char)(load_I/256);		//���26
modbus_registers[51]=(char)(load_I%256);
modbus_registers[52]=(char)(load_I/256);		//���27
modbus_registers[53]=(char)(load_I%256);
modbus_registers[54]=(char)(load_I/256);		//���28
modbus_registers[55]=(char)(load_I%256);
modbus_registers[56]=(char)(load_I/256);		//���29
modbus_registers[57]=(char)(load_I%256);
modbus_registers[58]=(char)(TBAT/256);			//���30
modbus_registers[59]=(char)(TBAT%256);
modbus_registers[60]=(char)(UMAX/256);			//���31
modbus_registers[61]=(char)(UMAX%256);
modbus_registers[62]=(char)((UB20-DU)/256);		//���32
modbus_registers[63]=(char)((UB20-DU)%256);
modbus_registers[64]=(char)(UB0/256);			//���33
modbus_registers[65]=(char)(UB0%256);
modbus_registers[66]=(char)(UB20/256);			//���34
modbus_registers[67]=(char)(UB20%256);
modbus_registers[68]=(char)(USIGN/256);		//���35
modbus_registers[69]=(char)(USIGN%256);
modbus_registers[70]=(char)(UMN/256);		//���36
modbus_registers[71]=(char)(UMN%256);
modbus_registers[72]=(char)(U0B/256);		//���37
modbus_registers[73]=(char)(U0B%256);
modbus_registers[74]=(char)(IKB/256);		//���38
modbus_registers[75]=(char)(IKB%256);
modbus_registers[76]=(char)(IZMAX/256);		//���39
modbus_registers[77]=(char)(IZMAX%256);
modbus_registers[78]=(char)(IMAX/256);		//���40
modbus_registers[79]=(char)(IMAX%256);
modbus_registers[80]=(char)(IMIN/256);		//���41
modbus_registers[81]=(char)(IMIN%256);
modbus_registers[82]=(char)(UVZ/256);		//���42
modbus_registers[83]=(char)(UVZ%256);
modbus_registers[84]=(char)(TZAS/256);		//���43
modbus_registers[85]=(char)(TZAS%256);
modbus_registers[86]=(char)(TMAX/256);		//���44
modbus_registers[87]=(char)(TMAX%256);
modbus_registers[88]=(char)(TSIGN/256);		//���45
modbus_registers[89]=(char)(TSIGN%256);
modbus_registers[90]=(char)(TBATMAX/256);		//���46
modbus_registers[91]=(char)(TBATMAX%256);
modbus_registers[92]=(char)(TBATSIGN/256);		//���47
modbus_registers[93]=(char)(TBATSIGN%256);
modbus_registers[94]=(char)(load_I/256);		//���48
modbus_registers[95]=(char)(load_I%256);
modbus_registers[96]=(char)(load_I/256);		//���49
modbus_registers[97]=(char)(load_I%256);
modbus_registers[98]=(char)(load_I/256);		//���50
modbus_registers[99]=(char)(load_I%256);




modbus_tx_buff[0]=adr;
modbus_tx_buff[1]=func;
//modbus_tx_buff[2]=(char)(reg_adr/256);
//modbus_tx_buff[3]=(char)(reg_adr%256);
modbus_tx_buff[2]=(char)(reg_quantity*2);
//modbus_tx_buff[5]=(char)(reg_quantity%256);


mem_copy((char*)&modbus_tx_buff[3],(char*)&modbus_registers[(reg_adr-1)*2],reg_quantity*2);

crc_temp=CRC16_2(modbus_tx_buff,(reg_quantity*2)+3);

modbus_tx_buff[3+(reg_quantity*2)]=crc_temp%256;
modbus_tx_buff[4+(reg_quantity*2)]=crc_temp/256;

//int2lcdyx(reg_quantity,0,10,0);

for (i=0;i<15;i++)
	{
	putchar0(modbus_tx_buff[i]);
	} 
for (i=0;i<15;i++)
	{
	putchar_sc16is700(modbus_tx_buff[i]);
	}
}*/
/*
//-----------------------------------------------
void modbus_hold_register_transmit(unsigned char adr,unsigned char func,unsigned short reg_adr)
{
char modbus_registers[150];
//char modbus_tx_buff[150];
unsigned short crc_temp;
char i;

modbus_registers[20]=(char)((LPC_RTC->YEAR)/256);			//���11  �����, ���
modbus_registers[21]=(char)((LPC_RTC->YEAR)%256);
modbus_registers[22]=(char)((LPC_RTC->MONTH)/256);		//���12  �����, �����
modbus_registers[23]=(char)((LPC_RTC->MONTH)%256);
modbus_registers[24]=(char)((LPC_RTC->DOM)/256);			//���13  �����, ���� ������
modbus_registers[25]=(char)((LPC_RTC->DOM)%256);
modbus_registers[26]=(char)((LPC_RTC->HOUR)/256);			//���14  �����, ���
modbus_registers[27]=(char)((LPC_RTC->HOUR)%256);
modbus_registers[28]=(char)((LPC_RTC->MIN)/256);			//���15  �����, ������
modbus_registers[29]=(char)((LPC_RTC->MIN)%256);
modbus_registers[30]=(char)((LPC_RTC->SEC)/256);			//���16  �����, �������
modbus_registers[31]=(char)((LPC_RTC->SEC)%256);
modbus_registers[38]=(char)(NUMIST/256);				//���20  ���������� ������������ � ���������
modbus_registers[39]=(char)(NUMIST%256);
modbus_registers[40]=(char)(PAR/256);					//���21  ������������ ������ ������������ ���./����.
modbus_registers[41]=(char)(PAR%256);
modbus_registers[42]=(char)(ZV_ON/256);					//���22  �������� ��������� ������������ ���./����.
modbus_registers[43]=(char)(ZV_ON%256);
modbus_registers[46]=(char)(UBM_AV/256);				//���24  ��������� ������� ���������� ���������� ������� ����� �������, %
modbus_registers[47]=(char)(UBM_AV%256);
modbus_registers[58]=(char)(TBAT/256);					//���30  ������ �������� ���� �������, �����.
modbus_registers[59]=(char)(TBAT%256);
modbus_registers[60]=(char)(UMAX/256);					//���31  ������������ (���������) ���������� ������������, 0.1�
modbus_registers[61]=(char)(UMAX%256);
modbus_registers[62]=(char)((UB20-DU)/256);				//���32  ����������� (���������) ���������� ������������, 0.1�
modbus_registers[63]=(char)((UB20-DU)%256);
modbus_registers[64]=(char)(UB0/256);					//���33  ���������� ���������� ������� ��� 0��, 0.1�
modbus_registers[65]=(char)(UB0%256);
modbus_registers[66]=(char)(UB20/256);					//���34  ���������� ���������� ������� ��� 20��, 0.1�
modbus_registers[67]=(char)(UB20%256);
modbus_registers[68]=(char)(USIGN/256);					//���35  ����������� (����������) ���������� �������, 1�
modbus_registers[69]=(char)(USIGN%256);
modbus_registers[70]=(char)(UMN/256);					//���36  ����������� (���������) ���������� �������� ����, 1�
modbus_registers[71]=(char)(UMN%256);
modbus_registers[72]=(char)(U0B/256);					//���37  ������� ���������� ��� ����������� ��������, 0.1�
modbus_registers[73]=(char)(U0B%256);
modbus_registers[74]=(char)(IKB/256);					//���38  ��� �������� ������� �������, 0.1�
modbus_registers[75]=(char)(IKB%256);
modbus_registers[76]=(char)(IZMAX/256);					//���39  ��� ������ ������� ������������, 0.1�
modbus_registers[77]=(char)(IZMAX%256);
modbus_registers[78]=(char)(IMAX/256);					//���40  ��� ������������ �� ������� ���-�� ������������, 0.1�
modbus_registers[79]=(char)(IMAX%256);
modbus_registers[80]=(char)(IMIN/256);					//���41  ��� ������������ �� ������� ���-�� ������������, 0.1�
modbus_registers[81]=(char)(IMIN%256);
modbus_registers[82]=(char)(UVZ/256);					//���42  ���������� �������������� ������, 0.1�
modbus_registers[83]=(char)(UVZ%256);
modbus_registers[84]=(char)(TZAS/256);					//���43  ����� �������� ��������� ������������, ���
modbus_registers[85]=(char)(TZAS%256);
modbus_registers[86]=(char)(TMAX/256);					//���44  ����������� ������������ ���������, 1��
modbus_registers[87]=(char)(TMAX%256);
modbus_registers[88]=(char)(TSIGN/256);					//���45  ����������� ������������ ����������, 1��
modbus_registers[89]=(char)(TSIGN%256);
modbus_registers[90]=(char)(TBATMAX/256);				//���46  ����������� ������� ���������, 1��
modbus_registers[91]=(char)(TBATMAX%256);
modbus_registers[92]=(char)(TBATSIGN/256);				//���47  ����������� ������� ����������, 1��
modbus_registers[93]=(char)(TBATSIGN%256);
modbus_registers[94]=(char)(speedChrgCurr/256);					//���48  ��� ����������� ������, 0.1�
modbus_registers[95]=(char)(speedChrgCurr%256);
modbus_registers[96]=(char)(speedChrgVolt/256);				//���49	 ���������� ����������� ������, 0.1� 
modbus_registers[97]=(char)(speedChrgVolt%256);
modbus_registers[98]=(char)(speedChrgTimeInHour/256);				//���50	 ����� ����������� ������, 1�
modbus_registers[99]=(char)(speedChrgTimeInHour%256);


modbus_tx_buff[0]=adr;
modbus_tx_buff[1]=func;
modbus_tx_buff[2]=(char)(reg_adr/256);
modbus_tx_buff[3]=(char)(reg_adr%256);

mem_copy((char*)&modbus_tx_buff[4],(char*)&modbus_registers[(reg_adr-1)*2],2);

crc_temp=CRC16_2(modbus_tx_buff,6);

modbus_tx_buff[6]=crc_temp%256;
modbus_tx_buff[7]=crc_temp/256;

for (i=0;i<8;i++)
	{
	putchar0(modbus_tx_buff[i]);
	}
for (i=0;i<8;i++)
	{
	putchar_sc16is700(modbus_tx_buff[i]);
	}
}
*/

//-----------------------------------------------
void modbus_hold_registers_transmit(unsigned char adr,unsigned char func,unsigned short reg_adr,unsigned short reg_quantity, char prot)
{
signed char modbus_registers[2500];
signed short tempSS;
//char modbus_tx_buff[150];
unsigned short crc_temp;
char i;
long temp_time;

modbus_registers[0]=(char)((hmi_cntrl_reg)>>8);				//���1  ������� ���������� ��� �������
modbus_registers[1]=(char)((hmi_cntrl_reg));
modbus_registers[2]=(char)((hmi_avg_reg)>>8);				//���2  ������� ���������� ������������� ����� ��� �������
modbus_registers[3]=(char)((hmi_avg_reg));
modbus_registers[4]=(char)((hmi_cntrl_fb_reg)>>8);			//���3  ������� �������� ����� ���������� ��� �������
modbus_registers[5]=(char)((hmi_cntrl_fb_reg));

modbus_registers[20]=(char)((BKP->DR1)>>8);					//���11  �����, ���
modbus_registers[21]=(char)((BKP->DR1));
modbus_registers[22]=(char)((BKP->DR2)>>8);					//���12  �����, �����
modbus_registers[23]=(char)((BKP->DR2));
modbus_registers[24]=(char)((BKP->DR3)>>8);					//���13  �����, ���� ������
modbus_registers[25]=(char)((BKP->DR3));
temp_time=(((long)(RTC->CNTH))<<16)+((long)(RTC->CNTL));
modbus_registers[26]=(char)((temp_time)/3600)>>8;			//���14  �����, ���
modbus_registers[27]=(char)((temp_time)/3600);
modbus_registers[28]=(char)(((temp_time)%3600)/60)>>8;		//���15  �����, ������
modbus_registers[29]=(char)(((temp_time)%3600)/60);
modbus_registers[30]=(char)(((temp_time)%3600)%60)>>8;		//���16  �����, �������
modbus_registers[31]=(char)(((temp_time)%3600)%60);
modbus_registers[38]=(char)(NUMIST>>8);				//���20  ���������� ������������ � ���������
modbus_registers[39]=(char)(NUMIST);
///*/

//PAR=123;

modbus_registers[40]=(char)(PAR>>8);					//���21  ������������ ������ ������������ ���./����.
modbus_registers[41]=(char)(PAR);
modbus_registers[42]=(char)(ZV_ON>>8);					//���22  �������� ��������� ������������ ���./����.
modbus_registers[43]=(char)(ZV_ON);
modbus_registers[46]=(char)(UBM_AV>>8);					//���24  ��������� ������� ���������� ���������� ������� ����� �������, %
modbus_registers[47]=(char)(UBM_AV);
modbus_registers[58]=(char)(TBAT>>8);					//���30  ������ �������� ���� �������, �����.
modbus_registers[59]=(char)(TBAT);
modbus_registers[60]=(char)(UMAX>>8);					//���31  ������������ (���������) ���������� ������������, 0.1�
modbus_registers[61]=(char)(UMAX);
modbus_registers[62]=(char)((UB20-DU)>>8);				//���32  ����������� (���������) ���������� ������������, 0.1�
modbus_registers[63]=(char)((UB20-DU));
modbus_registers[64]=(char)(UDCDC>>8);					//���33  ��������� ��������� ���������� ��� ������� DCDC, 0.1�
modbus_registers[65]=(char)(UDCDC);
modbus_registers[66]=(char)(UB20>>8);					//���34  ���������� ���������� ������� ��� 20��, 0.1�
modbus_registers[67]=(char)(UB20);
modbus_registers[68]=(char)(USIGN>>8);					//���35  ����������� (����������) ���������� �������, 1�
modbus_registers[69]=(char)(USIGN);
modbus_registers[70]=(char)(UMN>>8);					//���36  ����������� (���������) ���������� �������� ����, 1�
modbus_registers[71]=(char)(UMN);
modbus_registers[72]=(char)(U0B>>8);					//���37  ������� ���������� ��� ����������� ��������, 0.1�
modbus_registers[73]=(char)(U0B);
modbus_registers[74]=(char)(IKB>>8);					//���38  ��� �������� ������� �������, 0.1�
modbus_registers[75]=(char)(IKB);
modbus_registers[76]=(char)(IZMAX>>8);					//���39  ��� ������ ������� ������������, 0.1�
modbus_registers[77]=(char)(IZMAX);
modbus_registers[78]=(char)(IMAX>>8);					//���40  ��� ������������ �� ������� ���-�� ������������, 0.1�
modbus_registers[79]=(char)(IMAX);
modbus_registers[80]=(char)(IMIN>>8);					//���41  ��� ������������ �� ������� ���-�� ������������, 0.1�
modbus_registers[81]=(char)(IMIN);
modbus_registers[82]=(char)(UVZ>>8);					//���42  ���������� �������������� ������, 0.1�
modbus_registers[83]=(char)(UVZ);
modbus_registers[84]=(char)(TZAS>>8);					//���43  ����� �������� ��������� ������������, ���
modbus_registers[85]=(char)(TZAS);
modbus_registers[86]=(char)(TMAX>>8);					//���44  ����������� ������������ ���������, 1��
modbus_registers[87]=(char)(TMAX);
modbus_registers[88]=(char)(TSIGN>>8);					//���45  ����������� ������������ ����������, 1��
modbus_registers[89]=(char)(TSIGN);
modbus_registers[90]=(char)(TBATMAX>>8);				//���46  ����������� ������� ���������, 1��
modbus_registers[91]=(char)(TBATMAX);
modbus_registers[92]=(char)(TBATSIGN>>8);				//���47  ����������� ������� ����������, 1��
modbus_registers[93]=(char)(TBATSIGN);
modbus_registers[94]=(char)(speedChrgCurr>>8);			//���48  ��� ����������� ������, 0.1�
modbus_registers[95]=(char)(speedChrgCurr);
modbus_registers[96]=(char)(speedChrgVolt>>8);			//���49	 ���������� ����������� ������, 0.1� 
modbus_registers[97]=(char)(speedChrgVolt);
modbus_registers[98]=(char)(speedChrgTimeInHour>>8);	//���50	 ����� ����������� ������, 1�
modbus_registers[99]=(char)(speedChrgTimeInHour);
modbus_registers[100]=(char)(U_OUT_KONTR_MAX>>8);		//���51	 �������� ��������� ����������, Umax, 0.1�
modbus_registers[101]=(char)(U_OUT_KONTR_MAX);
modbus_registers[102]=(char)(U_OUT_KONTR_MIN>>8);		//���52	 �������� ��������� ����������, Umin, 0.1�
modbus_registers[103]=(char)(U_OUT_KONTR_MIN);
modbus_registers[104]=(char)(U_OUT_KONTR_DELAY>>8);		//���53	 �������� ��������� ����������, T��������, 1���.
modbus_registers[105]=(char)(U_OUT_KONTR_DELAY);
modbus_registers[106]=(char)(UDCDC>>8);					//���54	 ��������� ��������� ���������� ��� ������� DCDC
modbus_registers[107]=(char)(UDCDC);
modbus_registers[108]=(char)(UMAXN>>8);					//���55  ������������ (���������) ���������� �������� ����, 1�
modbus_registers[109]=(char)(UMAXN);
modbus_registers[138]=(char)(MODBUS_ADRESS>>8);			//���70	 MODBUS ADRESS
modbus_registers[139]=(char)(MODBUS_ADRESS);
modbus_registers[140]=(char)(MODBUS_BAUDRATE>>8);		//���71	 MODBUS BAUDRATE
modbus_registers[141]=(char)(MODBUS_BAUDRATE);
modbus_registers[142]=(char)(speedChrgAvtEn>>8);		//���72	 �������������� ���������� �����, ���(1) ����(0)
modbus_registers[143]=(char)(speedChrgAvtEn);
modbus_registers[144]=(char)(speedChrgDU>>8);			//���73	 �������������� ���������� ����� ������ (���������� �������� ���������), 1�
modbus_registers[145]=(char)(speedChrgDU);
//modbus_registers[146]=(char)(speedChrgBlckSrc>>8);		//���74	 �������� ���������� ����������� ������, ����(0) ��1(1) ��2(2)
//modbus_registers[147]=(char)(speedChrgBlckSrc);
//modbus_registers[148]=(char)(speedChrgBlckLog>>8);		//���75	 ������ ���������� ����������� ������, ����������(0) ��������(1) 
//modbus_registers[149]=(char)(speedChrgBlckLog);
modbus_registers[150]=(char)(SP_CH_VENT_BLOK>>8);		//���76	 ������������ ����������� ������ �����������, ���(1) ����(0) 
modbus_registers[151]=(char)(SP_CH_VENT_BLOK);
tempSS=0;
if(NUMPHASE==1)tempSS=1;
modbus_registers[152]=(char)(tempSS>>8);				//���77	 ���������� ��� �������� ����, 0 - (3)  1 - (1) 
modbus_registers[153]=(char)(tempSS);
modbus_registers[154]=(char)(RELE1SET>>8);				//���78	 ��������� ������������ ����1  
modbus_registers[155]=(char)(RELE1SET);					
modbus_registers[156]=(char)(RELE2SET>>8);				//���79	 ��������� ������������ ����2  
modbus_registers[157]=(char)(RELE2SET);					//�������� ����� ��� � � ���78
modbus_registers[158]=(char)(RELE3SET>>8);				//���80	 ��������� ������������ ����3  
modbus_registers[159]=(char)(RELE3SET);					//�������� ����� ��� � � ���78
modbus_registers[160]=(char)(BAT_C_POINT_20>>8);		//���81	 ������� ������� ��� 20-�� ������� �������  
modbus_registers[161]=(char)(BAT_C_POINT_20);			
modbus_registers[162]=(char)(BAT_C_POINT_10>>8);		//���82	 ������� ������� ��� 10-�� ������� �������  
modbus_registers[163]=(char)(BAT_C_POINT_10);
modbus_registers[164]=(char)(BAT_C_POINT_5>>8);			//���83	 ������� ������� ��� 5-�� ������� �������  
modbus_registers[165]=(char)(BAT_C_POINT_5);
modbus_registers[166]=(char)(BAT_C_POINT_3>>8);			//���84	 ������� ������� ��� 3-� ������� �������  
modbus_registers[167]=(char)(BAT_C_POINT_3);
modbus_registers[168]=(char)(BAT_C_POINT_1>>8);			//���85	 ������� ������� ��� 1-0 ������� �������  
modbus_registers[169]=(char)(BAT_C_POINT_1);
modbus_registers[170]=(char)(BAT_C_POINT_1_2>>8);		//���86	 ������� ������� ��� 1/2 ������� �������  
modbus_registers[171]=(char)(BAT_C_POINT_1_2);
modbus_registers[172]=(char)(BAT_C_POINT_1_6>>8);		//���87	 ������� ������� ��� 1/6 ������� �������  
modbus_registers[173]=(char)(BAT_C_POINT_1_6);
modbus_registers[174]=(char)(BAT_U_END_20>>8);		//���88	 ���������� ������������ ��������� ��� 20-�� ������� �������  
modbus_registers[175]=(char)(BAT_U_END_20);			
modbus_registers[176]=(char)(BAT_U_END_10>>8);		//���89	 ���������� ������������ ��������� ��� 10-�� ������� �������  
modbus_registers[177]=(char)(BAT_U_END_10);
modbus_registers[178]=(char)(BAT_U_END_5>>8);			//���90	 ���������� ������������ ��������� ��� 5-�� ������� �������  
modbus_registers[179]=(char)(BAT_U_END_5);
modbus_registers[180]=(char)(BAT_U_END_3>>8);			//���91	 ���������� ������������ ��������� ��� 3-� ������� �������  
modbus_registers[181]=(char)(BAT_U_END_3);
modbus_registers[182]=(char)(BAT_U_END_1>>8);			//���92	 ���������� ������������ ��������� ��� 1-0 ������� �������  
modbus_registers[183]=(char)(BAT_U_END_1);
modbus_registers[184]=(char)(BAT_U_END_1_2>>8);		//���93	 ���������� ������������ ��������� ��� 1/2 ������� �������  
modbus_registers[185]=(char)(BAT_U_END_1_2);
modbus_registers[186]=(char)(BAT_U_END_1_6>>8);		//���94	 ���������� ������������ ��������� ��� 1/6 ������� �������  
modbus_registers[187]=(char)(BAT_U_END_1_6);
modbus_registers[188]=(char)(BAT_C_POINT_NUM_ELEM>>8);		//���95	 ���������� 2��������� ��������� � �������  
modbus_registers[189]=(char)(BAT_C_POINT_NUM_ELEM);
modbus_registers[190]=(char)(BAT_K_OLD>>8);					//���96	 ����������� �������� �������  
modbus_registers[191]=(char)(BAT_K_OLD);
modbus_registers[192]=(char)(FZ_U1>>8);						//���97	 ����������� �����, ���������� ���� 1, 0.1�  
modbus_registers[193]=(char)(FZ_U1);
modbus_registers[194]=(char)(FZ_IMAX1>>8);					//���98	 ����������� �����, ������������ ��� ���� 1, 0.1�  
modbus_registers[195]=(char)(FZ_IMAX1);
modbus_registers[196]=(char)(FZ_T1>>8);						//���99	 ����������� �����, ������������ ���� 1, 1�  
modbus_registers[197]=(char)(FZ_T1);
modbus_registers[198]=(char)(FZ_ISW12>>8);					//���100  ����������� �����, ��� ������������ � ���� 1 �� ���� 2, 0.1�  
modbus_registers[199]=(char)(FZ_ISW12);
modbus_registers[200]=(char)(FZ_U2>>8);						//���101  ����������� �����, ���������� ���� 2, 0.1�  
modbus_registers[201]=(char)(FZ_U2);
modbus_registers[202]=(char)(FZ_IMAX2>>8);					//���102  ����������� �����, ������������ ��� ���� 2, 0.1�  
modbus_registers[203]=(char)(FZ_IMAX2);
modbus_registers[204]=(char)(FZ_T2>>8);						//���103  ����������� �����, ������������ ���� 2, 1�  
modbus_registers[205]=(char)(FZ_T2);


/*

	ptrs[0]=	" I���.���.        !�";
	ptrs[1]=	" U���.���.        @�";
	ptrs[2]=	" T���.���.        #�";
	ptrs[3]=	" ��������������     ";
	ptrs[4]=	" �����.�����.      $";
	ptrs[5]=	" dU���.���.       %�";
	ptrs[6]=	" ������������      ^";
	ptrs[7]=	" ������ ������������";
	ptrs[8]=	"                   &";
	ptrs[9]=	" ������������       ";
	ptrs[10]=	" �����������       (";
	ptrs[11]=	" �����              ";

	
	if(sub_ind<index_set) index_set=sub_ind;
	else if((sub_ind-index_set)>2) index_set=sub_ind-2;

     bgnd_par(	"  ���������� �����  ",
			ptrs[index_set],
			ptrs[index_set+1],
			ptrs[index_set+2]);
	
	pointer_set(1);

	int2lcd(speedChrgCurr,'!',1);
	int2lcd(speedChrgVolt,'@',1);
	int2lcd(speedChrgTimeInHour,'#',0);
	if(speedChrgAvtEn==1)sub_bgnd("���.",'$',-3);   
    else sub_bgnd("����.",'$',-4); 
	int2lcd(speedChrgDU,'%',0);
	if(speedChrgBlckSrc==1)sub_bgnd("��1",'^',-2);
	else if(speedChrgBlckSrc==2)sub_bgnd("��2",'^',-2);   
    else if(speedChrgBlckSrc==0)sub_bgnd("����.",'^',-4);
	else sub_bgnd("������..",'^',-6); 
	if(speedChrgBlckLog==0)sub_bgnd("�������.",'&',-7);
	else if(speedChrgBlckLog==1) sub_bgnd("�����.",'&',-5);  
    else sub_bgnd("������.",'&',-6); 	  
	if(SP_CH_VENT_BLOK==1)sub_bgnd("���",'(',-2);
    else if(SP_CH_VENT_BLOK==0)sub_bgnd("����.",'(',-4);
	else sub_bgnd("������..",'(',-6);*/

//modbus_register_1000 = 2;
//modbus_register_1001 = 7890;
//modbus_register_1002 = 8765;hv_vz_up_cnt
modbus_registers[1878]=(signed char)(main_kb_cnt>>8);							//���940	������� �������� �������� ������� �������
modbus_registers[1879]=(signed char)(main_kb_cnt);
modbus_registers[1898]=(char)(log_buff_mb[0]>>8);								//���950	 ������� 0 ������ ������ �������
modbus_registers[1899]=(char)(log_buff_mb[0]);
modbus_registers[1900]=(char)(log_buff_mb[1]>>8);								//���951	 ������� 1 ������ ������ �������
modbus_registers[1901]=(char)(log_buff_mb[1]);
modbus_registers[1902]=(char)(log_buff_mb[2]>>8);								//���952	 ������� 2 ������ ������ �������
modbus_registers[1903]=(char)(log_buff_mb[2]);
modbus_registers[1904]=(char)(log_buff_mb[3]>>8);								//���953	 ������� 3 ������ ������ �������
modbus_registers[1905]=(char)(log_buff_mb[3]);
modbus_registers[1906]=(char)(log_buff_mb[4]>>8);								//���954	 ������� 4 ������ ������ �������
modbus_registers[1907]=(char)(log_buff_mb[4]);
modbus_registers[1908]=(char)(log_buff_mb[5]>>8);								//���955	 ������� 5 ������ ������ �������
modbus_registers[1909]=(char)(log_buff_mb[5]);
modbus_registers[1910]=(char)(log_buff_mb[6]>>8);								//���956	 ������� 6 ������ ������ �������
modbus_registers[1911]=(char)(log_buff_mb[6]);
modbus_registers[1912]=(char)(log_buff_mb[7]>>8);								//���957	 ������� 7 ������ ������ �������
modbus_registers[1913]=(char)(log_buff_mb[7]);
modbus_registers[1914]=(char)(log_buff_mb[8]>>8);								//���958	 ������� 8 ������ ������ �������
modbus_registers[1915]=(char)(log_buff_mb[8]);
modbus_registers[1916]=(char)(log_buff_mb[9]>>8);								//���959	 ������� 9 ������ ������ �������
modbus_registers[1917]=(char)(log_buff_mb[9]);
modbus_registers[1918]=(char)(log_buff_mb[10]>>8);								//���960	 ������� 10 ������ ������ �������
modbus_registers[1919]=(char)(log_buff_mb[10]);
modbus_registers[1920]=(char)(log_buff_mb[11]>>8);								//���961	 ������� 11 ������ ������ �������
modbus_registers[1921]=(char)(log_buff_mb[11]);
modbus_registers[1922]=(char)(log_buff_mb[12]>>8);								//���962	 ������� 12 ������ ������ �������
modbus_registers[1923]=(char)(log_buff_mb[12]);
modbus_registers[1924]=(char)(log_buff_mb[13]>>8);								//���963	 ������� 13 ������ ������ �������
modbus_registers[1925]=(char)(log_buff_mb[13]);
modbus_registers[1926]=(char)(log_buff_mb[14]>>8);								//���964	 ������� 14 ������ ������ �������
modbus_registers[1927]=(char)(log_buff_mb[14]);
modbus_registers[1928]=(char)(log_buff_mb[15]>>8);								//���965	 ������� 15 ������ ������ �������
modbus_registers[1929]=(char)(log_buff_mb[15]);
modbus_registers[1930]=(char)(log_debug0_mb>>8);								//���966	 ���������� �������0 ������� �������
modbus_registers[1931]=(char)(log_debug0_mb);
modbus_registers[1932]=(char)(log_debug1_mb>>8);								//���967	 ���������� �������1 ������� �������
modbus_registers[1933]=(char)(log_debug1_mb);
modbus_registers[1934]=(char)(log_deep_mb>>8);									//���968	 ������� ������� �������
modbus_registers[1935]=(char)(log_deep_mb);
modbus_registers[1936]=(char)(log_cmd_mb>>8);									//���969	 ��������� ������� ������� �������
modbus_registers[1937]=(char)(log_cmd_mb);										// <0...63> - ����������� ��� ������  ������ ������� � ��������������� �������
																				// 1000	- ������� ������ �������
																				// 2000 - �������� ������ � ������� �������
																				// 10001 - �������� ������ ���� �� �����������
																			   	// 10002 - �������� ������ ���� �� �����������

modbus_registers[1938]=(char)(test_hndl_rele1_cntrl>>8);						//���970	 ���������� ���� �1 � ����� (0 - �� �������, 1 - ������������� ���., 2 - ������������� ����)
modbus_registers[1939]=(char)(test_hndl_rele1_cntrl);
modbus_registers[1940]=(char)(test_hndl_rele2_cntrl>>8);						//���971	 ���������� ���� �2 � ����� (0 - �� �������, 1 - ������������� ���., 2 - ������������� ����)
modbus_registers[1941]=(char)(test_hndl_rele2_cntrl);
modbus_registers[1942]=(char)(test_hndl_rele3_cntrl>>8);						//���972	 ���������� ���� �3 � ����� (0 - �� �������, 1 - ������������� ���., 2 - ������������� ����)
modbus_registers[1943]=(char)(test_hndl_rele3_cntrl);
modbus_registers[1944]=(char)(test_hndl_releHV_cntrl>>8);						//���973	 ���������� ���� HV � ����� (0 - �� �������, 1 - ������������� ���., 2 - ������������� ����)
modbus_registers[1945]=(char)(test_hndl_releHV_cntrl);
modbus_registers[1946]=(char)(test_hndl_bps_number>>8);							//���974	 ���������� ������ � �����, ����� ��� (0-�� ��������� �������,1-3 - ����� ������������ ���, 255 - ��������� �����)
modbus_registers[1947]=(char)(test_hndl_bps_number);
modbus_registers[1948]=(char)(test_hndl_bps_state>>8);							//���975	 ���������� ������ � �����, (0 - ���������, 1 - �������� ������������ ���, 2 - �������� ����������� ���, 3 - �������� ���������������, 4 - �������� ���������)
modbus_registers[1949]=(char)(test_hndl_bps_state);
modbus_registers[1950]=(char)(zv_test_sign>>8);									//���976	 ���������� ������ � �����, (0 - ���������, 1 - �������� ������������ ���, 2 - �������� ����������� ���, 3 - �������� ���������������, 4 - �������� ���������)
modbus_registers[1951]=(char)(zv_test_sign);
modbus_registers[1952]=(char)(test_led_stat>>8);								//���977	 ���� ����������� 0 - ��������, 1 - �������(U�������), 2 - ������(�������), 3 - �������(������), 4 - �������(���), 5 - ��� ��������� 
modbus_registers[1953]=(char)(test_led_stat);


tempSS=0;
if((sp_ch_stat==scsSTEP1)||(sp_ch_stat==scsWRK))tempSS=1;
modbus_registers[1962]=(char)(tempSS>>8);										//���982	 ������������ ����������� ������ ���(1) ����(0) 
modbus_registers[1963]=(char)(tempSS);

tempSS=(short)(hv_vz_up_cnt/60L);
modbus_registers[1964]=(char)(tempSS>>8);										//���983	 ����� ������ ����������� ������ � �������  
modbus_registers[1965]=(char)(tempSS);


modbus_registers[1994]=(char)(modbus_register_998>>8);							//���998	 ����� ������������ ���������
modbus_registers[1995]=(char)(modbus_register_998);
modbus_registers[1996]=(char)(modbus_register_999>>8);							//���999	 ��������� ��������� ���� ��� ����������
modbus_registers[1997]=(char)(modbus_register_999);
modbus_registers[1998]=(char)(modbus_register_1000>>8);							//���54	 ��������� ��������� ���������� ��� ��� ��� �������(����-�������)
modbus_registers[1999]=(char)(modbus_register_1000);
modbus_registers[2000]=(char)(modbus_register_1001>>8);							//���54	 ��������� ��������� ���������� ��� ��� ��� �������(����-�������)
modbus_registers[2001]=(char)(modbus_register_1001);
modbus_registers[2002]=(char)(modbus_register_1002>>8);							//���54	 ��������� ��������� ���������� ��� ��� ��� �������(����-�������)
modbus_registers[2003]=(char)(modbus_register_1002);
modbus_registers[2004]=(char)(modbus_register_1003>>8);							//���54	 ��������� ��������� ���������� ��� ��� ��� �������(����-�������)
modbus_registers[2005]=(char)(modbus_register_1003);

modbus_registers[2042]=(char)(modbus_register_1022>>8);							//���1022	 ���������� U���. ���
modbus_registers[2043]=(char)(modbus_register_1022);
modbus_registers[2044]=(char)(modbus_register_1023>>8);							//���1023	 ���������� U���.���. ���
modbus_registers[2045]=(char)(modbus_register_1023);
modbus_registers[2046]=(char)(modbus_register_1024>>8);							//���1024	 ���������� I���. ���
modbus_registers[2047]=(char)(modbus_register_1024);
modbus_registers[2048]=(char)(modbus_register_1025>>8);							//���1025	 ���������� T���.���. ���
modbus_registers[2049]=(char)(modbus_register_1025);


if(prot==MODBUS_RTU_PROT)
	{
	modbus_tx_buff[0]=adr;
	modbus_tx_buff[1]=func;

	modbus_tx_buff[2]=(char)(reg_quantity*2);

	memcpy((signed char*)&modbus_tx_buff[3],(signed char*)&modbus_registers[(reg_adr-1)*2],reg_quantity*2);

	crc_temp=CRC16_2(modbus_tx_buff,(reg_quantity*2)+3);

	modbus_tx_buff[3+(reg_quantity*2)]=(char)crc_temp;
	modbus_tx_buff[4+(reg_quantity*2)]=crc_temp>>8;



	for (i=0;i<(5+(reg_quantity*2));i++)
		{
		putchar1(modbus_tx_buff[i]);
		}
	}
else if(prot==MODBUS_TCP_PROT)
	{
	//*/mem_copy((signed char*)modbus_tx_buff,(signed char*)&modbus_registers[(reg_adr-1)*2],reg_quantity*2);
///*/	modbus_tcp_out_ptr=(signed char*)modbus_tx_buff;
	}
}

//-----------------------------------------------
void modbus_input_registers_transmit(unsigned char adr,unsigned char func,unsigned short reg_adr,unsigned short reg_quantity, char prot)
{
signed char modbus_registers[2200];
//char modbus_tx_buff[200];
unsigned short crc_temp;
char i,tempC;
short tempS,tempS_;

static short out_U_pl;

t_ext[0]=bat[0]._Tb;
plazma_uart1[2]++;
/*
out_U=plazma_short;
bps_I=5678;

net_U=223;
bps[0]._Uii=281;
bps[0]._Ii=123;


bps_I=5678;
*/
//bps_U=modbus_plazma;
/*out_U_pl++;
if (out_U_pl>1000) out_U_pl=100;
if (out_U_pl<100) out_U_pl=1000;
out_U=out_U_pl;
//out_U=4567;
bps_I=out_U+50;
t_ext[0]=out_U-50;
cntrl_stat=t_ext[0]*2;
u_necc=t_ext[0]*3; 
Ib_ips_termokompensat=out_U_pl-500;*/

//bps_I=100*MODBUS_ADRESS + main_1HZ_cnt;

modbus_registers[0]=(signed char)(out_U>>8);					//���1   	���������� �������� ����, 0.1�
modbus_registers[1]=(signed char)(out_U);
modbus_registers[2]=(signed char)(bps_I>>8);					//���2   	��� ������������, 0.1�
modbus_registers[3]=(signed char)(bps_I);



modbus_registers[4]=(signed char)(net_U>>8);					//���3   	���������� ���� �������, 1�
modbus_registers[5]=(signed char)(net_U);
modbus_registers[6]=(signed char)(net_F>>8);					//���4   	������� ���� �������, 0.1��
modbus_registers[7]=(signed char)(net_F);
modbus_registers[8]=(signed char)(net_Ua>>8);					//���5	���������� ���� ������� ���� A, 1�	
modbus_registers[9]=(signed char)(net_Ua);		 	
modbus_registers[10]=(signed char)(net_Ub>>8);				//���6	���������� ���� ������� ���� B, 1�
modbus_registers[11]=(signed char)(net_Ub);
modbus_registers[12]=(signed char)(net_Uc>>8);				//���7	���������� ���� ������� ���� C, 1�
modbus_registers[13]=(signed char)(net_Uc);
modbus_registers[14]=(signed char)(bat[0]._Ub>>8);				//���8	���������� ������� �1, 0.1�
modbus_registers[15]=(signed char)(bat[0]._Ub);
modbus_registers[16]=(signed char)(Ib_ips_termokompensat/*bat[0]._Ib*/>>8);				//���9   	��� ������� �1, 0.01�
modbus_registers[17]=(signed char)(Ib_ips_termokompensat/*bat[0]._Ib*/);


modbus_registers[18]=(signed char)(t_ext[0]>>8);				//���10	����������� ������� �1, 1��
modbus_registers[19]=(signed char)(t_ext[0]);
/*
modbus_registers[18]=(signed char)(bat[0]._Tb>>8);				//���10	����������� ������� �1, 1��
modbus_registers[19]=(signed char)(bat[0]._Tb);
#endif*/
#ifdef UKU_ZVU
modbus_registers[20]=(signed char)(((short)(bat_hndl_zvu_Q/10000L))>>8);			//���11	����� ������� �1, %
modbus_registers[21]=(signed char)(((short)(bat_hndl_zvu_Q/10000L)));
#else
modbus_registers[20]=(signed char)(bat[0]._zar>>8);			//���11	����� ������� �1, %
modbus_registers[21]=(signed char)(bat[0]._zar);
#endif
modbus_registers[22]=(signed char)(bat[0]._Ubm>>8);			//���12	���������� ������� ����� ������� �1, 0.1�
modbus_registers[23]=(signed char)(bat[0]._Ubm);
modbus_registers[24]=(signed char)(bat[0]._dUbm>>8);			//���13	������ ������� ����� ������� �1, %
modbus_registers[25]=(signed char)(bat[0]._dUbm);
modbus_registers[26]=(signed char)(BAT_C_REAL[0]>>8);			//���14	�������� ������� ������� �1, 0.1�*�, ���� 0x5555 �� �� ����������
modbus_registers[27]=(signed char)(BAT_C_REAL[0]);
modbus_registers[28]=(signed char)(bat[1]._Ub>>8);				//���15	���������� ������� �1, 0.1�
modbus_registers[29]=(signed char)(bat[1]._Ub);
modbus_registers[30]=(signed char)(bat[1]._Ib>>8);				//���16   	��� ������� �1, 0.01�
modbus_registers[31]=(signed char)(bat[1]._Ib);
modbus_registers[32]=(signed char)(bat[1]._Tb>>8);				//���17	����������� ������� �1, 1��
modbus_registers[33]=(signed char)(bat[1]._Tb);
modbus_registers[34]=(signed char)(bat[1]._zar>>8);				//���18	����� ������� �1, %
modbus_registers[35]=(signed char)(bat[1]._zar);
modbus_registers[36]=(signed char)(bat[1]._Ubm>>8);				//���19	���������� ������� ����� ������� �1, 0.1�
modbus_registers[37]=(signed char)(bat[1]._Ubm);
modbus_registers[38]=(signed char)(bat[1]._dUbm>>8);			//���20	������ ������� ����� ������� �1, %
modbus_registers[39]=(signed char)(bat[1]._dUbm);
modbus_registers[40]=(signed char)(BAT_C_REAL[1]>>8);			//���21	�������� ������� ������� �1, 0.1�*�, ���� 0x5555 �� �� ����������
modbus_registers[41]=(signed char)(BAT_C_REAL[1]);
modbus_registers[42]=(signed char)(bps[0]._Uii>>8);				//���22	�������� ���������� ����������� �1, 0.1�
modbus_registers[43]=(signed char)(bps[0]._Uii);
modbus_registers[44]=(signed char)(bps[0]._Uin>>8);				//���23	���������� ���� ����������� �1, 0.1�
modbus_registers[45]=(signed char)(bps[0]._Uin);
modbus_registers[46]=(signed char)(bps[0]._Ii>>8);				//���24	�������� ��� ����������� �1, 0.1�
modbus_registers[47]=(signed char)(bps[0]._Ii);
modbus_registers[48]=(signed char)(bps[0]._Ti>>8);				//���25	����������� ��������� ����������� �1, 1��
modbus_registers[49]=(signed char)(bps[0]._Ti);
modbus_registers[50]=(signed char)(bps[0]._av>>8);				//���26	���� ������ ����������� �1, 0x01 - ��������, 0x02 �������� U���, 0x04 �������� U���, 0x08 - ����������� ����� � ������������
modbus_registers[51]=(signed char)(bps[0]._av);
modbus_registers[52]=(signed char)(bps[1]._Uii>>8);				//���27	�������� ���������� ����������� �2, 0.1�
modbus_registers[53]=(signed char)(bps[1]._Uii);
modbus_registers[54]=(signed char)(bps[1]._Uin>>8);				//���28	���������� ���� ����������� �2, 0.1�
modbus_registers[55]=(signed char)(bps[1]._Uin);
modbus_registers[56]=(signed char)(bps[1]._Ii>>8);				//���29	�������� ��� ����������� �2, 0.1�
modbus_registers[57]=(signed char)(bps[1]._Ii);
modbus_registers[58]=(signed char)(bps[1]._Ti>>8);				//���30	����������� ��������� ����������� �2, 1��
modbus_registers[59]=(signed char)(bps[1]._Ti);
modbus_registers[60]=(signed char)(bps[1]._av>>8);				//���31	���� ������ ����������� �2, 0x01 - ��������, 0x02 �������� U���, 0x04 �������� U���, 0x08 - ����������� ����� � ������������
modbus_registers[61]=(signed char)(bps[1]._av);
modbus_registers[62]=(signed char)(bps[2]._Uii>>8);				//���32	�������� ���������� ����������� �3, 0.1�
modbus_registers[63]=(signed char)(bps[2]._Uii);
modbus_registers[64]=(signed char)(bps[2]._Uin>>8);				//���33	���������� ���� ����������� �3, 0.1�
modbus_registers[65]=(signed char)(bps[2]._Uin);
modbus_registers[66]=(signed char)(bps[2]._Ii>>8);				//���34	�������� ��� ����������� �3, 0.1�
modbus_registers[67]=(signed char)(bps[2]._Ii);
modbus_registers[68]=(signed char)(bps[2]._Ti>>8);				//���35	����������� ��������� ����������� �3, 1��
modbus_registers[69]=(signed char)(bps[2]._Ti);
modbus_registers[70]=(signed char)(bps[2]._av>>8);				//���36	���� ������ ����������� �3, 0x01 - ��������, 0x02 �������� U���, 0x04 �������� U���, 0x08 - ����������� ����� � ������������
modbus_registers[71]=(signed char)(bps[2]._av);
modbus_registers[72]=(signed char)(bps[3]._Uii>>8);				//���37	�������� ���������� ����������� �4, 0.1�
modbus_registers[73]=(signed char)(bps[3]._Uii);
modbus_registers[74]=(signed char)(bps[3]._Uin>>8);				//���38	���������� ���� ����������� �4, 0.1�
modbus_registers[75]=(signed char)(bps[3]._Uin);
modbus_registers[76]=(signed char)(bps[3]._Ii>>8);				//���39	�������� ��� ����������� �4, 0.1�
modbus_registers[77]=(signed char)(bps[3]._Ii);
modbus_registers[78]=(signed char)(bps[3]._Ti>>8);				//���40	����������� ��������� ����������� �4, 1��
modbus_registers[79]=(signed char)(bps[3]._Ti);
modbus_registers[80]=(signed char)(bps[3]._av>>8);				//���41	���� ������ ����������� �4, 0x01 - ��������, 0x02 �������� U���, 0x04 �������� U���, 0x08 - ����������� ����� � ������������
modbus_registers[81]=(signed char)(bps[3]._av);
modbus_registers[82]=(signed char)(bps[4]._Uii>>8);				//���42	�������� ���������� ����������� �5, 0.1�
modbus_registers[83]=(signed char)(bps[4]._Uii);
modbus_registers[84]=(signed char)(bps[4]._Uin>>8);				//���43	���������� ���� ����������� �5, 0.1�
modbus_registers[85]=(signed char)(bps[4]._Uin);
modbus_registers[86]=(signed char)(bps[4]._Ii>>8);				//���44	�������� ��� ����������� �5, 0.1�
modbus_registers[87]=(signed char)(bps[4]._Ii);
modbus_registers[88]=(signed char)(bps[4]._Ti>>8);				//���45	����������� ��������� ����������� �5, 1��
modbus_registers[89]=(signed char)(bps[4]._Ti);
modbus_registers[90]=(signed char)(bps[4]._av>>8);				//���46	���� ������ ����������� �5, 0x01 - ��������, 0x02 �������� U���, 0x04 �������� U���, 0x08 - ����������� ����� � ������������
modbus_registers[91]=(signed char)(bps[4]._av);
modbus_registers[92]=(signed char)(bps[5]._Uii>>8);				//���47	�������� ���������� ����������� �6, 0.1�
modbus_registers[93]=(signed char)(bps[5]._Uii);
modbus_registers[94]=(signed char)(bps[5]._Uin>>8);				//���48	���������� ���� ����������� �6, 0.1�
modbus_registers[95]=(signed char)(bps[5]._Uin);
modbus_registers[96]=(signed char)(bps[5]._Ii>>8);				//���49	�������� ��� ����������� �6, 0.1�
modbus_registers[97]=(signed char)(bps[5]._Ii);
modbus_registers[98]=(signed char)(bps[5]._Ti>>8);				//���50	����������� ��������� ����������� �6, 1��
modbus_registers[99]=(signed char)(bps[5]._Ti);
modbus_registers[100]=(signed char)(bps[5]._av>>8);				//���51	���� ������ ����������� �6, 0x01 - ��������, 0x02 �������� U���, 0x04 �������� U���, 0x08 - ����������� ����� � ������������
modbus_registers[101]=(signed char)(bps[5]._av);
modbus_registers[102]=(signed char)(bps[6]._Uii>>8);			//���52	�������� ���������� ����������� �7, 0.1�
modbus_registers[103]=(signed char)(bps[6]._Uii);
modbus_registers[104]=(signed char)(bps[6]._Uin>>8);			//���53	���������� ���� ����������� �7, 0.1�
modbus_registers[105]=(signed char)(bps[6]._Uin);
modbus_registers[106]=(signed char)(bps[6]._Ii>>8);				//���54	�������� ��� ����������� �7, 0.1�
modbus_registers[107]=(signed char)(bps[6]._Ii);
modbus_registers[108]=(signed char)(bps[6]._Ti>>8);				//���55	����������� ��������� ����������� �7, 1��
modbus_registers[109]=(signed char)(bps[6]._Ti);
modbus_registers[110]=(signed char)(bps[6]._av>>8);				//���56	���� ������ ����������� �7, 0x01 - ��������, 0x02 �������� U���, 0x04 �������� U���, 0x08 - ����������� ����� � ������������
modbus_registers[111]=(signed char)(bps[6]._av);
modbus_registers[112]=(signed char)(bps[7]._Uii>>8);			//���57	�������� ���������� ����������� �8, 0.1�
modbus_registers[113]=(signed char)(bps[7]._Uii);
modbus_registers[114]=(signed char)(bps[7]._Uin>>8);			//���58	���������� ���� ����������� �8, 0.1�
modbus_registers[115]=(signed char)(bps[7]._Uin);
modbus_registers[116]=(signed char)(bps[7]._Ii>>8);				//���59	�������� ��� ����������� �8, 0.1�
modbus_registers[117]=(signed char)(bps[7]._Ii);
modbus_registers[118]=(signed char)(bps[7]._Ti>>8);				//���60	����������� ��������� ����������� �8, 1��
modbus_registers[119]=(signed char)(bps[7]._Ti);
modbus_registers[120]=(signed char)(bps[7]._av>>8);				//���61	���� ������ ����������� �8, 0x01 - ��������, 0x02 �������� U���, 0x04 �������� U���, 0x08 - ����������� ����� � ������������
modbus_registers[121]=(signed char)(bps[7]._av);
modbus_registers[122]=(signed char)(bps[0]._cnt>>8);			//���62	������� ����� ����������� �1
modbus_registers[123]=(signed char)(bps[0]._cnt);
modbus_registers[124]=(signed char)(bps[1]._cnt>>8);			//���63	������� ����� ����������� �2
modbus_registers[125]=(signed char)(bps[1]._cnt);
modbus_registers[126]=(signed char)(bps[2]._cnt>>8);			//���64	������� ����� ����������� �3
modbus_registers[127]=(signed char)(bps[2]._cnt);
modbus_registers[128]=(signed char)(bps[3]._cnt>>8);			//���65	������� ����� ����������� �4
modbus_registers[129]=(signed char)(bps[3]._cnt);
modbus_registers[130]=(signed char)(bps[4]._cnt>>8);			//���66	������� ����� ����������� �5
modbus_registers[131]=(signed char)(bps[4]._cnt);
modbus_registers[132]=(signed char)(bps[5]._cnt>>8);			//���67	������� ����� ����������� �6
modbus_registers[133]=(signed char)(bps[5]._cnt);
modbus_registers[134]=(signed char)(bps[6]._cnt>>8);			//���68	������� ����� ����������� �7
modbus_registers[135]=(signed char)(bps[6]._cnt);
modbus_registers[136]=(signed char)(bps[7]._cnt>>8);			//���69	������� ����� ����������� �8
modbus_registers[137]=(signed char)(bps[7]._cnt);


modbus_registers[138]=(signed char)(bps_U>>8);					//���70   	���������� ������������, 0.1�
modbus_registers[139]=(signed char)(bps_U);
tempS=0;
tempS=u_necc-bps_U;
modbus_registers[140]=(signed char)(tempS>>8);					//���71   	������� ����� ����������� ����������� � ����������� ������������, 0.1�
modbus_registers[141]=(signed char)(tempS);
tempS=0;
tempS=IZMAX_-Ib_ips_termokompensat/10;
modbus_registers[142]=(signed char)(tempS>>8);					//���72   	������� ����� ������������ ����� ������ �������(�����������) � ����� �������, 0.1�
modbus_registers[143]=(signed char)(tempS);


if(sk_stat[i]==ssON)
	{
	//bps[0]._av=0x01;
	//avar_stat|=(1<<(3+0));
	}
else
	{
	//bps[0]._av=0x00;
	//avar_stat=0;
	}


tempS=0;
tempC=0;														//���73	������� ����� ������ �������
tempS=uout_av;													//�������� ��������� ����������, (0 - �����, 1 - ��������, 2 - ��������)	
for (i=0;i<NUMIST;i++)
	{
	tempC=bps[i]._av;
	}
tempC<<=2;
tempC&=0x7c;

tempS|=(short)tempC;

tempS_=(short)net_av;
tempS_<<=7;
tempS_&=0x0180;
tempS|=tempS_;

tempC=0;
for (i=0;i<NUMIST;i++)
	{
	tempC|=bps[i]._flags_tm&0x01;
	}
tempC<<=7;
tempC&=0x80;
tempS|=(short)tempC;
modbus_registers[144]=(signed char)(tempS>>8);				
modbus_registers[145]=(signed char)(tempS);

tempS=0;													 	//���74	������� ������ ��������� �������
if(bat_ips._av)			tempS|=(1<<0);						 	// ��� 0	������ �������
if(net_av)   			tempS|=(1<<1);						 	//	��� 1	������ �������� ���� 
if(bps[0]._av&0x3f)		tempS|=(1<<2);						 	//	��� 2	������ ��� �������������� ����������� �1
if(bps[1]._av&0x3f)		tempS|=(1<<3);						 	//	��� 3	������ ��� �������������� ����������� �2
if(bps[2]._av&0x3f)		tempS|=(1<<4);						 	//	��� 4	������ ��� �������������� ����������� �3
if(bps[3]._av&0x3f)		tempS|=(1<<5);						 	//	��� 5	������ ��� �������������� ����������� �4
if(bps[4]._av&0x3f)		tempS|=(1<<6);						 	//	��� 6	������ ��� �������������� ����������� �5
if(bps[5]._av&0x3f)		tempS|=(1<<7);						 	//	��� 7	������ ��� �������������� ����������� �6
if(bps[6]._av&0x3f)		tempS|=(1<<8);						 	//	��� 8	������ ��� �������������� ����������� �7
if(bps[7]._av&0x3f)		tempS|=(1<<9);						 	//	��� 9	������ ��� �������������� ����������� �8
if(uout_av)				tempS|=(1<<14);							//	��� 14	������ ��� �������������� ��������� ����������
if(sk_stat[0]==ssON)	tempS|=(1<<15);						 	//	��� 15	������� ����� �������
//tempS=23456;
modbus_registers[146]=(signed char)(tempS>>8);
modbus_registers[147]=(signed char)(tempS);
/*
cntrl_stat=641;
u_necc=2345;
IZMAX_=1234;
bps_I=986;	*/


modbus_registers[166]=(signed char)(bps[0]._x_>>8);				//���84	��������� ������������ ����� ���1 
modbus_registers[167]=(signed char)(bps[0]._x_);
modbus_registers[168]=(signed char)(bps[1]._x_>>8);				//���85	��������� ������������ ����� ���2 
modbus_registers[169]=(signed char)(bps[1]._x_);
modbus_registers[170]=(signed char)(bps[2]._x_>>8);				//���86	��������� ������������ ����� ���3 
modbus_registers[171]=(signed char)(bps[2]._x_);
modbus_registers[172]=(signed char)(bps[3]._x_>>8);				//���87	��������� ������������ ����� ���4 
modbus_registers[173]=(signed char)(bps[3]._x_);
modbus_registers[174]=(signed char)(bps[4]._x_>>8);				//���88	��������� ������������ ����� ���5 
modbus_registers[175]=(signed char)(bps[4]._x_);
modbus_registers[176]=(signed char)(bps[5]._x_>>8);				//���89	��������� ������������ ����� ���6 
modbus_registers[177]=(signed char)(bps[5]._x_);
modbus_registers[178]=(signed char)(bps[6]._x_>>8);				//���90	��������� ������������ ����� ���7 
modbus_registers[179]=(signed char)(bps[6]._x_);
modbus_registers[180]=(signed char)(bps[7]._x_>>8);				//���91	��������� ������������ ����� ���8 
modbus_registers[181]=(signed char)(bps[7]._x_);

//bps[0]._av=123;
hmi_plazma[0]=bps[1]._rotor;
hmi_plazma[1]=avar_stat;
hmi_plazma[2]=bps[0]._av;
hmi_plazma[3]=bps[1]._av;
hmi_plazma[4]=bps[0]._flags_tm;
hmi_plazma[5]=bps[1]._flags_tm;
hmi_plazma[6]=bps[0]._temp_av_cnt;
hmi_plazma[7]=bps[0]._umin_av_cnt;
hmi_plazma[8]=bps[0]._umax_av_cnt;										//������� ������������ ����� �� ������;

modbus_registers[182]=(signed char)(hmi_plazma[0]>>8);			//���92	���������� ���������� ��� ������ 
modbus_registers[183]=(signed char)(hmi_plazma[0]);
modbus_registers[184]=(signed char)(hmi_plazma[1]>>8);			//���93	���������� ���������� ��� ������ 
modbus_registers[185]=(signed char)(hmi_plazma[1]);
modbus_registers[186]=(signed char)(hmi_plazma[2]>>8);			//���94	���������� ���������� ��� ������ 
modbus_registers[187]=(signed char)(hmi_plazma[2]);
modbus_registers[188]=(signed char)(hmi_plazma[3]>>8);			//���95	���������� ���������� ��� ������ 
modbus_registers[189]=(signed char)(hmi_plazma[3]);
modbus_registers[190]=(signed char)(hmi_plazma[4]>>8);			//���96	���������� ���������� ��� ������ 
modbus_registers[191]=(signed char)(hmi_plazma[4]);
modbus_registers[192]=(signed char)(hmi_plazma[5]>>8);			//���97	���������� ���������� ��� ������ 
modbus_registers[193]=(signed char)(hmi_plazma[5]);
modbus_registers[194]=(signed char)(hmi_plazma[6]>>8);			//���98	���������� ���������� ��� ������ 
modbus_registers[195]=(signed char)(hmi_plazma[6]);
modbus_registers[196]=(signed char)(hmi_plazma[7]>>8);			//���99	���������� ���������� ��� ������ 
modbus_registers[197]=(signed char)(hmi_plazma[7]);
modbus_registers[198]=(signed char)(hmi_plazma[8]>>8);			//���100	���������� ���������� ��� ������ 
modbus_registers[199]=(signed char)(hmi_plazma[8]);

															//��������������
modbus_registers[200]=(signed char)(cntrl_stat>>8);				//���101	���
modbus_registers[201]=(signed char)(cntrl_stat);
modbus_registers[202]=(signed char)(u_necc>>8);					//���102	���������� �����������
modbus_registers[203]=(signed char)(u_necc);
modbus_registers[204]=(signed char)(IZMAX_>>8);					//���103	��� ������ ������� ������������
modbus_registers[205]=(signed char)(IZMAX_);
modbus_registers[206]=(signed char)(bps_I>>8);					//���104	��������� ��� ���
modbus_registers[207]=(signed char)(bps_I);



///*/
/*
tempS=cntrl_stat_old;
if(	(main_kb_cnt==(TBAT*60)-21) || (main_kb_cnt==(TBAT*60)-20) || (main_kb_cnt==(TBAT*60)-19)) tempS=((short)TBAT)|0x4000;
//tempS=0x800f;
modbus_registers[198]=(signed char)(tempS>>8);				//???100	????????? ???
modbus_registers[199]=(signed char)(tempS);

tempS=t_ext[0];
if(ND_EXT[0])tempS=-1000;
modbus_registers[400]=(signed char)(tempS>>8);				//���201	������� ������ ����������� �1
modbus_registers[401]=(signed char)(tempS);
tempS=t_ext[1];
if(ND_EXT[1])tempS=-1000;
modbus_registers[402]=(signed char)(tempS>>8);				//���202	������� ������ ����������� �2
modbus_registers[403]=(signed char)(tempS);
tempS=t_ext[2];
if(ND_EXT[2])tempS=-1000;
modbus_registers[404]=(signed char)(tempS>>8);				//���203	������� ������ ����������� �3
modbus_registers[405]=(signed char)(tempS);
tempS=t_ext[3];
if(ND_EXT[3])tempS=-1000;
modbus_registers[406]=(signed char)(tempS>>8);				//���204	������� ������ ����������� �4
modbus_registers[407]=(signed char)(tempS);   */

///*/
/*
tempS=0;
if(sk_stat[0]==ssON) tempS|=0x0001;
if(sk_av_stat[0]==sasON) tempS|=0x0002;
modbus_registers[420]=(signed char)(tempS>>8);				//���211	���������  ������ �������� �1, (������� ��� - ���������� ���������, 1 - �������, 0 - ���������, ������ ��� - �����������, 1 - ������, 0 - �����)
modbus_registers[421]=(signed char)(tempS);
tempS=0;
if(sk_stat[1]==ssON) tempS|=0x0001;
if(sk_av_stat[1]==sasON) tempS|=0x0002;
modbus_registers[422]=(signed char)(tempS>>8);				//���212	���������  ������ �������� �1, (������� ��� - ���������� ���������, 1 - �������, 0 - ���������, ������ ��� - �����������, 1 - ������, 0 - �����)
modbus_registers[423]=(signed char)(tempS);
tempS=0;
if(sk_stat[2]==ssON) tempS|=0x0001;
if(sk_av_stat[2]==sasON) tempS|=0x0002;
modbus_registers[424]=(signed char)(tempS>>8);				//���213	���������  ������ �������� �1, (������� ��� - ���������� ���������, 1 - �������, 0 - ���������, ������ ��� - �����������, 1 - ������, 0 - �����)
modbus_registers[425]=(signed char)(tempS);
tempS=0;
if(sk_stat[3]==ssON) tempS|=0x0001;
if(sk_av_stat[3]==sasON) tempS|=0x0002;
modbus_registers[426]=(signed char)(tempS>>8);				//���214	���������  ������ �������� �1, (������� ��� - ���������� ���������, 1 - �������, 0 - ���������, ������ ��� - �����������, 1 - ������, 0 - �����)
modbus_registers[427]=(signed char)(tempS);
*/
//modbus_registers[


if(prot==MODBUS_RTU_PROT)
	{
	modbus_tx_buff[0]=adr;
	modbus_tx_buff[1]=func;

	modbus_tx_buff[2]=(char)(reg_quantity*2);

	memcpy((signed char*)&modbus_tx_buff[3],(signed char*)&modbus_registers[(reg_adr-1)*2],reg_quantity*2);

	crc_temp=CRC16_2(modbus_tx_buff,(reg_quantity*2)+3);

	modbus_tx_buff[3+(reg_quantity*2)]=(char)crc_temp;
	modbus_tx_buff[4+(reg_quantity*2)]=crc_temp>>8;

//	plazma_uart1[3]=reg_quantity;

	for (i=0;i<(5+(reg_quantity*2));i++)
		{
		putchar1(modbus_tx_buff[i]);
		}
	}
else if(prot==MODBUS_TCP_PROT)
	{
	//*/mem_copy((signed char*)modbus_tx_buff,(signed char*)&modbus_registers[(reg_adr-1)*2],reg_quantity*2);
///*/	modbus_tcp_out_ptr=(signed char*)modbus_tx_buff;
	}
}


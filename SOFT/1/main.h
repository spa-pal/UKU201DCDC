
#define delay_ms(x) {long xx; xx=(unsigned long)x * 12000UL; while(xx)xx--;}
#define delay_us(x) {long xx; xx=(unsigned long)x * 12UL; while(xx)xx--;}

#define MASK(lengt) 		(0xffffffff>>(32-lengt))
#define MASK_OFFSET(shift,lengt)	(MASK(lengt)<<shift)

#define GET_REG( reg, shift, lengt) 		( (reg & MASK_OFFSET(shift,lengt)) >> shift)
#define SET_REG( reg, val, shift, lengt)  	reg = ( (reg & ~MASK_OFFSET(shift,lengt)) | (val << shift) )

#define BIN__N(x) (x) | x>>3 | x>>6 | x>>9
#define BIN__B(x) (x) & 0xf | (x)>>12 & 0xf0
#define BIN8(v) (BIN__B(BIN__N(0x##v)))

//***********************************************
//��������
extern unsigned char t0cnt0, t0cnt1, t0cnt2, t0cnt3, t0cnt4, t0cnt5; 
extern bool b1000Hz, b100Hz, b50Hz, b1Hz, b10Hz, b5Hz, b2Hz;
extern bool bFL, bFL2, bFL5;
extern signed short main_10Hz_cnt;
extern signed short main_1Hz_cnt;
extern short main_1HZ_cnt;

//***********************************************
//������ �� EEPROM
extern signed short ICA_MODBUS_ADDRESS;//����� �������� ��� ������������ ����� �� ���� MODBUS-RTU
extern signed short MODBUS_ADRESS;
extern signed int MODBUS_BAUDRATE;

//***********************************************
//��������� ��������� ����
extern signed short net_U,net_Ustore,net_Ua,net_Ub,net_Uc, net_Umax;
extern char bFF,bFF_;
extern signed short net_F,hz_out,hz_out_cnt,net_F3;
extern signed char unet_drv_cnt;	//������� �� �������� ���������� ��������
extern signed char unet_max_drv_cnt; //������� �� ���������� ���������� ��������
extern char net_av;			//����������� �������� ���� 0 - �����, 1 - ��������, 2 - ��������

//***********************************************
//��������� �������
typedef struct
	{
	char 		_cnt_to_block;
	signed short	_Ub;
	signed short	_Ubm;
	signed short	_dUbm;
	signed short	_Ib;
	signed short	_Tb;
	char 		_nd;
	char 		_cnt_wrk;
	char 		_wrk;
	unsigned short _zar;
	char 		_full_ver;
	signed long 	_zar_cnt;
	signed long 	_zar_cnt_ke;
	unsigned short _Iintegr,_Iintegr_; 
	signed short 	_u_old[8];
	signed short	_u_old_cnt;
	unsigned long 	_wrk_date[2];
	char 		_rel_stat;
	char			_av;
	char			_time_cnt;
	char 		_temper_stat;
	//0��� - ��������
	//1��� - ��������
	signed short 	_sign_temper_cnt;
	signed short 	_max_temper_cnt;
	signed long 	_resurs_cnt;
	signed short 	_cnt_as; 	//������� �����������, ������� �� 5 ����� ��� ���������� ������� �����������, ����� ����������� - ����� � ������
	//signed short   _max_cell_volt;
	//signed short   _min_cell_volt;
	unsigned short _time_min_cnt_ke;
	} BAT_STAT; 
extern BAT_STAT bat[2],bat_ips;
extern signed short		bat_u_old_cnt;
extern signed short 	Ib_ips_termokompensat;
extern signed short		Ib_ips_termokompensat_temp;


//***********************************************
//��������� ����������
typedef struct
    {
	enum {dSRC=3,dINV=5,dNET_METR=7,dIBAT_METR=9,dMAKB=11}_device;
	char _av;
	//0��� - ������ �� ���������
	//1��� - ������ �� ����������� U���
	//2��� - ������ �� ����������� U���
	//3��� - ������ �� ������ �����	
	//4��� - ������ ����������� ���������(��������������) 
	//5��� - ����������� ����������� ��������� ���������� �����(��������������)   
 	enum {bsAPV,bsWRK,bsRDY,bsBL,bsAV=33,bsOFF_AV_NET}_state;
    char _cnt;
	char _cnt_old;
	char _cnt_more2;
	char _buff[20]; 
	signed _Uii; 
	signed _Uin;
	signed _Ii;
	signed _Ti; 
	char _flags_tu;
	signed _vol_u;
	signed _vol_i;
	char _is_on_cnt;
	int _ist_blok_host_cnt;
	short _blok_cnt; //������������ ���������� 
	char _flags_tm;
	signed short _overload_av_cnt;     
	signed short _temp_av_cnt;
	signed short _umax_av_cnt;
	signed short _umin_av_cnt;		  //������� ������ �� ����������� ���������� ����� ��� ��� ������� ��� ������ 
	signed short _umin_av_cnt_uku;	  //������� ������ �� ����������� ���������� ����� ��� ����� �������� ���������� �� ������ ���
	signed short _temp_warn_cnt;	  //������� �������������� �� ���������� �����������
	signed _rotor;
	signed  short _x_; 
	char _adr_ee;
	char _last_avar;
	char _vent_resurs_temp[4];
	unsigned short _vent_resurs;
	unsigned char _apv_timer_1_lev;		//������ ��� 1-�� ������, ������� ������
	unsigned char _apv_cnt_1_lev;		//������� ��� 1-�� ������, ������� 3 �������
	unsigned short _apv_timer_2_lev;	//������ ��� 2-�� ������, ������� ������������� ��� ���2 ������
	unsigned char _apv_reset_av_timer;	//������ ��� ������ ������ ����(���� �� ��������� �� ��� ������ ������ ��������)
	unsigned char _apv_succes_timer;	//������ �������� ������� �������� ������ ���, ��� ���������� ������ ���������� ��� 

	} BPS_STAT; 
extern BPS_STAT bps[40];

//***********************************************
//��������� ������
extern signed short bps_U;
extern signed short out_U;
extern signed short out_I;
extern signed short bps_I;
extern signed short bps_I_average;

//***********************************************
//��������� ��������
extern signed short load_U;
extern signed short load_I;

//***********************************************
//���������� �����
extern signed short speedChrgCurr;			//������������ ��� ����������� ������, ����������� �� ������
extern signed short speedChrgVolt;			//������������ ���������� ����������� ������, ����������� �� ������
extern signed short speedChrgTimeInHour; 	//������������ ����� ����������� ������ � �����, ����������� �� ������
extern signed short speedChrgAvtEn;	 	//�������������� ��������� ����������� ������ ��������/���������
extern signed short speedChrgDU;	    		//�������� ���������� ����������� ��� ��������� ����������� ������
extern signed short speedChIsOn;			//������� ��������� ����������� ������ ���/����
extern signed long  speedChTimeCnt;		//������� ������� ������ ����������� ������
//extern signed short speedChrgBlckSrc;		//�������� ������� ����������, 0-����., 1-��1, 2-��2
//extern signed short speedChrgBlckLog;		//������ ������� ����������, 1 - ���������� �� ���������� ��, 0 - �� ������������
extern signed short speedChrgBlckStat;		//������ ���������� ��� �������������� � ����������� ������.
extern char  		speedChrgShowCnt;		//������� ������ ��������������� ���������
extern signed short SP_CH_VENT_BLOK;		//���������� ����������� ������ ����������� (0-����, 1-����������, 2-�����������)
extern char spch_plazma[2];

//***********************************************
//����� ���������� �����
typedef enum  {scsOFF,scsSTEP1,scsWRK,scsERR1,scsERR2} enum_sp_ch_stat;
extern enum_sp_ch_stat sp_ch_stat,sp_ch_stat_old;
extern short sp_ch_stat_cnt;
extern long sp_ch_wrk_cnt;
extern char speedChargeStartCnt;
/*
//***********************************************
//�������� ��������� ����������
extern signed short outVoltContrHndlCnt;		//�������, ������� � ���� � ������ ���������� ������� ������
extern signed short outVoltContrHndlCnt_;		//�������, ������� � ���� � ������ ���������� ���������� ������� ������
extern char uout_av; */

//***********************************************
//����������� �����
#define CNT_SRC_MAX	60
extern signed short cnt_net_drv;
extern char max_net_slot;
extern bool bCAN_OFF;

//***********************************************
//�����������
typedef enum {spcOFF=0,spcKE, spcVZ}enum_spc_stat;
typedef enum {kssNOT=0,kssNOT_VZ,kssYES=100,kssNOT_BAT,kssNOT_BAT_AV,kssNOT_BAT_AV_T,kssNOT_BAT_AV_ASS,kssNOT_BAT_ZAR,kssNOT_BAT_RAZR,kssNOT_KE1,kssNOT_KE2}enum_ke_start_stat;
extern enum_spc_stat spc_stat;
extern enum_ke_start_stat ke_start_stat;
extern char spc_bat;
extern char spc_phase;
extern unsigned short vz_cnt_s,vz_cnt_s_,vz_cnt_h,vz_cnt_h_;
extern short cnt_end_ke;
extern unsigned long ke_date[2];
extern short __ee_vz_cnt;
extern short __ee_spc_stat;
extern short __ee_spc_bat;
extern short __ee_spc_phase;
extern char vz_error;   // ���������������, ���� ���. ����� ������������


//***********************************************
//������
//extern unsigned avar_stat;	 	//"�����������" ���� ��������� � ������ ������ ��������� � ����� �����
extern unsigned avar_ind_stat; 	//"�����������" ���� �� ������������� ��������� ��������� � ����� �����
extern unsigned avar_stat_old;
extern unsigned avar_stat_new,avar_stat_offed;
//��������� ����������
//1���  - �������� ����
//2���� - �������
//12��� - ����
//5���  - ���������
//4���� - ������� ������� �����������
//4���� - ������� ����� ��������
//1���	- �������� ��������� ����������

//**********************************************
//������������, ������������ �� EEPROM
extern signed short Ktsrc[2];
extern signed short Kusrc[2];
extern signed short Kisrc[2];
extern signed short Ki0src[2];
extern signed short Kubat[2];
extern signed short Kubatm[2];
extern unsigned short Kibat0[2];
extern signed short Kibat1[2];
extern signed short Ktbat[2];
extern signed short Kunet;
extern signed short Ktext[3];
extern signed short Kuload;
extern signed short Kunet_ext[3];
extern signed short KunetA;
extern signed short KunetB;
extern signed short KunetC;
extern signed short Kubps;
extern signed short Kuout;

extern signed short MAIN_IST;
extern signed short UMAX;
extern signed short UDCDC;
extern signed short UB20;
extern signed short TMAX;
extern signed short TSIGN;
extern signed short AV_OFF_AVT;
extern signed short USIGN;
extern signed short UMN;
extern signed short UMAXN;
extern signed short ZV_ON;
extern signed short IKB;
extern signed short UVZ;
extern signed short IMAX_VZ;
extern signed short IMAX;
extern signed short IMIN;
extern signed short APV_ON;
extern signed short IZMAX;
extern signed short U0B;
extern signed short TZAS;
extern signed short VZ_HR;
extern signed short TBAT;
extern signed short U_AVT;
extern signed short DU;
extern signed short PAR;
extern signed short TBATMAX;
extern signed short TBATSIGN;
extern signed short UBM_AV;
extern signed short RELE_LOG;
extern signed short TBOXMAX;
extern signed short TBOXREG;
extern signed short TBOXVENTMAX;
extern signed short TLOADDISABLE;
extern signed short TLOADENABLE;
extern signed short TBATDISABLE;
extern signed short TBATENABLE;
extern signed short TBOXMAX;
extern signed short TBOXREG;
extern signed short TBOXVENTMAX;
extern signed short TLOADDISABLE;
extern signed short TLOADENABLE;
extern signed short TBATDISABLE;
extern signed short TBATENABLE;
extern signed short TVENTON;
extern signed short TVENTOFF;
extern signed short TWARMON;
extern signed short TWARMOFF;
typedef enum {rvsAKB=0,rvsEXT,rvsBPS} enum_releventsign;
extern enum_releventsign RELEVENTSIGN;
extern signed short TZNPN;
extern signed short UONPN;
extern signed short UVNPN;
extern signed short dUNPN;
typedef enum {npnoOFF=0,npnoRELEVENT,npnoRELEAVBAT2} enum_npn_out;
extern enum_npn_out NPN_OUT;
typedef enum {npnsULOAD=0,npnsAVNET} enum_npn_sign;
extern enum_npn_sign NPN_SIGN;
extern signed short TERMOKOMPENS;
extern signed short TBOXVENTON; 
extern signed short TBOXVENTOFF;
extern signed short TBOXWARMON; 
extern signed short TBOXWARMOFF;
extern signed short BAT_TYPE;	//��� �������. 0 - ������� ���������, 1-�������� COSLIGHT, 2-�������� SACRED SUN , 3-�������� ZTT
extern signed short DU_LI_BAT;	//��������, ������������ ���������� ���������� �������� �������
extern signed short FORVARDBPSCHHOUR;	//������������������ �������� ��������� � �����. ���� 0 - ������� ��������� � ������� ������ ��������
extern signed short NUMBAT;
extern signed short NUMBAT_TELECORE;
extern signed short NUMIST;
extern signed short NUMINV;
extern signed short NUMDT;
extern signed short NUMSK;
extern signed short NUMEXT;
extern signed short NUMAVT;
extern signed short NUMMAKB;
extern signed short NUMBYPASS;
extern signed short NUMBDR;
extern signed short NUMENMV;
extern signed short NUMPHASE;  
extern signed short SMART_SPC;
extern signed short U_OUT_KONTR_MAX;
extern signed short U_OUT_KONTR_MIN;
extern signed short U_OUT_KONTR_DELAY;
extern signed short DOP_RELE_FUNC;
extern signed short CNTRL_HNDL_TIME;	//���������� ������� ������������� ���������� ��� ��������
extern signed short USODERG_LI_BAT;		//���������� ���������� �������� �������
extern signed short QSODERG_LI_BAT;		//����� ��� ������� �������� ����������� ���������� ���������� �������� �������
extern signed short TVENTMAX;			//������������ ������ �����������
extern signed short ICA_EN;				//������������ ������ ������������ ����� ���
extern signed short ICA_CH;				//����� ����� ��� ������������ �����, 0 - MODBUS, 1 - MODBUS-TCP
extern signed short ICA_MODBUS_ADDRESS;//����� �������� ��� ������������ ����� �� ���� MODBUS-RTU
extern signed short ICA_MODBUS_TCP_IP1,ICA_MODBUS_TCP_IP2,ICA_MODBUS_TCP_IP3,ICA_MODBUS_TCP_IP4;	//IP �������� ��� ������������ ����� �� ���� MODBUS-TCP
extern signed short ICA_MODBUS_TCP_UNIT_ID;	//UNIT ID �������� ��� ������������ ����� �� ���� MODBUS-TCP
extern signed short PWM_START;			//��������� ��� ��� ������
extern signed short KB_ALGORITM;		//2-� ��� 3-� ������������ �������� �������� ���� �������
extern signed short REG_SPEED;			//�������� �������������, 1- �����������, 2,3,4,5- ����������� � 2,3,4,5 ���

typedef enum {apvON=0x01,apvOFF=0x00}enum_apv_on;
extern enum_apv_on APV_ON1,APV_ON2;

extern signed short APV_ON2_TIME;
extern signed short RS485_QWARZ_DIGIT;
extern signed short UVENTOFF;			//���������� (1�) ��� ������� ���������� ���������� ����� ��������� �� ��� ����
extern signed short VZ_KIND;			//��� �������������� ������, 0 - �������(������������, ��������� ���������� �� �����), 
										//1- ��������������, � ��������� ���������� � ��������� � ���������
extern signed short SNTP_ENABLE;
extern signed short SNTP_GMT;

extern signed short RELE1SET;			//��������� ������������ ����1
extern signed short RELE2SET;			//��������� ������������ ����2, �������� ����� ��� � � ����1
extern signed short RELE3SET;		   	//��������� ������������ ����3, �������� ����� ��� � � ����1

extern signed short UZ_U;
extern signed short UZ_IMAX;
extern signed short UZ_T;

extern signed short FZ_U1;
extern signed short FZ_IMAX1;
extern signed short FZ_T1;
extern signed short FZ_ISW12;
extern signed short FZ_U2;
extern signed short FZ_IMAX2;
extern signed short FZ_T2;

//extern signed short RELE_SET_MASK[4];

typedef enum {bisON=0x0055,bisOFF=0x00aa}enum_bat_is_on;
extern enum_bat_is_on BAT_IS_ON[2];

extern signed short BAT_DAY_OF_ON[2];
extern signed short BAT_MONTH_OF_ON[2];
extern signed short BAT_YEAR_OF_ON[2];
extern signed short BAT_C_NOM[2];
extern signed short BAT_RESURS[2];
extern signed short BAT_C_REAL[2];
//extern signed short BAT_TYPE[2];

extern unsigned short AUSW_MAIN;
extern unsigned long AUSW_MAIN_NUMBER;
extern unsigned short AUSW_DAY;
extern unsigned short AUSW_MONTH;
extern unsigned short AUSW_YEAR;
extern unsigned short AUSW_UKU;
extern unsigned short AUSW_UKU_SUB;
extern unsigned long AUSW_UKU_NUMBER;
extern unsigned long	AUSW_BPS1_NUMBER;
extern unsigned long  AUSW_BPS2_NUMBER;
extern unsigned short AUSW_RS232;
extern unsigned short AUSW_PDH;
extern unsigned short AUSW_SDH;
extern unsigned short AUSW_ETH;

extern signed short TMAX_EXT_EN[3];
extern signed short TMAX_EXT[3];
extern signed short TMIN_EXT_EN[3];
extern signed short TMIN_EXT[3];
extern signed short T_EXT_REL_EN[3];
extern signed short T_EXT_ZVUK_EN[3];
extern signed short T_EXT_LCD_EN[3];
extern signed short T_EXT_RS_EN[3];

extern signed short SK_SIGN[4];
extern signed short SK_REL_EN[4];
extern signed short SK_ZVUK_EN[4];
extern signed short SK_LCD_EN[4];
extern signed short SK_RS_EN[4];

typedef enum {AVZ_1=1,AVZ_2=2,AVZ_3=3,AVZ_6=6,AVZ_12=12,AVZ_OFF=0}enum_avz;
extern enum_avz AVZ;

extern unsigned short HOUR_AVZ;
extern unsigned short MIN_AVZ;
extern unsigned short SEC_AVZ;
extern unsigned short DATE_AVZ;
extern unsigned short MONTH_AVZ;
extern unsigned short YEAR_AVZ;
extern unsigned short AVZ_TIME;
typedef enum {mnON=0x55,mnOFF=0xAA}enum_mnemo_on;
extern enum_mnemo_on MNEMO_ON;
extern unsigned short MNEMO_TIME;
extern signed short POWER_CNT_ADRESS;

extern signed short ETH_IS_ON;
extern signed short ETH_DHCP_ON;
extern signed short ETH_IP_1;
extern signed short ETH_IP_2;
extern signed short ETH_IP_3;
extern signed short ETH_IP_4;
extern signed short ETH_MASK_1;
extern signed short ETH_MASK_2;
extern signed short ETH_MASK_3;
extern signed short ETH_MASK_4;
extern signed short ETH_TRAP1_IP_1;
extern signed short ETH_TRAP1_IP_2;
extern signed short ETH_TRAP1_IP_3;
extern signed short ETH_TRAP1_IP_4;
extern signed short ETH_TRAP2_IP_1;
extern signed short ETH_TRAP2_IP_2;
extern signed short ETH_TRAP2_IP_3;
extern signed short ETH_TRAP2_IP_4;
extern signed short ETH_TRAP3_IP_1;
extern signed short ETH_TRAP3_IP_2;
extern signed short ETH_TRAP3_IP_3;
extern signed short ETH_TRAP3_IP_4;
extern signed short ETH_TRAP4_IP_1;
extern signed short ETH_TRAP4_IP_2;
extern signed short ETH_TRAP4_IP_3;
extern signed short ETH_TRAP4_IP_4;
extern signed short ETH_TRAP5_IP_1;
extern signed short ETH_TRAP5_IP_2;
extern signed short ETH_TRAP5_IP_3;
extern signed short ETH_TRAP5_IP_4;
extern signed short ETH_SNMP_PORT_READ;
extern signed short ETH_SNMP_PORT_WRITE;
extern signed short ETH_GW_1;
extern signed short ETH_GW_2;
extern signed short ETH_GW_3;
extern signed short ETH_GW_4;

extern signed short RELE_VENT_LOGIC;

extern signed short BAT_LINK;

extern signed short BAT_C_POINT_1_6;  	//������� ������� ��� ������� 1/6 ����
extern signed short BAT_C_POINT_1_2;  	//������� ������� ��� ������� 1/2 ����
extern signed short BAT_C_POINT_1;		//������� ������� ��� ������� 1 ���
extern signed short BAT_C_POINT_3;		//������� ������� ��� ������� 3 ����
extern signed short BAT_C_POINT_5;		//������� ������� ��� ������� 5 �����
extern signed short BAT_C_POINT_10;		//������� ������� ��� ������� 10 �����
extern signed short BAT_C_POINT_20;		//������� ������� ��� ������� 20 �����
extern signed short BAT_U_END_1_6;  	//�������� ���������� ������� ��� ������� 1/6 ����
extern signed short BAT_U_END_1_2;  	//�������� ���������� ������� ��� ������� 1/2 ����
extern signed short BAT_U_END_1;  		//�������� ���������� ������� ��� ������� 1 ���
extern signed short BAT_U_END_3;  		//�������� ���������� ������� ��� ������� 3 ����
extern signed short BAT_U_END_5;  		//�������� ���������� ������� ��� ������� 5 �����
extern signed short BAT_U_END_10;  		//�������� ���������� ������� ��� ������� 10 �����
extern signed short BAT_U_END_20;  		//�������� ���������� ������� ��� ������� 20 �����
extern signed short BAT_C_POINT_NUM_ELEM;	//���������� ��������� � �������
extern signed short BAT_K_OLD;			//����������� �������� �������

extern signed short SP_CH_VENT_BLOK;
extern signed short VZ_CH_VENT_BLOK;

//**********************************************
//��������� ��� �� ����� ��������� ���� �������
extern signed long ibat_metr_buff_[2];
extern short bIBAT_SMKLBR;
extern short bIBAT_SMKLBR_cnt;
extern short ibat_metr_cnt;

//**********************************************
//���������� ��������������
extern signed short Isumm;
extern signed short Isumm_;

//**********************************************
//������������� ��������� ��������
extern short factory_settings_hndl_main_iHz_cnt;
extern char factory_settings_led_reg0, factory_settings_led_reg1;

//*************************************************
//���������
#define MESS_DEEP	10

#define 	MESS_ZERO 		0
#define 	MESS_BAT1_OFF 		1
#define 	MESS_BAT2_OFF		2
#define 	MESS_ALL_SRC_OFF	3
#define 	MESS_ALL_SRC_ON	4
//#define 	MESS_RELSAM_ON		5
#define 	MESS_SRC1_OFF		6
#define 	MESS_SRC2_OFF		7
#define 	MESS_SRC3_OFF		8
#define 	MESS_SRC4_OFF		9
#define 	MESS_SRC5_OFF		10
#define 	MESS_SRC6_OFF		11
#define 	MESS_SRC7_OFF		12
#define 	MESS_SRC8_OFF		13
#define 	MESS_SRC9_OFF		14
#define 	MESS_SRC10_OFF		15
#define 	MESS_SRC11_OFF		16
#define 	MESS_SRC12_OFF		17
#define 	MESS_BAT_CONTROL	18
#define 	MESS_SRC_CONTROL	19
#define 	MESS_LOAD2_WAIT	9 
#define 	MESS_PONG			100
#define	MESS_SPA_UART_PONG	101 
#define	MESS_SPA_BLOK_BPS1	102
#define	MESS_SPA_BLOK_BPS2	103
#define	MESS_SPA_LEAVE_BPS1	104
#define	MESS_SPA_LEAVE_BPS2	105
#define 	MESS_SRC_ON_OFF	150
#define   _MESS_SRC_MASK_BLOK_2SEC		151
#define   _MESS_SRC_MASK_UNBLOK		152
#define 	_MESS_SRC_MASK_ON			153
#define	_MESS_SRC_PWM				154
#define	_MESS_U_NECC				155
#define   _MESS_FAST_REG				156
//#define   _MESS_U_AVT_GOOD			157

#define 	MESS_BAT_ON_OFF	160
#define   _MESS_BAT_MASK_BLOK_AFTER_2SEC		161
#define	_MESS_BAT_MASK_ON					162
#define	_MESS_BAT_MASK_OFF					163

		
//#define	MESS_SPA_UART_SRAM	200 
//#define	MESS_SPA_UART_CMND	201 

#define	MESS2UNECC_HNDL   					190
#define  		PARAM_UNECC_SET				 	191
#define	MESS2BAT_HNDL   					200
//#define	MESS2BAT_HNDL1   					201
#define		PARAM_BAT_ALL_OFF_AFTER_2SEC			201
#define		PARAM_BAT_MASK_OFF_AFTER_2SEC			202
//#define		PARAM_BAT_ON						202
#define	MESS2BPS_HNDL   					205
#define		PARAM_BPS_ALL_OFF_AFTER_2SEC			206
#define		PARAM_BPS_MASK_OFF_AFTER_2SEC			207
#define		PARAM_BPS_MASK_ON_OFF_AFTER_2SEC		208
#define		PARAM_BPS_MASK_ON					209
#define		PARAM_BPS_ALL_ON					210
#define 	MESS2RELE_HNDL						210
#define 	MESS2KLIMAT_CNTRL					211
#define		PARAM_RELE_SAMOKALIBR				100
#define		PARAM_RELE_AV						101
#define		PARAM_RELE_AV_NET					102
#define		PARAM_RELE_AV_BAT					103
#define		PARAM_RELE_LOAD_OFF					103
#define		PARAM_RELE_LIGHT					104
//#define		PARAM_RELE_WARM					104
#define		PARAM_RELE_AV_COMM					105
#define		PARAM_RELE_AV_BPS					106
#define		PARAM_RELE_VENT						107
#define		PARAM_RELE_VENT_WARM				107
#define		PARAM_RELE_AV_BAT1					108
#define		PARAM_RELE_AV_BAT2					109
#define		PARAM_RELE_NPN						110
#define		PARAM_RELE_WARM				     	111
#define 		PARAM_RELE_VVENT				112
#define 		PARAM_RELE_EXT					113
#define 		PARAM_RELE_BAT_IS_DISCHARGED	114
#define		PARAM_KLIMAT_CNTRL_VENT_INT			115
#define		PARAM_KLIMAT_CNTRL_VENT_EXT			116
#define		PARAM_RELE_BDR1						117
#define		PARAM_RELE_BDR2						118
#define		PARAM_RELE_BDR3						119
#define		PARAM_RELE_BDR4						120

#define	MESS2IND_HNDL						215
#define		PARAM_SAMOKALIBR					216
#define 		PARAM_U_AVT_GOOD					217
#define	MESS2MATEMAT						220
#define		PARAM_SAMOKALIBR					216
#define	MESS2CNTRL_HNDL   					225
#define		PARAM_CNTRL_STAT_PLUS				100
#define		PARAM_CNTRL_STAT_MINUS				105
#define		PARAM_CNTRL_STAT_STEP_DOWN			110
#define 		PARAM_CNTRL_STAT_SET		    		229
#define 		PARAM_CNTRL_STAT_FAST_REG		    	230	
#define	MESS2KB_HNDL   					230
#define		PARAM_CNTRL_IS_DOWN					231
#define	MESS2VENT_HNDL   					240
#define		PARAM_VENT_CB					241

#define MESS2NET_DRV							33
#define	PARAM_BPS_NET_OFF						34

//***********************************************
//���������� ����
extern char rele_output_stat;
//0 ���� -> "1" ���� 1 ��� ���
//1 ���� -> "1" ���� 2 ��� ���
//2 ���� -> "1" ���� 3 ��� ���
//3 ���� -> "1" ���� HV ��� ���

//***********************************************
//���������� ��������� ����������
extern unsigned short test_control_register,test_control_register_old;
extern char rele_output_stat_test_byte;
extern char rele_output_stat_test_mask;
extern char tst_hndl_cnt;
extern char test_hndl_rele1_cntrl,test_hndl_rele1_cnt;
extern char test_hndl_rele2_cntrl,test_hndl_rele2_cnt;
extern char test_hndl_rele3_cntrl,test_hndl_rele3_cnt;
extern char test_hndl_releHV_cntrl,test_hndl_releHV_cnt;
extern char test_hndl_bps_number;
extern char test_hndl_bps_state;
extern short test_hndl_bps_cnt;
extern char zv_test_cnt,zv_test_sign;
extern char test_led_stat, test_led_cnt;

//***********************************************
//���������� ������������
extern char ledUOUTGOOD;	//������� ��������� "�������� ���������� � �����"
extern char ledWARNING;  	//������ ��������� "������� � ����� �� ���������"
extern char ledERROR;		//������� ��������� "������ � ����� �� ���������"
extern char ledCAN;	   		//������� ��������� "����� �� ��� � �����"

//***********************************************
//������ ������� � ������
extern unsigned short log_buff_mb[16];
extern unsigned short log_deep_mb;
extern unsigned short log_cmd_mb;
extern unsigned short log_debug0_mb;
extern unsigned short log_debug1_mb;

//***********************************************
//���������� ��� �������
extern unsigned short hmi_cntrl_reg, hmi_cntrl_reg_old; 	//������� �������/����������� ��������� �� ������
extern unsigned short hmi_cntrl_reg_new;					//�������� ����� ����� � ������ ���������� ����������
extern unsigned short hmi_cntrl_fb_reg; 	  				//������� �������� ����� �������/����������� ��������� �� ������
extern unsigned short hmi_notconnect_cnt;					//������� ������ ����� � HMI, ��������� ������ 0,1� �� HMINOTCONNECTMAX, ���������� �� ������ � 1 ������� 3 ��������
extern unsigned short hmi_avg_reg;							//������� ������������ ����� �� ������
extern unsigned short hmi_unecc_reg;						//������� ���������� ����������� �� ������
extern unsigned short hmi_izmax_reg;						//������� ������������� ���� ������ �� ������
//***********************************************
//***********************************************
//***********************************************
//***********************************************
//�������
extern char plazma;
extern char plazma_uart1[5];
extern short plazma_short;
extern char plazma_debug_0,plazma_debug_1;
extern short hmi_plazma[9];


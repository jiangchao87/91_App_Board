
#include <hidef.h> 			// for EnableInterrupts macro  

#include "derivative.h" 	// include peripheral declarations  
#include "ic_washer.h"		// IC���йص�����

#include "zdevice.h"		// ��ͷ�ļ�����û����

void   Delay_us(uchar us);
void   Delay_ms10(uchar ms10);
void   MCU_init(void);
void   uart2_send(uchar retry);

extern RZK_DEVICE_CB_t *  SERIAL1; 					// �ṹ��  ����1  ����û����

uchar  g_uart2_index = 0;							// �������
uchar  g_rcvbuf[30];               
uchar  g_rcvnum;
uchar  g_cmd_received = 0;
uchar  g_rcvbcc;

#define  SENDBUFLEN  24              
uchar  g_sendbuf[SENDBUFLEN];           
uchar  g_sendnum;                       

uchar  time_s,rsv_AA;        
uchar  time_ms10;                 
uchar  card_ms10;                      

#include "500.h"	 					 	  			// ��Ƶ������ͷ�ļ�
uchar  card_find = 0;


// �¶ȴ��������
uint  temp_mv_yuanlai[138] =
{ 
	984, 981, 979, 977, 974, 971, 968, 965, 962, 959, 	// -31//-39��-30
	955, 951, 947, 943, 939, 935, 931, 926, 922, 917, 	// -29��-21
	912,                                              	// -20
	907, 901, 895, 890, 884, 877, 871, 865, 858, 852, 	// -19��-10
	845, 837, 830, 822, 815, 807, 798, 790, 782, 774, 	// -9��0
	759, 756, 748, 739, 729, 720, 711, 701, 692, 682, 	// 1-10
	672, 662, 653, 643, 633, 623, 613, 603, 593, 583, 	// 11-20
	572, 562, 553, 543, 533, 523, 513, 503, 494, 484, 	// 21-30
	474, 465, 456, 446, 437, 428, 419, 410, 401, 393, 	// 31-40
	384, 376, 367, 359, 351, 343, 335, 328, 320, 313, 	// 41-50
	306, 299, 292, 285, 278, 272, 265, 259, 252, 246, 	// 51-60
	241, 235, 229, 224, 219, 213, 208, 203, 198, 193, 	// 61-70
	189, 184, 179, 175, 171, 167, 163, 159, 155, 151, 	// 71-80
	147, 144, 140, 137, 134, 0,   0
};

uint  temp_mv[] =                                           // 2012 0727 ��������
{ 
	960 ,959 ,957 ,955 ,953 ,951 ,949 ,946 ,944 ,941 ,		// -40��-31	
	938 ,935 ,932 ,928 ,924 ,921 ,916 ,912 ,908 ,903 ,
	899 ,894 ,888 ,883 ,878 ,872 ,866 ,860 ,854 ,847 ,
	841 ,834 ,827 ,820 ,812 ,805 ,797 ,789 ,781 ,773 ,
	765 ,756 ,748 ,739 ,730 ,721 ,712 ,703 ,693 ,684 ,		// 0-9
	674 ,665 ,655 ,645 ,636 ,626 ,616 ,606 ,596 ,586 ,
	576 ,566 ,557 ,547 ,537 ,527 ,517 ,507 ,498 ,488 ,
	478 ,469 ,459 ,450 ,441 ,432 ,423 ,414 ,405 ,396 ,
	388 ,379 ,371 ,362 ,354 ,346 ,338 ,331 ,323 ,315 ,
	308 ,301 ,294 ,287 ,280 ,273 ,267 ,260 ,254 ,248 ,		// 50-59
	242 ,236 ,230 ,225 ,219 ,214 ,209 ,203 ,198 ,194 ,
	189 ,184 ,179 ,175 ,171 ,167 ,162 ,158 ,154 ,151 ,
	147 ,143 ,140 ,136 ,133 ,128 ,127 ,123 ,120 ,118 ,
	115 ,112 ,109 ,106 ,104 ,101 ,99  ,97  ,94  ,92  , 
	90 ,88 ,86 ,84 ,82 ,80 ,78 ,76 ,74 ,73 ,				// 100-109
	71 ,69 ,68 ,66 ,65 ,63 ,62 ,60 , 0,  0					// 110-119
};

#define    FINDIC         0X01
#define    FANGCAI_BJ     0X02
#define    CHANGEVOL      0X03
#define    NIGHT_BJ       0X05
#define    DAY_BJ         0X06

#define    KAIMEN         0X07
#define    LAIREN         0X08
#define    WUREN          0X09
#define    EXT_POWER_FAULT 0x10


//============================================================
#define 	IC_LED        PTGD_PTGD0  //ˢ��ָʾ

#define 	FANGCAI_PIN   PTDD_PTDD4  //���Ų�
//#define   NIGHT_PIN   PTDD_PTDD7
 
#define 	OPENLOCK      PTDD_PTDD4  //������
#define 	MENCI_PIN     PTDD_PTDD3  //�Ŵ�
#define 	KEY1_PIN      PTDD_PTDD2  //���Ű�ť 

#define     RT_PIN        PTGD_PTGD3  //  �����Ӧ


#define FANGCAI_DELAY 100
//#define NIGHT_DELAY   3
//#define DAY_DELAY     3
uchar  g_night_times = 0xff;
uchar  g_day_times = 0xff;
void   UART2_night(void);
void   UART2_day(void);

uchar  g_key1_times = 0xff;
#define KEY1_DELAY  10           

uchar alerted = 0;


void  VOL(uchar vol);
void  VOL_DEC(void);
void  VOL_INC(void);
void  UART2_ASW(uchar cmd);  	//Ӧ������

uint  cpu_reset_s = 0;

//�¶ȿ�����
//�¶ȿ��Ʒ�ʽ��ÿ40��һ�����ڣ��ڷ���0��ʱ�����ݵ�ǰ�¶ȸ��¼���ʱ�䡣�ﵽ����ʱ���ʱ���л���������
uchar g_temprature = 0;  	//��ǰ�¶�
uchar g_temprature1 = 0; 	//��ǰ�¶�1
uchar g_temprature2 = 0; 	//��ǰ�¶�2

uchar G_HEAT1 = 0;      	//cpu����ʱ��-������
uchar G_HEAT1_T = 0;    	//��ʾ������ʱ��-������ 
uchar g_heat_now = 0;  		//��ǰ����

//�ڲ���ģʽ��
uchar g_testing = 0; 		// 0-���ڲ���״̬��1-�ڲ���״̬
uchar re_menci = 0; 		// �ϴ��Ŵ�״̬
uchar sec = 0;      		// ��1�������ڲ���״̬��ʱ

//==============================================================================
uchar  rt_pinbz;			// �����Ӧ ƽʱ=0 ����400ms�ĸߵ�ƽ ���2��һ������
uint   rt_2s;				// 	
uint   rt_340;				// 
uchar  RT_FS_C1_g0,hl_chngbz;
uchar  led_rtdsp;


//*********************************************************************
// Ӧ������ 
void UART2_ASW(uchar cmd)				// Ӧ������
{
	uchar i;

	g_sendbuf[1] = 0Xff;
	g_sendbuf[2] = g_rcvbuf[2];		// ���������ı��
	g_sendbuf[3] = cmd;    				//0x01:  IC�������кţ�����Ϊ5Byte
	for (i=4; i<=22; i++)
		g_sendbuf[i] = 0x00;
	
	for (i=1; i<=21; i++)
		g_sendbuf[22] += g_sendbuf[i];
	
	g_sendbuf[23] = 0xaa;				// ��β 0xaa
    uart2_send(1);
}

//*********************************************************************
// Ҫ�� ��������
void UART2_VOL(void)  						//Ӧ������
{
	uchar i;

	g_sendbuf[1] = 0XFF;
	g_sendbuf[2] = g_rcvbuf[2];
	g_sendbuf[3] = CHANGEVOL;           	 //0x01:  IC�������кţ�����Ϊ5Byte
	for (i=4; i<=22; i++)
		g_sendbuf[i] = 0x00;
	
	for (i=1; i<=21; i++)					// 1-21�ֽ��ۼӺ�
		g_sendbuf[22] += g_sendbuf[i];
	
	g_sendbuf[23] = 0xaa;
    uart2_send(1);
}




//*********************************************************************
// �Ŵſ�
void UART2_menci_open(uchar time)
{
	uchar i;

	g_sendbuf[1] = g_uart2_index++;	// �������
	g_sendbuf[2] = g_rcvbuf[2];
	g_sendbuf[3] = 0x0a;            	//0x01:  IC�������кţ�����Ϊ5Byte
	for(i=4; i<=22; i++)
		g_sendbuf[i] = 0x00;
	
	for(i=1; i<=21; i++)
		g_sendbuf[22] += g_sendbuf[i];
	
	g_sendbuf[23] = 0xaa;
    uart2_send(time);
}

//*********************************************************************
// �ŴŹ�
void UART2_menci_close(uchar time)
{
	uchar i;
	g_sendbuf[1] = g_uart2_index++;	// �������
	g_sendbuf[2] = g_rcvbuf[2];
	g_sendbuf[3] = 0x0b;            	//0x01:  IC�������кţ�����Ϊ5Byte
	for(i=4;i<=22;i++) 
		g_sendbuf[i] = 0x00;
		
	for (i=1; i<=21; i++)
		g_sendbuf[22] += g_sendbuf[i];	
	g_sendbuf[23] = 0xaa;
    uart2_send(time);
}

//*********************************************************************
// ���� ���� ��
void UART2_renti_come(uchar time)
{
	uchar i;
	g_sendbuf[1] = g_uart2_index++;	// �������
	g_sendbuf[2] = g_rcvbuf[2];
	g_sendbuf[3] = 0x08;            	//0x01:  IC�������кţ�����Ϊ5Byte
	for(i=4;i<=22;i++) 
		g_sendbuf[i] = 0x00;			// Ӧ����0  �������ղ��� �Ÿ�6��������2012 0607
		//g_sendbuf[i] = 0x06;			// Ӧ����0  �������ղ��� �Ÿ�6��������2012 0607
		
	for(i=1; i<=21; i++)
		g_sendbuf[22] += g_sendbuf[i];	
	
	//g_sendbuf[4] = 0xDD; 
	//g_sendbuf[5] = rt_340>>8;
	//g_sendbuf[6] = rt_340&0xff;
	
	g_sendbuf[23] = 0xaa;
    uart2_send(time);
}    


//*********************************************************************
// ���� ���߿�
void UART2_renti_go(uchar time)
{
	uchar i;
	g_sendbuf[1] = g_uart2_index++;	// �������
	g_sendbuf[2] = g_rcvbuf[2];
	g_sendbuf[3] = 0x09;            	//0x01:  IC�������кţ�����Ϊ5Byte
	for(i=4;i<=22;i++) 
		g_sendbuf[i] = 0x00;
		//g_sendbuf[i] = 0x06;			// Ӧ����0  �������ղ��� �Ÿ�6��������2012 0607
		
	for (i=1; i<=21; i++)
		g_sendbuf[22] += g_sendbuf[i];	
	
	//g_sendbuf[4] = 0xee; 
	//g_sendbuf[5] = rt_2s>>8;
	//g_sendbuf[6] = rt_2s&0xff;
	
	g_sendbuf[23] = 0xaa;
    uart2_send(time);
}


//*********************************************************************
// ����
void UART2_day(void)
{
	uchar i;
	g_sendbuf[1] = g_uart2_index++;	// �������
	g_sendbuf[2] = g_rcvbuf[2];
	g_sendbuf[3] = DAY_BJ;            	//0x01:  IC�������кţ�����Ϊ5Byte
	for(i=4; i<=22; i++)
		g_sendbuf[i] = 0x00;
	
	for(i=1; i<=21; i++)
		g_sendbuf[22] += g_sendbuf[i];	
	g_sendbuf[23] = 0xaa;
    uart2_send(1);
}

//*********************************************************************
// ҹ��
void UART2_night(void)
{
	uchar i;

	g_sendbuf[1] = g_uart2_index++;	// �������
	g_sendbuf[2] = g_rcvbuf[2];
	g_sendbuf[3] = NIGHT_BJ;            //0x01:  IC�������кţ�����Ϊ5Byte
	for(i=4; i<=22; i++)
		g_sendbuf[i] = 0x00;
	
	for(i=1; i<=21; i++)
		g_sendbuf[22] += g_sendbuf[i];	
	g_sendbuf[23] = 0xaa;
	uart2_send(1);
}


void UART2_Ext_Power_Status(void)
{
	uchar i;

	g_sendbuf[1] = g_uart2_index++;	// �������
	g_sendbuf[2] = g_rcvbuf[2];
	g_sendbuf[3] = EXT_POWER_FAULT;
	for(i=4; i<=22; i++)
		g_sendbuf[i] = 0x00;
	
	for(i=1; i<=21; i++)
		g_sendbuf[22] += g_sendbuf[i];	
	g_sendbuf[23] = 0xAA;
	uart2_send(1);
}


void Check_Ext_Power_Status(void)
{
	if((PTGD_PTGD4 == 0)&&(alerted == 0)){
		Delay_ms10(20);
		if(PTGD_PTGD4 == 0){
			UART2_Ext_Power_Status();
			alerted = 1;
		}
	}
	if(PTGD_PTGD4)
		alerted = 0;
}


//==============================================
// CPU �˿ڳ�ʼ��
void port_init(void)
{
    PTAPE = 0XFF; 						//��������
    PTBPE = 0X7F;
    PTCPE = 0b00000111;
    //PTDPE = 0XFF;
    PTDPE = 0;
	PTEPE = 0;
	PTFPE = 0B00000000;
	PTGPE = 0X00;

    PTASE = 0X00;						//���ٽ�ֹ
    PTBSE = 0X00;
    PTCSE = 0X00;
    PTDSE = 0X00;
    PTESE = 0X00;
    PTFSE = 0X00;
    PTGSE = 0X00;

    PTAD = 0XFF; 						//����
    PTBD = 0XFF;
    PTCD = 0XF8;
    PTDD = 0XDF;
    PTED = 0XFF;
    PTFD = 0XFe; 						//���λ��CPU��λ��
    PTGD = 0XFF;

    PTADD = 0XFF; 						//����
    PTBDD = 0X7F; 						//B7-�¶�
    PTCDD = 0b11011100; 				//0XFF;
    PTDDD = 0XF2;						//D0-���¶ȣ�D4-Ԥ������D5-CPU����Ĥ��
    PTEDD = 0XDF;						//E5-�Լ죻����δ��
    PTFDD = 0X7F;						//F7-Ԥ����������δ��
    PTGDD = 0XE7;						//G0-ָʾ�ƣ�G4-������Ĥ��G5G6��������δ��
    
    Delay_us(20);
}
 
//ԭ������ 
void MCUinit(void)
{
	SOPT = 0XC0;
	SPMSC1 = 0X35; 				//��Դ����
	SPMSC2 = 0X30;
	ICGC1 = 0XF8;
	ICGC2 = 0X00;
	while(!ICGS1_ERCS) 
		{__RESET_WATCHDOG();}
	while(!ICGS1_LOCK) 
		{__RESET_WATCHDOG();}

	SCI2BDH = 0X00;
	SCI2BDL = 52;
	SCI2C1 = 0X00;
	SCI2C3 = 0X00;
	SCI2S2 = 0X00;
	SCI2S1 = 0X00;
	SCI2C2 = 0X2C;

	TPM2C1SC = 0;
	TPM2C0SC = 0;
	TPM2SC |= 0x40;  					//��ʱ��2: 1-TOF interrupts enabled
	TPM2SC &= 0xc0;
	TPM2SC |= 0x0c;
	TPM2MODH = 0x27;
	TPM2MODL = 0x10;
	//STHX     TPM2MODH             	;Store H:X (Index Reg.)
}

//////////////////////////////////////////////////////////////
//��ʱ1US
void Delay_us(uchar us)
{
	uchar i;

	if(us<1) 
		return;
	while(--us)
	{
		i = 2;
		while (--i);
	}
}

void Delay_ms10(uchar ms10)
{
	uchar dat;

	while(ms10--)
	{
		dat = time_ms10;
		while(dat==time_ms10) 
			__RESET_WATCHDOG();    			//AW60-AC60
	}
}



//*********************************************************************
//********************************************************************* 
// ѭ�� �ȴ� ���ڷ������� �������ַ���3��
void uart2_send(uchar retry)
{
	uchar dat;
	uint  timeout=0;

	//��������
	g_rcvnum = 0;
	while(retry--)
	{
		//��������
		dat = SCI2S1;						// ����״̬
		g_sendnum = 1;
		SCI2D = 0xbb;						// ����
		
		dat = SCI2C2;						// 
		dat |= 0x80;
		dat &= 0XBF;
		SCI2C2 = dat;						// ���ڿ���
      
		//�ȴ����ͽ���
		for(timeout=0; timeout<250; timeout++)
		{
			Delay_us(200);
			__RESET_WATCHDOG();
		}
	} 
}


//���Ƽ���Ĥ 
void Start_Heat1(void) 						//�� PTD5
{
	PTDDD_PTDDD5 = 1;
	PTDD_PTDD5 = 1;
}

void Stop_Heat1(void)  						//��
{
	PTDDD_PTDDD5 = 1;
	PTDD_PTDD5 = 0;
}


///////////////////////////////////////////////////////
//�����͸�λ 
//extern near void _Startup(void);       // reset interrupt service routine 
//void (* near const _vect[])() @0xFFFE = { /* RESET Interrupt vector */
//        _Startup
//};

//=====================================================
// ��ʱ�ж�  10ms
interrupt 14 void T2(void)
{
	//1oms
	TPM2SC &= 0x7f;

	card_ms10++;
	time_ms10++;	
	
	if(!RT_PIN)
	{
		if(hl_chngbz==0)
		{
			hl_chngbz = 1; 					// �ոձ仯�ߵ͵�ƽ  ��Ҫ���� 
			rt_340 = 0;
		}
		rt_340++;
	}
	else
	{
		if(hl_chngbz>0)
		{
			hl_chngbz = 0; 					// �ոձ仯�ߵ͵�ƽ  ��Ҫ����		
			rt_2s = 0;
		}
		rt_2s++; 
	}			  							// 	�����Ӧ���� 
	
	if(led_rtdsp>0)
		led_rtdsp--;

	if( time_ms10>=100 )					// 1����
	{
		time_ms10 = 0;
		time_s++;
		cpu_reset_s += 1;
		sec += 1;							// 1����
		
		//����Ĥʱ��
		g_heat_now += 1;					// ��ǰ�ܵ���
		if( g_heat_now==40 )
		{
			g_heat_now = 0;					//����װ�ؼ���ʱ��
			G_HEAT1_T = G_HEAT1; 
		}
		//Ĥ1����
		if(g_testing==0)
		{
			if( g_heat_now < G_HEAT1_T ) 
				Start_Heat1(); 					// ��������Ĥ
			else 
				Stop_Heat1(); 
		}
	}

	//��⿪�ż�
	if (!KEY1_PIN)
	{
		if (g_key1_times<KEY1_DELAY) 
			g_key1_times++; 
	}
	else 
		g_key1_times = 0; 
	//uchar  g_key1_times = 0xff;
    //#define KEY1_DELAY  100

}

//////////////////////////////////////////////////////
// SCI�ж�
interrupt 19 void SCI2_ERROR(void)
{
	uchar i;
	i = SCI2S1;
	return;
}

//*********************************************************************
// ���ڽ����ж�    						AA FF 13 01 05 3A 8D D4 6A 09 00 00 00 00 00 00 00 00 00 00 00 00 26 BB 
interrupt 20 void SCI2_R(void)		//  AA FF 00 08 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 07 BB
{
	uchar recei;

	recei = SCI2S1;
	recei = SCI2C3;
	recei = SCI2D;
	
	if(g_rcvnum>24)
		g_rcvnum = 24; 
	
	
	if(recei==0xaa)
	{
		if( (rsv_AA!=0xaa)&&(g_rcvnum!=22) )// ��һ���ֽڲ���AA �ұ��ֽڲ���У�� 
		{
			g_rcvnum = 0;					// ָ��
			g_rcvbcc = 0x56;				// 56 + aa = 0 У��
		}
	}
	rsv_AA = recei;							// ��������յ�2��AA ��ô�ڶ���AA�Ǳ������
	
	if(g_cmd_received==0)
	{
		g_rcvbuf[g_rcvnum] = recei;			//��������
		g_rcvnum++;
	
		if(g_rcvnum<22)
			g_rcvbcc += recei;	
	}
	
	if (g_rcvnum==24) 					//�����ֽ�
	{
		if(recei==0xbb)
		{			
			if( (g_rcvbuf[1]!=0xff)&&(g_rcvbuf[22]==g_rcvbcc) )
				g_cmd_received = 1; 
			g_rcvnum = 0;
			cpu_reset_s = 0;
		}
	}
}

// ���ڷ����ж�
interrupt 21 void SCI2_T(void)
{
	uchar i;
	i = SCI2S1;

	if( g_sendnum >= SENDBUFLEN )  		//�������  SENDBUFLEN = 24
		SCI2C2 = 0x2c; 
	else  								//�������� 
		SCI2D = g_sendbuf[g_sendnum++]; 

	return;
}





//=================================================================
//����Ѱ��  ���3�鿴�п���
uchar Find_card(void)
{
	Init_FM1702();
	if( Request(RF_CMD_REQUEST_ALL)==FM1702_OK ) 
		return FM1702_OK;
	
	__RESET_WATCHDOG();
	if( Request(RF_CMD_REQUEST_ALL)==FM1702_OK ) 
		return FM1702_OK;
	
	__RESET_WATCHDOG();
	if( Request(RF_CMD_REQUEST_ALL)==FM1702_OK ) 
		return FM1702_OK;
	
	return FM1702_REQERR;
}



//*********************************************************************
//��������
void UART2_kaimen(void)
{
	uchar i;
	g_sendbuf[1] = g_uart2_index++;		// �������
	g_sendbuf[2] = g_rcvbuf[2];			// 
	g_sendbuf[3] = KAIMEN;					// 
	for(i=4; i<=22; i++) 
		g_sendbuf[i] = 0x00; 
	
	for(i=1; i<=21; i++) 
		g_sendbuf[22] += g_sendbuf[i]; 
	g_sendbuf[23] = 0xaa;
    uart2_send(1);
}

// �����ȼ�� ���� ��ҹ
uchar Test_light(void) //0-day, 1-night
{
	uchar ad;
	//;��ʼ��AD�Ĵ���
	APCTL1 = 0;
	APCTL2 = 0x80;
	//APCTL3 = 0;      				//AW60-AC60
	ADC1CFG = 0x10;    				//;1 Low power configuration
									//;1 Long sample time
									//         ;00-bit conversion (N=8)
    ADC1SC2 = 0x00;   				// ;0 Conversion not in progress
									// ;0 Software trigger selected
									// ;0 Compare function disabled
									// ;0 Compare triggers when input is less than compare level
    ADC1SC1 = 0x1f;

    ADC1CFG = 0x10; 				//   ;8λ������
    ADC1SC1 = 15;   				//  ;��ʼת�� WritingADC1SC1 aborts the current conversion and initiates a new conversion
    while(!ADC1SC1_COCO)
		;
    ad = ADC1RL;
    ADC1SC1 = 0x1f; 				//   ;�ر�AD  1C-23  28-35

    APCTL2 = 0x00;
    return ad;
}



//��ȡ�¶�
uchar Read_temprature(uchar road)
{
	uchar i=0;
	uchar time=0;
	uint  ad=0;
	ulong ad_sum=0;
	uint j;

	for (time=0; time<100; time++)
	{ 
	    for(j=0; j<200; j++)	
			__RESET_WATCHDOG();;
     
		ADC1CFG = 0X18; 								//10λ������
		ADC1SC1 = road; 								//��һ·
		while( (ADC1SC1&0x80)==0 )
			;
		ad = ADC1R;
		ad_sum += ad;
	}
	ad = (uint)(ad_sum/100);

	//������¶�ֵ
	if( (ad>1007) || (ad<60) )   						//����-40��85�� 
		return 0xff; 

	//-40du
	for(i=0; i<128; i++)
	{
		if(ad>temp_mv[i]) 
			return i; 
	}
}


// �¶ȼ��Ϳ���
void Heat(void)
{
	//�¶ȼ��
	g_temprature1 = Read_temprature(7);					//�������� 20H-0��
	g_temprature2 = Read_temprature(8);					//���ô�����
	g_temprature = g_temprature1;
	if( (g_temprature==0) || (g_temprature>85+40) ) 	//������1���� 
		g_temprature = g_temprature2;
	
	if( (g_temprature==0) || (g_temprature>85+40) ) 	//������2���� 
		g_temprature = 0XFF; 
	
	//�˴��������Ĥ���Ƴ���
	//�¶ȴ���
	if (g_temprature==0xff)
		Stop_Heat1(); 									//�����ر���������Ĥ

	//���İ�
	if(g_temprature<=10) 								//<=-30������23S
		G_HEAT1 = 23;
	else if( g_temprature<=30 )							//-10��-30��ֹͣ����
		G_HEAT1 = 20;
	else if(g_temprature<=40) 							//0��-10
		G_HEAT1 = 15;
	else
		G_HEAT1 = 0;
}


//============================================================
//����״̬   �����ַ���
void UART2_test(void) 
{
	uchar i;
	
	g_sendbuf[1] = g_uart2_index++;				// �������
	g_sendbuf[2] = 0X11; 							//
	g_sendbuf[3] = 0X0C;
	g_sendbuf[4] = 4;
	g_sendbuf[5] = 1-re_menci;						// �ϴ��Ŵ�״̬
	g_sendbuf[6] = PTFD_PTFD7;						// Ԥ������
	g_sendbuf[6] = 1-g_sendbuf[6];
	g_sendbuf[7] = g_temprature1;
	g_sendbuf[8] = g_temprature2;
	
	for(i=9;i<=22;i++) 
		g_sendbuf[i] = 0x00; 
	
	for(i=1; i<=21; i++) 							//����У��� 
		g_sendbuf[22] += g_sendbuf[i]; 
	g_sendbuf[23] = 0xaa;
    uart2_send(1);
}



//=====================================================================================
//=====================================================================================
//===================================================================================== 
//��������� 
void main(void)
{
	uchar i;
	ulong delay;
	uchar test_heat=0;

	Stop_Heat1(); 							// �����ر���������Ĥ
	
	for(delay=0; delay<10000; delay++) 	// ������ʱ200ms
		__RESET_WATCHDOG(); 

	//��ʼ������
    DisableInterrupts;
	MCUinit();
	port_init();       						// I/O�˿ڳ�ʼ��
	
	mark_pwd_set = 0;
 	Init_FM1702();     						// ��ʼ��1702  IC��
	
    EnableInterrupts;						// ���ж�
	Delay_ms10(100);

    for(;;)
  	{
	    __RESET_WATCHDOG();
     
		//�Լ�ģʽ
		if(g_testing>0)  					// �����Լ�ģʽ  �Ƿ���ڸ�ģʽ���ɴ���ͨ�ž���
		{ 
			if(sec>0)						// 1�뵽��
			{
				sec = 0;
				UART2_test(); 				// ��� ���Է��͵� ����
				
				if (test_heat>0)			// �л�����Ĥ
				{
					test_heat = 0;
					Stop_Heat1(); 			// �����ر���������Ĥ
				}
				else
				{
					test_heat = 1;
					Start_Heat1();			// ��������Ĥ
				}
			}
		}
		
		Heat();								// �¶ȼ��Ϳ���
		
		Check_Ext_Power_Status();
	    //------------------------------------
		// �����Ӧ							// �����Ӧ ƽʱ=0 ����340ms�ĸߵ�ƽ ���2��һ������ 
		 if(RT_PIN) 
		{
			if( (rt_340>25)&&(rt_340<40) )  
			{
				rt_340 = 50; 
				//if(RT_FS_C1_g0==0)
				{
					RT_FS_C1_g0 = 1;
					UART2_renti_come(1);	// ������  
				}
				led_rtdsp = 250;
			}
			
			if( (rt_2s>500) && (rt_2s<600) ) 
			{
				rt_2s +=160;
				//if(RT_FS_C1_g0>0)
				{
					UART2_renti_go(1);		// ������ ���� 
					RT_FS_C1_g0 = 0;  
				} 
				led_rtdsp = 60;
			} 
		} 
		
		if(rt_340>50000)					// �ߵ�ƽʱ��
			rt_340 = 50000;
		if(rt_2s>50000)						// �͵�ƽʱ��
			rt_2s = 50000;
		
		
		
		
		if(led_rtdsp>0)
			IC_LED = 1;
		
		//---------------------------------------
	    if(MENCI_PIN) 						// �Ŵſ���
	        i = 1; 
	    else 
	     	i = 0; 
	    if(re_menci != i)
	    {
	        re_menci = i;					// �ϴ��Ŵ�״̬
	        if(re_menci==1) 
	         	UART2_menci_open(1);		// �Ǵſ�
	        else
	           	UART2_menci_close(1);		// �ŴŹ�
	    }
     
	  	//���Ű�ť
	  	if( (g_key1_times>=KEY1_DELAY) && (g_key1_times<0xff) )
	  	{
	  		g_key1_times = 0xff;
	  		UART2_kaimen();					// ����
	  	}
     
	  	//�յ������������������
	  	if(g_cmd_received>0)
	  	{
	  		g_cmd_received = 0;
	  		
	  		if (g_rcvbuf[3]==0x16) 		// �������״̬
	  		{
	  			UART2_ASW(g_rcvbuf[3]);  	// Ӧ��
	  			g_testing = 1;				// ����
	  		}
	  		if (g_rcvbuf[3]==0x17) 		// �˳�����״̬
	  		{
	  			UART2_ASW(g_rcvbuf[3]);  	// Ӧ�� 
	  			g_testing = 0;				
	  		}
	  		
	  		if (g_rcvbuf[3]==CHANGEVOL) 
	  		{
	  			UART2_ASW(g_rcvbuf[3]);  	// Ӧ�� 
	  			// VOL(g_rcvbuf[5]); 			// ��������
	  			//UART2_VOL();				// Ӧ��
	  		}
			
	  		if(g_rcvbuf[3]==0x08) 			// ���� 
	  		{
	  		    UART2_ASW(g_rcvbuf[3]);  	// Ӧ�� 
				PTCDD_PTCDD2 = 1;			// ����ִ�� �� ����
	  		    OPENLOCK = 1; 
	  		}			
	  		if (g_rcvbuf[3]==0x09) 		// ���� 
	  		{
	  		    UART2_ASW(g_rcvbuf[3]);  	// Ӧ��
				PTCDD_PTCDD2 = 1;			// ����ִ�� �� ����
	  		    OPENLOCK = 0;	  		    
	  		}
			
	  		if (g_rcvbuf[3]==0x14) 		// ��ѯ�Ŵ�
	  		{
	  		    if(re_menci==1) 			// �ϴ��Ŵ�״̬
	          	    UART2_menci_open(1); 	// �ظ��Ŵ� ���� ״̬
	            else 
	           	    UART2_menci_close(1); 
	  		}
			
	  		if (g_rcvbuf[3]==0x15) 		// ����WATCHDOG 
	  		    UART2_ASW(g_rcvbuf[3]);  	// Ӧ�� 
	  	}
		
		//ˢ��
		if( card_ms10>50 )  							// �򿨼�����500ms
		{
			//IC_LED = !IC_LED;  // ˢ��ָʾ��		%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
			card_ms10 = 0;
			
			//g_sendbuf[0] = 0xbb;
			//g_sendbuf[1] = g_rcvbcc;					// �������
			//g_sendbuf[2] = 0;
			//g_sendbuf[3] = 0xee;            	//0x01:  IC�������кţ�����Ϊ5Byte 
			//for(i=4; i<=22; i++)
			//	g_sendbuf[i] = g_rcvbuf[i-4];	
			//g_sendbuf[23] = 0xdd;
			//uart2_send(1);
			
			
			if(g_uart2_index>250)						// �������0xff����ACK����
				g_uart2_index = 1;
			
			if( Find_card()==FM1702_OK )                // ��⿨Ƭ,     ��⵽���ı�ʶtagtype
			{
				IC_LED = 1;								// ˢ��ָʾ��
				if (card_find>0)                        // ��û����
				{
					;                    
				}
			   	else if( AntiColl()==FM1702_OK )         // ����ͻ���,  ���Լ�⵽����UID��
			   	{
			       	if( Select_Card()==FM1702_OK )       // ѡ��Ƭ,    ���Լ�⵽����SIZE
					{
						RST = 1;                         // ֹͣ������
						g_sendbuf[1] = g_uart2_index++; // �������
						g_sendbuf[2] = g_rcvbuf[2];
						g_sendbuf[3] = FINDIC;          // 0x01:  IC�������кţ�����Ϊ5Byte
						g_sendbuf[4] = 0x05;
						
						for(i=5; i<=9; i++) 
							g_sendbuf[i] = UID[i-5]; 
						
						for(i=10; i<=22; i++)
							g_sendbuf[i] = 0x00;
						
						for(i=1; i<=21; i++)
							g_sendbuf[22] += g_sendbuf[i];
						g_sendbuf[23] = 0xaa;
					    uart2_send(1);
						card_find = 1;
						card_ms10 = 0;                  // ����ʱ=0
					}
				}
			}
			else  										// �޿�Ƭ
			{
				IC_LED = 0;								// ˢ��ָʾ��
				card_find = 0; 
			}
		}
	}
}

//====================END===============================================================



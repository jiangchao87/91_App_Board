
//定义状态机

#define  searching  0;
#define  reporting 1;

#define uchar unsigned char
#define uint unsigned int

//FM1702需要用到的常量和寄存器定义
//常量定义
#define FALSE 0
#define TRUE 1

//FM1702命令码
#define  Transceive 0x1E   		// 发送接收命令
#define  Transmit   0x1a   		// 发送命令
#define  ReadE2   	0x03  		// 读FM1702EEPROM命令
#define  WriteE2  	0x01  		// 写FM1702EEPROM命令
#define	 Authent1	0x0c  		// 验证命令认证过程第1步
#define  Authent2	0x14		// 验证命令认证过程第2步
#define  LoadKeyE2	0x0b		// 将密码从EEPROM复制到KEY缓存
#define	 LoadKey	0x19		// 将密码从FIFI复制到KEY缓存

#define	 Receive	0x16		// 激活接收电路

#define	 RF_TimeOut	0x2f  		// 发送命令延迟时间
#define	 Req		0x01
#define	 Sel		0x02



//FM1702地址定义
#define		Page_Sel		0x00   	//页选寄存器
#define		Command			0x01	//指令寄存器
#define		FIFODate		0x02	//64byteFIFO的输入输出寄存器
#define		PrimaryStatus	0x03	//发射器、接收器及FIFO的标识位
#define		FIFOLength		0x04 	//当前FIFO内 byte数
#define		SecondaryStatus	0x05	// 各种状态标识寄存器
#define		InterruptEn		0x06	//中断使能/禁止控制
#define		InterruptRq		0x07	//中断请求标识

#define		Control			0x09	//各种控制标识
#define		ErrorFlag		0x0a	//上一条指令结束后错误标识
#define		CollPos			0x0b	//侦测到的第一个冲突位的位置
#define		TimerValue		0x0c	//当前Timer值
#define		CRCResultLSB	0x0d	//CRC协处理器低8位
#define		CRCResultMSB	0x0e	//CRC协处理器高8位
#define		BitFraming		0x0f	//调整面向bit的侦格式

#define		TxControl		0x11	//发射器控制寄存器
#define		CWConductance	0x12	//选择发射脚TX1和TX2发射天线的阻抗
#define		ModWidth		0x15	//选择载波调制宽度

#define		RXControl1		0x19	//接收器控制寄存器
#define		DecoderControl	0x1a	//解码控制寄存器
#define		BitPhase		0x1b	//调整发射器和接收器时钟相差
#define		Rxthreshold		0x1c	//选择bit解码的X值
#define		RxControl2		0x1e	//解码控制及选择接收源
#define		ClockQControl	0x1f	//时钟产生控制

#define	 	RxWait				0x21	//选择发射和接收之间的时间
#define		ChannelRedundancy	0x22	//选择数据校验种类和模式
#define		CRCPresetLSB		0x23	//CRC预置寄存器低8位
#define		CRCPresetMSB		0x24
#define		MFOUTSelect			0x26	//选择MFOUT信号源

#define		FIFOLevel		0x29	//定义FIFO溢出级别
#define		TimerClock		0x2a	//选择Timer时钟的分频
#define		TimerControl	0x2b	//选择TIMER 启动/停止条件
#define		TimerReload		0x2c	//Timer预制值
#define		IRQPinConfig	0x2d	//IRQ 输出配置

//#define	CryptoSelect	0x31	//认证模式选择
#define		TypeSH			0x31	//认证模式选择

//射频卡通讯命令码定义
#define		RF_CMD_REQUEST_STD	0x26
#define		RF_CMD_REQUEST_ALL	0x52
#define		RF_CMD_ANTICOL		0x93
#define		RF_CMD_SELECT		0x93
#define		RF_CMD_AUTH_LA		0x60
#define		RF_CMD_AUTH_LB		0x61
#define		RF_CMD_READ			0x30
#define		RF_CMD_WRITE		0xa0
#define		RF_CMD_INC			0xc1
#define		RF_CMD_DEC			0xc0
#define		RF_CMD_RESTORE		0xc2
#define		RF_CMD_TRANSFER		0xb0
#define		RF_CMD_HALT			0x50
 
//Status Values
#define		ALL			0x01
#define		KEYB		0x04
#define		KEYA		0x00
#define		_AB			0x40
#define		CRC_A		1
#define		CRC_B		2

#define		CRC_OK		0
#define		CRC_ERR		1

#define		BCC_OK		0
#define		BCC_ERR		1

//函数错误代码定义
#define		FM1702_OK			0	//正确
#define		FM1702_NOTAGERR		1	//无卡
#define		FM1702_CRCERR		2	//卡片CRC校验错误
#define		FM1702_EMPTY		3	//数值溢出错误
#define		FM1702_AUTHERR		4	//验证不成功
#define		FM1702_PARITYERR	5	//卡片奇偶校验错误
#define		FM1702_CODEERR		6	//通讯错误(BCC校验错)
#define		FM1702_SERNRERR		8	//卡片序列号错误
#define		FM1702_SELECTERR	9	//卡片数据长度字节错误
#define		FM1702_NOTAUTHERR	10	//卡片没有通过验证
#define		FM1702_BITCOUTERR	11	//从卡片接收到的位数错误
#define		FM1702_BYTECOUNTERR	12	//从卡片接收到的直接数错误
#define		FM1702_RESTERR		13	//调用RESTORE函数出错
#define		FM1702_TRANSERR		14	//调用TRANSFER函数出错
#define		FM1702_WRITEERR		15	//调用WRITE函数出错
#define		FM1702_INCRERR		16	//调用INCREMENT函数出错
#define		FM1702_DECRERR		17	//调用DECREMENT函数出错
#define		FM1702_READERR		18	//调用READ函数出错
#define		FM1702_LOADKEYERR	19	//调用LOADKEY函数出错
#define		FM1702_FRAMINGERR	20	//FM1702侦错误
#define		FM1702_REQERR		21	//调用REQ函数出错
#define		FM1702_SELERR		22	//条用SEL函数出错
#define		FM1702_ANTICOLLERR	23	//条用ANTICOLL函数出错
#define		FM1702_INTIVALERR	24	//调用初始化函数出错
#define		FM1702_READVALERR	25	//调用高级读块值函数出错
#define		FM1702_DESELECTERR	26
#define		FM1702_CMD_ERR		42	//命令错误

#define 	old_pwd 	0    		//原始密码
#define 	new_pwd 	1   		//新密码

//串口通讯返回码定义
#define  UART_WR_OK  		0    	//写数据命令成功
#define  UART_PWD_OK   		0x10  	//写置换盒成功
#define  UART_BB_OK    		0x20  	//BB命令接收成功
#define  UART_HAND_OK  		0x30  	//握手命令成功
#define  UART_RSTPWD_OK 	0x40  	//还原密码成功

#define  UART_VAL_ERR  		1  		//扇区或块号错误
#define  UART_RD_ERR 		2   	//读数据错误
#define  UART_WR_ERR 		3  		//写数据错误
#define  UART_AUTH_ERR 		4 		//密码认证错误
#define  UART_PWD_ERR 		5 		//密码置换盒格式错误
#define  UART_NOPWD_ERR 	6   	//未写密码置换盒错误
#define  UART_OLDPWD_ERR 	7  		//使用原始密码认证错误
#define  UART_WRPWD_ERR 	8  		//写密码错误
#define  UART_CHESUM_ERR 	9  		//校验和错误
#define  UART_NOCARD  		0x0A  	//FM1702_NOTAGERR   //无卡

#define  UART_BUSY 		0x0B   		//串口忙,即上个任务还没处理完
#define  UART_TEST 		0x0C   		//上电发送一次
#define  UART_CARD_ERR  0x0D  		//卡号错误，即当前卡号与所要写的卡号不同;
#define  UART_NULL 		0x0f   		//无效命令





uchar tagtype[2];	//卡片标识字符


//FM1702变量定义
uchar buffer[20];				//FM1702命令发送接收缓冲区
uchar UID[5],UID_TMP[5];   	//序列号

//密码存放区
uchar  key[6] = {0xFF, 0xFF, 0xff, 0xff, 0xFF, 0xFF};
uchar key_buff[8] ;	//={0x10 , 0x32 , 0x54 , 0x76 , 0x98 , 0xba , 0xdc , 0xfe};   //用于保存密码置换盒中的数据
uchar key_pwd[6];   			//存放置换后的密码


uint shanqu=5,shanqu_tmp=0;   	//扇区号,双字节
uchar block;   					//块地址

uchar TX_BUFF[26],RX_BUFF[26];	//,RX_BUFF_TMP[20];
uchar RX_BUFF_TMP[26];
unsigned char mark_pwd_set;
uchar g_uart2_sending;

//函数列表
void port_init(void);
void read_data(void);
void Read_Card_Pro(void);
void write_data(void);
void write_pro_2(void);
void ack_send_tx(uchar ack_da);
void report_card(void);
void reques_again(void);
void send_data_tx(void);
void blank_send(void);
void  write_cmd(uchar addr,uchar data);
uchar read_cmd(uchar addr);
uchar Judge_Req(uchar *buff);
uchar Check_UID(void);
void Save_UID(uchar row,uchar col,uchar length);
void Set_BitFraming(uchar row,uchar col);
uchar FM1702_Bus_Sel(void);
void Init_FM1702(void);
uchar Read_FIFO(uchar *buff);
void Write_FIFO(uchar count,uchar *buff);
uchar Clear_FIFO(void);
uchar Command_Send(uchar count,uchar *buff,uchar Comm_Set);
uchar Write_E2(uchar lsb,uchar msb,uchar count,uchar *buff);
uchar M500HostCodeKey(uchar *uncoded,uchar *coded) ;
uchar Load_Key(uchar Secnr2,uchar *buff);
uchar Load_keyE2_CPY(uchar Secnr,uchar Mode);
uchar Request(uchar mode);
uchar AntiColl(void);
uchar Select_Card(void);
uchar  Authentication(uchar  *UID,uchar SecNR,uchar mode);
uchar MIF_READ(uchar  *buff,uchar Block_Adr);
uchar MIF_Write(uchar  *buff,uchar Block_Adr);
void pwd_convert(void);
void load_pwd(uchar pwd_type);
void CHECK_PWD(void);
void NET_IC_PRO(void);



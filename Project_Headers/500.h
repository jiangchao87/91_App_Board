
//引脚定义
//uchar PA_DR;
//uchar PA_DDR;
//void SetPAState(uchar bitindex, uchar level);
//void SetPBState(uchar bitindex, uchar level);
 
//控制引脚 
#define WR  PTBD_PTBD6 	 //6 PTBD_PTB6
#define RST PTBD_PTBD5 	 //5
#define CS  PTBD_PTBD4 	 //4
#define RD  PTBD_PTBD3 	 //3
#define A2  PTBD_PTBD2 	 //2
#define A1  PTBD_PTBD1 	 //1
#define A0  PTBD_PTBD0 	 //0
#define D0  PTAD_PTAD0 	 //0
#define D1  PTAD_PTAD1 	 //1
#define D2  PTAD_PTAD2 	 //2


//功能:该函数实现对放入FM1702 操作范围之内的卡片的Request操作
//输入:mode:ALL (检测所有FM1702操作范围之内的卡片)
//	        STD (检测在FM1702操作范围之内处入HALT状态的卡片)
//输出: FM1702_NOTAGERR:无卡
//	     FM1702_OK:     应答正确
//	     FM1702_REQERR: 应答错误 
uchar Request(uchar mode)						// 函数 反复询卡 中使用
{
	uchar temp;
	write_cmd(CRCPresetLSB,0x63);
	write_cmd(CWConductance,0x3f);
	buffer[0] = mode;                  			//Request 模式选择
	write_cmd(BitFraming,0x07);        			//发送7 bit
	write_cmd(ChannelRedundancy,0x03); 			//关闭CRC
	temp = read_cmd(Control);
	temp = temp & (0xf7);
	write_cmd(Control,temp);       	   			//屏蔽CRYPTO1
	temp = Command_Send(1,buffer,Transceive);
	//Read_FIFO(buffer);
	if(temp == FALSE) 
		return FM1702_NOTAGERR; 
	
	temp = Read_FIFO(buffer);
	temp = Judge_Req(buffer);
	if(temp == TRUE)
	{
		tagtype[0] = buffer[0];
		tagtype[1] = buffer[1];
		return FM1702_OK;
	}
	
	return FM1702_REQERR;
}
 
//功能:该函数实现对放入FM1702操作范围之内的卡片的防冲突检测 
//输出: FM1702_NOTAGERR:    无卡
//		FM1702_BYTECOUNTERR:接收字节错误
//		FM1702_SERNRERR:    卡片序列号应答错误
//		FM1702_OK:          卡片应答正确
uchar AntiColl(void)					// 防冲突检测 
{
	uchar temp;
	uchar i;
	uchar row,col;
	uchar pre_row;

	row =0;
	col =0;
	pre_row =0;

	write_cmd(CRCPresetLSB,  0x63);
	write_cmd(CWConductance, 0x3f);
	//write_cmd(ModConductance,0x3f);   		//在FM1702中无此寄存器
	buffer[0] = RF_CMD_ANTICOL;
	buffer[1] = 0x20;
	write_cmd(ChannelRedundancy,0x03);  		//关闭CRC,打开奇偶校验
	temp = Command_Send(2, buffer, Transceive);

	for(;;)
	{
		if (temp==FALSE) 
			return (FM1702_NOTAGERR);    
		temp = read_cmd(ErrorFlag);
		temp = read_cmd(FIFOLength);
		if (temp ==0) 
			return FM1702_BYTECOUNTERR; 
		
		temp = Read_FIFO(buffer);
		Save_UID(row, col, temp);   			//将接收到的UID放入UID数组中
	    //	Show_UID();							//显示UID
		temp = read_cmd(ErrorFlag);				//判断接收是否正确
		temp = temp&0x01;
		if(temp ==0x00)
		{
			temp = Check_UID();
			if (temp == FALSE) 
				return (FM1702_SERNRERR); 
			return FM1702_OK;
		}
		else
		{
			temp = read_cmd(CollPos);
			row = (temp/8);
			col = (temp%8);
			buffer[0] = RF_CMD_ANTICOL;
			Set_BitFraming(row+pre_row, col);
			pre_row = pre_row+row;
			for (i=0; i<pre_row+1; i++) 
				buffer[i+2] = UID[i]; 
			
			if(col != 0x00) 
				row = pre_row +1; 
			else 
				row = pre_row; 
			temp = Command_Send(row+2, buffer, Transceive);
		}
	}
}
 
//功能:该函数实现 
//输出
// 5.4.5 select 
//功能: 该函数实现对方入FM1702操作范围之内的某张卡片进行选择 
//输出：FM1702_NOTAGERR:    无卡
//      FM1702_PARITYERR:   奇偶校验错
//      FM1702_CRCERR:      CRC校验错
//      FM1702_BYTECOUNTERR:接收字节错误
//      FM1702_OK:          应答正确
//      FM1702_SELERR:      选卡出错
uchar Select_Card(void)										// 选 择 卡 
{
	uchar temp,i;
	
	write_cmd(CRCPresetLSB, 0x63);							// 
	write_cmd(CWConductance,0x3f);							// 
	buffer[0] = RF_CMD_SELECT;
	buffer[1] = 0x70;
	for(i=0; i<5; i++) 
	    buffer[i+2] = UID[i]; 
	
	// ChannelRedundancy=0x0f;                            	// 开启CRC,奇偶校验
	write_cmd(ChannelRedundancy, 0x0f);
	temp = Command_Send(7, buffer, Transceive);

	if(temp==FALSE) 
	   return(FM1702_NOTAGERR); 		 					// 卡 无 错误
	else
	{
	    // temp=ErrorFlag;									// 
	    temp = read_cmd(ErrorFlag);
	    if( (temp&0x02)==0x02 ) 
			return(FM1702_PARITYERR);						// 
		
	    if((temp&0x04)==0x04) 
			return(FM1702_FRAMINGERR);						// 
		
	    if( (temp&0x08)==0x08 ) 
			return(FM1702_CRCERR);							// 
		
	    //temp=FIFO_Length;
	    temp = read_cmd(FIFOLength);						// 
	    if(temp!=1) 
	    	return(FM1702_BYTECOUNTERR); 
	    temp = Read_FIFO(buffer);                         	// 从FIFO中读取应答信息
	    temp = *buffer;
	    if( (temp==0x08) || (temp==0x88) || (temp==0x53) )// 判断应答信号是否正确 
	        return(FM1702_OK); 
	    else 
	        return(FM1702_SELERR); 
	} 
} 


//向某一地址中写入数据
//addr:要写的地址，
//data:要写的数据
//使用独立的地址/数据口，在写数据时操作步骤:
//1.首先选择页(page)寄存器，即写A0~A2为0，D0~D2为页地址(0~7),D7=1 表示使用独立的总线
//  然后执行一次写命令
//2.往A0~A2的写某一页的地址，把所要写的数据送到D0~D7，再执行一次写操作
//3.完毕
void  write_cmd(uchar addr,uchar data)
{
	uchar dat;
	uchar delay;
	
    PTADD = 0XFF;   //输出
    
	WR = 1;
	RD = 1;
	for(delay=0; delay<10; delay++)
		;
	CS = 0;
	for(delay=0; delay<10; delay++)
		;

	A0 = 0;
	A1 = 0;
	A2 = 0;
	for(delay=0; delay<10; delay++)
		;

	dat  = addr>>3;
	dat &= 0x07;
	dat |= 0x80;
	PTAD = dat;
	for(delay=0; delay<10; delay++)
		;
	
    WR = 0;
 	for(delay=0; delay<10; delay++)
		;
	WR = 1;
	for(delay=0; delay<10; delay++)
		;
	CS = 1;
 	for(delay=0; delay<10; delay++)
		;
	CS = 0;
	for(delay=0; delay<10; delay++)
		;
	
	if ((addr&0x01)>0) 
		A0=1; 
	else 
		A0=0;
	
	if( (addr&0x02)>0 )  
		A1=1; 
	else 
		A1=0;
	
	if( (addr&0x04)>0 ) 
		A2=1; 
	else 
		A2=0;
	
	for(delay=0; delay<10; delay++)
		;	
	PTAD = data;

	WR = 0;
	for(delay=0; delay<10; delay++)
		;
	WR = 1;
	for(delay=0; delay<10; delay++)
		;
	CS = 1;
}


// 从某一地址中读数据
// 使用独立的地址/数据口，在读数据时操作步骤:
// 1.首先选择页(page)寄存器，即写A0~A2为0，D0~D2为页地址(0~7),D7=1 表示使用独立的总线
//  然后执行一次写命令
// 2.往A0~A2的写某一页的地址，然后就可以从D0~D7读取数据
// 3.完毕
uchar read_cmd(uchar addr)
{
	uchar dat;
	uchar delay;
	
	WR = 1;
	RD = 1;
	for(delay=0; delay<10; delay++)
		;
	A0 = 0;
	A1 = 0;
	A2 = 0;
	for(delay=0; delay<10; delay++)
		;
	
	dat = addr>>3;
	dat &= 0x07;
	dat |= 0x80;
	PTAD = dat;
	for(delay=0; delay<10; delay++)
		;
	CS = 0;
	for(delay=0; delay<10; delay++)
		;
	WR = 0;
	for(delay=0; delay<10; delay++)
		;
	WR = 1;
	for(delay=0; delay<10; delay++)
		;
	CS = 1;
	
	if((addr&0x01)>0) 
		A0=1; 
	else 
		A0=0;
	
	if ((addr&0x02)>0) 
		A1=1; 
	else 
		A1=0;
	
	if(	(addr&0x04)>0	) 
		A2=1; 
	else 
		A2=0;
	
	for(delay=0; delay<10; delay++)
		;	
	PTADD = 0x00;
	
	for(delay=0; delay<10; delay++)
		;
	CS = 0;
	
	for(delay=0; delay<10; delay++)
		;
	RD = 0;
	
	for(delay=0; delay<10; delay++)
		;
	dat =PTAD;
	
	PTADD = 0xFF;
	for(delay=0; delay<10; delay++)
		;
	RD = 1;
	
	for(delay=0; delay<10; delay++)
		;
	CS = 1;
	
	return dat;
}

// 功能:该函数实现对卡片复位应答信号的判断
// 输入:*buff,指向应答数据的指针
// 输出:TRUE，卡片应答信号正确
//	     FALSE，卡片应答信号错误
// 注: temp1=03;上海标准TOKEN卡
//                =04;MIFARE标准8K
//                 =05;MIFARE标准TOKEN卡
//                 =53;上海标准8K卡
uchar Judge_Req(uchar *buff)
{
	uchar temp1,temp2;

	temp1 = *buff;
	temp2 = *(buff+1);
	if( ((temp1 ==0x03)||(temp1 ==0x04)||(temp1==0x05)||(temp1 ==0x53))&&(temp2 ==0x00) ) 
		return TRUE;
	
	return FALSE;
}


/*
名称:Check_UID
功能:该函数实现对收到的卡片的序列号的判断
输出:TRUE 序列号正确
	      FALSE 序列号错误
*/
uchar Check_UID(void)
{
	uchar temp;
	uchar i;

	temp =0x00;
	for (i=0;i<5;i++) 
		temp = temp^UID[i]; 
	
	if (temp ==0) 
		return TRUE; 
	
	return FALSE;
}

/*
名称:Save_UID
功能:该函数实现保存卡片收到的序列号
输入: 	row:产生冲突的行
	  	col:产生冲突的列
	  	length:接收到 UID数据长度
输出:N/A
*/
void Save_UID(uchar row,uchar col,uchar length)
{
	uchar i;
	uchar temp;
	uchar temp1;

	if ((row ==0x00)&&(col ==0x00))
	{
		for (i=0; i<length; i++) 
			UID[i] = buffer[i]; 
	}
	else
	{
		temp = buffer[0];
		temp1 = UID[row -1];
		switch(col)
		{
			case 0:
				temp1 =0x00;
				row =row+1;
				break;
			case 1:
				temp =temp&0xFE;
				temp1 =temp1&0x01;
				break;
			case 2:
				temp =temp&0xFC;
				temp1 =temp1&0x03;
				break;
			case 3:
				temp =temp&0xF8;
				temp1 =temp1&0x07;
				break;
			case 4:
				temp =temp&0xF0;
				temp1 =temp1&0x0F;
				break;
			case 5:
				temp =temp&0xE0;
				temp1 =temp1&0x1F;
				break;
			case 6:
				temp =temp&0xC0;
				temp1 =temp1&0x3F;
				break;
			case 7:
				temp =temp&0x80;
				temp1 =temp1&0x7F;
				break;
			default:
				break;
		}
		buffer[0] = temp;
		UID[row-1] = temp1|temp;
		for (i=1; i<length; i++) 
			UID[row-1+i] = buffer[i]; 
	}
}


// 功能:该函数设置待发送数据的字节数
// 输入:row:产生冲度的行
//		col:产生冲突的列
void Set_BitFraming(uchar row,uchar col)
{
	switch (row)
	{
		case 0:
			buffer[1] =0x20;
			break;
		case 1:
			buffer[1] =0x30;
			break;
		case 2:
			buffer[1] =0x40;
			break;
		case 3:
			buffer[1] =0x50;
			break;
		case 4:
			buffer[1] =0x60;
			break;
		default:
			break;
	}
	
	switch (col)
	{
		case 0:
			write_cmd(BitFraming,0x00);
			break;
		case 1:
			write_cmd(BitFraming,0x11);
			buffer[1] = (buffer[1]|0x01);
			break;
		case 2:
			write_cmd(BitFraming,0x22);
			buffer[1] = (buffer[1]|0x02);
			break;
		case 3:
			write_cmd(BitFraming,0x33);
			buffer[1] = (buffer[1]|0x03);
			break;
		case 4:
			write_cmd(BitFraming,0x44);
			buffer[1] = (buffer[1]|0x04);
			break;
		case 5:
			write_cmd(BitFraming,0x55);
			buffer[1] = (buffer[1]|0x05);
			break;
		case 6:
			write_cmd(BitFraming,0x66);
			buffer[1] = (buffer[1]|0x06);
			break;
		case 7:
			write_cmd(BitFraming,0x77);
			buffer[1] = (buffer[1]|0x07);
			break;
		default:
			break;
	}
}

//总线选择
uchar FM1702_Bus_Sel(void)
{
	uint i;
	
	write_cmd(Page_Sel,0x80);
	for (i=0;i<RF_TimeOut;i++)
	{
		if (read_cmd(Command)==0x00)
		{
			write_cmd(Page_Sel,0x00);
			return TRUE;
		}
	}
	return FALSE;
}

// FM1702初始化
void Init_FM1702(void)
{
	uchar temp=0;
	uint i,j;

	RST = 0; //SetCTL(RST, 0);//MFRST =0;
	for (i=0; i<0x1fff; i++)
	{
	   j = j+1;
	   __RESET_WATCHDOG();
	}

	//读得Command为0x00
	j = 0;
	while( (j<0xff) && ((read_cmd(Command))!=0x00) )   //等待Command =0,FM1702复位成功
	//while ((read_cmd(Command))!=0x00)   //等待Command =0,FM1702复位成功
	{
		j = j+1;
		__RESET_WATCHDOG();
	}
	
	temp = FM1702_Bus_Sel();      		//FM1702 总线选择
	write_cmd(TimerClock,0x0b);   		//151us/per
	write_cmd(TimerControl,0x02); 		//发送结束开定时器，接收开始关定时器
	write_cmd(TimerReload,0x42);  		//10ms定时
	write_cmd(InterruptEn,0x7f);  		//关所有中断
	write_cmd(InterruptRq,0x7f);
	write_cmd(MFOUTSelect,0x02);  		//调试用
	write_cmd(TxControl,0x5b);    		//开启TX1 TX2

	write_cmd(0x3a,0x0a);         		//打开MFOUT
	write_cmd(TypeSH,0x00);
}


// 功能:该函数实现从FM1702的FIFO中读出X bytes数据
// 输入:buff,指向读出数据的指针 
uchar Read_FIFO(uchar *buff)
{
	uchar temp;
	uchar i;

	temp = read_cmd(FIFOLength);
	if(temp ==0) 
		return 0; 
	if(temp >=24)  			 //temp =255时，会进入死循环
		temp = 24;				//因此增加FIFO_Length越限判断
	for(i=0;i<temp;i++) 
		*(buff+i) = read_cmd(FIFODate); 
	return temp;
}


// 功能:该函数实现向FM1702的FIFO写入Xbytes 的数据
// 输入: count,待写入字节的长度
//            buff,指向待写入数据的指针
void Write_FIFO(uchar count,uchar *buff)
{
	uchar i;
	for(i=0;i<count;i++)
	{
		__RESET_WATCHDOG();
		write_cmd(FIFODate,*(buff+i));
	}
}


// 功能:该函数实现清空FM1702中FIFO的数据
// 输出:TRUE,FIFO被清空
// 	     FALSE,FIFO未被清空
uchar Clear_FIFO(void)
{
	uchar temp;
	uint i;

	temp = read_cmd(Control);  //清空FIFO
	temp = (temp | 0x01);
	write_cmd(Control, temp);
	for( i=0; i<RF_TimeOut; i++ )
	{					//检查FIFO是否被清空
		temp = read_cmd(FIFOLength);
		if (temp==0) 
			return TRUE; 
	}
	
	return FALSE;
}


// 功能:该函数实现向FM1702发送命令集的功能
// 输入:count,待发送命令集的长度
//	     buff,指向待发送数据的指针
//	     Comm_Set,命令码
//输出:TRUE,命令被正确执行
//	     FALSE,命令执行错误
uchar Command_Send(uchar count,uchar *buff,uchar Comm_Set)
{
	uint j;
	uchar temp,temp1;
	
	//InterruptRq = 0x7f;
	write_cmd(Command, 0x00);
	temp =Clear_FIFO();
	Write_FIFO(count, buff);
	
	//RxControl2 = 0x01;
	temp = read_cmd(MFOUTSelect);
	write_cmd(Command, Comm_Set);   	//命令执行
	
	for(j=0; j<RF_TimeOut; j++)   		//检查命令执行否
	{
		__RESET_WATCHDOG();
		temp = read_cmd(MFOUTSelect);
		temp = read_cmd(Command);
		temp1 = (read_cmd(InterruptRq)) & 0x80;
		if( (temp ==0x00) || (temp1==0x80) )
		{
			//RxControl2 =0x41;
			return TRUE;
		}
	}	
	//RxControl2 =0x41;
	return FALSE;
}













/*
//////////////////////////////////////////////////////////////////////////////
//读卡数据
//////////////////////////////////////////////////////////////////////////////
void read_data(void)
{
	uchar temp,temp1;
 
	CHECK_PWD();
  	temp1 = shanqu&0xff;
  	temp = Authentication(UID,temp1,KEYA);  //密码认证
  	if (temp == FM1702_OK)
  	{
   		temp = MIF_READ(buffer,temp1*4+block);
  	    if (temp == FM1702_OK)
   		{
     		send_data_tx();   //读取数据成功
    	}
  	    else
   		{
   		    ack_send_tx(UART_RD_ERR);  //读取数据错误
   		}
  	}
    else
  	{
  	    ack_send_tx(UART_AUTH_ERR);    //认证错误
  	}
}*/
/*
void Read_Card_Pro(void)
{
	//读出指定区域的信息
  	shanqu= RX_BUFF_TMP[4];
  	shanqu <<=8;
  	shanqu |= RX_BUFF_TMP[5];
  	block = RX_BUFF_TMP[6];
  	if ((shanqu>15)||(block>3))
    {
        //如果扇区号超过16或块号超过3,报告错误
    	ack_send_tx(UART_VAL_ERR);
	}
	else
	{
		read_data();
	}
}*/
/*
void write_data(void)
{
	uchar i,temp,temp1;
    uchar data_b[16];
    
	CHECK_PWD();
	temp1 = shanqu&0xff;
  	temp = Authentication(UID,temp1,KEYA);  //密码认证
  	if (temp == FM1702_OK)
    {
    		for (i=0;i<16;i++)
		  	{
			      data_b[i] = RX_BUFF_TMP[7+i];
		  	}
		    temp = MIF_Write(data_b,temp1*4+block);

		    if (temp == FM1702_OK)
			{
			      ack_send_tx(UART_WR_OK);  //写数据成功
			}
		    else
			{
			      ack_send_tx(UART_WR_ERR);  //写数据错误
			}
	}
	else
	{
		    ack_send_tx(UART_AUTH_ERR);    //认证错误
	}
}*/
/*
void write_pro_2(void)
{
	uchar i, UID_SUM;

	UID_SUM =0;
	for (i=0;i<4;i++)
	{
	    UID_SUM += UID[i];
	}
	if (RX_BUFF_TMP[23]!= UID_SUM)
  	{
		  ack_send_tx(UART_CARD_ERR);
  	}
	else
	{
		shanqu= RX_BUFF_TMP[4];
		shanqu <<=8;
		shanqu |= RX_BUFF_TMP[5];
		block = RX_BUFF_TMP[6];
		if ((shanqu>15)||(block>=3)||(RX_BUFF_TMP[2]<20))   //第3块为密码区,暂时禁止写,长度小于20个自己也不写
		{      //如果扇区号超过16或块号超过3,报告错误
			  ack_send_tx(UART_VAL_ERR);
		}
		else
		{

  			write_data();
		}
 	}
}

void ack_send_tx(uchar ack_da)
{
	TX_BUFF[0] = 0xff;//帧头
	TX_BUFF[1] = 0xff;//帧头
	TX_BUFF[2] = 0x01;//帧长度12
	TX_BUFF[3] = 0xaa;//
	TX_BUFF[4] = ack_da;//
	TX_BUFF[5] = TX_BUFF[2]+TX_BUFF[3]+TX_BUFF[4];//

  //RZKDevWrite(SERIAL1,(char*)TX_BUFF,6);
}*/

/*
//向上位机主动报告卡的状态
void report_card(void)
{
	uchar i;
	TX_BUFF[0] = 0xff;//帧头
	TX_BUFF[1] = 0xff;//帧头
	TX_BUFF[2] = 0x08;//帧长度12
	TX_BUFF[3] = 0xcc;//

	TX_BUFF[4] = 0x00;//
	TX_BUFF[5] = 0x00;//
	TX_BUFF[6] = 0x00;//
//	if (FLAG_F2.bit.PWD_SET )
  if (mark_pwd_set ==1) 
		TX_BUFF[7] = 0x01;//
	else
		TX_BUFF[7] = 0x00;//
	TX_BUFF[8] = UID[0];//
	TX_BUFF[9] = UID[1];//
	TX_BUFF[10] = UID[2];//
	TX_BUFF[11] = UID[3];//

	TX_BUFF[12] =0;
	for (i=2;i<=11;i++)
		TX_BUFF[12] +=TX_BUFF[i];

        //RZKDevWrite(SERIAL1,(char*)TX_BUFF,12);

}*/

/*
void reques_again(void)
{
	uchar i;

	TX_BUFF[0] = 0xff;//帧头
	TX_BUFF[1] = 0xff;//帧头
	TX_BUFF[2] = 0x08;//帧长度12
	TX_BUFF[3] = 0x77;//

	TX_BUFF[4] = 0x00;//
	TX_BUFF[5] = 0x00;//
	TX_BUFF[6] = 0x00;//
	if (mark_pwd_set == 1)
		TX_BUFF[7] = 0x01;//
	else
		TX_BUFF[7] = 0x00;//
	TX_BUFF[8] = UID[0];//
	TX_BUFF[9] = UID[1];//
	TX_BUFF[10] = UID[2];//
	TX_BUFF[11] = UID[3];//

	TX_BUFF[12] =0;
	for (i=2;i<=11;i++)
		TX_BUFF[12] +=TX_BUFF[i];

       //RZKDevWrite(SERIAL1,(char*)TX_BUFF,13);
}*/
/*
void send_data_tx(void)
{
	uchar i;

	TX_BUFF[0] = 0xff;//帧头
	TX_BUFF[1] = 0xff;//帧头
	TX_BUFF[2] = 17;//帧长度17
	TX_BUFF[3] = 0x66;//
	TX_BUFF[4] = 0x00;// ACK
	for (i=0;i<16;i++)
	{
		TX_BUFF[5+i] = buffer[i];
	}

	TX_BUFF[21] =0;
	for (i=2;i<=20;i++)	TX_BUFF[21] +=TX_BUFF[i];

	//RZKDevWrite(SERIAL1,(char*)TX_BUFF,22);
}

//空操作
void blank_send(void)
{
	TX_BUFF[0] = 0xff;//帧头
	TX_BUFF[1] = 0xff;//帧头
	TX_BUFF[2] = 0x02;//帧长度12
	TX_BUFF[3] = 0x88;//

	TX_BUFF[4] = 0x00;//
	TX_BUFF[5] = 0x00;//
	TX_BUFF[6] = 0x8a;//

        //RZKDevWrite(SERIAL1,(char*)TX_BUFF,7);
}*/

/*
名称:Write_E2
功能:该函数实现向FM1702中写入数据
输入:lsb,EE地址(低字节)
		msb,EE地址(高字节)
		count,待写入数据EE的字节个数
		buff,指向待写入数据的指针
输出:	TRUE，EE数据正确写入
		FALSE，EE数据写入有误
*/
/*
uchar Write_E2(uchar lsb,uchar msb,uchar count,uchar *buff)
{
	uchar temp,i;

	for (i=0;i<count;i++)
		{
		*(buff+count-i+2) = *(buff-i+count);
		}
	*buff =lsb;
	*(buff+1) =msb;
	temp = Command_Send(count+2,buff,WriteE2);
	temp = read_cmd(SecondaryStatus);
	temp = temp &0x40;
	if (temp ==0x40)
		{
		return TRUE;
		}
	return FALSE;
}*/

/////////////////////////////////////////////////////
//卡片操作基本函数

/*
EEPROM地址存放格式
块号	块地址	byte地址	读写权限	内容
0		0			00~0F		r				产品信息区

1		1			10~1F		r/w				寄存器复位初始值存放区
2		2			20~2F		r/w				寄存器复位初始值存放区

3		3			30~3F		r/w				寄存器初始值存放区
~
~
7		7			70~7F		r/w				寄存器初始值存放区

8		8			80~8F		w				密码存放区
~
~
31		1F			1F0~1FF		w				密码存放区

IC卡中密码保存地址为(huanghuaming设定)
block   地址
0         80~8F
1	    90~9F
~
~
14	    160~16F
15	    170~17F
*/
/*
///////////////////////////////////////////////////////////////////////
//                      C O D E   K E Y S
//    根据密码存放格式移位
///////////////////////////////////////////////////////////////////////
uchar M500HostCodeKey(uchar *uncoded,uchar *coded)      // 6 bytes key value uncoded                                                                        // 12 bytes key value coded
{

   uchar  cnt = 0;
   uchar  ln  = 0;     // low nibble
   uchar  hn  = 0;     // high nibble

   for (cnt = 0; cnt < 6; cnt++)
   	{
      ln = uncoded[cnt] & 0x0F;
      hn = uncoded[cnt] >> 4;
      coded[cnt * 2 + 1] = (~ln << 4) | ln;
      coded[cnt * 2 ] = (~hn << 4) | hn;
   	}
   return 1;
}

//名称:Load_Key
//功能:该函数实现把缓冲区的数据写进EE
//输入:	Block_Adr  块号,范围为0~63
//		buff  保存数据的缓冲区
//输出:TRUE:密码装载成功
//		FALSE:密码装载失败
uchar Load_Key(uchar Secnr2,uchar *buff)
{
	uchar temp,lsb,msb,keybuff[16];
	//Secnr=Block_Adr/4;  //计算扇区号


	temp = M500HostCodeKey(buff,keybuff);
	temp = Secnr2*12;    //设置EEPROM所放密码地址
	if (temp >= 0x80)
		{  //计算密码存放地址
		lsb = temp -0x80;
		msb = 0x01;
		}
	else
		{
		msb =0;
		lsb =temp + 0x80;
		}
	temp =Write_E2(lsb,msb,12,keybuff);

	if (temp ==1)
		return TRUE;
	else return FALSE;
	//return temp;
}

//名称:Load_keyE2
//功能:该函数实现把E2中密码存入FM1702的keybuffer中
//输入:Secnr:EE起始地址
//输出:TRUE:密码装载成功
//		FALSE:密码装载失败
//从此函数可以看出,密码的存取范围是:   //HHM
// KEYA:从0x80开始,一个密码12字节
//KEYB:从0x140开始,一个密码12字节
uchar Load_keyE2_CPY(uchar Secnr,uchar Mode)
{
	uchar temp;
	uchar msb=0;
	uchar lsb =0;

	temp = Secnr*12;
	if (Mode ==0)
		{
		if (temp >= 0x80)
			{  //计算密码存放地址
			lsb = temp -0x80;
			msb = 0x01;
			}
		else
			{
			msb =0;
			lsb =temp + 0x80;
			}
		}
	else
		{
		msb =0x01;
		lsb = temp+0x40;
		}

	buffer[0] = lsb;
	buffer[1] = msb;
	temp = Command_Send(2,buffer,LoadKeyE2);
	temp = ((read_cmd(ErrorFlag))&0x40);
	if (temp==0x40)
		{
		return FALSE;
		}
	return TRUE;
}
*/

/*
// 5.4.6 AUTHENTICATION
//名称：Authentication
//功能：该函数实现密码认证的功能
//输入：UID: 卡片序列号地址
//      SecNR: 扇区号
//      mode: 模式
//输出：FM1702_NOTAGERR:无卡
//      FM1702_PARITYERR:奇偶校验错
//      FM1702_CRCERR:CRC校验错
//      FM1702_OK:应答正确
//      FM1702_AUTHERR:权威认证有错
uchar  Authentication(uchar  *UID,uchar SecNR,uchar mode)
{
     uchar  i;
     uchar  temp,temp1;

     	//CRCPresetLSB=0x63;
     	//CWConductance=0x3f;
   //  MonConductance=0x3f;
	  write_cmd(CRCPresetLSB,0x63);
	  write_cmd(CWConductance,0x3f);
    // temp1=Control;
    temp1 = read_cmd(Control);
     temp1=temp1&0xf7;
     //Control=temp1;
     write_cmd(Control,temp1);

     if (mode==1)                             //AUTHENT1
         buffer[0]=RF_CMD_AUTH_LB;
     else
         buffer[0]=RF_CMD_AUTH_LA;
     buffer[1]=SecNR*4+3;
     for (i=0;i<4;i++)
     {
        buffer[2+i]=UID[i];
     }
     //ChannelRedundancy=0x0f;                  //开启CRC,奇偶校验
     write_cmd(ChannelRedundancy,0x0f);
     temp1 =0;
     temp1 = Command_Send(6,buffer,Authent1);
     if (temp1==FALSE)
     {
        return FM1702_NOTAGERR;
     }
    // temp=ErrorFlag;
    temp = read_cmd(ErrorFlag);
     if ((temp&0x02)==0x02)
          return FM1702_PARITYERR;
     if ((temp&0x04)==0x04)
          return FM1702_FRAMINGERR;
     if ((temp&0x08)==0x08)
          return FM1702_CRCERR;

     temp1= 0;
     temp1=Command_Send(0,buffer,Authent2);       //AUTHENT2
     if (temp1==FALSE)
     {
        return  FM1702_NOTAGERR;
     }
     temp=ErrorFlag;
    temp = read_cmd(ErrorFlag);
     if ((temp&0x02)==0x02)
        return  FM1702_PARITYERR;
     if ((temp&0x04)==0x04)
        return  FM1702_FRAMINGERR;
     if ((temp&0x08)==0x08)
        return  FM1702_CRCERR;
     //temp1=Control;
     temp1 = read_cmd(Control);
     temp1=temp1&0x08;                         //Cryptolon=1,验证通过
     if (temp1==0x08)
     {
        return FM1702_OK;
     }
     return  FM1702_AUTHERR;
   }

//5.4.7 READ
//名称：MIF_Read
//功能：该函数实现读MIFARE 卡块的数值
//输入: buff: 缓冲区首地址
//      Block_Adr: 块地址
//输出：FM1702_NOTAGERR: 无卡
//      FM1702_PARITYERR:奇偶校验错
//      FM1702_CRCERR:CRC检验错
//      FM1702_BYTECOUNTERR:接收字节错误
//      FM1702_OK:应答正确
uchar MIF_READ(uchar  *buff,uchar Block_Adr)
{
	uchar  temp;

   write_cmd(CRCPresetLSB,0x63);
   write_cmd(CWConductance,0x3f);
   write_cmd(ChannelRedundancy,0x0f);
   buff[0]=RF_CMD_READ;
   buff[1]=Block_Adr;
   temp=Command_Send(2,buff,Transceive);
   if (temp==0)
   {
        return FM1702_NOTAGERR;
   }
   
   temp = read_cmd(ErrorFlag);
   if ((temp&0x02)==0x02) return FM1702_PARITYERR;
   if ((temp&0x04)==0x04) return FM1702_FRAMINGERR;
   if ((temp&0x08)==0x08) return FM1702_CRCERR;
   	
   temp = read_cmd(FIFOLength);
   if (temp==0x10)                        //8k 卡读数据长度为16
   {
     temp = Read_FIFO(buff);
     return FM1702_OK;
   }
   else if (temp==0x04)                   //Token 卡读数据长度为16
   {
     temp = Read_FIFO(buff);
     return FM1702_OK;
   }
   else
   {
      return FM1702_BYTECOUNTERR;
   }
}

// 5.4.8  WRITE
//名称: MIF_Write
//功能：该函数实现写MIFARE 卡块的数值
//输入：buff: 缓冲区首地址
//      Block_Adr:块地址
//输出：FM1702_MOTAGERR:无卡
//      FM1702_BYTECOUNTERR:接收字节错误
//      FM1702_NOTAUTHERR:未经权威认证
//      FM1702_EMPTY:数据溢出错误
//      FM1702_CRCERR:CRC校验错
//      FM1702_PARITYERR:奇偶校验错
//      FM1702_WRITEERR:写卡块数据出错
//      FM1702_OK:应答正确
uchar MIF_Write(uchar  *buff,uchar Block_Adr)
{
      uchar  temp;
      uchar  *F_buff;

 	  write_cmd(CRCPresetLSB,0x63);
	  write_cmd(CWConductance,0x3f);
      F_buff=buff+0x10;
      write_cmd(ChannelRedundancy,0x07);
      *F_buff=RF_CMD_WRITE;
      *(F_buff+1)=Block_Adr;
      temp=Command_Send(2,F_buff,Transceive);
      if (temp==FALSE)
      {
          return(FM1702_NOTAGERR);
      }
      
      temp = read_cmd(FIFOLength);
      if (temp==0)
      {
          return(FM1702_BYTECOUNTERR);
      }
      temp = Read_FIFO(F_buff);
      temp=*F_buff;
      switch(temp)
      {
        case 0x00: return(FM1702_NOTAUTHERR);   //暂时屏蔽掉写错误
        case 0x04: return(FM1702_EMPTY);
        case 0x0a: break;
        case 0x01: return(FM1702_CRCERR);
        case 0x05: return(FM1702_PARITYERR);
        default:   return(FM1702_WRITEERR);
       }
      temp=Command_Send(16,buff,Transceive);
      if (temp==TRUE)
      {
        return(FM1702_OK);
      }
      else
      {
       // temp=ErrorFlag;
       temp = read_cmd(ErrorFlag);
        if ((temp&0x02)==0x02)
            return(FM1702_PARITYERR);
        else if ((temp&0x04)==0x04)
            return(FM1702_FRAMINGERR);
        else if ((temp&0x08)==0x08)
            return(FM1702_CRCERR);
        else
            return(FM1702_WRITEERR);
      }
}

//名称:pwd_convert
//功能:该函数实现密码置换功能
//          即使用key_buff中的顺序,
//          把shanqu, block,UID[4]共3个字置换后产生新密码key_pwd
//          例如:
//shanqu block = 0x05 0x03,合成字后用二进制表示为:
//           0 0 0 0 0 1 0 1  0 0 0 0 0 0 1 1
//顺序 f e d c b a 9 8  7 6 5 4 3 2 1 0
//使用置换盒为:(这里只是相临俩数字互换)
//           1 0 3 2 5 4 7 6  9 8 b a d c f e
//则置换后的数据变为:
//           1 1 0 0 0 0 0 0  0 1 0 1 0 0 0 0    =0xc050
//    UID= C6 FB 21 81
// C6 FB=1 1 0 0 0 1 1 0 1 1 1 1 1 0 1 1
// 转=   1 1 1 0 1 1 1 1 1 0 0 1 0 0 1 1    =0xef93
//21 81=0 0 1 0 0 0 0 1  1 0 0 0 0 0 0 1
// 转=   0 1 0 0 0 0 1 0  0 1 0 0 1 0 0 0    =0x4248
//输入:shanqu, block,UID[4],key_buff[8]
//输出:key_pwd[6]
void pwd_convert(void)
{
	uchar key_temp2[16];
	uchar i,j,shift,key_temp[16];
	uint temp,shift_t;

	for (i=0; i<6; i++)
		key_pwd[i] = 0;

//首先转换扇区和块号
//不能使用块作为密码,因为每个扇区有4块,而密码都相同
//我们暂定块的位置与扇区相同
//	temp = (shanqu<<8)|block;
	temp = (shanqu<<8)|(shanqu&0xff);
	shift_t = 0x01;
	for (i=0;i<16;i++)
	{
		key_temp[i] = (temp>>i)&shift_t;    //把16位数据保存到key_temp中
	}

	for (i=0, j=7; i<16; j--,i++)
	{       //按照KEY_BUFF中的顺序重新排列数据
		key_temp2[i++] = key_temp[key_buff[j]&0x0f];
		key_temp2[i] = key_temp[(key_buff[j]&0xf0)>>4];
	}

	shift = 0x01;
	for (i=0; i<8; i++)
	{   //重新排列的数据重新组合
		if (key_temp2[i])
			key_pwd[1] |= shift;
		shift <<=1;
	}

	key_pwd[0] = key_pwd[1];

    //第二转换卡号的低两位
	temp = (UID[0]<<8)|UID[1];
	shift_t = 0x01;
	for (i=0; i<16; i++)
	{
		key_temp[i] = (temp>>i)&shift_t;    //把16位数据保存到key_temp中
	}
	
	for (i=0,j=7;i<16;j--,i++)
	{
		key_temp2[i++] = key_temp[key_buff[j]&0x0f];
		key_temp2[i] = key_temp[(key_buff[j]&0xf0)>>4];
	}

	shift =0x01;
	for (i=0;i<8;i++)
	{
		if (key_temp2[i])
			key_pwd[3] |= shift;
		shift <<=1;
	}

	shift =0x01;
	for (i=8;i<16;i++)
	{
		if (key_temp2[i])
			key_pwd[2] |= shift;
		shift <<=1;
	}

    //第三转换卡号的高两位
	temp = (UID[2]<<8)|UID[3];
	shift_t = 0x01;
	for (i=0;i<16;i++)
		key_temp[i] = (temp>>i)&shift_t;    //把16位数据保存到key_temp中

	for (i=0,j=7;i<16;j--,i++)
	{
		key_temp2[i++] = key_temp[key_buff[j]&0x0f];
		key_temp2[i] = key_temp[(key_buff[j]&0xf0)>>4];
	}

	shift =0x01;
	for (i=0;i<8;i++)
	{
		if (key_temp2[i])
			key_pwd[5] |= shift;
		shift <<=1;
	}

	shift =0x01;
	for (i=8;i<16;i++)
	{
		if (key_temp2[i])
			key_pwd[4] |= shift;
		shift <<=1;
	}
}

//名称:load_pwd
//功能:该函数实现密码置换和加载程序
//输入:
//输出:
void load_pwd(uchar pwd_type)
{
	uchar temp;//,key_temp[6];

    //	for (temp=0;temp<6;temp++)
    //		key_temp[temp] = key[temp];
  	if (pwd_type==old_pwd)
  	{
		temp = Load_Key(1, key);
  	}
 	else
	{
	    pwd_convert();
	    temp = Load_Key(1, key_pwd);
	}

	temp = Load_keyE2_CPY(1, KEYA);  //将EE的密码存到keybuffer
}

/////////////////////////////////////////////////////////////////////////
//
//4mS时间处理程序，调用本程序的周期不可大于4mS
//
////////////////////////////////////////////////////////////////////////
void CHECK_PWD(void)
{
	uchar i,j;
		
	for (i=0;i<5;i++)
	{
	    if (UID_TMP[i]!=UID[i])
	    {
			for (j=0;j<5;j++)
				UID_TMP[j]=UID[j];
			break;
		}
	}
		
	if ((i!=5) || (shanqu_tmp!=shanqu))
	{
	 	shanqu_tmp =shanqu;
	    load_pwd(new_pwd);
	}
}*/
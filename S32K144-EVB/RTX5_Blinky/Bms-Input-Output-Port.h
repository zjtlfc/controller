/*==================================================BMS系统输入/输出接口文件===================================================
说明:该文件的目的是对BMS的输入/输出端口进行重定义，使BMS的输入输出端口名称（或符号）符合控制系统的需求，使程序的修改和使用更加简洁明了。
用户在修改不同的输入/输出驱动的情况下只需要去修改本文件中相应的端口就可以了，其他的文件的驱动部分完全可以不动。
===============================================================================================================================*/
/*========================================================系统中已经定义输出宏===================================================
ENABLE_12V                          //使能12V电源输出
DISABLE_12V                         //禁止12V电源输出
POWER_LOCK_ENABLE                   //设置使能电源自锁
POWER_LOCK_DISABLE                  //设置关闭电源自锁
SLOW_CHARGE_S2_CLOSE                //闭慢充开关S2
SLOW_CHARGE_S2_BREAK                //打开慢充开关S2
=========================================================
HS1_OUTPUT_HIGH                     //高端开关1输出高电平
HS2_OUTPUT_HIGH                     //高端开关2输出高电平
HS3_OUTPUT_HIGH                     //高端开关3输出高电平
HS4_OUTPUT_HIGH                     //高端开关4输出高电平
HS5_OUTPUT_HIGH                     //高端开关5输出高电平
HS6_OUTPUT_HIGH                     //高端开关6输出高电平
HS7_OUTPUT_HIGH	                  //高端开关7输出高电平
HS8_OUTPUT_HIGH                     //高端开关8输出高电平
==========================================================
HS1_OUTPUT_LOW                      //高端开关1输出低电平
HS2_OUTPUT_LOW                      //高端开关2输出低电平
HS3_OUTPUT_LOW                      //高端开关3输出低电平
HS4_OUTPUT_LOW                      //高端开关4输出低电平
HS5_OUTPUT_LOW                      //高端开关5输出低电平
HS6_OUTPUT_LOW                      //高端开关6输出低电平
HS7_OUTPUT_LOW                      //高端开关7输出低电平
HS8_OUTPUT_LOW                      //高端开关8输出低电平
==========================================================
LS1_ENABLE                          //低端开关1使能 
LS2_ENABLE                          //低端开关2使能
LS3_ENABLE                          //低端开关3使能
LS1_DISABLE                         //低端开关1禁止
LS2_DISABLE                         //低端开关2禁止
LS3_DISABLE                         //低端开关3禁止
LS3_TRIGGER                         //低端开关3翻转触发,PTD17=~PTD17
==========================================================
KBM1_RELAY_CLOSE                    //KBM1光MOSFET闭合
KBM2_RELAY_CLOSE                    //KBM2光MOSFET闭合
KBM3_RELAY_CLOSE                    //KBM3光MOSFET闭合
KBM4_RELAY_CLOSE                    //KBM4光MOSFET闭合
KBM1_RELAY_BREAK                    //KBM1光MOSFET断开
KBM2_RELAY_BREAK                    //KBM2光MOSFET断开
KBM3_RELAY_BREAK                    //KBM3光MOSFET断开
KBM4_RELAY_BREAK                    //KBM4光MOSFET断开
==================================所有输出端口的驱动状态检测=========================================
POWER_12V_CHECK                     //检测快充12V电源使能状态检测
HS1_OUTPUT_CHECK                    //高端开关1,PTC12状态检测
HS2_OUTPUT_CHECK                    //高端开关2,PTC13状态检测
HS3_OUTPUT_CHECK                    //高端开关3,PTC11状态检测
HS4_OUTPUT_CHECK                    //高端开关4,PTC10状态检测
HS5_OUTPUT_CHECK                    //高端开关5,PTC15状态检测
HS6_OUTPUT_CHECK                    //高端开关6,PTC14状态检测
HS7_OUTPUT_CHECK                    //高端开关7,PTB15状态检测
HS8_OUTPUT_CHECK                    //高端开关8,PTE8状态检测
LS1_OUTPUT_CHECK                    //低端开关1,PTD6状态检测
LS2_OUTPUT_CHECK                    //低端开关2,PTD7状态检测
LS3_OUTPUT_CHECK                    //低端开关3,PTD17状态检测
SLOW_S2_OUTPUT_CHECK                //打开慢充开关S2,PTB10状态检测
KBM4_RELAY_CHECK                    //KBM4光MOSFET,PTE0状态检测
KBM1_RELAY_CHECK                    //KBM1光MOSFET,PTE9状态检测
KBM2_RELAY_CHECK                    //KBM2光MOSFET,PTE12状态检测
KBM3_RELAY_CHECK                    //KBM3光MOSFET,PTE13状态检测
=============================================================系统中已经定义输入宏================================================
KEY_ON_WAKE_SW                      //钥匙唤醒输入信号	
CHARGE_WAKE_SW                      //充电唤醒输入信号
RTC_WAKE_SW                         //适时时钟RTC唤醒
CP_PWM_TECT                         //慢充CP点PWM信号检测
SB1_INPUT                           //开关SB1输入信号
SB2_INPUT                           //开关SB2输入信号
SB3_INPUT                           //开关SB3输入信号
SB4_INPUT                           //开关SB4输入信号
SB5_INPUT                           //开关SB5输入信号
SB6_INPUT                           //开关SB6输入信号
SB7_INPUT                           //开关SB7输入信号
SB8_INPUT                           //开关SB8输入信号
SB9_INPUT                           //开关SB9输入信号
SB10_INPUT                          //开关SB10输入信号
SB11_INPUT                          //开关SB11输入信号
SB12_INPUT                          //开关SB12输入信号
SB13_INPUT                          //开关SB13输入信号
SB14_INPUT                          //开关SB14输入信号
SB15_INPUT                          //开关SB15输入信号
SB16_INPUT                          //开关SB16输入信号
===================================================================================================================================*/
/*====================================================BMS系统接收CAN报文ID=========================================================*/
#define     SLOW_CHARGE_MECHINE_MESSAGE   0X18FF50E5        /*车载慢充机通讯报文ID=0X18FF50E5*/
#define     VCU_BMS_MESSAGE        	      0X0C30F3D0        /*VCU发送给BMS的信息，ID=0x0X0C30F3D0*/
#define     INV_BMS_MESSAGE         	0X0C09A6A7        /*电机控制器发送的数据信息，ID:0C09A6A7*/
/*===================================================系统主状态机宏定义============================================================*/
#define     MAIN_STATE_INITIAL            0                 /*主状态机初始化*/
#define     MAIN_STATE_FAULT              0XAA              /*主状态机进入错误状态*/
#define     MAIN_LOW_RELAY_CHECK          0X01              /*主负接触器粘连检测*/
#define     MAIN_STATE_TEST_SELF          0X05              /*BMS主状态机自检OK*/
#define     MAIN_STATE_DISCHARGE          0X10              /*BMS主状态机:放电状态*/
#define     MAIN_STATE_QUICK_CHARGE       0X15              /*BMS主状态机:国标快充状态*/
#define     MAIN_STATE_SLOW_CHARGE        0X18              /*BMS主状态机:车载慢充状态*/
#define     MAIN_STATE_KEY_OFF            0X1C              /*BMS主状态机:下电状态(Key_Off)*/
#define     MAIN_STATE_STEP_STEP          0X1E              /*BMS主状态机:单步手动执行状态*/
#define     SUB_DISCHARGE_INIT            0X20              /*系统进入放电子状态机:初始化状态*/
#define     SUB_DISCHARGE_PRECHARGE       0X22              /*系统进入放电子状态机:预充电状态*/
#define     SUB_DISCHARGE_MAIN_OUT        0X25              /*系统进入放电子状态机:主正输出状态*/
#define     SUB_DISCHARGE_KEY_OFF         0X28              /*系统进入放电子状态机:钥匙开关Key_Off*/
#define     SUB_DISCHARGE_ERROR           0X2A              /*系统进入放电子状态机:错误报警状态*/
#define     SUB_DISCHARGE_POWER_OFF       0X2C              /*系统进入放电子状态机:高压下电(EVCU发出信号)*/
#define     SUB_SLOWCHARGE_INIT           0X30              /*系统进入慢充子状态机:初始化状态*/
#define     SUB_SLOWCHARGE_CONNECT        0X32              /*系统进入慢充子状态机:慢充插头连接可靠*/
#define     SUB_SLOWCHARGE_TEMP_UNDER     0X34              /*系统进入慢充子状态机:温度过低无法充电，等待状态*/
#define     SUB_SLOWCHARGE_S2_CLOSE       0X36              /*系统进入慢充子状态机:S2开关闭合*/
#define     SUB_SLOWCHARGE_COMMU_OK       0X38              /*系统进入慢充子状态机:BMS与充电机握手OK*/
#define     SUB_SLOWCHARGE_CHARGEING      0X3A              /*系统进入慢充子状态机:开始充电过程*/
#define     SUB_SLOWCHARGE_COMPLETE       0X3C              /*系统进入慢充子状态机:充电完成状态*/
#define     SUB_SLOWCHARGE_KEY_OFF        0X3D              /*系统进入慢充子状态机:要是开关关闭*/
#define     SUB_SLOWCHARGE_ERROR          0X3F              /*系统进入慢充子状态机:充电中发生错误*/
#define     SUB_SLOWCHARGE_WAKE_OFF       0X3E              /*系统进入慢充子状态机:充电唤醒关闭*/
#define     SLOW_PLUG_LOCKING             0X02              /*慢充插座驱动锁止过程中*/
#define     SLOW_PLUG_LOCK_COMPLETE       0X03              /*慢充插座锁止完成*/
#define     SLOW_PLUG_UNLOCKING           0X06              /*慢充插座驱动解锁过程中*/
#define     SLOW_PLUG_UNLOCK_COMPLETE     0X07              /*慢充插座解锁完成*/
#define     SLOW_PLUG_NO_ACTION           0X00              /*慢充插座锁无动作*/
#define     SLOW_PLUG_NO_ACTION_COMMAND   0X00              /*慢充插座锁无动作命令*/
#define     SLOW_PLUG_LOCK_COMMAND        0X01              /*慢充插座锁止命令*/
#define     SLOW_PLUG_UNLOCK_COMMAND      0X02              /*慢充插座解锁锁止命令*/
#define     CHARGE_MACHINE_NORMAL         0X55              /*系统使用的慢充机为未知品牌*/
#define     CHARGE_MACHINE_ATL            0X01              /*系统使用ATL充电机*/
#define     CHARGE_MACHINE_208            0X02              /*系统使用208所充电机*/
#define     CHARGE_MACHINE_ELK_QUICK      0X03              /*系统使用的充电机为依莱克瓦雅的快速充电机*/
#define     CHARGE_MACHINE_GB_QUICK       0XAA              /*系统使用的快速充电机为符合国标的产品*/
/*====================================================EVCU工作状态定义=============================================================*/
#define     EVCU_RUN_NORMAL               0X00              /*EVCU工作正常,无故障*/
#define     EVCU_RUN_ALARM                0X01              /*EVCU行车报警*/
#define     EVCU_RUN_LIMIT                0X02              /*EVCU行车限制,跛行*/
#define     EVCU_RUN_DISABLE              0X03              /*EVCU禁止行车*/
#define     EVCU_CHARGE_ENABLE            0X10              /*EVCU允许充电*/
#define     EVCU_CHARGE_DISABLE           0X20              /*EVCU禁止充电*/
/*====================================================系统输出宏定义===============================================================*/
#define     SUB_ECU_POWER_ON              KBM1_RELAY_CLOSE        /*BMS系统子系统供电输出*/
#define     SUB_ECU_POWER_OFF             KBM1_RELAY_BREAK        /*BMS系统子系统供电停止*/

#define     MAIN_LOW_RELAY_CLOSE          HS3_OUTPUT_HIGH         /*主负接触器触点闭合*/
#define     MAIN_LOW_RELAY_BREAK          HS3_OUTPUT_LOW          /*主负接触器触点断开*/
#define     MAIN_LOW_STATUS_CHECK         HS3_OUTPUT_CHECK        /*主负接触器触点状态检测*/

#define     PRG_RELAY_CLOSE               HS1_OUTPUT_HIGH         /*预充电接触器触点闭合*/
#define     PRG_RELAY_BREAK               HS1_OUTPUT_LOW          /*预充电接触器触点断开*/ 
#define     PRE_RELAY_STATUS_CHECK        HS1_OUTPUT_CHECK        /*预充电接触器触点状态检测*/

#define     MAIN_HIGH_RELAY_CLOSE         HS4_OUTPUT_HIGH         /*主正接触器触点闭合*/
#define     MAIN_HIGH_RELAY_BREAK         HS4_OUTPUT_LOW          /*主正接触器触点断开*/
#define     MAIN_HIGH_STATUS_CHECK        HS4_OUTPUT_CHECK        /*主正接触器触点状态检测*/

#define     QUICK_RELAY_CLOSE             HS6_OUTPUT_HIGH         /*快充接触器触点闭合*/
#define     QUICK_RELAY_BREAK             HS6_OUTPUT_LOW          /*快充接触器触点断开*/
#define     QUICK_RELAY_STATUS_CHECK      HS6_OUTPUT_CHECK        /*快充接触器触点状态检测*/

#define     BMS_RELAY_AUX1_CLOSE          HS7_OUTPUT_HIGH         /*BMS辅助接触器1触点闭合*/
#define     BMS_RELAY_AUX1_BREAK          HS7_OUTPUT_LOW          /*BMS辅助接触器1触点断开*/
#define     BMS_RELAY_AUX1_CHECK          HS7_OUTPUT_CHECK        /*BMS辅助接触器1触点状态检测*/

#define     BMS_RELAY_AUX2_CLOSE          HS5_OUTPUT_HIGH         /*BMS辅助接触器2触点闭合*/
#define     BMS_RELAY_AUX2_BREAK          HS5_OUTPUT_LOW          /*BMS辅助接触器2触点断开*/
#define     BMS_RELAY_AUX2_CHECK          HS5_OUTPUT_CHECK        /*BMS辅助接触器2触点状态检测*/

#define     SLOW_RELAY_CLOSE              HS2_OUTPUT_HIGH         /*慢充接触器触点闭合*/
#define     SLOW_RELAY_BREAK              HS2_OUTPUT_LOW          /*慢充接触器触点断开*/
#define     SLOW_RELAY_STATUS_CHECK       HS2_OUTPUT_CHECK        /*慢充接触器触点状态检测*/

#define     BATTERY_HOT_RELAY_CLOSE       HS8_OUTPUT_HIGH         /*加热接触器触电闭合*/
#define     BATTERY_HOT_RELAY_BREAK       HS8_OUTPUT_LOW          /*加热接触器触点断开*/
#define     BATTERY_HOT_RELAY_CHECK       HS8_OUTPUT_CHECK        /*加热接触器触点状态检测*/

#define     BATTERY_COLD_RELAY_CLOSE      KBM2_RELAY_CLOSE        /*冷却接触器触点闭合*/
#define     BATTERY_COLD_RELAY_BREAK      KBM2_RELAY_BREAK        /*冷却接触器触点断开*/                 
#define     BATTERY_COLD_RELAY_CEHEK      KBM2_RELAY_CHECK        /*冷却接触器触点状态检测*/                 

#define     INTRENET_LOCK_PWM             LS3_TRIGGER             /*低端开关3翻转触发,产生PWM控制高压互锁检测*/ 
/*====================================================系统输入宏定义==============================================================*/
#define     SUB_ECU_POWER_CHECK           SB3_INPUT               /*BMS系统子系统供电检测*/
#define     INTRE_LOCK_TEST               SB5_INPUT               /*高压互锁检测*/
#define     INTRE_LOCK_PWM_STATE          LS3_OUTPUT_CHECK        /*高压互锁PWM输出电平状态*/

/*=====================================================系统所有核心变量数据宏定义区===============================================*/
#define     PARAMETER_VER_CODE            ParamMadeWork->ParaVer                    /*参数文件的版本号(unsigned 16Bit),用该数值除以100表示(V1.00)*/
#define     PROGRAM_VER_CODE              BMS_PROGRAM_VER                           /*BMS控制程序的版本号,用该数值除以100表示(V1.00)*/
#define     SYSTEM_RUN_TIME               SystemTimes                               /*当前系统上电后的运行时间,单位秒*/
#define     SYSTEM_RUN_TIME_ALL           TotalTimes                                /*BMS系统总的运行时间,单位秒*/
#define     BATTERYS_RAIL_CURRENT         RailCurrent                               /*系统母线电流(signed 32Bit),精度0.01A*/  
#define     BATTERYS_RAIL_VOLTAGE         RailVolate                                /*系统母线电压(unsigned 16Bit)，精度0.1V*/
#define     BATTERYS_POWER_OUTPUT         BmsMessage.BmsPower                       /*当前电池包系统输出功率(signed 32Bit),精度1mW*/
#define     BATTERYS_ENERY_ALL            BmsMessage.BatteryTotalEnery              /*目前电池包总能量(signed 32Bit),精度1WS*/  
#define     BATTERYS_ENERY_SURPLUS        BmsMessage.BatterysEnery                  /*电池包剩余能量(signed 32Bit),精度1WS*/
#define     BATTERYS_AH_ALL               BmsMessage.BatteryTotalAH                 /*电池包总的AH数(unsigned 16Bit),精度0.1AH*/
#define     BATTERYS_AH_SURPLUS           BmsMessage.BatteryAHValue                 /*目前电池包剩余AH数(unsigned 16Bit),精度0.1AH*/ 
#define     BATTERYS_SOC                  BmsMessage.BmsSoc                         /*BMS系统当前的SOC(unsigned 16Bit),精度0.1%*/
#define     BATTERYS_SOH                  BmsMessage.BmsSoh                         /*BMS系统当前的SOH(unsigned 8Bit),精度1%*/
#define     DC_DC_RELAY_OUT               DcdcRealyVoltage                          /*DC_DC接触器后端电压(unsigned 16Bit),精度0.1V*/
#define     MAIN_HIGH_RELAY_OUT           HighBoxInvVolate                          /*主正接触器后端电压(unsigned 16Bit),精度0.1V*/
#define     PTC_RELAY_OUT                 PtcAcRelayVoltage                         /*PTC/AC控制接触器后端电压(unsigned 16Bit),精度0.1V*/
#define     CHARGE_QUICK_RELAY_IN         GbQuickVolate                             /*国标快充接触器后端电压(unsigned 16Bit)，精度0.1V*/
#define     CHARGE_SLOW_RELAY_IN          SlowChargeVoltage                         /*车载慢充接触器后端电压(unsigned 16Bit)，精度0.1V*/
#define     TOTAL_CELL_VOL_COUNT          BmsMessage.BmsCellVoltageTotal            /*所有电芯电压累加和(unsigned 16Bit),精度0.1V*/
#define     BMS_V30_STATUS                BmsRunStatus                              /*BMS工作状态值(unsigned 8Bit),具体值见"BMS监控报文说明"中的定义*/
#define     SYSTEM_INSULATION             InsulateResiter                           /*综合绝缘电阻(unsigned 32Bit)，精度0.01MΩ*/
#define     SYSTEM_INSULATION_HIGH        HighIsulation                             /*高端绝缘电阻值(unsigned 32Bit)，精度0.001MΩ*/
#define     SYSTEM_INSULATION_LOW         LowIsolation                              /*低端绝缘电阻值(unsigned 32Bit)，精度0.001MΩ*/
#define     SUB_SYSTEM_NUMBER             BmsMessage.BcuNumber                      /*BMS系统中子系统BCU的总数量(unsigned 8Bit)*/
#define     BATTERYS_CELL_SERIAL_COUNT    BmsMessage.CellSerialCount                /*所有电芯总的串联数量(unsigned 16Bit)*/
#define     BATTERYS_RESITION_ALL         BmsMessage.BmsResition                    /*电池包总的内阻(unsigned 32Bit)，精度0.1mΩ*/
#define     CELL_VOLTAGE_MAX              BmsMessage.CellVoltageMax                 /*电芯电压最大值(unsigned 16Bit),精度0.1mV*/
#define     CELL_VOLTAGE_MIN              BmsMessage.CellVoltageMin                 /*电芯电压最小值(unsigned 16Bit),精度0.1mV*/
#define     CELL_VOLTAGE_AVE              BmsMessage.CellVoltageAve                 /*电芯电压平均值(unsigned 16Bit),精度0.1mV*/
#define     CELL_VOLTAGE_MAX_POS          BmsMessage.CellVolMaxPosition          	/*电芯最高电压所在位置，0-192*/
#define     CELL_VOLTAGE_MIN_POS          BmsMessage.CellVolMinPosition            	/*电芯最高低压所在位置，0-192*/
#define     TEMP_SIGNAL_ALL_NUMBER        BmsMessage.TempSignalNumber               /*温度传感器数量(unsigned 16Bit)*/
#define     CELL_TEMP_MAX                 BmsMessage.CellTempMax                    /*电芯最高温度值(signed 8Bit),精度1℃*/
#define     CELL_TEMP_MIN                 BmsMessage.CellTempMin                    /*电芯最低温度值(signed 8Bit),精度1℃*/
#define     CELL_TEMP_AVE                 BmsMessage.CellTempAve                    /*电芯平均温度值(signed 8Bit),精度1℃*/
#define     CELL_TEMP_MAX_POS             BmsMessage.TempMaxPosition                /*最高/最低温度点传感器位置*/
#define     CELL_TEMP_MIN_POS         	BmsMessage.TempMinPosition            	/*最高/最低温度点传感器位置*/
#define     SYSTEM_CONTROL_POWER          SysControlMessage.PowerUBValue            /*系统输入电源(unsigned 16Bit),精度0.1V*/
#define     SYSTEM_INTRE_POWER            SysControlMessage.PowerVMValue            /*系统内部电源(unsigned 16Bit),精度0.1V*/
#define     SYSTEM_OUT_POWER              SysControlMessage.PowerVs1Value           /*BMS系统输出5V传感器电源(unsigned 16Bit),精度0.01V*/
#define     CHARGE_QUICK_OUT_POWER        BmsMessage.PowerV12Value                  /*国标快充12V输出(unsigned 16Bit),精度0.1V*/
#define     CHARGE_QUICK_TEST2_POWER      SysControlMessage.QuickTest2Value         /*国标快充插座检测点2电压(unsigned 16Bit),精度0.1V*/
#define     CHARGE_SLOW_CC_RESITION       SysControlMessage.SlowCCResition          /*慢充唤醒CC电阻(unsigned 16Bit)，精度1Ω*/
#define     CHARGE_QUICK_HIGH_TEMP        SysControlMessage.QuickPlugHighTemp       /*快充插座高端温度值(unsigned 16Bit)，精度0.1℃*/  
#define     CHARGE_QUICK_LOW_TEMP         SysControlMessage.QuickPlugLowTemp        /*快充插座低端温度值(unsigned 16Bit)，精度0.1℃*/  
#define     CHARGE_SLOW_HIGH_TEMP         SysControlMessage.SlowPlugHighTemp        /*慢充插座高端温度值(unsigned 16Bit)，精度0.1℃*/ 
#define     CHARGE_SLOW_LOW_TEMP          SysControlMessage.SlowPlugLowTemp         /*慢充插座低端温度值(unsigned 16Bit)，精度0.1℃*/ 
#define     CHARGE_SLOW_PWM_PER           SysControlMessage.PwmValue                /*慢充CP信号PWM站空比(unsigned 16Bit)，精度0.1%*/
#define     CHARGE_SLOW_PWM_CYCLE         SysControlMessage.PwmCycleValue           /*慢充CP信号PWM周期(unsigned 16Bit)，精度1uS*/
#define     EVCU_OUTPUT_COMMAND           BmsMessage.EvcuRequstOutput               /*EVCU命令高压出数命令*/
#define     SYSTEM_CELLE_BALANCE_STATUS   CellBalanceCount                          /*BMS系统中电芯的动态平衡状态(unsigned 16Bit),0:系统没有进行电芯平衡动作；>0:系统有电芯正在进行平衡*/
#define     SYSTEM_ERROR_LEVEL            ErrorMessage.FaultLevel                   /*BMS系统当前最高的报警等级(unsigned 8Bit),0、1、2、3、5、6*/
#define     SYSTEM_ERRORCOUNT             ErrorMessage.F3Count+ErrorMessage.F2Count+ErrorMessage.F1Count      /*系统所有报警总数量(unsigned 8Bit)*/

/*=================================================================================================================================================*/



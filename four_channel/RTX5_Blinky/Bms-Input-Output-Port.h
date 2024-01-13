/*==================================================BMSϵͳ����/����ӿ��ļ�===================================================
˵��:���ļ���Ŀ���Ƕ�BMS������/����˿ڽ����ض��壬ʹBMS����������˿����ƣ�����ţ����Ͽ���ϵͳ������ʹ������޸ĺ�ʹ�ø��Ӽ�����ˡ�
�û����޸Ĳ�ͬ������/��������������ֻ��Ҫȥ�޸ı��ļ�����Ӧ�Ķ˿ھͿ����ˣ��������ļ�������������ȫ���Բ�����
===============================================================================================================================*/
/*========================================================ϵͳ���Ѿ����������===================================================
ENABLE_12V                          //ʹ��12V��Դ���
DISABLE_12V                         //��ֹ12V��Դ���
POWER_LOCK_ENABLE                   //����ʹ�ܵ�Դ����
POWER_LOCK_DISABLE                  //���ùرյ�Դ����
SLOW_CHARGE_S2_CLOSE                //�����俪��S2
SLOW_CHARGE_S2_BREAK                //�����俪��S2
=========================================================
HS1_OUTPUT_HIGH                     //�߶˿���1����ߵ�ƽ
HS2_OUTPUT_HIGH                     //�߶˿���2����ߵ�ƽ
HS3_OUTPUT_HIGH                     //�߶˿���3����ߵ�ƽ
HS4_OUTPUT_HIGH                     //�߶˿���4����ߵ�ƽ
HS5_OUTPUT_HIGH                     //�߶˿���5����ߵ�ƽ
HS6_OUTPUT_HIGH                     //�߶˿���6����ߵ�ƽ
HS7_OUTPUT_HIGH	                  //�߶˿���7����ߵ�ƽ
HS8_OUTPUT_HIGH                     //�߶˿���8����ߵ�ƽ
==========================================================
HS1_OUTPUT_LOW                      //�߶˿���1����͵�ƽ
HS2_OUTPUT_LOW                      //�߶˿���2����͵�ƽ
HS3_OUTPUT_LOW                      //�߶˿���3����͵�ƽ
HS4_OUTPUT_LOW                      //�߶˿���4����͵�ƽ
HS5_OUTPUT_LOW                      //�߶˿���5����͵�ƽ
HS6_OUTPUT_LOW                      //�߶˿���6����͵�ƽ
HS7_OUTPUT_LOW                      //�߶˿���7����͵�ƽ
HS8_OUTPUT_LOW                      //�߶˿���8����͵�ƽ
==========================================================
LS1_ENABLE                          //�Ͷ˿���1ʹ�� 
LS2_ENABLE                          //�Ͷ˿���2ʹ��
LS3_ENABLE                          //�Ͷ˿���3ʹ��
LS1_DISABLE                         //�Ͷ˿���1��ֹ
LS2_DISABLE                         //�Ͷ˿���2��ֹ
LS3_DISABLE                         //�Ͷ˿���3��ֹ
LS3_TRIGGER                         //�Ͷ˿���3��ת����,PTD17=~PTD17
==========================================================
KBM1_RELAY_CLOSE                    //KBM1��MOSFET�պ�
KBM2_RELAY_CLOSE                    //KBM2��MOSFET�պ�
KBM3_RELAY_CLOSE                    //KBM3��MOSFET�պ�
KBM4_RELAY_CLOSE                    //KBM4��MOSFET�պ�
KBM1_RELAY_BREAK                    //KBM1��MOSFET�Ͽ�
KBM2_RELAY_BREAK                    //KBM2��MOSFET�Ͽ�
KBM3_RELAY_BREAK                    //KBM3��MOSFET�Ͽ�
KBM4_RELAY_BREAK                    //KBM4��MOSFET�Ͽ�
==================================��������˿ڵ�����״̬���=========================================
POWER_12V_CHECK                     //�����12V��Դʹ��״̬���
HS1_OUTPUT_CHECK                    //�߶˿���1,PTC12״̬���
HS2_OUTPUT_CHECK                    //�߶˿���2,PTC13״̬���
HS3_OUTPUT_CHECK                    //�߶˿���3,PTC11״̬���
HS4_OUTPUT_CHECK                    //�߶˿���4,PTC10״̬���
HS5_OUTPUT_CHECK                    //�߶˿���5,PTC15״̬���
HS6_OUTPUT_CHECK                    //�߶˿���6,PTC14״̬���
HS7_OUTPUT_CHECK                    //�߶˿���7,PTB15״̬���
HS8_OUTPUT_CHECK                    //�߶˿���8,PTE8״̬���
LS1_OUTPUT_CHECK                    //�Ͷ˿���1,PTD6״̬���
LS2_OUTPUT_CHECK                    //�Ͷ˿���2,PTD7״̬���
LS3_OUTPUT_CHECK                    //�Ͷ˿���3,PTD17״̬���
SLOW_S2_OUTPUT_CHECK                //�����俪��S2,PTB10״̬���
KBM4_RELAY_CHECK                    //KBM4��MOSFET,PTE0״̬���
KBM1_RELAY_CHECK                    //KBM1��MOSFET,PTE9״̬���
KBM2_RELAY_CHECK                    //KBM2��MOSFET,PTE12״̬���
KBM3_RELAY_CHECK                    //KBM3��MOSFET,PTE13״̬���
=============================================================ϵͳ���Ѿ����������================================================
KEY_ON_WAKE_SW                      //Կ�׻��������ź�	
CHARGE_WAKE_SW                      //��绽�������ź�
RTC_WAKE_SW                         //��ʱʱ��RTC����
CP_PWM_TECT                         //����CP��PWM�źż��
SB1_INPUT                           //����SB1�����ź�
SB2_INPUT                           //����SB2�����ź�
SB3_INPUT                           //����SB3�����ź�
SB4_INPUT                           //����SB4�����ź�
SB5_INPUT                           //����SB5�����ź�
SB6_INPUT                           //����SB6�����ź�
SB7_INPUT                           //����SB7�����ź�
SB8_INPUT                           //����SB8�����ź�
SB9_INPUT                           //����SB9�����ź�
SB10_INPUT                          //����SB10�����ź�
SB11_INPUT                          //����SB11�����ź�
SB12_INPUT                          //����SB12�����ź�
SB13_INPUT                          //����SB13�����ź�
SB14_INPUT                          //����SB14�����ź�
SB15_INPUT                          //����SB15�����ź�
SB16_INPUT                          //����SB16�����ź�
===================================================================================================================================*/
/*====================================================BMSϵͳ����CAN����ID=========================================================*/
#define     SLOW_CHARGE_MECHINE_MESSAGE   0X18FF50E5        /*���������ͨѶ����ID=0X18FF50E5*/
#define     VCU_BMS_MESSAGE        	      0X0C30F3D0        /*VCU���͸�BMS����Ϣ��ID=0x0X0C30F3D0*/
#define     INV_BMS_MESSAGE         	0X0C09A6A7        /*������������͵�������Ϣ��ID:0C09A6A7*/
/*===================================================ϵͳ��״̬���궨��============================================================*/
#define     MAIN_STATE_INITIAL            0                 /*��״̬����ʼ��*/
#define     MAIN_STATE_FAULT              0XAA              /*��״̬���������״̬*/
#define     MAIN_LOW_RELAY_CHECK          0X01              /*�����Ӵ���ճ�����*/
#define     MAIN_STATE_TEST_SELF          0X05              /*BMS��״̬���Լ�OK*/
#define     MAIN_STATE_DISCHARGE          0X10              /*BMS��״̬��:�ŵ�״̬*/
#define     MAIN_STATE_QUICK_CHARGE       0X15              /*BMS��״̬��:������״̬*/
#define     MAIN_STATE_SLOW_CHARGE        0X18              /*BMS��״̬��:��������״̬*/
#define     MAIN_STATE_KEY_OFF            0X1C              /*BMS��״̬��:�µ�״̬(Key_Off)*/
#define     MAIN_STATE_STEP_STEP          0X1E              /*BMS��״̬��:�����ֶ�ִ��״̬*/
#define     SUB_DISCHARGE_INIT            0X20              /*ϵͳ����ŵ���״̬��:��ʼ��״̬*/
#define     SUB_DISCHARGE_PRECHARGE       0X22              /*ϵͳ����ŵ���״̬��:Ԥ���״̬*/
#define     SUB_DISCHARGE_MAIN_OUT        0X25              /*ϵͳ����ŵ���״̬��:�������״̬*/
#define     SUB_DISCHARGE_KEY_OFF         0X28              /*ϵͳ����ŵ���״̬��:Կ�׿���Key_Off*/
#define     SUB_DISCHARGE_ERROR           0X2A              /*ϵͳ����ŵ���״̬��:���󱨾�״̬*/
#define     SUB_DISCHARGE_POWER_OFF       0X2C              /*ϵͳ����ŵ���״̬��:��ѹ�µ�(EVCU�����ź�)*/
#define     SUB_SLOWCHARGE_INIT           0X30              /*ϵͳ����������״̬��:��ʼ��״̬*/
#define     SUB_SLOWCHARGE_CONNECT        0X32              /*ϵͳ����������״̬��:�����ͷ���ӿɿ�*/
#define     SUB_SLOWCHARGE_TEMP_UNDER     0X34              /*ϵͳ����������״̬��:�¶ȹ����޷���磬�ȴ�״̬*/
#define     SUB_SLOWCHARGE_S2_CLOSE       0X36              /*ϵͳ����������״̬��:S2���رպ�*/
#define     SUB_SLOWCHARGE_COMMU_OK       0X38              /*ϵͳ����������״̬��:BMS���������OK*/
#define     SUB_SLOWCHARGE_CHARGEING      0X3A              /*ϵͳ����������״̬��:��ʼ������*/
#define     SUB_SLOWCHARGE_COMPLETE       0X3C              /*ϵͳ����������״̬��:������״̬*/
#define     SUB_SLOWCHARGE_KEY_OFF        0X3D              /*ϵͳ����������״̬��:Ҫ�ǿ��عر�*/
#define     SUB_SLOWCHARGE_ERROR          0X3F              /*ϵͳ����������״̬��:����з�������*/
#define     SUB_SLOWCHARGE_WAKE_OFF       0X3E              /*ϵͳ����������״̬��:��绽�ѹر�*/
#define     SLOW_PLUG_LOCKING             0X02              /*�������������ֹ������*/
#define     SLOW_PLUG_LOCK_COMPLETE       0X03              /*���������ֹ���*/
#define     SLOW_PLUG_UNLOCKING           0X06              /*���������������������*/
#define     SLOW_PLUG_UNLOCK_COMPLETE     0X07              /*��������������*/
#define     SLOW_PLUG_NO_ACTION           0X00              /*����������޶���*/
#define     SLOW_PLUG_NO_ACTION_COMMAND   0X00              /*����������޶�������*/
#define     SLOW_PLUG_LOCK_COMMAND        0X01              /*���������ֹ����*/
#define     SLOW_PLUG_UNLOCK_COMMAND      0X02              /*�������������ֹ����*/
#define     CHARGE_MACHINE_NORMAL         0X55              /*ϵͳʹ�õ������Ϊδ֪Ʒ��*/
#define     CHARGE_MACHINE_ATL            0X01              /*ϵͳʹ��ATL����*/
#define     CHARGE_MACHINE_208            0X02              /*ϵͳʹ��208������*/
#define     CHARGE_MACHINE_ELK_QUICK      0X03              /*ϵͳʹ�õĳ���Ϊ���������ŵĿ��ٳ���*/
#define     CHARGE_MACHINE_GB_QUICK       0XAA              /*ϵͳʹ�õĿ��ٳ���Ϊ���Ϲ���Ĳ�Ʒ*/
/*====================================================EVCU����״̬����=============================================================*/
#define     EVCU_RUN_NORMAL               0X00              /*EVCU��������,�޹���*/
#define     EVCU_RUN_ALARM                0X01              /*EVCU�г�����*/
#define     EVCU_RUN_LIMIT                0X02              /*EVCU�г�����,����*/
#define     EVCU_RUN_DISABLE              0X03              /*EVCU��ֹ�г�*/
#define     EVCU_CHARGE_ENABLE            0X10              /*EVCU������*/
#define     EVCU_CHARGE_DISABLE           0X20              /*EVCU��ֹ���*/
/*====================================================ϵͳ����궨��===============================================================*/
#define     SUB_ECU_POWER_ON              KBM1_RELAY_CLOSE        /*BMSϵͳ��ϵͳ�������*/
#define     SUB_ECU_POWER_OFF             KBM1_RELAY_BREAK        /*BMSϵͳ��ϵͳ����ֹͣ*/

#define     MAIN_LOW_RELAY_CLOSE          HS3_OUTPUT_HIGH         /*�����Ӵ�������պ�*/
#define     MAIN_LOW_RELAY_BREAK          HS3_OUTPUT_LOW          /*�����Ӵ�������Ͽ�*/
#define     MAIN_LOW_STATUS_CHECK         HS3_OUTPUT_CHECK        /*�����Ӵ�������״̬���*/

#define     PRG_RELAY_CLOSE               HS1_OUTPUT_HIGH         /*Ԥ���Ӵ�������պ�*/
#define     PRG_RELAY_BREAK               HS1_OUTPUT_LOW          /*Ԥ���Ӵ�������Ͽ�*/ 
#define     PRE_RELAY_STATUS_CHECK        HS1_OUTPUT_CHECK        /*Ԥ���Ӵ�������״̬���*/

#define     MAIN_HIGH_RELAY_CLOSE         HS4_OUTPUT_HIGH         /*�����Ӵ�������պ�*/
#define     MAIN_HIGH_RELAY_BREAK         HS4_OUTPUT_LOW          /*�����Ӵ�������Ͽ�*/
#define     MAIN_HIGH_STATUS_CHECK        HS4_OUTPUT_CHECK        /*�����Ӵ�������״̬���*/

#define     QUICK_RELAY_CLOSE             HS6_OUTPUT_HIGH         /*���Ӵ�������պ�*/
#define     QUICK_RELAY_BREAK             HS6_OUTPUT_LOW          /*���Ӵ�������Ͽ�*/
#define     QUICK_RELAY_STATUS_CHECK      HS6_OUTPUT_CHECK        /*���Ӵ�������״̬���*/

#define     BMS_RELAY_AUX1_CLOSE          HS7_OUTPUT_HIGH         /*BMS�����Ӵ���1����պ�*/
#define     BMS_RELAY_AUX1_BREAK          HS7_OUTPUT_LOW          /*BMS�����Ӵ���1����Ͽ�*/
#define     BMS_RELAY_AUX1_CHECK          HS7_OUTPUT_CHECK        /*BMS�����Ӵ���1����״̬���*/

#define     BMS_RELAY_AUX2_CLOSE          HS5_OUTPUT_HIGH         /*BMS�����Ӵ���2����պ�*/
#define     BMS_RELAY_AUX2_BREAK          HS5_OUTPUT_LOW          /*BMS�����Ӵ���2����Ͽ�*/
#define     BMS_RELAY_AUX2_CHECK          HS5_OUTPUT_CHECK        /*BMS�����Ӵ���2����״̬���*/

#define     SLOW_RELAY_CLOSE              HS2_OUTPUT_HIGH         /*����Ӵ�������պ�*/
#define     SLOW_RELAY_BREAK              HS2_OUTPUT_LOW          /*����Ӵ�������Ͽ�*/
#define     SLOW_RELAY_STATUS_CHECK       HS2_OUTPUT_CHECK        /*����Ӵ�������״̬���*/

#define     BATTERY_HOT_RELAY_CLOSE       HS8_OUTPUT_HIGH         /*���ȽӴ�������պ�*/
#define     BATTERY_HOT_RELAY_BREAK       HS8_OUTPUT_LOW          /*���ȽӴ�������Ͽ�*/
#define     BATTERY_HOT_RELAY_CHECK       HS8_OUTPUT_CHECK        /*���ȽӴ�������״̬���*/

#define     BATTERY_COLD_RELAY_CLOSE      KBM2_RELAY_CLOSE        /*��ȴ�Ӵ�������պ�*/
#define     BATTERY_COLD_RELAY_BREAK      KBM2_RELAY_BREAK        /*��ȴ�Ӵ�������Ͽ�*/                 
#define     BATTERY_COLD_RELAY_CEHEK      KBM2_RELAY_CHECK        /*��ȴ�Ӵ�������״̬���*/                 

#define     INTRENET_LOCK_PWM             LS3_TRIGGER             /*�Ͷ˿���3��ת����,����PWM���Ƹ�ѹ�������*/ 
/*====================================================ϵͳ����궨��==============================================================*/
#define     SUB_ECU_POWER_CHECK           SB3_INPUT               /*BMSϵͳ��ϵͳ������*/
#define     INTRE_LOCK_TEST               SB5_INPUT               /*��ѹ�������*/
#define     INTRE_LOCK_PWM_STATE          LS3_OUTPUT_CHECK        /*��ѹ����PWM�����ƽ״̬*/

/*=====================================================ϵͳ���к��ı������ݺ궨����===============================================*/
#define     PARAMETER_VER_CODE            ParamMadeWork->ParaVer                    /*�����ļ��İ汾��(unsigned 16Bit),�ø���ֵ����100��ʾ(V1.00)*/
#define     PROGRAM_VER_CODE              BMS_PROGRAM_VER                           /*BMS���Ƴ���İ汾��,�ø���ֵ����100��ʾ(V1.00)*/
#define     SYSTEM_RUN_TIME               SystemTimes                               /*��ǰϵͳ�ϵ�������ʱ��,��λ��*/
#define     SYSTEM_RUN_TIME_ALL           TotalTimes                                /*BMSϵͳ�ܵ�����ʱ��,��λ��*/
#define     BATTERYS_RAIL_CURRENT         RailCurrent                               /*ϵͳĸ�ߵ���(signed 32Bit),����0.01A*/  
#define     BATTERYS_RAIL_VOLTAGE         RailVolate                                /*ϵͳĸ�ߵ�ѹ(unsigned 16Bit)������0.1V*/
#define     BATTERYS_POWER_OUTPUT         BmsMessage.BmsPower                       /*��ǰ��ذ�ϵͳ�������(signed 32Bit),����1mW*/
#define     BATTERYS_ENERY_ALL            BmsMessage.BatteryTotalEnery              /*Ŀǰ��ذ�������(signed 32Bit),����1WS*/  
#define     BATTERYS_ENERY_SURPLUS        BmsMessage.BatterysEnery                  /*��ذ�ʣ������(signed 32Bit),����1WS*/
#define     BATTERYS_AH_ALL               BmsMessage.BatteryTotalAH                 /*��ذ��ܵ�AH��(unsigned 16Bit),����0.1AH*/
#define     BATTERYS_AH_SURPLUS           BmsMessage.BatteryAHValue                 /*Ŀǰ��ذ�ʣ��AH��(unsigned 16Bit),����0.1AH*/ 
#define     BATTERYS_SOC                  BmsMessage.BmsSoc                         /*BMSϵͳ��ǰ��SOC(unsigned 16Bit),����0.1%*/
#define     BATTERYS_SOH                  BmsMessage.BmsSoh                         /*BMSϵͳ��ǰ��SOH(unsigned 8Bit),����1%*/
#define     DC_DC_RELAY_OUT               DcdcRealyVoltage                          /*DC_DC�Ӵ�����˵�ѹ(unsigned 16Bit),����0.1V*/
#define     MAIN_HIGH_RELAY_OUT           HighBoxInvVolate                          /*�����Ӵ�����˵�ѹ(unsigned 16Bit),����0.1V*/
#define     PTC_RELAY_OUT                 PtcAcRelayVoltage                         /*PTC/AC���ƽӴ�����˵�ѹ(unsigned 16Bit),����0.1V*/
#define     CHARGE_QUICK_RELAY_IN         GbQuickVolate                             /*������Ӵ�����˵�ѹ(unsigned 16Bit)������0.1V*/
#define     CHARGE_SLOW_RELAY_IN          SlowChargeVoltage                         /*��������Ӵ�����˵�ѹ(unsigned 16Bit)������0.1V*/
#define     TOTAL_CELL_VOL_COUNT          BmsMessage.BmsCellVoltageTotal            /*���е�о��ѹ�ۼӺ�(unsigned 16Bit),����0.1V*/
#define     BMS_V30_STATUS                BmsRunStatus                              /*BMS����״ֵ̬(unsigned 8Bit),����ֵ��"BMS��ر���˵��"�еĶ���*/
#define     SYSTEM_INSULATION             InsulateResiter                           /*�ۺϾ�Ե����(unsigned 32Bit)������0.01M��*/
#define     SYSTEM_INSULATION_HIGH        HighIsulation                             /*�߶˾�Ե����ֵ(unsigned 32Bit)������0.001M��*/
#define     SYSTEM_INSULATION_LOW         LowIsolation                              /*�Ͷ˾�Ե����ֵ(unsigned 32Bit)������0.001M��*/
#define     SUB_SYSTEM_NUMBER             BmsMessage.BcuNumber                      /*BMSϵͳ����ϵͳBCU��������(unsigned 8Bit)*/
#define     BATTERYS_CELL_SERIAL_COUNT    BmsMessage.CellSerialCount                /*���е�о�ܵĴ�������(unsigned 16Bit)*/
#define     BATTERYS_RESITION_ALL         BmsMessage.BmsResition                    /*��ذ��ܵ�����(unsigned 32Bit)������0.1m��*/
#define     CELL_VOLTAGE_MAX              BmsMessage.CellVoltageMax                 /*��о��ѹ���ֵ(unsigned 16Bit),����0.1mV*/
#define     CELL_VOLTAGE_MIN              BmsMessage.CellVoltageMin                 /*��о��ѹ��Сֵ(unsigned 16Bit),����0.1mV*/
#define     CELL_VOLTAGE_AVE              BmsMessage.CellVoltageAve                 /*��о��ѹƽ��ֵ(unsigned 16Bit),����0.1mV*/
#define     CELL_VOLTAGE_MAX_POS          BmsMessage.CellVolMaxPosition          	/*��о��ߵ�ѹ����λ�ã�0-192*/
#define     CELL_VOLTAGE_MIN_POS          BmsMessage.CellVolMinPosition            	/*��о��ߵ�ѹ����λ�ã�0-192*/
#define     TEMP_SIGNAL_ALL_NUMBER        BmsMessage.TempSignalNumber               /*�¶ȴ���������(unsigned 16Bit)*/
#define     CELL_TEMP_MAX                 BmsMessage.CellTempMax                    /*��о����¶�ֵ(signed 8Bit),����1��*/
#define     CELL_TEMP_MIN                 BmsMessage.CellTempMin                    /*��о����¶�ֵ(signed 8Bit),����1��*/
#define     CELL_TEMP_AVE                 BmsMessage.CellTempAve                    /*��оƽ���¶�ֵ(signed 8Bit),����1��*/
#define     CELL_TEMP_MAX_POS             BmsMessage.TempMaxPosition                /*���/����¶ȵ㴫����λ��*/
#define     CELL_TEMP_MIN_POS         	BmsMessage.TempMinPosition            	/*���/����¶ȵ㴫����λ��*/
#define     SYSTEM_CONTROL_POWER          SysControlMessage.PowerUBValue            /*ϵͳ�����Դ(unsigned 16Bit),����0.1V*/
#define     SYSTEM_INTRE_POWER            SysControlMessage.PowerVMValue            /*ϵͳ�ڲ���Դ(unsigned 16Bit),����0.1V*/
#define     SYSTEM_OUT_POWER              SysControlMessage.PowerVs1Value           /*BMSϵͳ���5V��������Դ(unsigned 16Bit),����0.01V*/
#define     CHARGE_QUICK_OUT_POWER        BmsMessage.PowerV12Value                  /*������12V���(unsigned 16Bit),����0.1V*/
#define     CHARGE_QUICK_TEST2_POWER      SysControlMessage.QuickTest2Value         /*�������������2��ѹ(unsigned 16Bit),����0.1V*/
#define     CHARGE_SLOW_CC_RESITION       SysControlMessage.SlowCCResition          /*���份��CC����(unsigned 16Bit)������1��*/
#define     CHARGE_QUICK_HIGH_TEMP        SysControlMessage.QuickPlugHighTemp       /*�������߶��¶�ֵ(unsigned 16Bit)������0.1��*/  
#define     CHARGE_QUICK_LOW_TEMP         SysControlMessage.QuickPlugLowTemp        /*�������Ͷ��¶�ֵ(unsigned 16Bit)������0.1��*/  
#define     CHARGE_SLOW_HIGH_TEMP         SysControlMessage.SlowPlugHighTemp        /*��������߶��¶�ֵ(unsigned 16Bit)������0.1��*/ 
#define     CHARGE_SLOW_LOW_TEMP          SysControlMessage.SlowPlugLowTemp         /*��������Ͷ��¶�ֵ(unsigned 16Bit)������0.1��*/ 
#define     CHARGE_SLOW_PWM_PER           SysControlMessage.PwmValue                /*����CP�ź�PWMվ�ձ�(unsigned 16Bit)������0.1%*/
#define     CHARGE_SLOW_PWM_CYCLE         SysControlMessage.PwmCycleValue           /*����CP�ź�PWM����(unsigned 16Bit)������1uS*/
#define     EVCU_OUTPUT_COMMAND           BmsMessage.EvcuRequstOutput               /*EVCU�����ѹ��������*/
#define     SYSTEM_CELLE_BALANCE_STATUS   CellBalanceCount                          /*BMSϵͳ�е�о�Ķ�̬ƽ��״̬(unsigned 16Bit),0:ϵͳû�н��е�оƽ�⶯����>0:ϵͳ�е�о���ڽ���ƽ��*/
#define     SYSTEM_ERROR_LEVEL            ErrorMessage.FaultLevel                   /*BMSϵͳ��ǰ��ߵı����ȼ�(unsigned 8Bit),0��1��2��3��5��6*/
#define     SYSTEM_ERRORCOUNT             ErrorMessage.F3Count+ErrorMessage.F2Count+ErrorMessage.F1Count      /*ϵͳ���б���������(unsigned 8Bit)*/

/*=================================================================================================================================================*/



///////////////////////////////////////////////////////////////////////////////
//                MIEXER Race_test_with_Block_by_gusim Source
//	             Program by choi hong chul / 2005/12/26
////////////////////////////////////////////////////////////////////////////////

#include <iom128.h>
#include <inavr.h>
#include <stdio.h>
#include <comp_a90.h>

#include "MIEXER_Define.h" 
#include "MIEXER_Func.h"
#include "acc_tbl.h"
#include "smooth_turn_tbl.h" 

#define Lcd_Page_Limit 17
#define OneBlockCntStep 210
#define NORTH  0x10
#define EAST   0x20
#define SOUTH  0x40
#define WEST   0x80

#define LS_LOW 20
#define LC_LOW 25
#define LF_LOW 8
#define RF_LOW 8
#define RC_LOW 22
#define RS_LOW 25

unsigned int LM_Spd_index = 0;          //���� ���� ���ӵ� ���̺� �ӵ� �ε����� ����
unsigned int RM_Spd_index = 0;          //������ ���� ���ӵ� ���̺� �ӵ� �ε����� ����
unsigned int LM_Smpd_index = 0;         //���� ���� �������� ���̺� �ӵ� �ε���
unsigned int RM_Smpd_index = 0;         //������ ���� �������� ���̺� �ӵ� �ε���
unsigned char LM_Sm_Tn_mode = 0;        //���� �������� ��������?
unsigned char RM_Sm_Tn_mode = 0;        //���� �������� ��������?
unsigned char LM_Spd_Limit = 100;       //���ʸ��� �ְ�ӵ� ����
unsigned char RM_Spd_Limit = 100;       //������ ���� �ְ�ӵ� ����
unsigned char LM_Spd_Now = 0;            //���� ���� ���� �ӵ�
unsigned char RM_Spd_Now = 0;            //������ ���� ���� �ӵ�
unsigned char Mot_TnStep_Cnt = 0;       //���� ���ڸ� ���Ҷ� ������ ���� ī��Ʈ ����
unsigned int LMot_BlStep_Cnt = 110;       //���콺�� 1��� ������ �˻縦 ���� ���� ī��Ʈ ����
unsigned int RMot_BlStep_Cnt = 110;       //���콺�� 1��� ������ �˻縦 ���� ���� ī��Ʈ ����

unsigned char This_Block_Pos = 0x00;    //���콺 ���� ����� ��ġ
unsigned char Goal_Block_Pos[4] = {0x28, 0x23, 0x23, 0x23}; //���� ��ġ
unsigned char This_Dir = NORTH;          //���� 4��Ʈ �� 1000=>��, 0100=>��, 0010=>��, 0001=>��
unsigned char Maze_Info[256];           //�̷� �� ���� & ������ ���� ���� ����
                                        //���� 4��Ʈ �������� ����, ���� 4��Ʈ�� ������
unsigned char FastLoad[256];            //�ִܰŸ� ��������(����) ���庯�� 
unsigned char FastLoadIndex=0;
unsigned char BlockQueue[256];          //�̷� �ʴܰŸ� �˻�� queue
unsigned char BlockWeight[256];

unsigned char Turn_Limit = 0; 
unsigned char Turn_90Limit = 80;           //�� ���� ��
unsigned char Turn_180Limit = 160;         //�� ���� ��

unsigned char Sensor_data[6]={255, 255, 255, 255, 255, 255};    //���� ������
unsigned char Sen_sel = 0;             //���� ������ ������ ���� �ε���

unsigned char L90Tn_Offset = 20;        //����90�� �� �¿켾�� �� ������
unsigned char R90Tn_Offset = 20;        //������ 90�� �� �¿켾�� �� ������
unsigned char FWall_Chk_Offset = 50;    //���� ���Խ� ������ ���� ������
//unsigned char mode = 0;                      
unsigned int  num_cnt = 0;
//unsigned char menu_ok = 0;
//unsigned char menu_ok_cnt = 0;
unsigned char LCD_Page_Num = 4;         //���� LCD������ ��� �������� ����

/////////////////////////////////////////////////////////////
//		������ LCD�� ��� �Լ�
//-----------------------------------------------------------
void LCD_Page_Viwe(unsigned char page)
{    
     LCD_Clear();
     //�¿� ���� ���� ������ ǥ��
     if(page == 0)  
     {
          LCD_String(0X80, " LS ");
          LCD_String(0X84, " RS ");
          LCD_DatatoDec(0xc0, Sensor_data[0]);
          LCD_DatatoDec(0xc4, Sensor_data[5]);
     }
     //�¿� �밢�� ���� ������ ǥ��
     else if(page == 1)
     {
          LCD_String(0X80, " LC ");
          LCD_String(0X84, " RC ");
          LCD_DatatoDec(0xc0, Sensor_data[1]);
          LCD_DatatoDec(0xc4, Sensor_data[4]);
     }
     //�¿� ���� ���� ������ ǥ��
     else if(page == 2)
     {
          LCD_String(0X80, " LF ");
          LCD_String(0X84, " RF ");
          LCD_DatatoDec(0xc0, Sensor_data[2]);
          LCD_DatatoDec(0xc4, Sensor_data[3]);
     }
     //���� ���콺 ���� ǥ��
     else if(page == 3)
     {
          LCD_String(0X80, "ThisDir!");
          if((This_Dir >> 4)==0x01)
          {
               LCD_String(0Xc1, "North");
          }
          else if((This_Dir >> 4)==0x02)
          {
               LCD_String(0Xc2, "East");
          }
          else if((This_Dir >> 4)==0x04)
          {
               LCD_String(0Xc1, "South");
          }
          else
          {
               LCD_String(0xc2, "West");
          }
     }
      //���� ���콺 ��ġ ǥ��
     else if(page == 4)
     {
          LCD_String(0X80, "ThisPos!");
          LCD_DatatoHex2(0xc2, This_Block_Pos);
     }
     //���ʸ��� �� ���� ī��Ʈ ǥ��
     else if(page == 5)
     {
          LCD_String(0X80, "LMotStep");
          LCD_String(0Xc0, "Cnt:");
          LCD_DatatoDec(0xc5, LMot_BlStep_Cnt);
     }
     //�����ʸ��� ��� ���� ī��Ʈ ǥ��
     else if(page == 6)
     {
          LCD_String(0X80, "RMotStep");
          LCD_String(0Xc0, "Cnt:");
          LCD_DatatoDec(0xc5, RMot_BlStep_Cnt);
     }
     //�̷� ���� ǥ��
     else if(page ==7)
     {
          LCD_String(0x80, "MazeInfo");
          LCD_DatatoHex2(0xc0, This_Block_Pos);
          LCD_Command(0xc4);
          if((Maze_Info[This_Block_Pos] & 0x01) == 0x01)
               LCD_Data('N');
          else
               LCD_Data('_');
          if((Maze_Info[This_Block_Pos] & 0x02) == 0x02)
               LCD_Data('E');
          else
               LCD_Data('_');     
          if((Maze_Info[This_Block_Pos] & 0x04) == 0x04)
               LCD_Data('S');
          else
               LCD_Data('_');
          if((Maze_Info[This_Block_Pos] & 0x08) == 0x08)
               LCD_Data('W');
          else
               LCD_Data('_');
          LCD_Command(0xe0);
          LCD_Data('t');
     }
     //��Ͽ���Ʈ ���� Ȯ��
     else if(page == 8)
     {
          LCD_String(0X80, "BlWeight");
          LCD_DatatoHex2(0xc0, This_Block_Pos);
          LCD_DatatoDec(0xc5, BlockWeight[This_Block_Pos]);
     }
     //�ִܰŸ� ��� ǥ��
     else if(page == 9)
     {
          LCD_String(0X80, "FastLoad");
          LCD_DatatoDec(0xc0, FastLoadIndex);
          LCD_Command(0xc5);
          if(FastLoad[FastLoadIndex]== 0x10)
               LCD_Data('N');
          else if(FastLoad[FastLoadIndex]== 0x20)
               LCD_Data('E');
          else if(FastLoad[FastLoadIndex]== 0x40)
               LCD_Data('S');
          else if(FastLoad[FastLoadIndex]== 0x80)
               LCD_Data('W');
          else if(FastLoad[FastLoadIndex]== 0xf0)
               LCD_Data('G'); 
          else
               LCD_Data('_');    
          LCD_Command(0xe0);
          LCD_Data('t');
     }
     //���� on off ǥ��
     else if(page == 10)
     {
          LCD_String(0X80, "MotPower");
          if((PINE & 0x08)== 0)
          {
               LCD_String(0Xc0, "Now OFF!");
          }
          else
          {
               LCD_String(0Xc0, " Now ON!");
          }
     }
     //90�� �� ����Ʈ �� ǥ��
     else if(page == 11)
     {
          LCD_String(0X80, "T90Limit");
          LCD_DatatoDec(0xc3, Turn_90Limit);
     }
     //180�� �� ����Ʈ �� ǥ��
     else if(page == 12)
     {
          LCD_String(0X80, "T180Limit");
          LCD_DatatoDec(0xc3, Turn_180Limit);
     }
     //���� 90�� �� �¿켾�� �� ������
     else if(page == 13)
     {
          LCD_String(0X80, "LT90_Off");
          LCD_DatatoDec(0xc3, L90Tn_Offset);     
     }
     //������ 90�� �� �¿켾�� �� ������
     else if(page == 14)
     {
          LCD_String(0X80, "RT90_Off");
          LCD_DatatoDec(0xc3, R90Tn_Offset);
     }
     //���� �� ������ �������� �� ǥ��
     else if(page ==15)
     {
          LCD_String(0X80, "FWallChk");
          LCD_DatatoDec(0xc3, FWall_Chk_Offset);
     }
     //���ʸ��� ���ǵ� ����Ʈ �� ǥ��
     else if(page == 16)
     {
          LCD_String(0X80, "LM Speed");
          LCD_String(0Xc0, "Lmt->");
          LCD_DatatoDec(0xc5, LM_Spd_Limit);
     }
     //������ ���� ���ǵ� ����Ʈ �� ǥ��
     else if(page == 17)
     {
          LCD_String(0X80, "RM Speed");
          LCD_String(0Xc0, "Lmt->");
          LCD_DatatoDec(0xc5, RM_Spd_Limit);
     }
     
}
/////////////////////////////////////////////////////////////

/////////////////////////////////////////////////////////////
//		LCD�� ���� ������ ���� �Լ�
//-----------------------------------------------------------
void Edit_Data_in_Lcd(unsigned char page, unsigned char UpDn)
{
     if(page > 6)
     {
          //�̷����� Ȯ��
          if(page==7)
          {
               if(UpDn == 0)
               {
                    if((This_Block_Pos & 0x0f) != 0x0f)
                         This_Block_Pos += 0x01;
                    else
                         This_Block_Pos &= 0xF0;
               }
               else if(UpDn == 1)
               {
                    if((This_Block_Pos & 0xf0) != 0xf0)
                         This_Block_Pos += 0x10;
                    else
                         This_Block_Pos &= 0x0F;
               }         
          }
          //��� ����Ʈ Ȯ�ο�
          else if(page==8)
          {
               if(UpDn == 0)
               {
                    if((This_Block_Pos & 0x0f) != 0x0f)
                         This_Block_Pos += 0x01;
                    else
                         This_Block_Pos &= 0xF0;
               }
               else if(UpDn == 1)
               {
                    if((This_Block_Pos & 0xf0) != 0xf0)
                         This_Block_Pos += 0x10;
                    else
                         This_Block_Pos &= 0x0F;
               }         
          }
          //�ִܰ�� Ȯ�ο�
          else if(page==9)
          {
                if(UpDn == 0 && FastLoadIndex > 0)
                    FastLoadIndex--;
                else if(UpDn == 1 && FastLoadIndex < 0xff)
                    FastLoadIndex++;
          }
          //���� on off ��Ʈ��
          else if(page==10)
          {
               if((PINE & 0x08)== 0)
               {
                    Delay_ms(300);
                    MOTOR_ON();
                    LMotor_Inturrupt_Enable();            //���ʸ��� ���ͷ�Ʈ ���
                    RMotor_Inturrupt_Enable();            //������ ���� ���ͷ�Ʈ ���
               }
               else
               {
                    LMotor_Inturrupt_Disable();            //���ʸ��� ���ͷ�Ʈ ���
                    RMotor_Inturrupt_Disable();            //������ ���� ���ͷ�Ʈ ���
                    MOTOR_OFF();
               }
          }
          //90�� �� ����Ʈ �� ����
          else if(page==11)
          {
               if(UpDn == 0 && Turn_90Limit > 0)
                    Turn_90Limit--;
               else if(UpDn == 1 && Turn_90Limit < 255)
                    Turn_90Limit++;
          }
          //180�� �� ����Ʈ �� ����
          else if(page==12)
          {
               if(UpDn == 0 && Turn_180Limit > 0)
                    Turn_180Limit--;
               else if(UpDn == 1 && Turn_180Limit < 255)
                    Turn_180Limit++;
          }
          //���� 90�� �� �����°� ����
          else if(page==13)
          {
               if(UpDn == 0 && L90Tn_Offset > 0)
                    L90Tn_Offset--;
               else if(UpDn == 1 && L90Tn_Offset < 255)
                    L90Tn_Offset++;
          }
          //������ 90�� �� �����°� ����
          else if(page==14)
          {
               if(UpDn == 0 && R90Tn_Offset > 0)
                    R90Tn_Offset--;
               else if(UpDn == 1 && R90Tn_Offset < 255)
                    R90Tn_Offset++;
          }
          //���� ��üũ�� ���� ������ �� ����
          else if(page==15)
          {
               if(UpDn == 0 && FWall_Chk_Offset > 0)
                    FWall_Chk_Offset--;
               else if(UpDn == 1 && FWall_Chk_Offset < 255)
                    FWall_Chk_Offset++;
          }
          //���ʸ��� ���ǵ� ����Ʈ �� ����
          else if(page==16)
          {
               if(UpDn == 0 && LM_Spd_Limit > 0)
                    LM_Spd_Limit--;
               else if(UpDn == 1 && LM_Spd_Limit < 255)
                    LM_Spd_Limit++;
          }
          //������ ���� ���ǵ� ����Ʈ �� ����
          else if(page==17)
          {
               if(UpDn == 0 && RM_Spd_Limit > 0)
                    RM_Spd_Limit--;
               else if(UpDn == 1 && RM_Spd_Limit < 255)
                    RM_Spd_Limit++;
          }
          LCD_Page_Viwe(page);
     }
}
/////////////////////////////////////////////////////////////

/////////////////////////////////////////////////////////////
//		�ִܰŸ� Ž�� �Լ�
//-----------------------------------------------------------
void FindFastLoad(unsigned char pos)
{
     unsigned char head=1, tail=0;
     unsigned char Weight=0;
     unsigned int j;
     unsigned char tmp_Dir;
      
     for(j=0; j<256; j++)
     {
          BlockWeight[j]=0xff;     //��� ��� ����Ʈ �ʱ�ȭ
     }

     LCD_String(0X80, "Updating");
     LCD_String(0XC0, "BlWeight");
    
     //�켱 ��� ����� ��κ����� �Ÿ�����Ʈ�� �����
     //��ó�� ������ ��ġ�� �������̹Ƿ� ť�� ����ִ´�
     BlockQueue[0] = pos;
     BlockWeight[pos] = 0;
     for( ; head != tail ; )
     {
          pos = BlockQueue[tail++];
          Weight = BlockWeight[pos];
          //�������� ����� ��, ��, ��, ������ ���� ����, ����Ʈ���� �� ũ�� ť�� ���� �ְ� ����Ʈ�� ����
          //���� ��� �˻�
          if((Maze_Info[pos]&0x01)!=0x01)
          {
               if(BlockWeight[pos+0x10] > Weight + 1)
               {
                    BlockQueue[head++] = pos+0x10;
                    BlockWeight[pos+0x10] = Weight + 1;
               }
          }
          //���� ��� �˻�
          if((Maze_Info[pos]&0x02)!=0x02)
          {
               if(BlockWeight[pos+0x01] > Weight+1)
               {
                    BlockQueue[head++] = pos+0x01;
                    BlockWeight[pos+0x01] = Weight + 1;
               }
          }
          //�Ʒ��� ��� �˻�
          if((Maze_Info[pos]&0x04)!=0x04)
          {
               if(BlockWeight[pos-0x10] > Weight+1)
               {
                    BlockQueue[head++] = pos-0x10;
                    BlockWeight[pos-0x10] = Weight + 1;
               }
          }
          //���� ��� �˻�
          if((Maze_Info[pos]&0x08)!=0x08)
          {
               if(BlockWeight[pos-0x01] > Weight+1)
               {
                    BlockQueue[head++] = pos-0x01;
                    BlockWeight[pos-0x01] = Weight + 1;
               }
          }
     }

    	//�̷����� �ʱ�ȭ
     LCD_String(0X80, " Search ");
     LCD_String(0XC0, "FastLoad");
      //������ Ž���ؼ� �迭�� �����ϱ�
   	pos = This_Block_Pos;
   	Weight = BlockWeight[pos];
   	tmp_Dir=This_Dir >> 4;
     
   	for(head = 0; Weight > 0; head++)
   	{
   	     //LCD_DatatoDec(0x80, head);
   	     //LCD_DatatoHex2(0x84, pos);
   	     //LCD_DatatoDec(0xc0, Weight);
   	     //LCD_DatatoHex2(0xc4, tmp_Dir);
   	     //Delay_ms(1500);
          FastLoad[head] = 0;
   	     //�켱 ����������� �˻�
   	     if((Maze_Info[pos] & tmp_Dir)==0)
   	     {
   	          if(tmp_Dir==0x01)
   	          {
   	               if(BlockWeight[pos+0x10] == (Weight-1))
   	               {
        	               FastLoad[head] = tmp_Dir << 4;
        	               pos += 0x10;
        	          }    
   	          }
        	     else if(tmp_Dir==0x02)
   	          {
   	               if(BlockWeight[pos+0x01] == (Weight-1))
   	               {
   	                    FastLoad[head] = tmp_Dir << 4;
   	                    pos += 0x01;
   	               }
        	     }
   	          else if(tmp_Dir==0x04)
   	          {
   	               if(BlockWeight[pos-0x10] == (Weight-1))
   	               {
   	                    FastLoad[head] = tmp_Dir << 4;
   	                    pos -= 0x10;
   	               }
        	     }
   	          else if(tmp_Dir==0x08)
   	          {
   	               if(BlockWeight[pos-0x01] == (Weight-1))
   	               {
   	                    FastLoad[head] = tmp_Dir << 4;
   	                    pos -= 0x01;
   	               }     
        	     }
   	     }
        	if(FastLoad[head]==0)
   	     {
   	          if((tmp_Dir != 0x01) && (Maze_Info[pos] & 0x01)==0)
        	     {
   	               if(BlockWeight[pos+0x10] == (Weight-1))
   	               {
   	                    FastLoad[head] = 0x10;
   	                    pos += 0x10;
   	               }     
   	          }
        	     if((tmp_Dir != 0x02) && (Maze_Info[pos] & 0x02)==0)
   	          {
   	               if(BlockWeight[pos+0x01] == (Weight-1))
   	               {
   	                    FastLoad[head]= 0x20;
   	                    pos += 0x01;
   	               }     
        	     }
   	          if((tmp_Dir != 0x04) && (Maze_Info[pos] & 0x04)==0)
   	          {
   	               if(BlockWeight[pos-0x10] == (Weight-1))
   	               {
   	                    FastLoad[head] = 0x40;
   	                    pos -= 0x10;
   	               }     
        	     }
   	          if((tmp_Dir != 0x08) && (Maze_Info[pos] & 0x08)==0)
   	          {
   	               if(BlockWeight[pos-0x01]== (Weight-1))
   	               {
        	               FastLoad[head] = 0x80;
        	               pos -= 0x01;
        	          }     
   	          }
   	     }
   	     tmp_Dir = FastLoad[head] >> 4;
   	     
   	     Weight = BlockWeight[pos];
   	}
   	//������ ��ǥ�ø� �Ѵ�.
   	FastLoad[head]=0xf0;
	FastLoadIndex = 0;
     LCD_String(0X80, "FastLoad");
     LCD_String(0XC0, " FindOK ");
}
/////////////////////////////////////////////////////////////

/////////////////////////////////////////////////////////////
//		������ ������Ʈ �Լ�
//-----------------------------------------------------------
void UpdateWallInfo(void)
{
	//���� ����� �¿��� �������� ����
	if((Sensor_data[0] >= LS_LOW) && (Sensor_data[1] >= LC_LOW))
     {
     	if(This_Dir == NORTH && (This_Block_Pos & 0x0f) != 0x00)
     	{
          	Maze_Info[This_Block_Pos] |= 0x08;
          	Maze_Info[This_Block_Pos-0x01] |= 0x02;
          }	
         	else if(This_Dir == EAST && (This_Block_Pos & 0xf0) != 0xf0)
         	{
			Maze_Info[This_Block_Pos] |= 0x01;
			Maze_Info[This_Block_Pos+0x10] |= 0x04;
		}	
		else if(This_Dir == SOUTH && (This_Block_Pos & 0x0f) != 0x0f)
		{
			Maze_Info[This_Block_Pos] |= 0x02;
			Maze_Info[This_Block_Pos+0x01] |= 0x08;
		}	
		else if(This_Dir == WEST && (This_Block_Pos & 0xf0) != 0x00)
		{
          	Maze_Info[This_Block_Pos] |= 0x04;
          	Maze_Info[This_Block_Pos-0x10] |= 0x01;
          }	
	}
	if((Sensor_data[4] >= RC_LOW) && (Sensor_data[5] >= RS_LOW))
	{
		if(This_Dir == NORTH && (This_Block_Pos & 0x0f) != 0x0f)
		{
			Maze_Info[This_Block_Pos] |= 0x02;
			Maze_Info[This_Block_Pos+0x01] |= 0x08;
		}	
		else if(This_Dir == EAST && (This_Block_Pos & 0xf0) != 0x00)
		{
			Maze_Info[This_Block_Pos] |= 0x04;
			Maze_Info[This_Block_Pos-0x10] |= 0x01;
		}	
		else if(This_Dir == SOUTH && (This_Block_Pos & 0x0f) != 0x00)
		{
			Maze_Info[This_Block_Pos] |= 0x08;
			Maze_Info[This_Block_Pos-0x01] |= 0x02;
		}	
		else if(This_Dir == WEST && (This_Block_Pos & 0xf0) != 0xf0)
		{
			Maze_Info[This_Block_Pos] |= 0x01;
			Maze_Info[This_Block_Pos+0x10] |= 0x04;
		}	
	}
	if((Sensor_data[2] >= LF_LOW) && (Sensor_data[3] >= RF_LOW))
	{
	     if(This_Dir == NORTH && (This_Block_Pos & 0xf0) != 0xf0)
		{
			Maze_Info[This_Block_Pos] |= 0x01;
			Maze_Info[This_Block_Pos+0x10] |= 0x04;
		}	
		else if(This_Dir == EAST && (This_Block_Pos & 0x0f) != 0x0f)
		{
			Maze_Info[This_Block_Pos] |= 0x02;
			Maze_Info[This_Block_Pos+0x01] |= 0x08;
		}	
		else if(This_Dir == SOUTH && (This_Block_Pos & 0xf0) != 0x00)
		{
			Maze_Info[This_Block_Pos] |= 0x04;
			Maze_Info[This_Block_Pos-0x10] |= 0x01;
		}	
		else if(This_Dir == WEST && (This_Block_Pos & 0x0f) != 0x00)
		{
			Maze_Info[This_Block_Pos] |= 0x08;
			Maze_Info[This_Block_Pos-0x01] |= 0x02;
		}	
	}
}
/////////////////////////////////////////////////////////////

/////////////////////////////////////////////////////////////
//		ȸ������
//----------------------------------------------------------
void NormalTurnInit(void)
{
	LMotor_Inturrupt_Disable();            //���ʸ��� ���ͷ�Ʈ ����
	RMotor_Inturrupt_Disable();            //������ ���� ���ͷ�Ʈ ����
	Delay_ms(1000);
	//�ִܰŸ� �������� ȸ������
	if(This_Dir == NORTH)
	{    
		if(FastLoad[FastLoadIndex] == EAST)
		{
			LM_FORWARD_GO();
			RM_BACKWARD_GO();
			Turn_Limit = Turn_90Limit;
		}    
		else if(FastLoad[FastLoadIndex] == SOUTH)
		{
			LM_FORWARD_GO();
			RM_BACKWARD_GO();
			if(Sensor_data[0] > LS_LOW && Sensor_data[5] > RS_LOW)
				Turn_Limit = Turn_180Limit;//-10;    //�¿��� ���� ������ ���� �ϴ� ���
			else
				Turn_Limit = Turn_180Limit;
		}
		else if(FastLoad[FastLoadIndex] == WEST)
		{
			RM_FORWARD_GO();
			LM_BACKWARD_GO();
			Turn_Limit = Turn_90Limit;
		}
	}
	else if(This_Dir == EAST)
	{
		if(FastLoad[FastLoadIndex] == SOUTH)
		{
			LM_FORWARD_GO();
			RM_BACKWARD_GO();
			Turn_Limit = Turn_90Limit;
		}
		else if(FastLoad[FastLoadIndex] == WEST)
		{
			LM_FORWARD_GO();
			RM_BACKWARD_GO();
			if(Sensor_data[0] > LS_LOW && Sensor_data[5] > RS_LOW)
				Turn_Limit = Turn_180Limit;//-10;    //�¿��� ���� ������ ���� �ϴ� ���
			else
				Turn_Limit = Turn_180Limit;
		}
		else if(FastLoad[FastLoadIndex] == NORTH)
		{
			RM_FORWARD_GO();
			LM_BACKWARD_GO();
			Turn_Limit = Turn_90Limit;
		}
	}
	else if(This_Dir == SOUTH)
	{
		if(FastLoad[FastLoadIndex] == WEST)
		{
			LM_FORWARD_GO();
			RM_BACKWARD_GO();
			Turn_Limit = Turn_90Limit;
		}
		else if(FastLoad[FastLoadIndex] == NORTH)
		{
			LM_FORWARD_GO();
			RM_BACKWARD_GO();
			if(Sensor_data[0] > LS_LOW && Sensor_data[5] > RS_LOW)
				Turn_Limit = Turn_180Limit;//-10;    //�¿��� ���� ������ ���� �ϴ� ���
			else 
				Turn_Limit = Turn_180Limit;
		}
		else if(FastLoad[FastLoadIndex] == EAST)
		{
			RM_FORWARD_GO();
			LM_BACKWARD_GO();
			Turn_Limit = Turn_90Limit;
		}    
	}
	else if(This_Dir == WEST)
	{
		if(FastLoad[FastLoadIndex] == NORTH)
		{
			LM_FORWARD_GO();
			RM_BACKWARD_GO();
			Turn_Limit = Turn_90Limit;
		}
		else if(FastLoad[FastLoadIndex] == EAST)
		{
			LM_FORWARD_GO();
			RM_BACKWARD_GO();
			if(Sensor_data[0] > LS_LOW && Sensor_data[5] > RS_LOW)
				Turn_Limit = Turn_180Limit;//-10;    //�¿��� ���� ������ ���� �ϴ� ���
			else
				Turn_Limit = Turn_180Limit;
		}
		else if(FastLoad[FastLoadIndex] == SOUTH)
		{
			RM_FORWARD_GO();
			LM_BACKWARD_GO();
			Turn_Limit = Turn_90Limit;
		}
	}
	LM_Sm_Tn_mode = 0;
	RM_Sm_Tn_mode = 0;
	LM_Spd_Limit = 200;
	RM_Spd_Limit = 200;
	Mot_TnStep_Cnt = 1;
	//ȸ������
     LMotor_Inturrupt_Enable();            //���ʸ��� ���ͷ�Ʈ ���
	RMotor_Inturrupt_Enable();            //������ ���� ���ͷ�Ʈ ���
}
/////////////////////////////////////////////////////////////

/////////////////////////////////////////////////////////////
//		������ �ڼ����� �Լ�
//----------------------------------------------------------
void Mouse_Posture_CTR(void)
{
     //���� �����ȵǸ� �����ϸ鼭 �ڼ������� �ϰ�
	if(Sensor_data[1] > 45)
	{
	     LM_Spd_Limit =100-(Sensor_data[1]-45)/2;
		RM_Spd_Limit =100+(Sensor_data[1]-45)/2;
	}
	else if(Sensor_data[4] > 60)
	{
		RM_Spd_Limit =100-(Sensor_data[5]-60)/2;
		LM_Spd_Limit =100+(Sensor_data[5]-60)/2;                   
	}
	else
	{
		LM_Spd_Limit = 100;
		RM_Spd_Limit = 100;
	}
}
/////////////////////////////////////////////////////////////

/////////////////////////////////////////////////////////////
//		���ν���
//----------------------------------------------------------
int main(void)
{ 
     unsigned char this_speed;
     unsigned int tmp;
     unsigned int  i;
     
     MIEXER_Initialize();                  //�ʱ�ȭ
     USART0_Initialize();                  //��� �ʱ�ȭ
     EXINT_Initialize();
     Sensor_Initialize();
     
     Motor_GAL_Tri_Initialize();           //���� Ʈ���ſ� GAL �ʱ�ȭ
     Motor_Initialize();                   //���� �ʱ�ȭ
     MOTOR_OFF();
     LMotor_Inturrupt_Disable();            //���ʸ��� ���ͷ�Ʈ ����
     RMotor_Inturrupt_Disable();            //������ ���� ���ͷ�Ʈ ����
     
     Delay_ms(500);
     LCD_Initialize();
  
     LCD_String(0X80, "-MIEXER-");
     LCD_String(0XC0, "Start!!!");
     
     //�̷����� �ʱ�ȭ
     LCD_String(0X80, "-MIEXER-");
     LCD_String(0XC0, "MazeInit");
     
     //�̷����� �ʱ�ȭ
     for(i=0; i<256; i++)
     {
          Maze_Info[i]=0x00;            //�⺻������ 0���� �ʱ�ȭ
          if((i&0x0f)==0x00)
               Maze_Info[i] |= 0x08;  //�����׵θ�
          if((i&0x0f)==0x0f)
               Maze_Info[i] |= 0x02;  //�������׵θ�
          if((i&0xf0)==0x00)
               Maze_Info[i] |= 0x04;  //�Ʒ��� �׵θ�
          if((i&0xf0)==0xf0)
               Maze_Info[i] |= 0x01;  //���� �׵θ�
     }
       
     this_speed = _LPM(&ACC_TBL[0]);       //���� �������� �ӵ��� ����
  
     LM_THIS_SPD(this_speed);              //���ʸ��� �ʱ�ӵ� ����
     RM_THIS_SPD(this_speed);              //�����ʸ��� �ʱ�ӵ� ����
  
     SEI();                              //�۷ι� ���ͷ�Ʈ ���
     Delay_ms(1000);
     
     //�������� ����� �������� �˻�
     for(i=0; i<6; i++)
     {
          if(Sensor_data[i] > 1)
          {
               LCD_String(0X80, " Sensor ");
               LCD_String(0Xc0, "Check OK");
          }
          else
          {
               LCD_String(0X80, " Sensor ");
               LCD_String(0Xc0, " ERROR! ");
               for(;;);
          }
     }
  	//������ ����
     UpdateWallInfo();
     
     //�ִܰŸ� ����
     FindFastLoad(Goal_Block_Pos[0]);
     
     LM_FORWARD_GO();
     RM_FORWARD_GO();
   
   	//���콺 ���ʳ��� ��Ͽ� ���� ���
   	Maze_Info[This_Block_Pos] &= 0x0f;
	Maze_Info[This_Block_Pos] |= This_Dir; 
	
	Delay_ms(1000);
     MOTOR_ON();
	//LMotor_Inturrupt_Enable();
	//RMotor_Inturrupt_Enable();
	
     //Start Main Loop
     for(;;)
     {
         //LCD �������� ������Ʈ�� ����.
          //if(num_cnt % 50 == 0)
          //{
              LCD_Page_Viwe(LCD_Page_Num);
          //}
          //���콺 ����� ������ ������ ����?
     	if(This_Dir == FastLoad[FastLoadIndex])
     	{
          	//���� ������ ����� ���� ��Ͽ����� �������� ����?
     		if((FastLoad[FastLoadIndex] == FastLoad[FastLoadIndex+1]) || (FastLoad[FastLoadIndex+1] == 0xf0) )
     		{
     			//���콺 �̵��� ����ϰ�
	     		LMotor_Inturrupt_Enable();            //���ʸ��� ���ͷ�Ʈ ���
			     RMotor_Inturrupt_Enable();            //������ ���� ���ͷ�Ʈ ���
		          
     			//1��� �̵��߳�?
     			for(tmp=0; tmp<(OneBlockCntStep * 2); )
	     		{
	     		     //����� �߽ɿ��� �������� ������Ʈ �Ѵ�
	     		     if(tmp==OneBlockCntStep)
	     		          UpdateWallInfo();
	     		          
     				//������⿡ ���� �����ǳ�?
					if((Sensor_data[2] > LF_LOW) && (Sensor_data[3] > RF_LOW))
					{
						//���� �����Ǹ�
						//�ӵ��� ���̰� ������ ���� ���´�
						LM_Spd_Limit = 249;
						RM_Spd_Limit = 249;
						break;
					}
					else
					{
						//���� �����ȵǸ� �����ϸ鼭 �ڼ������� �ϰ�
						Mouse_Posture_CTR();
					}
					tmp = LMot_BlStep_Cnt+RMot_BlStep_Cnt;
				}
				if(tmp<(OneBlockCntStep * 2))
				{
					//���濡 ���� �־����Ƿ�
					//����� �߽ɿ� �ö����� �����Ѵ�
					for( ;tmp < OneBlockCntStep; )
					{
						tmp = LMot_BlStep_Cnt+RMot_BlStep_Cnt;
					}
					//����� �߾ӿ� ����
					//���콺�� �����ϰ�
					LMotor_Inturrupt_Disable();            //���ʸ��� ���ͷ�Ʈ ����
					RMotor_Inturrupt_Disable();            //������ ���� ���ͷ�Ʈ ����
					Delay_ms(1000);
					//�������� �����ϸ�
					UpdateWallInfo();
					
					//���ο� �ִܰ�θ� ã�´�
					FindFastLoad(Goal_Block_Pos[0]);
				}
				else
				{
					//���濡 ���� ���� 1��� �̵������Ƿ�
					//���� ���콺 ��ġ�� �����ϰ�
					if(This_Dir == NORTH)
						This_Block_Pos += 0x10;
					else if(This_Dir == EAST)
						This_Block_Pos += 0x01;
					else if(This_Dir == SOUTH)
						This_Block_Pos -= 0x10;
					else if(This_Dir == WEST)
						This_Block_Pos -= 0x01;
						
					//������Ͽ� ���콺 ���� ������ �����
					Maze_Info[This_Block_Pos] &= 0x0f;
					Maze_Info[This_Block_Pos] |= This_Dir;   
					
					//�������� �����ϸ�
					//UpdateWallInfo();
					
					//�ִܰ�� ����迭�� ���� �ε����� ����
					FastLoadIndex++;
					
					LMot_BlStep_Cnt=0;
     				RMot_BlStep_Cnt=0;
     				//LMotor_Inturrupt_Disable();            //���ʸ��� ���ͷ�Ʈ ����
					//RMotor_Inturrupt_Disable();            //������ ���� ���ͷ�Ʈ ����
     				//Delay_ms(1000);
					//LMotor_Inturrupt_Enable();            //���ʸ��� ���ͷ�Ʈ ����
					//RMotor_Inturrupt_Enable();
				}	
     		}
     		//���� ������ ����� ���� ��������� �ٸ���
     		//�������� �� �� �����Ƿ�
     		else
     		{
     			//���콺 �̵��� ����ϰ�
	     		LMotor_Inturrupt_Enable();            //���ʸ��� ���ͷ�Ʈ ���
			     RMotor_Inturrupt_Enable();            //������ ���� ���ͷ�Ʈ ���
     			//�켱 �������� �ϱ��� ������ ���� ������ �� 
     			//���� ��ġ�� ���� ���� ��ġ���� �����Ѵ�
     			if((LMot_BlStep_Cnt + RMot_BlStep_Cnt)<100)
     			{
	     			for(tmp=0;tmp <100;)
     				{
     					//�����̹Ƿ� ���� �ڼ������� �ϰ�
     					//�̵��ϸ鼭 ���콺 �ڼ��� �ٷ� ���
     					Mouse_Posture_CTR();
						tmp = LMot_BlStep_Cnt + RMot_BlStep_Cnt;
					}
				}
								 		
				//������ġ�� �����Ƿ� ���濡 ���� �ֳ� �˻�
				if((Sensor_data[2] < LF_LOW) && (Sensor_data[3] < RF_LOW))
				{
					//���濡 ���� �����Ƿ� �������� ����
					//�켱 �ϰ��� �������� ������ �̵��ϰ�
					for(tmp=0;tmp < 2;)
					{
						//�������� ������ �������̸�
	     				if(((((This_Dir>>1) & 0x0f)<<4)|(This_Dir>>1)) == FastLoad[FastLoadIndex+1])
     					{
     						//�켱 ���� �밢������ �����Ǹ� ��� 1����
     						if(tmp==0 && (Sensor_data[1] > LC_LOW))
     							tmp++;
     						if(tmp==1 && (Sensor_data[0] > LS_LOW))
     							tmp++;
     					}
     					//�������� �ƴϸ� ������
     					else
     					{
     						if(tmp==0 && (Sensor_data[4] > RC_LOW))
     							tmp++;
     						if(tmp==1 && (Sensor_data[5] > RS_LOW))
     							tmp++;
     					}
						//�����ϸ鼭 �ڼ������� �ϰ�
						Mouse_Posture_CTR();
     				}
     				//�ϰ��� ��尡 �Ǿ�����
     				//���� ���⿡ ���� �ִ��� �˻�
     				for(tmp=0;tmp == 0;)
     				{
     					//�������� ������ �������̸�
	     				if(((((This_Dir>>1) & 0x0f)<<4)|(This_Dir>>1)) == FastLoad[FastLoadIndex+1])
     					{
     						if(Sensor_data[1] < LC_LOW)
     						{
     							//���� �����Ƿ� �������� ����
     							tmp++;
     						}
     						else if((LMot_BlStep_Cnt + RMot_BlStep_Cnt)>(OneBlockCntStep * 2))
     						{
     							tmp=2;
     						}
     					}
     					else
     					{
     						if(Sensor_data[4] < RC_LOW)
     						{
     							//���� �����Ƿ�
     							tmp++;
     						}
     						else if((LMot_BlStep_Cnt + RMot_BlStep_Cnt)>(OneBlockCntStep * 2))
     						{
     							tmp=2;
     						}
     					}
     					//�����ϸ鼭 �ڼ������� �ϰ�
						Mouse_Posture_CTR();
     				}
     				//���������̸�
     				if(tmp==1)
     				{
     					//��������� ���� ������ �����
	     				Maze_Info[This_Block_Pos] &= 0x0f;
						Maze_Info[This_Block_Pos] |= FastLoad[FastLoadIndex++];
										
     					//��ǥ�� �����ϰ�
	                    	if(This_Dir == NORTH)
			     	     	This_Block_Pos += 0x10;
	          			else if(This_Dir == EAST)
     	          	     	This_Block_Pos += 0x01;
				          else if(This_Dir == SOUTH)
               				This_Block_Pos -= 0x10;
                    		else if(This_Dir == WEST)
				     		This_Block_Pos -= 0x01;
				                    	
     					for(;;)
     					{
	     					//���������� �̵�
     						for(tmp=0;tmp==0;)
     						{
     							//�������϶�
     							if(((((This_Dir>>1) & 0x0f)<<4)|(This_Dir>>1)) == FastLoad[FastLoadIndex])
     							{
     								if(Sensor_data[0] < LS_LOW)
     								{
				          	          	//�������� ����
	     								//UpdateWallInfo();
										
										//��������� ���� ������ �����
	     								Maze_Info[This_Block_Pos] &= 0x0f;
										Maze_Info[This_Block_Pos] |= FastLoad[FastLoadIndex];
										
	     								//���� ��ǥ�� �ٽ� ��ġ�� �����ϰ�
	          	          				if(This_Dir == NORTH)
			     		               		This_Block_Pos -= 0x01;
          					          	else if(This_Dir == EAST)
               	     						This_Block_Pos += 0x10;
			                    			else if(This_Dir == SOUTH)
               			     				This_Block_Pos += 0x01;
                    						else if(This_Dir == WEST)
				                    			This_Block_Pos -= 0x10;
					                    			
     									//�Ͻ���
     									LM_Sm_Tn_mode = 1;
               	     					RM_Sm_Tn_mode = 2;
     									tmp++;
     								}
     							}
     							//�������϶�
	     						else	
     							{
     								if(Sensor_data[5] < RS_LOW)
     								{
					                    	
					                    	//�������� ����
	     								//UpdateWallInfo();
										
										//��������� ���� ������ �����
	     								Maze_Info[This_Block_Pos] &= 0x0f;
										Maze_Info[This_Block_Pos] |= FastLoad[FastLoadIndex];
							
		     							//���� ��ǥ�� �ٽ� ��ġ�� �����ϰ�
		                    				if(This_Dir == NORTH)
				     	               		This_Block_Pos += 0x01;
          					          	else if(This_Dir == EAST)
               		     					This_Block_Pos -= 0x10;
			          	          		else if(This_Dir == SOUTH)
               				     			This_Block_Pos -= 0x01;
                    						else if(This_Dir == WEST)
				                    			This_Block_Pos += 0x10;
	     							
     									//�Ͻ���
	     								LM_Sm_Tn_mode = 2;
     	               					RM_Sm_Tn_mode = 1;
     									tmp++;
     								}
     							}
     						}
     						//������ ���� ���������� ���
             					for(;tmp>0; )
	              				{
     	      				     tmp = LM_Sm_Tn_mode+RM_Sm_Tn_mode;
          	   				}
										
          	   				//���콺 ������ ����
          	   				This_Dir=FastLoad[FastLoadIndex++];
          	   				
	          	          	//������ ���� ������ ���������� �˻�
	          	          	if(This_Dir==FastLoad[FastLoadIndex])
	          	          	{
	          	          		tmp=1;
	          	          		break;
	          	          	}
	          	          	else
	          	          	{
	          	          	 	if(((((This_Dir>>1) & 0x0f)<<4)|(This_Dir>>1)) == FastLoad[FastLoadIndex])
     							{
     								if(Sensor_data[1] > LC_LOW)
     								{
     									tmp=2;
     									break;
     								}
     							}
     							else
     							{
     								if(Sensor_data[4] > RC_LOW)
     								{
     									tmp=2;
     									break;
     								}
     							}
     						}
	                    	}
	                    	//�������� ��带 ������� �����̹Ƿ�
	                    	//���ʹ��� ����ī��Ʈ ����
	                    	LMot_BlStep_Cnt=0;
	                    	RMot_BlStep_Cnt=0;
	                    		
	                    	//���� �־ ���� �� ���� ������ 
	                    	//��� �߾ӱ��� ����
	                    	if(tmp==2)
	                    	{
							//�������� �����ϸ�
							UpdateWallInfo();
						
							//���ο� �ִܰ�θ� ã�´�
							FindFastLoad(Goal_Block_Pos[0]);
							
							//���ο� ��θ� ã�� ���� �������� ���콺 ������ �ٸ��� 
							if(This_Dir!=FastLoad[FastLoadIndex])
							{
							     //�켱 �ӵ��� ���߰�
     							LM_Spd_Limit = 249;
	                             		RM_Spd_Limit = 249;
     	                              
     	                              tmp = LMot_BlStep_Cnt+RMot_BlStep_Cnt;
     	                              
		     					for( ;tmp < OneBlockCntStep; )
			     				{
				     				tmp = LMot_BlStep_Cnt+RMot_BlStep_Cnt;
					     		}
						     	//����� �߾ӿ� ������
							     //���콺�� �����ϰ�
     							LMotor_Inturrupt_Disable();            //���ʸ��� ���ͷ�Ʈ ����
	     						RMotor_Inturrupt_Disable();            //������ ���� ���ͷ�Ʈ ����
							
							     Delay_ms(1000);
					          }
	                    	}	
     				}
     				//��������̸�
     				else if(tmp==2)
     				{
                         	LMot_BlStep_Cnt=0;
                         	RMot_BlStep_Cnt=0;
                         	//���� ��ġ�� �����ϰ�
	                    	if(This_Dir == NORTH)
			     	    		This_Block_Pos += 0x10;
	                    	else if(This_Dir == EAST)
     	     				This_Block_Pos += 0x01;
			         		else if(This_Dir == SOUTH)
                    			This_Block_Pos -= 0x10;
               			else if(This_Dir == WEST)
			             		This_Block_Pos -= 0x01;
			             	
			             	//LCD_String(0X80, "-MIEXER-");
     					//LCD_String(0XC0, "Stop!!!!");	
     					
     					//�������� �����ϸ�
						UpdateWallInfo();
						
						//���ο� �ִܰ�θ� ã�´�
						FindFastLoad(Goal_Block_Pos[0]);
						
						if(This_Dir != FastLoad[FastLoadIndex])
						{
						     //�켱 �ӵ��� ���߰�
						     LM_Spd_Limit = 249;
                         	     RM_Spd_Limit = 249;
                         	
						     for( ;tmp < OneBlockCntStep; )
						     {
						     	tmp = LMot_BlStep_Cnt+RMot_BlStep_Cnt;
						     }
						
					     	//����� �߾ӿ� ������
						     //���콺�� �����ϰ�
						     LMotor_Inturrupt_Disable();            //���ʸ��� ���ͷ�Ʈ ����
						     RMotor_Inturrupt_Disable();            //������ ���� ���ͷ�Ʈ ����
						
						     Delay_ms(1000);
				          }
     				}
				}
				//���濡 ���� �ֱ⶧���� ��� �߽ɱ��� �̵��� ó��
				else
				{
				     if(LMot_BlStep_Cnt+RMot_BlStep_Cnt < OneBlockCntStep)
				     {
	     				//�켱 �ӵ��� ���߰�
		     			LM_Spd_Limit = 249;
                              RM_Spd_Limit = 249;
                         
				     	for( ;tmp < OneBlockCntStep; )
					     {
						     tmp = LMot_BlStep_Cnt+RMot_BlStep_Cnt;
					     }
				     }
					//����� �߾ӿ� ������
					//���콺�� �����ϰ�
					LMotor_Inturrupt_Disable();            //���ʸ��� ���ͷ�Ʈ ����
					RMotor_Inturrupt_Disable();            //������ ���� ���ͷ�Ʈ ����
					
					//�������� �����ϸ�
					UpdateWallInfo();
					
					//���ο� �ִܰ�θ� ã�´�
					FindFastLoad(Goal_Block_Pos[0]);
					
					Delay_ms(1000);
				}
     		}
     	}
          else
          {
          	//���� ���콺 ����� ��������� Ʋ���Ƿ�
          	//ȸ���Ѵ�
          	//���ڸ� ������ �����ϰ�
     		NormalTurnInit();
     		for(;Mot_TnStep_Cnt != 0;)
     		{
     			//��� �����Ѵ�.
     			//180�� ȸ���̸鼭 ���ʿ� ���� ������
		     	if(Turn_Limit != Turn_180Limit && Turn_Limit > Turn_90Limit)
          		{
		         		if((Mot_TnStep_Cnt >= Turn_Limit) && (Sensor_data[0]+Sensor_data[5] > 145) && (Sensor_data[1]+Sensor_data[4] > 80))
          	     	{
		              		Mot_TnStep_Cnt = 0;
          	  		}
		         	}
          		//���ʿ� ���� �ϳ��� ������
		         	else
		         	{
          			if(Mot_TnStep_Cnt >= Turn_Limit)
               		{
		                   	Mot_TnStep_Cnt = 0;
          	     	}
         			}
     		}
     		//���� ������ ������ �����Ѵ�
     		LMotor_Inturrupt_Disable();            //���ʸ��� ���ͷ�Ʈ ����
			RMotor_Inturrupt_Disable();            //������ ���� ���ͷ�Ʈ ����
               Delay_ms(50);
		     LM_FORWARD_GO();
          	RM_FORWARD_GO();
               Turn_Limit = 0;
		     Mot_TnStep_Cnt = 0;
          	LM_Spd_Limit = 100;
               RM_Spd_Limit = 100;
		     LMot_BlStep_Cnt=110;
          	RMot_BlStep_Cnt=110;
               This_Dir = FastLoad[FastLoadIndex];
		     Maze_Info[This_Block_Pos] &= 0x0f;
          	Maze_Info[This_Block_Pos] |= This_Dir;
               //LMotor_Inturrupt_Enable();            //���ʸ��� ���ͷ�Ʈ ����
		     //RMotor_Inturrupt_Enable(); 
          }
     }//End Main Loop     
}//End Main Function
/////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////
//    ����ġ ���ͷ�Ʈ ó����ƾ ����
//-----------------------------------------------------------------
#pragma vector=INT4_vect
__interrupt void SW_INT(void)
{
  unsigned char sw_data;
  sw_data = PINE & 0x60;                     //����ġ ������ �޴´�
  if(sw_data == SW_RD)                       //������ �Ʒ���ư ������
  {
     if(LCD_Page_Num > 0)                    //LCD��� �������� ��������
     {
       LCD_Page_Num--;  
       LCD_Page_Viwe(LCD_Page_Num); 
     }
  }
  else if(sw_data == SW_RU)                  //������ �� ����ġ ������
  {
     if(LCD_Page_Num  < Lcd_Page_Limit)
     {
          LCD_Page_Num++;                    //LCD�������� ������������
          LCD_Page_Viwe(LCD_Page_Num); 
     }
  }
  else if(sw_data == SW_LD)                  //���� �Ʒ���ư ������ 
  {
     Edit_Data_in_Lcd(LCD_Page_Num, 0);      //���� LCD�� ��µǾ� �ִ� �����������Ͱ� ����
  }
  else if(sw_data == SW_LU)                  //���� �� ��ư�� ������
  {
     Edit_Data_in_Lcd(LCD_Page_Num, 1);      //���� LCD�� ǥ�õ� ������ ������ ����
  }
}
////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////
//    ���ʸ��� ���ͷ�Ʈ ó����ƾ ����
//-----------------------------------------------------------------
#pragma vector=TIMER1_COMPA_vect         //���ʸ��� Ÿ�̸� ���ͷ�Ʈ ó����ƾ
__interrupt void LMotor_Int_A(void)
{
     unsigned char this_speed;
  
     LM_PULSE_CLR();                        //LMotor_Int_B ���ͷ�Ʈ ��ƾ���� ���� High�޽��� 100us �Ŀ� Low�� �����
  
     if(LM_Sm_Tn_mode==1)
     {
          this_speed = _LPM(&Sm_Tbl_In[LM_Smpd_index]);    //���� ������ ���� �ӵ��� �ε�
          if(LM_Smpd_index < 91)
          {
               LM_Smpd_index++;
               this_speed = _LPM(&Sm_Tbl_In[LM_Smpd_index]);
          }
          else
          {
               LM_Sm_Tn_mode = 0;
               LM_Smpd_index = 0;
          }
          
     }
     else if(LM_Sm_Tn_mode == 2)
     {
          this_speed = _LPM(&Sm_Tbl_Out[LM_Smpd_index]);    //���� ������ ���� �ӵ��� �ε�
          if(LM_Smpd_index < 243)
          {
               LM_Smpd_index++;
               this_speed = _LPM(&Sm_Tbl_Out[LM_Smpd_index]);
          }
          else
          {
               LM_Sm_Tn_mode = 0;
               LM_Smpd_index = 0;
          }
     }
     else
     {
          this_speed = _LPM(&ACC_TBL[LM_Spd_index]);    //���� ������ ���� �ӵ��� �ε�
    
          if(this_speed > LM_Spd_Limit)         //����ӵ� & ��ǥ�ӵ� ��
          {
               LM_Spd_index++;                     //��ǥ�ӵ� �ʰ��� �ӵ� ����
          }
          else if(this_speed < LM_Spd_Limit)
          {
               LM_Spd_index--;                     //��ǥ�ӵ� �̴޽� �ӵ� ����
          }
  
     	LMot_BlStep_Cnt++;
  
  		if(Turn_Limit > 0)
  		{
     		Mot_TnStep_Cnt++;
  		}
  		
          this_speed = _LPM(&ACC_TBL[LM_Spd_index]);  //�ӵ� ����
     }
     LM_Spd_Now = this_speed;
     LM_THIS_SPD(this_speed);                    //�����ӵ� ����
 
     TCNT1H = 0x00;                              //Ÿ�̸� ����
     TCNT1L = 0x00;
}

#pragma vector=TIMER1_COMPB_vect
__interrupt void LMotor_Int_B(void)
{ 
  LM_PULSE_SET();                           //���ʸ��� ��Ÿ�� �޽�  
  
  //TX_Char('L');
}
///////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////
//    �����ʸ��� ���ͷ�Ʈ ó����ƾ ����
//-----------------------------------------------------------------
#pragma vector=TIMER3_COMPA_vect            //������ ���� Ÿ�̸� ���ͷ�Ʈ ó����ƾ
__interrupt void RMotor_Int_A(void)
{
     unsigned char this_speed;
  
     RM_PULSE_CLR();                            //RMotor_Int_B ���ͷ�Ʈ ��ƾ���� ���� High�޽��� 100us �Ŀ� Low�� �����
     
     if(RM_Sm_Tn_mode==1)
     {
          this_speed = _LPM(&Sm_Tbl_In[RM_Smpd_index]);    //���� ������ ���� �ӵ��� �ε�
          if(RM_Smpd_index < 91)
          {
               RM_Smpd_index++;
               this_speed = _LPM(&Sm_Tbl_In[RM_Smpd_index]);
          }
          else
          {
               RM_Sm_Tn_mode = 0;
               RM_Smpd_index = 0;
          }
          
     }
     else if(RM_Sm_Tn_mode == 2)
     {
          this_speed = _LPM(&Sm_Tbl_Out[RM_Smpd_index]);    //���� ������ ���� �ӵ��� �ε�
          if(RM_Smpd_index < 243)
          {
               RM_Smpd_index++;
               this_speed = _LPM(&Sm_Tbl_Out[RM_Smpd_index]);
          }
          else
          {
               RM_Sm_Tn_mode = 0;
               RM_Smpd_index = 0;
          }
     }
     else
     {
          this_speed = _LPM(&ACC_TBL[RM_Spd_index]);
  
          if(this_speed > RM_Spd_Limit)
          {
               RM_Spd_index++;
          }
          else if(this_speed < RM_Spd_Limit)
          {
               RM_Spd_index--;
          }
  		
     	RMot_BlStep_Cnt++;
  
          this_speed = _LPM(&ACC_TBL[RM_Spd_index]);
     }
     RM_THIS_SPD(this_speed);
     RM_Spd_Now = this_speed;
     TCNT3H = 0x00;
     TCNT3L = 0x00;
}

#pragma vector=TIMER3_COMPB_vect
__interrupt void RMotor_Int_B(void)
{
  RM_PULSE_SET();                               //������ ���� ��Ÿ�� �޽� 
  
  //TX_Char('R');
}
////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////
//    A/D��ȯ ���ͷ�Ʈ ó����ƾ ����
//-----------------------------------------------------------------
#pragma vector=ADC_vect
__interrupt void Sensor_Int(void)
{
  unsigned char tmp_num;
  tmp_num = 5 - Sen_sel;
  Sensor_data[tmp_num] = (ADC >> 2);     //���� ������ ����
  Sen_sel++;
  if(Sen_sel == 6)
  {
   	Sen_sel = 0;
     num_cnt++;
     if(num_cnt==65535)
     {
          num_cnt = 0;
     }
  }
  Sen_El_Start(Sen_sel);                //���� ���� �߱� ����
}
////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////
//    �ø��� ��� ������ ���� ���ͷ�Ʈ ó����ƾ ����
//-----------------------------------------------------------------
#pragma vector=USART0_RXC_vect
__interrupt void Sensor_data_receive(void)
{
  unsigned char sen_data;
    
  sen_data = SERIAL_BUFF;
  
  if(sen_data == 'S')
  {
    MOTOR_ON();
    LMotor_Inturrupt_Enable();            //���ʸ��� ���ͷ�Ʈ ���
    RMotor_Inturrupt_Enable();            //������ ���� ���ͷ�Ʈ ���
  }
  else if(sen_data == 'E')
  {
    LMotor_Inturrupt_Disable();            //���ʸ��� ���ͷ�Ʈ ����
    RMotor_Inturrupt_Disable();            //������ ���� ���ͷ�Ʈ ����
    MOTOR_OFF();
  }
  else if(sen_data == 'F')
  {
    LM_FORWARD_GO();                      //���ʸ��� ������ ����
    RM_FORWARD_GO();                      //�����ʸ��� ������ ����
  }
  else if(sen_data == 'B')
  {
    LM_BACKWARD_GO();                     //���ʸ��� �ڷ� ����
    RM_BACKWARD_GO();                     //�����ʸ��� �ڷ� ����
  }
  else if(sen_data == 'D')
  {
    Sen_sel = 0;                         //���������� �ޱ� ����
  }
  else if(Sen_sel < 6)
  {
    Sensor_data[Sen_sel++] = sen_data;    //���� ������
  }
}
////////////////////////////////////////////////////////////////////

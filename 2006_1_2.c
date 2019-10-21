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

unsigned int LM_Spd_index = 0;          //왼쪽 모터 가속도 테이블 속도 인덱스값 저장
unsigned int RM_Spd_index = 0;          //오른쪽 모터 가속도 테이블 속도 인덱스값 저장
unsigned int LM_Smpd_index = 0;         //왼쪽 모터 스무스턴 테이블 속도 인덱스
unsigned int RM_Smpd_index = 0;         //오른쪽 모터 스무스턴 테이블 속도 인덱스
unsigned char LM_Sm_Tn_mode = 0;        //현재 스무스턴 진입인지?
unsigned char RM_Sm_Tn_mode = 0;        //현재 스무스턴 진입인지?
unsigned char LM_Spd_Limit = 100;       //왼쪽모터 최고속도 지정
unsigned char RM_Spd_Limit = 100;       //오른쪽 모터 최고속도 지정
unsigned char LM_Spd_Now = 0;            //왼쪽 모터 현재 속도
unsigned char RM_Spd_Now = 0;            //오른쪽 모터 현재 속도
unsigned char Mot_TnStep_Cnt = 0;       //모터 제자리 턴할때 참조할 스텝 카운트 변수
unsigned int LMot_BlStep_Cnt = 110;       //마우스가 1블록 갔는지 검사를 위한 스텝 카운트 변수
unsigned int RMot_BlStep_Cnt = 110;       //마우스가 1블록 갔는지 검사를 위한 스텝 카운트 변수

unsigned char This_Block_Pos = 0x00;    //마우스 현제 블록의 위치
unsigned char Goal_Block_Pos[4] = {0x28, 0x23, 0x23, 0x23}; //골블록 위치
unsigned char This_Dir = NORTH;          //상위 4비트 중 1000=>서, 0100=>남, 0010=>동, 0001=>북
unsigned char Maze_Info[256];           //미로 벽 정보 & 지나간 흔적 저장 변수
                                        //상위 4비트 지나갔던 방위, 하위 4비트는 벽정보
unsigned char FastLoad[256];            //최단거리 주행정보(방위) 저장변수 
unsigned char FastLoadIndex=0;
unsigned char BlockQueue[256];          //미로 초단거리 검사용 queue
unsigned char BlockWeight[256];

unsigned char Turn_Limit = 0; 
unsigned char Turn_90Limit = 80;           //턴 스텝 수
unsigned char Turn_180Limit = 160;         //턴 스텝 수

unsigned char Sensor_data[6]={255, 255, 255, 255, 255, 255};    //센서 데이터
unsigned char Sen_sel = 0;             //센서 데이터 데이터 저장 인덱스

unsigned char L90Tn_Offset = 20;        //왼쪽90도 턴 좌우센서 비교 오프셋
unsigned char R90Tn_Offset = 20;        //오른쪽 90도 턴 좌우센서 비교 오프셋
unsigned char FWall_Chk_Offset = 50;    //전방 진입시 정지할 기준 센서값
//unsigned char mode = 0;                      
unsigned int  num_cnt = 0;
//unsigned char menu_ok = 0;
//unsigned char menu_ok_cnt = 0;
unsigned char LCD_Page_Num = 4;         //현제 LCD데이터 출력 페이지값 저장

/////////////////////////////////////////////////////////////
//		데이터 LCD에 출력 함수
//-----------------------------------------------------------
void LCD_Page_Viwe(unsigned char page)
{    
     LCD_Clear();
     //좌우 측면 센서 데이터 표시
     if(page == 0)  
     {
          LCD_String(0X80, " LS ");
          LCD_String(0X84, " RS ");
          LCD_DatatoDec(0xc0, Sensor_data[0]);
          LCD_DatatoDec(0xc4, Sensor_data[5]);
     }
     //좌우 대각선 센서 데이터 표시
     else if(page == 1)
     {
          LCD_String(0X80, " LC ");
          LCD_String(0X84, " RC ");
          LCD_DatatoDec(0xc0, Sensor_data[1]);
          LCD_DatatoDec(0xc4, Sensor_data[4]);
     }
     //좌우 전면 센서 데이터 표시
     else if(page == 2)
     {
          LCD_String(0X80, " LF ");
          LCD_String(0X84, " RF ");
          LCD_DatatoDec(0xc0, Sensor_data[2]);
          LCD_DatatoDec(0xc4, Sensor_data[3]);
     }
     //현제 마우스 방위 표시
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
      //현제 마우스 위치 표시
     else if(page == 4)
     {
          LCD_String(0X80, "ThisPos!");
          LCD_DatatoHex2(0xc2, This_Block_Pos);
     }
     //왼쪽모터 블럭 스텝 카운트 표시
     else if(page == 5)
     {
          LCD_String(0X80, "LMotStep");
          LCD_String(0Xc0, "Cnt:");
          LCD_DatatoDec(0xc5, LMot_BlStep_Cnt);
     }
     //오른쪽모터 블록 스텝 카운트 표시
     else if(page == 6)
     {
          LCD_String(0X80, "RMotStep");
          LCD_String(0Xc0, "Cnt:");
          LCD_DatatoDec(0xc5, RMot_BlStep_Cnt);
     }
     //미로 정보 표시
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
     //블록웨이트 정보 확인
     else if(page == 8)
     {
          LCD_String(0X80, "BlWeight");
          LCD_DatatoHex2(0xc0, This_Block_Pos);
          LCD_DatatoDec(0xc5, BlockWeight[This_Block_Pos]);
     }
     //최단거리 경로 표시
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
     //모터 on off 표시
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
     //90도 턴 리미트 값 표시
     else if(page == 11)
     {
          LCD_String(0X80, "T90Limit");
          LCD_DatatoDec(0xc3, Turn_90Limit);
     }
     //180도 턴 리미트 값 표시
     else if(page == 12)
     {
          LCD_String(0X80, "T180Limit");
          LCD_DatatoDec(0xc3, Turn_180Limit);
     }
     //왼쪽 90도 턴 좌우센서 비교 오프셋
     else if(page == 13)
     {
          LCD_String(0X80, "LT90_Off");
          LCD_DatatoDec(0xc3, L90Tn_Offset);     
     }
     //오른쪽 90도 턴 좌우센서 비교 오프셋
     else if(page == 14)
     {
          LCD_String(0X80, "RT90_Off");
          LCD_DatatoDec(0xc3, R90Tn_Offset);
     }
     //전방 벽 감지시 정지설정 값 표시
     else if(page ==15)
     {
          LCD_String(0X80, "FWallChk");
          LCD_DatatoDec(0xc3, FWall_Chk_Offset);
     }
     //왼쪽모터 스피드 리미트 값 표시
     else if(page == 16)
     {
          LCD_String(0X80, "LM Speed");
          LCD_String(0Xc0, "Lmt->");
          LCD_DatatoDec(0xc5, LM_Spd_Limit);
     }
     //오른쪽 모터 스피드 리미트 값 표시
     else if(page == 17)
     {
          LCD_String(0X80, "RM Speed");
          LCD_String(0Xc0, "Lmt->");
          LCD_DatatoDec(0xc5, RM_Spd_Limit);
     }
     
}
/////////////////////////////////////////////////////////////

/////////////////////////////////////////////////////////////
//		LCD를 통한 데이터 변경 함수
//-----------------------------------------------------------
void Edit_Data_in_Lcd(unsigned char page, unsigned char UpDn)
{
     if(page > 6)
     {
          //미로정보 확인
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
          //블록 웨이트 확인용
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
          //최단경로 확인용
          else if(page==9)
          {
                if(UpDn == 0 && FastLoadIndex > 0)
                    FastLoadIndex--;
                else if(UpDn == 1 && FastLoadIndex < 0xff)
                    FastLoadIndex++;
          }
          //모터 on off 컨트롤
          else if(page==10)
          {
               if((PINE & 0x08)== 0)
               {
                    Delay_ms(300);
                    MOTOR_ON();
                    LMotor_Inturrupt_Enable();            //왼쪽모터 인터럽트 허용
                    RMotor_Inturrupt_Enable();            //오른쪽 모터 인터럽트 허용
               }
               else
               {
                    LMotor_Inturrupt_Disable();            //왼쪽모터 인터럽트 허용
                    RMotor_Inturrupt_Disable();            //오른쪽 모터 인터럽트 허용
                    MOTOR_OFF();
               }
          }
          //90도 턴 리미트 값 변경
          else if(page==11)
          {
               if(UpDn == 0 && Turn_90Limit > 0)
                    Turn_90Limit--;
               else if(UpDn == 1 && Turn_90Limit < 255)
                    Turn_90Limit++;
          }
          //180도 턴 리미트 값 변경
          else if(page==12)
          {
               if(UpDn == 0 && Turn_180Limit > 0)
                    Turn_180Limit--;
               else if(UpDn == 1 && Turn_180Limit < 255)
                    Turn_180Limit++;
          }
          //왼쪽 90도 턴 오프셋값 조정
          else if(page==13)
          {
               if(UpDn == 0 && L90Tn_Offset > 0)
                    L90Tn_Offset--;
               else if(UpDn == 1 && L90Tn_Offset < 255)
                    L90Tn_Offset++;
          }
          //오른쪽 90도 턴 오프셋값 조정
          else if(page==14)
          {
               if(UpDn == 0 && R90Tn_Offset > 0)
                    R90Tn_Offset--;
               else if(UpDn == 1 && R90Tn_Offset < 255)
                    R90Tn_Offset++;
          }
          //전방 벽체크후 정지 오프셋 값 조정
          else if(page==15)
          {
               if(UpDn == 0 && FWall_Chk_Offset > 0)
                    FWall_Chk_Offset--;
               else if(UpDn == 1 && FWall_Chk_Offset < 255)
                    FWall_Chk_Offset++;
          }
          //왼쪽모터 스피드 리미트 값 조정
          else if(page==16)
          {
               if(UpDn == 0 && LM_Spd_Limit > 0)
                    LM_Spd_Limit--;
               else if(UpDn == 1 && LM_Spd_Limit < 255)
                    LM_Spd_Limit++;
          }
          //오른쪽 모터 스피드 리미트 값 조정
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
//		최단거리 탐색 함수
//-----------------------------------------------------------
void FindFastLoad(unsigned char pos)
{
     unsigned char head=1, tail=0;
     unsigned char Weight=0;
     unsigned int j;
     unsigned char tmp_Dir;
      
     for(j=0; j<256; j++)
     {
          BlockWeight[j]=0xff;     //모든 블록 웨이트 초기화
     }

     LCD_String(0X80, "Updating");
     LCD_String(0XC0, "BlWeight");
    
     //우선 모든 블록의 골로부터의 거리웨이트를 만든다
     //맨처음 시작할 위치가 골지점이므로 큐에 집어넣는다
     BlockQueue[0] = pos;
     BlockWeight[pos] = 0;
     for( ; head != tail ; )
     {
          pos = BlockQueue[tail++];
          Weight = BlockWeight[pos];
          //현제기준 블록의 상, 하, 좌, 우측에 벽이 없고, 웨이트값이 더 크면 큐에 집어 넣고 웨이트를 갱신
          //위쪽 블록 검사
          if((Maze_Info[pos]&0x01)!=0x01)
          {
               if(BlockWeight[pos+0x10] > Weight + 1)
               {
                    BlockQueue[head++] = pos+0x10;
                    BlockWeight[pos+0x10] = Weight + 1;
               }
          }
          //우측 블록 검사
          if((Maze_Info[pos]&0x02)!=0x02)
          {
               if(BlockWeight[pos+0x01] > Weight+1)
               {
                    BlockQueue[head++] = pos+0x01;
                    BlockWeight[pos+0x01] = Weight + 1;
               }
          }
          //아래쪽 블록 검사
          if((Maze_Info[pos]&0x04)!=0x04)
          {
               if(BlockWeight[pos-0x10] > Weight+1)
               {
                    BlockQueue[head++] = pos-0x10;
                    BlockWeight[pos-0x10] = Weight + 1;
               }
          }
          //좌측 블록 검사
          if((Maze_Info[pos]&0x08)!=0x08)
          {
               if(BlockWeight[pos-0x01] > Weight+1)
               {
                    BlockQueue[head++] = pos-0x01;
                    BlockWeight[pos-0x01] = Weight + 1;
               }
          }
     }

    	//미로정보 초기화
     LCD_String(0X80, " Search ");
     LCD_String(0XC0, "FastLoad");
      //빠른길 탐색해서 배열에 저장하기
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
   	     //우선 진행방향으로 검사
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
   	//마지막 골표시를 한다.
   	FastLoad[head]=0xf0;
	FastLoadIndex = 0;
     LCD_String(0X80, "FastLoad");
     LCD_String(0XC0, " FindOK ");
}
/////////////////////////////////////////////////////////////

/////////////////////////////////////////////////////////////
//		벽정보 업데이트 함수
//-----------------------------------------------------------
void UpdateWallInfo(void)
{
	//현제 블록의 좌우측 벽정보를 갱신
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
//		회전설정
//----------------------------------------------------------
void NormalTurnInit(void)
{
	LMotor_Inturrupt_Disable();            //왼쪽모터 인터럽트 금지
	RMotor_Inturrupt_Disable();            //오른쪽 모터 인터럽트 금지
	Delay_ms(1000);
	//최단거리 방향으로 회전설정
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
				Turn_Limit = Turn_180Limit;//-10;    //좌우측 벽이 있을때 턴을 하는 경우
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
				Turn_Limit = Turn_180Limit;//-10;    //좌우측 벽이 있을때 턴을 하는 경우
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
				Turn_Limit = Turn_180Limit;//-10;    //좌우측 벽이 있을때 턴을 하는 경우
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
				Turn_Limit = Turn_180Limit;//-10;    //좌우측 벽이 있을때 턴을 하는 경우
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
	//회전시작
     LMotor_Inturrupt_Enable();            //왼쪽모터 인터럽트 허용
	RMotor_Inturrupt_Enable();            //오른쪽 모터 인터럽트 허용
}
/////////////////////////////////////////////////////////////

/////////////////////////////////////////////////////////////
//		직진시 자세제어 함수
//----------------------------------------------------------
void Mouse_Posture_CTR(void)
{
     //벽이 감지안되면 진행하면서 자세보정을 하고
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
//		메인시작
//----------------------------------------------------------
int main(void)
{ 
     unsigned char this_speed;
     unsigned int tmp;
     unsigned int  i;
     
     MIEXER_Initialize();                  //초기화
     USART0_Initialize();                  //통신 초기화
     EXINT_Initialize();
     Sensor_Initialize();
     
     Motor_GAL_Tri_Initialize();           //모터 트리거용 GAL 초기화
     Motor_Initialize();                   //모터 초기화
     MOTOR_OFF();
     LMotor_Inturrupt_Disable();            //왼쪽모터 인터럽트 금지
     RMotor_Inturrupt_Disable();            //오른쪽 모터 인터럽트 금지
     
     Delay_ms(500);
     LCD_Initialize();
  
     LCD_String(0X80, "-MIEXER-");
     LCD_String(0XC0, "Start!!!");
     
     //미로정보 초기화
     LCD_String(0X80, "-MIEXER-");
     LCD_String(0XC0, "MazeInit");
     
     //미로정보 초기화
     for(i=0; i<256; i++)
     {
          Maze_Info[i]=0x00;            //기본적으로 0으로 초기화
          if((i&0x0f)==0x00)
               Maze_Info[i] |= 0x08;  //왼쪽테두리
          if((i&0x0f)==0x0f)
               Maze_Info[i] |= 0x02;  //오른쪽테두리
          if((i&0xf0)==0x00)
               Maze_Info[i] |= 0x04;  //아래쪽 테두리
          if((i&0xf0)==0xf0)
               Maze_Info[i] |= 0x01;  //위쪽 테두리
     }
       
     this_speed = _LPM(&ACC_TBL[0]);       //모터 구동시작 속도값 저장
  
     LM_THIS_SPD(this_speed);              //왼쪽모터 초기속도 설정
     RM_THIS_SPD(this_speed);              //오른쪽모터 초기속도 설정
  
     SEI();                              //글로벌 인터럽트 허용
     Delay_ms(1000);
     
     //센서값이 제대로 들어오는지 검사
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
  	//벽정보 갱신
     UpdateWallInfo();
     
     //최단거리 생성
     FindFastLoad(Goal_Block_Pos[0]);
     
     LM_FORWARD_GO();
     RM_FORWARD_GO();
   
   	//마우스 최초놓인 블록에 방향 기록
   	Maze_Info[This_Block_Pos] &= 0x0f;
	Maze_Info[This_Block_Pos] |= This_Dir; 
	
	Delay_ms(1000);
     MOTOR_ON();
	//LMotor_Inturrupt_Enable();
	//RMotor_Inturrupt_Enable();
	
     //Start Main Loop
     for(;;)
     {
         //LCD 센서정보 업데이트를 위한.
          //if(num_cnt % 50 == 0)
          //{
              LCD_Page_Viwe(LCD_Page_Num);
          //}
          //마우스 방향과 진행할 방향이 같나?
     	if(This_Dir == FastLoad[FastLoadIndex])
     	{
          	//현재 진행할 방향과 다음 블록에서의 진행방향과 같나?
     		if((FastLoad[FastLoadIndex] == FastLoad[FastLoadIndex+1]) || (FastLoad[FastLoadIndex+1] == 0xf0) )
     		{
     			//마우스 이동을 허락하고
	     		LMotor_Inturrupt_Enable();            //왼쪽모터 인터럽트 허용
			     RMotor_Inturrupt_Enable();            //오른쪽 모터 인터럽트 허용
		          
     			//1블록 이동했나?
     			for(tmp=0; tmp<(OneBlockCntStep * 2); )
	     		{
	     		     //블록의 중심에서 벽정보를 업데이트 한다
	     		     if(tmp==OneBlockCntStep)
	     		          UpdateWallInfo();
	     		          
     				//진행방향에 벽이 감지되나?
					if((Sensor_data[2] > LF_LOW) && (Sensor_data[3] > RF_LOW))
					{
						//벽이 감지되면
						//속도를 줄이고 루프를 빠져 나온다
						LM_Spd_Limit = 249;
						RM_Spd_Limit = 249;
						break;
					}
					else
					{
						//벽이 감지안되면 진행하면서 자세보정을 하고
						Mouse_Posture_CTR();
					}
					tmp = LMot_BlStep_Cnt+RMot_BlStep_Cnt;
				}
				if(tmp<(OneBlockCntStep * 2))
				{
					//전방에 벽이 있었으므로
					//블록의 중심에 올때까지 진행한다
					for( ;tmp < OneBlockCntStep; )
					{
						tmp = LMot_BlStep_Cnt+RMot_BlStep_Cnt;
					}
					//블록의 중앙에 오면
					//마우스를 정지하고
					LMotor_Inturrupt_Disable();            //왼쪽모터 인터럽트 금지
					RMotor_Inturrupt_Disable();            //오른쪽 모터 인터럽트 금지
					Delay_ms(1000);
					//벽정보를 갱신하며
					UpdateWallInfo();
					
					//새로운 최단경로를 찾는다
					FindFastLoad(Goal_Block_Pos[0]);
				}
				else
				{
					//전방에 벽이 없이 1블록 이동했으므로
					//현재 마우스 위치를 갱신하고
					if(This_Dir == NORTH)
						This_Block_Pos += 0x10;
					else if(This_Dir == EAST)
						This_Block_Pos += 0x01;
					else if(This_Dir == SOUTH)
						This_Block_Pos -= 0x10;
					else if(This_Dir == WEST)
						This_Block_Pos -= 0x01;
						
					//이전블록에 마우스 진행 흔적을 남기고
					Maze_Info[This_Block_Pos] &= 0x0f;
					Maze_Info[This_Block_Pos] |= This_Dir;   
					
					//벽정보를 갱신하며
					//UpdateWallInfo();
					
					//최단경로 저장배열의 참조 인덱스를 증가
					FastLoadIndex++;
					
					LMot_BlStep_Cnt=0;
     				RMot_BlStep_Cnt=0;
     				//LMotor_Inturrupt_Disable();            //왼쪽모터 인터럽트 금지
					//RMotor_Inturrupt_Disable();            //오른쪽 모터 인터럽트 금지
     				//Delay_ms(1000);
					//LMotor_Inturrupt_Enable();            //왼쪽모터 인터럽트 금지
					//RMotor_Inturrupt_Enable();
				}	
     		}
     		//현재 진행할 방향과 다음 진행방향이 다르면
     		//스무스턴 할 수 있으므로
     		else
     		{
     			//마우스 이동을 허락하고
	     		LMotor_Inturrupt_Enable();            //왼쪽모터 인터럽트 허용
			     RMotor_Inturrupt_Enable();            //오른쪽 모터 인터럽트 허용
     			//우선 스무스턴 하기전 전방의 벽이 감지될 수 
     			//없는 위치면 측정 가능 위치까지 진행한다
     			if((LMot_BlStep_Cnt + RMot_BlStep_Cnt)<100)
     			{
	     			for(tmp=0;tmp <100;)
     				{
     					//직진이므로 역시 자세보정을 하고
     					//이동하면서 마우스 자세도 바로 잡고
     					Mouse_Posture_CTR();
						tmp = LMot_BlStep_Cnt + RMot_BlStep_Cnt;
					}
				}
								 		
				//센싱위치에 왔으므로 전방에 벽이 있나 검사
				if((Sensor_data[2] < LF_LOW) && (Sensor_data[3] < RF_LOW))
				{
					//전방에 벽이 없으므로 스무스턴 가능
					//우선 턴가능 지점까지 앞으로 이동하고
					for(tmp=0;tmp < 2;)
					{
						//스무스턴 방향이 좌측턴이면
	     				if(((((This_Dir>>1) & 0x0f)<<4)|(This_Dir>>1)) == FastLoad[FastLoadIndex+1])
     					{
     						//우선 우측 대각센서가 감지되면 모드 1증가
     						if(tmp==0 && (Sensor_data[1] > LC_LOW))
     							tmp++;
     						if(tmp==1 && (Sensor_data[0] > LS_LOW))
     							tmp++;
     					}
     					//좌측턴이 아니면 우측턴
     					else
     					{
     						if(tmp==0 && (Sensor_data[4] > RC_LOW))
     							tmp++;
     						if(tmp==1 && (Sensor_data[5] > RS_LOW))
     							tmp++;
     					}
						//진행하면서 자세보정도 하고
						Mouse_Posture_CTR();
     				}
     				//턴가능 모드가 되었으면
     				//턴할 방향에 벽이 있는지 검사
     				for(tmp=0;tmp == 0;)
     				{
     					//스무스턴 방향이 좌측턴이면
	     				if(((((This_Dir>>1) & 0x0f)<<4)|(This_Dir>>1)) == FastLoad[FastLoadIndex+1])
     					{
     						if(Sensor_data[1] < LC_LOW)
     						{
     							//벽이 없으므로 스무스턴 가능
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
     							//벽이 없으므로
     							tmp++;
     						}
     						else if((LMot_BlStep_Cnt + RMot_BlStep_Cnt)>(OneBlockCntStep * 2))
     						{
     							tmp=2;
     						}
     					}
     					//진행하면서 자세보정도 하고
						Mouse_Posture_CTR();
     				}
     				//스무스턴이면
     				if(tmp==1)
     				{
     					//블록정보에 진행 흔적을 남기고
	     				Maze_Info[This_Block_Pos] &= 0x0f;
						Maze_Info[This_Block_Pos] |= FastLoad[FastLoadIndex++];
										
     					//좌표를 갱신하고
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
	     					//턴지점까지 이동
     						for(tmp=0;tmp==0;)
     						{
     							//좌측턴일때
     							if(((((This_Dir>>1) & 0x0f)<<4)|(This_Dir>>1)) == FastLoad[FastLoadIndex])
     							{
     								if(Sensor_data[0] < LS_LOW)
     								{
				          	          	//벽정보를 갱신
	     								//UpdateWallInfo();
										
										//블록정보에 진행 흔적을 남기고
	     								Maze_Info[This_Block_Pos] &= 0x0f;
										Maze_Info[This_Block_Pos] |= FastLoad[FastLoadIndex];
										
	     								//최종 좌표로 다시 위치를 갱신하고
	          	          				if(This_Dir == NORTH)
			     		               		This_Block_Pos -= 0x01;
          					          	else if(This_Dir == EAST)
               	     						This_Block_Pos += 0x10;
			                    			else if(This_Dir == SOUTH)
               			     				This_Block_Pos += 0x01;
                    						else if(This_Dir == WEST)
				                    			This_Block_Pos -= 0x10;
					                    			
     									//턴시작
     									LM_Sm_Tn_mode = 1;
               	     					RM_Sm_Tn_mode = 2;
     									tmp++;
     								}
     							}
     							//우측턴일때
	     						else	
     							{
     								if(Sensor_data[5] < RS_LOW)
     								{
					                    	
					                    	//벽정보를 갱신
	     								//UpdateWallInfo();
										
										//블록정보에 진행 흔적을 남기고
	     								Maze_Info[This_Block_Pos] &= 0x0f;
										Maze_Info[This_Block_Pos] |= FastLoad[FastLoadIndex];
							
		     							//최종 좌표로 다시 위치를 갱신하고
		                    				if(This_Dir == NORTH)
				     	               		This_Block_Pos += 0x01;
          					          	else if(This_Dir == EAST)
               		     					This_Block_Pos -= 0x10;
			          	          		else if(This_Dir == SOUTH)
               				     			This_Block_Pos -= 0x01;
                    						else if(This_Dir == WEST)
				                    			This_Block_Pos += 0x10;
	     							
     									//턴시작
	     								LM_Sm_Tn_mode = 2;
     	               					RM_Sm_Tn_mode = 1;
     									tmp++;
     								}
     							}
     						}
     						//스무스 턴이 끝날때까지 대기
             					for(;tmp>0; )
	              				{
     	      				     tmp = LM_Sm_Tn_mode+RM_Sm_Tn_mode;
          	   				}
										
          	   				//마우스 방향을 갱신
          	   				This_Dir=FastLoad[FastLoadIndex++];
          	   				
	          	          	//스무스 턴이 끝나면 연속턴인지 검사
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
	                    	//스무스턴 모드를 벗어났으면 직진이므로
	                    	//양쪽바퀴 스텝카운트 조정
	                    	LMot_BlStep_Cnt=0;
	                    	RMot_BlStep_Cnt=0;
	                    		
	                    	//벽이 있어서 턴할 수 없기 때문에 
	                    	//블록 중앙까지 간다
	                    	if(tmp==2)
	                    	{
							//벽정보를 갱신하며
							UpdateWallInfo();
						
							//새로운 최단경로를 찾는다
							FindFastLoad(Goal_Block_Pos[0]);
							
							//새로운 경로를 찾고 현재 진행방향과 마우스 방향이 다르면 
							if(This_Dir!=FastLoad[FastLoadIndex])
							{
							     //우선 속도를 낮추고
     							LM_Spd_Limit = 249;
	                             		RM_Spd_Limit = 249;
     	                              
     	                              tmp = LMot_BlStep_Cnt+RMot_BlStep_Cnt;
     	                              
		     					for( ;tmp < OneBlockCntStep; )
			     				{
				     				tmp = LMot_BlStep_Cnt+RMot_BlStep_Cnt;
					     		}
						     	//블록의 중앙에 왔으면
							     //마우스를 정지하고
     							LMotor_Inturrupt_Disable();            //왼쪽모터 인터럽트 금지
	     						RMotor_Inturrupt_Disable();            //오른쪽 모터 인터럽트 금지
							
							     Delay_ms(1000);
					          }
	                    	}	
     				}
     				//정지모드이면
     				else if(tmp==2)
     				{
                         	LMot_BlStep_Cnt=0;
                         	RMot_BlStep_Cnt=0;
                         	//현재 위치를 갱신하고
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
     					
     					//벽정보를 갱신하며
						UpdateWallInfo();
						
						//새로운 최단경로를 찾는다
						FindFastLoad(Goal_Block_Pos[0]);
						
						if(This_Dir != FastLoad[FastLoadIndex])
						{
						     //우선 속도를 낮추고
						     LM_Spd_Limit = 249;
                         	     RM_Spd_Limit = 249;
                         	
						     for( ;tmp < OneBlockCntStep; )
						     {
						     	tmp = LMot_BlStep_Cnt+RMot_BlStep_Cnt;
						     }
						
					     	//블록의 중앙에 왔으면
						     //마우스를 정지하고
						     LMotor_Inturrupt_Disable();            //왼쪽모터 인터럽트 금지
						     RMotor_Inturrupt_Disable();            //오른쪽 모터 인터럽트 금지
						
						     Delay_ms(1000);
				          }
     				}
				}
				//전방에 벽이 있기때문에 블록 중심까지 이동후 처리
				else
				{
				     if(LMot_BlStep_Cnt+RMot_BlStep_Cnt < OneBlockCntStep)
				     {
	     				//우선 속도를 낮추고
		     			LM_Spd_Limit = 249;
                              RM_Spd_Limit = 249;
                         
				     	for( ;tmp < OneBlockCntStep; )
					     {
						     tmp = LMot_BlStep_Cnt+RMot_BlStep_Cnt;
					     }
				     }
					//블록의 중앙에 왔으면
					//마우스를 정지하고
					LMotor_Inturrupt_Disable();            //왼쪽모터 인터럽트 금지
					RMotor_Inturrupt_Disable();            //오른쪽 모터 인터럽트 금지
					
					//벽정보를 갱신하며
					UpdateWallInfo();
					
					//새로운 최단경로를 찾는다
					FindFastLoad(Goal_Block_Pos[0]);
					
					Delay_ms(1000);
				}
     		}
     	}
          else
          {
          	//현재 마우스 방향과 진행방향이 틀리므로
          	//회전한다
          	//제자리 턴으로 설정하고
     		NormalTurnInit();
     		for(;Mot_TnStep_Cnt != 0;)
     		{
     			//계속 턴을한다.
     			//180도 회전이면서 양쪽에 벽이 있을때
		     	if(Turn_Limit != Turn_180Limit && Turn_Limit > Turn_90Limit)
          		{
		         		if((Mot_TnStep_Cnt >= Turn_Limit) && (Sensor_data[0]+Sensor_data[5] > 145) && (Sensor_data[1]+Sensor_data[4] > 80))
          	     	{
		              		Mot_TnStep_Cnt = 0;
          	  		}
		         	}
          		//양쪽에 벽이 하나라도 없을때
		         	else
		         	{
          			if(Mot_TnStep_Cnt >= Turn_Limit)
               		{
		                   	Mot_TnStep_Cnt = 0;
          	     	}
         			}
     		}
     		//턴이 끝나면 방향을 갱신한다
     		LMotor_Inturrupt_Disable();            //왼쪽모터 인터럽트 금지
			RMotor_Inturrupt_Disable();            //오른쪽 모터 인터럽트 금지
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
               //LMotor_Inturrupt_Enable();            //왼쪽모터 인터럽트 금지
		     //RMotor_Inturrupt_Enable(); 
          }
     }//End Main Loop     
}//End Main Function
/////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////
//    스위치 인터럽트 처리루틴 시작
//-----------------------------------------------------------------
#pragma vector=INT4_vect
__interrupt void SW_INT(void)
{
  unsigned char sw_data;
  sw_data = PINE & 0x60;                     //스위치 정보만 받는다
  if(sw_data == SW_RD)                       //오른쪽 아래버튼 누를때
  {
     if(LCD_Page_Num > 0)                    //LCD출력 페이지를 이전으로
     {
       LCD_Page_Num--;  
       LCD_Page_Viwe(LCD_Page_Num); 
     }
  }
  else if(sw_data == SW_RU)                  //오른쪽 위 스위치 누르면
  {
     if(LCD_Page_Num  < Lcd_Page_Limit)
     {
          LCD_Page_Num++;                    //LCD페이지를 다음페이지로
          LCD_Page_Viwe(LCD_Page_Num); 
     }
  }
  else if(sw_data == SW_LD)                  //왼쪽 아래버튼 누르면 
  {
     Edit_Data_in_Lcd(LCD_Page_Num, 0);      //현제 LCD에 출력되어 있는 페이지데이터값 감소
  }
  else if(sw_data == SW_LU)                  //왼쪽 위 버튼을 누르면
  {
     Edit_Data_in_Lcd(LCD_Page_Num, 1);      //현제 LCD에 표시된 페이지 데이터 증가
  }
}
////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////
//    왼쪽모터 인터럽트 처리루틴 시작
//-----------------------------------------------------------------
#pragma vector=TIMER1_COMPA_vect         //왼쪽모터 타이머 인터럽트 처리루틴
__interrupt void LMotor_Int_A(void)
{
     unsigned char this_speed;
  
     LM_PULSE_CLR();                        //LMotor_Int_B 인터럽트 루틴에서 만든 High펄스를 100us 후에 Low로 만든다
  
     if(LM_Sm_Tn_mode==1)
     {
          this_speed = _LPM(&Sm_Tbl_In[LM_Smpd_index]);    //새로 적용할 모터 속도를 로드
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
          this_speed = _LPM(&Sm_Tbl_Out[LM_Smpd_index]);    //새로 적용할 모터 속도를 로드
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
          this_speed = _LPM(&ACC_TBL[LM_Spd_index]);    //새로 적용할 모터 속도를 로드
    
          if(this_speed > LM_Spd_Limit)         //현재속도 & 목표속도 비교
          {
               LM_Spd_index++;                     //목표속도 초과시 속도 감소
          }
          else if(this_speed < LM_Spd_Limit)
          {
               LM_Spd_index--;                     //목표속도 미달시 속도 증가
          }
  
     	LMot_BlStep_Cnt++;
  
  		if(Turn_Limit > 0)
  		{
     		Mot_TnStep_Cnt++;
  		}
  		
          this_speed = _LPM(&ACC_TBL[LM_Spd_index]);  //속도 적용
     }
     LM_Spd_Now = this_speed;
     LM_THIS_SPD(this_speed);                    //다음속도 적용
 
     TCNT1H = 0x00;                              //타이머 시작
     TCNT1L = 0x00;
}

#pragma vector=TIMER1_COMPB_vect
__interrupt void LMotor_Int_B(void)
{ 
  LM_PULSE_SET();                           //왼쪽모터 온타임 펄스  
  
  //TX_Char('L');
}
///////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////
//    오른쪽모터 인터럽트 처리루틴 시작
//-----------------------------------------------------------------
#pragma vector=TIMER3_COMPA_vect            //오른쪽 모터 타이머 인터럽트 처리루틴
__interrupt void RMotor_Int_A(void)
{
     unsigned char this_speed;
  
     RM_PULSE_CLR();                            //RMotor_Int_B 인터럽트 루틴에서 만든 High펄스를 100us 후에 Low로 만든다
     
     if(RM_Sm_Tn_mode==1)
     {
          this_speed = _LPM(&Sm_Tbl_In[RM_Smpd_index]);    //새로 적용할 모터 속도를 로드
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
          this_speed = _LPM(&Sm_Tbl_Out[RM_Smpd_index]);    //새로 적용할 모터 속도를 로드
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
  RM_PULSE_SET();                               //오른쪽 모터 온타임 펄스 
  
  //TX_Char('R');
}
////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////
//    A/D변환 인터럽트 처리루틴 시작
//-----------------------------------------------------------------
#pragma vector=ADC_vect
__interrupt void Sensor_Int(void)
{
  unsigned char tmp_num;
  tmp_num = 5 - Sen_sel;
  Sensor_data[tmp_num] = (ADC >> 2);     //센서 데이터 저장
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
  Sen_El_Start(Sen_sel);                //다음 센서 발광 시작
}
////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////
//    시리얼 통신 데이터 수신 인터럽트 처리루틴 시작
//-----------------------------------------------------------------
#pragma vector=USART0_RXC_vect
__interrupt void Sensor_data_receive(void)
{
  unsigned char sen_data;
    
  sen_data = SERIAL_BUFF;
  
  if(sen_data == 'S')
  {
    MOTOR_ON();
    LMotor_Inturrupt_Enable();            //왼쪽모터 인터럽트 허용
    RMotor_Inturrupt_Enable();            //오른쪽 모터 인터럽트 허용
  }
  else if(sen_data == 'E')
  {
    LMotor_Inturrupt_Disable();            //왼쪽모터 인터럽트 금지
    RMotor_Inturrupt_Disable();            //오른쪽 모터 인터럽트 금지
    MOTOR_OFF();
  }
  else if(sen_data == 'F')
  {
    LM_FORWARD_GO();                      //왼쪽모터 앞으로 진행
    RM_FORWARD_GO();                      //오른쪽모터 앞으로 진행
  }
  else if(sen_data == 'B')
  {
    LM_BACKWARD_GO();                     //왼쪽모터 뒤로 진행
    RM_BACKWARD_GO();                     //오른쪽모터 뒤로 진행
  }
  else if(sen_data == 'D')
  {
    Sen_sel = 0;                         //센서데이터 받기 시작
  }
  else if(Sen_sel < 6)
  {
    Sensor_data[Sen_sel++] = sen_data;    //센서 데이터
  }
}
////////////////////////////////////////////////////////////////////

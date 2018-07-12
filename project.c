/*
0번모터 42~2004 (높아질수록 좌회전)
1번모터 768~256 (높을수록 앞으로 굽음) 
2번모터 60~940
3번모터 820~140
4번모터 1~1023 (높아질수록 우회전)

0번 센서(4번 모터) 0~255 (높아질 수록 우회전)
1번 센서(3번 모터) 15~244 (높아질수록 안으로 굽음)
2번 센서(2번 모터) 0~255 (높아질수록 안으로 굽음)
3번 센서(1번 모터) 15~240 (높아질수록 안으로 굽음)
4번 센서(0번 모터) 255~0  (낮아질수록 좌화전)
*/

#define F_CPU 16000000      // 시스템 클럭 설정
#define MOTOR_NUM 5

#define TORQUE_MODE 0x60
#define RESIST_MODE 0x40
#define FREE_MODE 0x00

#define TORQUE_CONTROL 52
#define CALI_POS 58
#define MAX_CAPTURE_NUM 250

#include <stdio.h>
#include <avr/io.h>         // AVR 레지스터 정의
#include <util/delay.h>      // Delay 헤더
#include <avr/interrupt.h>   // 인터럽트 정보
#include "clcd.h"

#include <herkulex.h>      //herkulex 라이브러리의 헤더 파일을 포함
//#include <USART0.h>
#include <TIMER0.h>


// About SW
#define Up_SW_on      (!(PINC&0x01))
#define Down_SW_on      (!(PINC&0x04))
#define Left_SW_on      (!(PINC&0x02))
#define Right_SW_on      (!(PINC&0x10))
#define Mid_SW_on      (!(PINC&0x08))
#define Grab_SW_on      (!(PINC&0x20))
#define Up_SW_off      (PINC&0x01)
#define Down_SW_off      (PINC&0x04)
#define Left_SW_off      (PINC&0x02)
#define Rignt_SW_off   (PINC&0x10)
#define Mid_SW_off      (PINC&0x08)
#define Grab_SW_off      (PINC&0x20)

// About buzzer
#define buzzer_on   DDRE |= 0x08;
#define buzzer_off  DDRE &= 0xF7;
unsigned int freq = 6814;
char ch = 0;

// About Grab
#define Grab_on      OCR1AH=220>>8 , OCR1AL=220;  // 200~320~480 servo motor
#define Grab_off   OCR1AH=620>>8 , OCR1AL=620;  // 200~320~480 servo motor
unsigned int grab = 320;
int motor_pos_record[MAX_CAPTURE_NUM][MOTOR_NUM] = {0,};
char sw_record[MAX_CAPTURE_NUM] = {0,};
int sequence = 0;
int cap[6] = {0,0,0,0,0,0};
int pre_cap[6] = {0,0,0,0,0,0};
int cap_buff = 0;

typedef struct DrsCaliPos
{
   int            iPosition : 13;
   unsigned int   uiGPIO1 : 1;
   unsigned int   uiGPIO2 : 1;
   unsigned int   reserved : 1;
}DrsCaliPos;

typedef union DrsUnionCaliPos
{
   DrsCaliPos      stCaliPos;
   unsigned int   uiCaliPos;
}DrsUnionCaliPos;

// Func
void gpio_init();
void timer_init();
void adc_init();
void SW_test();
void ADC_test();
void usart_init();
void putch(char data);
char getch();
void putangle(int i,int angle);
void buzzer();
void bbip();
void grab_set();
void angle_test(unsigned char* ID, int* ipos, DrsPacket* stSendPacket);
void Run(unsigned char* ID, int* ipos, DrsPacket* stSendPacket);
void Master_Mode();
void Record_master(unsigned char* ID, int* ipos, DrsPacket* stSendPacket);
void Free_run(unsigned char* ID, int* ipos, DrsPacket* stSendPacket);
void Non_Master_Mode();
void Play(unsigned char* ID, DrsPacket* stSendPacket);
void Non_Master_Record(unsigned char* ID, DrsPacket* stSendPacket, DrsPacket* stRcvPacket, DrsUnionCaliPos* unCaliPos);
void spline_record(unsigned char* ID, int* ipos, DrsPacket* stSendPacket);
void en_test();

unsigned char receive(unsigned char ID, DrsPacket* stSendPacket, DrsPacket* stRcvPacket);
void send(unsigned char* ID, int* ipos, DrsPacket* stSendPacket);
char decode(unsigned char ucResult, DrsUnionCaliPos* unCaliPos, DrsPacket* stRcvPacket);
void setMode(unsigned char* ID, int MODE);
void setModePerMotor(unsigned char id, int MODE);


//About Menu
int menu = 0;
char LCD_D[40];

// About ADC
unsigned char adc_data[8] = {0, 0, 0, 0, 0, 0, 0, 0};
unsigned char mux = 0;

//DRS-0101의 Calibrated Position 정보를 사용하기 위한 구조체


//Calibrated Position 정보를 쉽게 다루기 위한 공용체


ISR(ADC_vect)
{
   adc_data[mux]=ADCH;
   mux++;
   if(mux >= 8) mux=0;
   ADMUX = mux | 0x60;
   ADCSRA |= 0x40;
}

int main(void)
{
   //보낼 패킷과 받을 패킷이 저장될 패킷 구조체 선언
   DrsPacket stSendPacket, stRcvPacket;
   //Calibrated Position을 저장할 공용체 변수 선언
   DrsUnionCaliPos unCaliPos;

   //사용할 변수들 선언
   unsigned char ucResult;
   int ipos[MOTOR_NUM] = {0,};
   unsigned char ID[MOTOR_NUM] = {0,1,2,3,4};
   int current_pos[MOTOR_NUM] = {0,};
   
   sei();
   gpio_init();
   timer_init();
   adc_init();
   usart_init();
   LCD_init();
   //SW_test();
   //ADC_test();
   
   //전체 인터럽트를 비활성화
   cli();
   
   //HerkuleX를 사용하기 위해 초기화
   hklx_Init(115200);
   
   //전체 인터럽트를 활성화
   sei();
   
   //Torque Control에 0x60을 써서 토크를 거는 패킷 구성
   setMode(ID, TORQUE_MODE);
   
   while (1)
   {
      /*
      if(Up_SW_on)
      {
         bbip();
         Non_Master_Record(ID, &stSendPacket, &stRcvPacket, &unCaliPos);
      }
      else if(Mid_SW_on)
      {
         bbip();
         //Play(ID, &stSendPacket);
         en_test();
      }*/
      /*else if(Down_SW_on)
      {
         bbip();
         Play(ID, &stSendPacket);
      }*/
      
      
      if(Up_SW_on)
      {
         menu ++;
         bbip();
      }
      else if(Down_SW_on)
      {
         menu --;
         bbip();
      }
      else buzzer_off;
      if(menu < 0) menu = 5;
      else if(menu > 5) menu = 0;
      switch(menu)
      {
         case 0 :
         LCD_clear();
         LCD_goto(0,0);
         LCD_string("1.SW_test");
         if(Mid_SW_on)   SW_test();
         _delay_ms(100);
         break;
         
         case 1 :
         LCD_clear();
         LCD_goto(0,0);
         LCD_string("2.ADC Test");
         if(Mid_SW_on)   ADC_test();
         _delay_ms(100);
         break;
         
         case 2 :
         LCD_clear();
         LCD_goto(0,0);
         LCD_string("3.Grab Test");
         if(Mid_SW_on)   grab_set();
         _delay_ms(100);
         break;
         
         case 3 :
         LCD_clear();
         LCD_goto(0,0);
         LCD_string("4.En Test");
         if(Mid_SW_on)   en_test();
         _delay_ms(100);
         break;
      
         case 4 :
         LCD_clear();
         LCD_goto(0,0);
         LCD_string("5.Run");
         if(Mid_SW_on)   Run(ID, ipos, &stSendPacket);   
         _delay_ms(100);
         break;
         
         case 5 :
         LCD_clear();
         LCD_goto(0,0);
         LCD_string("6.spline Record");
         if(Mid_SW_on)   spline_record(ID, ipos, &stSendPacket);
         _delay_ms(100);
         break;

      }
      
      
   }
}
void gpio_init()
{
   DDRA = 0xFF;   // C-LCD
   PORTA = 0x00;
   
   DDRB = 0xFF;   // servo motor
   PORTB = 0x00;
   
   DDRC = 0x00;   // Switch (0~5)
   PORTC = 0x00;
   
   DDRD = 0x08;   // 3:RX, 4:TX
   PORTD = 0x00;
   
   DDRE = 0xF7;   // 0:RX, 1:TX, 3:Buzzer
   PORTE = 0x00;
   
   DDRF = 0x00;   // ADC
   PORTF = 0x00;
   DDRG = 0xff;
}
void timer_init()
{
   // timer/counter 0 initiation
   ASSR=0x00;
   TCCR0=0x00;
   TCNT0=0x00;
   OCR0=0x00;
   
   // timer/counter 1 initiation
   TCCR1A=0xA2;
   TCCR1B=0x1B;
   TCNT1H=0x00;
   TCNT1L=0x00;
   ICR1H=0x13;
   ICR1L=0x87;
   OCR1AH=320>>8;  // 200~320~480 servo motor
   OCR1AL=320;
   OCR1BH=0x00;
   OCR1BL=0x00;
   OCR1CH=0x00;
   OCR1CL=0x00;
   
   // timer/counter 2 initiation
   TCCR2=0x00;
   TCNT2=0x00;
   OCR2=0x00;

   // timer/counter 3 initiation
   TCCR3A=0xA2;
   TCCR3B=0x19;
   TCNT3H=0x00;
   TCNT3L=0x00;
   ICR3H=0x00;
   ICR3L=0xC7;
   OCR3AH=0x00;
   OCR3AL=0x00;
   OCR3BH=0x00;
   OCR3BL=0x00;
   OCR3CH=0x00;
   OCR3CL=0x00;
   
}
void adc_init()
{
   ADMUX=0x20;
   ADCSRA=0xCF;
}
void usart_init()
{
   DDRD = 0x04;
   UCSR1A=0x00;
   UCSR1B=0x18;    // 송신부 작동 허용
   UCSR1C=0x86;    // 8bit 전송
   UBRR1H = 0x00;
   UBRR1L = 0x67; // Baud Rate = 9600 bps
}
void putch(char data)
{
   while(!(UCSR1A & 0x20));
   UDR1 = data;
} 
char getch()
{
   while(!(UCSR1A & 0x80));
   return UDR1;
}
void putangle(int i,int angle)
{
   ch = 0x00;
   ch |= (i <<6);
   ch |= 0x20;
   if(i == 0) ch |= ((angle/2) >> 5);
   else
   {
      ch |= (angle >> 5);
   }
   putch(ch);
   _delay_ms(20);
   ch = 0x00;
   ch |= (i <<6);
   if(i == 0) ch |= ((angle/2) & 0x1F);
   else
   {
      ch |= (angle & 0x1F);
   }
   putch(ch);
   _delay_ms(20);

}
void SW_test()
{
   bbip();
   int signal = 0;
   LCD_clear();
   _delay_ms(200);
   while(Mid_SW_off)
   {
      signal = PINC;
      LCD_clear();
      LCD_goto(0,0);
      LCD_string("NUM");
      LCD_goto(1,0);
      LCD_sprintf(LCD_D, (63-signal) );
      LCD_string(LCD_D);
      _delay_ms(100);
   }
   bbip();
   _delay_ms(200);
   
}
void ADC_test()
{
   bbip();
   unsigned char adc = 0;
   _delay_ms(200);
   while(Mid_SW_off)
   {
      if(Up_SW_on)
      {
         adc ++;
         bbip();
      }
      if(Down_SW_on)
      {
         adc --;
         bbip();
      }
      if(adc >= 8) adc = 0;
      LCD_clear();
      LCD_sprintf(LCD_D, (adc) );
      LCD_string(LCD_D);
      LCD_goto(1,0);
      LCD_sprintf(LCD_D, (adc_data[adc]) );
      LCD_string(LCD_D);
      _delay_ms(100);
   }
   bbip();
   _delay_ms(200);
}
void buzzer(unsigned int fre)
{
   ICR3H=fre>>8;
   ICR3L=fre;
   OCR3AH=(int)(freq/2)>>8;
   OCR3AL=(int)(freq/2);
}
void bbip()
{
   buzzer_on;
   buzzer(freq);
   _delay_ms(30);
   buzzer_off;
}
void grab_set()
{
   bbip();
   _delay_ms(200);
   while(Mid_SW_off)
   {
      if(Up_SW_on)
      {
         grab += 5;
         bbip();
      }
      if(Down_SW_on)
      {
         grab -= 5;
         bbip();
      }
      OCR1AH=grab>>8;  // 200~320~480 servo motor
      OCR1AL=grab;
      
      LCD_clear();
      LCD_goto(0,0);
      LCD_string("OCR");
      LCD_goto(1,0);
      LCD_sprintf(LCD_D, (grab) );
      LCD_string(LCD_D);
      _delay_ms(50);
   }
}
void angle_test(unsigned char* ID, int* ipos, DrsPacket* stSendPacket)
{
   bbip();
   int angle = 500;
   _delay_ms(200);
   while(Mid_SW_off)
   {
      if(Up_SW_on)
      {
         angle ++;
         bbip();
      }
      if(Down_SW_on)
      {
         angle --;
         bbip();
      }
      if(angle >= 1000) angle = 1000;
      
      for(int i = 0; i < MOTOR_NUM; i++){
         ipos[i] = angle;
      }
      send(ID, ipos, stSendPacket);
      
      //다 움직일 때 까지 1초 대기
      _delay_ms(100);
      
      
      LCD_clear();
      LCD_sprintf(LCD_D, (angle) );
      LCD_string(LCD_D);
   }
   bbip();
   _delay_ms(200);
}
char decode(unsigned char ucResult, DrsUnionCaliPos* unCaliPos, DrsPacket* stRcvPacket){
   if(ucResult == DRS_RXCOMPLETE){
      //받은 데이터 2바이트를 공용체 변수에 저장
      unCaliPos->uiCaliPos = stRcvPacket->unData.stRWData.ucData[0] |
      (stRcvPacket->unData.stRWData.ucData[1]<<8);
      return 1;
   }
   //패킷 수신이 정상적으로 이루어지지 않았을 때 LED 모두 켬
   else{
      return 0;
   }
}
void send(unsigned char* ID, int* ipos, DrsPacket* stSendPacket){

   stSendPacket->ucPacketSize = MIN_PACKET_SIZE + CMD_I_JOG_STRUCT_SIZE * MOTOR_NUM;
   stSendPacket->ucChipID = BROADCAST_ID;
   stSendPacket->ucCmd = CMD_I_JOG;

   for(int i = 0; i < MOTOR_NUM; i++){
      stSendPacket->unData.stIJogData.stIJog[i].stJog.uiValue = ipos[i];
      stSendPacket->unData.stIJogData.stIJog[i].stSet.ucStopFlag = 0;
      stSendPacket->unData.stIJogData.stIJog[i].stSet.ucMode = 0;
      stSendPacket->unData.stIJogData.stIJog[i].stSet.ucLedGreen = 1;
      stSendPacket->unData.stIJogData.stIJog[i].stSet.ucLedBlue = 1;
      stSendPacket->unData.stIJogData.stIJog[i].stSet.ucLedRed = 1;
      stSendPacket->unData.stIJogData.stIJog[i].stSet.ucJogInvalid = 0;
      stSendPacket->unData.stIJogData.stIJog[i].ucId = ID[i];
      stSendPacket->unData.stIJogData.stIJog[i].ucPlayTime = 50;
   }

   //패킷 보내기
   hklx_SendPacket(*stSendPacket);
}
unsigned char receive(unsigned char ID, DrsPacket* stSendPacket, DrsPacket* stRcvPacket){

   unsigned char result = 0;

   stSendPacket->ucPacketSize = MIN_PACKET_SIZE + 2;
   stSendPacket->ucChipID = ID;
   stSendPacket->ucCmd = CMD_RAM_READ;
   stSendPacket->unData.stRWData.ucAddress = CALI_POS;
   stSendPacket->unData.stRWData.ucLen = 2;

   hklx_SendPacket(*stSendPacket);

   gucTimerTick=25;
   while(1){
      //패킷을 받는 함수를 호출해 결과를 result에 저장
      result = hklx_ucReceivePacket(stRcvPacket);
      //결과 값이 DRS_RXWAITING이 아니면 빠져나옴
      if(result != DRS_RXWAITING){
         break;
      }
      //30ms가 지나서 gucTimerTick이 0이 되면 빠져나옴
      if(gucTimerTick==0){
         result = DRS_RXTIMEOUT;
         break;
      }
   }

   return result;
}
void setMode(unsigned char* ID, int MODE){

   for(int i = 0; i < MOTOR_NUM; i++){
      
      setModePerMotor(ID[i], MODE);
      _delay_ms(10);  
   }
}
void setModePerMotor(unsigned char id, int MODE){
   DrsPacket stSendPacket;

   stSendPacket.ucPacketSize = MIN_PACKET_SIZE + 3;
   stSendPacket.ucChipID = id;
   stSendPacket.ucCmd = CMD_RAM_WRITE;
   stSendPacket.unData.stRWData.ucAddress = TORQUE_CONTROL;
   stSendPacket.unData.stRWData.ucLen = 1;
   stSendPacket.unData.stRWData.ucData[0] = MODE;

   //패킷 보내기
   hklx_SendPacket(stSendPacket);
}
void Run(unsigned char* ID, int* ipos, DrsPacket* stSendPacket)
{
   int sub_menu = 0;
   
   bbip();
   _delay_ms(200);
   while(Left_SW_off)
   {
      if(Up_SW_on)
      {
         sub_menu ++;
         bbip();
      }
      else if(Down_SW_on)
      {
         sub_menu --;
         bbip();
      }
      else buzzer_off;
      if(sub_menu < 0) sub_menu = 1;
      else if(sub_menu > 1) sub_menu = 0;
      
      switch(sub_menu)
      {
         case 0 :
               LCD_clear();
               LCD_goto(0,0);
               LCD_string("1.Free Run Mode");
               if(Mid_SW_on)   Free_run(ID, ipos, stSendPacket);
               _delay_ms(100);
               break;
         
         case 1 :
               LCD_clear();
               LCD_goto(0,0);
               LCD_string("2.Master Mode");
               if(Mid_SW_on)   Master_Mode(ID, ipos, stSendPacket);
               _delay_ms(100);
               break;
         
         case 2 :
               LCD_clear();
               LCD_goto(0,0);
               LCD_string("3.NonMaster Mode");
               if(Mid_SW_on)   grab_set();
               _delay_ms(100);
               break;
      }
         
   }   
   bbip();
   _delay_ms(200);
}
void Master_Mode(unsigned char* ID, int* ipos, DrsPacket* stSendPacket)
{
   bbip();
   _delay_ms(200);
   while(Left_SW_off)
   {
      if(Up_SW_on)
      {
         LCD_clear();
         Record_master(ID, ipos, stSendPacket);
      }
      if(Down_SW_on)
      {
         LCD_clear();
         Play(ID, stSendPacket);
      }
      LCD_clear();
      LCD_goto(0,0);
      LCD_string("Up : Record");
      LCD_goto(1,0);
      LCD_string("Down : Run!");
      _delay_ms(100);
   }      
}
void Record_master(unsigned char* ID, int* ipos, DrsPacket* stSendPacket)
{
   LCD_clear();
   _delay_ms(200);
   sequence = 0;
   while(1)
   {
      bbip();
      if((sequence == MAX_CAPTURE_NUM)||(Up_SW_on)) break;
      
      motor_pos_record[sequence][0] = (int)((1962.0/255.0)*(adc_data[4]) + 42.0);
      motor_pos_record[sequence][1] = (int)(-1.0*(512.0/225.0)*(adc_data[3]-15.0) + 768.0);
      motor_pos_record[sequence][2] = (int)((880.0/255.0)*(adc_data[2]) + 60.0);
      motor_pos_record[sequence][3] = (int)(-1*(680.0/229.0)*(adc_data[1]-15.0) + 820.0);
      motor_pos_record[sequence][4] = (int)((1022.0/255.0)*(adc_data[0]) + 1.0);
      ipos[0] = (int)((1962.0/255.0)*(adc_data[4]) + 42.0);
      ipos[1] = (int)(-1.0*(512.0/225.0)*(adc_data[3]-15.0) + 768.0);
      ipos[2] = (int)((880.0/255.0)*(adc_data[2]) + 60.0);
      ipos[3] = (int)(-1*(680.0/229.0)*(adc_data[1]-15.0) + 820.0);
      ipos[4] = (int)((1022.0/255.0)*(adc_data[0]) + 1.0);
      if(Grab_SW_on)
      {
         Grab_on;
         sw_record[sequence] = 1;
      }
      else
      {
          Grab_off;
          sw_record[sequence] = 0;
      }
      sequence++;
      send(ID, ipos, stSendPacket);
      _delay_ms(50);
   }   
   _delay_ms(200);
}
void Free_run(unsigned char* ID, int* ipos, DrsPacket* stSendPacket)
{   
   bbip();
   LCD_clear();
   _delay_ms(200);
   while(Left_SW_off)
   {
      ipos[0] = (int)((1962.0/255.0)*(adc_data[4]) + 42.0);
      ipos[1] = (int)(-1.0*(512.0/225.0)*(adc_data[3]-15.0) + 768.0);
      ipos[2] = (int)((880.0/255.0)*(adc_data[2]) + 60.0);
      ipos[3] = (int)(-1*(680.0/229.0)*(adc_data[1]-15.0) + 820.0);
      ipos[4] = (int)((1022.0/255.0)*(adc_data[0]) + 1.0);
      send(ID, ipos, stSendPacket);
      if(Grab_SW_on)
      {
         Grab_on;
      }
      else
      {
         Grab_off;
      }
      
   }
   bbip();
   _delay_ms(200);   
}
void Non_Master_Record(unsigned char* ID, DrsPacket* stSendPacket, DrsPacket* stRcvPacket, DrsUnionCaliPos* unCaliPos)
{
   _delay_ms(200);
   setMode(ID, FREE_MODE);
   putch(0xFF); // start signal
   bbip();
   LCD_clear();
   sequence = 0;
   
   while(Left_SW_off){
      ////TODO///////////
      
      //if((sequence == MAX_CAPTURE_NUM)||(Up_SW_on)) break;
      
      //sequence++;
      if(Mid_SW_on){
         bbip();
         for(int i = 0; i < MOTOR_NUM; i++){
            unsigned char ucResult;
            ucResult = receive(ID[i], stSendPacket, stRcvPacket);
            _delay_ms(500);
         
         /*
            LCD_clear();
            LCD_goto(0,0);
            switch(ucResult){
               case DRS_RXWAITING:
               LCD_string("DRS_RXWAITING");
                  break;
               case DRS_RXCOMPLETE:
               LCD_string("DRS_RXCOMPLETE");
               break;
               case DRS_HEADERNOTFOUND:
               LCD_string("DRS_HEADERNOTFOUND");
               break;
               case DRS_INVALIDSIZE:
               LCD_string("DRS_INVALIDSIZE");
               break;
               case DRS_UNKNOWNCMD:
               LCD_string("DRS_UNKNOWNCMD");
               break;
               case DRS_INVALIDID:
               LCD_string("DRS_INVALIDID");
               break;
               case DRS_CHKSUMERROR:
               LCD_string("DRS_CHKSUMERROR");
               break;
               case DRS_RXTIMEOUT:
               LCD_string("DRS_RXTIMEOUT");
               break;
               default:
               break;
            }*/
         
            if(decode(ucResult, unCaliPos, stRcvPacket)){
               putangle(i,unCaliPos->stCaliPos.iPosition);
               //motor_pos_record[sequence][i] = unCaliPos->stCaliPos.iPosition;
            }else{
               //motor_pos_record[sequence][i] = -1;
               //return;
               putangle(i,0);
            }
         }
      }
      _delay_ms(200);
   }   
   putch(0xFE); // End signal
   bbip();
   setMode(ID, TORQUE_MODE);
   _delay_ms(200);
}
void spline_record(unsigned char* ID, int* ipos, DrsPacket* stSendPacket)
{
   int init_flag = 0;
   unsigned int cnt = 0;
   unsigned int sigma_cnt = 0;
   int i = 0;
   int j = 0;
   _delay_ms(200);
   bbip();
   LCD_clear();
   putch(0xFF);
   while(Left_SW_off)
   {
      ipos[0] = (int)((1962.0/255.0)*(adc_data[4]) + 42.0);
      ipos[1] = (int)(-1.0*(512.0/225.0)*(adc_data[3]-15.0) + 768.0);
      ipos[2] = (int)((880.0/255.0)*(adc_data[2]) + 60.0);
      ipos[3] = (int)(-1*(680.0/229.0)*(adc_data[1]-15.0) + 820.0);
      ipos[4] = (int)((1022.0/255.0)*(adc_data[0]) + 1.0);
      send(ID, ipos, stSendPacket);
      if(Grab_SW_on)
      {
         Grab_on;
      }
      else
      {
         Grab_off;
      }
      if(cnt>=250)
      {
         sequence = cnt;
         bbip();
         cap[0] = (int)((1962.0/255.0)*(adc_data[4]) + 42.0);
         cap[1] = (int)(-1.0*(512.0/225.0)*(adc_data[3]-15.0) + 768.0);
         cap[2] = (int)((880.0/255.0)*(adc_data[2]) + 60.0);
         cap[3] = (int)(-1*(680.0/229.0)*(adc_data[1]-15.0) + 820.0);
         cap[4] = (int)((1022.0/255.0)*(adc_data[0]) + 1.0);
         if(Grab_SW_on) cap[5] = 1;
         else cap[5] = 0;
         
         for(i = sigma_cnt ; i <= cnt; i++)
         {
            for(j = 0; j <= 4; j++)
            {
               motor_pos_record[i][j] = (int)(((float)(cap[j]-pre_cap[j])/(cnt-sigma_cnt))*(i-sigma_cnt)+pre_cap[j]);
            }
            sw_record[i] = cap[5];
         }
         sigma_cnt = cnt;
         for(j = 0; j <= 5; j++)
         {
            pre_cap[j] = cap[j];
         }
         
         _delay_ms(200);
          break;
      }
      if(init_flag == 1) cnt ++;
         
      if((Mid_SW_on)&&(init_flag == 1))
      {
         sequence = cnt;
         bbip();
         cap[0] = (int)((1962.0/255.0)*(adc_data[4]) + 42.0);
         cap[1] = (int)(-1.0*(512.0/225.0)*(adc_data[3]-15.0) + 768.0);
         cap[2] = (int)((880.0/255.0)*(adc_data[2]) + 60.0);
         cap[3] = (int)(-1*(680.0/229.0)*(adc_data[1]-15.0) + 820.0);
         cap[4] = (int)((1022.0/255.0)*(adc_data[0]) + 1.0);
         if(Grab_SW_on) cap[5] = 1;
         else cap[5] = 0;
         
         for(i = sigma_cnt ; i <= cnt; i++)
         {
            for(j = 0; j <= 4; j++)
            {
               motor_pos_record[i][j] = (int)(((float)(cap[j]-pre_cap[j])/(cnt-sigma_cnt))*(i-sigma_cnt)+pre_cap[j]);
            }
            sw_record[i] = cap[5];
         }
         sigma_cnt = cnt;
         for(j = 0; j <= 5; j++)
         {
            pre_cap[j] = cap[j];
         }
         
         _delay_ms(200);
      }
      else if((Mid_SW_on)&&(init_flag == 0))
      {
         bbip();
         _delay_ms(200);
         bbip();
         pre_cap[0] = (int)((1962.0/255.0)*(adc_data[4]) + 42.0);
         pre_cap[1] = (int)(-1.0*(512.0/225.0)*(adc_data[3]-15.0) + 768.0);
         pre_cap[2] = (int)((880.0/255.0)*(adc_data[2]) + 60.0);
         pre_cap[3] = (int)(-1*(680.0/229.0)*(adc_data[1]-15.0) + 820.0);
         pre_cap[4] = (int)((1022.0/255.0)*(adc_data[0]) + 1.0);
         if(Grab_SW_on) pre_cap[5] = 1;
         else pre_cap[5] = 0;
         cnt = 0;
         init_flag = 1;
         _delay_ms(200);
      }
      
      _delay_ms(60);
   }
   
   bbip();
   putch(0xFE);
   _delay_ms(200);
}
void Play(unsigned char* ID, DrsPacket* stSendPacket)
{
   _delay_ms(200);
   int current_sequence = 0;

   setMode(ID, TORQUE_MODE);
   
   while(1){
      ////TODO///////////
      if(Down_SW_on) break;
      else{
         if(sw_record[current_sequence] == 1) 
         {
            Grab_on;
         }
         else 
         {
            Grab_off;
         }
         send(ID, motor_pos_record[current_sequence], stSendPacket);
         current_sequence++;
         if(current_sequence > sequence) current_sequence = 0;
         _delay_ms(60);
      }
      
   }
   _delay_ms(500);
   
}
void en_test()
{
   bbip();
   unsigned char num = 0;
   unsigned int timing = 0;
   _delay_ms(200);
   while(Mid_SW_off)
   {
      if(Up_SW_on)
      {
         num ++;
         bbip();
      }
      if(Down_SW_on)
      {
         num --;
         bbip();
      }
      if(Left_SW_on)
      {
         timing ++;
         bbip();
      }
      if(Right_SW_on)
      {
         timing --;
         bbip();
      }
      if(num >= 5) num = 0;
      LCD_clear();
      LCD_goto(0,0);
      LCD_sprintf(LCD_D, (num) );
      LCD_string(LCD_D);
      LCD_goto(0,8);
      LCD_sprintf(LCD_D, (timing) );
      LCD_string(LCD_D);
      LCD_goto(1,0);
      LCD_sprintf(LCD_D, (motor_pos_record[timing][num]) );
      LCD_string(LCD_D);
      _delay_ms(100);
   }
   bbip();
   _delay_ms(200);
}
//=========================================================
// PSoC5LP Project
//=========================================================
// File Name : main.c
// Function  : Main Routine
//---------------------------------------------------------
// Rev.01 2013.02.02 Munetomo Maruyama
//---------------------------------------------------------
// Copyright (C) 2012-2013 Munetomo Maruyama
//=========================================================
// ---- License Information -------------------------------
// Anyone can FREELY use this code fully or partially
// under conditions shown below.
// 1. You may use this code only for individual purpose,
//    and educational purpose.
//    Do not use this code for business even if partially.
// 2. You can copy, modify and distribute this code.
// 3. You should remain this header text in your codes
//   including Copyright credit and License Information.
// 4. Your codes should inherit this license information.
//=========================================================
// ---- Patent Notice -------------------------------------
// I have not cared whether this system (hw + sw) causes
// infringement on the patent, copyright, trademark,
// or trade secret rights of others. You have all
// responsibilities for determining if your designs
// and products infringe on the intellectual property
// rights of others, when you use technical information
// included in this system for your business.
//=========================================================
// ---- Disclaimers ---------------------------------------
// The function and reliability of this system are not
// guaranteed. They may cause any damages to loss of
// properties, data, money, profits, life, or business.
// By adopting this system even partially, you assume
// all responsibility for its use.
//=========================================================

//------------------------
// Notes on emFile 
//------------------------
// (1) FS_X_OS.c is changed to connect it to FreeRTOS.
// (2) IMPORTANT NOTICE REGARDING LONG FILE NAMES:
// If you configure the software to support long file names on FAT filesystems, 
// you should review the information at 
// <http://www.microsoft.com/about/legal/en/us/IntellectualProperty/IPLicensing/Programs/FATFileSystem.aspx>
// to determine whether a license from Microsoft is required. Cypress and its suppliers grant no license 
// under Microsoft's intellectual property rights and assume no liability for any use of the software 
// without obtaining any license that may be required.
//---------------------------
// PSoC5 Includes
//---------------------------
#include <project.h>
#include <math.h>
#include <stdlib.h>
//---------------------------
// RTOS Includes
//---------------------------
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"
//---------------------------
// API Includes
//---------------------------
#include "com.h"
#include "utility.h"
//---------------------------
// ringbuf Includes
//---------------------------
#include "ringbuf.h"
//---------------------------
// Define Prototype
//---------------------------
void prvHardwareSetup(void);
//----------------------------------
// Define FreeRTOS related Resource
//----------------------------------
#define TASK_STACK_LED      512
#define TASK_STACK_USBUART 1024
//
#define TASK_PRIORITY_LED      (tskIDLE_PRIORITY + 2)
// In this example USBUART priority is lower than LED
#define TASK_PRIORITY_USBUART  (tskIDLE_PRIORITY + 1)
//
void Task_LED     (void *pvParameters);
void Task_USBUART (void *pvParameters);

void set_msgDFL(uint8 *buf, int16 *maxsize,int bufsize);

xSemaphoreHandle xMutex_COM; // Mutex for COM Port
uint16 timestamp;
#define PUT_MSGBUFSIZE 250    //100 is not enough
#define TREAD  485.0 //new 485.0  old 405.0
#define KP 0.10045670              //P gain of PID Control
#define KI_R 500.0              //I gain of PID Control
#define KI_L 600.0              //I gain of PID Control
#define DELTA_TIME 0.001      //control loop time

#define WHEEL_RADIUS 0.205  //0.208  //new0.208  old 0.202
#define BIAS 0.485 //trad
#define COEF 40.58325 //coeficiation G25 20.291625 G50 40.58325 

uint8 buff[PUT_MSGBUFSIZE];
uint8 bufset;
uint8 msgsize;
uint8 lpwm,rpwm;
uint8 led_flg;
tRingBufObject comrbuf;
float omega,omegarate;
unsigned char  pbuf[PUT_MSGBUFSIZE];
uint8 recbuffer[512];
int gcount;
uint8 cliff_on,dockIr_on,inertial_on,current_on,
    firm_on,hard_on,gyro_on,udid_on,gpio_on;
//=====================================
// Main Routine
//=====================================
int main()
{
    //--------------------------------------
    // Initialize Hardware related to RTOS
    //--------------------------------------
    prvHardwareSetup();
    CYGlobalIntEnable; // Enable Global Interrupts
    //---------------------
    // Initialize Hardware
    //---------------------
    //
    // Initialize API
    Init_COM();         // USBUART
    PWM_1_Start(); 
    PWM_2_Start();
   
    CY_SET_REG8(PWM_1_COMPARE1_LSB_PTR, 0);
    CY_SET_REG8(PWM_2_COMPARE1_LSB_PTR, 0);
    
    Control_Reg_1_Write(6);//reset = 0,hi = 1, hi = 1 inc, inc 
    Counter_1_Start();
    Counter_2_Start();
    omega = 0.0;omegarate=0.0;

//    COM_Wait_PC_Term(); // Wait for setup PC Terminal
    //--------------------
    // Create Task
    //--------------------
    xMutex_COM = xSemaphoreCreateMutex();
    //
    RingBufInit(&comrbuf,pbuf,PUT_MSGBUFSIZE);
    bufset=0;
    
    xTaskCreate(Task_LED,    (signed portCHAR *)"LED"    , TASK_STACK_LED    , NULL, TASK_PRIORITY_LED    , NULL);　// PsockのLEDちかちか
    xTaskCreate(Task_USBUART,(signed portCHAR *)"USBUART", TASK_STACK_USBUART, NULL, TASK_PRIORITY_USBUART, NULL);
    //-------------------
    // Start RTOS Kernel 
    //-------------------
    vTaskStartScheduler();
    while(1);
}

//=======================================
// Hooks for Illegal Operations
//=======================================
void vApplicationStackOverflowHook(xTaskHandle pxTask, signed char *pcTaskName)
{
    // The stack space has been execeeded for a task, considering allocating more.
    taskDISABLE_INTERRUPTS();
	for( ;; );
}
void vApplicationMallocFailedHook(void)
{
    // The heap space has been execeeded.
    taskDISABLE_INTERRUPTS();
	for( ;; );
}

//======================================
// Hardware Setup
//======================================
void prvHardwareSetup(void)
{
    // Port layer functions that need to be copied into the vector table.
    extern void xPortPendSVHandler(void);
    extern void xPortSysTickHandler(void);
    extern void vPortSVCHandler(void);
    extern cyisraddress CyRamVectors[];

    // Install the OS Interrupt Handlers.
    CyRamVectors[11] = (cyisraddress) vPortSVCHandler;
    CyRamVectors[14] = (cyisraddress) xPortPendSVHandler;
    CyRamVectors[15] = (cyisraddress) xPortSysTickHandler;

    // Initialize Your Hardware
}

//===========================
// Task : LED
//===========================
void Task_LED(void *pvParameters)
{
    // Block for 500ms
    int16 n;

    const portTickType xDelay
            = 10 / portTICK_RATE_MS;
    // Task Loop
    led_flg = 0;
    while(1)
    {
        timestamp +=20;
        // Toggle LED
       // flg= ~flg;
        LED_Write(led_flg);
        if(bufset == 0){
            set_msgDFL(buff,&n, PUT_MSGBUFSIZE);
            msgsize=n;
            bufset = 1;
        }
        // Block  me
        vTaskDelay(xDelay);
    }
}

//===================================
// Task : USBUART
//===================================
#define BUFSIZE 16
#define GET_BUFSIZE 255
//void com_gets(uint8 *buf,int16 *len );
void run_com1(uint8,uint8);
//int com_search_header();
//

typedef struct kobukidata {
    uint8 id;
    uint8 len;
    uint8 *data;
    int16 speed;
    int16 rad;
} CMD;

// 1. USBからのデータを取り込む
void import_com_data(int8 delta){
    unsigned char ch[64+10]; 
    int16 datalen=0;
    int16 i,cnt;
    
    //delta=64;
    while(delta >0){
        cnt = UART_1_GetRxBufferSize();
        if(cnt > 0){
            for(i=0;i<cnt && i<74;i++){ // 1バイトずつ
                // char型のデータを取り込む
                ch[datalen++] = UART_1_GetChar();
            }
            if(((gcount++) % 5 )<3){
                led_flg=0;
            }
            for(i=0;i<datalen;i++){ 
                // 取り込んだデータをバッファに格納
                RingBufWriteOne(&comrbuf,ch[i]);
            }
            delta -=datalen;
        }
    }
}

// 2. 書き込まれたデータを取り出して，dataに格納する
int8 data_exist(CMD *c){
    char buf[255]; // 8 bit
    int8 i,delta; // delta: データの長さ
    led_flg=1;
    if(RingBufEmpty(&comrbuf) != 0){
        return 0;
    }
    if(RingBufUsed(&comrbuf) < 7){
            if(((gcount++) % 50) <30){
                led_flg=0;
            }
        return 0;
    }
    
    while(RingBufReadOne(&comrbuf)!= 0xAA ){
        if(RingBufEmpty(&comrbuf) != 0){
            return 0;
        }
    }
    if(RingBufReadOne(&comrbuf)!= 0x55 ){
        return 0;
    }                
    c->len = RingBufReadOne(&comrbuf); // データの長さ(バイト数)
    delta =RingBufUsed(&comrbuf) - c->len-1;
    if(delta < 0){
        import_com_data(-delta);
    }
    for(i=0;i<c->len; i++){
        buf[i]=RingBufReadOne(&comrbuf);
    }
    led_flg=0;
    c->id=buf[0];
    for(i=0;i<buf[1];i++){
        c->data[i] = buf[i+2]; // 2, 3, 4, 5 
    }
    return 1;
}
void process(CMD *cmd){
    int32 work;
    if(cmd->id == 1){
        work = cmd->data[0]+cmd->data[1]*256;
        if(work > 32767){
            work = work - 65536;
        }
        cmd->speed = work;
        work = cmd->data[2]+cmd->data[3]*256;
        if(work > 32767){
            work = work - 65536;
        }
        cmd->rad = work;
        
    }
}

void send_feedback(){
    uint8 *pt=buff;
    uint8 len=msgsize;
    if(bufset == 1){
      UART_1_PutArray(pt,len);
      bufset=0;
    }
}
#define LIMITPULSE  200

void Task_USBUART(void *pvParameters) {
    int8 rf, lf;
    int16 pulseL = 0, pulseR = 0, tmp_speed = 0;
    int16 diff_pulse = 0, diff_pulse_old = 0;
    double integral = 0;
    const int16 target_pulse = 0;
    uint16 counter_L = 0, counter_R = 0, counter_L_old = 0, counter_R_old = 0;
    _Bool cnt_flg = false, stop_flg = false;
    CMD cmd;
    cmd.data = recbuffer;
    cmd.speed = 0; // -1200[rpm]
    cmd.rad = 0; // 回転半径
    const portTickType xDelay = 20 / portTICK_RATE_MS;
    cliff_on = 1; dockIr_on = 0; inertial_on = 1; current_on = 0; firm_on = 0; hard_on = 0;
    gyro_on = 0; udid_on = 0; gpio_on = 0;

    while (1) {
        counter_L = Counter_2_ReadCounter();
        counter_R = Counter_1_ReadCounter();
        xSemaphoreTake(xMutex_COM, portMAX_DELAY);

        if (data_exist(&cmd)) {
            process(&cmd);
            lf = 2;
            rf = 4;
            omegarate = 0;
            if (cmd.speed > 1200) cmd.speed = 1200;
            if (cmd.speed < -1200) cmd.speed = -1200;

            if (cmd.rad == 0) {
                if (cmd.speed != 0) {
                    tmp_speed = cmd.speed;
                    diff_pulse = target_pulse - ((counter_R - counter_R_old) - (counter_L - counter_L_old));
                    integral += (diff_pulse + diff_pulse_old) / 2.0 * DELTA_TIME;
                    if (!cnt_flg) {
                        diff_pulse = 0;
                        integral = 0;
                    }
                    stop_flg = false;
                } else {
                    diff_pulse = 0;
                    integral = 0;
                    if (!stop_flg) {
                        tmp_speed *= 0.9;
                        cmd.speed = tmp_speed;
                    }
                }
                pulseL = KP * cmd.speed - (int16)(KI_L * integral);
                pulseR = KP * cmd.speed + (int16)(KI_R * integral);
                diff_pulse_old = diff_pulse;
            } else if (cmd.rad == 1) {
                pulseR = -KP * cmd.speed; // 
                pulseL = +KP * cmd.speed;
                if (pulseR > LIMITPULSE) pulseR = LIMITPULSE;
                if (pulseL > LIMITPULSE) pulseL = LIMITPULSE;
                if (-pulseR > LIMITPULSE) pulseR = -LIMITPULSE;
                if (-pulseL > LIMITPULSE) pulseL = -LIMITPULSE;
                cnt_flg = false;
                stop_flg = true;
            } else {
                // これが本命な計算な気がする．
                if (cmd.speed > abs(cmd.rad)) cmd.speed = abs(cmd.rad);
                if (cmd.speed < -abs(cmd.rad)) cmd.speed = -abs(cmd.rad);
                pulseL = (int)(KP * cmd.speed * (cmd.rad - TREAD / 2) / cmd.rad);
                pulseR = (int)(KP * cmd.speed * (cmd.rad + TREAD / 2) / cmd.rad);
                if (pulseR > LIMITPULSE) pulseR = LIMITPULSE;
                if (pulseL > LIMITPULSE) pulseL = LIMITPULSE;
                if (-pulseR > LIMITPULSE) pulseR = -LIMITPULSE;
                if (-pulseL > LIMITPULSE) pulseL = -LIMITPULSE;
                cnt_flg = false;
                stop_flg = false;
            }
            if (pulseR < 0) {
                pulseR = -pulseR;
                rf = 0;
            }
            if (pulseL < 0) {
                pulseL = -pulseL;
                lf = 0;
            }
            if (pulseL > LIMITPULSE) pulseL = LIMITPULSE;
            if (pulseR > LIMITPULSE) pulseR = LIMITPULSE;
            Control_Reg_1_Write(lf + rf);
            CY_SET_REG8(PWM_1_COMPARE1_LSB_PTR, (uint8)(pulseL));
            CY_SET_REG8(PWM_2_COMPARE1_LSB_PTR, (uint8)(pulseR));
        } else {
            send_feedback();
            len = UART_1_GetRxBufferSize();
            if (len > 0) import_com_data(len);
        }
        xSemaphoreGive(xMutex_COM);
        vTaskDelay(xDelay); 
        counter_L_old = counter_L;
        counter_R_old = counter_R;
    }
}

void add_basic_sensor_data(uint8 *bufp,uint8 *size){
    uint8 workh;
    uint8 workl;
    uint32 workhl;
    const uint8 BUMPER=0;
    const uint8 CLIFF=0;
    const uint8 WHEELDROP=0;
    const uint8 BUTTON=0;
    const uint8 CHARGE=0;
    const uint8 BAT=160;
    const uint8 OC=0;
    bufp[ 0]=0x01;
    bufp[ 1]=0x0f;
    bufp[ 2]=0xff & timestamp;
    bufp[ 3]=timestamp>>8;
    bufp[ 4]=BUMPER;
    bufp[ 5]=WHEELDROP;
    bufp[ 6]=CLIFF;


    workhl = (10*Counter_2_ReadCounter())&0xffff;
    workl = workhl & 0xff;
    workh = (workhl >> 8)& 0xff;
    bufp[ 7]=workl;
    bufp[ 8]=workh;
    workhl = (10*Counter_1_ReadCounter())&0xffff;
    workl = workhl & 0xff;
    workh = (workhl >> 8)& 0xff;
    bufp[ 9]=workl;
    bufp[10]=workh;
    
    bufp[11]=lpwm;
    bufp[12]=rpwm;
    bufp[13]=BUTTON;
    bufp[14]=CHARGE;
    bufp[15]=BAT;
    bufp[16]=OC;
    *size=17;
}

void add_dockIr_data(uint8 *bufp,uint8 *size){
    bufp[0]=0x03;
    bufp[1]=0x03;
    bufp[2]=0;
    bufp[3]=0x20;
    bufp[4]=0;
    *size=5;
}
void add_inertial_data(uint8 *bufp,uint8 *size){
    int32 w,wr;
    if(omega>3.141592){
        omega -= 2*3.141592;
    }else if( omega < -3.141592){
        omega += 2*3.141592;
    }
    w = omega * 18000/3.14159;
    wr = omegarate * 18000/3.14159;
    if( w < 0 ) {
        w = 65536 + w;
    }
    if( wr < 0 ) {
        wr = 65536 + wr;
    }
    bufp[0]=0x04;
    bufp[1]=7;
    bufp[2]=w & 0xff;
    bufp[3]=w>>8;
    bufp[4]=wr & 0xff;
    bufp[5]=wr >> 8;
    bufp[6]=0;
    bufp[7]=0;
    bufp[8]=0;
    *size=9;
}
void add_cliff_data(uint8 *bufp,uint8 *size){
    bufp[0]=0x05;
    bufp[1]=0x06;
    bufp[2]=0x20;
    bufp[3]=0;
    bufp[4]=0x20;
    bufp[5]=0;
    bufp[6]=0x20;
    bufp[7]=0;
    *size=8;
}
void add_current_data(uint8 *bufp,uint8 *size){
    bufp[0]=0x06;
    bufp[1]=2;
    bufp[2]=2;
    bufp[3]=2;
    *size=4;
}
void add_hard_data(uint8 *bufp,uint8 *size){
    bufp[0]=0x0a;
    bufp[1]=0x04;
    bufp[2]=1;
    bufp[3]=0;
    bufp[4]=1;
    bufp[5]=0;
    *size=6;
}
void add_firm_data(uint8 *bufp,uint8 *size){
    bufp[0]=0x0b;
    bufp[1]=0x04;
    bufp[2]=1;
    bufp[3]=0;
    bufp[4]=1;
    bufp[5]=0;
    *size=6;
}

void add_gyro_data(uint8 *bufp,uint8 *size){
    bufp[0]=0xd;
    bufp[1]=8;
    bufp[2]=1;
    bufp[3]=3;
    bufp[4]=0;
    bufp[5]=0;
    bufp[6]=0;
    bufp[7]=0;
    bufp[8]=0;
    bufp[9]=0;
    *size=10;
}
void add_gpio_data(uint8 *bufp,uint8 *size){
    bufp[0]=0x10;
    bufp[1]=0x10;
    bufp[2]=0;
    bufp[3]=0;
    bufp[4]=0;
    bufp[5]=0;
    bufp[6]=0;
    bufp[7]=0;
    bufp[8]=0;
    bufp[9]=0;
    bufp[10]=0;
    bufp[11]=0;
    bufp[12]=0;
    bufp[13]=0;
    bufp[14]=0;
    bufp[15]=0;
    bufp[16]=0;
    bufp[17]=0;
    *size=18;
}
void add_udid_data(uint8 *bufp,uint8 *size){
    bufp[0]=0x13;
    bufp[1]=0xc;
    bufp[2]=0;
    bufp[3]=0;
    bufp[4]=0;
    bufp[5]=0;
    bufp[6]=0;
    bufp[7]=0;
    bufp[8]=0;
    bufp[9]=0;
    bufp[10]=0;
    bufp[11]=0;
    bufp[12]=0;
    bufp[13]=0;
    *size=14;
}

void set_msgDFL(uint8 *buf, int16 *maxsize,int bufsize){

    int16 i;
    uint8 cd;
    uint8 *bufp =buf; 
    uint8 n;
    uint8 len=0;
    if(bufsize < 3+17) return ;
    bufp[0]=0xaa;
    bufp[1]=0x55;
    len =3;
    bufp+=3;
//    bufp[2]=3+16+1;//?    
    add_basic_sensor_data(bufp,&n);
    len+=n;
    bufp+=n;
    if( dockIr_on){//3
        add_dockIr_data(bufp,&n);
        len+=n;        bufp+=n;
    }
    if( inertial_on){//4
        add_inertial_data(bufp,&n);
        len+=n;    bufp+=n;
    }
    if( cliff_on){//5
        add_cliff_data(bufp,&n);
        len+=n;    bufp+=n;
    }
    if(current_on){//6
        add_current_data(bufp,&n);
        len+=n;
        bufp+=n;
    }
    if( hard_on){//a
        add_hard_data(bufp,&n);
        len+=n;    bufp+=n;
    }
    if( firm_on){//b
        add_firm_data(bufp,&n);
        len+=n;    bufp+=n;   
    }

    if( gyro_on){//d
        add_gyro_data(bufp,&n);
        len+=n;
        bufp+=n;
    }
    if( gpio_on ){//10
        add_gpio_data(bufp,&n);
        len+=n;    bufp+=n;
    }
    if( udid_on){//13
        add_udid_data(bufp,&n);
        len+=n;    bufp+=n;
    }

    
    buf[2]=len-3;
    cd=0;
    for(i=2; i<len; i++){
        cd ^=buf[i];
    }
    buf[len] = cd;
    *maxsize = len+1;
}
    

//=========================================================
// End of Program
//=========================================================
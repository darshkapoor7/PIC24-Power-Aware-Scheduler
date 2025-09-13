/* Microchip Technology Inc. and its subsidiaries.  You may use this software 
 * and any derivatives exclusively with Microchip products. 
 * 
 * THIS SOFTWARE IS SUPPLIED BY MICROCHIP "AS IS".  NO WARRANTIES, WHETHER 
 * EXPRESS, IMPLIED OR STATUTORY, APPLY TO THIS SOFTWARE, INCLUDING ANY IMPLIED 
 * WARRANTIES OF NON-INFRINGEMENT, MERCHANTABILITY, AND FITNESS FOR A 
 * PARTICULAR PURPOSE, OR ITS INTERACTION WITH MICROCHIP PRODUCTS, COMBINATION 
 * WITH ANY OTHER PRODUCTS, OR USE IN ANY APPLICATION. 
 *
 * IN NO EVENT WILL MICROCHIP BE LIABLE FOR ANY INDIRECT, SPECIAL, PUNITIVE, 
 * INCIDENTAL OR CONSEQUENTIAL LOSS, DAMAGE, COST OR EXPENSE OF ANY KIND 
 * WHATSOEVER RELATED TO THE SOFTWARE, HOWEVER CAUSED, EVEN IF MICROCHIP HAS 
 * BEEN ADVISED OF THE POSSIBILITY OR THE DAMAGES ARE FORESEEABLE.  TO THE 
 * FULLEST EXTENT ALLOWED BY LAW, MICROCHIP'S TOTAL LIABILITY ON ALL CLAIMS 
 * IN ANY WAY RELATED TO THIS SOFTWARE WILL NOT EXCEED THE AMOUNT OF FEES, IF 
 * ANY, THAT YOU HAVE PAID DIRECTLY TO MICROCHIP FOR THIS SOFTWARE.
 *
 * MICROCHIP PROVIDES THIS SOFTWARE CONDITIONALLY UPON YOUR ACCEPTANCE OF THESE 
 * TERMS. 
 */

/* 
 * File:   
 * Author: 
 * Comments:
 * Revision history: 
 */

// This is a guard condition so that contents of this file are not included
// more than once.  
#ifndef SCHEDULER_H
#define	SCHEDULER_H

#include <xc.h> // include processor files - each processor file is guarded.  
#include <stdbool.h>

typedef struct{
    void           (*taskFunc)(void);
    uint16_t        taskPeriod;
    uint16_t        elapsedTime;
}Task;

typedef enum{
    SPOR_NONE = 0,
    SPOR_PRINT_ERR,        
    SPOR_TOGGLE_LED0,
    SPOR_TOGGLE_LED1,
    SPOR_SLEEP,
    SPOR_PRINT_POW,
    SPOR_EXIT_IDLE,
}Spor_cmd;

typedef struct {
    // Save LED pin states (scheduler LEDs on RA0/RA1, RGB on RD9/10/11)
    uint8_t ra0_lat, ra1_lat, ra0_tris, ra1_tris;
    uint8_t rd9_lat, rd10_lat, rd11_lat, rd9_tris, rd10_tris, rd11_tris;
} QuietSnapshot;

#define LED_LATA0       LATAbits.LATA0  
#define LED_LATA1       LATAbits.LATA1
#define LED_RED         LATDbits.LATD9
#define LED_GREEN       LATDbits.LATD10
#define LED_BLUE        LATDbits.LATD11
#define UART_LAT_RX     LATDbits.LATD14

#define LED_TRISA0      TRISAbits.TRISA0
#define LED_TRISA1      TRISAbits.TRISA1  
#define LED_TRISD9      TRISDbits.TRISD9
#define LED_TRISD10     TRISDbits.TRISD10
#define LED_TRISD11     TRISDbits.TRISD11
#define BTN_TRISG15     TRISGbits.TRISG15
#define BTN_TRISD13     TRISDbits.TRISD13
#define UART_TX         TRISDbits.TRISD15
#define UART_RX         TRISDbits.TRISD14

#define TR1_ANSEL   ANSBbits.ANSB8
#define TR1_TRIS    TRISBbits.TRISB8
#define TR1_MASK    (1u<<8)   // OUT1: RB8

#define TR2_ANSEL   ANSBbits.ANSB9
#define TR2_TRIS    TRISBbits.TRISB9
#define TR2_MASK    (1u<<9)   // OUT2: RB9


#define MAX_TASK        8
#define BAUDRATE        115200
#define FCY             4000000UL
#define XTAL_FREQ       8000000UL
#define IDLE_TICK_HZ    1u   // Timer1 in IDLE: 1 Hz => step every 1000 ms
#define SPOR_Q_SIZE     32  
#define U2TX_Q_SIZE     64
#define CLI_Q_BUF       64

extern volatile bool tickFlag;
extern volatile bool aperiodicMode;
extern volatile bool sporadicFlag;
extern uint8_t rgbState;
extern uint16_t rgbCount; 

void initSystem(void);
void initTimer1(void);
void setTickHz(uint32_t hz, uint8_t prescale_sel);
void aperiodicTask(void);
void enter_aperiodicTask(void);
void exit_aperiodicTask(void);
void uart2INIT(void);
void uart2_write(const void *data, uint16_t len); // enqueue len bytes
void uart2_puts(const char *s);                   // enqueue a C-string
void uart2_flush(void);                           // block until all sent
void sporadicTask(void);
bool sporadic_enqueue(Spor_cmd cmd);
bool sporadic_dequeue(Spor_cmd *out);
void toggle_LED1(void);
void toggle_LED2(void);
void task_print_power(void);
void measure_sleep_power(uint16_t sleep_ms);
void schedulerRun(void);

//CLI API
void cli_init(void);
void cli_service(void); 
void request_idle_oneshot(void (*fn)(void), uint16_t ms);
bool run_idle_oneshot_if_pending(void);


#endif

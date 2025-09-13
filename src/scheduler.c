/*
 * File:   scheduler.c
 * Author: Dkapoor
 *
 * Created on 20 July 2025, 11:30
 */


#include "xc.h"
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h> 
#include <stdio.h>
#include "scheduler.h"
#include "button.h"
#include "ina219.h" 

/*
 * delay_ms(ticks)
 * ---------------
 * Very coarse, CPU-busy delay loop. Not calibrated; used only for tiny settle
 * delays after mode transitions (e.g., wake from SLEEP/IDLE).
 * NOTE: Blocks the CPU; do not use in time-critical paths.
 */
void delay_ms(int ticks){
    for(int i=0; i<ticks; i++){
        for(int j=0;j<255;j++);
    }
}


/* UART2 software TX queue (filled by uart2_write; drained by U2TX ISR) */
static volatile uint8_t tx_head = 0, tx_tail = 0;
static uint8_t tx_buf[U2TX_Q_SIZE];

/* Sporadic command queue (commands from CLI/keys processed in foreground) */
static volatile uint8_t s_q_head = 0 , s_q_tail = 0;
static Spor_cmd s_q_buf[SPOR_Q_SIZE];

/* CLI line buffer (built in RX ISR, executed in foreground) */
static volatile uint8_t cli_len = 0;
static volatile bool    cli_ready;
static char             cli_line[CLI_Q_BUF];

/*"run a one-shot function, then request IDLE" helper */
static volatile void(*IDLE_fn)(void) = NULL;
static volatile uint16_t IDLE_fn_ms = 0;
static volatile bool IDLE_fn_flag = false;

/* Quiet window controls (used by power measurement helpers) */
static volatile bool g_pause_scheduler  = false;  // when true, schedulerRun() early-returns
static volatile bool g_quiet_aperiodic  = false;  // when true, aperiodicTask() does nothing


Task task[MAX_TASK] = {
    {toggle_LED1, 500 , 0},
    {toggle_LED2, 1000 , 0}    
};

static inline void quiet_enter(QuietSnapshot *q)
{
    // Pause periodic/aperiodic work
    g_pause_scheduler = true;
    g_quiet_aperiodic = true;

    // Save LEDs
    q->ra0_lat  = LATAbits.LATA0; q->ra1_lat  = LATAbits.LATA1;
    q->ra0_tris = TRISAbits.TRISA0; q->ra1_tris = TRISAbits.TRISA1;
    q->rd9_lat  = LATDbits.LATD9;  q->rd10_lat = LATDbits.LATD10; q->rd11_lat = LATDbits.LATD11;
    q->rd9_tris = TRISDbits.TRISD9; q->rd10_tris = TRISDbits.TRISD10; q->rd11_tris = TRISDbits.TRISD11;

    // Turn off scheduler LEDs
    LATAbits.LATA0 = 0; LATAbits.LATA1 = 0;
    // Force RGB OFF and high-Z so no LED current flows through MCU VDD
    LATDbits.LATD9  = 0; LATDbits.LATD10 = 0; LATDbits.LATD11 = 0;
    TRISDbits.TRISD9  = 1; TRISDbits.TRISD10 = 1; TRISDbits.TRISD11 = 1;

    // Small settle so edges/interrupts don't skew the sample
    delay_ms(2);
}

static inline void quiet_exit(const QuietSnapshot *q)
{
    // Restore LED directions first, then levels
    TRISAbits.TRISA0 = q->ra0_tris; TRISAbits.TRISA1 = q->ra1_tris;
    TRISDbits.TRISD9 = q->rd9_tris; TRISDbits.TRISD10 = q->rd10_tris; TRISDbits.TRISD11 = q->rd11_tris;
    LATAbits.LATA0   = q->ra0_lat;  LATAbits.LATA1   = q->ra1_lat;
    LATDbits.LATD9   = q->rd9_lat;  LATDbits.LATD10  = q->rd10_lat;  LATDbits.LATD11  = q->rd11_lat;

    // Resume normal work
    g_quiet_aperiodic = false;
    g_pause_scheduler = false;
}

/*
 * cli_clear()
 * -----------
 * Reset the command line buffer state.
 * Called from CLI service after a command is consumed.
 *
 */
static inline void cli_clear(void){
    cli_len = 0;
    cli_ready = false;
}

/*
 * strcasecmp_local(a,b)
 * ---------------------
 * Case-insensitive string compare (local, fixed to avoid pulling in libc variants).
 * Returns 0 if equal, <0 or >0 otherwise.
 */
static int strcasecmp_local(const char *a, const char *b){
    for(;;){
           unsigned char char_a = (unsigned char)*a++;
           unsigned char char_b = (unsigned char)*b++;

            if(char_a >= 'A' && char_a <= 'Z'){
                char_a = (unsigned char)(char_a + ('a' - 'A'));
            }
            if(char_b >= 'A' && char_b <= 'Z'){
                char_b = (unsigned char)(char_b + ('a' - 'A'));
            }
            if(char_a != char_b){
                return (int)char_a - (int)char_b;
            }
            if(char_a == '\0')
                return 0;
        }
}

/*
 * char_cmp/char_eq
 * ----------------
 * Thin wrappers around strcasecmp_local() for clarity.
 */
static inline int char_cmp(const char *a , const char *b){
    return strcasecmp_local(a,b);
}


static inline bool char_eq(const char *a , const char *b){
    return strcasecmp_local(a,b) == 0;
}

/*
 * cli_init()
 * ----------
 * Initialize CLI state (clears input buffer and ready flag).
 */
void cli_init(){
    cli_clear();
}

/*
 * tx_empty()
 * ----------
 * Return true if the UART2 software TX queue is empty.
 * Used by uart2_flush() and the TX ISR.
 */
static inline bool tx_empty(void){
    return tx_head == tx_tail;
}

/*
 * cli_rx_char(c)
 * --------------
 * Accumulate a CLI character from UART RX ISR. When CR/LF arrives and there is
 * at least one character, marks the line as ready (cli_ready=true).
 * NOTE: Called from ISR context.
 */
static inline void cli_rx_char(char c){
    if(c == '\r' || c == '\n'){
        if(cli_len > 0){
            cli_line[cli_len] = '\0';
            cli_ready = true;
        }
        return;
    }
    
    if(cli_len < (CLI_Q_BUF - 1)){
        cli_line[cli_len++] = c;
    }
    else{
        cli_len = 0;
    }   
}

/*
 * tx_full()
 * ---------
 * Return true if the UART2 software TX queue is full (next write would overrun).
 */
static inline bool tx_full(void){
    return ((uint8_t)(tx_head + 1u) & (U2TX_Q_SIZE - 1u)) == tx_tail;
}


/*
 * map_to_char(c)
 * --------------
 * Map a single keystroke to a Spor_cmd enum (legacy single-key commands).
 * Most commands now come from full CLI lines; this remains for convenience.
 */
static inline Spor_cmd map_to_char(char c){
    switch(c){
        case '1': return SPOR_TOGGLE_LED0;
        case '2': return SPOR_TOGGLE_LED1;
        case 'S': case 's': return SPOR_SLEEP;
        case 'P': case 'p': return SPOR_PRINT_POW;
        case 'W': case 'w': return SPOR_EXIT_IDLE;
        default: return SPOR_NONE;
    }
}

/*
 * sleep_wdt(ms)
 * -------------
 * Enter true SLEEP for approximately 'ms' milliseconds and wake on Timer1 match.
 * Implementation:
 *   - Temporarily reconfigure T1 to use SOSC (TCS=1) and 1:256 prescale (?128 Hz).
 *   - Program PR1 to match after the requested duration, enable T1 interrupt.
 *   - Disable UART2 RX interrupt to avoid immediate wake on pending RX.
 *   - Enter SLEEP (pwrsav(0)); wake on T1 interrupt; restore normal tick to ACTIVE/IDLE profile.
 */ 
static inline void  sleep_wdt(uint16_t ms){
    uint16_t save_T1CON = T1CON , save_PR1 = PR1;
    uint8_t save_IE = IEC0bits.T1IE;
    
    IFS0bits.T1IF = 0;
    T1CONbits.TON = 0;
    IEC0bits.T1IE = 0;
    
    __builtin_write_OSCCONL(OSCCON | (1u << _OSCCON_SOSCEN_POSITION)); // some parts use SOSCEN
    
    T1CON = 0;
    T1CONbits.TCS = 1;
    T1CONbits.TCKPS = 3; //1:256 ? 32768/256 = 128 Hz

    uint32_t ticks = ((uint32_t)ms * 128u + 999u) / 1000u;
    if(ticks == 0)ticks = 1;
    if(ticks > 65535u)ticks = 65535u;
    PR1 = (uint16_t)ticks - 1;
    TMR1 = 0;
    
    IFS0bits.T1IF = 0;
    IEC0bits.T1IE = 1;         // T1 match will wake us from SLEEP
    T1CONbits.TON = 1;

    // Avoid immediate wake due to pending UART RX
    IEC1bits.U2RXIE = 0; IFS1bits.U2RXIF = 0;

   // out2_pulse();
    __builtin_pwrsav(0);       // enter SLEEP ? wakes on Timer1 interrupt

    // ---- resumes here after wake ----
    T1CONbits.TON = 0;
    IEC0bits.T1IE = 0;
    IFS0bits.T1IF = 0;

    // Restore your normal 1 kHz scheduler tick
    if(aperiodicMode){
        setTickHz(IDLE_TICK_HZ, 3);
    }else{
        setTickHz(1000, 0);
    }
    // Restore UART RX
    IFS1bits.U2RXIF = 0; IEC1bits.U2RXIE = 1;

    LED_GREEN = 1;
    delay_ms(100);
    LED_GREEN = 0;
}

/*
 * _T1Interrupt()
 * --------------
 * Timer1 ISR.
 *   ACTIVE: set tickFlag (unless a quiet window is pausing the scheduler).
 *   IDLE  : run aperiodicTask() (unless a quiet window asked to suppress it).
 * Keep ISR work minimal; avoid heavy processing here.
 */
void __attribute__((__interrupt__, no_auto_psv)) _T1Interrupt(void){
    IFS0bits.T1IF = 0;

    if (!aperiodicMode) {
        if (!g_pause_scheduler) {
            //out1_pulse();           // <-- only when not paused
            tickFlag = true;
        }
        return;
    }
    if (!g_quiet_aperiodic) {
        //out1_pulse();               // optional for IDLE path too
        aperiodicTask();
    }
} 

/*
 * _U2RXInterrupt()
 * ----------------
 * UART2 RX ISR.
 * - Clears OERR if needed.
 * - Discards bytes with frame/parity errors.
 * - Feeds characters to CLI line buffer via cli_rx_char().
 * - When a line is complete (CR/LF), raises sporadicFlag so main loop runs CLI.
 */
void __attribute__((__interrupt__, auto_psv)) _U2RXInterrupt(void) {
    if (U2STAbits.OERR) U2STAbits.OERR = 0;

    while (U2STAbits.URXDA) {
        char c = U2RXREG;
        
        // Toggle to show activity
        //LATAbits.LATA0 ^= 1;

        if (U2STAbits.FERR || U2STAbits.PERR) {
            volatile char dump = c; (void)dump;
            continue;
        }
        
        Spor_cmd cmd = map_to_char(c);
   
        // Only collect into the CLI buffer; no single-key actions here
        cli_rx_char(c);

        // When Enter arrives, run the CLI soon (in the foreground/task context)
        if (cli_ready) {
            out1_pulse();
            sporadicFlag = true;
        }
    }
    IFS1bits.U2RXIF = 0;
}

/*
 * _U2TXInterrupt()
 * ----------------
 * UART2 TX ISR.
 * - Moves bytes from software TX queue into HW TXREG until either queue is empty
 *   or the hardware FIFO reports full.
 * - Disables TX interrupt when the queue drains.
 */
void __attribute__((__interrupt__, auto_psv)) _U2TXInterrupt(void){
    // Fill TXREG until HW FIFO full or SW queue empty
    while (!tx_empty() && !U2STAbits.UTXBF){
        U2TXREG = tx_buf[tx_tail];
        tx_tail = (uint8_t)((tx_tail + 1u) & (U2TX_Q_SIZE - 1u));
    }
    if (tx_empty()){
        IEC1bits.U2TXIE = 0;    // nothing left to send
    }
    IFS1bits.U2TXIF = 0;        // clear TX flag
}

/*
 * initSystem()
 * ------------
 * Configure GPIO directions and initial states for LEDs, buttons, UART pins.
 * NOTE: Peripheral remap for UART is done in uart2INIT().
 */
void initSystem(void){
    LED_LATA0 = 0;
    LED_LATA1 = 0;
    LED_TRISA0 = 0;
    LED_TRISA1 = 0;
    
    LED_RED = 0;
    LED_GREEN = 0;
    LED_BLUE = 0;
    LED_TRISD9 = 0;
    LED_TRISD10 = 0;
    LED_TRISD11 = 0;
    BTN_TRISD13 = 1;
    BTN_TRISG15 = 1;
    UART_TX = 0;
    UART_RX = 1;
    UART_LAT_RX = 0;
}

/*
 * setTickHz(hz, prescale_sel)
 * ---------------------------
 * Program Timer1 to generate a periodic interrupt at 'hz'.
 * prescale_sel: 0->1:1, 1->1:8, 2->1:64, 3->1:256.
 * - Computes PR1 with rounding.
 * - Enables T1 interrupt.
 * Used by both ACTIVE (1 kHz) and IDLE (IDLE_TICK_HZ with prescale 1:256) profiles.
 */
void setTickHz(uint32_t hz, uint8_t prescale_sel){
    if(hz == 0)return;
    
    uint32_t prescale = (prescale_sel == 0) ? 1: (prescale_sel == 1) ? 8 : (prescale_sel == 2) ? 64 : 256;    
    
    T1CON = 0;
    T1CONbits.TCKPS = (prescale_sel & 0x3);
    
    uint32_t denom = prescale * hz;
    uint32_t pr = (FCY + (denom/2)) / denom; // rounded
    if (pr == 0) pr = 1;
    pr -= 1;
    if (pr > 0xFFFFu) pr = 0xFFFFu;

    PR1  = (uint16_t)pr;
    TMR1 = 0;

    IPC0bits.T1IP = 2;
    IFS0bits.T1IF = 0;
    IEC0bits.T1IE = 1;
    T1CONbits.TON = 1;
}

/*
 * enter_aperiodicTask()
 * ---------------------
 * Switch the system into the IDLE profile:
 *   - Lower Timer1 tick to IDLE_TICK_HZ (prescale 1:256).
 *   - Set 'aperiodicMode' true so T1 ISR runs aperiodicTask() instead of tickFlag.
 *   - Apply peripheral SIDL/PMD settings for low power (button.c helpers).
 *   - Reset RGB state and ensure LEDs are off.
 *   - Disable S1 button debouncer from triggering re-entry.
 */
void enter_aperiodicTask(void){
    if (!IEC0bits.T1IE || T1CONbits.TON == 0) {
    LED_LATA1 = 1;
    }
    setTickHz(IDLE_TICK_HZ, 3);
    
    aperiodicMode = true;
    
    btn1_enable(false);

    set_peripherals_sidl_profile_for_idle();
    //T1CONbits.TSIDL = 0;          // run T1 in IDLE (critical)   
    LED_RED = 0; LED_GREEN = 0; LED_BLUE = 0;
    
    //Scheduler LEDs are off in IDLE
    LED_LATA0 = 0; LED_LATA1 = 0;
}

/*
 * exit_aperiodicTask()
 * --------------------
 * Leave IDLE profile and return to ACTIVE:
 *   - Restore 1 kHz scheduler tick.
 *   - Restore active peripheral profile.
 *   - Ensure RGB off.
 *   - Re-enable button to allow next IDLE request.
 */
void exit_aperiodicTask(void){
   req_enter_idle = false;
   aperiodicMode = false;

    // Restore scheduler tick
    setTickHz(1000, 0);            // 1 kHz

    set_peripherals_active_profile();

    delay_ms(2);
    // Stop RGB
    LED_RED = 0; LED_GREEN = 0; LED_BLUE = 0;

    // Allow S1 to request IDLE again
    btn1_enable(true);
}

/*
 * aperiodicTask()
 * ---------------
 * Work run directly from the Timer1 ISR in IDLE mode at a reduced tick.
 * Keep this extremely light; it must complete quickly inside the ISR.
 * This sample just blinks the BLUE LED.
 */
void aperiodicTask(void){
    if(g_quiet_aperiodic)return;
    LED_BLUE ^= 1;
}

/*
 * uart2INIT()
 * -----------
 * Initialize UART2:
 *   - Remap pins (U2RX on RP43, U2TX on RP5).
 *   - Configure BRG for BAUDRATE with BRGH=1.
 *   - Enable RX interrupt (TX interrupt is enabled lazily when data is queued).
 */
void uart2INIT(void){
    
     __builtin_write_OSCCONL(OSCCON & 0xBF);   // clear IOLOCK
    
    RPINR19bits.U2RXR = 43;   // RD14 is RP43  (U2RX)   <-- verify in xc.h/DFP
    RPOR2bits.RP5R = 5; 
    __builtin_write_OSCCONL(OSCCON | 0x40);   // set IOLOCK
    
    TRISDbits.TRISD14 = 1;    // RX
    TRISDbits.TRISD15 = 0;    // TX

    LATDbits.LATD14 = 0;
    LATDbits.LATD15 = 0;
    
    U2MODE = 0;
    U2STA  = 0;

    U2MODEbits.BRGH = 1;
    U2BRG = ((FCY + (4UL * BAUDRATE) - 1) / (4UL * BAUDRATE)) - 1;
    U2MODEbits.UARTEN = 1;
    U2STAbits.UTXEN   = 1;

    IPC7bits.U2RXIP = 2;
    IFS1bits.U2RXIF  = 0;
    IEC1bits.U2RXIE  = 1;
    
    IPC7bits.U2TXIP = 2;
    IFS1bits.U2TXIF = 0;
    IEC1bits.U2TXIE = 0;    //Leave TX IRQ disabled until we have data
    
}

/*
 * uart2_write(data, len)
 * ----------------------
 * Queue 'len' bytes to the software TX FIFO and kick the TX interrupt
 * to start draining the queue into the UART hardware.
 * Blocks if the software queue is full (spins until space).
 */
void uart2_write(const void* data, uint16_t len){
    const uint8_t *p = (const uint8_t *)data;
    while(len--){
        uint8_t next = ((uint8_t)(tx_head + 1u) & (U2TX_Q_SIZE - 1u));
        while (next == tx_tail) { /* queue full: wait or break to drop */ }
        tx_buf[tx_head] = *p++;        // <-- correct index
        tx_head = next;
    }
    IEC1bits.U2TXIE = 1;
    IFS1bits.U2TXIF = 1;
}

/*
 * uart2_puts(s)
 * -------------
 * Convenience helper: send a zero-terminated string through uart2_write().
 */
void uart2_puts(const char* s){
    uart2_write(s, (uint16_t)strlen(s));
}

/*
 * uart2_flush()
 * -------------
 * Busy-wait until the software queue is empty and the hardware shifter is empty.
 * Use sparingly; this blocks the CPU.
 */
void uart2_flush(void){
    // Wait until SW queue is empty?
    while (!tx_empty()) { /* spin */ }
    // ?and HW shifter is empty
    while (!U2STAbits.TRMT) { /* spin */ }
}

/*
 * measure_sleep_power(sleep_ms)
 * -----------------------------
 * Take two snapshots around a timed SLEEP window:
 *   - snapshot #1 right before entering SLEEP (quiet window active)
 *   - SLEEP for 'sleep_ms' using Timer1 on SOSC
 *   - snapshot #2 immediately after wake (quiet window still active)
 * Prints both as "SLEEP ..." (pre) and "AWAKE ..." (post).
 */
void measure_sleep_power(uint16_t sleep_ms){
    
    uint16_t v1, v2;
    int32_t i1, i2;
    uint32_t p1, p2;
    
    QuietSnapshot snap;
    quiet_enter(&snap);
    
    bool sleep = ina219_read_all(&v1, &i1, &p1);
    uart2_flush();
    sleep_wdt(sleep_ms);
    //delay_ms(2);
    bool awake = ina219_read_all(&v2, &i2, &p2);
    
    quiet_exit(&snap);
    
    if(sleep){
        uint32_t i1_abs = (i1 >= 0) ? (uint32_t)i1 : (uint32_t)-i1;
       //uint32_t p1_calc = ((uint32_t)v1 * (uint32_t)i1 ? i1 : -i1)/1000;
        uint32_t p1_calc = (uint32_t)(((uint64_t)v1 * (uint64_t)i1_abs)/1000);
       char pre[64];
       snprintf(pre,sizeof(pre),"-->SLEEP %ums: V=%u.%03uV I=%ldmA P_ina=%lumW P_calc=%lumW\r\n",
                    (unsigned)sleep_ms, v1/1000, v1%1000, (long)i1,
                    (unsigned long)p1, (unsigned long)p1_calc);
            uart2_puts(pre);
    }else{
        uart2_puts("INAREAD ERROR\r\n");
    }
    
    if(awake){
        //uint32_t p2_calc = ((uint32_t)v2 * (uint32_t)i2 ? i2 : -i2)/1000;
        uint32_t i2_abs = (i2 >= 0) ? (uint32_t)i2 : (uint32_t)-i2;
        uint32_t p2_calc = (uint32_t)(((uint64_t)v2 * (uint64_t)i2_abs)/1000);
        char post[64];
        snprintf(post,sizeof(post),"-->AWAKE %ums: V=%u.%03uV I=%ldmA P_ina=%lumW P_calc=%lumW\r\n",
                (unsigned)sleep_ms, v2/1000, v2%1000, (long)i2,
                (unsigned long)p2, (unsigned long)p2_calc);
        uart2_puts(post);
    }else{
        uart2_puts("INAREAD ERROR\r\n"); 
    }
 
}

/*
 * sporadic_enqueue(cmd)
 * ---------------------
 * Enqueue a sporadic command (from CLI/RX ISR) to be processed in foreground.
 * Returns false if the queue is full or cmd==NONE.
 * Sets 'sporadicFlag' to notify the main loop.
 */
bool sporadic_enqueue(Spor_cmd cmd){
    if(cmd == SPOR_NONE)return false;
    bool empty = (s_q_head == s_q_tail);
    uint8_t next = (uint8_t)((s_q_head + 1u) % SPOR_Q_SIZE); 
    if(next == s_q_tail) return false;
    s_q_buf[s_q_head] = cmd;
    s_q_head = next;
    sporadicFlag = true;
    //if(empty)out1_pulse();
    return true;
}


/*
 * sporadic_dequeue(out)
 * ---------------------
 * Pop one command from the sporadic queue into *out; returns false if empty.
 */

bool sporadic_dequeue(Spor_cmd* out){
    if(s_q_head == s_q_tail)return false;
    *out = s_q_buf[s_q_tail];
    s_q_tail = (uint8_t)((s_q_tail + 1u) % SPOR_Q_SIZE);
    
    return true;
}

/*
 * sporadicTask()
 * --------------
 * Execute all pending sporadic commands (CLI operations, print power, sleep).
 * Runs in the main loop context (not in ISR).
 */
void sporadicTask(void){
    
    cli_service();
    
    Spor_cmd cmd;
    
    while (sporadic_dequeue(&cmd)){
        //out2_pulse();
        switch(cmd){
            case SPOR_TOGGLE_LED0:
                uart2_puts("OK\r\n");
                uart2_flush();
                LED_LATA0 ^= 1;
                break;
                
            case SPOR_TOGGLE_LED1:
                uart2_puts("OK\r\n");
                LED_LATA1 ^= 1;
                break;
            
            case SPOR_SLEEP:
                //LED_GREEN ^= 1;
                uart2_puts("OK\r\n");
                uart2_flush();
                //sleep_wdt(2000);
                out1_pulse();
                measure_sleep_power(2000);
                break;
            
            case SPOR_EXIT_IDLE:
                if(req_enter_idle){
                    req_enter_idle = false;
                    uart2_puts("OK\r\n");
                }else{
                    (void)sporadic_enqueue(SPOR_PRINT_ERR);
                }
                break;
            
            case SPOR_PRINT_POW:              
                out1_pulse();
                task_print_power();
                uart2_puts("OK\r\n");  
                break;
            
            case SPOR_PRINT_ERR:
                uart2_puts("\nIncorrect Command\r\n");
                break;
             
            default:
                (void)sporadic_enqueue(SPOR_PRINT_ERR);
                break;
        }
    }
    sporadicFlag = false;
}
/*
 * toggle_LED1()
 * -------------
 * Demo task: toggles scheduler LED0 (RA0). Wrapped with trace pulses.
 */
void toggle_LED1(){
    out1_hi();
    LED_LATA0 ^= 1;
    out1_lo();
}

/*
 * toggle_LED2()
 * -------------
 * Demo task: toggles scheduler LED0 (RA1). Wrapped with trace pulses.
 */
void toggle_LED2(){
    out2_pulse();
    LED_LATA1 ^= 1;
}

/*
 * task_print_power()
 * ------------------
 * Print instantaneous bus voltage (mV), current (mA), and power:
 *   P_ina  = INA219 power register (quantized by its Power_LSB)
 *   P_calc = (V_bus[mV] * |I[mA]| + 500) / 1000  (rounded to nearest mW)
 */
void task_print_power(void){
    uint16_t bus_mv; 
    int32_t  cur_mA; 
    uint32_t p_mW;   // INA219 power register (quantized)
    
    
    if (ina219_read_all(&bus_mv, &cur_mA, &p_mW)) {
        // 1) Higher-resolution power (mW) from V×I
        uint32_t p_calc_mW = ((uint32_t)bus_mv * (uint32_t)(cur_mA >= 0 ? cur_mA : -cur_mA)) / 1000u;

        // 2) Print both
        char line[96]; // a bit larger buffer for the extra text
        const char *mode = aperiodicMode ? "IDLE" : "ACTIVE";
        snprintf(line, sizeof(line),
                 "%s: V=%u.%03uV I=%ldmA P_ina=%lumW P_calc=%lumW\r\n",
                 mode,
                 bus_mv/1000, bus_mv%1000,
                 (long)cur_mA,
                 (unsigned long)p_mW,
                 (unsigned long)p_calc_mW);
        uart2_puts(line);
    } else {
        uart2_puts("INA219 read error\r\n");
    }
    
    
}

/*
 * find_free_slot()
 * ----------------
 * Return the first free task slot index, or -1 if none available.
 */
static int find_free_slot(void){
    for(int i=0; i<MAX_TASK; i++){
        if(task[i].taskFunc == NULL)return i;
    }
    return -1;
}

/*
 * task_set(idx, fn, period_ms)
 * ----------------------------
 * Initialize one task slot with a function pointer, period, and clear elapsed time.
 */
static void task_set(int idx , void(*fn)(void), uint16_t period_ms){
    task[idx].taskFunc = fn;
    task[idx].taskPeriod = period_ms;
    task[idx].elapsedTime = 0;
}

/*
 * request_idle_oneshot(fn, ms)
 * ----------------------------
 * Ask the system to run 'fn' once for 'ms' (blocking delay), then request entering IDLE.
 * The one-shot is executed in the ACTIVE scheduler context by run_idle_oneshot_if_pending().
 */
void request_idle_oneshot(void(*fn)(void), uint16_t ms){
    IDLE_fn = fn;
    IDLE_fn_ms = ms;
    IDLE_fn_flag = true;
}

/*
 * run_idle_oneshot_if_pending()
 * -----------------------------
 * If a one-shot was requested and we are in ACTIVE mode, pause the scheduler,
 * run the one-shot function for the given duration, then request entering IDLE.
 * Returns true if a one-shot was run (caller can 'continue' to next loop).
 */
bool run_idle_oneshot_if_pending(){
    if(!IDLE_fn_flag || aperiodicMode)return false;
        IDLE_fn_flag = false;
        
    // Pause periodic tasks while we do the one-shot (keeps it deterministic)
    bool saved_pause = g_pause_scheduler;
    g_pause_scheduler = true;    
    
    uart2_flush();
    
    void(*f)(void) = IDLE_fn;
    uint16_t f_time = IDLE_fn_ms;
    if(f){
        f();
        delay_ms(f_time);
        f();
    }
    g_pause_scheduler = saved_pause;
    
    req_enter_idle = true;
    
    return true;
}

/*
 * cli_exec(line)
 * --------------
 * Parse and execute a CLI command line. Recognizes:
 *   '1','2','s','p','w' single-character shortcuts (enqueued to sporadic queue).
 *   add <led0|led1> <ms>  -> create/update a periodic LED task
 *   rm <idx>              -> remove a task and turn its LED off if applicable
 *   set <idx> <ms>        -> change a task's period
 * Lines are executed in foreground (not ISR).
 */
static void cli_exec(const char *line){
    if (line[0] && line[1] == '\0') {
        switch (line[0]) {
            case '1': (void)sporadic_enqueue(SPOR_TOGGLE_LED0); return;
            case '2': (void)sporadic_enqueue(SPOR_TOGGLE_LED1); return;
            case 's': case 'S': (void)sporadic_enqueue(SPOR_SLEEP); return;
            case 'p': case 'P': (void)sporadic_enqueue(SPOR_PRINT_POW);return;
            case 'w': case 'W': (void)sporadic_enqueue(SPOR_EXIT_IDLE); return;
            default: break;
        }
    }    
    
    
    char buf[CLI_Q_BUF];
    strncpy(buf,line,sizeof(buf));
    buf[sizeof(buf)-1] = '\0';
    
    char *token = strtok(buf," ");
    
    if(!token)return;

        if(char_eq(token,"add")){
        char *which_task = strtok(NULL," ");
        char *ms_str = strtok(NULL, " ");
        if(!which_task || !ms_str){
            uart2_puts("ERR! Add Usage and Time Period\r\n");
            return;
        }
        unsigned long ms = strtoul(ms_str,NULL,10);
        if(ms < 1 || ms > 65535){
            uart2_puts("ERR ms out of range\r\n");
            return;
        }
        void(*fn)(void) = NULL;
        if(char_eq(which_task,"led0")) fn = toggle_LED1;
        else if(char_eq(which_task,"led1")) fn = toggle_LED2;
        else{uart2_puts("ERR: Task not found\r\n");}
        
        // If the same task already exists, just update its period
        for (int i = 0; i < MAX_TASK; i++) {
            if (task[i].taskFunc == fn) {
                task[i].taskPeriod  = (uint16_t)ms;
                task[i].elapsedTime = 0;
                uart2_puts("OK: updated\r\n");
            
            
            if (aperiodicMode) {
                request_idle_oneshot(fn, (uint16_t)ms);      // run once in ACTIVE
                (void)sporadic_enqueue(SPOR_EXIT_IDLE);      // leave IDLE now
            }
                return;
            }
        }
        
        int slot = find_free_slot();
        if(slot < 0){uart2_puts("ERR: Slot not available\r\n"); return;}
        
        task_set(slot, fn, (uint16_t)ms);
        
        char ok[40]; snprintf(ok,sizeof(ok),"OK: added #%d\r\n",slot);
        uart2_puts(ok);
        return;   
    }
    else if(char_eq(token,"rm")){
        char *idx = strtok(NULL, " ");
        if (!idx) { uart2_puts("ERR: rm <idx>\r\n"); return; }
        int i = (int)strtol(idx, NULL, 10);
        if (i<0 || i>=MAX_TASK){ uart2_puts("ERR: idx\r\n"); return; }
        bool saved_pause = g_pause_scheduler;
        g_pause_scheduler = true;
        
        void(*fn)(void) = task[i].taskFunc;
        
        task[i].taskFunc = NULL; 
        task[i].taskPeriod = 0; 
        task[i].elapsedTime = 0;
        
        if(fn == toggle_LED1){LED_LATA0 = 0;}
        if(fn == toggle_LED2){LED_LATA1 = 0;}
        
        g_pause_scheduler = saved_pause;
        
        uart2_puts("OK\r\n");
        
        
        return;
    }
    else if(char_eq(token, "set")){
        char *idx = strtok(NULL, " ");
        char *ms_str = strtok(NULL, " ");
        if (!idx || !ms_str) { uart2_puts("ERR: set <idx> <ms>\r\n"); return; }
        int i = (int)strtol(idx, NULL, 10);
        unsigned long ms = strtoul(ms_str, NULL, 10);
        if (i<0 || i>=MAX_TASK || !task[i].taskFunc){ uart2_puts("ERR: idx\r\n"); return; }
        if (ms == 0 || ms > 65535UL) { uart2_puts("ERR: ms 1..65535\r\n"); return; }
        task[i].taskPeriod = (uint16_t)ms;
        task[i].elapsedTime = 0;
        uart2_puts("OK\r\n");
        return;
    }
}

/*
 * cli_service()
 * -------------
 * If a complete line is available from the RX ISR, copy it out of the shared
 * buffer (minimizing time with shared state), clear the buffer, then execute
 * the command via cli_exec().
 */
void cli_service(void){
    if(!cli_ready)return;
    
    IEC1bits.U2RXIE = 0;      // guard the copy
    char line[CLI_Q_BUF];
    
    uint8_t n = cli_len;
    
    if(n >= CLI_Q_BUF) n = CLI_Q_BUF - 1;
    
    memcpy(line,(const void*)cli_line, n);
    
    line[n] = '\0';
    
    cli_clear();
    IEC1bits.U2RXIE = 1; 
    cli_exec(line);
    
}

/*
 * schedulerRun()
 * --------------
 * Cooperative scheduler dispatcher. For each active task, increment elapsed
 * time every tick and call the task when its period has elapsed.
 * Returns immediately if g_pause_scheduler is set (used by quiet windows).
 */
void schedulerRun(void){   
    if (g_pause_scheduler) return;
    for(int i=0; i<MAX_TASK; i++){
        if(!task[i].taskFunc)continue;
        task[i].elapsedTime++;
    
        if(task[i].elapsedTime >= task[i].taskPeriod){
            task[i].elapsedTime = 0;    //Set elapsed time to 0
            task[i].taskFunc();
        }
    }
}




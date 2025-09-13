/*
 * File:   main.c
 * Author: Dkapoor
 *
 * Created on 20 July 2025, 11:23
 */


#include "xc.h"
#include <stdint.h>
#include <stdbool.h>
#include <libpic30.h>
#include "scheduler.h"
#include "button.h"
#include "ina219.h"
#include "trace.h"

#pragma config FNOSC = FRC
#pragma config FWDTEN = OFF     //Watchdog Timer
#pragma config JTAGEN = OFF     //JTAG Debugger OFF
#pragma config FWPSA  = PR128    // base tick ? 4 ms
#pragma config WDTPS  = PS512    // 4 ms * 512 ? ~2.0 s wake

volatile bool tickFlag = false;
volatile bool aperiodicMode = false;
volatile bool sporadicFlag = false;

int main(void) {
    initSystem();
    btn_init();
    setTickHz(1000, 0);
   
    uart2INIT();
    cli_init();
    __builtin_enable_interrupts();
    trace_init();
    uart2_puts("BOOT SUCCESSFULL\r\n");
    uart2_flush();   // wait until queued + shifter empty     
    i2c1_init_100k();  // NEW: I2C1
    ina219_begin_custom(INA219_ADDR_DEFAULT, 100 , 50);
    set_peripherals_active_profile();   

    while (1)
    {
        // --- LOW-POWER / IDLE path (requested via S1) ---
        if (req_enter_idle)
        {
            // Configure aperiodic profile (Timer1 runs in IDLE, LEDs, SIDLs, etc.)
            enter_aperiodicTask();

            // Stay in IDLE until S2 (or other logic) clears the latch
            while (req_enter_idle)
            {
                static bool a2i_close_pending = true;     // re-armed each time we enter this while
                if (a2i_close_pending) { _LATB8 = 0; a2i_close_pending = false; }  // end A?I window
                // Park the core in IDLE (peripherals keep running; T1 still ticks)
                __builtin_pwrsav(1);

                cli_service();
                
                // We just woke up due to some interrupt.
                // If any ISR requested sporadic work, do it *now* and go back to IDLE.
                
                if (sporadicFlag)
                {
                    sporadicTask();
                    sporadicFlag = false;

                }

                if (woke_from_s2) {
                    static bool i2a = true;
                    if(i2a){_LATB9 = 0;i2a = false;}
                    out1_pulse();
                    req_enter_idle = false;
                    woke_from_s2 = false;
                }

                // Loop will re-check req_enter_idle; if still true we re-enter IDLE.
            }

            // Latch is cleared (e.g., S2 pressed) -> restore ACTIVE profile once
            exit_aperiodicTask();
        }
        
        cli_service();
        
        // --- NORMAL ACTIVE scheduler path ---
        if (tickFlag)
        {
            out1_pulse();
            tickFlag = false;
            if(run_idle_oneshot_if_pending()){
                continue;
            }
            if (sporadicFlag) {
                out2_hi();
                sporadicTask();
                out2_lo();
                sporadicFlag = false;
            } else {
                out2_hi();
                schedulerRun();
                out2_lo();
                LED_RED = 0; LED_GREEN = 0; LED_BLUE = 0;
            }
        }
    }
}
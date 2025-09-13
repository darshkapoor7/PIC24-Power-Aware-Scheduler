/*
 * File:   trace.c
 * Author: Dkapoor
 *
 * Created on 15 August 2025, 14:40
 */


#include "xc.h"

void trace_init(void){
    ANSBbits.ANSB8 = 0; _TRISB8 = 0; _LATB8 = 0;   // RB8 digital out, low
    ANSBbits.ANSB9 = 0; _TRISB9 = 0; _LATB9 = 0;   // RB9 digital out, low
}

void out1_hi(void){  _LATB8 = 1; }
void out1_lo(void){  _LATB8 = 0; }
void out1_pulse(void){ _LATB8 = 1; _LATB8 = 0; }

void out2_hi(void){  _LATB9 = 1; }
void out2_lo(void){  _LATB9 = 0; }
void out2_pulse(void){ _LATB9 = 1; _LATB9 = 0; }

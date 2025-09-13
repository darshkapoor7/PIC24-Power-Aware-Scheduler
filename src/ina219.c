/*
 * File:   ina219.c
 * Author: Dkapoor
 *
 * Created on 13 August 2025, 14:19
 */


#include "xc.h"
#include "ina219.h"

// ---- Register map ----
#define REG_CONFIG     0x00
#define REG_SHUNT_V    0x01
#define REG_BUS_V      0x02
#define REG_POWER      0x03
#define REG_CURRENT    0x04
#define REG_CALIB      0x05

// ---- Config bits (safe defaults) ----
// We'll rely mostly on device defaults after reset: 32V range, 320mV shunt range,
// 12-bit conversion, shunt+bus continuous. We'll still issue a soft reset.
#define CONFIG_RESET   0x8000
#define CONFIG_DEFAULT 0x019F  // (optional) typical: 32V, 320mV, 12-bit, continuous

// Internal state
static uint8_t  s_addr = INA219_ADDR_DEFAULT;
static uint16_t s_cal  = 0;
static uint16_t s_rshunt_milliohms = 100;     // default 0.10?
static uint16_t s_current_lsb_uA    = 100;    // 100 µA/bit  => power LSB = 2 mW/bit

static inline void i2c1_wait_idle(void){
    while (I2C1CON1bits.SEN  || I2C1CON1bits.PEN  || I2C1CON1bits.RSEN ||
           I2C1CON1bits.RCEN || I2C1CON1bits.ACKEN || I2C1STATbits.TRSTAT) { ; }
}

void i2c1_init_100k(void){
    // FCY from your project is 4 MHz (scheduler.h)
    // BRG ~= (FCY / (2*Fsck)) - 1  -> (4e6 / 200000) - 1 = 19
    I2C1CON1bits.I2CEN = 0;
    I2C1BRG = 19;
    
    #ifdef _I2C1CONL_DISSLW_MASK
        I2C1CON1bits.DISSLW = 1;  // 100 kHz mode
    #endif
    
    I2C1CON1bits.I2CEN = 1;
}

static bool i2c1_start(void){
    i2c1_wait_idle();
    I2C1CON1bits.SEN = 1; while (I2C1CON1bits.SEN); return true;
}
static void i2c1_stop(void){
    i2c1_wait_idle();
    I2C1CON1bits.PEN = 1; while (I2C1CON1bits.PEN);
}
static bool i2c1_restart(void){
    i2c1_wait_idle();
    I2C1CON1bits.RSEN = 1; while (I2C1CON1bits.RSEN); return true;
}
static bool i2c1_write(uint8_t b){
    i2c1_wait_idle();
    I2C1TRN = b;
    while (I2C1STATbits.TRSTAT);
    return (I2C1STATbits.ACKSTAT == 0); // got ACK?
}
static uint8_t i2c1_read(bool ack){
    i2c1_wait_idle();
    I2C1CON1bits.RCEN = 1; while (!I2C1STATbits.RBF);
    uint8_t d = I2C1RCV;
    i2c1_wait_idle();
    I2C1CON1bits.ACKDT = (ack ? 0 : 1); // ACK=0, NACK=1
    I2C1CON1bits.ACKEN = 1; while (I2C1CON1bits.ACKEN);
    return d;
}

// 16-bit big-endian register write
static bool ina219_w16(uint8_t reg, uint16_t valBE){
    if (!i2c1_start()) return false;
    if (!i2c1_write((s_addr << 1) | 0)) { i2c1_stop(); return false; }
    if (!i2c1_write(reg))               { i2c1_stop(); return false; }
    if (!i2c1_write((uint8_t)(valBE >> 8))) { i2c1_stop(); return false; }
    if (!i2c1_write((uint8_t)(valBE & 0xFF))) { i2c1_stop(); return false; }
    i2c1_stop();
    return true;
}

// 16-bit big-endian register read
static bool ina219_r16(uint8_t reg, uint16_t *outBE){
    if (!i2c1_start()) return false;
    if (!i2c1_write((s_addr << 1) | 0)) { i2c1_stop(); return false; }
    if (!i2c1_write(reg))               { i2c1_stop(); return false; }
    if (!i2c1_restart())                { i2c1_stop(); return false; }
    if (!i2c1_write((s_addr << 1) | 1)) { i2c1_stop(); return false; }
    uint8_t hi = i2c1_read(true);
    uint8_t lo = i2c1_read(false);
    i2c1_stop();
    *outBE = ((uint16_t)hi << 8) | lo;
    return true;
}

static bool ina219_reset(void){
    return ina219_w16(REG_CONFIG, CONFIG_RESET);
}

static uint16_t compute_calibration(uint16_t rshunt_milliohms,
                                    uint16_t current_lsb_uA){
    // From datasheet: CAL = 0.04096 / (Current_LSB[A] * Rshunt[?])
    // Work in integers:
    // CAL = (40960) / (current_lsb_uA * rshunt_ohms_milliohms / 1000)
    //     = (40960 * 1000) / (current_lsb_uA * rshunt_milliohms)
    // Clamp to 15 bits
    uint32_t num   = 40960UL * 1000UL; // 40.960 / 1e-3 to keep precision
    uint32_t denom = (uint32_t)current_lsb_uA * (uint32_t)rshunt_milliohms;
    if (denom == 0) denom = 1;
    uint32_t cal = num / denom;
    if (cal > 0xFFFEu) cal = 0xFFFEu;
    return (uint16_t)cal;
}

bool ina219_begin_custom(uint8_t addr,
                         uint16_t rshunt_milliohms,
                         uint16_t current_lsb_uA)
{
    s_addr = addr;
    s_rshunt_milliohms = rshunt_milliohms;
    s_current_lsb_uA   = current_lsb_uA;

    // Reset, (optionally) write a config, then calibration
    if (!ina219_reset()) return false;

    // Optional explicit config (defaults are already good on most parts)
    (void)ina219_w16(REG_CONFIG, CONFIG_DEFAULT);

    s_cal = compute_calibration(s_rshunt_milliohms, s_current_lsb_uA);
    if (!ina219_w16(REG_CALIB, s_cal)) return false;

    return true;
}

bool ina219_begin(uint8_t addr){
    // default for 0.10? shunt and 0.1 mA LSB (100 µA/bit)
    return ina219_begin_custom(addr, 100 /*m?*/, 100 /*µA*/);
}

bool ina219_read_bus_mv(uint16_t *bus_mv){
    uint16_t be;
    if (!ina219_r16(REG_BUS_V, &be)) return false;
    // Bits 15..13 = CNVR/OVF/0 ; 12..3 = Bus voltage, LSB = 4 mV
    uint16_t raw = (be >> 3) & 0x0FFFu; // 12-bit
    *bus_mv = (uint16_t)(raw * 4u);
    return true;
}

bool ina219_wait_ready(uint16_t timeout_ms)
{
    uint16_t be;
    while (timeout_ms--) {
        if (!ina219_r16(REG_BUS_V, &be)) return false;
        if (be & 0x0002) {            // CNVR bit set -> fresh data available
            return true;              // (reading REG_BUS_V just now cleared CNVR)
        }
        delay_ms(1);
    }
    return false;                     // timed out; use last sample if you want
}

bool ina219_read_current_mA(int32_t *mA){
    uint16_t be;
    if (!ina219_r16(REG_CURRENT, &be)) return false;
    int16_t raw = (int16_t)be; // signed
    // current[mA] = raw * currentLSB[µA] / 1000
    int32_t mA_local = ((int32_t)raw * (int32_t)s_current_lsb_uA) / 1000;
    *mA = mA_local;
    return true;
}

bool ina219_read_power_mW(uint32_t *mW){
    uint16_t be;
    if (!ina219_r16(REG_POWER, &be)) return false;
    uint16_t raw = be;
    // Power LSB = 20 * current_LSB
    // If current_LSB = 100 µA -> power LSB = 2 mW
    uint32_t power_lsb_mW = 20u * (uint32_t)s_current_lsb_uA / 1000u; // mW/bit
    if (power_lsb_mW == 0) power_lsb_mW = 1; // avoid zero if tiny LSB chosen
    *mW = (uint32_t)raw * power_lsb_mW;
    return true;
}

bool ina219_read_all(uint16_t *bus_mv, int32_t *mA, uint32_t *mW){
    if(!ina219_wait_ready(10))return false;
    bool ok1 = ina219_read_bus_mv(bus_mv);
    bool ok2 = ina219_read_current_mA(mA);
    bool ok3 = ina219_read_power_mW(mW);
    return ok1 && ok2 && ok3;
}
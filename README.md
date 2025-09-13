# Power-Aware Task Scheduler on PIC24FJ512GU410

A lightweight **bare-metal scheduler** with runtime task control and **power-aware state management** for the PIC24FJ512GU410 (DM240018 board).  

The project shows how to achieve **real-time behavior and measurable power savings** on a low-cost microcontroller **without an RTOS or DVFS**, by combining a cooperative scheduler, UART CLI, and the MCU’s built-in low-power modes.

---
## ✨ Key Features
- **Cooperative Scheduler**
  - 1 kHz system tick in **ACTIVE**, throttled to 1 Hz in **IDLE**
  - Supports **periodic**, **sporadic**, and **aperiodic** tasks
  - Compact task table (8 entries: callback | period | elapsed | enabled)

- **UART2 Command-Line Interface**
  - Add, remove, or retime tasks at runtime
  - Commands:
    - `add <task> <ms>` → install a new periodic task
    - `rm <slot>` → remove a task
    - `set <slot> <ms>` → update a task’s period
  - Single-character shortcuts:
    - `p` → print power metrics
    - `s` → enter timed sleep + measure
    - `w` → wake from IDLE

- **Low-Power Modes**
  - **ACTIVE**: 1 kHz tick, all peripherals available
  - **IDLE**: 1 Hz tick, unused modules gated with PMD/SIDL
  - **SLEEP**: timed sleep (Timer1/WDT wake) with quiet measurement window
  - **Race-to-halt policy**: finish tasks quickly, then drop into IDLE/SLEEP

- **Peripherals & Drivers**
  - **UART2**: interrupt-driven RX, TX ring buffer (non-blocking)
  - **Buttons (S1/S2)**: request IDLE, exit IDLE/SLEEP
  - **INA219 sensor (I²C1)**: voltage/current/power reporting
  - **Trace pins & LEDs**: verify timing and latency on a scope

- **Measurement Hooks**
  - GPIO pulses to trace execution and scheduler ticks
  - Verified with oscilloscope (500 ms / 333 ms tasks, 2 s sleep gaps)
  - Power logs over UART:
    - ACTIVE ≈ 13 mW
    - IDLE ≈ 9–11 mW
    - SLEEP ≈ 6–10 mW
      
---
## 🧭 System Architecture

The runtime combines four main blocks:
1. **Timer1 ISR** → raises tick flags (1 kHz / 1 Hz)  
2. **Cooperative scheduler** → scans the task table and dispatches callbacks  
3. **UART CLI** → live task management + power commands  
4. **Power-Mode Controller** → manages ACTIVE/IDLE/SLEEP with PMD/SIDL  

---
## 📂 Repository Layout
```
├── src/                                           # all source code (.c/.h)
│ ├── main.c
│ ├── scheduler.c
│ ├── scheduler.h
│ ├── ina219.c
│ ├── ina219.h
│ ├── button.c
│ ├── button.h
│ ├── trace.c
│ └── trace.h
├── docs/                                          # documents + diagrams + results
│ ├── PowerAwareScheduler_PIC24_Dissertation.pdf
│ ├── System_Architecture.pdf
│ └── results/
│       ├── LED0_500_Scope.jpeg
│       ├── LED1_333_Scope.jpeg
│       ├── Power_Latency_Scope.jpeg
│       ├── Power_UART_CLI.png
│       ├── Scheduler_busy_Scope.jpeg
│       └── Sleep_2s_gap.jpeg
├── README.md
└── .gitignore
```
---
## 🛠️ Build & Setup

- **Toolchain**: [Microchip XC16](https://www.microchip.com/en-us/tools-resources/develop/mplab-xc-compilers) (tested with v2.x)  
- **Board**: PIC24FJ512GU410 on [DM240018](https://www.microchip.com/developmenttools/ProductDetails/DM240018)  
- **Connections**:
  - UART2 ↔ USB-UART bridge (U2RX = RP43/RD14, U2TX = RP5)
  - I²C1 ↔ INA219 sensor (addr `0x40`)
  - Buttons: S1 (RA4 = enter IDLE), S2 (RB4 = wake)
  - RA0/RA1: LED/trace pins for timing checks
- **Build**: open MPLAB X project, compile with XC16, program via ICD/PKOB  
- **Console**: 115200-8N1 over USB-UART  

---
## 🎛️ Example Session
```
> add led0 500
ok: added #0 (500 ms blink)

> set 0 1000
ok: updated #0 to 1000 ms

> p
ACTIVE: V=3.296 V, I=4 mA, P=13 mW

> s
...sleeping 2000 ms...
AWAKE: V=3.296 V, I=2 mA, P=6 mW

> w
ok: exited IDLE
```
---
## 📊 Validation Results

Timing accuracy
500 ms and 333 ms LED tasks within ±1 ms of target

Latency
Sporadic p command = ~35–40 ms foreground work

Power savings
IDLE saves ~25–30% (13 → 9 mW)
SLEEP saves ~50% (13 → 6 mW)

---
## 🚧 Limitations
Heavy UART prints block the foreground (~40 ms)

IDLE tick = 1 Hz (no sub-second tasks in IDLE)

No DVFS: energy efficiency relies on DPM only

INA219 measurements are trend-accurate, not lab-grade metrology

---
## 🔮 Future Work
Preemptive scheduling (RR, RMS, EDF) with context-switch benchmarking

Absolute next-release scheduling (reduce phase drift)

Tickless ACTIVE mode + auto-sleep heuristics

More accurate power measurement (quiet sampling, averaging)

Extended CLI: priorities, deadlines, preemption toggle


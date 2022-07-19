// Clough42 Electronic Leadscrew
// https://github.com/clough42/electronic-leadscrew
//
// MIT License
//
// Copyright (c) 2019 James Clough
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.


#include "F28x_Project.h"
#include "Configuration.h"
#include "SanityCheck.h"
#include "ControlPanel.h"
#include "EEPROM.h"
//#include "StepperDrive.h"
//#include "Encoder.h"

#include "Core.h"
#include "UserInterface.h"
#include "Debug.h"
#include "cla_shared.h"

#define SYS_FREQ            100000000UL
#define EPWM_CLKDIV         32UL
#define EPWM1_FREQ          200000UL
#define EPWM1_PERIOD        (uint16_t)(SYS_FREQ/(EPWM_CLKDIV*EPWM1_FREQ))

#pragma DATA_SECTION("Cla1ToCpuMsgRAM");
uint16_t rpm_out;
#pragma DATA_SECTION("CpuToCla1MsgRAM");
float feed = 0;
#pragma DATA_SECTION("CpuToCla1MsgRAM");
int16 feedDirection = 0;
#pragma DATA_SECTION("Cla1ToCpuMsgRAM");
bool alarm = 0;
#pragma DATA_SECTION("Cla1DataRam");
bool powerOn = 0;
// Fetch Access Violation Address
volatile uint32_t fetchAddress;


// Linker Defined variables
extern uint32_t Cla1ProgRunStart, Cla1ProgLoadStart, Cla1ProgLoadSize;
extern uint32_t Cla1ConstRunStart, Cla1ConstLoadStart, Cla1ConstLoadSize;

//*****************************************************************************
// Function Prototypes
//*****************************************************************************
//void initADC( void );
//void initEPWM( void );
void initCLA( void );
void configCLAMemory(void);
void initEPWM( void );
void initClaPeripherals(void);
//__attribute__((interrupt))  void cla1Isr1( void );
__attribute__((interrupt))  void accessViolationISR(void);


__interrupt void cpu_timer0_isr(void);


//
// DEPENDENCY INJECTION
//
// Declare all of the main components and wire them together
//

// Debug harness
Debug debug;

// Feed table factory
FeedTableFactory feedTableFactory;

// Common SPI Bus driver
SPIBus spiBus;

// Control Panel driver
ControlPanel controlPanel(&spiBus);

// EEPROM driver
EEPROM eeprom(&spiBus);

/*// Encoder driver
Encoder encoder;

// Stepper driver
StepperDrive stepperDrive;*/

// Core engine
Core core;

// User interface
UserInterface userInterface(&controlPanel, &core, &feedTableFactory);

void main(void)
{
#ifdef _FLASH
    // Copy time critical code and Flash setup code to RAM
    // The RamfuncsLoadStart, RamfuncsLoadEnd, and RamfuncsRunStart
    // symbols are created by the linker. Refer to the linker files.
    memcpy(&RamfuncsRunStart, &RamfuncsLoadStart, (size_t)&RamfuncsLoadSize);

    // Initialize the flash instruction fetch pipeline
    // This configures the MCU to pre-fetch instructions from flash.
    InitFlash();
#endif

    // Initialize System Control:
    // PLL, WatchDog, enable Peripheral Clocks
    InitSysCtrl();

    // Disable CPU interrupts
    DINT;

    // Initialize the PIE control registers to their default state.
    InitPieCtrl();

    // Disable CPU interrupts and clear all CPU interrupt flags
    IER = 0x0000;
    IFR = 0x0000;

    // Initialize the PIE vector table with pointers to shell Interrupt
    // Service Routines (ISR) to help with debugging.
    InitPieVectTable();

    configCLAMemory();
    initCLA();

    EALLOW;
    CpuSysRegs.PCLKCR0.bit.TBCLKSYNC = 0;
    EDIS;
    initEPWM();
    EALLOW;
    CpuSysRegs.PCLKCR0.bit.TBCLKSYNC = 1;
    EDIS;

    initClaPeripherals();

    userInterface.init();

    // Set up the CPU0 timer ISR
    EALLOW;
    PieVectTable.TIMER0_INT = &cpu_timer0_isr;
    EDIS;

    // initialize the CPU timer
    InitCpuTimers();   // For this example, only initialize the Cpu Timers
    ConfigCpuTimer(&CpuTimer0, CPU_CLOCK_MHZ, STEPPER_CYCLE_US);

    // Use write-only instruction to set TSS bit = 0
    CpuTimer0Regs.TCR.all = 0x4001;

    // Initialize peripherals and pins
    debug.initHardware();
    spiBus.initHardware();
    controlPanel.initHardware();
    eeprom.initHardware();
    //stepperDrive.initHardware();
    //encoder.initHardware();

    // Enable CPU INT1 which is connected to CPU-Timer 0
    IER |= M_INT1;

    // Enable TINT0 in the PIE: Group 1 interrupt 7
    PieCtrlRegs.PIEIER1.bit.INTx7 = 1;

    // CLA Task 1 set to trigger an end-of-task interrupt in the PIE
    //        so we create an end-of-task1 interrupt service routine and we
    //        assign it to the PieVectTable (see TRM)
    //   - Enable the PIE group and subgroup for that particular interrupt
    //PieVectTable.CLA1_1_INT          = &cla1Isr1;
    //PieCtrlRegs.PIEIER11.bit.INTx1   = 0x01;
    //IER |= (M_INT11);

    // Enable global Interrupts and higher priority real-time debug events
    EINT;
    ERTM;

    // User interface loop
    for(;;) {
        // mark beginning of loop for debugging
        debug.begin2();

        // check for step backlog and panic the system if it occurs
        // TODO: need to reaccomplish this, make backlog accessible from CLA?
        /*if( stepperDrive.checkStepBacklog() ) {
            userInterface.panicStepBacklog();
        }*/

        // service the user interface
        userInterface.loop();

        // mark end of loop for debugging
        debug.end2();

        // delay
        DELAY_US(1000000 / UI_REFRESH_RATE_HZ);
    }
}


// CPU Timer 0 ISR
__interrupt void
cpu_timer0_isr(void)
{
    CpuTimer0.InterruptCount++;

    // flag entrance to ISR for timing
    debug.begin1();

    // service the Core engine ISR, which in turn services the StepperDrive ISR
    //core.ISR();

    // flag exit from ISR for timing
    debug.end1();

    //
    // Acknowledge this interrupt to receive more interrupts from group 1
    //
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;
}


void initCLA( void ){
    EALLOW;
    Cla1Regs.MVECT1 = (uint16_t)(&Cla1Task1);
//  Cla1Regs.MVECT2 = (uint16_t)(&Cla1Task2);
//  Cla1Regs.MVECT3 = (uint16_t)(&Cla1Task3);
//  Cla1Regs.MVECT4 = (uint16_t)(&Cla1Task4);
//  Cla1Regs.MVECT5 = (uint16_t)(&Cla1Task5);
//  Cla1Regs.MVECT6 = (uint16_t)(&Cla1Task6);
//  Cla1Regs.MVECT7 = (uint16_t)(&Cla1Task7);
    Cla1Regs.MVECT8 = (uint16_t)(&Cla1Task8);

    DmaClaSrcSelRegs.CLA1TASKSRCSEL1.bit.TASK1  = CLA_TRIG_EPWM1INT;
    DmaClaSrcSelRegs.CLA1TASKSRCSEL2.bit.TASK8  = CLA_TRIG_NOPERPH;
    Cla1Regs.MIER.all                           = (M_INT1 | M_INT8);


    // Enable software forcing and run task 8 once
    //    - The MCTL register allows for a task to be triggered by software
    //    - Run any one-time tasks by using the software force. "F28004x_Cla_defines.h"
    //      has some useful macro functions to force each task
    Cla1Regs.MCTL.bit.IACKE  = 1;
    EDIS;
    Cla1ForceTask8andWait();
}

//! \brief Configure the RAMs that will be the program and data spaces
//! for the CLA
//! This function will
//! -# copy over code and const from flash to CLA program and data ram respectively
//! -# map program and data spaces to the CLA
//
void configCLAMemory(void)
{
    // Copy over the "Cla1Prog" and ".const_cla" sections from Flash to RAM
    memcpy((uint32_t *)&Cla1ProgRunStart, (uint32_t *)&Cla1ProgLoadStart,
        (uint32_t)&Cla1ProgLoadSize );
    memcpy((uint32_t *)&Cla1ConstRunStart, (uint32_t *)&Cla1ConstLoadStart,
        (uint32_t)&Cla1ConstLoadSize );
    EALLOW;

    // Initialize and wait for CLA1ToCPUMsgRAM
    MemCfgRegs.MSGxINIT.bit.INIT_CLA1TOCPU = 1;
    while(MemCfgRegs.MSGxINITDONE.bit.INITDONE_CLA1TOCPU != 1){};

    // Initialize and wait for CPUToCLA1MsgRAM
    MemCfgRegs.MSGxINIT.bit.INIT_CPUTOCLA1 = 1;
    while(MemCfgRegs.MSGxINITDONE.bit.INITDONE_CPUTOCLA1 != 1){};

    // Give CLA control over program and data RAM(s)
    //    - The MemCfgRegs register is described in TRM 2.14.17
    // Configure LS4RAM as program space for the CLA
    // First configure the CLA to be the master for LS4 and then
    // set the spaces to be program blocks
    MemCfgRegs.LSxMSEL.bit.MSEL_LS4 = 1;
    MemCfgRegs.LSxCLAPGM.bit.CLAPGM_LS4 = 1;

    // Configure RAMLS7 as data space for the CLA
    // First configure the CLA to be the master for each block and then
    // set the spaces to be data blocks
    MemCfgRegs.LSxMSEL.bit.MSEL_LS7 = 1;
    MemCfgRegs.LSxCLAPGM.bit.CLAPGM_LS7 = 0;

    /*MemCfgRegs.LSxMSEL.bit.MSEL_LS3 = 1;
    MemCfgRegs.LSxCLAPGM.bit.CLAPGM_LS3 = 0;

    MemCfgRegs.LSxMSEL.bit.MSEL_LS4 = 1;
    MemCfgRegs.LSxCLAPGM.bit.CLAPGM_LS4 = 0;

    MemCfgRegs.LSxMSEL.bit.MSEL_LS5 = 1;
    MemCfgRegs.LSxCLAPGM.bit.CLAPGM_LS5 = 0;*/

    // Detect any CLA fetch access violations, enable
    // interrupt for it (TRM SPRUHM8, 2.11.1.7.4 & 2.14.18)
    AccessProtectionRegs.NMAVSET.bit.CLA1FETCH = 1;
    AccessProtectionRegs.NMAVINTEN.bit.CLA1FETCH = 1;
    // Set the ISR for access violation fault
    PieVectTable.RAM_ACCESS_VIOLATION_INT = accessViolationISR;
    PieCtrlRegs.PIEIER12.bit.INTx12       = 1;
    IER                                  |= M_INT12;

    EDIS;
}

void initEPWM( void ){
    EALLOW;
    EPwm1Regs.TBCTL.bit.CLKDIV  = 4;  // CLKDIV = /16 HSPCLKDIV =/2 (default)
                                      // effective clock rate = SYSCLK/32
    EPwm1Regs.TBPRD             = EPWM1_PERIOD;
    EPwm1Regs.TBCTL.bit.CTRMODE = TB_COUNT_UP;
    EPwm1Regs.ETSEL.bit.INTSEL  = ET_CTR_ZERO; // Trigger INT on at Zero event
    EPwm1Regs.ETPS.bit.INTPRD = ET_1ST; // Generate INT on N'th event
    //EPwm1Regs.ETSEL.bit.SOCAEN  = 0; // disable ADC Start of Conversion A pulse
    //EPwm1Regs.ETSEL.bit.SOCASEL = ET_CTR_ZERO;
    //EPwm1Regs.ETPS.bit.SOCAPRD  = ET_DISABLE;
    EPwm1Regs.AQCTLA.bit.ZRO = AQ_TOGGLE;
    EPwm1Regs.TBCTL.bit.FREE_SOFT = 3;
    EPwm1Regs.ETSEL.bit.INTEN = 1;     // Enable Epwm1 interrupt (Trigger CLA Task 1)
    EDIS;

    /* Note: can't do the following, because EPWM1_A is only available at GPIO0 which is already in use
    // For testing - monitor the EPWM1A output
    // which toggles once every ePWM period (i.e
    // at the start of conversion)
    EALLOW;
    GpioCtrlRegs.GPAMUX1.bit.GPIO0 = 1;
    EDIS;
    EPwm1Regs.AQCTLA.bit.ZRO = AQ_TOGGLE;
    EPwm1Regs.TBCTL.bit.FREE_SOFT = 3;*/
}

void initClaPeripherals(void){
    EALLOW;
    GpioCtrlRegs.GPAMUX1.bit.GPIO0 = 0;
    GpioCtrlRegs.GPAMUX1.bit.GPIO1 = 0;
    GpioCtrlRegs.GPAMUX1.bit.GPIO6 = 0;
    GpioCtrlRegs.GPAMUX1.bit.GPIO7 = 0;

    GpioCtrlRegs.GPADIR.bit.GPIO0 = 1;
    GpioCtrlRegs.GPADIR.bit.GPIO1 = 1;
    GpioCtrlRegs.GPADIR.bit.GPIO6 = 1;
    GpioCtrlRegs.GPADIR.bit.GPIO7 = 0; // input

    // Turn over control of these GPIO to CLA
    GpioCtrlRegs.GPACSEL1.bit.GPIO0 = GPIO_MUX_CPU1CLA;
    GpioCtrlRegs.GPACSEL1.bit.GPIO1 = GPIO_MUX_CPU1CLA;
    GpioCtrlRegs.GPACSEL1.bit.GPIO6 = GPIO_MUX_CPU1CLA;
    GpioCtrlRegs.GPACSEL1.bit.GPIO7 = GPIO_MUX_CPU1CLA;

    // Initialize Encoder GPIO
#ifdef ENCODER_USE_EQEP1
    GpioCtrlRegs.GPBPUD.bit.GPIO35 = 0;     // Enable pull-up on GPIO35 (EQEP1A)
    GpioCtrlRegs.GPBPUD.bit.GPIO37 = 0;     // Enable pull-up on GPIO371 (EQEP1B)
    GpioCtrlRegs.GPBPUD.bit.GPIO59 = 0;     // Enable pull-up on GPIO59 (EQEP1I)

    GpioCtrlRegs.GPBQSEL1.bit.GPIO35 = 0;   // Sync to SYSCLKOUT GPIO35 (EQEP1A)
    GpioCtrlRegs.GPBQSEL1.bit.GPIO37 = 0;   // Sync to SYSCLKOUT GPIO37 (EQEP1B)
    GpioCtrlRegs.GPBQSEL2.bit.GPIO59 = 0;   // Sync to SYSCLKOUT GPIO59 (EQEP1I)

    GpioCtrlRegs.GPBMUX1.bit.GPIO35 = 1;    // Configure GPIO35 as EQEP1A
    GpioCtrlRegs.GPBGMUX1.bit.GPIO35 = 2;
    GpioCtrlRegs.GPBMUX1.bit.GPIO37 = 1;    // Configure GPIO37 as EQEP1B
    GpioCtrlRegs.GPBGMUX1.bit.GPIO37 = 2;
    GpioCtrlRegs.GPBMUX2.bit.GPIO59 = 3;    // Configure GPIO59 as EQEP1I
    GpioCtrlRegs.GPBGMUX2.bit.GPIO59 = 2;

    // Turn over control of these GPIO to CLA
    GpioCtrlRegs.GPBCSEL1.bit.GPIO35 = GPIO_MUX_CPU1CLA;
    GpioCtrlRegs.GPBCSEL1.bit.GPIO37 = GPIO_MUX_CPU1CLA;
    GpioCtrlRegs.GPBCSEL4.bit.GPIO59 = GPIO_MUX_CPU1CLA;
#endif
#ifdef ENCODER_USE_EQEP2
    GpioCtrlRegs.GPAPUD.bit.GPIO14 = 0;     // Enable pull-up on GPIO14 (EQEP2A)
    GpioCtrlRegs.GPAPUD.bit.GPIO15 = 0;     // Enable pull-up on GPIO15 (EQEP2B)
    GpioCtrlRegs.GPAPUD.bit.GPIO26 = 0;     // Enable pull-up on GPIO26 (EQEP2I)

    GpioCtrlRegs.GPAQSEL1.bit.GPIO14 = 0;   // Sync to SYSCLKOUT GPIO14 (EQEP2A)
    GpioCtrlRegs.GPAQSEL1.bit.GPIO15 = 0;   // Sync to SYSCLKOUT GPIO15 (EQEP2B)
    GpioCtrlRegs.GPAQSEL2.bit.GPIO26 = 0;   // Sync to SYSCLKOUT GPIO26 (EQEP2I)

    GpioCtrlRegs.GPAMUX1.bit.GPIO14 = 2;    // Configure GPIO14 as EQEP2A
    GpioCtrlRegs.GPAGMUX1.bit.GPIO14 = 2;
    GpioCtrlRegs.GPAMUX1.bit.GPIO15 = 2;    // Configure GPIO15 as EQEP2B
    GpioCtrlRegs.GPAGMUX1.bit.GPIO15 = 2;
    GpioCtrlRegs.GPAMUX2.bit.GPIO26 = 2;    // Configure GPIO26 as EQEP2I
    GpioCtrlRegs.GPAGMUX2.bit.GPIO26 = 0;

    // Turn over control of these GPIO to CLA
    GpioCtrlRegs.GPACSEL2.bit.GPIO14 = GPIO_MUX_CPU1CLA;
    GpioCtrlRegs.GPACSEL2.bit.GPIO15 = GPIO_MUX_CPU1CLA;
    GpioCtrlRegs.GPACSEL4.bit.GPIO26 = GPIO_MUX_CPU1CLA;
#endif

    EDIS;

    // Initialize encoder
    ENCODER_REGS.QDECCTL.bit.QSRC = 0;         // QEP quadrature count mode
    ENCODER_REGS.QDECCTL.bit.IGATE = 1;        // gate the index pin
    ENCODER_REGS.QDECCTL.bit.QAP = 1;          // invert A input
    ENCODER_REGS.QDECCTL.bit.QBP = 1;          // invert B input
    ENCODER_REGS.QDECCTL.bit.QIP = 1;          // invert index input
    ENCODER_REGS.QEPCTL.bit.FREE_SOFT = 2;     // unaffected by emulation suspend
    ENCODER_REGS.QEPCTL.bit.PCRM = 1;          // position count reset on maximum position
    ENCODER_REGS.QPOSMAX = _ENCODER_MAX_COUNT;  // Max position count

    ENCODER_REGS.QUPRD = CPU_CLOCK_HZ / RPM_CALC_RATE_HZ; // Unit Timer latch at RPM_CALC_RATE_HZ Hz
    ENCODER_REGS.QEPCTL.bit.UTE=1;             // Unit Timeout Enable
    ENCODER_REGS.QEPCTL.bit.QCLM=1;            // Latch on unit time out

    ENCODER_REGS.QEPCTL.bit.QPEN=1;            // QEP enable
}

#pragma CODE_SECTION(".TI.ramfunc")
__attribute__((interrupt))  void accessViolationISR(void)
{
    // Read the fetch address where the violation occurred
    fetchAddress = AccessProtectionRegs.NMCLA1FAVADDR;

    EALLOW;
    // clear the fault
    AccessProtectionRegs.NMAVCLR.bit.CLA1FETCH = 1;
    EDIS;
    __asm(" ESTOP0");
    // Acknowledge the RAM access violation interrupt
    PieCtrlRegs.PIEACK.all = M_INT12;
}

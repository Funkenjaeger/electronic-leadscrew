/*
 * CLA_init.cpp
 *
 *  Created on: Jul 19, 2022
 *      Author: evand
 */

#include "F28x_Project.h"
#include "Configuration.h"
#include "CLA_init.h"
#include "cla_shared.h"
#include "Encoder.h"

void initCLA( void ) {
    EALLOW;
    Cla1Regs.MVECT1 = (uint16_t)(&Cla1Task1);
    Cla1Regs.MVECT8 = (uint16_t)(&Cla1Task8);

    DmaClaSrcSelRegs.CLA1TASKSRCSEL1.bit.TASK1  = CLA_TRIG_EPWM1INT;
    DmaClaSrcSelRegs.CLA1TASKSRCSEL2.bit.TASK8  = CLA_TRIG_NOPERPH;
    Cla1Regs.MIER.all                           = (M_INT1 | M_INT8);


    // Enable software forcing and run task 8 once
    Cla1Regs.MCTL.bit.IACKE  = 1;
    EDIS;
    Cla1ForceTask8andWait();
}

//! \brief Configure the RAMs that will be the program and data spaces for the CLA
//! This function will
//! -# copy over code and const from flash to CLA program and data ram respectively
//! -# map program and data spaces to the CLA
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

    // Configure LS4RAM as program space for the CLA
    MemCfgRegs.LSxMSEL.bit.MSEL_LS4 = 1; // CLA master
    MemCfgRegs.LSxCLAPGM.bit.CLAPGM_LS4 = 1; // Program space

    // Configure RAMLS7 as data space for the CLA
    MemCfgRegs.LSxMSEL.bit.MSEL_LS7 = 1; // CLA master
    MemCfgRegs.LSxCLAPGM.bit.CLAPGM_LS7 = 0; // Data space

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
    EPwm1Regs.AQCTLA.bit.ZRO = AQ_TOGGLE;
    EPwm1Regs.TBCTL.bit.FREE_SOFT = 3;
    EPwm1Regs.ETSEL.bit.INTEN = 1;     // Enable Epwm1 interrupt (Trigger CLA Task 1)
    EDIS;
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
    EDIS;

    initEncoderHardware();

    EALLOW;
    // Turn over control of encoder GPIO to CLA
#ifdef ENCODER_USE_EQEP1
    GpioCtrlRegs.GPBCSEL1.bit.GPIO35 = GPIO_MUX_CPU1CLA;
    GpioCtrlRegs.GPBCSEL1.bit.GPIO37 = GPIO_MUX_CPU1CLA;
    GpioCtrlRegs.GPBCSEL4.bit.GPIO59 = GPIO_MUX_CPU1CLA;
#endif
#ifdef ENCODER_USE_EQEP2
    GpioCtrlRegs.GPACSEL2.bit.GPIO14 = GPIO_MUX_CPU1CLA;
    GpioCtrlRegs.GPACSEL2.bit.GPIO15 = GPIO_MUX_CPU1CLA;
    GpioCtrlRegs.GPACSEL4.bit.GPIO26 = GPIO_MUX_CPU1CLA;
#endif
    EDIS;
}


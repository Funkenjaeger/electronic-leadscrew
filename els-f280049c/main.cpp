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
#include "Encoder.h"
#include "Core.h"
#include "UserInterface.h"
#include "Debug.h"
#include "cla_shared.h"
#include "CLA_init.h"

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

//*****************************************************************************
// Function Prototypes
//*****************************************************************************
void initCLA( void );
void configCLAMemory(void);
void initEPWM( void );
void initClaPeripherals(void);

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

// Core engine
Core core;

// User interface
UserInterface userInterface(&controlPanel, &core, &feedTableFactory);

void main(void)
{
#ifdef _FLASH
    // Copy time critical code and Flash setup code to RAM
    memcpy(&RamfuncsRunStart, &RamfuncsLoadStart, (size_t)&RamfuncsLoadSize);

    // Initialize the flash instruction fetch pipeline
    InitFlash();
#endif

    // Initialize System Control: PLL, WatchDog, enable Peripheral Clocks
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

    // Initialize peripherals and pins
    debug.initHardware();
    spiBus.initHardware();
    controlPanel.initHardware();
    eeprom.initHardware();

    // Enable global Interrupts and higher priority real-time debug events
    EINT;
    ERTM;

    // User interface loop
    for(;;) {
        // check for step backlog and panic the system if it occurs
        // TODO: need to reaccomplish this, make backlog accessible from CLA?
        /*if( stepperDrive.checkStepBacklog() ) {
            userInterface.panicStepBacklog();
        }*/

        // service the user interface
        userInterface.loop();

        // delay
        DELAY_US(1000000 / UI_REFRESH_RATE_HZ);
    }
}

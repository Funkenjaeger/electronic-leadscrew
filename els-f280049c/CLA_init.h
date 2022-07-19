/*
 * CLA_init.h
 *
 *  Created on: Jul 19, 2022
 *      Author: evand
 */

#ifndef CLA_INIT_H_
#define CLA_INIT_H_

#define SYS_FREQ            100000000UL
#define EPWM_CLKDIV         32UL
#define EPWM1_FREQ          200000UL
#define EPWM1_PERIOD        (uint16_t)(SYS_FREQ/(EPWM_CLKDIV*EPWM1_FREQ))

void initCLA( void );
void configCLAMemory(void);
void initEPWM( void );
void initClaPeripherals(void);

// Linker Defined variables
extern uint32_t Cla1ProgRunStart, Cla1ProgLoadStart, Cla1ProgLoadSize;
extern uint32_t Cla1ConstRunStart, Cla1ConstLoadStart, Cla1ConstLoadSize;

#endif /* CLA_INIT_H_ */

#ifndef CLA_SHARED_H_
#define CLA_SHARED_H_
#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

//Task 1 (C) Variables
extern float feed;
extern bool powerOn;
extern bool alarm;
extern int16 feedDirection;
extern Uint16 rpm_out;

//Task 8 (C) Variables

//Common (C) Variables

//CLA C Tasks defined in Cla1Tasks_C.cla
__attribute__((interrupt))  void Cla1Task1();
__attribute__((interrupt))  void Cla1Task8();

#ifdef __cplusplus
}
#endif /* extern "C" */
#endif /*CLA_SHARED_H_*/

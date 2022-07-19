#ifndef CLA_SHARED_H_
#define CLA_SHARED_H_
#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

//Task 1 (C) Variables
#define FILTER_ORDER    99
#define FILTER_LENGTH   (FILTER_ORDER + 1)
extern volatile uint16_t runFFT;
extern const float coeffs[FILTER_LENGTH];
extern int16_t filter_in;
extern float filter_out;
extern float IOBuffer[];
extern float IOBuffer2[];

//Task 2 (C) Variables

//Task 3 (C) Variables

//Task 4 (C) Variables

//Task 5 (C) Variables

//Task 6 (C) Variables

//Task 7 (C) Variables

//Task 8 (C) Variables

//Common (C) Variables
extern float D[FILTER_LENGTH];

//CLA C Tasks defined in Cla1Tasks_C.cla
__attribute__((interrupt))  void Cla1Task1();
__attribute__((interrupt))  void Cla1Task2();
__attribute__((interrupt))  void Cla1Task3();
__attribute__((interrupt))  void Cla1Task4();
__attribute__((interrupt))  void Cla1Task5();
__attribute__((interrupt))  void Cla1Task6();
__attribute__((interrupt))  void Cla1Task7();
__attribute__((interrupt))  void Cla1Task8();

#ifdef __cplusplus
}
#endif /* extern "C" */
#endif /*CLA_SHARED_H_*/

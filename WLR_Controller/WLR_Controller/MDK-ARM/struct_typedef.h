#ifndef STRUCT_TYPEDEF_H
#define STRUCT_TYPEDEF_H


typedef signed char int8_t;
typedef signed short int int16_t;
typedef signed int int32_t;
typedef signed long long int64_t;

/* exact-width unsigned integer types */
typedef unsigned char uint8_t;
typedef unsigned short int uint16_t;
typedef unsigned int uint32_t;
typedef unsigned long long uint64_t;
typedef unsigned char bool_t;
typedef float fp32;
typedef double fp64;


int32_t float_to_uint(float x, float x_min, float x_max, int bits);
    
    
fp32 uint_to_float(int x_int, float x_min, float x_max, int bits);

float fmaxf(float x, float y);
 
float fminf(float x, float y);

void user_delay_us(uint16_t us);
#endif






/*
*  com_lib/com_lib.h
*/
#ifndef _COM_LIB_H_
#define _COM_LIB_H_

#include <Arduino.h>
#include <stdio.h>
#include <time.h>

namespace foxbot3{

typedef unsigned long long usec_t;
//typedef unsigned long  usec_t;

inline unsigned long micros_2(){
    //uint32_t m = millis();
    //unsigned long t = micros();
    return micros();
}

inline uint64_t micros_(){
    //uint32_t m = millis();
    //unsigned long t = micros();
    return ((uint64_t)millis()) * 1000 + ((uint64_t)micros())%1000;
}

}
#endif
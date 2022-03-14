/*
  Timer.cpp - April 2016
  Copyright (c) 2016.  All rights reserved.
  Auteur : Jérémy LONGER
  Company : Kuehne + Nagel
  Modifiation : April 2016
  Description : 
 */
#include "Timer.h"

Timer::Timer(unsigned long T) : m_G_Dt(T/1000), m_timer(0), m_timer_old(0), m_T(T)
{
  
}

void Timer::setDelay(unsigned long T) {
  m_T = T;
}

void Timer::start(unsigned long M) { //M = millis() for Arduino
  m_timer = M;
}

bool Timer::delay(unsigned long M, unsigned long T)
{
  m_T = T;
  if((M-m_timer) >= m_T)
  { 
    m_timer_old = m_timer;
    m_timer = M;
    if (m_timer > m_timer_old)
      m_G_Dt = (m_timer - m_timer_old)/1000.0;    // Real time of loop run. We use this on the DCM algorithm (gyro integration time) and on the PID (controller)
    else
      m_G_Dt = 0;
    return true;
  }
  else
    return false;
}

bool Timer::delay(unsigned long M)
{
  if((M - m_timer) >= m_T)
  { 
    m_timer_old = m_timer;
    m_timer = M;
    m_G_Dt = (m_timer - m_timer_old)/1000.0;    // Real time of loop run. We use this on the DCM algorithm (gyro integration time) and on the PID (controller)
    return true;
  }
  else
    return false;
}

float Timer::getG_Dt() const
{
  return m_G_Dt;
}


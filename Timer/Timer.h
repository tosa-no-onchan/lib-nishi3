/*
  Timer.h - April 2016
  Copyright (c) 2016.  All rights reserved.
  Auteur : Jérémy LONGER
  Company : Kuehne + Nagel
  Modifiation : April 2016
  Description : 
 */
#ifndef DEF_TIMER
#define DEF_TIMER

class Timer
{
  public:
    Timer(unsigned long T);
    void setDelay(unsigned long T);
    bool delay(unsigned long M, unsigned long T);
    bool delay(unsigned long M);
    float getG_Dt() const;
    void start(unsigned long M);

  private:
    float m_G_Dt;
    unsigned long m_timer;
    unsigned long m_timer_old;
    unsigned long m_T;
    // float m_G_Dt; /* move upper line by nishi */
  
};

#endif

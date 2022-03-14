#ifndef MEDIANFILTER_HPP_INCLUDED
#define MEDIANFILTER_HPP_INCLUDED

#include <Arduino.h>

class MedianFilter
{
public:
    // constructor
    MedianFilter();
    MedianFilter(int filter_size);

    // destructor
    //~MedianFilter();


    // method
    void    in(float sample);
    float   out();
    void    reset(float sample);
    void    setSize(int filter_size);
    int     getSize();


private:
    int     _filter_size;
    int     _current_filter_size;
    float*  _buffer;


};

#endif // MEDIANFILTER_HPP_INCLUDED

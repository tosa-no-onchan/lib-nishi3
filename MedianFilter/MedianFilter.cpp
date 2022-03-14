#include "MedianFilter.h"

MedianFilter::MedianFilter()
{
    _filter_size = 1;
    _current_filter_size = 0;
    _buffer = new float[_filter_size];
}

MedianFilter::MedianFilter(int filter_size)
{
    _filter_size = filter_size;
    _current_filter_size = 0;
    _buffer = new float[_filter_size];
}

void MedianFilter::setSize(int filter_size)
{
    _filter_size = filter_size;

    float * tmpBuffer = new float[filter_size];
    memcpy(tmpBuffer, _buffer, filter_size * sizeof(float) );

    delete [] _buffer;

    _buffer = tmpBuffer;
    _current_filter_size = 0;

}

int MedianFilter::getSize()
{
    return _filter_size;
}

int floatCompare(const void* a, const void* b)
{
    if(*(const float*)a < *(const float*)b)
        return -1;
    return *(const float*)a > *(const float*)b;
}

void MedianFilter::in(float sample)
{
    // feed buffer
    if (_current_filter_size<_filter_size) {
        _buffer[_current_filter_size] = sample;
        _current_filter_size++;
    }
    else {
        // circular rotation
        for (int i = 0; i < _filter_size-1; i++) {
            _buffer[i] = _buffer[i+1];
        }
        _buffer[_filter_size-1] = sample;
    }

}

float MedianFilter::out()
{
    float _sorted_buffer[_current_filter_size];
    memcpy(_sorted_buffer, _buffer, sizeof(_buffer[0]) * _current_filter_size);
    qsort(_sorted_buffer, _current_filter_size, sizeof(float), floatCompare);

    return _sorted_buffer[_current_filter_size/2];
}

#include <Timer.h>

Timer _timer_100(100), // loop all 100ms
      _timer_10(10), // loop all 10ms
      _timer_20(20);  //loop all 20ms
      // etc ...

void setup()
{
	_timer_100.start((unsigned long) millis());
	_timer_10.start((unsigned long) millis());
	_timer_20.start((unsigned long) millis());
}

void loop()
{
//***** RUN AT 10Hz *****
	if(_timer_100.delay(millis()))
	{
  		//...
  		//your code here
  	}
//***** RUN AT 100Hz *****
	if(_timer_10.delay(millis()))
	{
		//...
		//your code here
	}
//***** RUN AT 50Hz *****
	if(_timer_20.delay(millis()))
	{
		//...
		//your code here
  }
}
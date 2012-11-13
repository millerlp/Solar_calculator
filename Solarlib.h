
/* Solarlib.h
 * A library to calculate solar position, sunrise, and sunset
 * times.
 *
 *  */
#ifndef Solarlib_h
#define Solarlib_h

#include <Arduino.h>
#include <math.h>
#include <Time.h>

class SolarCalc {
public:
	SolarCalc();
	// need to define the correct return type (array?)
	double SolarTime(time_t t, int timeZoneOffset,
			float lat, float lon);



};
#endif

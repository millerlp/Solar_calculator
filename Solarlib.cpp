/*Nothing here yet


*/
#include <math.h>
#include <Time.h>

#define timeZoneOffset -8 // Time zone offset (hr), zones west of GMT are negative
#define julianUnixEpoch  2440587.5 // julian days to start of unix epoch

// Define some variables
double timeFracDay, JDN, JCN, GMLS, GMAS, EEO, SEC, STL, STA, SRV,
	SAL, MOE, OC, SRA, SDec, vy, EOT, HAS, SolarNoon, SolarNoonDays,
	Sunrise, Sunset, TST, HA, SZA, SEA, AAR, SEC_Corr, SAA;
long unixDays;
time_t SolarNoonTime, Sunset_t, Sunrise_t;

// Constructor function
SolarCalc::SolarCalc(time_t now, int timeZoneOffset,
		float lat, float lon){
	init(now, timeZoneOffset, lat, lon);
}


SolarCalc::SolarTime(time_t t, int timeZoneOffset,
		float lat, float lon){
	// Calculate the time past midnight, as a fractional day value
	// e.g. if it's noon, the result should be 0.5.
	timeFracDay = ((((double)(second(t)/60) + minute(t))/60) +
			hour(t))/24;
  // unixDays is the number of days(+fractional days) since start
  // of the Unix epoch. The division sign will truncate any remainder
  // since this will be done as integer division.
  unixDays = myUnixDate / 86400;
  // calculate Julian Day Number
  JDN = julianUnixEpoch + unixDays;
  // Calculate Julian Century Number
  JCN = (JDN - 2451545) / 36525;
  // Geometric Mean Longitude of Sun (degrees)
  GMLS = (280.46646 + JCN * (36000.76983 + JCN * 0.0003032));
  // Finish GMLS calculation by calculating modolu(GMLS,360) as
  // it's done in R or Excel. C's fmod doesn't work in the same
  // way. The floor() function is from the math.h library.
  GMLS = GMLS - (360 * (floor(GMLS/360)) );
  // Geometric Mean Anomaly of Sun (degrees)
  GMAS = 357.52911 + (JCN * (35999.05029 - 0.0001537 * JCN));

  // Eccentricity of Earth Orbit
  EEO = 0.016708634 - (JCN * (0.000042037 + 0.0000001267 * JCN));
  // Sun Equation of Center
  SEC = sin(GMAS * DEG_TO_RAD) * (1.914602 -
    (JCN * (0.004817 + 0.000014 * JCN))) +
    sin((2*GMAS)* DEG_TO_RAD)*(0.019993-0.000101*JCN) +
    sin((3*GMAS)* DEG_TO_RAD) * 0.000289;

  // Sun True Longitude (degrees)
  STL = GMLS + SEC;

  // Sun True Anomaly (degrees)
  STA = GMAS + SEC;
  // Sun Radian Vector
  SRV = (1.000001018 * (1- EEO * EEO))/(1 + EEO *
      cos(STA * DEG_TO_RAD));
  // Sun Apparent Longitude (degrees)
  SAL = STL - 0.00569 - (0.00478 *
      sin((125.04 - 1934.136 * JCN) * DEG_TO_RAD));

  // Mean Oblique Ecliptic (degrees)
  MOE = 23 + (26 + (21.448-JCN * ( 46.815 + JCN *
    (0.00059 - JCN * 0.001813)))/60)/60;

  // Oblique correction (degrees)
  OC = MOE + 0.00256 * cos((125.04-1934.136*JCN)*DEG_TO_RAD);

  // Sun Right Ascension (degrees)
  SRA = (atan2(cos(OC * DEG_TO_RAD) * sin(SAL * DEG_TO_RAD),
      cos(SAL * DEG_TO_RAD))) * RAD_TO_DEG;

  // Sun Declination (degrees)
  SDec = (asin(sin(OC * DEG_TO_RAD) *
      sin(SAL * DEG_TO_RAD))) * RAD_TO_DEG;

  // var y
  vy = tan((OC/2) * DEG_TO_RAD) * tan((OC/2) * DEG_TO_RAD);

  // Equation of Time
  EOT = 4 * ((vy * sin(2 * (GMLS * DEG_TO_RAD)) -
      2 * EEO * sin(GMAS * DEG_TO_RAD) +
      4 * EEO * vy * sin(GMAS * DEG_TO_RAD) * cos(2*(GMLS*DEG_TO_RAD)) -
      0.5 * vy * vy * sin(4*(GMLS * DEG_TO_RAD)) -
      1.25 * EEO * EEO * sin(2*(GMAS* DEG_TO_RAD))) * RAD_TO_DEG);

  // Hour Angle Sunrise (degrees)
  HAS = acos((cos(90.833*DEG_TO_RAD)/
      (cos(lat*DEG_TO_RAD) * cos(SDec*DEG_TO_RAD))) -
      tan(lat * DEG_TO_RAD) * tan(SDec * DEG_TO_RAD)) * RAD_TO_DEG ;

  // Solar Noon - result is given as fraction of a day
  // Time value is in GMT
  SolarNoon = (720 - 4 * lon - EOT) / 1440 ;
  // SolarNoon is given as a fraction of a day. Add this
  // to the unixDays value, which currently holds the
  // whole days since 1970-1-1 00:00
  SolarNoonDays = unixDays + SolarNoon;
  // SolarNoonDays is in GMT time zone, correct it to
  // the input time zone
  SolarNoonDays = SolarNoonDays + ((double)timeZoneOffset / 24);
  // Then convert SolarNoonDays to seconds
  SolarNoonTime = SolarNoonDays * 86400;
  // Sunrise Time, given as fraction of a day
  Sunrise = SolarNoon - HAS * 4/1440;
  // Convert Sunrise to days since 1970-1-1
  Sunrise = unixDays + Sunrise;
  // Correct Sunrise to local time zone from GMT
  Sunrise = Sunrise + ((double)timeZoneOffset / 24);
  // Convert Sunrise to seconds since 1970-1-1
  Sunrise = Sunrise * 86400;
  // Convert Sunrise to a time_t object (Time library)
  Sunrise_t = (time_t)Sunrise;
  // Sunset Time
  Sunset = SolarNoon + HAS * 4/1440;
  // Convert Sunset to days since 1970-1-1
  Sunset = unixDays + Sunset;
  // Correct Sunset to local time zone from GMT
  Sunset = Sunset + ((double)timeZoneOffset / 24);
  // Convert Sunset to seconds since 1970-1-1
  Sunset = Sunset * 86400;
  // Convert Sunset to a time_t object (Time library)
  Sunset_t = (time_t)Sunset;
  // Sunlight Duration (minutes)
  SunDuration = 8 * HAS;
  // True Solar Time (minutes)
  TST = (timeFracDay * 1440 +
      EOT + 4 * lon - 60 * timeZoneOffset);
  // Finish TST calculation by calculating modolu(TST,360) as
  // it's done in R or Excel. C's fmod doesn't work in the same
  // way. The floor() function is from the math.h library.
  TST = TST - (1440 * (floor(TST/1440)) );
  // Hour Angle (degrees)
  if (TST/4 < 0) {
    HA = TST/4 + 180;
  } else if (TST/4 >= 0) {
    HA = TST/4 - 180;
  }
  // Solar Zenith Angle (degrees)
  SZA = (acos(sin(lat * DEG_TO_RAD) *
      sin(SDec* DEG_TO_RAD) +
      cos(lat * DEG_TO_RAD) *
      cos(SDec * DEG_TO_RAD) *
      cos(HA * DEG_TO_RAD))) * RAD_TO_DEG;
  // Solar Elevation Angle (degrees)
  SEA = 90 - SZA;
  // Approximate Atmospheric Refraction (degrees)
  if (SEA > 85) {
    AAR = 0;
  } else if (SEA > 5) {
    AAR = (58.1 / tan(SEA * DEG_TO_RAD)) -
        0.07 / (pow(tan(SEA * DEG_TO_RAD),3)) +
        0.000086 / (pow(tan(SEA * DEG_TO_RAD),5));
  } else if (SEA > -0.575) {
    AAR = 1735 + SEA * (-581.2 * SEA *
        (103.4 + SEA * (-12.79 + SEA * 0.711)));
  } else {
    AAR = -20.772 / tan(SEA * DEG_TO_RAD);
  }
  AAR = AAR / 3600.0;
  // Solar Elevation Corrected for Atmospheric
  // refraction (degrees)
  SEC_Corr = SEA + AAR;
  // Solar Azimuth Angle (degrees clockwise from North)
  if (HA > 0) {
    SAA = (((acos((sin(lat * DEG_TO_RAD) *
        cos(SZA * DEG_TO_RAD) -
        sin(SDec * DEG_TO_RAD)) /
        (cos(lat * DEG_TO_RAD) *
        sin(SZA * DEG_TO_RAD))) ) *
        RAD_TO_DEG) + 180);
    SAA = SAA - (360 * (floor(SAA/360)));
  } else {
     SAA = (540 - (acos((((sin(lat * DEG_TO_RAD) *
        cos(SZA * DEG_TO_RAD))) -
        sin(SDec * DEG_TO_RAD)) /
        (cos(lat * DEG_TO_RAD) *
        sin(SZA * DEG_TO_RAD)))) *
        RAD_TO_DEG);
     SAA = SAA - (360 * (floor(SAA/360)));
  }

}


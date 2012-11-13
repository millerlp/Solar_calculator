
#include <math.h>
#include <Time.h>

#define timeZoneOffset -8 // Time zone offset (hr), zones west of GMT are negative
#define julianUnixEpoch  2440587.5 // julian days to start of unix epoch
//#define myUnixDate 1352030400 // used for testing 2012-11-04 12:00
#define myUnixDate 1352029680 // used for testing 
#define lat 36.62  // latitude, values north of equator are positive
#define lon -121.9 // longitude, values west of GMT are negative
void setup () {

    Serial.begin(115200); // Set serial port speed
    
    // while the serial stream is not open, do nothing:
    while (!Serial) ;

}

void loop() {
  
  // Initialize a Time value (seconds since 1970-1-1)
  setTime(myUnixDate); // set time using Time library function
  time_t t = now(); // store time value in t
  Serial.println();
  Serial.print("Input time: ");
  printDateTime(t);
  // Print out unixtime representation (seconds since 1970-1-1)
  Serial.println(t); 
  
  
  // Calculate the time past midnight, as a fractional day value
  // e.g. if it's noon, the result should be 0.5.
  double timeFracDay = ((((double)(second(t)/60) + minute(t))/60) + hour(t))/24;
  Serial.println(timeFracDay,7);
  // unixDays is the number of days(+fractional days) since start
  // of the Unix epoch. The division sign will truncate any remainder
  // since this will be done as integer division.
  long unixDays = myUnixDate / 86400;

  // calculate Julian Day Number
  double JDN = julianUnixEpoch + unixDays;
  // Add the fractional day value to the Julian Day number. If the
  // input value was in the GMT time zone, we could proceed directly
  // with this value. 
  JDN = JDN + timeFracDay;
  // Adjust JDN to GMT time zone
  JDN = JDN - ((double)timeZoneOffset / 24);
  // Calculate Julian Century Number
  double JCN = (JDN - 2451545) / 36525;
  // Geometric Mean Longitude of Sun (degrees)
  double GMLS = (280.46646 + JCN * (36000.76983 + JCN * 0.0003032));
  // Finish GMLS calculation by calculating modolu(GMLS,360) as
  // it's done in R or Excel. C's fmod doesn't work in the same
  // way. The floor() function is from the math.h library. 
  GMLS = GMLS - (360 * (floor(GMLS/360)) );
  
  // Geometric Mean Anomaly of Sun (degrees)
  double GMAS = 357.52911 + (JCN * (35999.05029 - 0.0001537 * JCN));
  
  // Eccentricity of Earth Orbit
  double EEO = 0.016708634 - (JCN * (0.000042037 + 0.0000001267 * JCN));
  
  // Sun Equation of Center
  double SEC = sin(GMAS * DEG_TO_RAD) * (1.914602 -
    (JCN * (0.004817 + 0.000014 * JCN))) + 
    sin((2*GMAS)* DEG_TO_RAD)*(0.019993-0.000101*JCN) +
    sin((3*GMAS)* DEG_TO_RAD) * 0.000289;
  
  // Sun True Longitude (degrees)
  double STL = GMLS + SEC;
  
  // Sun True Anomaly (degrees)
  double STA = GMAS + SEC;
  
  // Sun Radian Vector
  double SRV = (1.000001018 * (1- EEO * EEO))/(1 + EEO *  
      cos(STA * DEG_TO_RAD));
  
  // Sun Apparent Longitude (degrees)
  double SAL = STL - 0.00569 - (0.00478 * 
      sin((125.04 - 1934.136 * JCN) * DEG_TO_RAD));
  
  // Mean Oblique Ecliptic (degrees)
  double MOE = 23 + (26 + (21.448-JCN * ( 46.815 + JCN * 
    (0.00059 - JCN * 0.001813)))/60)/60;
  
  // Oblique correction (degrees)    
  double OC = MOE + 0.00256 * cos((125.04-1934.136*JCN)*DEG_TO_RAD);      
  
  // Sun Right Ascension (degrees)
  double SRA = (atan2(cos(OC * DEG_TO_RAD) * sin(SAL * DEG_TO_RAD), 
      cos(SAL * DEG_TO_RAD))) * RAD_TO_DEG;
  
  // Sun Declination (degrees)
  double SDec = (asin(sin(OC * DEG_TO_RAD) * 
      sin(SAL * DEG_TO_RAD))) * RAD_TO_DEG;
      
  // var y    
  double vy = tan((OC/2) * DEG_TO_RAD) * tan((OC/2) * DEG_TO_RAD);      
  
  // Equation of Time
  double EOT = 4 * ((vy * sin(2 * (GMLS * DEG_TO_RAD)) - 
      2 * EEO * sin(GMAS * DEG_TO_RAD) + 
      4 * EEO * vy * sin(GMAS * DEG_TO_RAD) * cos(2*(GMLS*DEG_TO_RAD)) -
      0.5 * vy * vy * sin(4*(GMLS * DEG_TO_RAD)) - 
      1.25 * EEO * EEO * sin(2*(GMAS* DEG_TO_RAD))) * RAD_TO_DEG);
  
  // Hour Angle Sunrise (degrees)
  double HAS = acos((cos(90.833*DEG_TO_RAD)/
      (cos(lat*DEG_TO_RAD) * cos(SDec*DEG_TO_RAD))) - 
      tan(lat * DEG_TO_RAD) * tan(SDec * DEG_TO_RAD)) * RAD_TO_DEG ;
  
  // Solar Noon - result is given as fraction of a day
  // Time value is in GMT
  double SolarNoon = (720 - 4 * lon - EOT) / 1440 ;
  // SolarNoon is given as a fraction of a day. Add this
  // to the unixDays value, which currently holds the
  // whole days since 1970-1-1 00:00
  double SolarNoonDays = unixDays + SolarNoon;
  // SolarNoonDays is in GMT time zone, correct it to 
  // the input time zone
  SolarNoonDays = SolarNoonDays + ((double)timeZoneOffset / 24);
  // Then convert SolarNoonDays to seconds
  time_t SolarNoonTime = SolarNoonDays * 86400;
  Serial.print("Solar Noon Time: ");
  printDateTime(SolarNoonTime);

  
  // Sunrise Time, given as fraction of a day
  double Sunrise = SolarNoon - HAS * 4/1440;
  // Convert Sunrise to days since 1970-1-1
  Sunrise = unixDays + Sunrise;
  // Correct Sunrise to local time zone from GMT
  Sunrise = Sunrise + ((double)timeZoneOffset / 24);
  // Convert Sunrise to seconds since 1970-1-1
  Sunrise = Sunrise * 86400;
  // Convert Sunrise to a time_t object (Time library)
  Sunrise = (time_t)Sunrise; 
  Serial.print("Sunrise time: ");
  printDateTime(Sunrise);
   
  // Sunset Time
  double Sunset = SolarNoon + HAS * 4/1440;
  // Convert Sunset to days since 1970-1-1
  Sunset = unixDays + Sunset;
  // Correct Sunset to local time zone from GMT
  Sunset = Sunset + ((double)timeZoneOffset / 24);
  // Convert Sunset to seconds since 1970-1-1
  Sunset = Sunset * 86400;
  // Convert Sunset to a time_t object (Time library)
  Sunset = (time_t)Sunset;
  Serial.print("Sunset time: ");
  printDateTime(Sunset);
  
  // Sunlight Duration (minutes)
  double SunDuration = 8 * HAS;
  
  // True Solar Time (minutes)
  double TST = (timeFracDay * 1440 + 
      EOT + 4 * lon - 60 * timeZoneOffset);    
  // Finish TST calculation by calculating modolu(TST,360) as
  // it's done in R or Excel. C's fmod doesn't work in the same
  // way. The floor() function is from the math.h library. 
  TST = TST - (1440 * (floor(TST/1440)) );
  // Hour Angle (degrees)
  double HA;
  if (TST/4 < 0) {
    HA = TST/4 + 180;
  } else if (TST/4 >= 0) {
    HA = TST/4 - 180; 
  }
  // Solar Zenith Angle (degrees)
  double SZA = (acos(sin(lat * DEG_TO_RAD) * 
      sin(SDec* DEG_TO_RAD) + 
      cos(lat * DEG_TO_RAD) *
      cos(SDec * DEG_TO_RAD) *
      cos(HA * DEG_TO_RAD))) * RAD_TO_DEG;
  // Solar Elevation Angle (degrees)
  double SEA = 90 - SZA;
  // Approximate Atmospheric Refraction (degrees)
  double AAR;
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
  double SEC_Corr = SEA + AAR;
  // Solar Azimuth Angle (degrees clockwise from North)
  double SAA;
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
  
  Serial.print("Solar Azimuth: ");
  Serial.println(SAA,6);
  Serial.print("Solar Elevation: ");
  Serial.println(SEC_Corr,6);
  delay(1000);
  
}  // end of main loop


// Utility function to print time and date nicely
void printDateTime(time_t t){
  Serial.print(year(t));
  printDateDigits(month(t));
  printDateDigits(day(t));
  Serial.print(" ");
  Serial.print(hour(t));
  printDigits(minute(t));
  printDigits(second(t));
  Serial.println();
}
// Utility function to print month/day digits nicely
void printDateDigits(int digits){
  Serial.print("-");
  if(digits < 10) Serial.print("0");
  Serial.print(digits); 
}
// Utility function for time value printing
void printDigits(int digits){
  Serial.print(":");
  if(digits < 10)
    Serial.print('0');
  Serial.print(digits);
}

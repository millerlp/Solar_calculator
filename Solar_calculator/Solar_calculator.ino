
#include <math.h>
//#include <Wire.h>
//#include <SPI.h>  // not used here, but needed to prevent a RTClib compile error
//#include "RTClib.h"

//RTC_DS1307 RTC; 

#define timeZoneOffset -8 // Time zone offset (hr), zones west of GMT are negative
#define julianUnixEpoch  2440587.5 // julian days to start of unix epoch
#define myUnixDate 1352059200
#define lat 36.62  // latitude, values north of equator are positive
void setup () {
//    Wire.begin(); // Start the I2C
//    RTC.begin(); // Init RTC
    Serial.begin(115200); // Set serial port speed
    
    // while the serial stream is not open, do nothing:
    while (!Serial) ;

}

void loop() {
//  DateTime now = RTC.now(); // Read current date and time
//  DateTime now = DateTime(2012,11,4,12,00,00); // define a fixed time value
//  Serial.print("My date: ");
//  char buff[32]; // character buffer to hold date/time as string
//  Serial.println(now.toString(buff,32));
  // Calculate number of days (+ fractional days) since start of
  // unix epoch
//  double unixDays = now.unixtime() / 86400.0;
  double unixDays = myUnixDate / 86400.0;
  // calculate Julian Day Number
  double JDN = julianUnixEpoch + unixDays; 
  // Adjust JDN to GMT time zone
  JDN = JDN - (timeZoneOffset / 24);
  
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
  double Solar_Noon = (720 - 4 * lon - EOT) / 1440 
  
  //TODO: figure out how to convert fractional day to DateTime or Time
  
  Serial.println(SDec,6);
  Serial.println(HAS,6);
  delay(1000);
  
}

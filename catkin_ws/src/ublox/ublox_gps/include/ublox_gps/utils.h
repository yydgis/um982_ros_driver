#ifndef UBLOX_GPS_UTILS_H
#define UBLOX_GPS_UTILS_H

#include <math.h>
#include <time.h>
#include "ublox_msgs/NavPVT.h"

extern "C" {
  #include "ublox_gps/mkgmtime.h"
}

/**
 * @brief Convert date/time to UTC time in seconds.
 */
template<typename NavPVT>
long toUtcSeconds(const NavPVT& msg) {
  // Create TM struct for mkgmtime
  struct tm time = {0};
  time.tm_year = msg.year - 1900; 
  time.tm_mon = msg.month - 1; 
  time.tm_mday = msg.day;
  time.tm_hour = msg.hour;   
  time.tm_min = msg.min; 
  time.tm_sec = msg.sec;
  // C++ STL doesn't have a mkgmtime (though other libraries do)
  // STL mktime converts date/time to seconds in local time 
  // A modified version of code external library is used for mkgmtime
  return mkgmtime(&time);
}

const static double gpst0[]={1980,1, 6,0,0,0}; 
#define   LEAPS   18 // should got from agric leap_sec
 
typedef struct {        /* time struct */ 
    time_t time;        /* time (s) expressed by standard time_t */ 
    double sec;         /* fraction of second under 1 s */ 
} gtime_t; 
 
gtime_t epoch2time(const double *ep) 
{ 
    const int doy[]={1,32,60,91,121,152,182,213,244,274,305,335}; 
    gtime_t time={0}; 
    int days,sec,year=(int)ep[0],mon=(int)ep[1],day=(int)ep[2]; 
     
    if (year<1970||2099<year||mon<1||12<mon) return time; 
     
    /* leap year if year%4==0 in 1901-2099 */ 
    days=(year-1970)*365+(year-1969)/4+doy[mon-1]+day-2+(year%4==0&&mon>=3?1:0); 
    sec=(int)floor(ep[5]); 
    time.time=(time_t)days*86400+(int)ep[3]*3600+(int)ep[4]*60+sec; 
    time.sec=ep[5]-sec; 
    return time; 
} 
 
gtime_t gpst2time(int week, double sec) 
{ 
    gtime_t t=epoch2time(gpst0); 
     
    if (sec<-1E9||1E9<sec) sec=0.0; 
    t.time+=86400*7*week+(int)sec; 
    t.sec=sec-(int)sec; 
    return t; 
} 
 
gtime_t timeadd(gtime_t t, double sec) 
{ 
    double tt; 
    t.sec+=sec; tt=floor(t.sec); t.time+=(int)tt; t.sec-=tt; 
    return t; 
} 
 
gtime_t GPSTime2UTCTime(int week,double sec,double leapsec) 
{ 
    gtime_t gpst = gpst2time(week,sec); 
    return timeadd(gpst,-leapsec); 
}

gtime_t epoch2time_gnssindex(int gnss)
{
    static gtime_t s_gtimes[] = {
    /*gps*/{315964800,  0.0},
    /*gal*/{935280000,  0.0},
    /*bds*/{1136073600, 0.0},
    };

    if((gnss < 0) || (gnss >= (int)(sizeof(s_gtimes)/sizeof(s_gtimes[0])))) gnss = 0;

    return s_gtimes[gnss];
}

double time2gpst(gtime_t t, int *week)
{
	/*gtime_t t0 = epoch2time(gpst0);*/
    gtime_t t0 = epoch2time_gnssindex(0);

	time_t sec = t.time - t0.time;
	int w = (int)(sec / (86400 * 7));

	if (week) *week = w;
	return (double)(sec - w * 86400 * 7) + t.sec;
}
	
// utc time	
gtime_t timeget(double timeoffset)
{
    gtime_t time;
    double ep[6]={0};

    struct timeval tv;
    struct tm *tt, ltm;
    
    if (!gettimeofday(&tv,NULL)&&(tt=gmtime_r(&tv.tv_sec, &ltm))) {
        ep[0]=tt->tm_year+1900; ep[1]=tt->tm_mon+1; ep[2]=tt->tm_mday;
        ep[3]=tt->tm_hour; ep[4]=tt->tm_min; ep[5]=tt->tm_sec+tv.tv_usec*1E-6;
    }

    time=epoch2time(ep);

    return timeadd(time, timeoffset);
}
	
	
#endif

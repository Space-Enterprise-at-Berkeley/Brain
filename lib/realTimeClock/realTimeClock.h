/*
  realTimeClock.h - A c++ library to initialize the Teensy 4.1's real-time clock and related functions.
  Created by Salvador Salcedo.
*/

#ifndef __REALTIMECLOCK__
#define __REALTIMECLOCK__

#include <TimeLib.h>
#include <DS1307RTC.h>
#include <stdlib.h>
#include <stdio.h>

using namespace std;

namespace RealTimeClock {
    void init() {
      if(timeStatus() != timeSet) 
        setSyncProvider(RTC.get);
    }

    string getFileTime() {
      char buffer[8];

      string result = std::string(itoa(month(), buffer, 10)) + "-";
      result += string(itoa(day(), buffer, 10)) + "-";
      result += string(itoa(year(), buffer, 10)) + "_T";
      result += string(itoa(hour(), buffer, 10)) + ":";
      result += string(itoa(minute(), buffer, 10)) + ":";
      result += string(itoa(second(), buffer, 10)) + "_" + "+8:00";
      return result;
    }
}

#endif /* end of include guard: REALTIMECLOCK */
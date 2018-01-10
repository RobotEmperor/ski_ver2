#include "get_time.h"

namespace GetTime
{
  double GetMillis()
  {
    timespec time;
    clock_gettime(CLOCK_REALTIME, &time);

    long current_ms = time.tv_nsec * 0.000001;
    static double start_sec = time.tv_sec + (double)current_ms * 0.001;
    double result = time.tv_sec + (double)current_ms * 0.001;
    return result - start_sec;
  }

  void GetDate(char *date)
  {
    time_t current_time = time(NULL);
    struct tm *time = localtime(&current_time);

    sprintf(date, "%02d%02d%02d_%02d%02d%02d", 
        time->tm_year - 100, time->tm_mon + 1, time->tm_mday, 
        time->tm_hour, time->tm_min, time->tm_sec);
  }
}

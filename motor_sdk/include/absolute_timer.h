#ifndef ABSOLUTE_TIMER_H
#define ABSOLUTE_TIMER_H

#include <sys/timerfd.h>
#include <stdint.h>

class AbsoluteTimer{
public:
    AbsoluteTimer(double waitTimeS);
    ~AbsoluteTimer();
    void start();
    bool wait();
private:
    void _updateWaitTime(double waitTimeS);
    int _timerFd;
    uint64_t _missed;
    double _waitTime;
    double _startTime;
    double _leftTime;
    double _nextWaitTime;
    itimerspec _timerSpec;
};

class Timer {
 public:

  /*!
   * Construct and start timer
   */
  explicit Timer() { start(); }

  /*!
   * Start the timer
   */
  void start() { clock_gettime(CLOCK_MONOTONIC, &_startTime); }


  void setStart(timespec startTime){ _startTime = startTime;}

  /*!
   * Get milliseconds elapsed
   */
  double getMs() { return (double)getNs() / 1.e6; }

  /*!
   * Get nanoseconds elapsed
   */
  int64_t getNs() {
    struct timespec now;
    clock_gettime(CLOCK_MONOTONIC, &now);
    return (int64_t)(now.tv_nsec - _startTime.tv_nsec) +
           1000000000 * (now.tv_sec - _startTime.tv_sec);
  }

  /*!
   * Get seconds elapsed
   */
  double getSeconds() { return (double)getNs() / 1.e9; }

  struct timespec _startTime;
};

long long getSystemTime();

double getTimeSecond();

#endif  // ABSOLUTETIMER_H

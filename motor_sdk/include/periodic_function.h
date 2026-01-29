#ifndef PROJECT_FUNCTION_H
#define PROJECT_FUNCTION_H

#include <string>
#include <thread>
#include <vector>
#include <functional>

class PeriodicFunction {
 public:
  PeriodicFunction(float period, std::string name,  std::function<void()> func, int priority = -1);
  PeriodicFunction(float period, std::string name,  std::function<void()> func, int bindCPU, int priority = -1);
  void start();
  void stop();
  ~PeriodicFunction() { stop(); }

  void setPeriod(float period) {
      _period = period;
      _reset_period_flag = true;
  }
  /*!
   * Get the desired period for the task
   */
  float getPeriod() { return _period; }


 private:

  std::function<void()> _function;
  void loopFunction();
  void run();
  void init();
  float _period;
  int _bindCPU;
  int _priority = 0;
  bool _bind_cpu_flag = false;
  bool _reset_period_flag = false;
  bool _running = false;

  std::string _name;
  std::thread _thread;
};


#endif  // PROJECT_PERIODICTASK_H

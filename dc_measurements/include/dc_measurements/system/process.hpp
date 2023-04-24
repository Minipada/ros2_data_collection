#ifndef PROCESS_HPP
#define PROCESS_HPP

#include <ctime>
#include <string>
#include <vector>

/********* Process class *******
 *  Initializes a queue structure, preparing it for usage.
 */
class Process
{
public:
  Process(int pid);
  int pid();
  std::string user();
  std::string command();
  float cpuUtilization() const;
  std::string cpuPretty(int precision) const;
  int ram();
  long upTime();
  bool operator<(Process const& that) const;
  bool operator>(Process const& that) const;
  void updateCpuData();

private:
  int pid_{ 0 };
  std::string user_ = "";
  std::string command_ = "";
  long start_time_{ 0 };

  void addValue(int jiffy_type, long value);
  std::vector<std::vector<long>> cpu_data_ = {};
  float cpu_util_{ 0 };
};

enum JiffyData
{
  K_JIFFIES_SYS = 0,
  K_JIFFIES_PROC
};

#endif

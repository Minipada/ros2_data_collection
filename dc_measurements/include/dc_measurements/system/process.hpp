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
  int Pid();
  std::string User();
  std::string Command();
  float CpuUtilization() const;
  std::string CpuPretty(int precision) const;
  int Ram();
  long UpTime();
  bool operator<(Process const& that) const;
  bool operator>(Process const& that) const;
  void UpdateCpuData();

private:
  int pid_{ 0 };
  std::string user_ = "";
  std::string command_ = "";
  long start_time_{ 0 };

  void AddValue(int jiffy_type, long value);
  std::vector<std::vector<long>> cpu_data_ = {};
  float cpu_util_{ 0 };
};

enum JiffyData
{
  kJiffiesSys_ = 0,
  kJiffiesProc_
};

#endif

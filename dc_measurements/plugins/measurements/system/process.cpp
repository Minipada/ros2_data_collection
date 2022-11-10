#include "process.hpp"

#include <unistd.h>

#include <cctype>
#include <cmath>
#include <ctime>
#include <iomanip>
#include <iostream>
#include <numeric>
#include <sstream>
#include <string>
#include <vector>

#include "linux_parser.hpp"

using std::string;
using std::to_string;
using std::vector;

namespace lp = LinuxParser;

/********* Process::Process *******
 *  Process object constructor that sets up initial process related values.
 *  Inputs: Numeric process identification number.
 *  Outputs: A Process object is created and added to the System process vector.
 */
Process::Process(int pid) : pid_(pid)
{
  int uid = lp::Uid(pid);
  command_ = lp::Command(pid);
  user_ = lp::User(uid);
  start_time_ = lp::StartTime(pid);
  vector<long> data = {};
  cpu_data_.push_back(data);
  cpu_data_.push_back(data);
}

/********* Process::Pid *******
 *  Returns the Process process ID.
 *  Inputs: None.
 *  Outputs: Integer representing the process ID of this Process.
 */
int Process::Pid()
{
  return pid_;
}

/********* Process::CpuUtilization *******
 *  Returns this process' current utilization as a percentage of total CPU time.
 *  Inputs: None.
 *  Outputs: A float representing process CPU usage.
 */
float Process::CpuUtilization() const
{
  return cpu_util_;
}

/********* Process::CpuPretty *******
 *  CPU usage in a format suitable for NCursesDisplay::DisplayProcesses.
 *  Inputs: None.
 *  Outputs: String representation Process CPU usage in a rounded form.
 */
string Process::CpuPretty(int precision) const
{
  string cpu_usage = "";
  float cpu_float = cpu_util_;

  std::stringstream ss;
  ss << std::fixed << std::setprecision(precision) << cpu_float;
  cpu_usage = ss.str();
  return cpu_usage;
}

/********* Process::Command *******
 *  Accesses the command that launched this Process.
 *  Inputs: None.
 *  Outputs: String of this Process' invoking command.
 */
string Process::Command()
{
  return command_;
}

/********* Process::Ram *******
 *  A friendly output of memory in use by this process using SI units.
 *  Inputs: None.
 *  Outputs: A string of Process RAM usage in giga, mega, or kilo-bytes.
 */
int Process::Ram()
{
  return lp::Ram(pid_);
}

/********* Process::User *******
 *  Initializes a queue structure, preparing it for usage.
 *  Inputs: Pointer to Queue_t, number of elements as size, queue_data pointer,
 *          pointer to a putFunction and a getFunction,
 *          function pointer callback to handle queue xfer from the scheduler.
 *  Outputs: None
 */
string Process::User()
{
  return user_;
}

/********* Process::UpTime *******
 *  Returns the age of the Process in seconds.
 *  Inputs: None.
 *  Outputs: Long integer of Process uptime in seconds.
 */
long Process::UpTime()
{
  long now;
  long uptime{ 0l };

  now = lp::UpTime();
  uptime = now - start_time_;
  return uptime;
}

/********* Process::operator> *******
 *  Greater-than operator for sorting.
 *  Inputs: A reference to another Process object being compared.
 *  Outputs: A boolean true or false, depending on the outcome of the test.
 */
bool Process::operator>(Process const& that) const
{
  return (std::isless(CpuUtilization(), that.CpuUtilization()));
}

/********* Process::operator< *******
 *  Less-than operator for sorting.
 *  Inputs: A reference to another Process object being compared.
 *  Outputs: A boolean true or false, depending on the outcome of the test.
 */
bool Process::operator<(Process const& that) const
{
  return (std::isgreater(CpuUtilization(), that.CpuUtilization()));
}

/********* Process::UpdateCpuData *******
 *  Updates the relevant system data to maintain an accurate approximation of
 *  the CPU being used by this Process.
 *  Inputs: None.
 *  Outputs: None.
 */
void Process::UpdateCpuData()
{
  std::vector<long> jiffies_proc;
  long jiffies_proc_total{ 0l };
  long jiffies_system{ 0l };
  float cpu_util{ 0.0f };

  jiffies_system = lp::TotalJiffies();
  AddValue(kJiffiesSys_, jiffies_system);
  jiffies_proc = lp::PidStat(pid_);
  jiffies_proc_total = std::accumulate(jiffies_proc.begin(), jiffies_proc.end() - 1, 0);
  AddValue(kJiffiesProc_, jiffies_proc_total);

  cpu_util = 1000 * ((float)cpu_data_[kJiffiesProc_][lp::kPresent_] - (float)cpu_data_[kJiffiesProc_][lp::kPast_]) /
             ((float)cpu_data_[kJiffiesSys_][lp::kPresent_] - (float)cpu_data_[kJiffiesSys_][lp::kPast_]);

  if (std::isnan(cpu_util))
  {
    cpu_util = 0.0f;
  }
  cpu_util_ = cpu_util;
}

/********* Process::AddValue *******
 *  Initializes a queue structure, preparing it for usage.
 *  Inputs: Type integer from enum JiffyData and a long integer representing
 *          total processor Jiffies for the process and systemwide.
 *  Outputs: None
 */
void Process::AddValue(int jiffy_type, long value)
{
  if (cpu_data_[jiffy_type].size() != 2)
  {
    cpu_data_[jiffy_type].push_back(value);
    cpu_data_[jiffy_type].push_back(value);
  }
  else
  {
    cpu_data_[jiffy_type].erase(cpu_data_[jiffy_type].begin());
    cpu_data_[jiffy_type].push_back(value);
  }
}

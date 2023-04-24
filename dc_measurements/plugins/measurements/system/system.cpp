#include "dc_measurements/system/system.hpp"

#include <unistd.h>

#include <algorithm>
#include <cstddef>
#include <iostream>
#include <set>
#include <string>
#include <vector>

#include "dc_measurements/system/linux_parser.hpp"
#include "dc_measurements/system/process.hpp"
#include "dc_measurements/system/processor.hpp"

namespace lp = LinuxParser;

using std::set;
using std::size_t;
using std::string;
using std::vector;

/********* System::Cpu *******
 *  Provides access to the Processor object representing the system CPU usage.
 *  Inputs: None.
 *  Outputs: A reference to the Processor object living inside the System.
 */
Processor& System::cpu()
{
  return cpu_;
}

/********* System::Processes *******
 *  Maintains a list of processes currently running. Removes terminated
 *  processes, adds newly launched processes, and sorts according to CPU usage.
 *  Inputs: None.
 *  Outputs: A reference to the System vector of Process objects.
 */
vector<Process>& System::processes()
{
  vector<int> current_pids{ dumpPids() };
  vector<int> fresh_pids{ lp::Pids() };
  vector<int> remove_pids;
  vector<int> add_pids;
  if (current_pids.empty())
  {
    addProcs(fresh_pids);
  }
  else
  {
    remove_pids = noPartner(fresh_pids, current_pids);
    add_pids = noPartner(current_pids, fresh_pids);
    addProcs(add_pids);
    pruneProcs(remove_pids);
  }
  sortProcsByCpu();
  return processes_;
}

/********* System::Kernel *******
 *  Version information for the system kernel.
 *  Inputs: None.
 *  Outputs: A string representing the system kernel version.
 */
std::string System::kernel()
{
  return lp::Kernel();
}

/********* System::MemoryUtilization *******
 *  System memory usage as a percentage of total installed RAM.
 *  Inputs: None.
 *  Outputs: A float of RAM percentage in use.
 */
float System::memoryUtilization()
{
  return lp::MemoryUtilization();
}

/********* System::OperatingSystem *******
 *  Retrieves the host distribution name.
 *  Inputs: None.
 *  Outputs: A string containing the host OS name.
 */
std::string System::operatingSystem()
{
  return lp::OperatingSystem();
}

/********* System::RunningProcesses *******
 *  Number of active processes.
 *  Inputs: None.
 *  Outputs: Integer counting active processes.
 */
int System::runningProcesses()
{
  return lp::RunningProcesses();
}

/********* System::TotalProcesses *******
 *  Number of processes running.
 *  Inputs: None.
 *  Outputs: Integer of the number of processes.
 */
int System::totalProcesses()
{
  return lp::TotalProcesses();
}

/********* System::UpTime *******
 *  Seconds since the system started up.
 *  Inputs: None.
 *  Outputs: A long integer type of the system uptime in seconds.
 */
long System::upTime()
{
  return lp::UpTime();
}

/********* System::DumpPids *******
 *  Retrieves all the process IDs stored in the System object's Process vector.
 *  Inputs: None.
 *  Outputs: A vector of PIDs as ints.
 */
vector<int> System::dumpPids()
{
  vector<int> pid_list = {};
  for (Process process : processes_)
  {
    pid_list.push_back(process.pid());
  }
  return pid_list;
}

/********* System::NoPartner *******
 *  If partner_a and partner_b are not a couple, then b is returned.
 *  Inputs: A vector of integers to be searched, and vector of search ints.
 *  Outputs: A vector of int from b that are not found in a.
 */
vector<int> System::noPartner(vector<int> partner_a, const vector<int>& partner_b)
{
  vector<int> not_list;
  std::vector<int>::iterator i;

  for (int b : partner_b)
  {
    i = std::find(partner_a.begin(), partner_a.end(), b);
    if (i == partner_a.end())
    {
      not_list.push_back(b);
    }
  }
  return not_list;
}

/********* System::AddProcs *******
 *  Adds Process objects to the Process vector.
 *  Inputs: A vector of PIDs as ints.
 *  Outputs: None.
 */
void System::addProcs(const vector<int>& pid_list)
{
  for (int pid : pid_list)
  {
    processes_.emplace_back(pid);
  }
}

/********* System::pruneProcs *******
 *  Removes from the Process vector processes that are no longer running.
 *  Inputs: A vector of ints representing PIDs that have been terminated.
 *  Outputs: None.
 */
void System::pruneProcs(vector<int> dead_pids)
{
  vector<Process> new_procs;
  std::vector<int>::iterator i;

  for (Process process : processes_)
  {
    i = std::find(dead_pids.begin(), dead_pids.end(), process.pid());
    if (i == dead_pids.end())
    {
      new_procs.push_back(process);
    }
  }
  processes_ = new_procs;
}

/********* System::SortProcsByCpu *******
 *  Updates CPU usage data for each of the System's vector of Process objects
 *  and sorts according to CPU usage in descending order.
 *  Inputs: None.
 *  Outputs: None.
 */
void System::sortProcsByCpu()
{
  for (Process& process : processes_)
  {
    process.updateCpuData();
  }
  std::sort(processes_.begin(), processes_.end());
}

#ifndef SYSTEM_HPP
#define SYSTEM_HPP

#include <string>
#include <vector>

#include "process.hpp"
#include "processor.hpp"

/********* System class *******
 *  Unifier that provides interfacing to the system monitor modules.
 */
class System
{
public:
  Processor& cpu();
  std::vector<Process>& processes();
  float memoryUtilization();
  long upTime();
  int totalProcesses();
  int runningProcesses();
  std::string kernel();
  std::string operatingSystem();

private:
  Processor cpu_ = {};
  std::vector<Process> processes_ = {};

  std::vector<int> dumpPids();
  std::vector<int> noPartner(const std::vector<int> partner_a, const std::vector<int>& partner_b);
  void addProcs(const std::vector<int>& pid_list);
  void pruneProcs(std::vector<int> dead_pids);
  void sortProcsByCpu();
};

#endif

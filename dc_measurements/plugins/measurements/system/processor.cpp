#include "processor.hpp"

#include <sys/time.h>

#include <cmath>
#include <ctime>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

#include "linux_parser.hpp"

namespace lp = LinuxParser;

/********* Processor::Utilization *******
 *  Provides usage as a percentage of total CPU time for each system processor.
 *  Inputs: None.
 *  Outputs: A vector of floats representing CPU usage for each processor.
 */
std::vector<float> Processor::Utilization()
{
  UpdateData();
  UpdateResult();
  return cpu_result_;
}

/********* Processor::UpdateData *******
 *  Updates CPU usage information, sampling CPU time for each processor.
 *  Inputs: None.
 *  Outputs: None.
 */
void Processor::UpdateData()
{
  std::string line = "";
  std::string line_elements = "";
  int state_value{ 0 };
  int count{ 0 };
  std::vector<int> cpu_state;
  int idle{ 0 };
  int active{ 0 };

  std::ifstream fs(lp::kProcDirectory + lp::kStatFilename);
  if (fs.is_open())
  {
    std::getline(fs, line);
    while (std::getline(fs, line))
    {
      if (line.find("cpu") == std::string::npos)
      {
        break;
      }
      // line_elements = line.substr(6);
      line_elements = line.substr(line.find_first_of(" \t") + 1);

      std::stringstream ss(line_elements);
      while (ss >> state_value)
      {
        cpu_state.push_back(state_value);
      }

      idle = cpu_state[lp::kIdle_] + cpu_state[lp::kIOwait_];
      active = cpu_state[lp::kUser_] + cpu_state[lp::kNice_] + cpu_state[lp::kSystem_] + cpu_state[lp::kIRQ_] +
               cpu_state[lp::kSoftIRQ_] + cpu_state[lp::kSteal_];

      AddCpuSample(count, idle, active);
      cpu_state.clear();
      count++;
    }
    fs.close();
  }
}

/********* Processor::UpdateResult *******
 *  Updates the CPU usage as percentage of total system time for each CPU.
 *  Inputs: None.
 *  Outputs: None.
 */
void Processor::UpdateResult()
{
  float total{ 0.0f };
  float total_prev{ 0.0f };
  float totald{ 0.0f };
  float idled{ 0.0f };
  float cpu_usage_pct{ 0.0f };
  std::vector<float> result;

  for (std::vector<CpuNData> cpu : cpu_data_)
  {
    total = cpu[lp::kPresent_].idle + cpu[lp::kPresent_].active;
    total_prev = cpu[lp::kPast_].idle + cpu[lp::kPast_].active;
    totald = total - total_prev;
    idled = cpu[lp::kPresent_].idle - cpu[lp::kPast_].idle;
    cpu_usage_pct = (totald - idled) / totald;
    if (std::isnan(cpu_usage_pct))
    {
      result.push_back(0.0f);
    }
    else
    {
      result.push_back(cpu_usage_pct);
    }
  }
  cpu_result_ = result;
}

/********* Processor::AddCpuSample *******
 *  Adds a sample to the cpu_data_ vector, containing a vector for each CPU
 *  composed of two periodic samples of idle and active time.
 */
void Processor::AddCpuSample(int cpu_id, int idle, int active)
{
  size_t count = cpu_id;
  CpuNData data = { idle, active };

  // If vector for this CPUn doesn't yet exist, we need to first create
  // a new CPUn vector, and prime its CpuNData vector with two samples.
  if (count + 1 > cpu_data_.size())
  {
    std::vector<CpuNData> new_cpu{ data };
    cpu_data_.push_back(new_cpu);
    cpu_data_[cpu_id].push_back(data);
  }
  // Otherwise we will remove the oldest samples and push in the new samples.
  else
  {
    cpu_data_[cpu_id].erase(cpu_data_[cpu_id].begin());
    cpu_data_[cpu_id].push_back(data);
  }
}

/********* Processor::NumCpus *******
 *  Provides the number of CPUs currently readable from procfs. This is used by
 *  NCursesDisplay::Display to determine the system_window vertical size.
 *  Inputs: None.
 *  Outputs: Number of CPUs in the system.
 */
int Processor::NumCpus()
{
  if (cpu_data_.size() == 0)
  {
    UpdateData();
  }
  return cpu_data_.size();
}

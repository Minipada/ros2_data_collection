#ifndef PROCESSOR_HPP
#define PROCESSOR_HPP

#include <ctime>
#include <vector>

/********* Processor class *******
 *  Abstracts the system CPUs, collecting and calculating processor usage.
 */

struct CpuNData
{
  int idle;
  int active;
};

class Processor
{
public:
  std::vector<float> utilization();
  void updateData();
  void updateResult();
  int numCpus();

private:
  // Each vector represents CPU n, with a nested vector of CPU samples.
  std::vector<std::vector<CpuNData>> cpu_data_;
  // Each float represents the percentage of active time for CPU n.
  std::vector<float> cpu_result_;

  void addCpuSample(int cpu_id, int idle, int active);
};

#endif

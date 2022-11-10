#ifndef SYSTEM_PARSER_HPP
#define SYSTEM_PARSER_HPP

#include <algorithm>
#include <fstream>
#include <regex>
#include <sstream>
#include <string>
#include <vector>

namespace LinuxParser
{

/********* Paths *******
 *  Initializes a queue structure, preparing it for usage.
 *  Inputs: Pointer to Queue_t, number of elements as size, queue_data pointer,
 *          pointer to a putFunction and a getFunction,
 *          function pointer callback to handle queue xfer from the scheduler.
 *  Ouputs: None
 */
const std::string kProcDirectory{ "/proc/" };
const std::string kCmdlineFilename{ "/cmdline" };
const std::string kCpuinfoFilename{ "/cpuinfo" };
const std::string kStatusFilename{ "/status" };
const std::string kStatFilename{ "/stat" };
const std::string kUptimeFilename{ "/uptime" };
const std::string kMeminfoFilename{ "/meminfo" };
const std::string kVersionFilename{ "/version" };
const std::string kOSPath{ "/etc/os-release" };
const std::string kPasswordPath{ "/etc/passwd" };

/********* System *******
 *  Initializes a queue structure, preparing it for usage.
 *  Inputs: Pointer to Queue_t, number of elements as size, queue_data pointer,
 *          pointer to a putFunction and a getFunction,
 *          function pointer callback to handle queue xfer from the scheduler.
 *  Ouputs: None
 */
//
float MemoryUtilization();
long UpTime();
std::vector<int> Pids();
int TotalProcesses();
int RunningProcesses();
std::string OperatingSystem();
std::string Kernel();

/********* Queue_init *******
 *  Initializes a queue structure, preparing it for usage.
 *  Inputs: Pointer to Queue_t, number of elements as size, queue_data pointer,
 *          pointer to a putFunction and a getFunction,
 *          function pointer callback to handle queue xfer from the scheduler.
 *  Ouputs: None
 */
enum CPUState
{
  kUser_ = 0,
  kNice_,
  kSystem_,
  kIdle_,
  kIOwait_,
  kIRQ_,
  kSoftIRQ_,
  kSteal_,
  kGuest_,
  kGuestNice_
};

enum CPUData
{
  kPast_ = 0,
  kPresent_
};

enum MemInfo
{
  kMemTotal_ = 0,
  kMemFree_,
};

enum PidStat
{
  kUTime_ = 0,
  kSTime_,
  kCUTime_,
  kCSTime_,
  kStartTime_
};

/********* Queue_init *******
 *  Initializes a queue structure, preparing it for usage.
 *  Inputs: Pointer to Queue_t, number of elements as size, queue_data pointer,
 *          pointer to a putFunction and a getFunction,
 *          function pointer callback to handle queue xfer from the scheduler.
 *  Ouputs: None
 */
std::string Command(int pid);
std::vector<long> PidStat(int pid);
int Ram(int pid);
long TotalJiffies();
int Uid(int pid);
std::string User(int pid);
long StartTime(int pid);

/********* Queue_init *******
 *  Initializes a queue structure, preparing it for usage.
 *  Inputs: Pointer to Queue_t, number of elements as size, queue_data pointer,
 *          pointer to a putFunction and a getFunction,
 *          function pointer callback to handle queue xfer from the scheduler.
 *  Ouputs: None
 */
std::string NthToken(std::string line, int token_pos);
int SysClk();
}  // namespace LinuxParser

#endif

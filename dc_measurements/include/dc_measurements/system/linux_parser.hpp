#ifndef SYSTEM_PARSER_HPP
#define SYSTEM_PARSER_HPP

#include <algorithm>
#include <filesystem>
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
 *  Outputs: None
 */
const std::string K_PROC_DIRECTORY{ "/proc/" };
const std::string K_CMDLINE_FILENAME{ "/cmdline" };
const std::string K_CPUINFO_FILENAME{ "/cpuinfo" };
const std::string K_STATUS_FILENAME{ "/status" };
const std::string K_STAT_FILENAME{ "/stat" };
const std::string K_UPTIME_FILENAME{ "/uptime" };
const std::string K_MEMINFO_FILENAME{ "/meminfo" };
const std::string K_VERSION_FILENAME{ "/version" };
const std::string K_OS_PATH{ "/etc/os-release" };
const std::string K_PASSWORD_PATH{ "/etc/passwd" };

/********* System *******
 *  Initializes a queue structure, preparing it for usage.
 *  Inputs: Pointer to Queue_t, number of elements as size, queue_data pointer,
 *          pointer to a putFunction and a getFunction,
 *          function pointer callback to handle queue xfer from the scheduler.
 *  Outputs: None
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
 *  Outputs: None
 */
enum CPUState
{
  K_USER = 0,
  K_NICE,
  K_SYSTEM,
  K_IDLE,
  K_IO_WAIT,
  K_IRQ,
  K_SOFT_IRQ,
  K_STEAL,
  K_GUEST,
  K_GUEST_NICE
};

enum CPUData
{
  K_PAST = 0,
  K_PRESENT
};

enum MemInfo
{
  K_MEM_TOTAL = 0,
  K_MEM_FREE,
};

enum PidStat
{
  K_U_TIME = 0,
  K_S_TIME,
  K_CU_TIME,
  K_CS_TIME,
  K_START_TIME
};

/********* Queue_init *******
 *  Initializes a queue structure, preparing it for usage.
 *  Inputs: Pointer to Queue_t, number of elements as size, queue_data pointer,
 *          pointer to a putFunction and a getFunction,
 *          function pointer callback to handle queue xfer from the scheduler.
 *  Outputs: None
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
 *  Outputs: None
 */
std::string NthToken(const std::string& line, int token_pos);
int SysClk();
}  // namespace LinuxParser

#endif

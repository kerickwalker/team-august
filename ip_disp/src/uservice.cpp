/* #***************************************************************************
 #*   Copyright (C) 2024 by DTU
 #*   jcan@dtu.dk
 #*
 #*
 #* The MIT License (MIT)  https://mit-license.org/
 #*
 #* Permission is hereby granted, free of charge, to any person obtaining a copy of this software
 #* and associated documentation files (the “Software”), to deal in the Software without restriction,
 #* including without limitation the rights to use, copy, modify, merge, publish, distribute,
 #* sublicense, and/or sell copies of the Software, and to permit persons to whom the Software
 #* is furnished to do so, subject to the following conditions:
 #*
 #* The above copyright notice and this permission notice shall be included in all copies
 #* or substantial portions of the Software.
 #*
 #* THE SOFTWARE IS PROVIDED “AS IS”, WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED,
 #* INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR
 #* PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE
 #* FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE,
 #* ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 #* THE SOFTWARE. */


#include <stdio.h>
#include <signal.h>
#include "CLI/CLI.hpp"
#include <filesystem>
#include <ifaddrs.h>
#include <netinet/in.h>
#include <string.h>
#include <arpa/inet.h>
#include <netpacket/packet.h>
#include <linux/wireless.h>
#include <sys/ioctl.h>
#include <unistd.h>
#include <future>
#include <string>

#include "uservice.h"
#include "steensy.h"
#include "sgpiod.h"
#include "umqtt.h"
#include <mutex>

#define REV "$Id: uservice.cpp 1235 2026-02-01 15:17:04Z jcan $"
// define the service class
UService service;
// make a configuration structure
mINI::INIStructure ini;

void signal_callback_handler(int signum)
{ // called when pressing ctrl-C
  UTime t("now");
  const int MSL = 100;
  char s[MSL];
  printf("# at %s Interrupted by signal %d\n", t.getDateTimeAsString(s, true), signum);
  service.terminate();
  exit(signum);
}

bool UService::setup(int argc,char **argv)
{ // Interrupt signal handler
  signal(SIGINT, signal_callback_handler);
  signal(SIGQUIT, signal_callback_handler); // 3
  signal(SIGHUP, signal_callback_handler); // 1
  signal(SIGPWR, signal_callback_handler); // 30
  signal(SIGTERM, signal_callback_handler); // 15 (pkill default)
  //
  CLI::App cli{"IP_display app"};
  // cli.add_option("-d,--device", service.usbDev, "USB device name for Teensy (default is /dev/ttyACM0)");
  // add reply to version request
  bool version{false};
  cli.add_flag("-v,--version", version, "Latest SVN version (for uservice.cpp)");
  bool nologging = false;
  cli.add_flag("-l,--logging-off", nologging, "Do not start logging right away (wait for MQTT log message)");
  bool api_list{false};
  cli.add_flag("-a,--api-list", api_list, "List available modules");
  bool no_gpio{false};
  cli.add_flag("-g,--no-gpio", no_gpio, "Don't try to access GPIO pins (i.e. not running on a Raspberry)");
  cli.allow_windows_style_options();
  CLI11_PARSE(cli, argc, argv);
  //
  // create an ini-file structure
  iniFile = new mINI::INIFile(iniFileName);
  // and read the file (if any, else just create the 'ini' structure)
  iniFile->read(ini);
  if (not ini.has("service"))
  { // no data yet, so generate some default values
    ini["service"]["use_robot_hardware"] = "true";
    ini["service"]["logpath"] = "log_%d/";
    ini["service"]["; The '%d' will be replaced with date and timestamp (Must end with a '/')."] = "";
    ini["service"]["max_logging_minutes"] = "60.0";
    ini["service"]["log_service"] = "true";
  }
  // make logfile structure
  // logging
  logPath = ini["service"]["logpath"];
  int n = logPath.find("%d");
  if (n > 0)
  { // date should be added to path
    UTime t("now");
    std::string dpart = t.getForFilename();
    logPath.replace(n, 2, dpart);
  }
  std::error_code e;
  bool ok = std::filesystem::create_directory(logPath, e);
  if (ok)
  {
    printf("# UService:: created directory %s\n", logPath.c_str());
    std::string s = "cp robot.ini " + logPath;
    system(s.c_str());
    // printf("# UService:: robot.ini copied to %s/robot.ini\n", logPath.c_str());
    //startedLogging.now();
  }
  else if (n > 0)
  { // failed (probably: path exist already)
    std::perror("#*** UService:: Failed to create log path:");
  }
  if (nologging)
  { // command line option for no logging at start
    stop_logging = true;
    printf("# UService:: no logging at start\n");
  }
  if (ini["service"]["log_service"] == "true")
  { // open logfile
    std::string fn = service.logPath + "log_service.txt";
    // open logfile and write legend
    logfile = fopen(fn.c_str(), "w");
    fprintf(logfile, "%% IP display service for Teensy\n");
    fprintf(logfile, "%% Version %s\n", getVersionString().c_str());
    fprintf(logfile, "%% 1 \tTime (sec) from system\n");
    fprintf(logfile, "%% 2 \tNumber of users on Linux system\n");
    fprintf(logfile, "%% 3 \tCPU temperature (deg C)\n");
    fprintf(logfile, "%% 4 \t%% (to help import to Matlab)\n");
    fprintf(logfile, "%% 5 \tMessage string (possibly more lines)\n");
  }
    //
  bool theEnd = false;
  //
  if (version)
  {
    printf("RAUBASE SVN service version%s\n", getVersionString().c_str());
    theEnd = true;
  }
  if (api_list)
  {
    printf("Available API modules:\n");
    printf("\tteensy1  Communication withe Teensy\n");
    theEnd = true;
  }
  if (not theEnd)
  { // open the main data source/destination
    // teensy1.setup();
    if (not no_gpio)
      gpio.setup();
    mqtt.setup();
    setupComplete = true;
    usleep(2000);
    //
  }
  return not theEnd;
}

bool UService::decode(const char* msg, UTime&)
{ // decode messages from Teensy
  bool used = true;
  if      (false) {}
  //
  else if (strncmp(msg, "dname ", 6) == 0)
  {
    const char * p1 = strrchr(msg, ' ');
    while (*p1 == ' ')
      p1++;
    saveRobotName(p1);
    toLog(msg);
//     got_id = true;
    printf("# UService::decode: got robot ID: %s\n", msg);
  }
  else if (strncmp(msg, "hbt ", 4) == 0)
  {
    toLog(msg);
  }//
  else
    used = false;
  return used;
}

bool UService::mqttDecode(const char* topic, const char *, UTime&)
{ // decode messages from Teensy
  bool used = true;
  if (strcmp(topic, "robobot/cmd/shutdown") == 0)
  {
    toLog(topic);
    printf("# UService::mqttDecode: got shutdown topic: %s\n", topic);
    terminate();
  }
  else
  {
    used = false;
    printf("# UService::mqttDecode: got unused topic: %s\n", topic);
  }
  return used;
}


void UService::terminate()
{ // Terminate modules (especially threads and log files)
  if (terminating or not setupComplete)
    return;
  printf("# --------- terminating -----------\n");
  terminating = true;
  stop = true; // stop all threads, when finished current activity
  // terminate sensors before Teensy
  // teensy1.terminate();
  gpio.terminate();
  mqtt.terminate();
  //
  if (not ini.has("ini"))
  {
    ini["ini"]["; set 'saveConfig' to 'false' to avoid autosave"] = "";
    ini["ini"]["saveConfig"] = "true";
  }
  std::string shouldSave = ini["ini"]["saveConfig"];
  // fflush(NULL);
  if (shouldSave != "false")
  { // write any changes ini-file values (and structures)
    ini["ini"]["version"] = getVersionString();
    printf("# UService:: Trying to save %s (crashes sometimes)\r\n", iniFileName.c_str());
    iniFile->write(ini, true);
    printf("# UService:: configuration saved to %s\n", iniFileName.c_str());
  }
  // service must be the last to close
  if (logfile != nullptr)
  {
    std::string tm = exec("date");
    toLog(tm.c_str());
    toLog("Closed log");
    fclose(logfile);
    printf("# UService:: logfile closed\n");
  }
}

std::string UService::getVersionString()
{
  // #define REV "$Id: uservice.cpp 1235 2026-02-01 15:17:04Z jcan $"
  std::string ver = REV;
  int n1 = ver.find(' ', 10);
  int n2 = ver.rfind("Z ");
  std::string part = ver.substr(n1, n2-n1);
  return part;
}

////////// run //////////////

void UService::run()
{ // potential keyboard input
  // bool ip_list_changed;
  // bool ip_sentToTeensy = false;
  int loop = 0;
  // bool ipRemoved = false;
  std::string tm = exec("date");
  toLog(tm.c_str());
  std::cout << tm << " ip_disp started (date may be from last clean shutdown)\n";
  while (not stop)
  {
    // findIPs();
    // // update message
    // ip_list_changed = updateIPlist();
    // tell Regbot display about the host IP
    float t = measureCPUtemp();
    if (fabsf(t - cpuTemp) > 0.5)
    {
      cpuTemp = t;
      toLog("");
      // publish(const char * topic, const char * payload, UTime * msgTime, int qos)
    }
    const int MSL = 150;
    char s[MSL];
    char s2[50];
    UTime tn("now");
    snprintf(s, MSL, "%.1f deg %s\n", t, tn.getDateTimeAsString(s2));
    mqtt.publish("robobot/ipdisp/temp", s, &tn, 0);
    // printf("# send %s\n", s);
             // if (true)
    // { // No longer needed, taken over by teensy_interface.
    //   // check if someone is using the Teensy using lsof (list open files)
    //   bool fileOK = file_exists(teensy1.usbDevName);
    //   if (fileOK)
    //   { // file exist, is it open?
    //     std::string cmd = "lsof " + teensy1.usbDevName;
    //     std::string s = exec(cmd);
    //     teensyFileFree = s.size() < 10;
    //     if (not teensyFileFree)
    //     {  // someone is using the Teensy, so the display is
    //       // used for something else
    //       if (not ipRemoved)
    //       {
    //         printf("Someone is using the Teensy (fine)\n");
    //         toLog("Teensy occupied (fine)");
    //       }
    //       ipRemoved = true;
    //     }
    //     else if (ipRemoved)
    //     {
    //       printf("Teensy is free again - sending my IP-list\n");
    //     }
    //   }
    //   else
    //   {
    //     if (not ipRemoved)
    //       printf("Someone unplugged the Teensy (no %s found)\n", teensy1.usbDevName.c_str());
    //     ipRemoved = true;
    //   }
    // }
    int n;
    std::string users = getUsers(&n);
    if (n != userCnt)
    { // number of users has changed, just note, for debug only
      toLog(users.c_str());
      printf("# number of users changed from %d to %d\n", userCnt, n);
      userCnt = n;
    }
    //
    // if (ip_list_changed)
    //   ip_sentToTeensy = false;
    // if (not ip_sentToTeensy or ipRemoved)
    // { // tell Teensy and show IP on display
    //   // but don't if a mission is running
    //   if (teensyFileFree)
    //   { // device present and no user
    //     toLog("Teensy available");
    //     printf("# Teensy available\n");
    //     teensy1.openToTeensy();
    //     if (teensy1.teensyConnectionOpen)
    //     {
    //       const int MSL = 200;
    //       char s[MSL];
    //       snprintf(s, MSL, "disp %s\n", ip4list);
    //       teensy1.send(s);
    //       // request also robot name
    //       usleep(2000);
    //       teensy1.send("idi\n");
    //       // allow communication to finish
    //       // handled by Teensy thread
    //       sleep(1);
    //       toLog("IP updated to Teensy display, stopping connection to Teensy");
    //       teensy1.stopUSB = true;
    //       ip_sentToTeensy = true;
    //       ip_list_changed = false;
    //       ipRemoved = false;
    //     }
    //   }
    // }
    // test evert 3 seconds only
    sleep(3);
    loop++;
  }
  tm = exec("date");
  toLog(tm.c_str());
  std::cout << tm << "ip_disp terminated\n";
}

bool UService::findIPs()
{
  struct ifaddrs * ifAddrStruct=NULL;
  struct ifaddrs * ifa=NULL;
  void * tmpAddrPtr=NULL;
  bool changed = false;
  //
  getifaddrs(&ifAddrStruct);
  ipsCnt = 0;
//   macCnt = 0;
  for (ifa = ifAddrStruct; ifa != NULL; ifa = ifa->ifa_next)
  { // all interface names and types
    if (ifa->ifa_addr->sa_family == AF_INET)
    { // is a valid IP4 Address
      tmpAddrPtr=&((struct sockaddr_in *)ifa->ifa_addr)->sin_addr;
      char addressBuffer[INET_ADDRSTRLEN];
      inet_ntop(AF_INET, tmpAddrPtr, addressBuffer, INET_ADDRSTRLEN);
      if (strcmp(ifa->ifa_name, "lo") != 0)
      { // not loop back - loopback is skipped
        int sl = strlen(addressBuffer);
        if (sl > 0 and strncmp(ips[ipsCnt], addressBuffer, sl) != 0)
        {
          changed = true;
          snprintf(ips[ipsCnt], MHL, "%s", addressBuffer);
          printf("#### found IP %d: %s %s\n", ipsCnt, ifa->ifa_name, ips[ipsCnt]);
        }
        if (ipsCnt < MIPS - 1)
          ipsCnt++;
      }
    }
  }
  return changed;
}

bool UService::updateIPlist()
{ // update IP list
  int n = 0;
  char * p1 = ip4list;
  bool changed = false;
  for (int i = 0; i < ipsCnt; i++)
  {
    snprintf(p1, MHL2 - n, " %s", ips[i]);
    n += strlen(p1);
    p1 = &ip4list[n];
  }
  // see if it has changed
  if (strcmp(ip4list, ip4listLast) != 0)
  { // make IP list message
    toLog(ip4list);
    changed = true;
    strncpy(ip4listLast, ip4list, MHL2);
  }
  return changed;
}

float UService::measureCPUtemp()
{
  FILE * temp;
  const char * tf = "/sys/class/thermal/thermal_zone0/temp";
  temp = fopen(tf, "r");
  float t = 0;
  if (temp != NULL)
  {
    const int MSL = 20;
    char s[MSL];
    char * p1 = s;
    int n = fread(p1, 1, 1, temp);
    int m = n;
    while (n > 0)
    {
      n = fread(++p1, 1, 1, temp);
      m += n;
    }
    s[m] = '\0';
    if (m > 3)
    {
      t = strtof(s, &p1);
    }
    //     printf("#got %d bytes (%g deg) as:%s\n", m, t/1000.0, s);
    fclose(temp);
  }
  return t/1000.0;
}

void UService::toLog(const char* message)
{
  if (logfile != nullptr)
  {
    log_mutex.lock();
    UTime t("now");
    fprintf(logfile, "%lu.%04ld %d %.1f %% %s\n", t.getSec(), t.getMicrosec()/100,
            userCnt,
            cpuTemp,
            message);
    log_mutex.unlock();
    fflush(logfile);
  }
}

std::string UService::exec(std::string cmd)
{
  // toLog(cmd.c_str());
  std::array<char, 100> buffer;
  std::string result;
  std::unique_ptr<FILE, decltype(&pclose) > pipe(popen(cmd.c_str(), "r"), pclose);
  if (!pipe) {
    throw std::runtime_error("popen() failed!");
  }
  while (fgets(buffer.data(), buffer.size(), pipe.get()) != nullptr) {
    result += buffer.data();
  }
  // toLog(result.c_str());
  return result;
}

int UService::isThisProcessRunning(std::string name)
{
  std::string cmd = "pgrep " + name;
  std::string s = exec(cmd);
  // printf("# isThisProcessRunning found '%s' times %s using %s\n", s.c_str(), name.c_str(), cmd.c_str());
  int cnt = 0;
  int pos = 0;
  while (true)
  {
    auto n = s.find('\n', pos);
    if (n == std::string::npos)
      break;
    else
    {
      pos = n+1;
      cnt ++;
    }
  }
  return cnt;
}


std::string UService::getUsers(int* userCnt)
{
  std::string users = exec("w");
  const char * p1 = strstr(users.c_str(), ", ");
  if (p1 != nullptr)
  {
    p1 += 2;
    int n = strtol(p1, nullptr, 10);
//     printf("Found %d users (in %s)\n", n, users.c_str());
    *userCnt = n;
  }
  else
    printf("Failed to find user count (%s)\n", users.c_str());
  return users;
}

void UService::saveRobotName(const char * newName)
{
  //   printf("# UTeensy::renameHostTest: hertil\n");
  const int MNL = 32;
  char nn[MNL];
  int nnCnt = 0;
//   const char p1 = newName;
  for (int i = 0; i < (int)strlen(newName); i++)
  { // use non white only
    if (newName[i] > ' ')
    { // and use lower case only
      nn[nnCnt++] = tolower(newName[i]);
    }
  }
  nn[nnCnt] = '\0';
  printf("UService::saveRobotName: saves '%s' to robotname\n", nn);
  FILE * sc;
  sc = fopen("robotname","w");
  fprintf(sc,"%s", nn);
  fclose(sc);
}

void UService::shutdownRequest()
{
  UTime t("now");
  // start logging (mostly debug)
  mqtt.publish("robobot/cmd/ti","log 1", &t, 1);
  // initiate shutdown procedure
  mqtt.publish("robobot/cmd/shutdown","GPIO button", &t, 1);
}

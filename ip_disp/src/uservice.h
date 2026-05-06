/* #***************************************************************************
 #*   Copyright (C) 2023 by DTU
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

#pragma once

#include <string>
#include <thread>
#include "utime.h"
#include <sys/stat.h>
#include <mutex>
#include "uini.h"

class UService
{
public:
    /**
     * Initialize all message data items */
    bool setup(int argc,char **argv);
    /**
     * decode messages from Teensy
     * \param msg already CRC checked text line from teensy */
    bool decode(const char* msg, UTime&);
    /**
     * Decode messages from MQTT
     * especially topic 'robobot/cmd/shutdown'
     * \param message is ignored,
     * \param time is ignored
     * \returns true if message is used. */
    bool mqttDecode(const char* topic, const char *, UTime&);
    /**
     * decode command-line parameters */
    bool readCommandLineParameters(int argc, char ** argv);
    /**
     * shut down and save ini-file */
    void terminate();
    /**
    * thread to listen to keyboard */
    void run();
    /**
     * Return the SVN version string (version part) */
    std::string getVersionString();
    /**
     * request shutdown (from GPIO) */
    void shutdownRequest();

public:
    // stop all processing
    bool stop = false;
    bool mission_app_running = false;
    // file with calibration values etc.
    std::string iniFileName = "robot.ini";
    mINI::INIFile * iniFile;
    bool stop_logging = false;
    std::string logPath = ""; // this directory
    FILE * logfile = nullptr;
    /**
     * Check is a file exist */
    inline bool file_exists(const std::string& name) {
        struct stat buffer;
        return (stat (name.c_str(), &buffer) == 0);
    }
    /**
     * run a bash command and get console output */
    std::string exec(std::string cmd);
    bool teensyFileFree = false;
    /**
     * Check number of times this name exist in the process list
     * \param name is the process name or part of a name to check.
     * \returns the number of times this mane occur in the process list */
    int isThisProcessRunning(std::string name);

private:
    /**
     * Find IP4 on this host */
    bool findIPs();
    /**
     * Find MAC - probably not used */
//     bool findWifiMACs();
//     bool check_wireless(const char* ifname, char* protocol);
    bool updateIPlist();
    std::string getUsers(int * userCnt);
    void saveRobotName(const char * newName);
    /**
     * variables for IP/MAC */
    int lastIpCnt = 0;
    static const int MHL = 100;
    char hostname[MHL];
    static const int MIPS = 7;
    char ips[MIPS][MHL] = {{'\0'}}; // list of IP strings
//     char macs[MIPS][MHL] = {{'\0'}}; // list of MAC strings
    int ipsCnt = 0;
//     int macCnt = 0;
    static const int MHL2 = 150;
    char ip4list[MHL2];
    char ip4listLast[MHL2];
    char maclist[MHL2];
    /**
     * Base */
    float measureCPUtemp();
    int userCnt = 0;
    float cpuTemp = 0.0;
    void toLog(const char* message);
    bool terminating = false;
    bool setupComplete = false;
    std::mutex log_mutex;

};

extern UService service;
extern mINI::INIStructure ini;

#ifndef LIBS_MAVIO_INCLUDE_SYSTEMMANAGER_H_
#define LIBS_MAVIO_INCLUDE_SYSTEMMANAGER_H_

#include "CircularBuffer.h"
#include "MAVLinkLib.h"
#include "timelib.h"
#include "Logger.h"

#include <atomic>
#include <string>
#include <thread>
#include <chrono>

#include <iostream>
#include <stdio.h>
#include <stdlib.h>

namespace mavio {

class SystemManager {
    public:

    SystemManager();
    ~SystemManager();

    bool init();

    void close();

    bool get_status_bitmask(uint8_t& bitmask);

    private:

    void process_task();

    std::string get_console_result(const char* cmd);

    uint32_t text_to_bin(char* octet);

    int32_t text_to_bin_check(char* octet);

    bool get_status_bitmask_from_response(std::string response);

    std::atomic<bool> _running;

    std::thread process_thread;

    timelib::Stopwatch timer_process_task;
    std::chrono::milliseconds period_process_task = timelib::sec2ms(2);
    std::chrono::milliseconds period_sleep = timelib::sec2ms(0.2);

    bool _last_reading_valid;
    
    // status bitmas, from rpi documentation of vcgencmd get_throttled:
    
    // 0 undervoltage detected
    // 1 arm frequency capped
    // 2 currently throttled
    // 3 soft temperature limit active
    // 16 undervoltage has ocurred
    // 17 arm frequency capping has ocurred
    // 18 throttling has ocurred
    // 19 soft temperature limit has ocurred
    
    // which are directly mapped in a the _status_bitmask byte.

    std::atomic<uint8_t> _status_bitmask;

};

} // namespace mavio

#endif // LIBS_MAVIO_INCLUDE_SYSTEMMANAGER
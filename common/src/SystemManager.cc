#include "SystemManager.h"

namespace mavio {

SystemManager::SystemManager() 
    : timer_process_task() {

    _running = false;
}

SystemManager::~SystemManager() {
    close();
}

bool SystemManager::init() {
    if (!_running) {
        _running = true;

        std::thread process_th(&SystemManager::process_task, this);
        process_thread.swap(process_th);
    }

    timer_process_task.reset();

    return true;
}

void SystemManager::close() {
    if (_running) {
        _running = false;

        process_thread.join();
    }
}

bool SystemManager::get_status_bitmask(uint8_t& bitmask) {
    if ( !_last_reading_valid ) {
        return false;
    } 
    
    bitmask = _status_bitmask;
    return true;
}

std::string SystemManager::get_console_result(const char* cmd) {
    FILE* pipe = popen(cmd, "r");

    if (!pipe) {
        return "ERROR";
    }

    char buffer[128];
    std::string result = "";

    while (!feof(pipe)) {
        if (fgets(buffer, 128, pipe) != NULL ) {
            result += buffer;
        }

    }

    pclose(pipe);
    return(result);
}

void SystemManager::process_task() {
    while (_running) {
        if (timer_process_task.elapsed_time() > period_process_task) {
            timer_process_task.reset();
            
            _last_reading_valid = false;
            _status_bitmask = 0;

            const char* command = "/opt/vc/bin/vcgencmd get_throttled";
            std::string result = get_console_result(command);

            if ( get_status_bitmask_from_response(result) ) {
                _last_reading_valid = true;
            }

        } else {

            timelib::sleep(period_sleep);
        }
    }
}

bool SystemManager::get_status_bitmask_from_response(std::string result) {
    uint32_t hex_response = 0;
    char prepared_answer[6];
    bool success = false;
    
    for ( size_t i = 0; i < result.size(); i++ ) {
        
        if ( result.c_str()[i] == 'x' ) {
            for ( int u = 0; u < 6; u++ ) {
                prepared_answer[u] = result.c_str()[i + u + 1];
                  // mavio::log(LOG_INFO, "prepared answer %d read: %c", u, prepared_answer[u]);
                if ( !isxdigit(  prepared_answer[u] )) {
                  // mavio::log(LOG_INFO, "prepared answer %d not digit: %c", u, prepared_answer[u]);
                  for ( int y = u; y < 6; y++ ) {
                    prepared_answer[y] = 48;
                  } 
                  break;
                }
            }
            // mavio::log(LOG_INFO, "out of loop");
            break;
        }
    }

    // mavio::log(LOG_INFO, "prepared answer: %s", prepared_answer);
    // mavio::log(LOG_INFO, "prepared answer size: %d", sizeof(prepared_answer));

    if ( text_to_bin_check(prepared_answer) >= 0 ) {
        
        hex_response = text_to_bin_check(prepared_answer);

        success = true;

        // transfer just the used bytes to our status bitmask byte
        // as the original answer are 20 bits

        if ( hex_response & (1 << 0) ) 
            _status_bitmask += 1; // undervoltage detected
        if ( hex_response & (1 << 1) ) 
            _status_bitmask += 2; // arm frecuency capped
        if ( hex_response & (1 << 2) ) 
            _status_bitmask += 4; // currently throttled
        if ( hex_response & (1 << 3) ) 
            _status_bitmask += 8; // soft temperature limit active
        if ( hex_response & (1 << 16) ) 
            _status_bitmask += 16; // undervoltage has ocurred
        if ( hex_response & (1 << 17) ) 
            _status_bitmask += 32; // arm frequency capping has ocurred
        if ( hex_response & (1 << 18) ) 
            _status_bitmask += 64; // throttling has ocurred
        if ( hex_response & (1 << 19) ) 
            _status_bitmask += 128; // soft temperature limit has ocurred
    }

    // just for debug
    // uint8_t prov_bitmask = _status_bitmask;
    // mavio::log(LOG_INFO, "hex: %x", hex_response);
    // mavio::log(LOG_INFO, "bitmask: %d", prov_bitmask);

    return success;
}

uint32_t SystemManager::text_to_bin(char* octet) /* converts an octet to a 8-Bit value */
{
  uint32_t result=0;

  if (octet[0]>57)
    result+=octet[0]-55;
  else
    result+=octet[0]-48;
  result=result<<4;
  if (octet[1]>57)
    result+=octet[1]-55;
  else
    result+=octet[1]-48;
  result=result<<4;
  if (octet[2]>57)
    result+=octet[2]-55;
  else
    result+=octet[2]-48;
  result=result<<4;
  if (octet[3]>57)
    result+=octet[3]-55;
  else
    result+=octet[3]-48;
  result=result<<4;
  if (octet[4]>57)
    result+=octet[4]-55;
  else
    result+=octet[4]-48;
  return result;
}

int32_t SystemManager::text_to_bin_check(char *octet)
{
  if (octet[0] == 0)
    return -1;
  if (octet[1] == 0)
    return -2;
  if (!isxdigit(octet[0]))
    return -3;
  if (!isxdigit(octet[1]))
    return -4;
  if (!isxdigit(octet[2]))
    return -5;
  if (!isxdigit(octet[3]))
    return -6;
  if (!isxdigit(octet[4]))
    return -7;
  return text_to_bin(octet);
}

} // namespace mavio
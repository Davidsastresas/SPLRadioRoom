#include "VehicleManager.h"
#include "MAVLinkLogger.h"
#include "Config.h"

namespace mavio {

constexpr size_t max_vehicle_manager_queue_size = 1024;
constexpr size_t max_vehicle_manager_queue_size_high_lat = 256;

const std::chrono::milliseconds vehicle_manager_sleep_interval(1);


VehicleManager::VehicleManager() 
    : _send_queue_tcp(max_vehicle_manager_queue_size),
      _receive_queue_tcp(max_vehicle_manager_queue_size),
      _send_queue_rfd(max_vehicle_manager_queue_size),
      _receive_queue_rfd(max_vehicle_manager_queue_size),
      _send_queue_gsm(max_vehicle_manager_queue_size_high_lat),
      _receive_queue_gsm(max_vehicle_manager_queue_size_high_lat),
      _send_queue_sbd(max_vehicle_manager_queue_size_high_lat),
      _receive_queue_sbd(max_vehicle_manager_queue_size_high_lat) {

    if (_singleton != nullptr) {
        // panic log message
        mavio::log(LOG_ERR,"Vehicle manager must be singleton!");
        return;
    }
    _singleton = this;

}

VehicleManager::~VehicleManager() {
    close();
}

void VehicleManager::close() {
    if (_running) {
        _running = false;

        receive_thread.join();
        send_thread.join();
    }
}

bool VehicleManager::init() {

    // vehicle 0
    for ( int i = 0; i < max_vehicles_allowed; i++ ) {
        if ( _vehicles[i].init( i + 1 ) ) {
           _vehicles_initialized++;

            mavio::log(LOG_INFO,"Vehicle %d initialized!", ( i + 1 ));
            mavio::log(LOG_INFO," ");

        } else {
            mavio::log(LOG_INFO,"Vehicle %d NOT initialized", ( i + 1 ));
        }
    }

    mavio::log(LOG_INFO, "Groundstation heartbeat_period: %d", radioroom::config.get_heartbeat_period() );
    mavio::log(LOG_INFO, "Groundstation rfd_timeout: %d", radioroom::config.get_rfd_timeout() );
    mavio::log(LOG_INFO, "Groundstation sms_timeout: %d", radioroom::config.get_sms_timeout() );
    mavio::log(LOG_INFO, "Groundstation sms_hearbeat_period: %d", radioroom::config.get_sms_heartbeat_period() );

    // init threads
    if (!_running) {
        _running = true;

        std::thread send_th(&VehicleManager::send_task, this);
        send_thread.swap(send_th);

        std::thread receive_th(&VehicleManager::receive_task, this);
        receive_thread.swap(receive_th);
    }

    return true;
}

bool VehicleManager::push_queue_tcp(const mavlink_message_t& msg) {
    _send_queue_tcp.push(msg);
    return true;
}

bool VehicleManager::pull_queue_tcp(mavlink_message_t& msg) {
    return _receive_queue_tcp.pop(msg);
}

bool VehicleManager::push_queue_rfd(const mavlink_message_t& msg) {
    _send_queue_rfd.push(msg);
    return true;
}

bool VehicleManager::pull_queue_rfd(mavlink_message_t& msg) {
    return _receive_queue_rfd.pop(msg);
}

bool VehicleManager::push_queue_gsm(const mavio::SMSmessage sms) {
    _send_queue_gsm.push(sms);
    return true;
}

bool VehicleManager::pull_queue_gsm(mavio::SMSmessage& sms) {
    return _receive_queue_gsm.pop(sms);
}

bool VehicleManager::push_queue_sbd(const mavio::SBDmessage& sbdmessage) {
    _send_queue_sbd.push(sbdmessage);
    return true;
}

bool VehicleManager::pull_queue_sbd(mavio::SBDmessage& sbdmessage) {
    return _receive_queue_sbd.pop(sbdmessage);
}


void VehicleManager::send_task() {
    while (_running) {
        bool sleeping = true;

        // update each vehicle, and if messages available, load to buffers
        for ( int i = 0; i < _vehicles_initialized; i++ ) {
            if ( !_vehicles[i].initialized() ) {
                continue;
            }

            _vehicles[i].update();

            mavlink_message_t mavmsg;

            // Here is where we extract the messages comming from the local TCPchannel (wifi short range)
            
            if ( _vehicles[i].pull_queue_tcp_cli(mavmsg) ) {
                sleeping = false;
                _receive_queue_tcp.push(mavmsg);
            }

            if ( _vehicles[i].pull_queue_mavmsg(mavmsg) ) {
                sleeping = false;
                _receive_queue_tcp.push(mavmsg);
            }

            mavio::SMSmessage smsmsg;

            if ( _vehicles[i].pull_queue_smsmsg(smsmsg) ) {
                sleeping = false;
                _receive_queue_gsm.push(smsmsg);
            }

            mavio::SBDmessage sbdmsg;

            if ( _vehicles[i].pull_queue_sbdmsg(sbdmsg) ) {
                sleeping = false;
                _receive_queue_sbd.push(sbdmsg);
            }
        }

        if (sleeping) {
            timelib::sleep(vehicle_manager_sleep_interval);    
        }
    }
}

void VehicleManager::receive_task() {
    while (_running) {
        bool sleeping = true;

        mavlink_message_t msg;
        int v_instance = 0;

        if ( _send_queue_tcp.pop(msg) ) {
            sleeping = false;

            for ( int i = 0; i < _vehicles_initialized; i++ ) {
                _vehicles[i].process_gcs_message(msg);
            }
        }

        if ( _send_queue_rfd.pop(msg) ) {
            sleeping = false;

            if ( get_vehicle_by_mav_id(msg.sysid, v_instance) ) {
                _vehicles[v_instance].process_message(msg);
            }
        }

        mavio::SMSmessage smsmessage;

        if ( _send_queue_gsm.pop(smsmessage) ) {
            sleeping = false;

            if ( get_vehicle_by_mav_id(smsmessage.get_mavlink_msg().sysid, v_instance)) {
                _vehicles[v_instance].process_message(smsmessage);
            }
        }

        mavio::SBDmessage sbdmessage;

        if ( _send_queue_sbd.pop(sbdmessage) ) {
            sleeping = false;

            if ( get_vehicle_by_mav_id(sbdmessage.get_mavlink_msg().sysid, v_instance)) {
                _vehicles[v_instance].process_message(sbdmessage);
            }
        }

        if (sleeping) {
            timelib::sleep(vehicle_manager_sleep_interval);
        }
    }

}

bool VehicleManager::get_vehicle_by_mav_id(int id, int& vehicleinstance) {
    if (!id) {

        return false;
    }

    for ( int i = 0; i < _vehicles_initialized; i++ ) {
        if ( _vehicles[i].get_mavid() == id ) {
            vehicleinstance = i;

            return true;
        }
    }

    return false;
}

VehicleManager *VehicleManager::_singleton;

} // namespace mavio
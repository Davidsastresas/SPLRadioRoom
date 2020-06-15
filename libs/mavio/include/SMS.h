/*
SMS.h

MAVIO MAVLink I/O library.

Original work Copyright (C) 2013-4 Mikal Hart
Modified work Copyright (C) 2017-9 Envirover

All rights reserved.

This library is free software; you can redistribute it and/or
modify it under the terms of the GNU Lesser General Public
License as published by the Free Software Foundation; either
version 2.1 of the License, or (at your option) any later version.

This library is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
Lesser General Public License for more details.

You should have received a copy of the GNU Lesser General Public
License along with this library; if not, write to the Free Software
Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
*/

#ifndef LIBS_MAVIO_INCLUDE_SMS_H_
#define LIBS_MAVIO_INCLUDE_SMS_H_

#include "Serial.h"

#include <chrono>

#define GSM_SUCCESS 0
#define GSM_ALREADY_AWAKE 1
#define GSM_SERIAL_FAILURE 2
#define GSM_PROTOCOL_ERROR 3
#define GSM_CANCELLED 4
#define GSM_NO_MODEM_DETECTED 5
#define GSM_SBDIX_FATAL_ERROR 6
#define GSM_SENDRECEIVE_TIMEOUT 7
#define GSM_RX_OVERFLOW 8
#define GSM_REENTRANT 9
#define GSM_IS_ASLEEP 10
#define GSM_NO_SLEEP_PIN 11
#define GSM_INBOX_EMPTY 12
#define GSM_MSG_TO_SEND_EMPTY 13

#define GSM_MAX_MT_MGS_SIZE 270

namespace mavio {

/**
 * POSIX implementation for Iridium SBD transceiver communication.
 */
class SMS {
 public:
  SMS(Serial& serial);

  int begin();

  int getTransceiverModel(char* buffer, size_t bufferSize);
  int getTransceiverSerialNumber(char* buffer, size_t bufferSize);
  int sendSBDText(const char* message);
  int sendSBDBinary(const uint8_t* txData, size_t txDataSize);
  int sendReceiveSBDText(const char* message, uint8_t* rxBuffer,
                         size_t& rxBufferSize);
  int sendReceiveSBDBinary(const uint8_t* txData, size_t txDataSize,
                           uint8_t* rxBuffer, size_t& rxBufferSize);

  int sendSMSBinary(const uint8_t* txData, size_t txDataSize);
  
  int receiveSMSBinary(uint8_t* rxBuffer, size_t& rxBufferSize, bool& inbox_empty);

  int deleteSMSlist(void);

  int readSMSlist(void);
  
  int getSignalQuality(int& quality);
  int queryRingIndicationStatus(int& sri);

  // This command returns current state of the mobile originated and mobile
  // terminated buffers, and the SBD ring alert status.
  int getStatusExtended(uint16_t& moFlag, uint16_t& moMSN, uint16_t& mtFlag,
                        uint16_t& mtMSN, uint16_t& raFlag,
                        uint16_t& msgWaiting);

  int getWaitingMessageCount();
  int sleep();
  bool isAsleep();

  void adjustATTimeout(std::chrono::milliseconds ms);  // default value = 20 seconds
  void adjustSendReceiveTimeout(std::chrono::milliseconds ms);  // default value = 300 seconds
  void setMinimumSignalQuality(
      int quality);  // a number between 1 and 5, default
                     // ISBD_DEFAULT_CSQ_MINIMUM
  void useMSSTMWorkaround(
      bool useWorkAround);  // true to use workaround from Iridium Alert 5/7

  struct sms_struct {
    uint8_t smsc_size;
    uint8_t type_address;
    uint8_t service_center_number[16]; // for 15 digit numbers maximum as international format
    uint8_t first_byte = 0x04;
    uint8_t address_lenght;
    uint8_t address_type;
    uint8_t sender_number[16]; // 15 digits maximum international format
    uint8_t TP_PID;
    uint8_t TO_DCS;
    uint8_t timestamp[15];
    uint8_t data_lenght;
    uint8_t data[160];
  };

 private:
  // Internal utilities
  bool smartWait(std::chrono::milliseconds ms);
  bool waitForATResponse(char* response = NULL, int responseSize = 0,
                         const char* prompt = NULL,
                         const char* terminator = "OK\r\n");

  bool waitForATResponseDebug(char* response = NULL, int responseSize = 0,
                         const char* prompt = NULL,
                         const char* terminator = "OK\r\n");
  
  bool waitForATResponseDebug(char* response = NULL, int responseSize = 0,
                         const char* prompt = NULL,
                         const char* terminator = "OK\r\n");
  
  bool waitForATResponseCSQ(char* response = NULL, int responseSize = 0,
                         const char* prompt = NULL,
                         const char* terminator = "OK\r\n");

  bool waitforSMSlist(uint8_t* response, size_t& responseSize, bool& inbox_empty);

  int internalBegin();
  int internalGetTransceiverModel(char* buffer, size_t bufferSize);
  int internalGetTransceiverSerialNumber(char* buffer, size_t bufferSize);
  int internalSendReceiveSBD(const char* txTxtMessage, const uint8_t* txData,
                             size_t txDataSize, uint8_t* rxBuffer,
                             size_t* prxBufferSize);
  int internalQueryRingIndicationStatus(int& sri);
  int internalGetStatusExtended(uint16_t& moFlag, uint16_t& moMSN,
                                uint16_t& mtFlag, uint16_t& mtMSN,
                                uint16_t& raFlag, uint16_t& msgWaiting);
  int internalGetSignalQuality(int& quality);
  int internalMSSTMWorkaround(bool& okToProceed);
  int internalSleep();

  int internalsendSMSBinary(const uint8_t* txData, size_t txDataSize);
  
  int internalreceiveSMSBinary(uint8_t* rxBuffer, size_t& rxBufferSize, bool& inbox_empty);

  int internalreadSMSlist(void);

  int internaldeleteSMSlist(void);

  int doSBDIX(uint16_t& moCode, uint16_t& moMSN, uint16_t& mtCode,
              uint16_t& mtMSN, uint16_t& mtLen, uint16_t& mtRemaining);
  int doSBDRB(uint8_t* rxBuffer, size_t* prxBufferSize);  // in/out
  int readUInt(uint16_t& u);
  void power(bool on);

  void send(const char* str);
  void send(uint16_t n);

  bool cancelled();

  int octet2bin(char* octet);    
  int octet2bin_check(char* octet);  
  void binary2pdu(char* binary, int lenght, char* pdu);

  Serial& stream;  // Communicating with the Iridium

  // Timings milliseconds
  std::chrono::milliseconds csqInterval;
  std::chrono::milliseconds sbdixInterval;
  std::chrono::milliseconds atTimeout;
  std::chrono::milliseconds sendReceiveTimeout;

  // State variables
  int remainingMessages;
  int sleepPin;
  bool asleep;
  bool reentrant;
  int minimumCSQ;
  bool useWorkaround;
  unsigned long lastPowerOnTime;

  sms_struct buffersms;
};

}  // namespace mavio

#endif  // LIBS_MAVIO_INCLUDE_SMS_H_
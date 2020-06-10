/*
SMS.cc

Original work Copyright (C) 2013-4 Mikal Hart
Modified work Copyright (C) 2017   Envirover

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

#include "SMS.h"

#include <limits.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include <iostream>

#include <Logger.h>
#include "timelib.h"

using timelib::Stopwatch;

namespace mavio {

#define GSM_LIBRARY_REVISION 2
#define GSM_DEFAULT_CSQ_MINIMUM 2

const std::chrono::milliseconds GSM_startup_timeout(500);
const std::chrono::milliseconds GSM_default_at_timeout(4000);
const std::chrono::milliseconds GSM_default_csq_interval(10000);
const std::chrono::milliseconds GSM_default_csq_interval_usb(20000);
const std::chrono::milliseconds GSM_default_sbdix_interval(30000);
const std::chrono::milliseconds GSM_default_sbdix_interval_usb(30000);
const std::chrono::milliseconds GSM_default_sendreceive_time(60000);
const std::chrono::milliseconds GSM_startup_max_time(10000);
const std::chrono::milliseconds smart_wait_sleep(1);

extern bool isbdCallback() __attribute__((weak));

#define UNUSED(x) (void)(x)

bool isbdCallback() { return true; }

SMS::SMS(Serial& serial)
    : stream(serial),
      csqInterval(GSM_default_csq_interval),
      sbdixInterval(GSM_default_sbdix_interval),
      atTimeout(GSM_default_at_timeout),
      sendReceiveTimeout(GSM_default_sendreceive_time),
      remainingMessages(-1),
      sleepPin(-1),
      asleep(false),
      reentrant(false),
      minimumCSQ(GSM_DEFAULT_CSQ_MINIMUM),
      useWorkaround(true),
      lastPowerOnTime(0UL) {}

// Power on the RockBLOCK or return from sleep
int SMS::begin() {
  if (this->reentrant) {
    return GSM_REENTRANT;
  }

  this->reentrant = true;
  int ret = internalBegin();
  this->reentrant = false;

  // Absent a successful startup, keep the device turned off
  if (ret != GSM_SUCCESS) {
    power(false);
  }

  return ret;
}

int SMS::getTransceiverModel(char* buffer, size_t bufferSize) {
  if (this->reentrant) {
    return GSM_REENTRANT;
  }

  this->reentrant = true;
  int ret = internalGetTransceiverModel(buffer, bufferSize);
  this->reentrant = false;

  return ret;
}

int SMS::getTransceiverSerialNumber(char* buffer, size_t bufferSize) {
  if (this->reentrant) {
    return GSM_REENTRANT;
  }

  this->reentrant = true;
  int ret = internalGetTransceiverSerialNumber(buffer, bufferSize);
  this->reentrant = false;

  return ret;
}

// Transmit a binary message
int SMS::sendSBDBinary(const uint8_t* txData, size_t txDataSize) {
  if (this->reentrant) {
    return GSM_REENTRANT;
  }

  this->reentrant = true;
  int ret = internalSendReceiveSBD(NULL, txData, txDataSize, NULL, NULL);
  this->reentrant = false;
  return ret;
}

// Transmit and receive a binary message
int SMS::sendReceiveSBDBinary(const uint8_t* txData, size_t txDataSize,
                                     uint8_t* rxBuffer, size_t& rxBufferSize) {
  if (this->reentrant) {
    return GSM_REENTRANT;
  }

  this->reentrant = true;
  int ret =
      internalSendReceiveSBD(NULL, txData, txDataSize, rxBuffer, &rxBufferSize);
  this->reentrant = false;
  return ret;
}

// Transmit and receive a binary message
int SMS::sendSMSBinary(const uint8_t* txData, size_t txDataSize) {
  if (this->reentrant) {
    return GSM_REENTRANT;
  }

  this->reentrant = true;
  int ret = internalsendSMSBinary(txData, txDataSize);
  this->reentrant = false;
  return ret;
}

// Transmit and receive a binary message
int SMS::receiveSMSBinary(uint8_t* rxBuffer, size_t& rxBufferSize) {
  if (this->reentrant) {
    return GSM_REENTRANT;
  }

  this->reentrant = true;
  int ret = internalreceiveSMSBinary(rxBuffer, rxBufferSize);
  this->reentrant = false;
  return ret;
}

// Transmit and receive a binary message
int SMS::readSMSlist() {
  if (this->reentrant) {
    return GSM_REENTRANT;
  }

  this->reentrant = true;
  int ret = internalreadSMSlist();
  this->reentrant = false;
  return ret;
}



// Transmit a text message
int SMS::sendSBDText(const char* message) {
  if (this->reentrant) {
    return GSM_REENTRANT;
  }

  this->reentrant = true;
  int ret = internalSendReceiveSBD(message, NULL, 0, NULL, NULL);
  this->reentrant = false;
  return ret;
}

// Transmit a text message and receive reply
int SMS::sendReceiveSBDText(const char* message, uint8_t* rxBuffer,
                                   size_t& rxBufferSize) {
  if (this->reentrant) {
    return GSM_REENTRANT;
  }

  this->reentrant = true;
  int ret = internalSendReceiveSBD(message, NULL, 0, rxBuffer, &rxBufferSize);
  this->reentrant = false;
  return ret;
}

// High-level wrapper for AT+CSQ
int SMS::getSignalQuality(int& quality) {
  if (this->reentrant) {
    return GSM_REENTRANT;
  }

  this->reentrant = true;
  int ret = internalGetSignalQuality(quality);
  this->reentrant = false;
  return ret;
}

// Query ring indication status
int SMS::queryRingIndicationStatus(int& sri) {
  if (this->reentrant) {
    return GSM_REENTRANT;
  }

  this->reentrant = true;
  int ret = internalQueryRingIndicationStatus(sri);
  this->reentrant = false;

  return ret;
}

int SMS::getStatusExtended(uint16_t& moFlag, uint16_t& moMSN,
                                  uint16_t& mtFlag, uint16_t& mtMSN,
                                  uint16_t& raFlag, uint16_t& msgWaiting) {
  if (this->reentrant) {
    return GSM_REENTRANT;
  }

  this->reentrant = true;
  int ret = internalGetStatusExtended(moFlag, moMSN, mtFlag, mtMSN, raFlag,
                                      msgWaiting);
  this->reentrant = false;

  return ret;
}

// Gracefully put device to lower power mode (if sleep pin provided)
int SMS::sleep() {
  if (this->reentrant) {
    return GSM_REENTRANT;
  }

  if (this->sleepPin == -1) {
    return GSM_NO_SLEEP_PIN;
  }

  this->reentrant = true;
  int ret = internalSleep();
  this->reentrant = false;

  if (ret == GSM_SUCCESS) {
    power(false);  // power off
  }
  return ret;
}

// Return sleep state
bool SMS::isAsleep() { return this->asleep; }

// Return number of pending messages
int SMS::getWaitingMessageCount() { return this->remainingMessages; }

// Tweak AT timeout
void SMS::adjustATTimeout(std::chrono::milliseconds ms) {
  this->atTimeout = ms;
}

// Tweak Send/Receive SBDIX process timeout
void SMS::adjustSendReceiveTimeout(std::chrono::milliseconds ms) {
  this->sendReceiveTimeout = ms;
}

// a number between 1 and 5, default GSM_DEFAULT_CSQ_MINIMUM
void SMS::setMinimumSignalQuality(int quality) {
  if (quality >= 1 && quality <= 5) {
    this->minimumCSQ = quality;
  }
}

// true to use workaround from Iridium Alert 5/7
void SMS::useMSSTMWorkaround(bool useWorkAround) {
  this->useWorkaround = useWorkAround;
}

/*
Private interface
*/

int SMS::internalBegin() {
  // diag << "Calling internalBegin\n";

  // if (!this->asleep) {
  //   mavio::log(LOG_INFO, "already awake");
  //   return GSM_ALREADY_AWAKE;
  // }

  power(true);  // power on

  bool modemAlive = false;

  Stopwatch timer;
  while (timer.elapsed_time() < GSM_startup_timeout)
    if (cancelled()) {
      return GSM_CANCELLED;
    }

  // Turn on modem and wait for a response from "AT" command to begin
  timer.reset();
  while (timer.elapsed_time() < GSM_startup_max_time && !modemAlive) {
    send("AT\r");
    modemAlive = waitForATResponse();
    if (cancelled()) {
      return GSM_CANCELLED;
    }
  }

  if (!modemAlive) {
    // diag << "No modem detected.\r\n";
    return GSM_NO_MODEM_DETECTED;
  }

  const char* pinstr = "AT+CPIN=\"3621\"\r";
  const char* askpinstr = "AT+CPIN?\r";
  const char* pinready = "+CPIN: READY";
  const char* pinneeded = "+CPIN: SIM PIN";
  
  char buffer[256];

  // Ask if pin is already inserted
  send(askpinstr);
  if (!waitForATResponse(buffer, sizeof(buffer), "AT+CPIN?\r\r\n", "OK")) {
    return false;
  }

  // Check if pin was already inserted or not. If not, insert it
  if ( !strcmp(buffer, pinready) ) {

    mavio::log(LOG_NOTICE, "GSM: pin already inserted");

  } else if ( !strcmp(buffer, pinneeded ) ) {

    send(pinstr);
    if (!waitForATResponse()) {
      return cancelled() ? GSM_CANCELLED : GSM_PROTOCOL_ERROR;
    }

    mavio::log(LOG_NOTICE, "GSM: pin setup succesfully");

  } else {

    mavio::log(LOG_NOTICE, "GSM: problem initializing gsm pin");
  }

  // maybe we need here bin instead
  // AT+CGSMS=1 for using gsm over gprs, although gsm is default 
  // random setup at commands TODO send proper sms encoding
  const char* strings[2] = {"AT+CMGF=0\r", "AT+CMEE=1\r"};
  for (int i = 0; i < 2; ++i) {
    send(strings[i]);
    if (!waitForATResponse()) {
      return cancelled() ? GSM_CANCELLED : GSM_PROTOCOL_ERROR;
    }
  }
  // diag << "InternalBegin: success!\n";
  return GSM_SUCCESS;
}

int SMS::internalGetTransceiverModel(char* buffer, size_t bufferSize) {
  if (this->asleep) {
    return GSM_IS_ASLEEP;
  }

  send("AT+CGMM\r");

  if (!waitForATResponse(buffer, bufferSize, "AT+CGMM\r\r\n", "OK")) {
    return cancelled() ? GSM_CANCELLED : GSM_PROTOCOL_ERROR;
  }

  return GSM_SUCCESS;
}

int SMS::internalGetTransceiverSerialNumber(char* buffer,
                                                   size_t bufferSize) {
  if (this->asleep) {
    return GSM_IS_ASLEEP;
  }

  send("AT+CGSN\r");

  if (!waitForATResponse(buffer, bufferSize, "AT+CGSN\r\r\n", "OK")) {
    return cancelled() ? GSM_CANCELLED : GSM_PROTOCOL_ERROR;
  }

  return GSM_SUCCESS;
}

int SMS::internalSendReceiveSBD(const char* txTxtMessage,
                                       const uint8_t* txData, size_t txDataSize,
                                       uint8_t* rxBuffer,
                                       size_t* prxBufferSize) {
  // diag << "internalSendReceive\n";

  if (this->asleep) {
    return GSM_IS_ASLEEP;
  }

  if (!txTxtMessage && !txDataSize) {  // Just receive, clear MO message buffer
    // send("AT+SBDD0\r");
    // if (!waitForATResponse(NULL, 0, NULL, "\r")) {
    //   return cancelled() ? GSM_CANCELLED : GSM_PROTOCOL_ERROR;
    // }
    return GSM_CANCELLED;

  } else if (txData && txDataSize) {  // Binary transmission?
    send("AT+SBDWB=");
    send(txDataSize);
    send("\r");
    if (!waitForATResponse(NULL, 0, NULL, "READY\r\n")) {
      return cancelled() ? GSM_CANCELLED : GSM_PROTOCOL_ERROR;
    }

    uint16_t checksum = 0;
    for (size_t i = 0; i < txDataSize; ++i) {
      stream.write(txData[i]);
      checksum += (uint16_t)txData[i];
    }

    // cons << "[" << txDataSize << " bytes]";

    // diag << "Checksum:" << checksum << "\n";

    stream.write(checksum >> 8);
    stream.write(checksum & 0xFF);

    if (!waitForATResponse(NULL, 0, NULL, "0\r\n\r\nOK\r\n")) {
      return cancelled() ? GSM_CANCELLED : GSM_PROTOCOL_ERROR;
    }
  } else {  // Text transmission
    send("AT+SBDWT=");
    if (txTxtMessage) {  // It's ok to have a NULL txtTxtMessage if the
                         // transaction is RX only
      send(txTxtMessage);
    }
    send("\r");
    if (!waitForATResponse(NULL)) {
      return cancelled() ? GSM_CANCELLED : GSM_PROTOCOL_ERROR;
    }
  }

  // Long SBDIX loop begins here
  Stopwatch timer;
  while (timer.elapsed_time() < GSM_default_sendreceive_time) {
    int strength = 0;
    bool okToProceed = true;
    int ret = internalGetSignalQuality(strength);
    if (ret != GSM_SUCCESS) {
      return ret;
    }

    mavio::log(LOG_INFO, "SMS signal quality: %d", strength);

    if (useWorkaround && strength >= minimumCSQ) {
      okToProceed = false;
      ret = internalMSSTMWorkaround(okToProceed);
      if (ret != GSM_SUCCESS) {
        return ret;
      }
    }

    if (okToProceed && strength >= minimumCSQ) {
      uint16_t moCode = 0, moMSN = 0, mtCode = 0, mtMSN = 0, mtLen = 0,
               mtRemaining = 0;
      ret = doSBDIX(moCode, moMSN, mtCode, mtMSN, mtLen, mtRemaining);
      if (ret != GSM_SUCCESS) {
        return ret;
      }

      // diag << "SBDIX MO code: " << moCode << "\n";

      if (moCode <= 4) {  // successful return!
        // diag << "SBDIX success!\n";

        this->remainingMessages = mtRemaining;
        if (mtCode == 1 && rxBuffer) {  // retrieved 1 message
          // diag << "Incoming message!\n";
          return doSBDRB(rxBuffer, prxBufferSize);
        } else {
          // No data returned
          if (prxBufferSize) {
            *prxBufferSize = 0;
          }
        }
        return GSM_SUCCESS;
      } else if (moCode == 12 || moCode == 14 || moCode == 16) {
        // fatal failure: no retry
        // diag << "SBDIX fatal!\n";
        return GSM_SBDIX_FATAL_ERROR;
      } else {  // retry
        // diag << "Waiting for SBDIX retry...\n";
        if (!smartWait(sbdixInterval)) {
          return GSM_CANCELLED;
        }
      }
    } else {  // signal strength == 0
      // diag << "Waiting for CSQ retry...\n";
      if (!smartWait(csqInterval)) {
        return GSM_CANCELLED;
      }
    }
  }  // big wait loop

  // diag << "SBDIX timeout!\n";
  return GSM_SENDRECEIVE_TIMEOUT;
}

int SMS::internalQueryRingIndicationStatus(int& sri) {
  if (this->asleep) {
    return GSM_IS_ASLEEP;
  }

  send("AT+CRIS\r");

  char crisResponseBuf[8];

  if (!waitForATResponse(crisResponseBuf, sizeof(crisResponseBuf), "+CRIS:")) {
    return cancelled() ? GSM_CANCELLED : GSM_PROTOCOL_ERROR;
  }

  if (strlen(crisResponseBuf) == 7) {
    sri = atoi(crisResponseBuf + 4);
    return GSM_SUCCESS;
  }

  return GSM_PROTOCOL_ERROR;
}

int SMS::internalGetStatusExtended(uint16_t& moFlag, uint16_t& moMSN,
                                          uint16_t& mtFlag, uint16_t& mtMSN,
                                          uint16_t& raFlag,
                                          uint16_t& msgWaiting) {
  if (this->asleep) {
    return GSM_IS_ASLEEP;
  }

  send("AT+SBDSX\r");

  char sbdsxResponseBuf[34];

  if (!waitForATResponse(sbdsxResponseBuf, sizeof(sbdsxResponseBuf),
                         "+SBDSX:")) {
    return cancelled() ? GSM_CANCELLED : GSM_PROTOCOL_ERROR;
  }

  uint16_t* values[6] = {&moFlag, &moMSN,  &mtFlag,
                         &mtMSN,  &raFlag, &msgWaiting};

  for (int i = 0; i < 6; ++i) {
    char* p = strtok(i == 0 ? sbdsxResponseBuf : NULL, ", ");
    if (p == NULL) {
      return GSM_PROTOCOL_ERROR;
    }
    *values[i] = atol(p);
  }

  return GSM_SUCCESS;
}

int SMS::internalGetSignalQuality(int& quality) {
  if (this->asleep) {
    return GSM_IS_ASLEEP;
  }

  send("AT+CSQ\r");

  char Buffer[3];

  if (!waitForATResponseCSQ(Buffer, sizeof(Buffer), "+CSQ:")) {
    return cancelled() ? GSM_CANCELLED : GSM_PROTOCOL_ERROR;
  }

  if ( strlen(Buffer) == 2 ) {
    quality = atoi(Buffer);
    if ( quality == 99 ) {
      quality = 0;
    }
    return GSM_SUCCESS;
  }
  
  return GSM_PROTOCOL_ERROR;
}


int SMS::internalMSSTMWorkaround(bool& okToProceed) {
  /*
  According to Iridium 9602 Product Bulletin of 7 May 2013, to overcome a
  system erratum:

  "Before attempting any of the following commands: +SBDDET, +SBDREG, +SBDI,
  +SBDIX, +SBDIXA the field application should issue the AT command �MSSTM to
  the transceiver and evaluate the response to determine if it is valid or
  not:

  Valid Response: "---MSSTM: XXXXXXXX" where XXXXXXXX is an eight---digit
  hexadecimal number.

  Invalid Response: "---MSSTM: no network service"

  If the response is invalid, the field application should wait and recheck
  system time until a valid response is obtained before proceeding.

  This will ensure that the Iridium SBD transceiver has received a valid
  system time before attempting SBD communication. The Iridium SBD transceiver
  will receive the valid system time from the Iridium network when it has a
  good link to the satellite. Ensuring that the received signal strength
  reported in response to AT command +CSQ and +CIER is above 2---3 bars before
  attempting SBD communication will protect against lockout.
  */
  char msstmResponseBuf[24];

  send("AT-MSSTM\r");
  if (!waitForATResponse(msstmResponseBuf, sizeof(msstmResponseBuf),
                         "-MSSTM: ")) {
    return cancelled() ? GSM_CANCELLED : GSM_PROTOCOL_ERROR;
  }

  // Response buf now contains either an 8-digit number or the string "no
  // network service"
  okToProceed = isxdigit(msstmResponseBuf[0]);
  return GSM_SUCCESS;
}

int SMS::internalSleep() {
  if (this->asleep) {
    return GSM_IS_ASLEEP;
  }

  // Best Practices Guide suggests this before shutdown
  send("AT*F\r");

  if (!waitForATResponse()) {
    return cancelled() ? GSM_CANCELLED : GSM_PROTOCOL_ERROR;
  }

  return GSM_SUCCESS;
}

bool SMS::smartWait(std::chrono::milliseconds ms) {
  Stopwatch timer;
  while (timer.elapsed_time() < ms) {
    if (cancelled()) {
      return false;
    }

    timelib::sleep(smart_wait_sleep);  // sleep for 1 millisecond
  }

  return true;
}

// Wait for response from previous AT command.  This process terminates when
// "terminator" string is seen or upon timeout. If "prompt" string is provided
// (example "+CSQ:"), then all characters following prompt up to the next CRLF
// are stored in response buffer for later parsing by caller.
bool SMS::waitForATResponse(char* response, int responseSize,
                                   const char* prompt, const char* terminator) {
  // diag << "Waiting for response " << terminator << "\n";

  if (response) {
    memset(response, 0, responseSize);
  }

  // bool done = false;
  int matchPromptPos = 0;      // Matches chars in prompt
  int matchTerminatorPos = 0;  // Matches chars in terminator

  enum { LOOKING_FOR_PROMPT, GATHERING_RESPONSE, LOOKING_FOR_TERMINATOR };

  int promptState = prompt ? LOOKING_FOR_PROMPT : LOOKING_FOR_TERMINATOR;

  Stopwatch timer;
  while (timer.elapsed_time() < atTimeout) {
    if (cancelled()) {
      return false;
    }

    int cc = stream.read();
    if (cc >= 0) {
      char c = cc;

      char ca[1];
      ca[0] = c; 

      mavio::log(LOG_INFO, "rd: %s", ca);

      if (prompt) {
        switch (promptState) {
          case LOOKING_FOR_PROMPT:
            if (c == prompt[matchPromptPos]) {
              ++matchPromptPos;
              if (prompt[matchPromptPos] == '\0') {
                promptState = GATHERING_RESPONSE;
                mavio::log(LOG_INFO, "gathering response");
              }
            } else {
              matchPromptPos = c == prompt[0] ? 1 : 0;
            }

            break;
          case GATHERING_RESPONSE:  // gathering reponse from end of prompt
                                    // to first \r
            if (response) {
              if (c == '\r' || responseSize < 2) {
                promptState = LOOKING_FOR_TERMINATOR;
                mavio::log(LOG_INFO, "looking for terminator");
              } else {
                *response++ = c;
                responseSize--;
                mavio::log(LOG_INFO, "response kept");
              }
            }
            break;
        }  // switch
      }    // prompt

      if (c == terminator[matchTerminatorPos]) {
        mavio::log(LOG_INFO, "terminator matches");
        ++matchTerminatorPos;
        if (terminator[matchTerminatorPos] == '\0') {
          mavio::log(LOG_INFO, "terminator finito");
          return true;
        }
      } else {
        matchTerminatorPos = c == terminator[0] ? 1 : 0;
      }
    }  // if (cc >= 0)
  }    // timer loop
  mavio::log(LOG_INFO, "timer out");

  return false;
}

bool SMS::waitForATResponseCSQ(char* response, int responseSize,
                                   const char* prompt, const char* terminator) {

  if (response) {
    memset(response, 0, responseSize);
  }

  int matchPromptPos = 0;      // Matches chars in prompt
  int matchTerminatorPos = 0;  // Matches chars in terminator

  enum { LOOKING_FOR_PROMPT, GATHERING_RESPONSE, LOOKING_FOR_TERMINATOR };

  int promptState = prompt ? LOOKING_FOR_PROMPT : LOOKING_FOR_TERMINATOR;

  Stopwatch timer;
  while (timer.elapsed_time() < atTimeout) {
    if (cancelled()) {
      return false;
    }

    int cc = stream.read();
    if (cc >= 0) {
      char c = cc;

      char ca[1];
      ca[0] = c; 
      mavio::log(LOG_INFO, "rd: %s", ca);

      if (prompt) {
        switch (promptState) {
          case LOOKING_FOR_PROMPT:
            if (c == prompt[matchPromptPos]) {
              ++matchPromptPos;
              if (prompt[matchPromptPos] == '\0') {
                promptState = GATHERING_RESPONSE;
                mavio::log(LOG_INFO, "gathering response");
              }
            } else {
              matchPromptPos = c == prompt[0] ? 1 : 0;
            }

            break;
          case GATHERING_RESPONSE:  // gathering reponse from end of prompt
                                    // to first \r
            if (response) {
              if (c == '\r' || responseSize < 2) {
                promptState = LOOKING_FOR_TERMINATOR;
                 mavio::log(LOG_INFO, "looking for terminator");
              } else {
                if ( c != ' ') {
                  *response++ = c;
                  responseSize--;
                }
              }
            }
            break;
        }  // switch
      }    // prompt

      if (c == terminator[matchTerminatorPos]) {
        ++matchTerminatorPos;
        if (terminator[matchTerminatorPos] == '\0') {
          return true;
        }
      } else {
        matchTerminatorPos = c == terminator[0] ? 1 : 0;
      }
    }  // if (cc >= 0)
  }    // timer loop

  return false;
}

bool SMS::cancelled() {
  if (isbdCallback != NULL) {
    return !isbdCallback();
  }

  return false;
}

int SMS::doSBDIX(uint16_t& moCode, uint16_t& moMSN, uint16_t& mtCode,
                        uint16_t& mtMSN, uint16_t& mtLen,
                        uint16_t& mtRemaining) {
  // xx, xxxxx, xx, xxxxx, xx, xxx
  char sbdixResponseBuf[32];
  send("AT+SBDIX\r");
  if (!waitForATResponse(sbdixResponseBuf, sizeof(sbdixResponseBuf),
                         "+SBDIX: ")) {
    return cancelled() ? GSM_CANCELLED : GSM_PROTOCOL_ERROR;
  }

  uint16_t* values[6] = {&moCode, &moMSN, &mtCode,
                         &mtMSN,  &mtLen, &mtRemaining};
  for (int i = 0; i < 6; ++i) {
    char* p = strtok(i == 0 ? sbdixResponseBuf : NULL, ", ");
    if (p == NULL) {
      return GSM_PROTOCOL_ERROR;
    }
    *values[i] = atol(p);
  }

  return GSM_SUCCESS;
}

int SMS::doSBDRB(uint8_t* rxBuffer, size_t* prxBufferSize) {
  bool rxOverflow = false;

  send("AT+SBDRB\r");
  if (!waitForATResponse(NULL, 0, NULL,
                         "AT+SBDRB\r")) {  // waits for its own echo
    return cancelled() ? GSM_CANCELLED : GSM_PROTOCOL_ERROR;
  }

  // Time to read the binary data: size[2], body[size], checksum[2]
  uint16_t size;
  int ret = readUInt(size);
  if (ret != GSM_SUCCESS) {
    return ret;
  }

  // cons << "[Binary size:" << size << "]";

  Stopwatch timer;

  for (uint16_t bytesRead = 0; bytesRead < size;) {
    if (cancelled()) {
      return GSM_CANCELLED;
    }

    int cc = stream.read();

    if (cc >= 0) {
      bytesRead++;

      if (rxBuffer && prxBufferSize) {
        if (*prxBufferSize > 0) {
          // cons << (char)cc;
          *rxBuffer++ = cc;
          (*prxBufferSize)--;
        } else {
          rxOverflow = true;
        }
      }
    }

    if (timer.elapsed_time() >= atTimeout) {
      return GSM_SENDRECEIVE_TIMEOUT;
    }
  }

  uint16_t checksum;
  ret = readUInt(checksum);
  if (ret != GSM_SUCCESS) {
    return ret;
  }

  // cons << "[csum:" << checksum << "]";

  // Return actual size of returned buffer
  if (prxBufferSize) {
    *prxBufferSize = (size_t)size;
  }

  return rxOverflow ? GSM_RX_OVERFLOW : GSM_SUCCESS;
}

int SMS::readUInt(uint16_t& u) {
  int sbuf[2];
  int n = 0;
  u = 0;

  Stopwatch timer;
  while (timer.elapsed_time() < atTimeout && n < 2) {
    if (cancelled()) {
      return GSM_CANCELLED;
    }

    sbuf[n] = stream.read();

    if (sbuf[n] >= 0) {
      n++;
    }
  }

  if (n < 2) {
    return GSM_SENDRECEIVE_TIMEOUT;
  }

  u = 256 * sbuf[0] + sbuf[1];

  return GSM_SUCCESS;
}

void SMS::power(bool on) { UNUSED(on); }

void SMS::send(const char* str) {
  // cons << str;
  stream.write(str, strlen(str));
}

void SMS::send(uint16_t n) {
  char str[32];
  snprintf(str, sizeof(str), "%u", n);
  // cons << str;
  stream.write(str, strlen(str));
}

int SMS::internalsendSMSBinary(const uint8_t* txData, size_t txDataSize) {

  mavio::log(LOG_INFO, "mav msg size: %d", txDataSize);
  
  uint data_size_int = txDataSize;

  char pdu[160];
  memset(pdu, 0, sizeof(pdu));

  char data_size[3];
  memset(data_size, 0, sizeof(data_size));
  
  char data[140];
  memset(data, 0, sizeof(data));
  
  char command[80];
  memset(command, 0, sizeof(command));
  
  char pdu_size[3];
  memset(pdu_size, 0, sizeof(pdu_size));

  int pdu_size_int = 0;

  pdu_size_int = data_size_int + 1 + 13;

  mavio::log(LOG_INFO, "pdu size int: %d", pdu_size_int);

  snprintf(pdu_size, 3, "%d",(unsigned char)pdu_size_int);

  mavio::log(LOG_INFO, "pdu size str: %s", pdu_size);

  pdu_size[2] = '\0';


  mavio::log(LOG_INFO, "data size int: %d", data_size_int);

  snprintf(data_size, 3, "%d",(unsigned char)data_size_int);
  
  data_size[2] = '\0';

  mavio::log(LOG_INFO, "data size str: %s", data_size);

  mavio::log(LOG_INFO, "strlen %d", strlen(data_size));

  char ca[1];

  char fuckthefuck[2];

  snprintf(fuckthefuck, 3, "%x", data_size_int);
// nos hemos quedado aqui: tras 0002AA, lo siguiente deberia ser size of data, size del actual data to send
// el int no nos corresponde con el hex en character enviado. ahi estamos
// -----------------------------------

  int character;
  char octett[10];

  for (character=0 ; character < ( data_size_int*2) ; character++)
  {
    sprintf(octett,"%02X",(unsigned char) txData[character]);
    strcat(data,octett);
    ca[0] = data[character];
    mavio::log(LOG_INFO, "data %d byte: %s", character, ca);
  }

    mavio::log(LOG_INFO, "strlen %d", strlen(data));
// --------------------------------

  // snprintf(pdu, sizeof(pdu), "%s%s%s%s", "0011000B914336575652F60002AA", "33", data, "\x1A");

  // for (int x = 0; x < sizeof(pdu); x++ ) {
  //   ca[0] = pdu[x];
  //   mavio::log(LOG_INFO, "pdu byte: %s", ca);
  // }

  // mavio::log(LOG_INFO, "pdu size: %d", sizeof(pdu));
  
  if (txData && txDataSize) {  
    
    // snprintf(command, sizeof(command), "%s%s%s", "AT+CMGS=", pdu_size, "\r");
    // send(command);
    send("AT+CMGS=");
    send(pdu_size);
    send("\r");


    if(waitForATResponse(NULL, 0, NULL, "> ")) {
      mavio::log(LOG_INFO, "at > ok");
    } else {
      mavio::log(LOG_INFO, "at > NOT ok");
    }
    send("0011000B914336575652F60004AA");
    send(fuckthefuck);
    stream.write(data, data_size_int*2);
    send("\x1A");

    // send("0011000B914336575652F60002AA0AE8329BFD4697D9BC47");
    // send("\x1A");

    // for (size_t i = 0; i < txDataSize; ++i) {
    //   stream.write(txData[i]);
    // }

    if (!waitForATResponse(NULL, 0, NULL, "OK\r\n")) {
      return cancelled() ? GSM_CANCELLED : GSM_PROTOCOL_ERROR;
    }
  }
  return GSM_SUCCESS;
}

int SMS::internalreceiveSMSBinary(uint8_t* rxBuffer, size_t& rxBufferSize) {

  send("AT+CMGL\r");
  return waitforSMSlist(rxBuffer, rxBufferSize);
}

int SMS::internalreadSMSlist(void) {
  
  // char Buffer[160];
  // send("AT+CMGL\r");

  // if (!waitforSMSlist(Buffer, sizeof(Buffer)) ) {
  //   return cancelled() ? GSM_CANCELLED : GSM_PROTOCOL_ERROR;
  // } 

  // mavio::log(LOG_INFO, "cmgl: %s", Buffer);

  return true;
}

bool SMS::waitforSMSlist(uint8_t* response, size_t responseSize) {

  if (response) {
    memset(response, 0, responseSize);
  }

  int matchPromptPos = 0;      // Matches chars in prompt

  sms_struct _buffer_sms;

  char indexresponse[3];  // allow for 3 digits indexes
  int indexresponse_pos = 0;
  uint8_t index;
  
  uint8_t pdu_size;
  char pdu_size_response[3]; // allow for 3 digits size, we need more than 100
  int pdu_size_response_pos = 0;

  char bytechar[2];
  int counterbyte = 0;
  int counter = 0;

  uint8_t smsc_size;
  char smsc_size_response[2]; // allow for 3 digits size, we need more than 100
  int smsc_size_response_pos = 0;

  int _step = 0;
  int sender_number_pos = 0;
  int timestamp_pos = 0;
  int data_pos = 0;

  enum { LOOKING_FOR_PROMPT, GATHERING_RESPONSE, LOOKING_FOR_TERMINATOR };

  const char prompt[] = "+CMGL: ";
  int promptState = LOOKING_FOR_PROMPT;

  Stopwatch timer;
  while (timer.elapsed_time() < atTimeout) {
    if (cancelled()) {
      return false;
    }

    int cc = stream.read();
    if (cc >= 0) {
      char c = cc;

      char ca[1];
      ca[0] = c; 
      mavio::log(LOG_INFO, "rd: %s", ca);

      if (prompt) {
        switch (promptState) {
          case LOOKING_FOR_PROMPT:
            if (c == prompt[matchPromptPos]) {
              ++matchPromptPos;
              if (prompt[matchPromptPos] == '\0') {
                promptState = GATHERING_RESPONSE;
                mavio::log(LOG_INFO, "gathering response");
              }
            } else {
              matchPromptPos = c == prompt[0] ? 1 : 0;
            }

            break;
          case GATHERING_RESPONSE: 
                if ( c != ' ') {
                  switch (_step) {
                    case 0: // index
                      if ( c != ',' || indexresponse_pos >= 3 ) {
                        indexresponse[indexresponse_pos] = c;
                        indexresponse_pos++;
                        break;
                      } else {
                        index = atoi(indexresponse);
                        mavio::log(LOG_INFO, "index: %d", index);
                        indexresponse_pos = 0;
                        _step++;
                        [[fallthrough]];
                      } 
                    case 1: // size, etc, ignore by the moment
                      if ( c != ',' ) {
                        pdu_size_response[pdu_size_response_pos] = c;
                        pdu_size_response_pos++;
                      } else {
                        pdu_size_response_pos = 0;
                      }
                      if ( c == '\r' ) {
                        pdu_size = atoi(pdu_size_response);
                        mavio::log(LOG_INFO, "pdu size: %d", pdu_size);
                        pdu_size_response_pos = 0;
                        _step++;
                      }
                      break;
                    case 2: // sms struct
                      if ( isxdigit(c) ) {
                        smsc_size_response[smsc_size_response_pos] = c;
                        smsc_size_response_pos++;
                      }
                      if ( smsc_size_response_pos >= 2 ) {
                        smsc_size = octet2bin(smsc_size_response);
                        mavio::log(LOG_INFO, "smsc size: %d", smsc_size);
                        smsc_size_response_pos = 0;
                        _step++;
                      }
                      break;
                    case 3: // type_address
                      if ( isxdigit(c) ) {
                        bytechar[counter] = c;
                        counter++;
                      }
                      if ( counter >= 2 ) {
                        counter = 0;
                        buffersms.type_address = octet2bin(bytechar);
                        mavio::log(LOG_INFO, "type address: %x", buffersms.type_address);
                        _step++;

                        memset(buffersms.service_center_number, 0, sizeof(buffersms.service_center_number));
                      }
                      break;
                    case 4: // service_center_number
                      if ( isxdigit(c) ) {
                        if ( !(counter & 1) ) { // even
                          buffersms.service_center_number[counter + 1] = c;
                        } else { // odd
                          buffersms.service_center_number[counter - 1] = c;
                        }
                        counter++;
                      }
                      if ( counter >= ( ( smsc_size - 1 ) << 1 ) ) {
                        if ( !isdigit(buffersms.service_center_number[counter-1]) ) {
                          buffersms.service_center_number[counter-1] = '\0';
                        } else {
                          buffersms.service_center_number[counter] = '\0';
                        }
                        counter = 0;
                        mavio::log(LOG_INFO, "service center number: %s", buffersms.service_center_number);
                        _step++;
                      }
                      break;
                    case 5:
                      bytechar[counter] = c;
                      counter++;
                      if ( counter >= 2 ) {
                        counter = 0;
                        buffersms.first_byte = octet2bin(bytechar);
                        mavio::log(LOG_INFO, "fist fyte: %x", buffersms.first_byte);
                        _step++;\
                      }
                      break;
                    case 6:
                      if ( isxdigit(c) ) {
                        bytechar[counter] = c;
                        counter++;
                      }
                      if ( counter >= 2 ) {
                        counter = 0;
                        buffersms.address_lenght = octet2bin(bytechar);
                        mavio::log(LOG_INFO, "address lenght: %d", buffersms.address_lenght);
                        _step++;

                        memset(buffersms.sender_number, 0, sizeof(buffersms.sender_number));
                      }
                      break;
                    case 7:
                      bytechar[counter] = c;
                      counter++;
                      if ( counter >= 2 ) {
                        counter = 0;
                        buffersms.address_type = octet2bin(bytechar);
                        mavio::log(LOG_INFO, "address type: %x", buffersms.address_type);
                        _step++;
                      }
                      break;
                    case 8:
                      if ( isxdigit(c) ) {
                        if ( !(counter & 1) ) { // even
                          buffersms.sender_number[counter + 1] = c;
                        } else { // odd
                          buffersms.sender_number[counter - 1] = c;
                        }
                        counter++;
                      }
                      if ( counter > buffersms.address_lenght ) {
                        if ( !isdigit(buffersms.sender_number[counter-1]) ) {
                          buffersms.sender_number[counter-1] = '\0';
                        } else {
                          buffersms.sender_number[counter] = '\0';
                        }
                        counter = 0;
                        mavio::log(LOG_INFO, "sender number: %s", buffersms.sender_number);
                        _step++;
                      }
                      break;
                    case 9:
                      bytechar[counter] = c;
                      counter++;
                      if ( counter >= 2 ) {
                        counter = 0;
                        buffersms.TP_PID = octet2bin(bytechar);
                        mavio::log(LOG_INFO, "tc_pid: %x", buffersms.TP_PID);
                        _step++;
                      }
                      break;
                    case 10:
                      bytechar[counter] = c;
                      counter++;
                      if ( counter >= 2 ) {
                        counter = 0;
                        buffersms.TO_DCS = octet2bin(bytechar);
                        mavio::log(LOG_INFO, "to_dcs: %x", buffersms.TO_DCS);
                        _step++;
                      }
                      break;
                    case 11:
                      if ( isxdigit(c) ) {
                        if ( !(counter & 1) ) { // even
                          buffersms.timestamp[counter + 1] = c;
                        } else { // odd
                          buffersms.timestamp[counter - 1] = c;
                        }
                        counter++;
                      }
                      if ( counter >= 14 /*timestamp format*/ ) {
                        buffersms.timestamp[counter] = '\0';
                        counter = 0;
                        mavio::log(LOG_INFO, "timestamp: %s", buffersms.timestamp);
                        _step++;
                      }
                      break;
                    case 12:
                      bytechar[counter] = c;
                      counter++;
                      if ( counter >= 2 ) {
                        counter = 0;
                        buffersms.data_lenght = octet2bin(bytechar);
                        mavio::log(LOG_INFO, "data_lenght: %x", buffersms.data_lenght);
                        _step++;

                        memset(buffersms.data, 0, sizeof(buffersms.data));
                      }
                      break;
                    case 13:
                      if ( isxdigit(c) ) {
                        bytechar[counterbyte] = c;
                        counterbyte++;
                      }
                      if ( counterbyte >=2 ) {
                        buffersms.data[counter] = octet2bin(bytechar);
                        counterbyte = 0;
                        mavio::log(LOG_INFO, "byte[%d] = %x", counter, buffersms.data[counter]);
                        counter++;
                      }
                      if ( counter >= buffersms.data_lenght ) {
                        counter = 0;
                        for ( int i=0 ; i < buffersms.data_lenght ; i++ ) { 
                          mavio::log(LOG_INFO, "data: %x", buffersms.data[i]);
                        }
                        _step = 0;
                        promptState = LOOKING_FOR_TERMINATOR;
                      }
                      break;
                  }
                } 
            break;
          case LOOKING_FOR_TERMINATOR:
            if (waitForATResponse()) {
              mavio::log(LOG_INFO, "ok");
            }

            mavio::log(LOG_INFO, "sender number: %s", buffersms.sender_number);
            for ( int i=0 ; i < buffersms.data_lenght ; i++ ) { 
                          response[i] = buffersms.data[i];
                          mavio::log(LOG_INFO, "data: %x", response[i]);
                        }
            responseSize = buffersms.data_lenght;
            mavio::log(LOG_INFO, "rsponsesize: %d", responseSize);
            // READ only first message by the moment, then skip until ok
            if (waitForATResponse()) {
              mavio::log(LOG_INFO, "ok");
            }
            return true;
        }
      }
    }  
  }
  return false;
}

int SMS::octet2bin(char* octet) /* converts an octet to a 8-Bit value */
{
  int result=0;

  if (octet[0]>57)
    result+=octet[0]-55;
  else
    result+=octet[0]-48;
  result=result<<4;
  if (octet[1]>57)
    result+=octet[1]-55;
  else
    result+=octet[1]-48;
  return result;
}

// Converts an octet to a 8bit value,
// returns < in case of error.
int SMS::octet2bin_check(char *octet)
{
  if (octet[0] == 0)
    return -1;
  if (octet[1] == 0)
    return -2;
  if (!isxdigit(octet[0]))
    return -3;
  if (!isxdigit(octet[1]))
    return -4;
  return octet2bin(octet);
}

/* Converts binary to PDU string, this is basically a hex dump. */
void SMS::binary2pdu(char* binary, int length, char* pdu)
{
  int character;
  char octett[10];

  if (length>140) // 140 is max sms size 
    length=140;
  pdu[0]=0;
  for (character=0;character<length; character++)
  {
    sprintf(octett,"%02X",(unsigned char) binary[character]);
    strcat(pdu,octett);
  }
}

}  // namespace mavio

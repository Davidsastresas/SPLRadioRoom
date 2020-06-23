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

const std::chrono::milliseconds GSM_startup_timeout(500);
const std::chrono::milliseconds GSM_default_at_timeout(4000);
const std::chrono::milliseconds GSM_startup_max_time(10000);

extern bool isbdCallback() __attribute__((weak));

#define UNUSED(x) (void)(x)

bool isbdCallback() { return true; }

SMS::SMS(Serial& serial)
    : stream(serial),
      atTimeout(GSM_default_at_timeout),
      reentrant(false) {}

// Power on the RockBLOCK or return from sleep
int SMS::begin(std::string pin) {
  if (this->reentrant) {
    return GSM_REENTRANT;
  }

  this->reentrant = true;
  int ret = internalBegin(pin);
  this->reentrant = false;

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
int SMS::sendSMSBinary(const uint8_t* txData, size_t txDataSize, std::string tlf) {
  if (this->reentrant) {
    return GSM_REENTRANT;
  }

  this->reentrant = true;
  int ret = internalsendSMSBinary(txData, txDataSize, tlf);
  this->reentrant = false;
  return ret;
}

// Transmit a text message
int SMS::sendSMSText(const uint8_t* txData, size_t txDataSize, std::string tlf) {
  if (this->reentrant) {
    return GSM_REENTRANT;
  }

  this->reentrant = true;
  int ret = internalsendSMSText(txData, txDataSize, tlf);
  this->reentrant = false;
  return ret;
}

// Transmit and receive a binary message
int SMS::receiveSMSBinary(uint8_t* rxBuffer, size_t& rxBufferSize, bool& inbox_emtpy) {
  if (this->reentrant) {
    return GSM_REENTRANT;
  }

  this->reentrant = true;
  int ret = internalreceiveSMSBinary(rxBuffer, rxBufferSize, inbox_emtpy);
  this->reentrant = false;
  return ret;
}

// Transmit and receive a binary message
int SMS::receiveSMSText(uint8_t* rxBuffer, size_t& rxBufferSize, bool& inbox_emtpy) {
  if (this->reentrant) {
    return GSM_REENTRANT;
  }

  this->reentrant = true;
  int ret = internalreceiveSMSText(rxBuffer, rxBufferSize, inbox_emtpy);
  this->reentrant = false;
  return ret;
}

int SMS::deleteSMSlist() {
  if (this->reentrant) {
    return GSM_REENTRANT;
  }

  this->reentrant = true;
  int ret = internaldeleteSMSlist();
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

/*
Private interface
*/

int SMS::internalBegin(std::string pin) {

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

  // question
  const char* askpinstr = "AT+CPIN?\r";

  // answers
  const char* pinready = "+CPIN: READY";
  const char* pinneeded = "+CPIN: SIM PIN";
  
  // prepare to ask pin
  const char* pinstrpref = "AT+CPIN=\"";
  const char* pinstrsuf = "\"\r";
  const char* pinstr = pin.c_str();
  
  char buffer[256];

  // Ask if pin is already inserted
  send(askpinstr);
  if (!waitForATResponse(buffer, sizeof(buffer), "AT+CPIN?\r\r\n", "OK")) {
    return GSM_PROTOCOL_ERROR;
  }

  // Check if pin was already inserted or not. If not, insert it
  if ( !strcmp(buffer, pinready) ) {

    mavio::log(LOG_NOTICE, "SMS: pin already inserted");

  } else if ( !strcmp(buffer, pinneeded ) ) {

    // send(pinstr);
    send(pinstrpref);
    send(pinstr);
    send(pinstrsuf);
    if (!waitForATResponse()) {
      return cancelled() ? GSM_CANCELLED : GSM_PROTOCOL_ERROR;
    }

    mavio::log(LOG_NOTICE, "SMS: pin setup succesful");

  } else {

    mavio::log(LOG_WARNING, "SMS: problem introducing gsm pin!");
    return GSM_PROTOCOL_ERROR;
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

  send("AT+CGMM\r");

  if (!waitForATResponse(buffer, bufferSize, "AT+CGMM\r\r\n", "OK")) {
    return cancelled() ? GSM_CANCELLED : GSM_PROTOCOL_ERROR;
  }

  return GSM_SUCCESS;
}

int SMS::internalGetTransceiverSerialNumber(char* buffer,
                                                   size_t bufferSize) {

  send("AT+CGSN\r");

  if (!waitForATResponse(buffer, bufferSize, "AT+CGSN\r\r\n", "OK")) {
    return cancelled() ? GSM_CANCELLED : GSM_PROTOCOL_ERROR;
  }

  return GSM_SUCCESS;
}

int SMS::internalGetSignalQuality(int& quality) {

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

      // mavio::log(LOG_INFO, "rd: %c", c);

      if (prompt) {
        switch (promptState) {
          case LOOKING_FOR_PROMPT:
            if (c == prompt[matchPromptPos]) {
              ++matchPromptPos;
              if (prompt[matchPromptPos] == '\0') {
                promptState = GATHERING_RESPONSE;
                // mavio::log(LOG_INFO, "gathering response");
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
                //mavio::log(LOG_INFO, "looking for terminator");
              } else {
                *response++ = c;
                responseSize--;
                //mavio::log(LOG_INFO, "response kept");
              }
            }
            break;
        }  // switch
      }    // prompt

      if (c == terminator[matchTerminatorPos]) {
        // mavio::log(LOG_INFO, "terminator matches");
        ++matchTerminatorPos;
        if (terminator[matchTerminatorPos] == '\0') {
          // mavio::log(LOG_INFO, "terminator finished");
          return true;
        }
      } else {
        matchTerminatorPos = c == terminator[0] ? 1 : 0;
      }
    }  // if (cc >= 0)
  }    // timer loop
   //mavio::log(LOG_INFO, "timer out");

  return false;
}

bool SMS::waitForATResponseDebug(char* response, int responseSize,
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
      mavio::log(LOG_INFO, "cancelled");
      return false;
    }

    int cc = stream.read();
    if (cc >= 0) {
      char c = cc;

      mavio::log(LOG_INFO, "rd: %c", c);

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
          mavio::log(LOG_INFO, "terminator finished");
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

      // mavio::log(LOG_INFO, "rd: %c", c);

      if (prompt) {
        switch (promptState) {
          case LOOKING_FOR_PROMPT:
            if (c == prompt[matchPromptPos]) {
              ++matchPromptPos;
              if (prompt[matchPromptPos] == '\0') {
                promptState = GATHERING_RESPONSE;
                // mavio::log(LOG_INFO, "gathering response");
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
                //mavio::log(LOG_INFO, "looking for terminator");
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

int SMS::internalsendSMSBinary(const uint8_t* txData, size_t txDataSize, std::string tlf) {

  // mavio::log(LOG_INFO, "mav msg size: %d", txDataSize);
  const char* tlfstr = tlf.c_str();
  // mavio::log(LOG_INFO, "Tlf number: %s", tlfstr);
  
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

  // mavio::log(LOG_INFO, "pdu size int: %d", pdu_size_int);

  snprintf(pdu_size, 3, "%d",(unsigned char)pdu_size_int);

  pdu_size[2] = '\0';

  // mavio::log(LOG_INFO, "data size int: %d", data_size_int);

  snprintf(data_size, 3, "%d",(unsigned char)data_size_int);
  
  data_size[2] = '\0';

  // mavio::log(LOG_INFO, "data size str: %s", data_size);

  // mavio::log(LOG_INFO, "strlen %d", strlen(data_size));

  // char ca[1];

  char data_size_str[2];

  snprintf(data_size_str, 3, "%x", data_size_int);

// -----------------------------------

  uint character;
  char octett[10];

  for (character=0 ; character < ( data_size_int*2) ; character++)
  {
    sprintf(octett,"%02X",(unsigned char) txData[character]);
    strcat(data,octett);
    // ca[0] = data[character];
    // mavio::log(LOG_INFO, "data %d byte: %s", character, ca);
  }

    // mavio::log(LOG_INFO, "strlen %d", strlen(data));
// --------------------------------

  // snprintf(pdu, sizeof(pdu), "%s%s%s%s", "0011000B914336575652F60002AA", "33", data, "\x1A");

  // for (int x = 0; x < sizeof(pdu); x++ ) {
  //   ca[0] = pdu[x];
  //   // mavio::log(LOG_INFO, "pdu byte: %s", ca);
  // }

  // // mavio::log(LOG_INFO, "pdu size: %d", sizeof(pdu));
  
  if (txData && txDataSize) {  
    
    send("AT+CMGS=");
    send(pdu_size);
    send("\r");

    if(waitForATResponse(NULL, 0, NULL, "> ")) {
      // mavio::log(LOG_INFO, "at > ok");
    } else {
      // mavio::log(LOG_INFO, "at > NOT ok");
    }

    send("0011000C91");
    send(tlfstr);    
    send("0004AA");
    send(data_size_str);
    stream.write(data, data_size_int*2);
    send("\x1A");

    if (!waitForATResponse(NULL, 0, NULL, "OK\r\n")) {
      return cancelled() ? GSM_CANCELLED : GSM_PROTOCOL_ERROR;
    }
    return GSM_SUCCESS;
  } 
  return GSM_MSG_TO_SEND_EMPTY;
}

int SMS::internalsendSMSText(const uint8_t* txData, size_t txDataSize, std::string tlf) {

  // mavio::log(LOG_INFO, "mav msg size: %d", txDataSize);
  const char* tlfstr = tlf.c_str();
  // mavio::log(LOG_INFO, "Tlf number: %s", tlfstr);
  
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

  // mavio::log(LOG_INFO, "pdu size int: %d", pdu_size_int);

  snprintf(pdu_size, 3, "%d",(unsigned char)pdu_size_int);

  // mavio::log(LOG_INFO, "pdu size str: %s", pdu_size);

  pdu_size[2] = '\0';

  // mavio::log(LOG_INFO, "data size int: %d", data_size_int);

  snprintf(data_size, 3, "%d",(unsigned char)data_size_int);
  
  data_size[2] = '\0';

  // mavio::log(LOG_INFO, "data size str: %s", data_size);

  // mavio::log(LOG_INFO, "strlen %d", strlen(data_size));

  // char ca[1];

  char data_size_str[2];

  snprintf(data_size_str, 3, "%x", data_size_int);

// -----------------------------------

  uint character;
  char octett[10];

  for (character=0 ; character < ( data_size_int*2) ; character++)
  {
    sprintf(octett,"%02X",(unsigned char) txData[character]);
    strcat(data,octett);
    // ca[0] = data[character];
    // mavio::log(LOG_INFO, "data %d byte: %s", character, ca);
  }

    // mavio::log(LOG_INFO, "strlen %d", strlen(data));
// --------------------------------

  // snprintf(pdu, sizeof(pdu), "%s%s%s%s", "0011000B914336575652F60002AA", "33", data, "\x1A");

  // for (int x = 0; x < sizeof(pdu); x++ ) {
  //   ca[0] = pdu[x];
  //   // mavio::log(LOG_INFO, "pdu byte: %s", ca);
  // }

  // // mavio::log(LOG_INFO, "pdu size: %d", sizeof(pdu));
  
  if (txData && txDataSize) {  
    
    send("AT+CMGS=");
    send(pdu_size);
    send("\r");

    if(waitForATResponse(NULL, 0, NULL, "> ")) {
      // mavio::log(LOG_INFO, "at > ok");
    } else {
      // mavio::log(LOG_INFO, "at > NOT ok");
    }

    send("0011000C91");
    send(tlfstr);    
    send("0004AA");
    send(data_size_str);
    stream.write(data, data_size_int*2);
    send("\x1A");

    if (!waitForATResponse(NULL, 0, NULL, "OK\r\n")) {
      return cancelled() ? GSM_CANCELLED : GSM_PROTOCOL_ERROR;
    }
    return GSM_SUCCESS;
  } 
  return GSM_MSG_TO_SEND_EMPTY;
}

int SMS::internalreceiveSMSBinary(uint8_t* rxBuffer, size_t& rxBufferSize, bool& inbox_empty) {

  send("AT+CMGL\r");
  int ret;
  ret = waitforSMSlistBin(rxBuffer, rxBufferSize, inbox_empty);
  return ret;
}

int SMS::internalreceiveSMSText(uint8_t* rxBuffer, size_t& rxBufferSize, bool& inbox_empty) {

  send("AT+CMGL\r");
  int ret;
  ret = waitforSMSlistText(rxBuffer, rxBufferSize, inbox_empty);
  return ret;
}

bool SMS::waitforSMSlistBin(uint8_t* response, size_t& responseSize, bool& inbox_empty) {

  if (response) {
    memset(response, 0, responseSize);
  }

  sms_struct _buffer_sms;

  char indexresponse[3]; 
  memset(indexresponse, 0, sizeof(indexresponse));  // allow for 3 digits indexes
  int indexresponse_pos = 0;
  uint8_t index;
  
  // uint8_t pdu_size;
  char pdu_size_response[3]; // allow for 3 digits size, we need more than 100
  memset(pdu_size_response, 0, sizeof(pdu_size_response));  
  int pdu_size_response_pos = 0;

  char bytechar[2];
  memset(bytechar, 0, sizeof(bytechar));  
  int counterbyte = 0;
  int counter = 0;

  uint8_t smsc_size;
  char smsc_size_response[2]; // allow for 3 digits size, we need more than 100
  memset(smsc_size_response, 0, sizeof(smsc_size_response));  
  int smsc_size_response_pos = 0;

  int _step = 0;

  enum { LOOKING_FOR_PROMPT, GATHERING_RESPONSE, LOOKING_FOR_TERMINATOR };
  int promptState = LOOKING_FOR_PROMPT;

  const char prompt[] = "+CMGL: ";
  int matchPromptPos = 0;      // Matches chars in prompt
  
  const char prompt_no_msg[] = "OK\r\n";
  int matchPromptPos_no_msg = 0; 

  // parsing variables
  int cc;
  char c;

  Stopwatch timer;
  while (timer.elapsed_time() < atTimeout) {
    if (cancelled()) {
      return false;
    }

    cc = stream.read();
    if (cc >= 0) {
      c = cc;

      // mavio::log(LOG_INFO, "SMS rd: %c", c);

      if (prompt) {
        switch (promptState) {
          case LOOKING_FOR_PROMPT:
            if (c == prompt[matchPromptPos]) {
              ++matchPromptPos;
              if (prompt[matchPromptPos] == '\0') {
                promptState = GATHERING_RESPONSE;
                // mavio::log(LOG_INFO, "gathering response");
              }
            } else {
              matchPromptPos = c == prompt[0] ? 1 : 0;
            }

            if (c == prompt_no_msg[matchPromptPos_no_msg]) {
              // mavio::log(LOG_INFO, "prompt_no_msg matches");
              ++matchPromptPos_no_msg;
              if (prompt_no_msg[matchPromptPos_no_msg] == '\0') {
                // mavio::log(LOG_INFO, "prompt_no_msg finished");
                inbox_empty = true;
                return GSM_INBOX_EMPTY;
              }
            } else {
              matchPromptPos_no_msg = c == prompt_no_msg[0] ? 1 : 0;
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
                        // mavio::log(LOG_INFO, "SMS index: %d", index);
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
                        // pdu_size = atoi(pdu_size_response);
                        // mavio::log(LOG_INFO, "pdu size: %d", pdu_size);
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
                        // mavio::log(LOG_INFO, "smsc size: %d", smsc_size);
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
                        // mavio::log(LOG_INFO, "type address: %x", buffersms.type_address);
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
                        // mavio::log(LOG_INFO, "service center number: %s", buffersms.service_center_number);
                        _step++;
                      }
                      break;
                    case 5:
                      bytechar[counter] = c;
                      counter++;
                      if ( counter >= 2 ) {
                        counter = 0;
                        buffersms.first_byte = octet2bin(bytechar);
                        // mavio::log(LOG_INFO, "fist fyte: %x", buffersms.first_byte);
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
                        // mavio::log(LOG_INFO, "address lenght: %d", buffersms.address_lenght);
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
                        // mavio::log(LOG_INFO, "address type: %x", buffersms.address_type);
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
                        // mavio::log(LOG_INFO, "sender number: %s", buffersms.sender_number);
                        _step++;
                      }
                      break;
                    case 9:
                      bytechar[counter] = c;
                      counter++;
                      if ( counter >= 2 ) {
                        counter = 0;
                        buffersms.TP_PID = octet2bin(bytechar);
                        // mavio::log(LOG_INFO, "tc_pid: %x", buffersms.TP_PID);
                        _step++;
                      }
                      break;
                    case 10:
                      bytechar[counter] = c;
                      counter++;
                      if ( counter >= 2 ) {
                        counter = 0;
                        buffersms.TO_DCS = octet2bin(bytechar);
                        // mavio::log(LOG_INFO, "to_dcs: %x", buffersms.TO_DCS);
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
                        // mavio::log(LOG_INFO, "timestamp: %s", buffersms.timestamp);
                        _step++;
                      }
                      break;
                    case 12:
                      bytechar[counter] = c;
                      counter++;
                      if ( counter >= 2 ) {
                        counter = 0;
                        buffersms.data_lenght = octet2bin(bytechar);
                        // mavio::log(LOG_INFO, "data_lenght: %x", buffersms.data_lenght);
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
                        // mavio::log(LOG_INFO, "byte[%d] = %x", counter, buffersms.data[counter]);
                        counter++;
                      }
                      if ( counter >= buffersms.data_lenght ) {
                        counter = 0;
                        // for ( int i=0 ; i < buffersms.data_lenght ; i++ ) { 
                        //   mavio::log(LOG_INFO, "data: %x", buffersms.data[i]);
                        // }
                        _step = 0;
                        promptState = LOOKING_FOR_TERMINATOR;
                      }
                      break;
                  }
                } 
            break;
          case LOOKING_FOR_TERMINATOR:
            if (waitForATResponse()) {
              // mavio::log(LOG_INFO, "ok");
            }

            // mavio::log(LOG_INFO, "sender number: %s", buffersms.sender_number);
            
            for ( int i=0 ; i < buffersms.data_lenght ; i++ ) { 
                          response[i] = buffersms.data[i];
                          // mavio::log(LOG_INFO, "data: %x", response[i]);
            }

            responseSize = buffersms.data_lenght;
            // mavio::log(LOG_INFO, "rsponsesize: %d", responseSize);

            if ( index > 5 ) {
              mavio::log(LOG_WARNING, "SMS: More than 5 SMS in inbox! Deleting ...");
              send("AT+CMGD=0,4\r");
            } else {
              send("AT+CMGD=");
              
              char index_char[3];
              memset(index_char, 0, sizeof(index_char));
              int index_char_int = 0;

              index_char_int = index;
              snprintf(index_char, 3, "%d",(unsigned char)index_char_int);
              index_char[2] = '\0';

              send(index_char);

              send("\r");
            }

            if (waitForATResponse()) {
              // mavio::log(LOG_INFO, "ok");
            }
            inbox_empty = false;
            return GSM_SUCCESS;
        }
      }
    }  
  }
  mavio::log(LOG_WARNING, "SMS: Read sms list timeout");
  return GSM_PROTOCOL_ERROR;
}

bool SMS::waitforSMSlistText(uint8_t* response, size_t& responseSize, bool& inbox_empty) {

  if (response) {
    memset(response, 0, responseSize);
  }

  sms_struct _buffer_sms;

  char indexresponse[3]; 
  memset(indexresponse, 0, sizeof(indexresponse));  // allow for 3 digits indexes
  int indexresponse_pos = 0;
  uint8_t index;
  
  // uint8_t pdu_size;
  char pdu_size_response[3]; // allow for 3 digits size, we need more than 100
  memset(pdu_size_response, 0, sizeof(pdu_size_response));  
  int pdu_size_response_pos = 0;

  char bytechar[2];
  memset(bytechar, 0, sizeof(bytechar));  
  int counterbyte = 0;
  int counter = 0;

  uint8_t smsc_size;
  char smsc_size_response[2]; // allow for 3 digits size, we need more than 100
  memset(smsc_size_response, 0, sizeof(smsc_size_response));  
  int smsc_size_response_pos = 0;

  int _step = 0;

  enum { LOOKING_FOR_PROMPT, GATHERING_RESPONSE, LOOKING_FOR_TERMINATOR };
  int promptState = LOOKING_FOR_PROMPT;

  const char prompt[] = "+CMGL: ";
  int matchPromptPos = 0;      // Matches chars in prompt
  
  const char prompt_no_msg[] = "OK\r\n";
  int matchPromptPos_no_msg = 0; 

  // parsing variables
  int cc;
  char c;

  Stopwatch timer;
  while (timer.elapsed_time() < atTimeout) {
    if (cancelled()) {
      return false;
    }

    cc = stream.read();
    if (cc >= 0) {
      c = cc;

      // mavio::log(LOG_INFO, "SMS rd: %c", c);

      if (prompt) {
        switch (promptState) {
          case LOOKING_FOR_PROMPT:
            if (c == prompt[matchPromptPos]) {
              ++matchPromptPos;
              if (prompt[matchPromptPos] == '\0') {
                promptState = GATHERING_RESPONSE;
                // mavio::log(LOG_INFO, "gathering response");
              }
            } else {
              matchPromptPos = c == prompt[0] ? 1 : 0;
            }

            if (c == prompt_no_msg[matchPromptPos_no_msg]) {
              // mavio::log(LOG_INFO, "prompt_no_msg matches");
              ++matchPromptPos_no_msg;
              if (prompt_no_msg[matchPromptPos_no_msg] == '\0') {
                // mavio::log(LOG_INFO, "prompt_no_msg finished");
                inbox_empty = true;
                return GSM_INBOX_EMPTY;
              }
            } else {
              matchPromptPos_no_msg = c == prompt_no_msg[0] ? 1 : 0;
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
                        // mavio::log(LOG_INFO, "SMS index: %d", index);
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
                        // pdu_size = atoi(pdu_size_response);
                        // mavio::log(LOG_INFO, "pdu size: %d", pdu_size);
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
                        // mavio::log(LOG_INFO, "smsc size: %d", smsc_size);
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
                        // mavio::log(LOG_INFO, "type address: %x", buffersms.type_address);
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
                        // mavio::log(LOG_INFO, "service center number: %s", buffersms.service_center_number);
                        _step++;
                      }
                      break;
                    case 5:
                      bytechar[counter] = c;
                      counter++;
                      if ( counter >= 2 ) {
                        counter = 0;
                        buffersms.first_byte = octet2bin(bytechar);
                        // mavio::log(LOG_INFO, "fist fyte: %x", buffersms.first_byte);
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
                        // mavio::log(LOG_INFO, "address lenght: %d", buffersms.address_lenght);
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
                        // mavio::log(LOG_INFO, "address type: %x", buffersms.address_type);
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
                        // mavio::log(LOG_INFO, "sender number: %s", buffersms.sender_number);
                        _step++;
                      }
                      break;
                    case 9:
                      bytechar[counter] = c;
                      counter++;
                      if ( counter >= 2 ) {
                        counter = 0;
                        buffersms.TP_PID = octet2bin(bytechar);
                        // mavio::log(LOG_INFO, "tc_pid: %x", buffersms.TP_PID);
                        _step++;
                      }
                      break;
                    case 10:
                      bytechar[counter] = c;
                      counter++;
                      if ( counter >= 2 ) {
                        counter = 0;
                        buffersms.TO_DCS = octet2bin(bytechar);
                        // mavio::log(LOG_INFO, "to_dcs: %x", buffersms.TO_DCS);
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
                        // mavio::log(LOG_INFO, "timestamp: %s", buffersms.timestamp);
                        _step++;
                      }
                      break;
                    case 12:
                      bytechar[counter] = c;
                      counter++;
                      if ( counter >= 2 ) {
                        counter = 0;
                        buffersms.data_lenght = octet2bin(bytechar);
                        // mavio::log(LOG_INFO, "data_lenght: %x", buffersms.data_lenght);
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
                        // mavio::log(LOG_INFO, "byte[%d] = %x", counter, buffersms.data[counter]);
                        counter++;
                      }
                      if ( counter >= buffersms.data_lenght ) {
                        counter = 0;
                        // for ( int i=0 ; i < buffersms.data_lenght ; i++ ) { 
                        //   mavio::log(LOG_INFO, "data: %x", buffersms.data[i]);
                        // }
                        _step = 0;
                        promptState = LOOKING_FOR_TERMINATOR;
                      }
                      break;
                  }
                } 
            break;
          case LOOKING_FOR_TERMINATOR:
            if (waitForATResponse()) {
              // mavio::log(LOG_INFO, "ok");
            }

            // mavio::log(LOG_INFO, "sender number: %s", buffersms.sender_number);
            
            for ( int i=0 ; i < buffersms.data_lenght ; i++ ) { 
                          response[i] = buffersms.data[i];
                          // mavio::log(LOG_INFO, "data: %x", response[i]);
            }

            responseSize = buffersms.data_lenght;
            // mavio::log(LOG_INFO, "rsponsesize: %d", responseSize);

            if ( index > 5 ) {
              mavio::log(LOG_WARNING, "SMS: More than 5 SMS in inbox! Deleting ...");
              send("AT+CMGD=0,4\r");
            } else {
              send("AT+CMGD=");
              
              char index_char[3];
              memset(index_char, 0, sizeof(index_char));
              int index_char_int = 0;

              index_char_int = index;
              snprintf(index_char, 3, "%d",(unsigned char)index_char_int);
              index_char[2] = '\0';

              send(index_char);

              send("\r");
            }

            if (waitForATResponse()) {
              // mavio::log(LOG_INFO, "ok");
            }
            inbox_empty = false;
            return GSM_SUCCESS;
        }
      }
    }  
  }
  mavio::log(LOG_WARNING, "SMS: Read sms list timeout");
  return GSM_PROTOCOL_ERROR;
}

int SMS::internaldeleteSMSlist(void) {
  // index 0 ( does not matter ), flag 4, delete all messages
  send("AT+CMGD=0,4\r");
  
  if (!waitForATResponse()) {
      return cancelled() ? GSM_CANCELLED : GSM_PROTOCOL_ERROR;
    }

  // mavio::log(LOG_DEBUG, "SMS delete message went right");
  return GSM_SUCCESS;

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

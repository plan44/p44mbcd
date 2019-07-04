//
//  main.cpp
//  p44mbutil
//
//  Created by Lukas Zeller on 2019-04-26
//  Copyright (c) 2019 plan44.ch. All rights reserved.
//

#include "application.hpp"

#include "macaddress.hpp"
#include "modbus.hpp"
#include "utils.hpp"

#include <stdio.h>

#define DEFAULT_MODBUS_RTU_PARAMS "9600,8,N,1" // [baud rate][,[bits][,[parity][,[stopbits][,[H]]]]]
#define DEFAULT_MODBUS_IP_PORT 1502

#define ENABLE_IRQTEST 1

using namespace p44;


class P44mbutil : public CmdLineApp
{
  typedef CmdLineApp inherited;

  // modbus
  ModbusMaster modBus;

public:

  P44mbutil()
  {
    modBus.isMemberVariable();
  }

  virtual int main(int argc, char **argv)
  {
    const char *usageText =
      "Usage: %1$s [options] <command> [<commandarg>...]\n"
      "Commands:\n"
      "  readreg <regno> [<count>]  : read from modbus register(s)\n"
      "  writereg <regno> <value>   : write value to modbus register\n";
    const CmdLineOptionDescriptor options[] = {
      { 'c', "connection",      true,  "connspec;serial interface for RTU or IP address for TCP (/device or IP[:port])" },
      { 0  , "rs485txenable",   true,  "pinspec;a digital output pin specification for TX driver enable, 'RTS' or 'RS232'" },
      { 0  , "rs485txdelay",    true,  "delay;delay of tx enable signal in uS" },
      { 'd', "debugmodbus",     false, "enable libmodbus debug messages to stderr" },
      { 's', "slave",           true,  "slave;slave to address (0 for broadcast)" },
      CMDLINE_APPLICATION_LOGOPTIONS,
      CMDLINE_APPLICATION_STDOPTIONS,
      { 0, NULL } // list terminator
    };

    // parse the command line, exits when syntax errors occur
    setCommandDescriptors(usageText, options);
    parseCommandLine(argc, argv);
    processStandardLogOptions(false); // command line utility defaults, not daemon

    // app now ready to run
    return run();
  }


  virtual void initialize()
  {
    // init modbus
    string mbconn;
    if (!getStringOption("connection", mbconn)) {
      terminateAppWith(TextError::err("must specify --connection"));
      return;
    }
    string txen;
    getStringOption("rs485txenable", txen);
    int txDelayUs = Never;
    getIntOption("rs485txdelay", txDelayUs);
    ErrorPtr err = modBus.setConnectionSpecification(mbconn.c_str(), DEFAULT_MODBUS_IP_PORT, DEFAULT_MODBUS_RTU_PARAMS, txen.c_str(), txDelayUs);
    if (!Error::isOK(err)) {
      terminateAppWith(err->withPrefix("Invalid modbus connection: "));
      return;
    }
    int slave = 1;
    getIntOption("slave", slave);
    modBus.setSlaveAddress(slave);
    modBus.setDebug(getOption("debugmodbus"));
    // now execute commands
    terminateAppWith(executeCommands());
  }


  ErrorPtr executeCommands()
  {
    string cmd;
    if (!getStringArgument(0, cmd)) return TextError::err("missing command");
    if (cmd=="readreg") {
      int reg;
      if (!getIntArgument(1, reg) || reg<0 || reg>0xFFFF) return TextError::err("missing or invalid register address");
      uint16_t val;
      int cnt = 1;
      if (getIntArgument(2, cnt) && (cnt<1 || cnt>125)) return TextError::err("invalid register count (1..125 allowed)");
      uint16_t vals[125];
      ErrorPtr err = modBus.readRegisters(reg, cnt, vals);
      if (Error::isOK(err)) {
        for (int i=0; i<cnt; i++) {
          printf("Register %5d = %5d (0x%04x)\n", reg+i, vals[i], vals[i]);
        }
      }
      return err;
    }
    else if (cmd=="writereg") {
      int reg;
      if (!getIntArgument(1, reg) || reg<0 || reg>0xFFFF) return TextError::err("missing or invalid register address");
      int val;
      if (!getIntArgument(2, val) || val<0 || val>0xFFFF) return TextError::err("missing or invalid register value");
      return modBus.writeRegister(reg, (uint16_t)val);
    }
    else if (cmd=="readbit") {
      int reg;
      if (!getIntArgument(1, reg) || reg<0 || reg>0xFFFF) return TextError::err("missing or invalid register address");
      uint16_t val;
      int cnt = 1;
      if (getIntArgument(2, cnt) && (cnt<1 || cnt>125)) return TextError::err("invalid register count (1..125 allowed)");
      uint16_t vals[125];
      ErrorPtr err = modBus.readRegisters(reg, cnt, vals);
      if (Error::isOK(err)) {
        for (int i=0; i<cnt; i++) {
          printf("Register %5d = %5d (0x%04x)\n", reg+i, vals[i], vals[i]);
        }
      }
      return err;
    }
    else if (cmd=="writereg") {
      int reg;
      if (!getIntArgument(1, reg) || reg<0 || reg>0xFFFF) return TextError::err("missing or invalid register address");
      int val;
      if (!getIntArgument(2, val) || val<0 || val>0xFFFF) return TextError::err("missing or invalid register value");
      return modBus.writeRegister(reg, (uint16_t)val);
    }
    return TextError::err("unknown command '%s'", cmd.c_str());
  }





  // MARK: - ubus API

  #if asgfhdjsagfhdsjagfshjadgfsadhjk

  #define MAX_REG 64

  void initUbusApi()
  {
    ubusApiServer = UbusServerPtr(new UbusServer(MainLoop::currentMainLoop()));
    UbusObjectPtr u = new UbusObject("P44mbutil", boost::bind(&P44mbutil::ubusApiRequestHandler, this, _1, _2, _3));
    u->addMethod("log", logapi_policy);
    u->addMethod("api", p44mbcapi_policy);
    u->addMethod("quit");
    u->addMethod("buttonup");
    u->addMethod("buttondown");
    ubusApiServer->registerObject(u);
  }

  void ubusApiRequestHandler(UbusRequestPtr aUbusRequest, const string aMethod, JsonObjectPtr aJsonRequest)
  {
    if (aMethod=="log") {
      if (aJsonRequest) {
        JsonObjectPtr o;
        if (aJsonRequest->get("level", o)) {
          int oldLevel = LOGLEVEL;
          int newLevel = o->int32Value();
          SETLOGLEVEL(newLevel);
          LOG(newLevel, "\n\n========== changed log level from %d to %d ===============", oldLevel, newLevel);
        }
        if (aJsonRequest->get("deltastamps", o)) {
          SETDELTATIME(o->boolValue());
        }
      }
      aUbusRequest->sendResponse(JsonObjectPtr());
    }
    else if (aMethod=="buttonup") {
      buttonPressed(1);
      aUbusRequest->sendResponse(JsonObjectPtr());
    }
    else if (aMethod=="buttondown") {
      buttonPressed(2);
      aUbusRequest->sendResponse(JsonObjectPtr());
    }
    else if (aMethod=="quit") {
      LOG(LOG_WARNING, "terminated via UBUS quit method");
      terminateApp(1);
      aUbusRequest->sendResponse(JsonObjectPtr());
    }
    else if (aMethod=="api") {
      ErrorPtr err;
      JsonObjectPtr result;
      if (aJsonRequest) {
        JsonObjectPtr o;
        if (aJsonRequest->get("modbus", o)) {
          // modbus commands
          string cmd = o->stringValue();
          if (cmd=="debug_on") {
            modBus.setDebug(true);
          }
          else if (cmd=="debug_off") {
            modBus.setDebug(false);
          }
          else if (cmd=="read_registers") {
            int reg = -1;
            int numReg = 1;
            int slave = -1;
            if (aJsonRequest->get("slave", o)) slave = o->int32Value();
            if (aJsonRequest->get("reg", o)) reg = o->int32Value();
            if (aJsonRequest->get("count", o)) numReg = o->int32Value();
            if (reg<0 || numReg<1 || numReg>=MAX_REG || slave<0) {
              err = TextError::err("invalid reg=%d, count=%d, slave=%d combination", reg, numReg, slave);
            }
            else {
              uint16_t tab_reg[MAX_REG];
              modBus.setSlaveAddress(slave);
              err = modBus.readRegisters(reg, numReg, tab_reg);
              if (Error::isOK(err)) {
                result = JsonObject::newArray();
                for (int i=0; i<numReg; i++) {
                  result->arrayAppend(JsonObject::newInt32(tab_reg[i]));
                }
              }
            }
          }
          else if (cmd=="write_registers") {
            int reg = -1;
            int slave = -1;
            if (aJsonRequest->get("slave", o)) slave = o->int32Value();
            if (aJsonRequest->get("reg", o)) reg = o->int32Value();
            int numReg = 0;
            uint16_t tab_reg[MAX_REG];
            if (aJsonRequest->get("values", o)) {
              if (o->isType(json_type_array)) {
                // multiple
                for(int i=0; i<o->arrayLength(); i++) {
                  tab_reg[i] = o->arrayGet(i)->int32Value();
                  numReg++;
                }
              }
              else {
                // single
                numReg = 1;
                tab_reg[0] = o->int32Value();
              }
            }
            if (reg<0 || numReg<1 || numReg>=MAX_REG || slave<0) {
              err = TextError::err("invalid reg=%d, values=[%d], slave=%d combination", reg, numReg, slave);
            }
            else {
              modBus.setSlaveAddress(slave);
              err = modBus.writeRegisters(reg, numReg, tab_reg);
              if (Error::isOK(err)) {
                result = JsonObject::newBool(true);
              }
            }
          }
          else {
            err = TextError::err("unknown modbus command");
          }
        }
      }
      else {
        err = TextError::err("missing command object");
      }
      JsonObjectPtr response = JsonObject::newObj();
      if (result) response->add("result", result);
      if (err) response->add("error", JsonObject::newString(err->description()));
      aUbusRequest->sendResponse(response);
    }
    else {
      // no other methods implemented yet
      aUbusRequest->sendResponse(JsonObjectPtr(), UBUS_STATUS_INVALID_COMMAND);
    }
  }

  #endif // erwzuirzqwiurzqwuiro

};



int main(int argc, char **argv)
{
  // prevent all logging until command line determines level
  SETLOGLEVEL(LOG_EMERG);
  SETERRLEVEL(LOG_EMERG, false); // messages, if any, go to stderr

  // create app with current mainloop
  P44mbutil *application = new(P44mbutil);
  // pass control
  int status = application->main(argc, argv);
  // done
  delete application;
  return status;
}


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

#define DEFAULT_MODBUS_RTU_PARAMS "115200,8,N,1" // [baud rate][,[bits][,[parity][,[stopbits][,[H]]]]]
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
      "  writereg <regno> <value>   : write value to modbus register\n"
      "  readinfo                   : read slave info\n";
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
      terminateAppWith(TextError::err("must specify connection"));
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
    int slave;
    if (!getIntOption("slave", slave)) {
      terminateAppWith(TextError::err("must specify slave address"));
      return;
    }
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
      if (!getIntArgument(1, reg) || reg<0 || reg>0xFFFF) return TextError::err("missing or invalid bit address");
      int cnt = 1;
      if (getIntArgument(2, cnt) && (cnt<1 || cnt>125)) return TextError::err("invalid bit count (1..125 allowed)");
      uint16_t vals[125];
      ErrorPtr err = modBus.readRegisters(reg, cnt, vals);
      if (Error::isOK(err)) {
        for (int i=0; i<cnt; i++) {
          printf("Register %5d = %5d (0x%04x)\n", reg+i, vals[i], vals[i]);
        }
      }
      return err;
    }
    else if (cmd=="writebit") {
      int reg;
      if (!getIntArgument(1, reg) || reg<0 || reg>0xFFFF) return TextError::err("missing or invalid bit address");
      int val;
      if (!getIntArgument(2, val) || val<0 || val>1) return TextError::err("missing or invalid bit value");
      return modBus.writeRegister(reg, (uint16_t)val);
    }
    else if (cmd=="readinfo") {
      string id;
      bool runIndicator;
      ErrorPtr err = modBus.readSlaveInfo(id, runIndicator);
      if (Error::isOK(err)) {
        printf("Run indicator = %s, Slave ID = '%s'\n", runIndicator ? "ON" : "OFF", id.c_str());
      }
      return err;
    }
    return TextError::err("unknown command '%s'", cmd.c_str());
  }

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


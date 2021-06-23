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
      "  read <addr> [<count>]                 : read from modbus register(s) / bit(s)\n"
      "  write <addr> <value>                  : write value to modbus register/bit\n"
      "  monitor <addr> [<interval in ms>]     : monitor (constantly poll) register/bit, default interval = 200mS\n"
      "  readinfo                              : read slave info\n"
      "  flush                                 : just flush the communication channel and display number of bytes flushed\n"
      "  scan [<from> <to>]                    : scan for slaves on the bus by querying slave info\n"
      "  sendfile <path> <fileno> [<dest>...]  : send file to destination (<dest> can be ALL, idMatch or slave addresses)\n"
      "  getfile <path> <fileno>               : get file from slave\n";
    const CmdLineOptionDescriptor options[] = {
      { 'i', "input",           false, "read input-only register / bit" },
      { 'b', "bit",             false, "access bit (not register)" },
      { 0  , "linkrecovery",    false, "enable link level modbus recovery" },
      { 0  , "protocolrecovery",false, "enable protocol level modbus recovery" },
      { 0  , "bit",             false, "access bit (not register)" },
      { 0  , "verify",          false, "verify write access by reading value back immediately" },
      { 'c', "connection",      true,  "connspec;serial interface for RTU or IP address for TCP (/device or IP[:port])" },
      { 0  , "rs485txenable",   true,  "pinspec;a digital output pin specification for TX driver enable, 'RTS' or 'RS232'" },
      { 0  , "rs485txdelay",    true,  "delay;delay of tx enable signal in uS" },
      { 0  , "rs485rxenable",   true,  "pinspec;a digital output pin specification for RX input enable" },
      { 0  , "bytetime",        true,  "time;custom time per byte in nS" },
      { 0  , "stdmodbusfiles",  false, "disable p44 file handling, just use standard modbus file record access" },
      { 0  , "debugmodbus",     false, "enable libmodbus debug messages to stderr" },
      { 's', "slave",           true,  "slave;slave to address (default=1)" },
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
    int byteTimeNs = 0;
    getIntOption("bytetime", byteTimeNs);
    int recoveryMode = MODBUS_ERROR_RECOVERY_NONE;
    if (getOption("linkrecovery")) recoveryMode |= MODBUS_ERROR_RECOVERY_LINK;
    if (getOption("protocolrecovery")) recoveryMode |= MODBUS_ERROR_RECOVERY_PROTOCOL;
    ErrorPtr err = modBus.setConnectionSpecification(
      mbconn.c_str(),
      DEFAULT_MODBUS_IP_PORT, DEFAULT_MODBUS_RTU_PARAMS,
      txen.c_str(), txDelayUs,
      getOption("rs485rxenable"), // can be NULL if there is no separate rx enable
      byteTimeNs,
      (modbus_error_recovery_mode)recoveryMode
    );
    if (Error::notOK(err)) {
      terminateAppWith(err->withPrefix("Invalid modbus connection: "));
      return;
    }
    int slave = 1;
    getIntOption("slave", slave);
    modBus.setSlaveAddress(slave);
    modBus.setDebug(getOption("debugmodbus"));
    // now execute commands
    err = executeCommands();
    terminateAppWith(err);
  }


  ErrorPtr executeCommands()
  {
    string cmd;
    if (!getStringArgument(0, cmd)) return TextError::err("missing command");
    bool isInput = getOption("input");
    bool isBit = getOption("bit");
    if (cmd=="read") {
      ErrorPtr err;
      int addr;
      if (!getIntArgument(1, addr) || addr<0 || addr>0xFFFF) return TextError::err("missing or invalid address");
      int cnt = 1;
      if (getIntArgument(2, cnt) && (cnt<1 || cnt>125)) return TextError::err("invalid count (1..125 allowed)");
      if (isBit) {
        uint8_t bits[125];
        ErrorPtr err = modBus.readBits(addr, cnt, bits, isInput);
        if (Error::isOK(err)) {
          for (int i=0; i<cnt; i++) {
            printf("%s bit %5d = %d\n", isInput ? "Input" : "Coil", addr+i, bits[i]);
          }
        }
      }
      else {
        uint16_t vals[125];
        err = modBus.readRegisters(addr, cnt, vals, isInput);
        if (Error::isOK(err)) {
          for (int i=0; i<cnt; i++) {
            printf("%s register %5d = %5d (0x%04x)\n", isInput ? "Input" : "R/W", addr+i, vals[i], vals[i]);
          }
        }
      }
      return err;
    }
    else if (cmd=="monitor") {
      int addr;
      if (!getIntArgument(1, addr) || addr<0 || addr>0xFFFF) return TextError::err("missing or invalid address");
      int interval = 200;
      if (getIntArgument(2, interval) && (interval<10)) return TextError::err("invalid interval (>=10mS allowed)");
      while (!isTerminated()) {
        ErrorPtr err;
        if (isBit) {
          bool bitVal;
          err = modBus.readBit(addr, bitVal, isInput);
          if (Error::isOK(err)) printf("%s bit %4d: %d\r", isInput ? "Input" : "Coil", addr, bitVal);
        }
        else {
          uint16_t regVal;
          err = modBus.readRegister(addr, regVal, isInput);
          if (Error::isOK(err)) printf("%s Register %4d: %5hd (0x%04hX)\r", isInput ? "Input" : "R/W", addr, regVal, regVal);
        }
        if (Error::notOK(err)) {
          printf("Register %4d: error: %s\r", addr, err->text());
        }
        fflush(stdout);
        MainLoop::sleep(interval*MilliSecond);
      }
      printf("\r\n");
      return ErrorPtr();
    }
    else if (cmd=="write") {
      int addr;
      if (!getIntArgument(1, addr) || addr<0 || addr>0xFFFF) return TextError::err("missing or invalid address");
      int val;
      if (!getIntArgument(2, val) || val<0 || val>0xFFFF) return TextError::err("missing or invalid value");
      ErrorPtr err;
      bool verify = getOption("verify");
      if (isBit) {
        err = modBus.writeBit(addr, (bool)val);
        if (Error::isOK(err) && verify) {
          bool vval;
          err = modBus.readBit(addr, vval);
          if (Error::isOK(err) && (vval!=(bool)val)) {
            err = TextError::err("Bit write verification failed, written %d, read back %d", val, vval);
          }
        }
        return err;
      }
      else {
        err = modBus.writeRegister(addr, (uint16_t)val);
        if (Error::isOK(err) && verify) {
          uint16_t vval;
          err = modBus.readRegister(addr, vval);
          if (Error::isOK(err) && (vval!=(uint16_t)val)) {
            err = TextError::err("Reg write verification failed, written %d, read back %d", val, vval);
          }
        }
        return err;
      }
    }
    else if (cmd=="readinfo") {
      string id;
      bool runIndicator;
      ErrorPtr err = modBus.readSlaveInfo(id, runIndicator);
      if (Error::isOK(err)) {
        printf("Slave ID = '%s', Run indicator = %s\n", id.c_str(), runIndicator ? "ON" : "OFF");
      }
      return err;
    }
    else if (cmd=="scan") {
      printf("Scanning for modbus slave devices...\n");
      ModbusMaster::SlaveAddrList slaves;
      int first = 1;
      int last = 10;
      getIntArgument(1, first);
      getIntArgument(2, last);
      if (first<1 || last>255 || first>last) return TextError::err("invalid scan range (1..255 allowed)");
      for (int sa = first; sa<=last; sa++) {
        modBus.setSlaveAddress(sa);
        string id;
        bool runIndicator;
        ErrorPtr err = modBus.readSlaveInfo(id, runIndicator);
        if (Error::isOK(err)) {
          printf("+ Slave %3d : ID = '%s', Run indicator = %s\n", sa, id.c_str(), runIndicator ? "ON" : "OFF");
        }
        else if (Error::isError(err, ModBusError::domain(), ETIMEDOUT)) {
          printf("- Slave %3d : no answer\n", sa);
        }
        else {
          printf("! Slave %3d : error: %s\n", sa, err->text());
        }
      }
      printf("Scan complete...\n");
      return ErrorPtr();
    }
    else if (cmd=="sendfile") {
      string path;
      if (!getStringArgument(1, path)) return TextError::err("missing file path");
      int fileNo;
      if (!getIntArgument(2, fileNo) || fileNo<1 || fileNo>0xFFFF) return TextError::err("missing or invalid file number");
      ModbusMaster::SlaveAddrList slaves;
      if (numArguments()>3) {
        // parse destination: ALL, idmatch or several slave addresses
        string dest;
        int argidx = 3;
        while(getStringArgument(argidx, dest)) {
          int saddr;
          if (sscanf(dest.c_str(), "%d", &saddr)==1) {
            slaves.push_back(saddr);
          }
          else if (argidx>3) {
            return TextError::err("invalid list of slave addresses");
          }
          else {
            // argidx==2 can also be "ALL" or a id string match
            if (dest=="ALL") dest.clear(); // no id match requirement
            modBus.findSlaves(slaves, dest);
            break;
          }
          argidx++;
        }
        if (slaves.size()<1) return TextError::err("no slave to send file to");
        return modBus.broadcastFile(slaves, path, fileNo, !getOption("stdmodbusfiles"));
      }
      else {
        // use standard, non-broadcast transfer
        return modBus.sendFile(path, fileNo, !getOption("stdmodbusfiles"));
      }
    }
    else if (cmd=="getfile") {
      string path;
      if (!getStringArgument(1, path)) return TextError::err("missing file path");
      int fileNo;
      if (!getIntArgument(2, fileNo) || fileNo<1 || fileNo>0xFFFF) return TextError::err("missing or invalid file number");
      return modBus.receiveFile(path, fileNo, !getOption("stdmodbusfiles"));
    }
    else if (cmd=="flush") {
      modBus.connect(false); // prevent implicit flush
      int f = modBus.flush(); // now explicitly flush
      printf("Flushed %d bytes\n", f);
      modBus.close();
      return ErrorPtr();
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


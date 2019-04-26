//
//  main.cpp
//  p44mbc
//
//  Created by Lukas Zeller on 2019-04-26
//  Copyright (c) 2019 plan44.ch. All rights reserved.
//

#include "application.hpp"

#include "macaddress.hpp"
#include "digitalio.hpp"
#include "utils.hpp"

#include <stdio.h>

#define DEFAULT_LOGLEVEL LOG_NOTICE

using namespace p44;


class P44mbcd : public CmdLineApp
{
  typedef CmdLineApp inherited;

public:

  P44mbcd()
  {
  }


  virtual int main(int argc, char **argv)
  {
    const char *usageText =
    "Usage: %1$s [options]\n";
    const CmdLineOptionDescriptor options[] = {
      { 0  , "rs485connection", true,  "serial_if;RS485 serial interface where display is connected (/device or IP:port)" },
      { 0  , "rs485txenable",   true,  "pinspec;a digital output pin specification for TX driver enable or DTR or RTS" },
      { 0  , "rs485txoffdelay", true,  "delay;time to keep tx enabled after sending [ms], defaults to 0" },
      { 0  , "rs485rxenable",   true,  "pinspec;a digital output pin specification for RX driver enable" },
      { 0  , "ubusapi",         false, "enable ubus api" },
      { 'l', "loglevel",        true,  "level;set max level of log message detail to show on stderr" },
      { 0  , "deltatstamps",    false, "show timestamp delta between log lines" },
      { 'V', "version",         false, "show version" },
      { 'h', "help",            false, "show this text" },
      { 0, NULL } // list terminator
    };

    // parse the command line, exits when syntax errors occur
    setCommandDescriptors(usageText, options);
    parseCommandLine(argc, argv);

    if (numOptions()<1) {
      // show usage
      showUsage();
      terminateApp(EXIT_SUCCESS);
    }

    // log level?
    int loglevel = DEFAULT_LOGLEVEL;
    getIntOption("loglevel", loglevel);
    SETLOGLEVEL(loglevel);
    SETERRLEVEL(loglevel, false); // all diagnostics go to stderr
    SETDELTATIME(getOption("deltatstamps"));

    // app now ready to run
    return run();
  }


  // MARK: ===== command line actions

  virtual void initialize()
  {
    LOG(LOG_NOTICE, "p44mbcd. initialize");
  }

};


int main(int argc, char **argv)
{
  // prevent all logging until command line determines level
  SETLOGLEVEL(LOG_EMERG);
  SETERRLEVEL(LOG_EMERG, false); // messages, if any, go to stderr
  // create app with current mainloop
  P44mbcd *application = new(P44mbcd);
  // pass control
  int status = application->main(argc, argv);
  // done
  delete application;
  return status;
}


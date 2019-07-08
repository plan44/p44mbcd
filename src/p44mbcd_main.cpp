//
//  main.cpp
//  p44mbc
//
//  Created by Lukas Zeller on 2019-04-26
//  Copyright (c) 2019 plan44.ch. All rights reserved.
//

#include "application.hpp"

#include "macaddress.hpp"
#include "modbus.hpp"
#include "utils.hpp"

#if ENABLE_UBUS
  #include "ubus.hpp"
#endif

#include <stdio.h>

#include "lvgl/lvgl.h"

#if defined(__APPLE__)
  // test&debugging build on MacOS, using SDL2 for graphics
  // - little vGL
  #include "lv_drivers/display/monitor.h"
  #include "lv_drivers/indev/mouse.h"
#else
  // target platform build
  // - little vGL
  #include "lv_drivers/display/fbdev.h"
  #include "lv_drivers/indev/evdev.h"
#endif

// FIXME: remove later
#include "lv_examples/lv_apps/demo/demo.h"


#define DEFAULT_MODBUS_RTU_PARAMS "115200,8,N,1" // [baud rate][,[bits][,[parity][,[stopbits][,[H]]]]]
#define DEFAULT_MODBUS_IP_PORT 1502

#define ENABLE_IRQTEST 1

using namespace p44;


#if ENABLE_UBUS
static const struct blobmsg_policy logapi_policy[] = {
  { .name = "level", .type = BLOBMSG_TYPE_INT8 },
  { .name = "deltastamps", .type = BLOBMSG_TYPE_BOOL },
  { .name = NULL, .type = BLOBMSG_TYPE_INT32 },
};

static const struct blobmsg_policy p44mbcapi_policy[] = {
  { .name = "method", .type = BLOBMSG_TYPE_STRING },
  { .name = NULL, .type = BLOBMSG_TYPE_UNSPEC },
};
#endif


class P44mbcd : public CmdLineApp
{
  typedef CmdLineApp inherited;

  #if ENABLE_UBUS
  UbusServerPtr ubusApiServer; ///< ubus API for openwrt web interface
  #endif
  #if ENABLE_IRQTEST
  DigitalIoPtr irqTestPin;
  #endif

  // littlevGL
  lv_disp_t *dispdev; ///< the display device
  lv_indev_t *pointer_indev; ///< the input device for pointer (touch, mouse)
  lv_indev_t *keyboard_indev; ///< the input device for keyboard
  MLTicket lvglTicket; ///< the display tasks timer
  MLMicroSeconds lastLvglTick; ///< last tick
  bool showCursor = false;

  // modbus
  ModbusSlave modBus;

  // app
  MLTicket appTicket;
  bool testmode;

public:

  P44mbcd() :
    dispdev(NULL),
    pointer_indev(NULL),
    keyboard_indev(NULL),
    lastLvglTick(Never)
  {
    modBus.isMemberVariable();
  }

  virtual int main(int argc, char **argv)
  {
    const char *usageText =
    "Usage: %1$s [options]\n";
    const CmdLineOptionDescriptor options[] = {
      { 0  , "connection",      true,  "connspec;serial interface for RTU or IP address for TCP (/device or IP[:port])" },
      { 0  , "rs485txenable",   true,  "pinspec;a digital output pin specification for TX driver enable, 'RTS' or 'RS232'" },
      { 0  , "rs485txdelay",    true,  "delay;delay of tx enable signal in uS" },
      { 0  , "slave",           true,  "slave;use this slave by default" },
      { 'd', "debugmodbus",     false, "enable libmodbus debug messages to stderr" },
      { 0  , "mousecursor",     false, "show mouse cursor" },
      { 0  , "testmode",        false, "FCU test mode" },
      #if ENABLE_IRQTEST
      { 0  , "irqtest",         true,  "pinspec;IRQ input test" },
      #endif
      #if ENABLE_UBUS
      { 0  , "ubusapi",         false, "enable ubus API" },
      #endif
      CMDLINE_APPLICATION_LOGOPTIONS,
      CMDLINE_APPLICATION_STDOPTIONS,
      { 0, NULL } // list terminator
    };

    // parse the command line, exits when syntax errors occur
    setCommandDescriptors(usageText, options);
    parseCommandLine(argc, argv);
    processStandardLogOptions(true); // daemon defaults

    if (numOptions()<1) {
      // show usage
      showUsage();
      terminateApp(EXIT_SUCCESS);
    }

    #if ENABLE_UBUS
    // Prepare ubus API
    if (getOption("ubusapi")) {
      initUbusApi();
    }
    #endif

    #if ENABLE_IRQTEST
    string spec;
    if (getStringOption("irqtest", spec)) {
      irqTestPin = DigitalIoPtr(new DigitalIo(spec.c_str(), false, false));
      irqTestPin->setInputChangedHandler(boost::bind(&P44mbcd::irqTestHandler, this, _1), 0, Infinite);
    }
    #endif


    // app now ready to run
    return run();
  }


  #if ENABLE_IRQTEST
  void irqTestHandler(bool aNewState)
  {
    LOG(LOG_NOTICE, "New state is %d", aNewState);
  }
  #endif

  // MARK: - ubus API

  #if ENABLE_UBUS

  #define MAX_REG 64

  void initUbusApi()
  {
    ubusApiServer = UbusServerPtr(new UbusServer(MainLoop::currentMainLoop()));
    UbusObjectPtr u = new UbusObject("p44mbcd", boost::bind(&P44mbcd::ubusApiRequestHandler, this, _1, _2, _3));
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

  #endif // ENABLE_UBUS


  // MARK: - initialisation

  virtual void initialize()
  {
    LOG(LOG_NOTICE, "p44mbcd: initialize");
    #if ENABLE_UBUS
    // start ubus API, if we have it
    if (ubusApiServer) {
      ubusApiServer->startServer();
    }
    #endif
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
    modBus.setSlaveId(string_format("p44mbc %s %06llX", version().c_str(), macAddress()));
    modBus.setDebug(getOption("debugmodbus"));
    modBus.setRegisterModel(
      0, 0, // coils
      0, 0, // input bits
      101, 129-101+1, // registers
      0, 0 // input registers
    );
    modBus.setValueAccessHandler(boost::bind(&P44mbcd::modbusValueAccessHandler, this, _1, _2, _3, _4));
    err = modBus.connect();
    if (!Error::isOK(err)) {
      terminateAppWith(err->withPrefix("Failed to start modbus slave server: "));
      return;
    }
    // start littlevGL
    initLvgl();
//    // start app
//    if (!isTerminated())
//      initApp();
  }



  ErrorPtr modbusValueAccessHandler(int aAddress, bool aBit, bool aInput, bool aWrite)
  {
    uint16_t val = modBus.getValue(aAddress, aBit, aInput);
    LOG(LOG_NOTICE,
      "%s%s %d accessed for %s, value = %d (0x%04X)",
      aInput ? "Readonly " : "",
      aBit ? "Bit" : "Register",
      aAddress,
      aWrite ? "write" : "read",
      val, val
    );
    if (aAddress==128) {
      return TextError::err("Special register 128 not yet implemented");
    }
    return ErrorPtr();
  }


  // MARK: - app logic

  #if OLDTESTCODEENABLED

  uint16_t reg104 = 0;
  uint16_t reg202 = 0;

  uint16_t reg137 = -1;

  static const char *reg104Texts(int aReg)
  {
    switch (aReg) {
      case 0 : return "AUS";
      case 1 : return "Auto";
      case 2 : return "Heizen";
      case 3 : return "Kühlen";
      case 4 : return "Ventilation";
      default : return "<unbekannt>";
    }
  }

  static const char *reg202Texts(int aReg)
  {
    switch (aReg) {
      case 0 : return "AUS";
      case 1 : return "Manuell 1";
      case 2 : return "Manuell 2";
      case 3 : return "Manuell 3";
      case 4 : return "Auto 1";
      case 5 : return "Auto 2";
      case 6 : return "Auto 3";
      default : return "<unbekannt>";
    }
  }


  void initApp()
  {
    testmode = getOption("testmode");
    // get status
    modBus.readRegister(104, reg104);
    modBus.readRegister(202, reg202);
    // start in normal mode
    reg137 = 0;
    modBus.writeRegister(137, reg137);
    // repeatedly
    appTicket.executeOnce(boost::bind(&P44mbcd::appTask, this, _1, _2));
  }


  #define APP_TASK_PERIOD (2*Second)

  void appTask(MLTimer &aTimer, MLMicroSeconds aNow)
  {
    ErrorPtr err = modBus.readRegister(104, reg104);
    if (Error::isOK(err)) {
      if (testmode) {
        if (reg104==1 || reg104==2) {
          // heating
          if (reg137!=1) {
            reg137 = 1; // heating test
            modBus.writeRegister(137, reg137);
          }
        }
        else if (reg104==3 || reg104==4) {
          // cooling
          if (reg137!=2) {
            reg137 = 2; // cooling test
            modBus.writeRegister(137, reg137);
          }
        }
        else if (reg104==0) {
          reg137 = 0; // off
          modBus.writeRegister(137, reg137);
        }
      }
      modBus.readRegister(202, reg202);
      update_display();
    }
    else {
      // error
      demo_setNewText(err->description().c_str());
    }
    // again
    MainLoop::currentMainLoop().retriggerTimer(aTimer, APP_TASK_PERIOD);
  }


  void update_display()
  {
    string s = string_format(
      "Mode: %s [%d]\nFan: %s [%d]",
      reg104Texts(reg104), reg104,
      reg202Texts(reg202), reg202
    );
    demo_setNewText(s.c_str());
  }


  void buttonPressed(int aButtonId)
  {
    // id 1 = plus
    // id 2 = minus
    if (aButtonId==1) {
      if (reg104<4) {
        reg104++;
      }
      modBus.writeRegister(104, reg104);
      update_display();
    }
    if (aButtonId==2) {
      if (reg104>0) {
        reg104--;
      }
      modBus.writeRegister(104, reg104);
      update_display();
    }
  }

  #endif


  // MARK: - littlevGL

  #define SHOW_MOUSE_CURSOR 1

  static void demoButtonPressed(int aButtonId)
  {
    P44mbcd *p44mbcd = dynamic_cast<P44mbcd *>(Application::sharedApplication());
    #if OLDTESTCODEENABLED
    p44mbcd->buttonPressed(aButtonId);
    #endif
  }


  void initLvgl()
  {
    LOG(LOG_NOTICE, "initializing littlevGL");
    showCursor = getOption("mousecursor");
    // - init library
    lv_init();
    #if defined(__APPLE__)
    // - init display driver
    monitor_init();
    lv_disp_drv_t disp_drv;
    lv_disp_drv_init(&disp_drv);
    disp_drv.disp_flush = monitor_flush;
    disp_drv.disp_fill = monitor_fill;
    disp_drv.disp_map = monitor_map;
    dispdev = lv_disp_drv_register(&disp_drv);
    // - init input driver
    mouse_init();
    lv_indev_drv_t pointer_indev_drv;
    lv_indev_drv_init(&pointer_indev_drv);
    pointer_indev_drv.type = LV_INDEV_TYPE_POINTER;
    pointer_indev_drv.read = mouse_read;
    pointer_indev = lv_indev_drv_register(&pointer_indev_drv);  /*Register the driver in LittlevGL*/
    #else
    // - init display driver
    fbdev_init();
    lv_disp_drv_t disp_drv;
    lv_disp_drv_init(&disp_drv);
    disp_drv.disp_flush = fbdev_flush;      /*It flushes the internal graphical buffer to the frame buffer*/
    dispdev = lv_disp_drv_register(&disp_drv);
    // - init input driver
    evdev_init();
    lv_indev_drv_t pointer_indev_drv;
    lv_indev_drv_init(&pointer_indev_drv);
    pointer_indev_drv.type = LV_INDEV_TYPE_POINTER;
    pointer_indev_drv.read = evdev_read;
    pointer_indev = lv_indev_drv_register(&pointer_indev_drv);  /*Register the driver in LittlevGL*/
    if (showCursor) {
      #if SHOW_MOUSE_CURSOR
      lv_obj_t *cursor;
      cursor = lv_obj_create(lv_scr_act(), NULL);
      lv_obj_set_size(cursor, 24, 24);
      static lv_style_t style_round;
      lv_style_copy(&style_round, &lv_style_plain);
      style_round.body.radius = LV_RADIUS_CIRCLE;
      style_round.body.main_color = LV_COLOR_RED;
      style_round.body.opa = LV_OPA_COVER;
      lv_obj_set_style(cursor, &style_round);
      lv_indev_set_cursor(pointer_indev, cursor);
      #endif
    }
    #endif
    // - create demo
    demo_create();
    demo_setNewText("Ready");
    demo_setButtonCallback(demoButtonPressed);
    // - schedule updates
    lvglTicket.executeOnce(boost::bind(&P44mbcd::lvglTask, this, _1, _2));
  }


  #define LVGL_TICK_PERIOD (5*MilliSecond)

  void lvglTask(MLTimer &aTimer, MLMicroSeconds aNow)
  {
    if (lastLvglTick==Never) lastLvglTick=aNow;
    uint32_t ms = (uint32_t)((aNow-lastLvglTick)/1000);
    lv_tick_inc(ms);
    lv_task_handler();
    lastLvglTick = aNow;
    MainLoop::currentMainLoop().retriggerTimer(aTimer, LVGL_TICK_PERIOD);
    #if defined(__APPLE__)
    // also need to update SDL2
    monitor_sdl_refr_core();
    #endif
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


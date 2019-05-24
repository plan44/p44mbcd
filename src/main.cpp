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
#include "serialcomm.hpp"
#include "utils.hpp"

#if ENABLE_UBUS
  #include "ubus.hpp"
#endif

#include <stdio.h>

// little vGL
#include "lvgl/lvgl.h"
#include "lv_drivers/display/fbdev.h"
#include "lv_drivers/indev/evdev.h"

// FIXME: remove later
#include "lv_examples/lv_apps/demo/demo.h"

// modbus
#include <modbus/modbus-rtu.h>


#define DEFAULT_LOGLEVEL LOG_NOTICE
#define DEFAULT_MODBUS_RTU_PARAMS "9600,8,N,1" // [baud rate][,[bits][,[parity][,[stopbits][,[H]]]]]
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


class ModBusError : public Error
{
public:
  static const char *domain() { return "Modbus"; }
  virtual const char *getErrorDomain() const { return ModBusError::domain(); };
  ModBusError(ErrorCode aError) : Error(aError, modbus_strerror((int)aError)) {};
};


extern "C" {
  void setRts(modbus_t *ctx, int on);
}


class P44mbcd : public CmdLineApp
{
  typedef CmdLineApp inherited;

  friend void setRts(modbus_t *ctx, int on);


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
  int slave = 1;
  modbus_t *modbus;
  DigitalIoPtr modbusRTSEnable;

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
  }

  virtual int main(int argc, char **argv)
  {
    const char *usageText =
    "Usage: %1$s [options]\n";
    const CmdLineOptionDescriptor options[] = {
      { 0  , "connection",      true,  "connspec;serial interface for RTU or IP address for TCP (/device or IP[:port])" },
      { 0  , "rs485txenable",   true,  "pinspec;a digital output pin specification for TX driver enable or DTR or RTS" },
      { 0  , "rs485rtsdelay",   true,  "delay;delay of tx enable signal (RTS) in uS" },
      { 0  , "rs232",           false, "use RS-232 for RTU" },
      { 0  , "slave",           true,  "slave;use this slave by default" },
      { 0  , "mousecursor",     false, "show mouse cursor" },
      { 0  , "testmode",        false, "FCU test mode" },
      #if ENABLE_IRQTEST
      { 0  , "irqtest",         true,  "pinspec;IRQ input test" },
      #endif
      #if ENABLE_UBUS
      { 0  , "ubusapi",         false, "enable ubus API" },
      #endif
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
    SETERRLEVEL(LOG_ERR, true); // errors and more serious go to stderr, all log goes to stdout
    SETDELTATIME(getOption("deltatstamps"));

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

  // MARK: ===== ubus API

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
        if (aJsonRequest->get("modbus", o) && modbus) {
          // modbus commands
          string cmd = o->stringValue();
          if (cmd=="debug_on") {
            modbus_set_debug(modbus, 1);
          }
          else if (cmd=="debug_off") {
            modbus_set_debug(modbus, 0);
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
              err = mb_read_registers(slave, reg, numReg, tab_reg);
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
              err = mb_write_registers(slave, reg, numReg, tab_reg);
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


  ErrorPtr mb_read_registers(int slave, int addr, int nb, uint16_t *dest)
  {
    ErrorPtr err;
    if (modbus_set_slave(modbus, slave)<0) {
      err = ModBusError::err<ModBusError>(errno);
    }
    else {
      if (modbus_connect(modbus)<0) {
        err = ModBusError::err<ModBusError>(errno);
      }
      else {
        if (modbus_read_registers(modbus, addr, nb, dest)<0) {
          err = ModBusError::err<ModBusError>(errno);
        }
      }
    }
    modbus_close(modbus);
    return err;
  }


  ErrorPtr mb_write_registers(int slave, int addr, int nb, uint16_t *src)
  {
    ErrorPtr err;
    if (modbus_set_slave(modbus, slave)<0) {
      err = ModBusError::err<ModBusError>(errno);
    }
    else {
      if (modbus_connect(modbus)<0) {
        err = ModBusError::err<ModBusError>(errno);
      }
      else {
        if (modbus_write_registers(modbus, addr, nb, src)<0) {
          err = ModBusError::err<ModBusError>(errno);
        }
      }
    }
    modbus_close(modbus);
    return err;
  }



  #endif

  // MARK: ===== initialisation

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
    initModbus();
    // start littlevGL
    initLvgl();
    // start app
    initApp();
  }


  // MARK: ===== modbus


  void initModbus()
  {
    LOG(LOG_NOTICE, "initializing modbus");
    // init modbus library
    string rs485conn;
    if (!getStringOption("connection", rs485conn)) {
      terminateAppWith(TextError::err("must specify --rs485connection"));
      return;
    }
    bool rs232 = getOption("rs232")!=NULL;
    if (!rs232) {
      string rtsEn;
      if (getStringOption("rs485txenable", rtsEn)) {
        modbusRTSEnable = DigitalIoPtr(new DigitalIo(rtsEn.c_str(), true, false));
      }
      else {
        LOG(LOG_WARNING, "no --rs485txenable specified, RTS of serial port must exist");
      }
    }
    string connectionPath;
    uint16_t connectionPort;
    int baudRate;
    int charSize; // character size 5..8 bits
    bool parityEnable;
    bool evenParity;
    bool twoStopBits;
    bool hardwareHandshake;
    bool rtu = SerialComm::parseConnectionSpecification(
      rs485conn.c_str(), DEFAULT_MODBUS_IP_PORT, DEFAULT_MODBUS_RTU_PARAMS,
      connectionPath,
      baudRate,
      charSize,
      parityEnable,
      evenParity,
      twoStopBits,
      hardwareHandshake,
      connectionPort
    );
    int mberr = 0;
    if (rtu) {
      if (baudRate==0 || connectionPath.empty()) {
        terminateAppWith(TextError::err("invalid RTU connection params"));
        return;
      }
      modbus = modbus_new_rtu(
        connectionPath.c_str(),
        baudRate,
        parityEnable ? (evenParity ? 'E' : 'O') : 'N',
        charSize,
        twoStopBits ? 2 : 1
      );
      if (modbus==0) mberr = errno;
      if (mberr==0) {
        if (rs232) {
          if (modbus_rtu_set_serial_mode(modbus, MODBUS_RTU_RS232)<0) mberr = errno;
        }
        else {
          // set custom RTS if needed (FIRST, otherwise modbus_rtu_set_serial_mode() might fail when TIOCSRS485 does not work)
          if (mberr==0 && modbusRTSEnable) {
            if (modbus_rtu_set_custom_rts(modbus, setRts)<0) mberr = errno;
          }
          if (mberr==0) {
            if (modbus_rtu_set_serial_mode(modbus, MODBUS_RTU_RS485)<0) mberr = errno;
          }
          if (mberr==0) {
            if (modbus_rtu_set_rts(modbus, MODBUS_RTU_RTS_UP)<0) mberr = errno;
          }
        }
        if (mberr==0) {
          int rtsDelayUs;
          if (getIntOption("rs485rtsdelay", rtsDelayUs)) {
            if (modbus_rtu_set_rts_delay(modbus, rtsDelayUs)<0) mberr = errno;
          }
        }
      }
    }
    else {
      if (connectionPath.empty()) {
        terminateAppWith(TextError::err("invalid TCP connection params"));
        return;
      }
      modbus = modbus_new_tcp(connectionPath.c_str(), connectionPort);
      if (modbus==0) mberr = errno;
    }
    if (mberr!=0) {
      terminateAppWith(ModBusError::err<ModBusError>(mberr));
      return;
    }
    getIntOption("slave", slave);
    LOG(LOG_NOTICE, "successfully initialized modbus");
  }


  // MARK: ===== app logic

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
    mb_read_registers(slave, 104, 1, &reg104);
    mb_read_registers(slave, 202, 1, &reg202);
    // start in normal mode
    reg137 = 0;
    mb_write_registers(slave, 137, 1, &reg137);
    // repeatedly
    appTicket.executeOnce(boost::bind(&P44mbcd::appTask, this, _1, _2));
  }


  #define APP_TASK_PERIOD (2*Second)

  void appTask(MLTimer &aTimer, MLMicroSeconds aNow)
  {
    ErrorPtr err = mb_read_registers(slave, 104, 1, &reg104);
    if (Error::isOK(err)) {
      if (testmode) {
        if (reg104==1 || reg104==2) {
          // heating
          if (reg137!=1) {
            reg137 = 1; // heating test
            mb_write_registers(slave, 137, 1, &reg137);
          }
        }
        else if (reg104==3 || reg104==4) {
          // cooling
          if (reg137!=2) {
            reg137 = 2; // cooling test
            mb_write_registers(slave, 137, 1, &reg137);
          }
        }
        else if (reg104==0) {
          reg137 = 0; // off
          mb_write_registers(slave, 137, 1, &reg137);
        }
      }
      mb_read_registers(slave, 202, 1, &reg202);
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
      mb_write_registers(slave, 104, 1, &reg104);
      update_display();
    }
    if (aButtonId==2) {
      if (reg104>0) {
        reg104--;
      }
      mb_write_registers(slave, 104, 1, &reg104);
      update_display();
    }
  }



  // MARK: ===== littlevGL

  #define SHOW_MOUSE_CURSOR 1

  static void demoButtonPressed(int aButtonId)
  {
    P44mbcd *p44mbcd = dynamic_cast<P44mbcd *>(Application::sharedApplication());
    p44mbcd->buttonPressed(aButtonId);
  }


  void initLvgl()
  {
    LOG(LOG_NOTICE, "initializing littlevGL");
    showCursor = getOption("mousecursor");
    // - init library
    lv_init();
    // - init frame buffer driver
    fbdev_init();
    // - add display
    lv_disp_drv_t disp_drv;
    lv_disp_drv_init(&disp_drv);
    disp_drv.disp_flush = fbdev_flush;      /*It flushes the internal graphical buffer to the frame buffer*/
    dispdev = lv_disp_drv_register(&disp_drv);
    // - init evdev driver
    evdev_init();
    // - add pointer (touch, mouse)
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
  }



};


extern "C" {

  void setRts(modbus_t *ctx, int on) {
    P44mbcd* app = dynamic_cast<P44mbcd*>(Application::sharedApplication());
    if (app) {
      if (app->modbusRTSEnable) app->modbusRTSEnable->set(on);
    }
  }

}




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


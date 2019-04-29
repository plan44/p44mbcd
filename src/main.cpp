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

#if ENABLE_UBUS
  #include "ubus.hpp"
#endif

#include <stdio.h>

#include "lvgl/lvgl.h"

#include "lv_drivers/display/fbdev.h"
#include "lv_drivers/indev/evdev.h"

#include "lv_examples/lv_apps/demo/demo.h"


#define DEFAULT_LOGLEVEL LOG_NOTICE

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

  // littlevGL
  lv_disp_t *dispdev; ///< the display device
  lv_indev_t *pointer_indev; ///< the input device for pointer (touch, mouse)
  lv_indev_t *keyboard_indev; ///< the input device for keyboard
  MLTicket lvglTicket; ///< the display tasks timer
  MLMicroSeconds lastLvglTick; ///< last tick

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
      { 0  , "rs485connection", true,  "serial_if;RS485 serial interface where display is connected (/device or IP:port)" },
      { 0  , "rs485txenable",   true,  "pinspec;a digital output pin specification for TX driver enable or DTR or RTS" },
      { 0  , "rs485txoffdelay", true,  "delay;time to keep tx enabled after sending [ms], defaults to 0" },
      { 0  , "rs485rxenable",   true,  "pinspec;a digital output pin specification for RX driver enable" },
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

    // app now ready to run
    return run();
  }


  // MARK: ===== ubus API

  #if ENABLE_UBUS

  void initUbusApi()
  {
    ubusApiServer = UbusServerPtr(new UbusServer(MainLoop::currentMainLoop()));
    UbusObjectPtr u = new UbusObject("p44mbcd", boost::bind(&P44mbcd::ubusApiRequestHandler, this, _1, _2, _3));
    u->addMethod("log", logapi_policy);
    u->addMethod("api", p44mbcapi_policy);
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
    else if (aMethod=="api") {
      aUbusRequest->sendResponse(JsonObjectPtr(), UBUS_STATUS_INVALID_COMMAND);
    }
    else {
      // no other methods implemented yet
      aUbusRequest->sendResponse(JsonObjectPtr(), UBUS_STATUS_INVALID_COMMAND);
    }
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

    // start littlevGL
    initLvgl();
  }


  #define SHOW_MOUSE_CURSOR 1

  void initLvgl()
  {
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
    // - create demo
    demo_create();
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


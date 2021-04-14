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
#include "jsonobject.hpp"
#include "analogio.hpp"
#include "gpio.hpp"

#if ENABLE_UBUS
  #include "ubus.hpp"
#endif

#include <stdio.h>
#include <math.h>

#include "lvglui.hpp"

// FIXME: remove later
#include "lv_examples/lv_apps/demo/demo.h"


#define DEFAULT_MODBUS_RTU_PARAMS "115200,8,N,1" // [baud rate][,[bits][,[parity][,[stopbits][,[H]]]]]
#define DEFAULT_MODBUS_IP_PORT 1502

#define P44_EXIT_FIRMWAREUPGRADE 3 // request firmware upgrade, platform restart

#define MAINSCRIPT_FILE_NAME "mainscript.txt"
#define COMMCONFIG_FILE_NAME "commconfig"

#define FATAL_ERROR_IMG "errorscreen.png"

#define FILENO_FIRMWARE 1
#define FILENO_LOG 90
#define FILENO_MAINSCRIPT 100
#define FILENO_TEMPCOMMCONFIG 101
#define FILENO_COMMCONFIG 102
#define FILENO_IMAGES_BASE 200
#define MAX_IMAGES 100
#define FILENO_JSON_BASE 300
#define MAX_JSON 100


#define REGISTER_FIRST 101
#define REGISTER_LAST 299

using namespace p44;
using namespace P44Script;

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


// MARK: - BackLightController

class BackLightController : public P44Obj
{
  const MLMicroSeconds fadeStep = 20 * MilliSecond;

  AnalogIoPtr backlightOutput; ///< the backlight output
  bool active; ///< set if in active mode
  double activeBrightness; ///< brightness when active
  double standbyBrightness; ///< brightness when in standby
  double currentBrightness; ///< (possibly intermediate) current brightness state
  bool preventActivation; ///< prevent next activation

  MLMicroSeconds fadeTime; ///< fade time

  MLTicket standbyTicket; ///< timer for switch to standby brightness
  MLTicket fadeTicket; ///< timer for smooth transitions

public:

  BackLightController(AnalogIoPtr aBacklightOutput) :
    backlightOutput(aBacklightOutput)
  {
    activeBrightness = 90;
    standbyBrightness = 40;
    currentBrightness = 0;
    preventActivation = false;
    fadeTime = 1*Second;
    active = true;
    updateBacklight();
  }


  void setActiveBrightness(double aBrightness)
  {
    if (aBrightness!=activeBrightness) {
      LOG(LOG_INFO, "Backlight active brightness set to %.1f", aBrightness);
      activeBrightness = aBrightness;
      updateBacklight();
    }
  }

  void setFadeTime(MLMicroSeconds aFadeTime)
  {
    fadeTime = aFadeTime;
  }


  void setStandbyBrightness(double aBrightness)
  {
    LOG(LOG_INFO, "Backlight standby brightness set to %.1f", aBrightness);
    if (aBrightness!=standbyBrightness) {
      standbyBrightness = aBrightness;
      updateBacklight();
    }
  }


  void setActive(bool aActive)
  {
    if (aActive!=active) {
      if (aActive && preventActivation) {
        active = true; // consider activated, but not showing
        LOG(LOG_INFO, "Set active but actual backlight activation suppressed for now");
        return;
      }
      preventActivation = false;
      LOG(LOG_INFO, "Backlight switched to %s", aActive ? "ACTIVE" : "STANDBY");
      active = aActive;
      updateBacklight();
    }
  }


  void suppressNextActivation()
  {
    preventActivation = true;
  }


  void updateBacklight()
  {
    if (active) fadeTo(activeBrightness, 0);
    else fadeTo(standbyBrightness, fadeTime);
  }


  void fadeTo(double aTo, MLMicroSeconds aFadeTime)
  {
    if(fabs(currentBrightness - aTo) < 1e-4) return;
    if (aTo!=currentBrightness) {
      MLMicroSeconds numSteps = aFadeTime / fadeStep;
      fadeTicket.executeOnce(boost::bind(&BackLightController::updateFading, this, _1, (aTo-currentBrightness) / (numSteps>0 ? numSteps : 1), aTo));
    }
  }

  void updateFading(MLTimer &aTimer, double aDv, double aTo)
  {
    double newValue = currentBrightness + aDv;
    bool done = false;
    if((aDv > 0 && newValue >= aTo) || (aDv < 0 && newValue <= aTo)) {
      newValue = aTo;
      done = true;
    }
    currentBrightness = newValue;
    LOG(LOG_DEBUG, "New brightness value = %.1f", currentBrightness);
    setBackLight(currentBrightness);
    if(!done) MainLoop::currentMainLoop().retriggerTimer(aTimer, fadeStep);
  }

  #define DIM_CURVE_EXPONENT 4

  void setBackLight(double aBrightness)
  {
    backlightOutput->setValue(100*((exp(aBrightness*DIM_CURVE_EXPONENT/100)-1)/(exp(DIM_CURVE_EXPONENT)-1)));
  }


};
typedef boost::intrusive_ptr<BackLightController> BackLightControllerPtr;


// MARK: - P44mbcd

class P44mbcd;

/// global script function lookup for this app
class P44mbcdLookup : public BuiltInMemberLookup
{
  typedef BuiltInMemberLookup inherited;
public:
  P44mbcd &mP44mbcd;
  P44mbcdLookup(P44mbcd &aP44mbcd);
};


class P44mbcd : public CmdLineApp
{
  typedef CmdLineApp inherited;

  #if ENABLE_UBUS
  // ubus API
  UbusServerPtr ubusApiServer; ///< ubus API for openwrt web interface
  #endif

  // modbus
  ModbusSlavePtr modBusSlave; ///< modbus slave
  ModbusMasterPtr modBusMaster; ///< modbus master
  DigitalIoPtr modbusRxEnable; ///< if set, modbus receive is enabled

  // app
  LvGLUi ui;
  bool active;

  // scripting
  ScriptSource mainScript;

  MLTicket exitTicket; ///< terminate delay

  // temperature sensor
  AnalogIoPtr tempSens; ///< the temperature sensor input

public:

  // LCD backlight control
  MLMicroSeconds backlightTimeout; ///< inactivity time that triggers backlight standby. 0 = never, -1 = always inactive
  BackLightControllerPtr backlight;
  // activity
  MLMicroSeconds activityTimeout; ///< inactivity time that triggers activityTimeoutScript

  P44mbcd() :
    mainScript(sourcecode+regular, "main") // only init script may have declarations
  {
    ui.isMemberVariable();
    active = true;
    activityTimeout = Never;
    backlightTimeout = Never;
    // let all scripts run in the same (ui) context
    mainScript.setSharedMainContext(ui.getScriptMainContext());
  }

  virtual int main(int argc, char **argv)
  {
    const char *usageText =
    "Usage: %1$s [options]\n";
    const CmdLineOptionDescriptor options[] = {
      { 0  , "connection",      true,  "connspec;serial interface for RTU or IP address for TCP (/device or IP[:port])" },
      { 0  , "rs485txenable",   true,  "pinspec;a digital output pin specification for TX driver enable, 'RTS' or 'RS232'" },
      { 0  , "rs485txdelay",    true,  "delay;delay of tx enable signal in uS" },
      { 0  , "rs485rxenable",   true,  "pinspec;a digital output pin specification for RX input enable" },
      { 0  , "bytetime",        true,  "time;custom time per byte in nS" },
      { 0  , "slave",           true,  "slave;use this slave by default (0: act as master)" },
      { 0  , "slaveswitch",     true,  "gpiono:numgpios;use GPIOs for slave address DIP switch, first GPIO=A0" },
      { 0  , "debugmodbus",     false, "enable libmodbus debug messages to stderr" },
      { 0  , "backlight",       true,  "pinspec;analog output for LCD backlight control" },
      { 0  , "tempsensor",      true,  "pinspec;analog input for temperature measurement" },
      #if MOUSE_CURSOR_SUPPORT
      { 0  , "mousecursor",     false, "show mouse cursor" },
      #endif
      #if ENABLE_UBUS
      { 0  , "ubusapi",         false, "enable ubus API" },
      #endif
      CMDLINE_APPLICATION_PATHOPTIONS,
      DAEMON_APPLICATION_LOGOPTIONS,
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

    // app now ready to run
    return run();
  }



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
            if (modBusSlave) modBusSlave->setDebug(true);
            if (modBusMaster) modBusMaster->setDebug(true);
          }
          else if (cmd=="debug_off") {
            if (modBusSlave) modBusSlave->setDebug(false);
            if (modBusMaster) modBusMaster->setDebug(false);
          }
          else if (modBusSlave && cmd=="read_registers") {
            int reg = -1;
            int numReg = 1;
            if (aJsonRequest->get("reg", o)) reg = o->int32Value();
            if (aJsonRequest->get("count", o)) numReg = o->int32Value();
            if (reg<0 || numReg<1 || numReg>=MAX_REG) {
              err = TextError::err("invalid reg=%d, count=%d combination", reg, numReg);
            }
            else {
              uint16_t tab_reg[MAX_REG];
              for (int i=0; i<numReg; i++) {
                result->arrayAppend(JsonObject::newInt32(modBusSlave->getReg(reg+i, false)));
              }
            }
          }
          else if (modBusSlave && cmd=="write_registers") {
            int reg = -1;
            if (aJsonRequest->get("reg", o)) reg = o->int32Value();
            int numReg = 0;
            uint16_t tab_reg[MAX_REG];
            if (reg<0) {
              err = TextError::err("invalid reg=%d");
            }
            else {
              if (aJsonRequest->get("values", o)) {
                if (o->isType(json_type_array)) {
                  // multiple
                  for(int i=0; i<o->arrayLength(); i++) {
                    modBusSlave->setReg(reg+i, false, o->arrayGet(i)->int32Value());
                  }
                }
                else {
                  // single
                  modBusSlave->setReg(reg, false, o->int32Value());
                }
              }
              else {
                err = TextError::err("missing 'values'");
              }
            }
            if (Error::isOK(err)) {
              result = JsonObject::newBool(true);
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
    ErrorPtr err;
    LOG(LOG_NOTICE, "p44mbcd: initialize");
    #if ENABLE_UBUS
    // start ubus API, if we have it
    if (ubusApiServer) {
      ubusApiServer->startServer();
    }
    #endif
    // slave address (or master when address==255)
    int slave = 1;
    // - can be sampled from GPIOs
    string dipcfg;
    if (getStringOption("slaveswitch", dipcfg)) {
      int firstGpio, numGpios;
      slave = 0;
      if (sscanf(dipcfg.c_str(), "%d:%d", &firstGpio, &numGpios)==2) {
        for (int bitpos=0; bitpos<numGpios; bitpos++) {
          IOPinPtr switchBit = IOPinPtr(new GpioPin(firstGpio+bitpos, false, false));
          if (switchBit->getState()==0) slave |= 1<<bitpos; // inverted
        }
        LOG(LOG_NOTICE, "Modbus slave address %d read from DIP-Switch (GPIO%d..%d)", slave, firstGpio, firstGpio+numGpios-1);
      }
    }
    // - can be overridden
    getIntOption("slave", slave);
    // General modbus connection params
    string mbconn;
    if (!getStringOption("connection", mbconn)) {
      terminateAppWith(TextError::err("must specify --connection"));
      return;
    }
    string txen;
    getStringOption("rs485txenable", txen);
    int txDelayUs = Never;
    getIntOption("rs485txdelay", txDelayUs);
    int byteTimeNs = 0;
    getIntOption("bytetime", byteTimeNs);
    string rxen;
    getStringOption("rs485rxenable", rxen);
    bool modbusDebug = getOption("debugmodbus");
    // Master or slave
    if (slave!=0) {
      // we are a modbus slave
      modBusSlave = ModbusSlavePtr(new ModbusSlave);
      err = modBusSlave->setConnectionSpecification(
        mbconn.c_str(),
        DEFAULT_MODBUS_IP_PORT, DEFAULT_MODBUS_RTU_PARAMS,
        txen.c_str(), txDelayUs,
        rxen.empty() ? NULL : rxen.c_str(), // NULL if there is no separate rx enable
        byteTimeNs
      );
      if (Error::notOK(err)) {
        terminateAppWith(err->withPrefix("Invalid modbus connection: "));
        return;
      }
      modBusSlave->setSlaveAddress(slave);
      modBusSlave->setSlaveId(string_format("p44mbc %s %06llX", version().c_str(), macAddress()));
      modBusSlave->setDebug(modbusDebug);
      // registers
      modBusSlave->setRegisterModel(
        0, 0, // coils
        0, 0, // input bits
        REGISTER_FIRST, REGISTER_LAST-REGISTER_FIRST+1, // registers
        0, 0 // input registers
      );
      // Files
      // - firmware
      modBusSlave->addFileHandler(ModbusFileHandlerPtr(new ModbusFileHandler(
        FILENO_FIRMWARE,
        9, // max segs
        1, // single file
        true, // p44 header
        "fwimg",
        false, // R/W
        tempPath("final_") // write to temp, then copy to data path
      )))->setFileWriteCompleteCB(boost::bind(&P44mbcd::modbusFWReceivedHandler, this, _1, _2, _3));
      // - log
      modBusSlave->addFileHandler(ModbusFileHandlerPtr(new ModbusFileHandler(
        FILENO_LOG,
        9, // max segs
        1, // single file
        true, // p44 header
        "/var/log/p44mbcd/current",
        true // read only
      )));
      // - json config
      modBusSlave->addFileHandler(ModbusFileHandlerPtr(new ModbusFileHandler(
        FILENO_MAINSCRIPT,
        1, // max segs
        1, // single file
        true, // p44 header
        MAINSCRIPT_FILE_NAME,
        false, // R/W
        dataPath()+"/" // write to temp, then copy to data path
      )))->setFileWriteCompleteCB(boost::bind(&P44mbcd::modbusFileReceivedHandler, this, _1, _2, _3));
      // - communication (daemon startup) config
      modBusSlave->addFileHandler(ModbusFileHandlerPtr(new ModbusFileHandler(
        FILENO_TEMPCOMMCONFIG,
        1, // max segs
        1, // single file
        true, // p44 header
        COMMCONFIG_FILE_NAME,
        false, // R/W
        tempPath()+"/" // keep in temp
      )))->setFileWriteCompleteCB(boost::bind(&P44mbcd::modbusFileReceivedHandler, this, _1, _2, _3));
      modBusSlave->addFileHandler(ModbusFileHandlerPtr(new ModbusFileHandler(
        FILENO_COMMCONFIG,
        1, // max segs
        1, // single file
        true, // p44 header
        COMMCONFIG_FILE_NAME,
        false, // R/W
        dataPath()+"/" // write to temp, then copy to data path
      )))->setFileWriteCompleteCB(boost::bind(&P44mbcd::modbusFileReceivedHandler, this, _1, _2, _3));
      // - UI images
      modBusSlave->addFileHandler(ModbusFileHandlerPtr(new ModbusFileHandler(
        FILENO_IMAGES_BASE,
        1, // max segs
        MAX_IMAGES, // number of files allowed
        true, // p44 header
        "image%03d.png",
        false, // R/W
        dataPath()+"/" // write to temp, then copy to data path
      )))->setFileWriteCompleteCB(boost::bind(&P44mbcd::modbusFileReceivedHandler, this, _1, _2, _3));
      // - JSON files
      modBusSlave->addFileHandler(ModbusFileHandlerPtr(new ModbusFileHandler(
        FILENO_JSON_BASE,
        1, // max segs
        MAX_JSON, // number of files allowed
        true, // p44 header
        "data%03d.json",
        false, // R/W
        dataPath()+"/" // write to temp, then copy to data path
      )))->setFileWriteCompleteCB(boost::bind(&P44mbcd::modbusFileReceivedHandler, this, _1, _2, _3));
      // connect
      err = modBusSlave->connect();
      if (Error::notOK(err)) {
        terminateAppWith(err->withPrefix("Failed to start modbus slave server: "));
        return;
      }
      // - modbus slave scripting functions
      StandardScriptingDomain::sharedDomain().registerMember("modbus", modBusSlave->representingScriptObj());
    }
    else {
      // Modbus master
      modBusMaster = ModbusMasterPtr(new ModbusMaster);
      err = modBusMaster->setConnectionSpecification(
        mbconn.c_str(),
        DEFAULT_MODBUS_IP_PORT, DEFAULT_MODBUS_RTU_PARAMS,
        txen.c_str(), txDelayUs,
        rxen.empty() ? NULL : rxen.c_str(), // NULL if there is no separate rx enable
        byteTimeNs
      );
      if (Error::notOK(err)) {
        terminateAppWith(err->withPrefix("Invalid modbus connection: "));
        return;
      }
      modBusMaster->setDebug(modbusDebug);
      // - modbus master scripting functions
      StandardScriptingDomain::sharedDomain().registerMember("modbus", modBusMaster->representingScriptObj());
    }
    // LCD backlight
    string blspec = "missing";
    getStringOption("backlight", blspec);
    AnalogIoPtr backlightOutput = AnalogIoPtr(new AnalogIo(blspec.c_str(), true, 100));
    backlight = BackLightControllerPtr(new BackLightController(backlightOutput));
    // optional temperature sensor
    string tsspec = "missing";
    if (getStringOption("tempsensor", tsspec)) {
      tempSens = AnalogIoPtr(new AnalogIo(tsspec.c_str(), false, 0));
    }
    // install app specific global predefined objects
    // - app functions
    StandardScriptingDomain::sharedDomain().registerMemberLookup(new P44mbcdLookup(*this));
    // - LVGL
    StandardScriptingDomain::sharedDomain().registerMember("ui", ui.representingScriptObj());
    // start littlevGL
    ui.setResourceLoadOptions(true, "");
    initLvgl();
    LvGL::lvgl().setTaskCallback(boost::bind(&P44mbcd::taskCallBack, this));
    // load and start main script
    string code;
    err = string_fromfile(dataPath(MAINSCRIPT_FILE_NAME), code);
    if (Error::notOK(err)) {
      err = string_fromfile(resourcePath(MAINSCRIPT_FILE_NAME), code);
      if (Error::notOK(err)) {
        err->prefixMessage("Cannot open '" MAINSCRIPT_FILE_NAME "': ");
      }
    }
    if (Error::isOK(err)) {
      mainScript.setSource(code);
      ScriptObjPtr res = mainScript.syntaxcheck();
      if (res && res->isErr()) {
        err = res->errorValue();
        err->prefixMessage("Syntax Error in mainscript: ");
      }
      else {
        LOG(LOG_NOTICE, "Starting mainscript");
        mainScript.run(inherit, boost::bind(&P44mbcd::mainScriptDone, this, _1));
      }
    }
    // display error
    if (Error::notOK(err)) {
      LOG(LOG_ERR, "Startup error: %s", Error::text(err));
      fatalErrorScreen(string_format("Startup error: %s", Error::text(err)));
    }
  }


  void mainScriptDone(ScriptObjPtr aResult)
  {
    if (aResult && aResult->isErr()) {
      LOG(LOG_ERR, "mainscript failed: %s", aResult->errorValue()->text());
      fatalErrorScreen(string_format("Mainscript error: %s", aResult->errorValue()->text()));
    }
    else {
      LOG(LOG_NOTICE, "mainscript terminated with result: %s", ScriptObj::describe(aResult).c_str());
    }
  }


  // from: https://www.openhacks.com/uploadsproductos/pt1000-temp-probe.pdf
  static double pt1000_Ohms_to_degreeC(double aResistance)
  {
    // T = -(SQRT(-0.00232*R + 17.59246) - 3.908)/0.00116
    double x = -0.00232*aResistance + 17.59246;
    if (x>=0) return -(sqrt(x) - 3.908)/0.00116;
    return -9999; // error
  }


  double getTemp()
  {
    double temp = -999;
    if (tempSens) {
      double adc = tempSens->value();
      // Luz:
      //  ADC 0 = 1201.5 Ohm
      //  ADC 1203 = 1000.0 Ohm
      // R = ADC/1023*(1000-1201.5)+1201.5
      //double res = adc/1023*(1000-1201.5)+1201.5;
      // Astrol:
      //  nbits = 10
      //  Rpu = 30e3
      //  Rref = 1e3
      //  Vu = 160
      //  Rpt = 1201.51 % PT1000 value
      // ADCVal = 2^nbits * (1 - Vu * (Rpt/(Rpt+Rpu) - Rref/(Rref+Rpu)) )
      // MuSimp/MuMath solves this for Rpt:
      //  RPT = (ADC*RPU*RREF+A*RPU^2-1024*VU*RPU*RREF-1024*RPU*RREF-1024*RPU^2)/(1024*RPU+1024*RREF-A*RPU-A*RREF-1024*VU*RPU)
      //  RPT = (-5867520000+930000*ADC)/(-4883456-31*ADC)
      double res = (-5867520000.0+930000.0*adc)/(-4883456.0-31.0*adc);
      // convert to temperature
      temp = pt1000_Ohms_to_degreeC(res);
      LOG(LOG_INFO, "tempsens raw value = %.2f -> resistance = %.2f -> temperature = %.2f", adc, res, temp);
    }
    return temp;
  }


  void taskCallBack()
  {
    MLMicroSeconds inactivetime = (MLMicroSeconds)lv_disp_get_inactive_time(NULL)*MilliSecond;
    // backlight standby
    if (backlight) {
      backlight->setActive(
        backlightTimeout>=0 && // not forced into standby...
        (backlightTimeout==Never || inactivetime<backlightTimeout) // ...and no timeout at all or timeout not yet reached
      );
    }
    // inactivity script
    if (activityTimeout && inactivetime>activityTimeout) {
      if (active) {
        active = false;
        ui.uiActivation(false);
      }
    }
    else {
      if (!active) {
        active = true;
        ui.uiActivation(true);
      }
    }
  }


  void modbusFileReceivedHandler(uint16_t aFileNo, const string aFinalPath, const string aTempPath)
  {
    // config or image file received, copy it to datadir
    if (!aTempPath.empty()) {
      if (aTempPath==aFinalPath) {
        // already in place (i.e. is a temp file anyway) -> nothing to copy
        modbusFileStored(aFileNo, aFinalPath, ErrorPtr());
        return;
      }
      MainLoop::currentMainLoop().fork_and_system(
        boost::bind(&P44mbcd::modbusFileStored, this, aFileNo, aFinalPath, _1),
        string_format("cp %s %s", aTempPath.c_str(), aFinalPath.c_str()).c_str()
      );
    }
  }


  void delayedTerminate(int aExitCode)
  {
    terminateApp(aExitCode);
  }


  void modbusFileStored(uint16_t aFileNo, const string aFinalPath, ErrorPtr aError)
  {
    if (Error::notOK(aError)) {
      LOG(LOG_ERR, "Error copying fileNo %d to %s: %s", aFileNo, aFinalPath.c_str(), aError->text());
      return;
    }
    LOG(LOG_NOTICE, "received file No %d, now stored in %s", aFileNo, aFinalPath.c_str());
    if (aFileNo==FILENO_MAINSCRIPT) {
      LOG(LOG_NOTICE, "new mainscript received -> restart daemon");
      exitTicket.executeOnce(boost::bind(&P44mbcd::delayedTerminate, this, EXIT_SUCCESS), 2*Second);
      return;
    }
    else if (aFileNo==FILENO_COMMCONFIG || aFileNo==FILENO_TEMPCOMMCONFIG) {
      LOG(LOG_NOTICE, "new communication config received -> restart daemon");
      exitTicket.executeOnce(boost::bind(&P44mbcd::delayedTerminate, this, EXIT_SUCCESS), 2*Second);
      return;
    }
  }


  void modbusFWReceivedHandler(uint16_t aFileNo, const string aFinalPath, const string aTempPath)
  {
    // firmware received, just move/rename it
    if (!aTempPath.empty()) {
      MainLoop::currentMainLoop().fork_and_system(
        boost::bind(&P44mbcd::modbusFWStored, this, aFileNo, aFinalPath, _1),
        string_format("mv %s %s", aTempPath.c_str(), aFinalPath.c_str()).c_str()
      );
    }
  }


  void modbusFWStored(uint16_t aFileNo, const string aFinalPath, ErrorPtr aError)
  {
    if (Error::notOK(aError)) {
      LOG(LOG_ERR, "Error moving fileNo %d to %s: %s", aFileNo, aFinalPath.c_str(), aError->text());
      return;
    }
    LOG(LOG_NOTICE, "triggering firmware update");
    terminateApp(P44_EXIT_FIRMWAREUPGRADE);
  }


  /*
  ErrorPtr modbusValueAccessHandler(int aAddress, bool aBit, bool aInput, bool aWrite)
  {
    uint16_t val = modBusSlave->getValue(aAddress, aBit, aInput);
    LOG(LOG_NOTICE,
      "%s%s %d accessed for %s, value = %d (0x%04X)",
      aInput ? "Readonly " : "",
      aBit ? "Bit" : "Register",
      aAddress,
      aWrite ? "write" : "read",
      val, val
    );
    return ErrorPtr();
  }
  */


  // MARK: - littlevGL


  void initLvgl()
  {
    LOG(LOG_NOTICE, "initializing littlevGL");
    LvGL::lvgl().init(getOption("mousecursor"));
    // create app UI
    // - init display
    ui.initForDisplay(lv_disp_get_default());
  }


  void fatalErrorScreen(const string aMessage)
  {
    lv_obj_t* errorScreen = lv_img_create(NULL, NULL);
    lv_img_set_src(errorScreen, resourcePath(FATAL_ERROR_IMG).c_str());
    // error label
    lv_obj_t* errLabel = lv_label_create(errorScreen, NULL);
    static lv_style_t errLabelStyle;
    lv_style_copy(&errLabelStyle, &lv_style_plain);
    errLabelStyle.text.font = &lv_font_roboto_16;
    lv_obj_set_style(errLabel, &errLabelStyle);
    lv_label_set_long_mode(errLabel, LV_LABEL_LONG_SROLL_CIRC);
    lv_label_set_align(errLabel, LV_LABEL_ALIGN_CENTER);
    lv_label_set_text(errLabel, aMessage.c_str());
    lv_obj_set_width(errLabel, lv_obj_get_width(errorScreen)-10); // expand to full width
    lv_obj_set_height(errLabel, 42);
    lv_obj_align(errLabel, NULL, LV_ALIGN_IN_BOTTOM_MID, 0, 0);
    // activate the error screen
    lv_scr_load(errorScreen);
  }


};


// MARK: - script functions

// activitytimeout(seconds)
static const BuiltInArgDesc activitytimeout_args[] = { { numeric } };
static const size_t activitytimeout_numargs = sizeof(activitytimeout_args)/sizeof(BuiltInArgDesc);
static void activitytimeout_func(BuiltinFunctionContextPtr f)
{
  P44mbcd& p44mbcd = static_cast<P44mbcdLookup*>(f->funcObj()->getMemberLookup())->mP44mbcd;
  p44mbcd.activityTimeout = f->arg(0)->doubleValue()*Second;
  f->finish();
}


// backlight(active, [standby, timeout [, fadetime]])
static const BuiltInArgDesc backlight_args[] = { { numeric }, { numeric|optionalarg }, { numeric|optionalarg }, { numeric|optionalarg } };
static const size_t backlight_numargs = sizeof(backlight_args)/sizeof(BuiltInArgDesc);
static void backlight_func(BuiltinFunctionContextPtr f)
{
  P44mbcd& p44mbcd = static_cast<P44mbcdLookup*>(f->funcObj()->getMemberLookup())->mP44mbcd;
  if (p44mbcd.backlight) {
    p44mbcd.backlight->setActiveBrightness(f->arg(0)->doubleValue());
    if (f->numArgs()>=3) {
      p44mbcd.backlight->setStandbyBrightness(f->arg(1)->doubleValue());
      MLMicroSeconds bt = f->arg(2)->doubleValue()*Second;
      if (bt!=p44mbcd.backlightTimeout) {
        p44mbcd.backlightTimeout = bt;
        if (p44mbcd.backlightTimeout<0) {
          p44mbcd.backlight->setActive(false); // force standby brightness
          p44mbcd.backlight->suppressNextActivation(); // avoid next activation
        }
      }
      if (f->numArgs()>=4) {
        p44mbcd.backlight->setFadeTime(f->arg(3)->doubleValue()*Second);
      }
    }
  }
  f->finish();
}


// temperature()
static void temperature_func(BuiltinFunctionContextPtr f)
{
  P44mbcd& p44mbcd = static_cast<P44mbcdLookup*>(f->funcObj()->getMemberLookup())->mP44mbcd;
  double temp = p44mbcd.getTemp();
  if (temp>-999) {
    f->finish(new NumericValue(temp));
    return;
  }
  f->finish(new AnnotatedNullValue("no temperature sensor"));
}


// exit(exitcode)
static const BuiltInArgDesc exit_args[] = { { numeric } };
static const size_t exit_numargs = sizeof(exit_args)/sizeof(BuiltInArgDesc);
static void exit_func(BuiltinFunctionContextPtr f)
{
  P44mbcd& p44mbcd = static_cast<P44mbcdLookup*>(f->funcObj()->getMemberLookup())->mP44mbcd;
  p44mbcd.terminateApp(f->arg(0)->intValue());
  f->finish();
}



static const BuiltinMemberDescriptor p44mbcdGlobals[] = {
  { "activitytimeout", executable|null, activitytimeout_numargs, activitytimeout_args, &activitytimeout_func },
  { "backlight", executable|null, backlight_numargs, backlight_args, &backlight_func },
  { "temperature", executable|numeric, 0, NULL, &temperature_func },
  { "exit", executable|null, exit_numargs, exit_args, &exit_func },
  { NULL } // terminator
};


P44mbcdLookup::P44mbcdLookup(P44mbcd &aP44mbcd) :
  inherited(p44mbcdGlobals),
  mP44mbcd(aP44mbcd)
{
}


// MARK: - main

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

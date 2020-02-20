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

#include "lvglui.hpp"

// FIXME: remove later
#include "lv_examples/lv_apps/demo/demo.h"


#define DEFAULT_MODBUS_RTU_PARAMS "115200,8,N,1" // [baud rate][,[bits][,[parity][,[stopbits][,[H]]]]]
#define DEFAULT_MODBUS_IP_PORT 1502

#define P44_EXIT_FIRMWAREUPGRADE 3 // request firmware upgrade, platform restart

#define UICONFIG_FILE_NAME "uiconfig.json"
#define COMMCONFIG_FILE_NAME "commconfig"

#define FATAL_ERROR_IMG "errorscreen.png"

#define FILENO_FIRMWARE 1
#define FILENO_LOG 90
#define FILENO_UICONFIG 100
#define FILENO_TEMPCOMMCONFIG 101
#define FILENO_COMMCONFIG 102
#define FILENO_IMAGES_BASE 200
#define MAX_IMAGES 100

#define TEMPSENS_REGISTER 124
#define TEMPSENS_POLLINTERVAL (15*Second)

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


class GlobScriptContext : public P44Obj
{
public:
  GlobScriptContext(int aAddr, bool aBit, bool aInput) : accessAddress(aAddr), accessBit(aBit), accessInput(aInput) {};
  int accessAddress;
  bool accessBit;
  bool accessInput;
};


class P44mbcd : public CmdLineApp
{
  typedef CmdLineApp inherited;

  #if ENABLE_UBUS
  // ubus API
  UbusServerPtr ubusApiServer; ///< ubus API for openwrt web interface
  #endif

  // modbus
  ModbusSlave modBus;
  DigitalIoPtr modbusRxEnable; ///< if set, modbus receive is enabled

  // app
  LvGLUi ui;
  bool active;
  MLMicroSeconds activityTimeout; ///< inactivity time that triggers activityTimeoutScript
  MLMicroSeconds backlightTimeout; ///< inactivity time that triggers backlight standby. 0 = never, -1 = always inactive

  // scripting
  string initScript;
  string modbusWriteScript;
  string modbusReadScript;
  string activityTimeoutScript;
  string activationScript;

  // LCD backlight control
  BackLightControllerPtr backlight;

  // temperature sensor
  AnalogIoPtr tempSens; ///< the temperature sensor input
  MLTicket tempSensTicket; ///< temperature sensor polling
  MLTicket exitTicket; ///< terminate delay

public:

  P44mbcd()
  {
    ui.isMemberVariable();
    modBus.isMemberVariable();
    active = true;
    activityTimeout = Never;
    backlightTimeout = Never;
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
      { 0  , "slave",           true,  "slave;use this slave by default" },
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
            modBus.setDebug(true);
          }
          else if (cmd=="debug_off") {
            modBus.setDebug(false);
          }
          else if (cmd=="read_registers") {
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
                result->arrayAppend(JsonObject::newInt32(modBus.getReg(reg+i, false)));
              }
            }
          }
          else if (cmd=="write_registers") {
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
                    modBus.setReg(reg+i, false, o->arrayGet(i)->int32Value());
                  }
                }
                else {
                  // single
                  modBus.setReg(reg, false, o->int32Value());
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
    // - rx enable, static for now
    string txen;
    getStringOption("rs485txenable", txen);
    int txDelayUs = Never;
    getIntOption("rs485txdelay", txDelayUs);
    int byteTimeNs = 0;
    getIntOption("bytetime", byteTimeNs);
    ErrorPtr err = modBus.setConnectionSpecification(
      mbconn.c_str(),
      DEFAULT_MODBUS_IP_PORT, DEFAULT_MODBUS_RTU_PARAMS,
      txen.c_str(), txDelayUs,
      getOption("rs485rxenable"), // can be NULL if there is no separate rx enable
      byteTimeNs
    );
    if (Error::notOK(err)) {
      terminateAppWith(err->withPrefix("Invalid modbus connection: "));
      return;
    }
    // slave address
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
    modBus.setSlaveAddress(slave);
    modBus.setSlaveId(string_format("p44mbc %s %06llX", version().c_str(), macAddress()));
    modBus.setDebug(getOption("debugmodbus"));
    // registers
    modBus.setRegisterModel(
      0, 0, // coils
      0, 0, // input bits
      101, 129-101+1, // registers
      0, 0 // input registers
    );
    modBus.setValueAccessHandler(boost::bind(&P44mbcd::modbusValueAccessHandler, this, _1, _2, _3, _4));
    // Files
    // - firmware
    modBus.addFileHandler(ModbusFileHandlerPtr(new ModbusFileHandler(
      FILENO_FIRMWARE,
      9, // max segs
      1, // single file
      true, // p44 header
      "fwimg",
      false, // R/W
      tempPath("final_") // write to temp, then copy to data path
    )))->setFileWriteCompleteCB(boost::bind(&P44mbcd::modbusFWReceivedHandler, this, _1, _2, _3));
    // - log
    modBus.addFileHandler(ModbusFileHandlerPtr(new ModbusFileHandler(
      FILENO_LOG,
      9, // max segs
      1, // single file
      true, // p44 header
      "/var/log/p44mbcd/current",
      true // read only
    )));
    // - json config
    modBus.addFileHandler(ModbusFileHandlerPtr(new ModbusFileHandler(
      FILENO_UICONFIG,
      1, // max segs
      1, // single file
      true, // p44 header
      UICONFIG_FILE_NAME,
      false, // R/W
      dataPath()+"/" // write to temp, then copy to data path
    )))->setFileWriteCompleteCB(boost::bind(&P44mbcd::modbusFileReceivedHandler, this, _1, _2, _3));
    // - communication (daemon startup) config
    modBus.addFileHandler(ModbusFileHandlerPtr(new ModbusFileHandler(
      FILENO_TEMPCOMMCONFIG,
      1, // max segs
      1, // single file
      true, // p44 header
      COMMCONFIG_FILE_NAME,
      false, // R/W
      tempPath()+"/" // keep in temp
    )))->setFileWriteCompleteCB(boost::bind(&P44mbcd::modbusFileReceivedHandler, this, _1, _2, _3));
    modBus.addFileHandler(ModbusFileHandlerPtr(new ModbusFileHandler(
      FILENO_COMMCONFIG,
      1, // max segs
      1, // single file
      true, // p44 header
      COMMCONFIG_FILE_NAME,
      false, // R/W
      dataPath()+"/" // write to temp, then copy to data path
    )))->setFileWriteCompleteCB(boost::bind(&P44mbcd::modbusFileReceivedHandler, this, _1, _2, _3));
    // - UI images
    modBus.addFileHandler(ModbusFileHandlerPtr(new ModbusFileHandler(
      FILENO_IMAGES_BASE,
      1, // max segs
      MAX_IMAGES, // number of files allowed
      true, // p44 header
      "image%03d.png",
      false, // R/W
      dataPath()+"/" // write to temp, then copy to data path
    )))->setFileWriteCompleteCB(boost::bind(&P44mbcd::modbusFileReceivedHandler, this, _1, _2, _3));
    // connect
    err = modBus.connect();
    if (Error::notOK(err)) {
      terminateAppWith(err->withPrefix("Failed to start modbus slave server: "));
      return;
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
      if (tempSens) {
        tempSensTicket.executeOnce(boost::bind(&P44mbcd::tempSensPoll, this, _1), 1*Second);
      }
    }
    // start littlevGL
    initLvgl();
    LvGL::lvgl().setTaskCallback(boost::bind(&P44mbcd::taskCallBack, this));
    // call init script
    ui.queueEventScript(LV_EVENT_REFRESH, LVGLUiElementPtr(), initScript);
  }


  // from: https://www.openhacks.com/uploadsproductos/pt1000-temp-probe.pdf
  static double pt1000_Ohms_to_degreeC(double aResistance)
  {
    // T = -(SQRT(-0.00232*R + 17.59246) - 3.908)/0.00116
    double x = -0.00232*aResistance + 17.59246;
    if (x>=0) return -(sqrt(x) - 3.908)/0.00116;
    return -9999; // error
  }


  void tempSensPoll(MLTimer &aTimer)
  {
    double val = tempSens->value();
    // ADC 0 = 1201.5 Ohm
    // ADC 1203 = 1000.0 Ohm
    // R = ADC/1023*(1000-1201.5)+1201.5
    double res = val/1023*(1000-1201.5)+1201.5;
    double temp = pt1000_Ohms_to_degreeC(res);
    LOG(LOG_INFO, "tempsens raw value = %.2f -> resistance = %.2f -> temperature = %.2f", val, res, temp);
    modBus.setReg(TEMPSENS_REGISTER, false, temp);
    MainLoop::currentMainLoop().retriggerTimer(aTimer, TEMPSENS_POLLINTERVAL);
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
        ui.queueEventScript(LV_EVENT_REFRESH, LVGLUiElementPtr(), activityTimeoutScript);
      }
    }
    else {
      if (!active) {
        active = true;
        ui.queueEventScript(LV_EVENT_REFRESH, LVGLUiElementPtr(), activationScript);
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
    if (aFileNo==FILENO_UICONFIG) {
      LOG(LOG_NOTICE, "new uiconfig received -> restart daemon");
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
    // report write access
    P44ObjPtr ctx = P44ObjPtr(new GlobScriptContext(aAddress, aBit, aInput));
    if (aWrite) {
      ui.queueGlobalScript(modbusWriteScript, ctx);
    }
    else {
      ui.queueGlobalScript(modbusReadScript, ctx);
    }
    return ErrorPtr();
  }


  // MARK: - script and config interface


  /// callback function for function evaluation
  /// @param aFunc the name of the function to execute, always passed in in all lowercase
  /// @param aArgs vector of function arguments, tuple contains expression starting position and value
  /// @param aResult set to function's result
  /// @return true if function executed, false if function signature (name, number of args) is unknown
  bool uiFunctionHandler(EvaluationContext* aEvalContext, const string& aFunc, const FunctionArguments& aArgs, ExpressionValue& aResult)
  {
    GlobScriptContext* ctx = dynamic_cast<GlobScriptContext *>(aEvalContext->getCallerContext().get());

    // function for modbus read and write handlers
    if (aFunc=="reg" && aArgs.size()<=1) {
      // reg([input]) returns accessed register number or null
      bool inp = aArgs[2].boolValue(); // null or false means non-input
      if (!ctx->accessBit && inp==ctx->accessInput) aResult.setNumber(ctx->accessAddress);
    }
    else if (aFunc=="bit" && aArgs.size()<=1) {
      // bit([input]) returns accessed bit number or null
      bool inp = aArgs[2].boolValue(); // null or false means non-input
      if (ctx->accessBit && inp==ctx->accessInput) aResult.setNumber(ctx->accessAddress);
    }
    // functions for UI scripts
    else if (aFunc=="modbus_setreg" && aArgs.size()>=2 && aArgs.size()<=3) {
      // modbus_setreg(regaddr, value [,input])
      int addr = aArgs[0].intValue();
      uint16_t val = aArgs[1].intValue();
      bool inp = aArgs[2].boolValue(); // null or false means non-input
      modBus.setReg(addr, inp, val);
    }
    else if (aFunc=="modbus_setbit" && aArgs.size()>=2 && aArgs.size()<=3) {
      // modbus_setreg(regaddr, value [,input])
      int addr = aArgs[0].intValue();
      bool bit = aArgs[1].boolValue();
      bool inp = aArgs[2].boolValue(); // null or false means non-input
      modBus.setBit(addr, inp, bit);
    }
    else if (aFunc=="modbus_getreg" && aArgs.size()>=1 && aArgs.size()<=2) {
      // modbus_getreg(regaddr [,input])
      int addr = aArgs[0].intValue();
      bool inp = aArgs[1].boolValue(); // null or false means non-input
      aResult.setNumber(modBus.getReg(addr, inp));
    }
    else if (aFunc=="modbus_getsreg" && aArgs.size()>=1 && aArgs.size()<=2) {
      // modbus_getsreg(regaddr [,input])
      int addr = aArgs[0].intValue();
      bool inp = aArgs[1].boolValue(); // null or false means non-input
      aResult.setNumber((int16_t)modBus.getReg(addr, inp));
    }
    else if (aFunc=="modbus_getbit" && aArgs.size()>=1 && aArgs.size()<=2) {
      // modbus_getbit(regaddr [,input])
      int addr = aArgs[0].intValue();
      bool inp = aArgs[1].boolValue(); // null or false means non-input
      aResult.setBool(modBus.getBit(addr, inp));
    }
    else if (aFunc=="activitytimeout" && aArgs.size()==1) {
      activityTimeout = aArgs[0].numValue()*Second;
    }
    else if (aFunc=="backlight" && (aArgs.size()==1 || aArgs.size()==3 || aArgs.size()==4)) {
      // backlight(active, [standby, timeout [, fadetime]])
      if (backlight) {
        backlight->setActiveBrightness(aArgs[0].numValue());
        if (aArgs.size()>=3) {
          backlight->setStandbyBrightness(aArgs[1].numValue());
          MLMicroSeconds bt = aArgs[2].numValue()*Second;
          if (bt!=backlightTimeout) {
            backlightTimeout = bt;
            if (backlightTimeout<0) {
              backlight->setActive(false); // force standby brightness
              backlight->suppressNextActivation(); // avoid next activation
            }
          }
          if (aArgs.size()>=4) {
            backlight->setFadeTime(aArgs[3].numValue()*Second);
          }
        }
      }
    }
    else if (aFunc=="exit" && aArgs.size()==1) {
      terminateApp(aArgs[0].numValue());
    }
    else {
      // unknown function
      return false;
    }
    return true;
  }


  ErrorPtr processModbusConfig(JsonObjectPtr aConfig)
  {
    ErrorPtr err;
    JsonObjectPtr mbCfg = aConfig->get("modbus");
    if (mbCfg) {
      JsonObjectPtr o;
      if (mbCfg->get("writehandler", o)) {
        modbusWriteScript = o->stringValue();
      }
      if (mbCfg->get("readhandler", o)) {
        modbusReadScript = o->stringValue();
      }
    }
    return err;
  }


  // MARK: - littlevGL


  void initLvgl()
  {
    LOG(LOG_NOTICE, "initializing littlevGL");
    LvGL::lvgl().init(getOption("mousecursor"));
    // create app UI
    // - install app specific script functions
    ui.uiScriptContext.registerFunctionHandler(boost::bind(&P44mbcd::uiFunctionHandler, this, _1, _2, _3, _4));
    // - init display
    ui.initForDisplay(lv_disp_get_default());
    // - load display config
    configUi();
  }


  void configUi()
  {
    ErrorPtr err;
    JsonObjectPtr uiConfig = JsonObject::objFromFile(dataPath(UICONFIG_FILE_NAME).c_str(), &err, true);
    if (Error::isError(err, SysError::domain(), ENOENT)) {
      // try resources
      err.reset();
      uiConfig = JsonObject::objFromFile(resourcePath(UICONFIG_FILE_NAME).c_str(), &err, true);
    }
    if (uiConfig && Error::isOK(err)) {
      LOG(LOG_INFO, "JSON read: %s", uiConfig->json_c_str());
      err = processModbusConfig(uiConfig);
      if (Error::isOK(err)) {
        // check for global UI script
        JsonObjectPtr o;
        if (uiConfig->get("initscript", o)) {
          initScript = o->stringValue();
        }
        if (uiConfig->get("activitytimeoutscript", o)) {
          activityTimeoutScript = o->stringValue();
        }
        if (uiConfig->get("activationscript", o)) {
          activationScript = o->stringValue();
        }
        // read UI config
        err = ui.setConfig(uiConfig);
      }
    }
    if (Error::notOK(err)) {
      LOG(LOG_ERR, "Failed creating UI from config: %s", Error::text(err));
      fatalErrorScreen(string_format("UI config error: %s", Error::text(err)));
    }
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


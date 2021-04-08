//
//  Copyright (c) 2019 plan44.ch / Lukas Zeller, Zurich, Switzerland
//

#ifndef __p44utils__config__
#define __p44utils__config__

#ifndef ENABLE_P44SCRIPT
  #define ENABLE_P44SCRIPT 1 // p44cript engine support in some of the p44utils components
#endif
#ifndef ENABLE_JSON_APPLICATION
  #define ENABLE_JSON_APPLICATION 1 // enables JSON utilities in Application
#endif
#define ENABLE_P44LRGRAPHICS 0 // p44lrgraphics support in some of the p44utils components
#define ENABLE_SOCKET_SCRIPT_FUNCS 0 // no socket support
#define ENABLE_HTTP_SCRIPT_FUNCS 0 // no http support
#define ENABLE_ANALOGIO_COLOR_SUPPORT 0 // no analog IO color support

#endif // __p44utils__config__

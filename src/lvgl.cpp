//
//  Copyright (c) 2019 plan44.ch / Lukas Zeller, Zurich, Switzerland
//
//  Author: Lukas Zeller <luz@plan44.ch>
//
//  This file is part of p44utils.
//
//  p44utils is free software: you can redistribute it and/or modify
//  it under the terms of the GNU General Public License as published by
//  the Free Software Foundation, either version 3 of the License, or
//  (at your option) any later version.
//
//  p44utils is distributed in the hope that it will be useful,
//  but WITHOUT ANY WARRANTY; without even the implied warranty of
//  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//  GNU General Public License for more details.
//
//  You should have received a copy of the GNU General Public License
//  along with p44utils. If not, see <http://www.gnu.org/licenses/>.
//

// File scope debugging options
// - Set ALWAYS_DEBUG to 1 to enable DBGLOG output even in non-DEBUG builds of this file
#define ALWAYS_DEBUG 0
// - set FOCUSLOGLEVEL to non-zero log level (usually, 5,6, or 7==LOG_DEBUG) to get focus (extensive logging) for this file
//   Note: must be before including "logger.hpp" (or anything that includes "logger.hpp")
#define FOCUSLOGLEVEL 7

#include "lvgl.hpp"

#if ENABLE_LVGL

using namespace p44;

LvGL::LvGL() :
  dispdev(NULL),
  pointer_indev(NULL),
  keyboard_indev(NULL),
  showCursor(false),
  buf1(NULL)
{
}


LvGL::~LvGL()
{
  if (buf1) {
    delete[] buf1;
    buf1 = NULL;
  }
}


static LvGL* lvglP = NULL;

LvGL& LvGL::lvgl()
{
  if (!lvglP) {
    lvglP = new LvGL;
  }
  return *lvglP;
}


#if LV_USE_LOG

extern "C" void lvgl_log_cb(lv_log_level_t level, const char *file, uint32_t line, const char *dsc)
{
  int logLevel = LOG_WARNING;
  switch (level) {
    case LV_LOG_LEVEL_TRACE: logLevel = LOG_DEBUG; break; // A lot of logs to give detailed information
    case LV_LOG_LEVEL_INFO: logLevel = LOG_INFO; break; // Log important events
    case LV_LOG_LEVEL_WARN: logLevel = LOG_WARNING; break; // Log if something unwanted happened but didn't caused problem
    case LV_LOG_LEVEL_ERROR: logLevel = LOG_ERR; break; // Only critical issue, when the system may fail
  }
  LOG(logLevel, "lvgl %s:%d - %s", file, line, dsc);
}

#endif // LV_USE_LOG


#define DISPLAY_BUFFER_LINES 10
#define DISPLAY_BUFFER_SIZE (LV_HOR_RES_MAX * DISPLAY_BUFFER_LINES)

void LvGL::init(bool aShowCursor)
{
  showCursor = aShowCursor;
  // init library
  lv_init();
  #if LV_USE_LOG
  lv_log_register_print_cb(lvgl_log_cb);
  #endif
  // init disply buffer
  buf1 = new lv_color_t[DISPLAY_BUFFER_SIZE];
  lv_disp_buf_init(&disp_buf, buf1, NULL, DISPLAY_BUFFER_SIZE);
  // init the display driver
  lv_disp_drv_t disp_drv;
  lv_disp_drv_init(&disp_drv);
  disp_drv.hor_res = LV_HOR_RES_MAX;
  disp_drv.ver_res = LV_VER_RES_MAX;
  disp_drv.buffer = &disp_buf;
  #if defined(__APPLE__)
  // - use SDL2 monitor
  monitor_init();
  disp_drv.flush_cb = monitor_flush;
  #else
  // - use fbdev framebuffer device
  fbdev_init();
  disp_drv.flush_cb = fbdev_flush;
  #endif
  dispdev = lv_disp_drv_register(&disp_drv);
  // init input driver
  lv_indev_drv_t pointer_indev_drv;
  lv_indev_drv_init(&pointer_indev_drv);
  pointer_indev_drv.type = LV_INDEV_TYPE_POINTER;
  #if defined(__APPLE__)
  // - use mouse
  mouse_init();
  pointer_indev_drv.read_cb = mouse_read;
  #else
  // - init input driver
  evdev_init();
  pointer_indev_drv.read_cb = evdev_read;
  #endif
  pointer_indev = lv_indev_drv_register(&pointer_indev_drv);  /*Register the driver in LittlevGL*/
  #if MOUSE_CURSOR_SUPPORT
  if (showCursor) {
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
  }
  #endif // MOUSE_CURSOR_SUPPORT
  // - schedule updates
  lvglTicket.executeOnce(boost::bind(&LvGL::lvglTask, this, _1, _2));
}


#define LVGL_TICK_PERIOD (5*MilliSecond)

#if !LV_TICK_CUSTOM
  #warning LV_TICK_CUSTOM must be set, p44::LvGL does not call lv_tick_inc
#endif


void LvGL::lvglTask(MLTimer &aTimer, MLMicroSeconds aNow)
{
  lv_task_handler();
  #if defined(__APPLE__)
  // also need to update SDL2
  monitor_sdl_refr_core();
  #endif
  MainLoop::currentMainLoop().retriggerTimer(aTimer, LVGL_TICK_PERIOD);
}





#endif // ENABLE_LVGL

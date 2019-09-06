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

#include "application.hpp"

#if NOT_WIP
#include <png.h>
#endif

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

// MARK: - logging

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


// MARK: - readonly file system for images

#if LV_USE_FILESYSTEM

typedef int pf_file_t; ///< platform file type

/// Open a native platform (posix) file
/// @param drv pointer to the current driver
/// @param file_p pointer to a file descriptor
/// @param fn name of the file.
/// @param mode element of 'fs_mode_t' enum or its 'OR' connection (e.g. FS_MODE_WR | FS_MODE_RD)
/// @return LV_FS_RES_OK: no error, the file is opened
///         any error from lv_fs_res_t enum
static lv_fs_res_t pf_open(lv_fs_drv_t*, void* file_p, const char* fn, lv_fs_mode_t mode)
{
  errno = 0;

  // make path relative to resource path
  string path = Application::sharedApplication()->resourcePath(fn);
  // read only
  if (mode!=LV_FS_MODE_RD) return LV_FS_RES_DENIED;
  pf_file_t fd = open(path.c_str(), O_RDONLY);
  if (fd<0) return LV_FS_RES_UNKNOWN;
  lseek(fd, 0, SEEK_SET);
  *((pf_file_t *)file_p) = fd;
  return LV_FS_RES_OK;
}


/// Close an opened file
/// @param drv pointer to the current driver
/// @param file_p pointer to a file descriptor
/// @return LV_FS_RES_OK: no error, the file is read
///         any error from lv__fs_res_t enum
static lv_fs_res_t pf_close(lv_fs_drv_t*, void *file_p)
{
  pf_file_t fd = *((pf_file_t *)file_p);
  close(fd);
  return LV_FS_RES_OK;
}


/// Read data from an opened file
/// @param drv pointer to the current driver
/// @param file_p pointer to a file descriptor
/// @param buf pointer to a memory block where to store the read data
/// @param btr number of Bytes To Read
/// @param br the real number of read bytes (Byte Read)
/// @return LV_FS_RES_OK: no error, the file is read
///         any error from lv__fs_res_t enum
static lv_fs_res_t pf_read(lv_fs_drv_t*, void* file_p, void *buf, uint32_t btr, uint32_t *br)
{
  pf_file_t fd = *((pf_file_t *)file_p);
  ssize_t by = read(fd, buf, btr);
  if (by<0) return LV_FS_RES_UNKNOWN;
  *br = (uint32_t)by;
  return LV_FS_RES_OK;
}

/// Set the read write pointer. Also expand the file size if necessary.
/// @param drv pointer to the current driver
/// @param file_p pointer to a file descriptor
/// @param pos the new position of read write pointer
/// @return LV_FS_RES_OK: no error, the file is read
///         any error from lv__fs_res_t enum
static lv_fs_res_t pf_seek(lv_fs_drv_t*, void* file_p, uint32_t pos)
{
  pf_file_t fd = *((pf_file_t *)file_p);
  off_t off = lseek(fd, pos, SEEK_SET);
  return off<0 ? LV_FS_RES_UNKNOWN : LV_FS_RES_OK;
}

/// Give the position of the read write pointer
/// @param drv pointer to the current driver
/// @param file_p pointer to a file descriptor
/// @param pos_p pointer to to store the result
/// @return LV_FS_RES_OK: no error, the file is read
///         any error from lv__fs_res_t enum
static lv_fs_res_t pf_tell(lv_fs_drv_t*, void* file_p, uint32_t* pos_p)
{
  pf_file_t fd = *((pf_file_t *)file_p);
  off_t off = lseek(fd, 0, SEEK_CUR);
  if (off<0) return LV_FS_RES_UNKNOWN;
  *pos_p = (uint32_t)off;
  return LV_FS_RES_OK;
}

#endif // LV_USE_FILESYSTEM


#ifdef NOT_WIP

// MARK: - PNG image decoder

/// Get info about a PNG image
/// @param decoder pointer to the decoder where this function belongs
/// @param src can be file name or pointer to a C array
/// @param header store the info here
/// @return LV_RES_OK: no error; LV_RES_INV: can't get the info
static lv_res_t png_decoder_info(lv_img_decoder_t * decoder, const void* src, lv_img_header_t* header)
{
  png_image pngImage; /// The control structure used by libpng
  lv_img_src_t imgtype = lv_img_src_get_type(src);
  if (imgtype==LV_IMG_SRC_FILE) {

  }
  else if (imgtype==LV_IMG_SRC_VARIABLE) {
    const lv_img_header_t &hdr = *((lv_img_header_t*)src);
  }


  size_t size = 100;
  if (png_image_begin_read_from_memory(&pngImage, src, size) == 0) {
    // error
    return LV_RES_INV;
  }

  /*
  if (png_image_begin_read_from_file(&pngImage, aPNGFileName.c_str()) == 0) {
    // error
    return TextError::err("could not open PNG file %s", aPNGFileName.c_str());
  }
  else {
    // We only need the luminance
    pngImage.format = PNG_FORMAT_RGBA;
    // Now allocate enough memory to hold the image in this format; the
    // PNG_IMAGE_SIZE macro uses the information about the image (width,
    // height and format) stored in 'image'.
    pngBuffer = (png_bytep)malloc(PNG_IMAGE_SIZE(pngImage));
    LOG(LOG_INFO, "Image size in bytes = %d", PNG_IMAGE_SIZE(pngImage));
    LOG(LOG_INFO, "Image width = %d", pngImage.width);
    LOG(LOG_INFO, "Image height = %d", pngImage.height);
    LOG(LOG_INFO, "Image width*height = %d", pngImage.height*pngImage.width);
    contentSizeX = pngImage.width;
    contentSizeY = pngImage.height;
    if (pngBuffer==NULL) {

   */


//  if(is_png(src) == false) return LV_RES_INV;
//
//  ...
//
//  header->cf = LV_IMG_CF_RAW_ALPHA;
//  header->w = width;
//  header->h = height;
  return LV_FS_RES_OK;
}

/**
 * Open a PNG image and return the decided image
 * @param decoder pointer to the decoder where this function belongs
 * @param dsc pointer to a descriptor which describes this decoding session
 * @return LV_RES_OK: no error; LV_RES_INV: can't get the info
 */
static lv_res_t png_decoder_open(lv_img_decoder_t * decoder, lv_img_decoder_dsc_t * dsc)
{

//  /*Check whether the type `src` is known by the decoder*/
//  if(is_png(src) == false) return LV_RES_INV;
//
//  /*Decode and store the image. If `dsc->img_data` the `read_line` function will be called to get the image data liny-by-line*/
//  dsc->img_data = my_png_decoder(src);
//
//  /*Change the color format if required. For PNG usually 'Raw' is fine*/
//  dsc->header.cf = LV_IMG_CF_...
//
//  /*Call a built in decoder function if required. It's not required if`my_png_decoder` opened the image in true color format.*/
//  lv_res_t res = lv_img_decoder_built_in_open(decoder, dsc);
//
//  return res;
  return LV_FS_RES_OK;
}

/**
 * Decode `len` pixels starting from the given `x`, `y` coordinates and store them in `buf`.
 * Required only if the "open" function can't open the whole decoded pixel array. (dsc->img_data == NULL)
 * @param decoder pointer to the decoder the function associated with
 * @param dsc pointer to decoder descriptor
 * @param x start x coordinate
 * @param y start y coordinate
 * @param len number of pixels to decode
 * @param buf a buffer to store the decoded pixels
 * @return LV_RES_OK: ok; LV_RES_INV: failed
 */
lv_res_t png_decoder_built_in_read_line(lv_img_decoder_t * decoder, lv_img_decoder_dsc_t * dsc, lv_coord_t x,
                                    lv_coord_t y, lv_coord_t len, uint8_t * buf)
{
  /*With PNG it's usually not required*/

  /*Copy `len` pixels from `x` and `y` coordinates in True color format to `buf` */

  return LV_FS_RES_OK;
}

/**
 * Free the allocated resources
 * @param decoder pointer to the decoder where this function belongs
 * @param dsc pointer to a descriptor which describes this decoding session
 */
static void png_decoder_close(lv_img_decoder_t * decoder, lv_img_decoder_dsc_t * dsc)
{
  /*Free all allocated data*/

  /*Call the built-in close function if the built-in open/read_line was used*/
//  lv_img_decoder_built_in_close(decoder, dsc);

}

#endif


// MARK: - littlevGL initialisation

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
    lv_obj_set_click(cursor, false); // important, or all clicks get caught by the cursor itself!
    lv_indev_set_cursor(pointer_indev, cursor);
  }
  #endif // MOUSE_CURSOR_SUPPORT
  // - register (readonly, only for getting images) file system support
  #if LV_USE_FILESYSTEM
  memset(&pf_fs_drv, 0, sizeof(lv_fs_drv_t));    // Initialization
  pf_fs_drv.file_size = sizeof(pf_file_t);       // Set up fields
  pf_fs_drv.letter = 'P';
  pf_fs_drv.open_cb = pf_open;
  pf_fs_drv.close_cb = pf_close;
  pf_fs_drv.read_cb = pf_read;
  pf_fs_drv.seek_cb = pf_seek;
  pf_fs_drv.tell_cb = pf_tell;
  lv_fs_drv_register(&pf_fs_drv);
  #endif
  #if NOT_WIP
  // - PNG decoder
  lv_img_decoder_t * dec = lv_img_decoder_create();
  lv_img_decoder_set_info_cb(dec, png_decoder_info);
  lv_img_decoder_set_open_cb(dec, png_decoder_open);
  lv_img_decoder_set_close_cb(dec, png_decoder_close);
  #endif
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

AUTOMAKE_OPTIONS = subdir-objects
ACLOCAL_AMFLAGS = ${ACLOCAL_FLAGS} -I m4

if P44_BUILD_WIN

bin_PROGRAMS = p44mbutil

else

bin_PROGRAMS = p44mbcd p44mbutil

endif

# embedded libmodbus
if EMBEDDED_LIBMODBUS

LIBMODBUS_SOURCES = \
  src/p44utils/thirdparty/libmodbus/src/modbus-data.c \
  src/p44utils/thirdparty/libmodbus/src/modbus-private.h \
  src/p44utils/thirdparty/libmodbus/src/modbus-rtu-private.h \
  src/p44utils/thirdparty/libmodbus/src/modbus-rtu.c \
  src/p44utils/thirdparty/libmodbus/src/modbus-rtu.h \
  src/p44utils/thirdparty/libmodbus/src/modbus-tcp-private.h \
  src/p44utils/thirdparty/libmodbus/src/modbus-tcp.c \
  src/p44utils/thirdparty/libmodbus/src/modbus-tcp.h \
  src/p44utils/thirdparty/libmodbus/src/modbus-version.h \
  src/p44utils/thirdparty/libmodbus/src/modbus.c \
  src/p44utils/thirdparty/libmodbus/src/modbus.h

LIBMODBUS_CFLAGS = -I ${srcdir}/src/p44utils/thirdparty/libmodbus/src -D EMBEDDED_LIBMODBUS=1

else

LIBMODBUS_CFLAGS = ${MODBUS_CFLAGS}
LIBMODBUS_LIBS = ${MODBUS_LIBS}

endif


# ubus support via libubus
if ENABLE_UBUS

UBUS_SOURCES = \
  src/p44utils/ubus.cpp \
  src/p44utils/ubus.hpp
UBUS_FLAGS = -D ENABLE_UBUS=1

else

UBUS_FLAGS = -D ENABLE_UBUS=0

endif


# littlevGL
LVGL_SOURCES = \
  src/lv_conf.h \
  src/lv_drv_conf.h \
  src/lvgl/src/lv_core/lv_debug.c \
  src/lvgl/src/lv_core/lv_disp.c \
  src/lvgl/src/lv_core/lv_group.c \
  src/lvgl/src/lv_core/lv_indev.c \
  src/lvgl/src/lv_core/lv_obj.c \
  src/lvgl/src/lv_core/lv_refr.c \
  src/lvgl/src/lv_core/lv_style.c \
  src/lvgl/src/lv_draw/lv_draw_arc.c \
  src/lvgl/src/lv_draw/lv_draw_basic.c \
  src/lvgl/src/lv_draw/lv_draw_img.c \
  src/lvgl/src/lv_draw/lv_draw_label.c \
  src/lvgl/src/lv_draw/lv_draw_line.c \
  src/lvgl/src/lv_draw/lv_draw_rect.c \
  src/lvgl/src/lv_draw/lv_draw_triangle.c \
  src/lvgl/src/lv_draw/lv_draw.c \
  src/lvgl/src/lv_draw/lv_img_cache.c \
  src/lvgl/src/lv_draw/lv_img_decoder.c \
  src/lvgl/src/lv_font/lv_font_fmt_txt.c \
  src/lvgl/src/lv_font/lv_font_roboto_12.c \
  src/lvgl/src/lv_font/lv_font_roboto_16.c \
  src/lvgl/src/lv_font/lv_font_roboto_22.c \
  src/lvgl/src/lv_font/lv_font_roboto_28.c \
  src/lvgl/src/lv_font/lv_font_unscii_8.c \
  src/lvgl/src/lv_font/lv_font.c \
  src/lvgl/src/lv_hal/lv_hal_disp.c \
  src/lvgl/src/lv_hal/lv_hal_indev.c \
  src/lvgl/src/lv_hal/lv_hal_tick.c \
  src/lvgl/src/lv_misc/lv_anim.c \
  src/lvgl/src/lv_misc/lv_area.c \
  src/lvgl/src/lv_misc/lv_async.c \
  src/lvgl/src/lv_misc/lv_circ.c \
  src/lvgl/src/lv_misc/lv_color.c \
  src/lvgl/src/lv_misc/lv_fs.c \
  src/lvgl/src/lv_misc/lv_gc.c \
  src/lvgl/src/lv_misc/lv_ll.c \
  src/lvgl/src/lv_misc/lv_log.c \
  src/lvgl/src/lv_misc/lv_math.c \
  src/lvgl/src/lv_misc/lv_mem.c \
  src/lvgl/src/lv_misc/lv_printf.c \
  src/lvgl/src/lv_misc/lv_task.c \
  src/lvgl/src/lv_misc/lv_txt.c \
  src/lvgl/src/lv_misc/lv_utils.c \
  src/lvgl/src/lv_objx/lv_arc.c \
  src/lvgl/src/lv_objx/lv_bar.c \
  src/lvgl/src/lv_objx/lv_btn.c \
  src/lvgl/src/lv_objx/lv_btnm.c \
  src/lvgl/src/lv_objx/lv_calendar.c \
  src/lvgl/src/lv_objx/lv_canvas.c \
  src/lvgl/src/lv_objx/lv_cb.c \
  src/lvgl/src/lv_objx/lv_chart.c \
  src/lvgl/src/lv_objx/lv_cont.c \
  src/lvgl/src/lv_objx/lv_ddlist.c \
  src/lvgl/src/lv_objx/lv_gauge.c \
  src/lvgl/src/lv_objx/lv_img.c \
  src/lvgl/src/lv_objx/lv_imgbtn.c \
  src/lvgl/src/lv_objx/lv_kb.c \
  src/lvgl/src/lv_objx/lv_label.c \
  src/lvgl/src/lv_objx/lv_led.c \
  src/lvgl/src/lv_objx/lv_line.c \
  src/lvgl/src/lv_objx/lv_list.c \
  src/lvgl/src/lv_objx/lv_lmeter.c \
  src/lvgl/src/lv_objx/lv_mbox.c \
  src/lvgl/src/lv_objx/lv_objx_templ.c \
  src/lvgl/src/lv_objx/lv_page.c \
  src/lvgl/src/lv_objx/lv_preload.c \
  src/lvgl/src/lv_objx/lv_roller.c \
  src/lvgl/src/lv_objx/lv_slider.c \
  src/lvgl/src/lv_objx/lv_spinbox.c \
  src/lvgl/src/lv_objx/lv_sw.c \
  src/lvgl/src/lv_objx/lv_ta.c \
  src/lvgl/src/lv_objx/lv_table.c \
  src/lvgl/src/lv_objx/lv_tabview.c \
  src/lvgl/src/lv_objx/lv_tileview.c \
  src/lvgl/src/lv_objx/lv_win.c \
  src/lvgl/src/lv_themes/lv_theme_alien.c \
  src/lvgl/src/lv_themes/lv_theme_default.c \
  src/lvgl/src/lv_themes/lv_theme_material.c \
  src/lvgl/src/lv_themes/lv_theme_mono.c \
  src/lvgl/src/lv_themes/lv_theme_nemo.c \
  src/lvgl/src/lv_themes/lv_theme_night.c \
  src/lvgl/src/lv_themes/lv_theme_templ.c \
  src/lvgl/src/lv_themes/lv_theme_zen.c \
  src/lvgl/src/lv_themes/lv_theme.c \
  src/lv_drivers/indev/FT5406EE8.c \
  src/lv_drivers/indev/keyboard.c \
  src/lv_drivers/indev/mouse.c \
  src/lv_drivers/indev/mousewheel.c \
  src/lv_drivers/indev/evdev.c \
  src/lv_drivers/indev/XPT2046.c \
  src/lv_drivers/display/fbdev.c \
  src/lv_drivers/display/monitor.c \
  src/lv_drivers/display/R61581.c \
  src/lv_drivers/display/SSD1963.c \
  src/lv_drivers/display/ST7565.c


LVGL_FLAGS = \
  -I ${srcdir}/src/lvgl/src/lv_core \
  -I ${srcdir}/src/lvgl/src/lv_misc \
  -I ${srcdir}/src/lvgl/src/lv_themes \
  -I ${srcdir}/src/lvgl/src/lv_font \
  -I ${srcdir}/src/lvgl/src/lv_hal \
  -I ${srcdir}/src/lvgl/src/lv_draw \
  -I ${srcdir}/src/lvgl/src/lv_objx \
  -I ${srcdir}/src/lv_drivers \
  -I ${srcdir}/src/lv_drivers/indev \
  -I ${srcdir}/src/lv_drivers/display \
  -I ${srcdir}/src/lv_examples/lv_apps/demo





# p44mbcd

if !P44_BUILD_WIN

if DEBUG
p44mbcd_DEBUG = -D DEBUG=1
endif

if P44_BUILD_OW
p44mbcd_PLATFORM = -D P44_BUILD_OW=1
endif


# main

p44mbcd_LDADD = ${JSONC_LIBS} ${LIBMODBUS_LIBS} ${PTHREAD_LIBS} ${SQLITE3_LIBS} $(PNG_LIBS)
p44mbcd_EXTRACFLAGS = -D NO_SSL_DL=1


p44mbcd_CPPFLAGS = \
  -I ${srcdir}/src/p44utils \
  -I ${srcdir}/src \
  -I ${srcdir}/src/p44utils/thirdparty/civetweb \
  ${BOOST_CPPFLAGS} \
  ${JSONC_CFLAGS} \
  ${LIBMODBUS_CFLAGS} \
  ${PTHREAD_CFLAGS} \
  ${PNG_CFLAGS} \
  ${UBUS_FLAGS} \
  ${LVGL_FLAGS} \
  ${p44mbcd_EXTRACFLAGS} \
  ${p44mbcd_PLATFORM} \
  ${p44mbcd_DEBUG}



p44mbcd_SOURCES = \
  ${UBUS_SOURCES} \
  ${LIBMODBUS_SOURCES} \
  ${LVGL_SOURCES} \
  src/p44utils/analogio.cpp \
  src/p44utils/analogio.hpp \
  src/p44utils/application.cpp \
  src/p44utils/application.hpp \
  src/p44utils/consolekey.cpp \
  src/p44utils/consolekey.hpp \
  src/p44utils/crc32.cpp \
  src/p44utils/crc32.hpp \
  src/p44utils/digitalio.cpp \
  src/p44utils/digitalio.hpp \
  src/p44utils/error.cpp \
  src/p44utils/error.hpp \
  src/p44utils/p44script.cpp \
  src/p44utils/p44script.hpp \
  src/p44utils/fdcomm.cpp \
  src/p44utils/fdcomm.hpp \
  src/p44utils/fnv.cpp \
  src/p44utils/fnv.hpp \
  src/p44utils/gpio.cpp \
  src/p44utils/gpio.h \
  src/p44utils/gpio.hpp \
  src/p44utils/httpcomm.cpp\
  src/p44utils/httpcomm.hpp\
  src/p44utils/i2c.cpp \
  src/p44utils/i2c.hpp \
  src/p44utils/iopin.cpp \
  src/p44utils/iopin.hpp \
  src/p44utils/jsonobject.cpp \
  src/p44utils/jsonobject.hpp \
  src/p44utils/jsonwebclient.cpp\
  src/p44utils/jsonwebclient.hpp\
  src/p44utils/logger.cpp \
  src/p44utils/logger.hpp \
  src/p44utils/lvgl.cpp \
  src/p44utils/lvgl.hpp \
  src/p44utils/lvglui.cpp \
  src/p44utils/lvglui.hpp \
  src/p44utils/macaddress.cpp \
  src/p44utils/macaddress.hpp \
  src/p44utils/mainloop.cpp \
  src/p44utils/mainloop.hpp \
  src/p44utils/modbus.cpp \
  src/p44utils/modbus.hpp \
  src/p44utils/p44obj.cpp \
  src/p44utils/p44obj.hpp \
  src/p44utils/pwm.cpp \
  src/p44utils/pwm.hpp \
  src/p44utils/serialcomm.cpp \
  src/p44utils/serialcomm.hpp \
  src/p44utils/socketcomm.cpp \
  src/p44utils/socketcomm.hpp \
  src/p44utils/spi.cpp \
  src/p44utils/spi.hpp \
  src/p44utils/timeutils.cpp \
  src/p44utils/timeutils.hpp \
  src/p44utils/utils.cpp \
  src/p44utils/utils.hpp \
  src/p44utils/extutils.cpp \
  src/p44utils/extutils.hpp \
  src/p44utils/thirdparty/civetweb/civetweb.c \
  src/p44utils/thirdparty/civetweb/civetweb.h \
  src/p44utils/thirdparty/civetweb/handle_form.inl \
  src/p44utils/thirdparty/civetweb/md5.inl \
  src/p44utils/thirdparty/civetweb/sha1.inl \
  src/p44utils/thirdparty/civetweb/hostcheck.inl \
  src/p44utils/thirdparty/civetweb/openssl_hostname_validation.inl \
  src/p44utils/p44utils_common.hpp \
  src/p44utils_config.hpp \
  src/p44mbcd_main.cpp

endif

# p44mbutil

if DEBUG
p44mbutil_DEBUG = -D DEBUG=1
endif

if P44_BUILD_OW
p44mbutil_PLATFORM = -D P44_BUILD_OW=1
endif

if P44_BUILD_WIN
p44mbutil_PLATFORM = -D P44_BUILD_WIN=1 -static -static-libgcc -static-libstdc++
p44mbutil_LDFLAGS = -static -static-libgcc -static-libstdc++
endif


# main

p44mbutil_LDADD = ${LIBMODBUS_LIBS} ${PTHREAD_LIBS}
p44mbutil_EXTRACFLAGS = -D NO_SSL_DL=1 -D ENABLE_P44SCRIPT=0 -D ENABLE_JSON_APPLICATION=0


p44mbutil_CPPFLAGS = \
  -I ${srcdir}/src/p44utils \
  -I ${srcdir}/src \
  ${BOOST_CPPFLAGS} \
  ${LIBMODBUS_CFLAGS} \
  ${PTHREAD_CFLAGS} \
  ${p44mbutil_EXTRACFLAGS} \
  ${p44mbutil_PLATFORM} \
  ${p44mbutil_DEBUG}



p44mbutil_SOURCES = \
  ${LIBMODBUS_SOURCES} \
  src/p44utils/application.cpp \
  src/p44utils/application.hpp \
  src/p44utils/consolekey.cpp \
  src/p44utils/consolekey.hpp \
  src/p44utils/digitalio.cpp \
  src/p44utils/digitalio.hpp \
  src/p44utils/i2c.cpp \
  src/p44utils/i2c.hpp \
  src/p44utils/spi.cpp \
  src/p44utils/spi.hpp \
  src/p44utils/pwm.cpp \
  src/p44utils/pwm.hpp \
  src/p44utils/analogio.cpp \
  src/p44utils/analogio.hpp \
  src/p44utils/error.cpp \
  src/p44utils/error.hpp \
  src/p44utils/fnv.cpp \
  src/p44utils/fnv.hpp \
  src/p44utils/crc32.cpp \
  src/p44utils/crc32.hpp \
  src/p44utils/fdcomm.cpp \
  src/p44utils/fdcomm.hpp \
  src/p44utils/gpio.cpp \
  src/p44utils/gpio.h \
  src/p44utils/gpio.hpp \
  src/p44utils/iopin.cpp \
  src/p44utils/iopin.hpp \
  src/p44utils/logger.cpp \
  src/p44utils/logger.hpp \
  src/p44utils/macaddress.cpp \
  src/p44utils/macaddress.hpp \
  src/p44utils/mainloop.cpp \
  src/p44utils/mainloop.hpp \
  src/p44utils/p44obj.cpp \
  src/p44utils/p44obj.hpp \
  src/p44utils/serialcomm.cpp \
  src/p44utils/serialcomm.hpp \
  src/p44utils/modbus.cpp \
  src/p44utils/modbus.hpp \
  src/p44utils/utils.cpp \
  src/p44utils/utils.hpp \
  src/p44utils/p44utils_common.hpp \
  src/p44utils_config.hpp \
  src/p44mbutil_main.cpp

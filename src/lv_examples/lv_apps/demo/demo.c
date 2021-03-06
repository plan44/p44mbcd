/**
 * @file demo.c
 *
 */

/*********************
 *      INCLUDES
 *********************/
#include "demo.h"
#if USE_LV_DEMO

#include <stdio.h>

/*********************
 *      DEFINES
 *********************/

/**********************
 *      TYPEDEFS
 **********************/

/**********************
 *  STATIC PROTOTYPES
 **********************/
static void write_create(lv_obj_t * parent);
static void textarea_event(lv_obj_t * text_area, lv_event_t event);
static void keyboard_hide_action(lv_obj_t * keyboard);
static void kb_hide_anim_end(lv_obj_t * keyboard);
static void list_create(lv_obj_t * parent);
static void chart_create(lv_obj_t * parent);
static void p44mbc_create(lv_obj_t * parent);
static lv_res_t slider_action(lv_obj_t * slider);
static lv_res_t list_btn_action(lv_obj_t * slider);
#if LV_DEMO_SLIDE_SHOW
static void tab_switcher(void * tv);
#endif
#if USE_THEME
static lv_theme_t *th;
#endif


/**********************
 *  STATIC VARIABLES
 **********************/
static lv_obj_t * chart;
static lv_obj_t * ta;
static lv_obj_t * kb;

// p44mbc
static lv_obj_t * plusButton;
static lv_obj_t * minusButton;
static lv_obj_t * dispLabel;

static p44BtnCallBack_t p44BtnCallBack = NULL;

static lv_style_t style_kb;
static lv_style_t style_kb_rel;
static lv_style_t style_kb_pr;

#if LV_DEMO_WALLPAPER
LV_IMG_DECLARE(img_bubble_pattern);
#endif

/**********************
 *      MACROS
 **********************/

/**********************
 *   GLOBAL FUNCTIONS
 **********************/

/**
 * Create a demo application
 */
void demo_create(void)
{

#if USE_THEME
    th = lv_theme_material_init(10, NULL);
    lv_theme_set_current(th);
#endif


#if LV_DEMO_WALLPAPER
    lv_obj_t * wp = lv_img_create(lv_scr_act(), NULL);
    lv_img_set_src(wp, &img_bubble_pattern);
    lv_obj_set_width(wp, LV_HOR_RES * 4);
    lv_obj_set_protect(wp, LV_PROTECT_POS);
#endif

    static lv_style_t style_tv_btn_bg;
    lv_style_copy(&style_tv_btn_bg, &lv_style_plain);
    style_tv_btn_bg.body.main_color = lv_color_hex(0x487fb7);
    style_tv_btn_bg.body.grad_color = lv_color_hex(0x487fb7);
    style_tv_btn_bg.body.padding.top = 0;
    style_tv_btn_bg.body.padding.bottom = 0;

    static lv_style_t style_tv_btn_rel;
    lv_style_copy(&style_tv_btn_rel, &lv_style_btn_rel);
    style_tv_btn_rel.body.border.width = 0;

    static lv_style_t style_tv_btn_pr;
    lv_style_copy(&style_tv_btn_pr, &lv_style_btn_pr);
    style_tv_btn_pr.body.radius = 0;
    style_tv_btn_pr.body.opa = LV_OPA_50;
    style_tv_btn_pr.body.main_color = LV_COLOR_WHITE;
    style_tv_btn_pr.body.grad_color = LV_COLOR_WHITE;
    style_tv_btn_pr.body.border.width = 0;
    style_tv_btn_pr.text.color = LV_COLOR_GRAY;

#if ONLY_DEMO
    p44mbc_create(lv_scr_act());
#else

    lv_obj_t * tv = lv_tabview_create(lv_scr_act(), NULL);

#if LV_DEMO_WALLPAPER
    lv_obj_set_parent(wp, ((lv_tabview_ext_t *) tv->ext_attr)->content);
    lv_obj_set_pos(wp, 0, -5);
#endif

    lv_obj_t * tab4 = lv_tabview_add_tab(tv, "Demo");
    lv_obj_t * tab1 = lv_tabview_add_tab(tv, "Write");
    lv_obj_t * tab2 = lv_tabview_add_tab(tv, "List");
    lv_obj_t * tab3 = lv_tabview_add_tab(tv, "Chart");

#if LV_DEMO_WALLPAPER == 0
    /*Blue bg instead of wallpaper*/
    lv_tabview_set_style(tv, LV_TABVIEW_STYLE_BG, &style_tv_btn_bg);
#endif
    lv_tabview_set_style(tv, LV_TABVIEW_STYLE_BTN_BG, &style_tv_btn_bg);
    lv_tabview_set_style(tv, LV_TABVIEW_STYLE_INDIC, &lv_style_plain);
    lv_tabview_set_style(tv, LV_TABVIEW_STYLE_BTN_REL, &style_tv_btn_rel);
    lv_tabview_set_style(tv, LV_TABVIEW_STYLE_BTN_PR, &style_tv_btn_pr);
    lv_tabview_set_style(tv, LV_TABVIEW_STYLE_BTN_TGL_REL, &style_tv_btn_rel);
    lv_tabview_set_style(tv, LV_TABVIEW_STYLE_BTN_TGL_PR, &style_tv_btn_pr);

    p44mbc_create(tab4);
    write_create(tab1);
    list_create(tab2);
    chart_create(tab3);

#if LV_DEMO_SLIDE_SHOW
    lv_task_create(tab_switcher, 3000, LV_TASK_PRIO_MID, tv);
#endif

#endif // !ONLY_DEMO

}


/**********************
 *   STATIC FUNCTIONS
 **********************/

static void write_create(lv_obj_t * parent)
{

    lv_page_set_style(parent, LV_PAGE_STYLE_BG, &lv_style_transp_fit);
    lv_page_set_style(parent, LV_PAGE_STYLE_SCRL, &lv_style_transp_fit);

    lv_page_set_sb_mode(parent, LV_SB_MODE_OFF);

    static lv_style_t style_ta;
    lv_style_copy(&style_ta, &lv_style_pretty);
    style_ta.body.opa = LV_OPA_30;
    style_ta.body.radius = 0;
    style_ta.text.color = lv_color_hex3(0x222);

    ta = lv_ta_create(parent, NULL);
    lv_obj_set_size(ta, lv_page_get_scrl_width(parent), lv_obj_get_height(parent) / 2);
    lv_ta_set_style(ta, LV_TA_STYLE_BG, &style_ta);
    lv_ta_set_text(ta, "");

    lv_obj_set_event_cb(ta, textarea_event);

    lv_style_copy(&style_kb, &lv_style_plain);
    style_kb.body.opa = LV_OPA_70;
    style_kb.body.main_color = lv_color_hex3(0x333);
    style_kb.body.grad_color = lv_color_hex3(0x333);
    style_kb.body.padding.top = 0;
    style_kb.body.padding.bottom = 0;
    style_kb.body.padding.left = 0;
    style_kb.body.padding.right = 0;
    style_kb.body.padding.inner = 0;

    lv_style_copy(&style_kb_rel, &lv_style_plain);
    style_kb_rel.body.radius = 0;
    style_kb_rel.body.border.width = 1;
    style_kb_rel.body.border.color = LV_COLOR_SILVER;
    style_kb_rel.body.border.opa = LV_OPA_50;
    style_kb_rel.body.main_color = lv_color_hex3(0x333);    /*Recommended if LV_VDB_SIZE == 0 and bpp > 1 fonts are used*/
    style_kb_rel.body.grad_color = lv_color_hex3(0x333);
    style_kb_rel.text.color = LV_COLOR_WHITE;

    lv_style_copy(&style_kb_pr, &lv_style_plain);
    style_kb_pr.body.radius = 0;
    style_kb_pr.body.opa = LV_OPA_50;
    style_kb_pr.body.main_color = LV_COLOR_WHITE;
    style_kb_pr.body.grad_color = LV_COLOR_WHITE;
    style_kb_pr.body.border.width = 1;
    style_kb_pr.body.border.color = LV_COLOR_SILVER;

    textarea_event(ta, LV_EVENT_RELEASED);
}

static void textarea_event(lv_obj_t * text_area, lv_event_t event)
{
    (void) text_area;    /*Unused*/

    if (event==LV_EVENT_RELEASED) {
        lv_obj_t * parent = lv_obj_get_parent(lv_obj_get_parent(ta));   /*Test area is on the scrollable part of the page but we need the page itself*/

        if(kb) {
            keyboard_hide_action(kb);
        } else {
            kb = lv_kb_create(parent, NULL);
            lv_obj_set_size(kb, lv_page_get_scrl_width(parent), lv_obj_get_height(parent) / 2);
            lv_obj_align(kb, ta, LV_ALIGN_OUT_BOTTOM_MID, 0, 0);
            lv_kb_set_ta(kb, ta);
            lv_kb_set_style(kb, LV_KB_STYLE_BG, &style_kb);
            lv_kb_set_style(kb, LV_KB_STYLE_BTN_REL, &style_kb_rel);
            lv_kb_set_style(kb, LV_KB_STYLE_BTN_PR, &style_kb_pr);
// TODO: v5.3 - how to do this in 6.0? -> see new demo?
//            lv_kb_set_hide_action(kb, keyboard_hide_action);
//            lv_kb_set_ok_action(kb, keyboard_hide_action);

        #if USE_LV_ANIMATION
            lv_obj_animate(kb, LV_ANIM_FLOAT_BOTTOM | LV_ANIM_IN, 300, 0, NULL);
        #endif
        }
    }
}

/**
 * Called when the close or ok button is pressed on the keyboard
 * @param keyboard pointer to the keyboard
 * @return
 */
static void keyboard_hide_action(lv_obj_t * keyboard)
{
    (void) keyboard;    /*Unused*/

#if USE_LV_ANIMATION
    lv_obj_animate(kb, LV_ANIM_FLOAT_BOTTOM | LV_ANIM_OUT, 300, 0, kb_hide_anim_end);
    kb = NULL;
#else
    lv_obj_del(kb);
    kb = NULL;
#endif
}

static void list_create(lv_obj_t * parent)
{
    lv_page_set_style(parent, LV_PAGE_STYLE_BG, &lv_style_transp_fit);
    lv_page_set_style(parent, LV_PAGE_STYLE_SCRL, &lv_style_transp_fit);

    lv_page_set_scrl_fit(parent, LV_FIT_NONE);
    lv_page_set_scrl_height(parent, lv_obj_get_height(parent));
    lv_page_set_sb_mode(parent, LV_SB_MODE_OFF);

    /*Create styles for the buttons*/
    static lv_style_t style_btn_rel;
    static lv_style_t style_btn_pr;
    lv_style_copy(&style_btn_rel, &lv_style_btn_rel);
    style_btn_rel.body.main_color = lv_color_hex3(0x333);
    style_btn_rel.body.grad_color = LV_COLOR_BLACK;
    style_btn_rel.body.border.color = LV_COLOR_SILVER;
    style_btn_rel.body.border.width = 1;
    style_btn_rel.body.border.opa = LV_OPA_50;
    style_btn_rel.body.radius = 0;

    lv_style_copy(&style_btn_pr, &style_btn_rel);
    style_btn_pr.body.main_color = LV_COLOR_MAKE(0x55, 0x96, 0xd8);
    style_btn_pr.body.grad_color = LV_COLOR_MAKE(0x37, 0x62, 0x90);
    style_btn_pr.text.color = LV_COLOR_MAKE(0xbb, 0xd5, 0xf1);

    lv_obj_t * list = lv_list_create(parent, NULL);
    lv_obj_set_height(list, 2 * lv_obj_get_height(parent) / 3);
    lv_list_set_style(list, LV_LIST_STYLE_BG, &lv_style_transp_tight);
    lv_list_set_style(list, LV_LIST_STYLE_SCRL, &lv_style_transp_tight);
    lv_list_set_style(list, LV_LIST_STYLE_BTN_REL, &style_btn_rel);
    lv_list_set_style(list, LV_LIST_STYLE_BTN_PR, &style_btn_pr);
    lv_obj_align(list, NULL, LV_ALIGN_IN_TOP_MID, 0, LV_DPI / 4);

// TODO: v5.3 - how to do this in 6.0? -> see new demo?
//    lv_list_add(list, SYMBOL_FILE, "New", list_btn_action);
//    lv_list_add(list, SYMBOL_DIRECTORY, "Open", list_btn_action);
//    lv_list_add(list, SYMBOL_TRASH, "Delete", list_btn_action);
//    lv_list_add(list, SYMBOL_EDIT, "Edit", list_btn_action);
//    lv_list_add(list, SYMBOL_SAVE, "Save", list_btn_action);
//    lv_list_add(list, SYMBOL_WIFI, "WiFi", list_btn_action);
//    lv_list_add(list, SYMBOL_GPS, "GPS", list_btn_action);

    lv_obj_t * mbox = lv_mbox_create(parent, NULL);
    lv_mbox_set_text(mbox, "Click a button to copy its text to the Text area ");
    lv_obj_set_width(mbox, LV_HOR_RES - LV_DPI);
// TODO: v5.3 - how to do this in 6.0? -> see new demo?
//    static const char * mbox_btns[] = {"Got it", ""};
//    lv_mbox_add_btns(mbox, mbox_btns, NULL);    /*The default action is close*/
    lv_obj_align(mbox, parent, LV_ALIGN_IN_TOP_MID, 0, LV_DPI / 2);
}

static void kb_hide_anim_end(lv_obj_t * keyboard)
{
    lv_obj_del(keyboard);
}

static void chart_create(lv_obj_t * parent)
{
    lv_page_set_style(parent, LV_PAGE_STYLE_BG, &lv_style_transp_fit);
    lv_page_set_style(parent, LV_PAGE_STYLE_SCRL, &lv_style_transp_fit);

    lv_page_set_scrl_fit(parent, LV_FIT_NONE);
    lv_page_set_scrl_height(parent, lv_obj_get_height(parent));
    lv_page_set_sb_mode(parent, LV_SB_MODE_OFF);

    static lv_style_t style_chart;
    lv_style_copy(&style_chart, &lv_style_pretty);
    style_chart.body.opa = LV_OPA_60;
    style_chart.body.radius = 0;
    style_chart.line.color = LV_COLOR_GRAY;

    chart = lv_chart_create(parent, NULL);
    lv_obj_set_size(chart, 2 * lv_obj_get_width(parent) / 3, lv_obj_get_height(parent) / 2);
    lv_obj_align(chart, NULL,  LV_ALIGN_IN_TOP_MID, 0, LV_DPI / 4);
    lv_chart_set_type(chart, LV_CHART_TYPE_COLUMN);
// TODO: v5.3 - how to do this in 6.0? -> see new demo?
//    lv_chart_set_style(chart, &style_chart);
    lv_chart_set_series_opa(chart, LV_OPA_70);
    lv_chart_series_t * ser1;
    ser1 = lv_chart_add_series(chart, LV_COLOR_RED);
    lv_chart_set_next(chart, ser1, 40);
    lv_chart_set_next(chart, ser1, 30);
    lv_chart_set_next(chart, ser1, 47);
    lv_chart_set_next(chart, ser1, 59);
    lv_chart_set_next(chart, ser1, 59);
    lv_chart_set_next(chart, ser1, 31);
    lv_chart_set_next(chart, ser1, 55);
    lv_chart_set_next(chart, ser1, 70);
    lv_chart_set_next(chart, ser1, 82);
    lv_chart_set_next(chart, ser1, 91);

    /*Create a bar, an indicator and a knob style*/
    static lv_style_t style_bar;
    static lv_style_t style_indic;
    static lv_style_t style_knob;

    lv_style_copy(&style_bar, &lv_style_pretty);
    style_bar.body.main_color =  LV_COLOR_BLACK;
    style_bar.body.grad_color =  LV_COLOR_GRAY;
    style_bar.body.radius = LV_RADIUS_CIRCLE;
    style_bar.body.border.color = LV_COLOR_WHITE;
    style_bar.body.opa = LV_OPA_60;
    style_bar.body.padding.left = 0;
    style_bar.body.padding.right = 0;
    style_bar.body.padding.top = LV_DPI / 10;
    style_bar.body.padding.bottom = LV_DPI / 10;

    lv_style_copy(&style_indic, &lv_style_pretty);
    style_indic.body.grad_color =  LV_COLOR_MAROON;
    style_indic.body.main_color =  LV_COLOR_RED;
    style_indic.body.radius = LV_RADIUS_CIRCLE;
    style_indic.body.shadow.width = LV_DPI / 10;
    style_indic.body.shadow.color = LV_COLOR_RED;
    style_indic.body.padding.left = LV_DPI / 30;
    style_indic.body.padding.right = LV_DPI / 30;
    style_indic.body.padding.top = LV_DPI / 30;
    style_indic.body.padding.bottom = LV_DPI / 30;

    lv_style_copy(&style_knob, &lv_style_pretty);
    style_knob.body.radius = LV_RADIUS_CIRCLE;
    style_knob.body.opa = LV_OPA_70;

    /*Create a second slider*/
    lv_obj_t * slider = lv_slider_create(parent, NULL);
    lv_slider_set_style(slider, LV_SLIDER_STYLE_BG, &style_bar);
    lv_slider_set_style(slider, LV_SLIDER_STYLE_INDIC, &style_indic);
    lv_slider_set_style(slider, LV_SLIDER_STYLE_KNOB, &style_knob);
    lv_obj_set_size(slider, lv_obj_get_width(chart), LV_DPI / 3);
    lv_obj_align(slider, chart, LV_ALIGN_OUT_BOTTOM_MID, 0, (LV_VER_RES - chart->coords.y2 - lv_obj_get_height(slider)) / 2); /*Align to below the chart*/
// TODO: v5.3 - how to do this in 6.0? -> see new demo?
//    lv_slider_set_action(slider, slider_action);
    lv_slider_set_range(slider, 10, 1000);
// TODO: v5.3 - how to do this in 6.0? -> see new demo?
//    lv_slider_set_value(slider, 700);
    slider_action(slider);          /*Simulate a user value set the refresh the chart*/
}


// MARK: === p44mbc

void demo_setButtonCallback(p44BtnCallBack_t aCallBack)
{
  p44BtnCallBack = aCallBack;
}

void demo_setNewText(const char *aText)
{
  lv_label_set_text(dispLabel, aText);
}



static void btn_event_handler(lv_obj_t * btn, lv_event_t event)
{
    if (event==LV_EVENT_RELEASED) {
        uint8_t id = (uint8_t)lv_obj_get_user_data(btn);
        char buf[32];

        if (p44BtnCallBack) {
            // custom
            p44BtnCallBack(id);
            return;
        }
        else {
            snprintf(buf, 32, "Button %d", id);
            lv_label_set_text(dispLabel, buf);
            return;
        }
    }
}


static void p44mbc_create(lv_obj_t * parent)
{
#if !ONLY_DEMO
  lv_page_set_style(parent, LV_PAGE_STYLE_BG, &lv_style_transp_fit);
  lv_page_set_style(parent, LV_PAGE_STYLE_SCRL, &lv_style_transp_fit);

  lv_page_set_scrl_fit(parent, false, false);
  lv_page_set_scrl_height(parent, lv_obj_get_height(parent));
  lv_page_set_sb_mode(parent, LV_SB_MODE_OFF);
#endif

  lv_obj_t *lbl; // temp

  plusButton = lv_btn_create(parent, NULL);
  lv_cont_set_fit2(plusButton, LV_FIT_NONE, LV_FIT_TIGHT); /*Enable resizing horizontally and vertically*/
  lv_obj_set_user_data(plusButton, (void *)1);   /*Set a unique number for the button*/
  lv_obj_set_event_cb(plusButton, btn_event_handler);
  lbl = lv_label_create(plusButton, NULL);
  lv_btn_set_fit2(plusButton, LV_FIT_NONE, LV_FIT_TIGHT);
  lv_label_set_text(lbl, "+ Plus +");
  lv_obj_set_width(plusButton, lv_obj_get_width(parent)-20); // expand to full width
  lv_obj_align(plusButton, NULL, LV_ALIGN_IN_TOP_MID, 0, 10);

  minusButton = lv_btn_create(parent, plusButton); // mostly same
  lv_obj_set_user_data(minusButton, (void *)2);   /*Set a unique number for the button*/
  lbl = lv_label_create(minusButton, NULL);
  lv_btn_set_fit2(plusButton, LV_FIT_NONE, LV_FIT_TIGHT);
  lv_label_set_text(lbl, "- Minus -");
  lv_obj_set_width(minusButton, lv_obj_get_width(parent)-20); // expand to full width
  lv_obj_align(minusButton, NULL, LV_ALIGN_IN_BOTTOM_MID, 0, -10);

  dispLabel = lv_label_create(parent, NULL);
  static lv_style_t dispLabelStyle;
  lv_style_copy(&dispLabelStyle, &lv_style_plain);
  dispLabelStyle.text.font = &lv_font_roboto_28;
  lv_obj_set_style(dispLabel, &dispLabelStyle);
  lv_label_set_long_mode(dispLabel, LV_LABEL_LONG_CROP);
  lv_label_set_recolor(dispLabel, true);
  lv_label_set_align(dispLabel, LV_LABEL_ALIGN_CENTER);
  lv_label_set_text(dispLabel, "-");
  lv_label_set_long_mode(dispLabel, LV_LABEL_LONG_SROLL_CIRC);
  lv_obj_set_width(dispLabel, lv_obj_get_width(parent)); // expand to full width
  lv_obj_set_height(dispLabel, 70);
  lv_obj_align(dispLabel, NULL, LV_ALIGN_CENTER, 0, 0);

#if RGB_SAMPLES

  static lv_style_t styleRed;
  lv_style_copy(&styleRed, &lv_style_plain);
  styleRed.body.main_color = LV_COLOR_RED;
  styleRed.body.grad_color = styleRed.body.main_color;
  styleRed.body.opa = LV_OPA_COVER;

  static lv_style_t styleGreen;
  lv_style_copy(&styleGreen, &styleRed);
  styleGreen.body.main_color = LV_COLOR_GREEN;
  styleGreen.body.grad_color = styleGreen.body.main_color;

  static lv_style_t styleBlue;
  lv_style_copy(&styleBlue, &styleRed);
  styleBlue.body.main_color = LV_COLOR_BLUE;
  styleBlue.body.grad_color = styleBlue.body.main_color;

  lv_obj_t *green = lv_obj_create(parent, NULL);
  lv_obj_set_style(green, &styleGreen);
  lv_obj_set_size(green, 40, 20);
  lv_obj_align(green, dispLabel, LV_ALIGN_OUT_BOTTOM_MID, 0, 5);

  lv_obj_t *red = lv_obj_create(parent, NULL);
  lv_obj_set_style(red, &styleRed);
  lv_obj_set_size(red, 40, 20);
  lv_obj_align(red, green, LV_ALIGN_OUT_LEFT_MID, -5, 0);

  lv_obj_t *blue = lv_obj_create(parent, NULL);
  lv_obj_set_style(blue, &styleBlue);
  lv_obj_set_size(blue, 40, 20);
  lv_obj_align(blue, green, LV_ALIGN_OUT_RIGHT_MID, 5, 0);

#endif

}




/**
 * Called when a new value on the slider on the Chart tab is set
 * @param slider pointer to the slider
 * @return LV_RES_OK because the slider is not deleted in the function
 */
static lv_res_t slider_action(lv_obj_t * slider)
{
    int16_t v = lv_slider_get_value(slider);
    v = 1000 * 100 / v; /*Convert to range modify values linearly*/
    lv_chart_set_range(chart, 0, v);

    return LV_RES_OK;
}

/**
 * Called when a a list button is clicked on the List tab
 * @param btn pointer to a list button
 * @return LV_RES_OK because the button is not deleted in the function
 */
static lv_res_t list_btn_action(lv_obj_t * btn)
{
    lv_ta_add_char(ta, '\n');
    lv_ta_add_text(ta, lv_list_get_btn_text(btn));

    return LV_RES_OK;
}

#if LV_DEMO_SLIDE_SHOW
/**
 * Called periodically (lv_task) to switch to the next tab
 */
static void tab_switcher(void * tv)
{
    static uint8_t tab = 0;

    tab++;
    if(tab >= 3) tab = 0;
    lv_tabview_set_tab_act(tv, tab, true);
}
#endif


#endif  /*USE_LV_DEMO*/

#include "factory_gui.h"
#include "Arduino.h"
#include "lvgl.h"

LV_FONT_DECLARE(font_Alibaba);

static lv_point_t line_points[] = {{-320, 0}, {320, 0}};

static void update_text_subscriber_cb(lv_event_t *e);
static void update_touch_point_subscriber_cb(lv_event_t *e);
static void timer_task(lv_timer_t *t);
static lv_obj_t *dis;


void ui_begin()
{

    dis = lv_tileview_create(lv_scr_act());
    lv_obj_align(dis, LV_ALIGN_TOP_RIGHT, 0, 0);
    lv_obj_set_size(dis, LV_PCT(100), LV_PCT(100));
    lv_obj_remove_style(dis, 0, LV_PART_SCROLLBAR);

    lv_obj_t *tv1 = lv_tileview_add_tile(dis, 0, 0, LV_DIR_VER);
    
    /* page 1 */
    lv_obj_t *main_cout = lv_obj_create(tv1);
    lv_obj_set_size(main_cout, LV_PCT(100), LV_PCT(100));
    lv_obj_clear_flag(main_cout, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_style_border_width(main_cout, 0, 0);
    lv_obj_set_style_bg_color(main_cout, UI_BG_COLOR, 0);

    lv_obj_t *hour_cout = lv_obj_create(main_cout);
    lv_obj_set_size(hour_cout, 140, 140);
    lv_obj_align(hour_cout, LV_ALIGN_CENTER, -85, 0);
    lv_obj_set_style_bg_color(hour_cout, UI_FRAME_COLOR, 0);
    lv_obj_clear_flag(hour_cout, LV_OBJ_FLAG_SCROLLABLE);

    lv_obj_t *min_cout = lv_obj_create(main_cout);
    lv_obj_set_size(min_cout, 140, 140);
    lv_obj_align(min_cout, LV_ALIGN_CENTER, 85, 0);
    lv_obj_set_style_bg_color(min_cout, UI_FRAME_COLOR, 0);
    lv_obj_clear_flag(min_cout, LV_OBJ_FLAG_SCROLLABLE);

    lv_obj_t *seg_text = lv_label_create(main_cout);
    lv_obj_align(seg_text, LV_ALIGN_CENTER, 0, -10);
    lv_obj_set_style_text_font(seg_text, &font_Alibaba, 0);
    lv_label_set_text(seg_text, ":");
    lv_obj_set_style_text_color(seg_text, UI_FONT_COLOR, 0);

    lv_obj_t *hour_text = lv_label_create(hour_cout);
    lv_obj_center(hour_text);
    lv_obj_set_style_text_font(hour_text, &font_Alibaba, 0);
    lv_label_set_text(hour_text, "12");
    lv_obj_set_style_text_color(hour_text, UI_FONT_COLOR, 0);

    lv_obj_add_event_cb(hour_text, update_text_subscriber_cb, LV_EVENT_MSG_RECEIVED, NULL);
    lv_msg_subsribe_obj(MSG_NEW_HOUR, hour_text, (void *)"%02d");

    lv_obj_t *min_text = lv_label_create(min_cout);
    lv_obj_center(min_text);
    lv_obj_set_style_text_font(min_text, &font_Alibaba, 0);
    lv_label_set_text(min_text, "34");
    lv_obj_set_style_text_color(min_text, UI_FONT_COLOR, 0);
    lv_obj_add_event_cb(min_text, update_text_subscriber_cb, LV_EVENT_MSG_RECEIVED, NULL);
    lv_msg_subsribe_obj(MSG_NEW_MIN, min_text, (void *)"%02d");

    static lv_style_t style_line;
    lv_style_init(&style_line);
    lv_style_set_line_width(&style_line, 4);
    lv_style_set_line_color(&style_line, UI_BG_COLOR);
    lv_style_set_line_rounded(&style_line, true);

    lv_obj_t *line;
    line = lv_line_create(main_cout);
    lv_line_set_points(line, line_points, 2);
    lv_obj_add_style(line, &style_line, 0);
    lv_obj_center(line);


    lv_timer_create(timer_task, 500, seg_text);
}

static void timer_task(lv_timer_t *t)
{
    lv_obj_t *seg = (lv_obj_t *)t->user_data;
    static bool j;
    if (j)
        lv_obj_add_flag(seg, LV_OBJ_FLAG_HIDDEN);
    else
        lv_obj_clear_flag(seg, LV_OBJ_FLAG_HIDDEN);
    j = !j;
}

static void update_text_subscriber_cb(lv_event_t *e)
{
    lv_obj_t *label = lv_event_get_target(e);
    lv_msg_t *m = lv_event_get_msg(e);

    const char *fmt = (const char *)lv_msg_get_user_data(m);
    const int32_t *v = (const int32_t *)lv_msg_get_payload(m);

    lv_label_set_text_fmt(label, fmt, *v);
}

static void update_touch_point_subscriber_cb(lv_event_t *e)
{
    lv_obj_t *label = lv_event_get_target(e);
    lv_msg_t *m = lv_event_get_msg(e);

    const char *fmt = (const char *)lv_msg_get_user_data(m);
    const char *t = (const char *)lv_msg_get_payload(m);

    lv_label_set_text_fmt(label, fmt, t);
}
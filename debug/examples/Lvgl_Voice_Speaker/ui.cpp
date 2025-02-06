/*
 * @Description: None
 * @Author: LILYGO_L
 * @Date: 2024-11-28 17:07:50
 * @LastEditTime: 2024-12-11 13:29:49
 * @License: GPL 3.0
 */
#include "ui.h"

void ui_init(lv_ui *ui)
{
    win_home_init(ui);
    lv_scr_load(ui->home);
}

void win_home_init(lv_ui *ui)
{
    // Write codes Home
    ui->home = lv_obj_create(NULL);
    lv_obj_set_size(ui->home, ui->win_width, ui->win_height);
    lv_obj_set_scrollbar_mode(ui->home, LV_SCROLLBAR_MODE_OFF);

    // Write style for Home, Part: LV_PART_MAIN, State: LV_STATE_DEFAULT.
    lv_obj_set_style_bg_opa(ui->home, 0, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui->home_tileview = lv_tileview_create(ui->home);

    ui->home_tile_1 = lv_tileview_add_tile(ui->home_tileview, 0, 0, (lv_dir_t)(LV_DIR_LEFT | LV_DIR_RIGHT));

    lv_obj_t *btn = lv_button_create(ui->home_tile_1);

    lv_obj_t *label = lv_label_create(btn);
    lv_label_set_text(label, "Scroll up or right");
    lv_obj_set_size(btn, LV_SIZE_CONTENT, LV_SIZE_CONTENT);
    lv_obj_center(btn);

    // /* Create a transition animation on width transformation and recolor. */
    // static lv_style_prop_t tr_prop[] = {LV_STYLE_TRANSFORM_WIDTH, LV_STYLE_IMAGE_RECOLOR_OPA, 0};
    // static lv_style_transition_dsc_t tr;
    // lv_style_transition_dsc_init(&tr, tr_prop, lv_anim_path_linear, 200, 0, NULL);

    // static lv_style_t style_def;
    // lv_style_init(&style_def);
    // lv_style_set_text_color(&style_def, lv_color_white());
    // lv_style_set_transition(&style_def, &tr);

    // /* Darken the button when pressed and make it smaller */
    // static lv_style_t style_pr;
    // lv_style_init(&style_pr);
    // lv_style_set_image_recolor_opa(&style_pr, LV_OPA_30);
    // lv_style_set_image_recolor(&style_pr, lv_color_black());
    // lv_style_set_transform_width(&style_pr, -5);  // 缩小5个单位
    // lv_style_set_transform_height(&style_pr, -5); // 缩小5个单位

    /* Create a transition animation on scale transformation and recolor. */
    static lv_style_prop_t tr_prop[] = {LV_STYLE_TRANSFORM_SCALE_X, LV_STYLE_TRANSFORM_SCALE_Y, LV_STYLE_IMAGE_RECOLOR_OPA, 0};
    static lv_style_transition_dsc_t tr;
    lv_style_transition_dsc_init(&tr, tr_prop, lv_anim_path_linear, 100, 0, NULL);

    static lv_style_t style_def;
    lv_style_init(&style_def);
    lv_style_set_text_color(&style_def, lv_color_white());
    lv_style_set_transition(&style_def, &tr);

    /* Darken the button when pressed and make it smaller */
    static lv_style_t style_pr;
    lv_style_init(&style_pr);
    lv_style_set_image_recolor_opa(&style_pr, LV_OPA_30);
    lv_style_set_image_recolor(&style_pr, lv_color_black());
    lv_style_set_transform_scale_x(&style_pr, 190);
    lv_style_set_transform_scale_y(&style_pr, 190);


    ui->home_icon_1 = lv_imagebutton_create(ui->home_tile_1);
    lv_obj_add_flag(ui->home_icon_1, LV_OBJ_FLAG_CHECKABLE);
    lv_imagebutton_set_src(ui->home_icon_1, LV_IMAGEBUTTON_STATE_RELEASED, NULL, &home_icon_60x60_1, NULL);
    lv_imagebutton_set_src(ui->home_icon_1, LV_IMAGEBUTTON_STATE_PRESSED, NULL, &home_icon_60x60_1, NULL);
    lv_obj_add_flag(ui->home_icon_1, LV_OBJ_FLAG_CHECKABLE);
    lv_obj_set_pos(ui->home_icon_1, 15, 25);
    lv_obj_set_size(ui->home_icon_1, 60, 60);
    lv_obj_add_style(ui->home_icon_1, &style_def, 0);
    lv_obj_add_style(ui->home_icon_1, &style_pr, LV_STATE_PRESSED);

    ui->home_tile_2 = lv_tileview_add_tile(ui->home_tileview, 1, 0, (lv_dir_t)(LV_DIR_LEFT | LV_DIR_RIGHT));
    lv_obj_t *list = lv_list_create(ui->home_tile_2);
    lv_obj_set_size(list, LV_PCT(100), LV_PCT(100));

    lv_list_add_button(list, NULL, "One");
    lv_list_add_button(list, NULL, "Two");
    lv_list_add_button(list, NULL, "Three");
    lv_list_add_button(list, NULL, "Four");
    lv_list_add_button(list, NULL, "Five");
    lv_list_add_button(list, NULL, "Six");
    lv_list_add_button(list, NULL, "Seven");
    lv_list_add_button(list, NULL, "Eight");
    lv_list_add_button(list, NULL, "Nine");
    lv_list_add_button(list, NULL, "Ten");

    // Update current screen layout.
    lv_obj_update_layout(ui->home);
}

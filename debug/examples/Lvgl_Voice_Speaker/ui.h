/*
 * @Description: None
 * @Author: LILYGO_L
 * @Date: 2024-11-28 17:07:50
 * @LastEditTime: 2024-12-11 11:04:53
 * @License: GPL 3.0
 */
#pragma once

#include "lvgl.h"
#include "pin_config.h"

LV_IMAGE_DECLARE(home_icon_60x60_1);

typedef struct
{
    const uint32_t win_width = LCD_WIDTH;
    const uint32_t win_height = LCD_HEIGHT;

    lv_obj_t *home;
    lv_obj_t *home_tileview;
    lv_obj_t *home_tile_1;
    lv_obj_t *home_tile_2;

    lv_obj_t *home_icon_1;

} lv_ui;

void ui_init(lv_ui *ui);
void win_home_init(lv_ui *ui);

extern lv_ui system_ui;
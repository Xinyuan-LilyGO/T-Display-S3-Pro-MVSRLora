/*
 * @Description: GFX test
 * @Author: LILYGO_L
 * @Date: 2023-07-25 13:45:02
 * @LastEditTime: 2025-02-06 10:35:29
 * @License: GPL 3.0
 */
#include <Arduino.h>
#include "Arduino_GFX_Library.h"
#include "pin_config.h"

Arduino_DataBus *bus = new Arduino_HWSPI(
    LCD_DC /* DC */, LCD_CS /* CS */, LCD_SCLK /* SCK */, LCD_MOSI /* MOSI */, LCD_MISO /* MISO */);

Arduino_GFX *gfx = new Arduino_ST7796(
    bus, LCD_RST /* RST */, 0 /* rotation */, true /* IPS */,
    LCD_WIDTH /* width */, LCD_HEIGHT /* height */,
    49 /* col offset 1 */, 0 /* row offset 1 */, 0 /* col_offset2 */, 0 /* row_offset2 */);

void setup(void)
{
    Serial.begin(115200);

    ledcAttachPin(LCD_BL, 1);
    ledcSetup(1, 2000, 8);
    ledcWrite(1, 255);

    gfx->begin();
    gfx->fillScreen(WHITE);

    gfx->setCursor(50,50);
    gfx->setTextColor(BLACK);
    gfx->setTextSize(2);
    gfx->setRotation(3);
    gfx->printf("qtyuwyeauyeuiawy");
}

void loop()
{
}
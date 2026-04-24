/*
 * @Description: speaker_screen_brightness_certification
 * @Author: LILYGO_L
 * @Date: 2026-04-24 11:22:13
 * @LastEditTime: 2026-04-24 11:56:11
 * @License: GPL 3.0
 */
#include "Arduino_DriveBus_Library.h"
#include "Arduino_GFX_Library.h"
#include "pin_config.h"

// 44.1 KHz
#define IIS_SAMPLE_RATE 44100  // 采样速率
#define IIS_DATA_BIT 16        // 数据位数

// 定义缓存大小 (为了让 44.1 变成整数，我们取 10 个周期，即 441 个采样点)
const int32_t kSamplesPerCycle = 441;
int16_t g_wave_buffer[kSamplesPerCycle * 2];  // *2 是因为 I2S 是双声道 (L/R)

std::shared_ptr<Arduino_IIS_DriveBus> g_i2s_bus_1 =
    std::make_shared<Arduino_HWIIS>(I2S_NUM_1, MAX98357A_BCLK, MAX98357A_LRCLK,
                                    MAX98357A_DATA);

std::unique_ptr<Arduino_IIS> g_max98357a(new Arduino_Amplifier(g_i2s_bus_1));

Arduino_DataBus* bus =
    new Arduino_HWSPI(LCD_DC /* DC */, LCD_CS /* CS */, LCD_SCLK /* SCK */,
                      LCD_MOSI /* MOSI */, LCD_MISO /* MISO */);

Arduino_GFX* gfx = new Arduino_ST7796(
    bus, LCD_RST /* RST */, 0 /* rotation */, true /* IPS */,
    LCD_WIDTH /* width */, LCD_HEIGHT /* height */, 49 /* col offset 1 */,
    0 /* row offset 1 */, 0 /* col_offset2 */, 0 /* row_offset2 */);

void setup() {
  Serial.begin(115200);

  pinMode(RT9080_EN, OUTPUT);
  digitalWrite(RT9080_EN, HIGH);

  pinMode(MAX98357A_EN, OUTPUT);
  digitalWrite(MAX98357A_EN, HIGH);

  gfx->begin();
  gfx->fillScreen(WHITE);

  ledcAttachPin(LCD_BL, 1);
  ledcSetup(1, 2000, 8);
  ledcWrite(1, 0);

  // 生成 1kHz 正弦波数据表
  float amplitude =
      10000.0;  // 声音增益 (最高 32767，建议设在 10000-20000 避免破音)
  for (int i = 0; i < kSamplesPerCycle; i++) {
    // 正弦波公式: A * sin(2 * pi * f * t)
    int16_t sample =
        (int16_t)(amplitude * sin(2.0 * PI * 1000.0 * i / 44100.0));
    g_wave_buffer[i * 2] = sample;      // 左声道
    g_wave_buffer[i * 2 + 1] = sample;  // 右声道
  }

  while (g_max98357a->begin(I2S_MODE_MASTER, AD_IIS_DATA_OUT,
                            I2S_CHANNEL_FMT_RIGHT_LEFT, IIS_DATA_BIT,
                            IIS_SAMPLE_RATE) == false) {
    Serial.println("g_max98357a init fail");
    delay(2000);
  }
  Serial.println("g_max98357a init success");

  for (size_t i = 0; i < 255; i++) {
    ledcWrite(1, i);
    delay(5);
  }

  gfx->setCursor(50, 50);
  gfx->setTextColor(BLACK);
  gfx->setTextSize(2);
  gfx->printf(
      "The screen brightness is set to 255, and the speaker outputs a 1kHz "
      "audio tone.");
}

void loop() {
  g_max98357a->IIS_Write_Data((uint8_t*)g_wave_buffer, sizeof(g_wave_buffer));
}
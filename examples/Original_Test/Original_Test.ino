/*
 * @Description: Original_Test
 * @Author: LILYGO_L
 * @Date: 2023-09-06 10:58:19
 * @LastEditTime: 2025-02-06 10:41:48
 * @License: GPL 3.0
 */

#include <Arduino.h>
#include "Arduino_GFX_Library.h"
#include "pin_config.h"
#include "Wire.h"
#include <WiFi.h>
#include <HTTPClient.h>
#include "Arduino_DriveBus_Library.h"
#include "Material_16Bit_222x480px.h"
#include "Audio.h"
#include <SPI.h>
#include <SD.h>
#include <FS.h>
#include "RadioLib.h"

#define SOFTWARE_NAME "Original_Test"

#define SOFTWARE_LASTEDITTIME "202412131703"
#define BOARD_VERSION "V1.0"

// 44.1 KHz
#define IIS_SAMPLE_RATE 44100 // 采样速率
#define IIS_DATA_BIT 16       // 数据位数

#define WIFI_SSID "xinyuandianzi"
#define WIFI_PASSWORD "AA15994823428"
// #define WIFI_SSID "LilyGo-AABB"
// #define WIFI_PASSWORD "xinyuandianzi"

#define WIFI_CONNECT_WAIT_MAX (10000)

#define NTP_SERVER1 "pool.ntp.org"
#define NTP_SERVER2 "time.nist.gov"
#define NTP_SERVER3 "asia.pool.ntp.org"
#define GMT_OFFSET_SEC 8 * 3600 // Time zone setting function, written as 8 * 3600 in East Eighth Zone (UTC/GMT+8:00)
#define DAY_LIGHT_OFFSET_SEC 0  // Fill in 3600 for daylight saving time, otherwise fill in 0

#define SD_FILE_NAME "/music.mp3"
#define SD_FILE_NAME_TEMP "/music_temp.mp3"
#define SD_FILE_SIZE 3638546

// set RF switch configuration for Wio WM1110
// Wio WM1110 uses DIO5 and DIO6 for RF switching
// NOTE: other boards may be different!
static const uint32_t rfswitch_dio_pins[] = {
    RADIOLIB_LR11X0_DIO5, RADIOLIB_LR11X0_DIO6,
    RADIOLIB_NC, RADIOLIB_NC, RADIOLIB_NC};

static const Module::RfSwitchMode_t rfswitch_table[] = {
    // mode                  DIO5  DIO6
    {LR11x0::MODE_STBY, {LOW, LOW}},
    {LR11x0::MODE_RX, {HIGH, LOW}},
    {LR11x0::MODE_TX, {HIGH, HIGH}},
    {LR11x0::MODE_TX_HP, {LOW, HIGH}},
    {LR11x0::MODE_TX_HF, {LOW, LOW}},
    {LR11x0::MODE_GNSS, {LOW, LOW}},
    {LR11x0::MODE_WIFI, {LOW, LOW}},
    END_OF_MODE_TABLE,
};

const uint64_t Local_MAC = ESP.getEfuseMac();

// 文件下载链接
// const char *fileDownloadUrl = "https://code.visualstudio.com/docs/?dv=win64user";//vscode
// const char *fileDownloadUrl = "https://dldir1.qq.com/qqfile/qq/PCQQ9.7.17/QQ9.7.17.29225.exe"; // 腾讯CDN加速
// const char *fileDownloadUrl = "https://cd001.www.duba.net/duba/install/packages/ever/kinsthomeui_150_15.exe"; // 金山毒霸
const char *fileDownloadUrl = "https://freetyst.nf.migu.cn/public/product9th/product45/2022/05/0716/2018%E5%B9%B409%E6%9C%8812%E6%97%A510%E7%82%B943%E5%88%86%E7%B4%A7%E6%80%A5%E5%86%85%E5%AE%B9%E5%87%86%E5%85%A5%E5%8D%8E%E7%BA%B3179%E9%A6%96/%E6%A0%87%E6%B8%85%E9%AB%98%E6%B8%85/MP3_128_16_Stero/6005751EPFG164228.mp3?channelid=02&msisdn=d43a7dcc-8498-461b-ba22-3205e9b6aa82&Tim=1728484238063&Key=0442fa065dacda7c";
// const char *fileDownloadUrl = "https://github.com/espressif/arduino-esp32/releases/download/3.0.1/esp32-3.0.1.zip";

struct Lora_Operator
{
    using state = enum {
        UNCONNECTED, // 未连接
        CONNECTED,   // 已连接
        CONNECTING,  // 正在连接
    };

    using mode = enum {
        LORA, // 普通LoRa模式
        FSK,  // 普通FSK模式
    };

    struct
    {
        float value = 868.1;
        bool change_flag = false;
    } frequency;
    struct
    {
        float value = 125.0;
        bool change_flag = false;
    } bandwidth;
    struct
    {
        uint8_t value = 12;
        bool change_flag = false;
    } spreading_factor;
    struct
    {
        uint8_t value = 6;
        bool change_flag = false;
    } coding_rate;
    struct
    {
        uint8_t value = 0xAB;
        bool change_flag = false;
    } sync_word;
    struct
    {
        int8_t value = 22;
        bool change_flag = false;
    } output_power;
    struct
    {
        float value = 140;
        bool change_flag = false;
    } current_limit;
    struct
    {
        int16_t value = 16;
        bool change_flag = false;
    } preamble_length;
    struct
    {
        bool value = false;
        bool change_flag = false;
    } crc;

    struct
    {
        uint64_t mac = 0;
        uint32_t send_data = 0;
        uint32_t receive_data = 0;
        uint8_t connection_flag = state::UNCONNECTED;
        bool send_flag = false;
        uint8_t error_count = 0;
    } device_1;

    uint8_t current_mode = mode::LORA;

    volatile bool transmission_flag = false;
    bool initialization_flag = false;
    bool mode_change_flag = false;

    uint8_t send_package[16] = {'M', 'A', 'C', ':',
                                (uint8_t)(Local_MAC >> 56), (uint8_t)(Local_MAC >> 48),
                                (uint8_t)(Local_MAC >> 40), (uint8_t)(Local_MAC >> 32),
                                (uint8_t)(Local_MAC >> 24), (uint8_t)(Local_MAC >> 16),
                                (uint8_t)(Local_MAC >> 8), (uint8_t)Local_MAC,
                                0, 0, 0, 0};

    float receive_rssi = 0;
    float receive_snr = 0;
};

bool Wifi_Connection_Flag = true;
bool SD_Initialization_Flag = false;
bool SD_File_Download_Check_Flag = false;

char IIS_Read_Buff[100];

size_t CycleTime = 0;
size_t CycleTime_1 = 0;
size_t CycleTime_2 = 0;
size_t CycleTime_3 = 0;
size_t CycleTime_4 = 0;

uint8_t Image_Flag = 0;
int8_t Volume_Value = 0;
uint8_t Music_Start_Playing_Count = 0;
bool Music_Start_Playing_Flag = false;

int8_t Strength_Value = 0;

bool Skip_Current_Test = false;

bool Lora_Value_Change_Mode = 0;

Audio audio(false, 3, I2S_NUM_1);

Lora_Operator Lora_OP;

LR1121 radio = new Module(LR1121_CS, LR1121_INT, LR1121_RST, LR1121_BUSY, SPI);

Arduino_DataBus *bus = new Arduino_HWSPI(
    LCD_DC /* DC */, LCD_CS /* CS */, LCD_SCLK /* SCK */, LCD_MOSI /* MOSI */, LCD_MISO /* MISO */);

Arduino_GFX *gfx = new Arduino_ST7796(
    bus, LCD_RST /* RST */, 0 /* rotation */, true /* IPS */,
    LCD_WIDTH /* width */, LCD_HEIGHT /* height */,
    49 /* col offset 1 */, 0 /* row offset 1 */, 0 /* col_offset2 */, 0 /* row_offset2 */);

std::shared_ptr<Arduino_IIC_DriveBus> IIC_Bus =
    std::make_shared<Arduino_HWIIC>(IIC_SDA, IIC_SCL, &Wire);

std::shared_ptr<Arduino_IIS_DriveBus> IIS_Bus =
    std::make_shared<Arduino_HWIIS>(I2S_NUM_0, -1, MP34DT05TR_LRCLK, MP34DT05TR_DATA);

std::unique_ptr<Arduino_IIS> IIS(new Arduino_MEMS(IIS_Bus));

void Arduino_IIC_Touch_Interrupt(void);
void Arduino_IIC_RTC_Interrupt(void);

std::unique_ptr<Arduino_IIC> CST226SE(new Arduino_CST2xxSE(IIC_Bus, CST226SE_DEVICE_ADDRESS,
                                                           TOUCH_RST, TOUCH_INT, Arduino_IIC_Touch_Interrupt));

std::unique_ptr<Arduino_IIC> SY6970(new Arduino_SY6970(IIC_Bus, SY6970_DEVICE_ADDRESS,
                                                       DRIVEBUS_DEFAULT_VALUE, DRIVEBUS_DEFAULT_VALUE));

std::unique_ptr<Arduino_IIC> PCF85063(new Arduino_PCF85063(IIC_Bus, PCF85063_DEVICE_ADDRESS,
                                                           DRIVEBUS_DEFAULT_VALUE, PCF85063_INT, Arduino_IIC_RTC_Interrupt));

void Arduino_IIC_Touch_Interrupt(void)
{
    CST226SE->IIC_Interrupt_Flag = true;
}
void Arduino_IIC_RTC_Interrupt(void)
{
    PCF85063->IIC_Interrupt_Flag = true;
}
void Lora_Transmission_Interrupt(void)
{
    Lora_OP.transmission_flag = true;
}

void Vibration_Start(void)
{
    ledcWrite(2, 255);
    delay(150);
    ledcWrite(2, 0);
}

void Skip_Test_Loop()
{
    uint8_t fingers_number = CST226SE->IIC_Read_Device_Value(CST226SE->Arduino_IIC_Touch::Value_Information::TOUCH_FINGER_NUMBER);

    if (fingers_number > 0)
    {
        int32_t touch_x = CST226SE->IIC_Read_Device_Value(CST226SE->Arduino_IIC_Touch::Value_Information::TOUCH_COORDINATE_X);
        int32_t touch_y = CST226SE->IIC_Read_Device_Value(CST226SE->Arduino_IIC_Touch::Value_Information::TOUCH_COORDINATE_Y);

        if (touch_x > 40 && touch_x < 140 &&
            touch_y > LCD_HEIGHT - (LCD_HEIGHT / 3) && touch_y < LCD_HEIGHT - (LCD_HEIGHT / 3) + 40)
        {
            Vibration_Start();
            Skip_Current_Test = true;
        }
    }
}

void SD_Test_Loop(void)
{
    Serial.println("Detecting SD card");

    if (SD.begin(SD_CS, SPI) == false) // SD boots
    {
        gfx->fillRect(0, 50, LCD_WIDTH, 100, WHITE);
        gfx->setTextSize(1);
        gfx->setTextColor(BLACK);
        gfx->setCursor(0, 60);

        Serial.println("SD bus initialization failed !");
        gfx->printf("SD bus init failed!\n");
    }
    else
    {
        gfx->fillRect(0, 50, LCD_WIDTH, 100, WHITE);
        gfx->setTextSize(1);
        gfx->setTextColor(BLACK);
        gfx->setCursor(0, 60);

        Serial.println("SD bus initialization successful !");
        gfx->printf("SD bus init successful!\n");

        uint8_t card_type = SD.cardType();
        uint64_t card_size = SD.cardSize() / (1024 * 1024);
        // uint8_t num_sectors = SD.numSectors();
        switch (card_type)
        {
        case CARD_NONE:
            Serial.println("No SD card attached");
            gfx->printf("No SD card attached\n");
            gfx->printf("SD card test failed\n");

            break;
        case CARD_MMC:
            Serial.print("SD Card Type: ");
            Serial.println("MMC");
            Serial.printf("SD Card Size: %lluMB\n", card_size);
            gfx->printf("SD Card Type:MMC\nSD Card Size:%lluMB\n", card_size);
            gfx->printf("SD card test successful\n");

            break;
        case CARD_SD:
            Serial.print("SD Card Type: ");
            Serial.println("SDSC");
            Serial.printf("SD Card Size: %lluMB\n", card_size);
            gfx->printf("SD Card Type:SDSC\nSD Card Size:%lluMB\n", card_size);
            gfx->printf("SD card tested successful\n");

            break;
        case CARD_SDHC:
            Serial.print("SD Card Type: ");
            Serial.println("SDHC");
            Serial.printf("SD Card Size: %lluMB\n", card_size);
            gfx->printf("SD Card Type:SDHC\nSD Card Size:%lluMB\n", card_size);
            gfx->printf("SD card tested successful\n");

            break;
        default:
            Serial.println("UNKNOWN");
            gfx->printf("UNKNOWN");
            gfx->printf("SD card test failed\n");

            break;
        }
    }
    SD.end();
}

void Wifi_STA_Test(void)
{
    String text;
    int wifi_num = 0;

    gfx->fillScreen(WHITE);
    gfx->setCursor(0, 0);
    gfx->setTextSize(1);
    gfx->setTextColor(BLACK);

    Serial.println("\nScanning wifi");
    gfx->printf("\n\n\nScanning wifi\n");
    WiFi.mode(WIFI_STA);
    WiFi.disconnect();
    WiFi.setTxPower(WIFI_POWER_19_5dBm);
    delay(100);

    wifi_num = WiFi.scanNetworks();
    if (wifi_num == 0)
    {
        text = "\nWiFi scan complete !\nNo wifi discovered.\n";
    }
    else
    {
        text = "\nWiFi scan complete !\n";
        text += wifi_num;
        text += " wifi discovered.\n\n";

        for (int i = 0; i < wifi_num; i++)
        {
            text += (i + 1);
            text += ": ";
            text += WiFi.SSID(i);
            text += " (";
            text += WiFi.RSSI(i);
            text += ")";
            text += (WiFi.encryptionType(i) == WIFI_AUTH_OPEN) ? " \n" : "*\n";
            delay(10);
        }
    }

    Serial.println(text);
    gfx->println(text);

    delay(3000);
    text.clear();
    gfx->fillScreen(WHITE);
    gfx->setCursor(0, 50);

    Serial.print("Connecting to ");
    gfx->printf("Connecting to\n");

    Serial.print(WIFI_SSID);
    gfx->printf("%s", WIFI_SSID);
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

    uint32_t last_tick = millis();

    Wifi_Connection_Flag = true;

    while (WiFi.status() != WL_CONNECTED)
    {
        Serial.print(".");
        gfx->printf(".");
        delay(100);

        if (millis() - last_tick > WIFI_CONNECT_WAIT_MAX)
        {
            Wifi_Connection_Flag = false;
            break;
        }
    }

    if (Wifi_Connection_Flag == true)
    {
        Serial.print("\nThe connection was successful ! \nTakes ");
        gfx->printf("\nThe connection was successful ! \nTakes ");
        Serial.print(millis() - last_tick);
        gfx->print(millis() - last_tick);
        Serial.println(" ms\n");
        gfx->printf(" ms\n");

        gfx->setTextColor(PURPLE);
        gfx->printf("\nWifi test passed!");
    }
    else
    {
        gfx->setTextColor(RED);
        gfx->printf("\nWifi test error!\n");
    }
}

void WIFI_HTTP_Download_SD_File(void)
{
    // 初始化HTTP客户端
    HTTPClient http;
    http.begin(fileDownloadUrl);
    // 获取重定向的URL
    const char *headerKeys[] = {"Location"};
    http.collectHeaders(headerKeys, 1);

    // 记录下载开始时间
    size_t startTime = millis();
    // 无用时间
    size_t uselessTime = 0;

    // 发起GET请求
    int httpCode = http.GET();

    while (httpCode == HTTP_CODE_MOVED_PERMANENTLY || httpCode == HTTP_CODE_FOUND)
    {
        String newUrl = http.header("Location");
        Serial.printf("Redirecting to: %s\n", newUrl.c_str());
        http.end(); // 关闭旧的HTTP连接

        // 使用新的URL重新发起GET请求
        http.begin(newUrl);
        httpCode = http.GET();
    }

    if (httpCode == HTTP_CODE_OK)
    {
        // 获取文件大小
        size_t fileSize = http.getSize();
        Serial.printf("Starting file download...\n");
        Serial.printf("file size: %f MB\n", fileSize / 1024.0 / 1024.0);
        gfx->printf("\n\n\nStarting file download...\n");
        gfx->printf("file size: %f MB\n", fileSize / 1024.0 / 1024.0);

        SD.remove(SD_FILE_NAME_TEMP);

        File file;
        // 打开文件，如果没有文件就创建文件
        file = SD.open(SD_FILE_NAME_TEMP, FILE_WRITE);

        if (file != 0)
        {
            // 读取HTTP响应
            WiFiClient *stream = http.getStreamPtr();

            size_t temp_count_s = 0;
            size_t temp_fileSize = fileSize;
            // uint8_t *buf_1 = (uint8_t *)heap_caps_malloc(64 * 1024, MALLOC_CAP_SPIRAM);
            uint8_t buf_1[4096] = {0};
            CycleTime = millis() + 3000; // 开始计时
            while (http.connected() && (temp_fileSize > 0 || temp_fileSize == -1))
            {
                // 获取可用数据的大小
                size_t availableSize = stream->available();
                if (availableSize)
                {
                    // temp_fileSize -= stream->read(buf_1, min(availableSize, (size_t)(64 * 1024)));
                    temp_fileSize -= stream->read(buf_1, min(availableSize, (size_t)(4096)));

                    // file.write(buf_1, min(availableSize, (size_t)(64 * 1024)));
                    file.write(buf_1, min(availableSize, (size_t)(4096)));

                    if (millis() > CycleTime)
                    {
                        size_t temp_time_1 = millis();
                        temp_count_s += 3;
                        Serial.printf("Download speed: %f KB/s\n", ((fileSize - temp_fileSize) / 1024.0) / temp_count_s);
                        Serial.printf("Remaining file size: %f MB\n\n", temp_fileSize / 1024.0 / 1024.0);

                        gfx->fillRect(0, 60, 200, 68, WHITE);
                        gfx->setCursor(0, 60);
                        gfx->printf("Speed: %f KB/s\n", ((fileSize - temp_fileSize) / 1024.0) / temp_count_s);
                        gfx->printf("Size: %f MB\n\n", temp_fileSize / 1024.0 / 1024.0);

                        delay(1);

                        if (temp_count_s > 50)
                        {
                            break;
                        }

                        CycleTime = millis() + 3000;
                        size_t temp_time_2 = millis();

                        uselessTime = uselessTime + (temp_time_2 - temp_time_1);
                    }
                }
                // delay(1);
            }

            // 关闭HTTP客户端
            http.end();

            // 记录下载结束时间并计算总花费时间
            size_t endTime = millis();

            file.close();
            file = SD.open(SD_FILE_NAME_TEMP);
            size_t temp_file_wifi_download_size = 0;
            if (file != 0)
            {
                temp_file_wifi_download_size = file.size();
            }
            file.close();
            SD.end();
            Serial.println("Writing SD card data completed");

            Serial.printf("Download completed!\n");
            Serial.printf("Total download time: %f s\n", (endTime - startTime - uselessTime) / 1000.0);
            Serial.printf("Average download speed: %f KB/s\n", (fileSize / 1024.0) / ((endTime - startTime - uselessTime) / 1000.0));

            gfx->printf("Completed!\n");
            gfx->printf("Time: %f s\n", (endTime - startTime - uselessTime) / 1000.0);
            gfx->printf("Speed: %f KB/s\n", (fileSize / 1024.0) / ((endTime - startTime - uselessTime) / 1000.0));

            if (temp_file_wifi_download_size == SD_FILE_SIZE)
            {
                SD_File_Download_Check_Flag = true;

                Serial.printf("\nFile verification successful\n");
                gfx->printf("\nFile verification successful\n");
            }
            else
            {
                SD_File_Download_Check_Flag = false;
                Serial.printf("\nFile verification failed\n");
                gfx->printf("\nFile verification failed\n");
            }

            Serial.printf("Original file size: %d Bytes\n", SD_FILE_SIZE);
            Serial.printf("Wifi download size: %d Bytes\n", temp_file_wifi_download_size);
            gfx->printf("Original file size: %d Bytes\n", SD_FILE_SIZE);
            gfx->printf("Wifi download size: %d Bytes\n", temp_file_wifi_download_size);
        }
        else
        {
            Serial.printf("Fail to open file\n");
            gfx->printf("Fail to open file\n\n");
        }
    }
    else
    {
        Serial.printf("Failed to download\n");
        Serial.printf("Error httpCode: %d \n", httpCode);

        gfx->printf("Failed to download\n");
        gfx->printf("Error httpCode: %d \n\n", httpCode);
    }
}

void PrintLocalTime(void)
{
    struct tm timeinfo;
    uint32_t temp_buf = 0;
    bool temp_buf_2 = true;

    while (getLocalTime(&timeinfo) == false)
    {
        Serial.printf("\n.");
        gfx->printf("\n.");
        configTime(GMT_OFFSET_SEC, DAY_LIGHT_OFFSET_SEC, NTP_SERVER1, NTP_SERVER2, NTP_SERVER3);
        temp_buf++;
        if (temp_buf > 5)
        {
            temp_buf_2 = false;
            break;
        }
    }
    if (temp_buf_2 == false)
    {
        Serial.println("Failed to obtain time");
        gfx->setCursor(80, 200);
        gfx->setTextColor(RED);
        gfx->print("Failed to obtain time!");
        return;
    }

    Serial.println("Get time success");
    Serial.println(&timeinfo, "%A,%B %d %Y %H:%M:%S"); // Format Output
    gfx->setCursor(80, 200);
    gfx->setTextColor(PURPLE);
    gfx->print(&timeinfo, " %Y");
    gfx->setCursor(80, 215);
    gfx->print(&timeinfo, "%B %d");
    gfx->setCursor(80, 230);
    gfx->print(&timeinfo, "%H:%M:%S");
}

void GFX_Print_Play_Progress(void)
{
    gfx->setTextSize(2);
    gfx->setCursor(35, 270);

    gfx->fillRect(35, 270, 182, 50, WHITE);
    gfx->printf("%d:%d", audio.getAudioCurrentTime() / 60, audio.getAudioCurrentTime() % 60);

    gfx->setCursor(100, 270);
    gfx->printf("->");

    gfx->setCursor(145, 270);
    gfx->printf("%d:%d", audio.getAudioFileDuration() / 60, audio.getAudioFileDuration() % 60);
}

void GFX_Print_RTC_Info_Loop()
{
    gfx->fillRect(0, 50, LCD_WIDTH, LCD_HEIGHT / 2, WHITE);
    gfx->setTextSize(1);
    gfx->setTextColor(BLACK);

    gfx->setCursor(20, 65);
    gfx->printf("PCF85063  Weekday: %s\n",
                PCF85063->IIC_Read_Device_State(PCF85063->Arduino_IIC_RTC::Status_Information::RTC_WEEKDAYS_DATA).c_str());
    gfx->setCursor(20, 80);
    gfx->printf("PCF85063  Year: %d\n",
                (int32_t)PCF85063->IIC_Read_Device_Value(PCF85063->Arduino_IIC_RTC::Value_Information::RTC_YEARS_DATA));
    gfx->setCursor(20, 95);
    gfx->printf("PCF85063 Date: %d.%d\n",
                (int32_t)PCF85063->IIC_Read_Device_Value(PCF85063->Arduino_IIC_RTC::Value_Information::RTC_MONTHS_DATA),
                (int32_t)PCF85063->IIC_Read_Device_Value(PCF85063->Arduino_IIC_RTC::Value_Information::RTC_DAYS_DATA));
    gfx->setCursor(20, 110);
    gfx->printf("PCF85063 Time: %d:%d:%d\n",
                (int32_t)PCF85063->IIC_Read_Device_Value(PCF85063->Arduino_IIC_RTC::Value_Information::RTC_HOURS_DATA),
                (int32_t)PCF85063->IIC_Read_Device_Value(PCF85063->Arduino_IIC_RTC::Value_Information::RTC_MINUTES_DATA),
                (int32_t)PCF85063->IIC_Read_Device_Value(PCF85063->Arduino_IIC_RTC::Value_Information::RTC_SECONDS_DATA));
}

void GFX_Print_RTC_Switch_Info()
{
    gfx->fillRect(LCD_WIDTH / 4, LCD_HEIGHT - (LCD_HEIGHT / 3) - 30, LCD_WIDTH / 2, 50, PALERED);
    gfx->drawRect(LCD_WIDTH / 4, LCD_HEIGHT - (LCD_HEIGHT / 3) - 30, LCD_WIDTH / 2, 50, RED);
    gfx->setTextSize(1);
    gfx->setTextColor(WHITE);

    gfx->setCursor(LCD_WIDTH / 4 + 30, LCD_HEIGHT - (LCD_HEIGHT / 3) - 40 + 30);
    gfx->printf("RTC Reset");
}

void GFX_Print_Touch_Info_Loop()
{
    int32_t touch_x_1 = CST226SE->IIC_Read_Device_Value(CST226SE->Arduino_IIC_Touch::Value_Information::TOUCH1_COORDINATE_X);
    int32_t touch_y_1 = CST226SE->IIC_Read_Device_Value(CST226SE->Arduino_IIC_Touch::Value_Information::TOUCH1_COORDINATE_Y);
    int32_t touch_x_2 = CST226SE->IIC_Read_Device_Value(CST226SE->Arduino_IIC_Touch::Value_Information::TOUCH2_COORDINATE_X);
    int32_t touch_y_2 = CST226SE->IIC_Read_Device_Value(CST226SE->Arduino_IIC_Touch::Value_Information::TOUCH2_COORDINATE_Y);
    int32_t touch_x_3 = CST226SE->IIC_Read_Device_Value(CST226SE->Arduino_IIC_Touch::Value_Information::TOUCH3_COORDINATE_X);
    int32_t touch_y_3 = CST226SE->IIC_Read_Device_Value(CST226SE->Arduino_IIC_Touch::Value_Information::TOUCH3_COORDINATE_Y);
    int32_t touch_x_4 = CST226SE->IIC_Read_Device_Value(CST226SE->Arduino_IIC_Touch::Value_Information::TOUCH4_COORDINATE_X);
    int32_t touch_y_4 = CST226SE->IIC_Read_Device_Value(CST226SE->Arduino_IIC_Touch::Value_Information::TOUCH4_COORDINATE_Y);
    int32_t touch_x_5 = CST226SE->IIC_Read_Device_Value(CST226SE->Arduino_IIC_Touch::Value_Information::TOUCH5_COORDINATE_X);
    int32_t touch_y_5 = CST226SE->IIC_Read_Device_Value(CST226SE->Arduino_IIC_Touch::Value_Information::TOUCH5_COORDINATE_Y);
    uint8_t fingers_number = CST226SE->IIC_Read_Device_Value(CST226SE->Arduino_IIC_Touch::Value_Information::TOUCH_FINGER_NUMBER);

    gfx->fillRect(0, 40, LCD_WIDTH, LCD_HEIGHT / 2, WHITE);
    gfx->setTextSize(1);
    gfx->setTextColor(BLACK);

    gfx->setCursor(20, 50);
    gfx->printf("ID: %#X ", (int32_t)CST226SE->IIC_Device_ID());

    gfx->setCursor(20, 65);
    gfx->printf("Fingers Number:%d ", fingers_number);

    gfx->setCursor(20, 80);
    gfx->printf("Touch X1:%d Y1:%d ", touch_x_1, touch_y_1);
    gfx->setCursor(20, 95);
    gfx->printf("Touch X2:%d Y2:%d ", touch_x_2, touch_y_2);
    gfx->setCursor(20, 110);
    gfx->printf("Touch X3:%d Y3:%d ", touch_x_3, touch_y_3);
    gfx->setCursor(20, 125);
    gfx->printf("Touch X4:%d Y4:%d ", touch_x_4, touch_y_4);
    gfx->setCursor(20, 140);
    gfx->printf("Touch X5:%d Y5:%d ", touch_x_5, touch_y_5);
}

void GFX_Print_Voice_Speaker_Info_Loop(int16_t left_channel, int16_t right_channel)
{
    gfx->fillRect(30, 30, 130, 30, WHITE);
    gfx->setTextSize(1);
    gfx->setTextColor(BLACK);

    gfx->setCursor(30, 30);
    gfx->printf("Voice Data:");

    gfx->setCursor(30, 40);
    gfx->printf("Left Channel:%d ", left_channel);

    gfx->setCursor(30, 50);
    gfx->printf("Right Channel:%d", right_channel);
}

void GFX_Print_Time_Info_Loop()
{
    gfx->fillRoundRect(35, 35, 152, 95, 10, WHITE);

    if (Wifi_Connection_Flag == true)
    {
        struct tm timeinfo;
        if (!getLocalTime(&timeinfo, 1000))
        {
            Serial.println("Failed to obtain time");
            gfx->setCursor(50, 45);
            gfx->setTextColor(RED);
            gfx->setTextSize(1);
            gfx->print("Time error");
            return;
        }
        Serial.println("Get time success");
        Serial.println(&timeinfo, "%A,%B %d %Y %H:%M:%S"); // Format Output
        gfx->setCursor(50, 45);
        gfx->setTextColor(ORANGE);
        gfx->setTextSize(1);
        gfx->print(&timeinfo, " %Y");
        gfx->setCursor(50, 60);
        gfx->print(&timeinfo, "%B %d");
        gfx->setCursor(50, 75);
        gfx->print(&timeinfo, "%H:%M:%S");
    }
    else
    {
        gfx->setCursor(50, 45);
        gfx->setTextSize(2);
        gfx->setTextColor(RED);
        gfx->print("Network error");
    }

    gfx->setCursor(50, 90);
    gfx->printf("SYS Time:%d", (uint32_t)millis() / 1000);
}

void GFX_Print_1()
{
    gfx->fillRect(20, LCD_HEIGHT - (LCD_HEIGHT / 4), 80, 40, ORANGE);
    gfx->drawRect(20, LCD_HEIGHT - (LCD_HEIGHT / 4), 80, 40, PURPLE);
    gfx->fillRect(120, LCD_HEIGHT - (LCD_HEIGHT / 4), 80, 40, PURPLE);
    gfx->drawRect(120, LCD_HEIGHT - (LCD_HEIGHT / 4), 80, 40, ORANGE);
    gfx->setTextSize(1);
    gfx->setTextColor(WHITE);
    gfx->setCursor(35, LCD_HEIGHT - (LCD_HEIGHT / 4) + 15);
    gfx->printf("Try Again");
    gfx->setCursor(135, LCD_HEIGHT - (LCD_HEIGHT / 4) + 15);
    gfx->printf("Next Test");
}

void GFX_Print_2()
{
    gfx->fillRect(40, LCD_HEIGHT - (LCD_HEIGHT / 3), 140, 40, RED);
    gfx->drawRect(40, LCD_HEIGHT - (LCD_HEIGHT / 3), 140, 40, CYAN);

    gfx->setTextSize(1);
    gfx->setTextColor(WHITE);
    gfx->setCursor(60, LCD_HEIGHT - (LCD_HEIGHT / 3) + 15);
    gfx->printf("Skip Current Test");
}

void GFX_Print_TEST(String s)
{
    Skip_Current_Test = false;

    gfx->fillScreen(WHITE);
    gfx->setCursor(LCD_WIDTH / 4 + 20, LCD_HEIGHT / 4);
    gfx->setTextSize(3);
    gfx->setTextColor(PALERED);
    gfx->printf("TEST");

    gfx->setCursor(20, LCD_HEIGHT / 4 + 40);
    gfx->setTextSize(2);
    gfx->setTextColor(BLACK);
    gfx->print(s);

    GFX_Print_2();

    gfx->setCursor(LCD_WIDTH / 2 - 17, LCD_HEIGHT / 2);
    gfx->setTextSize(4);
    gfx->setTextColor(RED);
    gfx->printf("3");
    delay(300);
    gfx->fillRect(LCD_WIDTH / 2 - 17, LCD_HEIGHT / 2, LCD_WIDTH / 2 + 20, 20, WHITE);
    gfx->setCursor(LCD_WIDTH / 2 - 17, LCD_HEIGHT / 2);
    gfx->printf("2");
    for (int i = 0; i < 100; i++)
    {
        Skip_Test_Loop();
        delay(10);

        if (Skip_Current_Test == true)
        {
            break;
        }
    }
    gfx->fillRect(LCD_WIDTH / 2 - 17, LCD_HEIGHT / 2, LCD_WIDTH / 2 + 20, 20, WHITE);
    gfx->setCursor(LCD_WIDTH / 2 - 17, LCD_HEIGHT / 2);
    gfx->printf("1");
    for (int i = 0; i < 100; i++)
    {
        Skip_Test_Loop();
        delay(10);

        if (Skip_Current_Test == true)
        {
            break;
        }
    }
}

void GFX_Print_FINISH()
{
    gfx->setCursor(LCD_WIDTH / 4, LCD_HEIGHT / 4);
    gfx->setTextSize(3);
    gfx->setTextColor(ORANGE);
    gfx->printf("FINISH");
}

void GFX_Print_START()
{
    gfx->setCursor(150, 220);
    gfx->setTextSize(6);
    gfx->setTextColor(RED);
    gfx->printf("START");
}

void GFX_Print_Volume()
{
    gfx->setCursor(LCD_WIDTH / 4 + 5, LCD_WIDTH / 4 + 40);
    gfx->setTextSize(3);
    gfx->setTextColor(ORANGE);
    gfx->printf("Volume");
}

void GFX_Print_Music_Volume_Value()
{
    gfx->fillRect(30, LCD_HEIGHT / 3, 50, 50, PALERED);
    gfx->drawRect(30, LCD_HEIGHT / 3, 50, 50, PURPLE);
    gfx->fillRect(140, LCD_HEIGHT / 3, 50, 50, PALERED);
    gfx->drawRect(140, LCD_HEIGHT / 3, 50, 50, PURPLE);
    gfx->setTextSize(2);
    gfx->setTextColor(WHITE);
    gfx->setCursor(50, (LCD_HEIGHT / 3) + 17);
    gfx->printf("-");
    gfx->setCursor(160, (LCD_HEIGHT / 3) + 17);
    gfx->printf("+");

    gfx->fillRect(LCD_WIDTH / 2 - 13, LCD_HEIGHT / 3 + 13, 35, 35, WHITE);
    gfx->setCursor(LCD_WIDTH / 2 - 13, LCD_HEIGHT / 3 + 13);
    gfx->setTextSize(3);
    gfx->setTextColor(RED);
    gfx->print(audio.getVolume());
}

void GFX_Print_Strength()
{
    gfx->setCursor(LCD_WIDTH / 4 - 15, LCD_WIDTH / 4 + 40);
    gfx->setTextSize(3);
    gfx->setTextColor(ORANGE);
    gfx->printf("Strength");
}

void GFX_Print_Motor_Strength_Value()
{
    gfx->fillRect(30, LCD_HEIGHT / 3, 50, 50, PALERED);
    gfx->drawRect(30, LCD_HEIGHT / 3, 50, 50, PURPLE);
    gfx->fillRect(140, LCD_HEIGHT / 3, 50, 50, PALERED);
    gfx->drawRect(140, LCD_HEIGHT / 3, 50, 50, PURPLE);
    gfx->setTextSize(2);
    gfx->setTextColor(WHITE);
    gfx->setCursor(50, (LCD_HEIGHT / 3) + 17);
    gfx->printf("-");
    gfx->setCursor(160, (LCD_HEIGHT / 3) + 17);
    gfx->printf("+");

    gfx->fillRect(LCD_WIDTH / 2 - 13, LCD_HEIGHT / 3 + 13, 35, 35, WHITE);
    gfx->setCursor(LCD_WIDTH / 2 - 13, LCD_HEIGHT / 3 + 13);
    gfx->setTextSize(3);
    gfx->setTextColor(RED);
    gfx->print(Strength_Value);
}

void GFX_Print_Play_Failed()
{
    gfx->setCursor(20, 30);
    gfx->setTextSize(1);
    gfx->setTextColor(RED);
    gfx->printf("Play failed\n");
}

void GFX_Print_LR1121_Operation_Button()
{
    gfx->fillRect(20, LCD_HEIGHT - (LCD_HEIGHT / 3) - 30, 80, 50, PALERED);
    gfx->drawRect(20, LCD_HEIGHT - (LCD_HEIGHT / 3) - 30, 80, 50, RED);
    gfx->fillRect(120, LCD_HEIGHT - (LCD_HEIGHT / 3) - 30, 80, 50, BLUE);
    gfx->drawRect(120, LCD_HEIGHT - (LCD_HEIGHT / 3) - 30, 80, 50, GREEN);
    gfx->setTextSize(1);
    gfx->setTextColor(WHITE);

    gfx->setCursor(35, LCD_HEIGHT - (LCD_HEIGHT / 3) - 40 + 30);
    gfx->printf("Reconnect");
    gfx->setCursor(128, LCD_HEIGHT - (LCD_HEIGHT / 3) - 40 + 30);
    gfx->printf("Switch Mode");
}

bool Lora_Configuration_Default_Value_Parameters(String *assertion)
{
    if (radio.setFrequency(Lora_OP.frequency.value) != RADIOLIB_ERR_NONE)
    {
        *assertion = "Failed to set frequency value";
        return false;
    }
    if (radio.setBandwidth(Lora_OP.bandwidth.value) != RADIOLIB_ERR_NONE)
    {
        *assertion = "Failed to set bandwidth value";
        return false;
    }
    if (radio.setOutputPower(Lora_OP.output_power.value) != RADIOLIB_ERR_NONE)
    {
        *assertion = "Failed to set output_power value";
        return false;
    }
    // if (radio.setCurrentLimit(Lora_OP.current_limit.value) != RADIOLIB_ERR_NONE)
    // {
    //     *assertion = "Failed to set current_limit value";
    //     return false;
    // }
    if (radio.setPreambleLength(Lora_OP.preamble_length.value) != RADIOLIB_ERR_NONE)
    {
        *assertion = "Failed to set preamble_length value";
        return false;
    }
    if (radio.setCRC(Lora_OP.crc.value) != RADIOLIB_ERR_NONE)
    {
        *assertion = "Failed to set crc value";
        return false;
    }
    if (Lora_OP.current_mode == Lora_OP.mode::LORA)
    {
        if (radio.setSpreadingFactor(Lora_OP.spreading_factor.value) != RADIOLIB_ERR_NONE)
        {
            *assertion = "Failed to set spreading_factor value";
            return false;
        }
        if (radio.setCodingRate(Lora_OP.coding_rate.value) != RADIOLIB_ERR_NONE)
        {
            *assertion = "Failed to set coding_rate value";
            return false;
        }
        if (radio.setSyncWord(Lora_OP.sync_word.value) != RADIOLIB_ERR_NONE)
        {
            *assertion = "Failed to set sync_word value";
            return false;
        }
    }
    else
    {
    }
    return true;
}

bool GFX_Print_LR1121_Info(void)
{
    gfx->fillScreen(WHITE);
    gfx->setTextSize(2);
    gfx->setTextColor(PURPLE);
    gfx->setCursor(40, 30);
    gfx->printf("LR1121 Info");

    gfx->setTextSize(1);
    gfx->setTextColor(BLACK);
    gfx->setCursor(20, 110);
    gfx->printf("[Local MAC]: %012llX", Local_MAC);

    gfx->setTextColor(ORANGE);
    gfx->setCursor(17, 160);
    gfx->printf("<----------Send Info----------> ");

    gfx->setTextColor(ORANGE);
    gfx->setCursor(15, 190);
    gfx->printf("<---------Receive Info---------> ");

    SPI.begin(LR1121_SCLK, LR1121_MISO, LR1121_MOSI);

    // set RF switch control configuration
    // this has to be done prior to calling begin()
    radio.setRfSwitchTable(rfswitch_dio_pins, rfswitch_table);

    // initialize LR1121 with default settings
    Serial.println("[LR1121] Initializing ... ");

    int16_t state = radio.begin();
    if (state == RADIOLIB_ERR_NONE)
    {
        Serial.println("success!");
    }
    else
    {
        Serial.print("failed, code ");
        Serial.println(state);
        delay(500);
    }

    if (state == RADIOLIB_ERR_NONE)
    {
        String temp_str;
        if (Lora_Configuration_Default_Value_Parameters(&temp_str) == false)
        {
            Serial.printf("LR1121 Failed to set default parameters\n");
            Serial.printf("LR1121 assertion: %s\n", temp_str.c_str());
            Lora_OP.initialization_flag = false;
            return false;
        }
        radio.setPacketReceivedAction(Lora_Transmission_Interrupt);
        if (radio.startReceive() != RADIOLIB_ERR_NONE)
        {
            Serial.printf("LR1121 Failed to start receive\n");
            Lora_OP.initialization_flag = false;
            return false;
        }
    }
    else
    {
        Serial.printf("LR1121 initialization failed\n");
        Serial.printf("Error code: %d\n", state);
        Lora_OP.initialization_flag = false;
        return false;
    }

    Serial.printf("LR1121 initialization successful\n");
    Lora_OP.initialization_flag = true;

    return true;
}

void GFX_Print_LR1121_Info_Loop()
{
    if (Lora_OP.initialization_flag == true)
    {
        if (millis() > CycleTime)
        {
            gfx->fillRect(0, 60, LCD_WIDTH, 50, WHITE);
            gfx->setTextSize(1);
            gfx->setTextColor(DARKGREEN);
            gfx->setCursor(20, 60);
            gfx->printf("[Status]: Init successful");
            gfx->setTextColor(BLACK);
            gfx->setCursor(20, 70);
            if (Lora_OP.current_mode == Lora_OP.mode::LORA)
            {
                gfx->printf("[Mode]: LoRa");
            }
            else
            {
                gfx->printf("[Mode]: FSK");
            }
            gfx->setTextColor(BLACK);
            gfx->setCursor(20, 80);
            gfx->printf("[Frequency]: %.1f MHz", Lora_OP.frequency.value);

            gfx->setCursor(20, 90);
            gfx->printf("[Bandwidth]: %.1f KHz", Lora_OP.bandwidth.value);

            gfx->setCursor(20, 100);
            gfx->printf("[Output Power]: %d dBm", Lora_OP.output_power.value);

            gfx->fillRect(0, 130, LCD_WIDTH, 30, WHITE);
            gfx->setCursor(20, 130);
            if (Lora_OP.device_1.connection_flag == Lora_OP.state::CONNECTED)
            {
                gfx->setTextColor(DARKGREEN);
                gfx->printf("[Connect]: Connected");
                gfx->setCursor(20, 140);
                gfx->printf("[Connecting MAC]: %012llX", Lora_OP.device_1.mac);

                gfx->fillRect(0, 170, LCD_WIDTH, 20, WHITE);
                gfx->setTextColor(BLACK);
                gfx->setCursor(20, 170);
                gfx->printf("[Send Data]: %u", Lora_OP.device_1.send_data);

                gfx->fillRect(0, 200, LCD_WIDTH, 40, WHITE);
                gfx->setTextColor(BLACK);
                gfx->setCursor(20, 200);
                gfx->printf("[Receive Data]: %u", Lora_OP.device_1.receive_data);
                gfx->setCursor(20, 210);
                gfx->printf("[Receive RSSI]: %.1f dBm", Lora_OP.receive_rssi);
                gfx->setCursor(20, 220);
                gfx->printf("[Receive SNR]: %.1f dB", Lora_OP.receive_snr);
            }
            else if (Lora_OP.device_1.connection_flag == Lora_OP.state::CONNECTING)
            {
                gfx->setTextColor(BLUE);
                gfx->printf("[Connect]: Connecting");

                gfx->fillRect(0, 170, LCD_WIDTH, 20, WHITE);
                gfx->setTextColor(BLACK);
                gfx->setCursor(20, 170);
                gfx->printf("[Send Data]: %u", Lora_OP.device_1.send_data);

                gfx->fillRect(0, 200, LCD_WIDTH, 40, WHITE);
                gfx->setTextColor(BLACK);
                gfx->setCursor(20, 200);
                gfx->printf("[Receive Data]: null");
            }
            else if (Lora_OP.device_1.connection_flag == Lora_OP.state::UNCONNECTED)
            {
                gfx->setTextColor(RED);
                gfx->printf("[Connect]: Unconnected");

                gfx->fillRect(0, 170, LCD_WIDTH, 20, WHITE);
                gfx->setTextColor(BLACK);
                gfx->setCursor(20, 170);
                gfx->printf("[Send Data]: null");

                gfx->fillRect(0, 200, LCD_WIDTH, 40, WHITE);
                gfx->setTextColor(BLACK);
                gfx->setCursor(20, 200);
                gfx->printf("[Receive Data]: null");
            }
            CycleTime = millis() + 500;
        }

        // if (Lora_OP.device_1.connection_flag == Lora_OP.state::CONNECTED)
        // {
        if (Lora_OP.device_1.send_flag == true)
        {
            if (millis() > CycleTime_2)
            {
                Lora_OP.device_1.send_flag = false;

                Lora_OP.send_package[12] = (uint8_t)(Lora_OP.device_1.send_data >> 24);
                Lora_OP.send_package[13] = (uint8_t)(Lora_OP.device_1.send_data >> 16);
                Lora_OP.send_package[14] = (uint8_t)(Lora_OP.device_1.send_data >> 8);
                Lora_OP.send_package[15] = (uint8_t)Lora_OP.device_1.send_data;

                // send another one
                Serial.println("[LR1121] Sending another packet ... ");

                radio.transmit(Lora_OP.send_package, 16);
                radio.startReceive();
            }
        }
        // }

        if (Lora_OP.transmission_flag == true)
        {
            Lora_OP.transmission_flag = false;

            uint8_t receive_package[16] = {0};
            if (radio.readData(receive_package, 16) == RADIOLIB_ERR_NONE)
            {

                if ((receive_package[0] == 'M') &&
                    (receive_package[1] == 'A') &&
                    (receive_package[2] == 'C') &&
                    (receive_package[3] == ':'))
                {
                    // uint64_t temp_mac =
                    //     ((uint64_t)receive_package[4] << 56) |
                    //     ((uint64_t)receive_package[5] << 48) |
                    //     ((uint64_t)receive_package[6] << 40) |
                    //     ((uint64_t)receive_package[7] << 32) |
                    //     ((uint64_t)receive_package[8] << 24) |
                    //     ((uint64_t)receive_package[9] << 16) |
                    //     ((uint64_t)receive_package[10] << 8) |
                    //     (uint64_t)receive_package[11];

                    uint64_t temp_mac = 0;
                    for (size_t i = 0; i < 8; ++i)
                    {
                        temp_mac |= (static_cast<uint64_t>(receive_package[4 + i]) << (56 - i * 8));
                    }

                    if (temp_mac != Local_MAC)
                    {
                        Lora_OP.device_1.mac = temp_mac;
                        Lora_OP.device_1.receive_data =
                            ((uint32_t)receive_package[12] << 24) |
                            ((uint32_t)receive_package[13] << 16) |
                            ((uint32_t)receive_package[14] << 8) |
                            (uint32_t)receive_package[15];

                        // packet was successfully received
                        Serial.printf("[LR1121] Received packet\n");

                        // print data of the packet
                        for (int i = 0; i < 16; i++)
                        {
                            Serial.printf("[LR1121] Data[%d]: %#X\n", i, receive_package[i]);
                        }

                        // print RSSI (Received Signal Strength Indicator)
                        Lora_OP.receive_rssi = radio.getRSSI();
                        Serial.printf("[LR1121] RSSI: %.1f dBm", Lora_OP.receive_rssi);

                        // print SNR (Signal-to-Noise Ratio)
                        Lora_OP.receive_snr = radio.getSNR();
                        Serial.printf("[LR1121] SNR: %.1f dB", Lora_OP.receive_snr);

                        Lora_OP.device_1.send_data = Lora_OP.device_1.receive_data + 1;

                        Lora_OP.device_1.send_flag = true;
                        Lora_OP.device_1.connection_flag = Lora_OP.state::CONNECTED;
                        // 清除错误计数看门狗
                        Lora_OP.device_1.error_count = 0;
                        CycleTime_2 = millis() + 500;
                    }
                }
            }
        }
        if (millis() > CycleTime_3)
        {
            Lora_OP.device_1.error_count++;
            if (Lora_OP.device_1.error_count > 10) // 10秒超时
            {
                Lora_OP.device_1.error_count = 11;
                Lora_OP.device_1.send_data = 0;
                Lora_OP.device_1.connection_flag = Lora_OP.state::UNCONNECTED;
            }
            CycleTime_3 = millis() + 1000;
        }
    }
    else
    {
        if (millis() > CycleTime)
        {
            gfx->fillRect(0, 60, LCD_WIDTH, 50, WHITE);
            gfx->setTextSize(1);
            gfx->setTextColor(RED);
            gfx->setCursor(20, 60);
            gfx->printf("[Status]: Init failed");
            CycleTime = millis() + 1000;
        }
    }
}

void Original_Test_1()
{
    gfx->fillScreen(WHITE);

    gfx->setTextSize(2);
    gfx->setTextColor(PURPLE);
    gfx->setCursor(50, 20);
    gfx->printf("Touch Info");

    GFX_Print_Touch_Info_Loop();

    GFX_Print_1();
}

void Original_Test_2()
{
    gfx->fillScreen(WHITE);

    gfx->setTextSize(2);
    gfx->setTextColor(PURPLE);
    gfx->setCursor(60, 30);
    gfx->printf("RTC Info");

    GFX_Print_RTC_Switch_Info();

    GFX_Print_1();
}

void Original_Test_3()
{
    gfx->fillScreen(WHITE);

    gfx->setTextSize(2);
    gfx->setTextColor(PURPLE);
    gfx->setCursor(20, 30);
    gfx->printf("Microphone Info");

    // gfx->setCursor(30, 40);
    // gfx->setTextColor(RED);
    // gfx->printf("Please speak into");

    // gfx->setCursor(30, 50);
    // gfx->printf("the microphone");

    pinMode(MP34DT05TR_EN, OUTPUT);
    digitalWrite(MP34DT05TR_EN, LOW);

    if (IIS->begin(I2S_MODE_PDM, AD_IIS_DATA_IN, I2S_CHANNEL_FMT_RIGHT_LEFT,
                   IIS_DATA_BIT, IIS_SAMPLE_RATE) == false)
    {
        Serial.println("MP34DT05TR initialization fail");
        delay(500);
    }
    else
    {
        Serial.println("MP34DT05TR initialization successfully");
    }

    GFX_Print_1();
}

void Original_Test_4()
{
    gfx->fillScreen(WHITE);

    gfx->setTextSize(2);
    gfx->setTextColor(PURPLE);
    gfx->setCursor(60, 30);
    gfx->printf("SD Info");

    gfx->setCursor(0, 100);
    gfx->setTextSize(1);
    gfx->setTextColor(BLACK);

    SPI.setFrequency(16000000);
    SPI.begin(SD_SCLK, SD_MISO, SD_MOSI);

    // gfx->fillScreen(WHITE);
    // GFX_Print_FINISH();
    GFX_Print_1();
}

void Original_Test_5()
{
    Wifi_STA_Test();

    if (Wifi_Connection_Flag == true)
    {
        // Obtain and set the time from the network time server
        // After successful acquisition, the chip will use the RTC clock to update the holding time
        configTime(GMT_OFFSET_SEC, DAY_LIGHT_OFFSET_SEC, NTP_SERVER1, NTP_SERVER2, NTP_SERVER3);
        PrintLocalTime();

        delay(2000);

        gfx->fillScreen(WHITE);
        gfx->setTextColor(BLACK);
        gfx->setCursor(0, 0);

        SD.end();
        SPI.begin(SD_SCLK, SD_MISO, SD_MOSI, SD_CS);
        if (SD.begin(SD_CS, SPI) == true)
        {
            SD_Initialization_Flag = true;
            WIFI_HTTP_Download_SD_File();
        }
        else
        {
            SD_Initialization_Flag = false;
            Serial.printf("SD card initialization failed\n");
            gfx->print("\n\n\nSD card initialization failed");
        }
    }
    else
    {
        gfx->setCursor(80, 200);
        gfx->setTextColor(RED);
        gfx->print("Not connected to the network");
    }
    // delay(3000);

    // gfx->fillScreen(WHITE);
    // GFX_Print_FINISH();
    GFX_Print_1();
}

void Original_Test_6()
{
    if (audio.setPinout(MAX98357A_BCLK, MAX98357A_LRCLK, MAX98357A_DATA) != true)
    {
        Serial.println("Music driver initialization failed");
        Music_Start_Playing_Flag = false;
        delay(1000);
    }
    else
    {
        Serial.println("Music driver initialization successfully");

        gfx->fillScreen(WHITE);
        gfx->setCursor(0, 100);
        gfx->setTextSize(1);
        gfx->setTextColor(PURPLE);
        gfx->println("Trying to play music...");

        while (1)
        {
            if (SD.begin(SD_CS, SPI) == true)
            {
                if (SD_File_Download_Check_Flag == true)
                {
                    if (audio.connecttoSD(SD_FILE_NAME_TEMP) == false)
                    {
                        Music_Start_Playing_Flag = false;
                    }
                    else
                    {
                        Music_Start_Playing_Flag = true;
                        break;
                    }
                }
                else
                {
                    if (audio.connecttoSD(SD_FILE_NAME) == false)
                    {
                        Music_Start_Playing_Flag = false;
                    }
                    else
                    {
                        Music_Start_Playing_Flag = true;
                        break;
                    }
                }
            }
            else
            {
                gfx->printf("SD card initialization failed\n");
            }

            gfx->print(".");

            Music_Start_Playing_Count++;
            if (Music_Start_Playing_Count > 10) // 10秒超时
            {
                Music_Start_Playing_Flag = false;
                break;
            }
            delay(1000);
        }

        Music_Start_Playing_Count = 0;

        while (audio.getAudioCurrentTime() <= 0)
        {
            audio.loop();

            if (millis() > CycleTime)
            {
                Music_Start_Playing_Count++;
                if (Music_Start_Playing_Count > 10) // 10秒后下载超时
                {
                    Music_Start_Playing_Flag = false;
                    break;
                }
                CycleTime = millis() + 1000;
            }
        }
    }

    gfx->fillScreen(WHITE);
    GFX_Print_1();

    if (Music_Start_Playing_Flag == true)
    {
        GFX_Print_Volume();
        GFX_Print_Music_Volume_Value();
        GFX_Print_Play_Progress();
    }
    else
    {
        GFX_Print_Play_Failed();
    }
}

void Original_Test_7()
{
    gfx->fillScreen(WHITE);

    gfx->setTextSize(2);
    gfx->setTextColor(PURPLE);
    gfx->setCursor(25, 30);
    gfx->printf("Vibration Info");

    GFX_Print_Strength();
    GFX_Print_Motor_Strength_Value();

    GFX_Print_1();
}

void Original_Test_8()
{
    gfx->fillScreen(WHITE);

    GFX_Print_LR1121_Info();
    GFX_Print_LR1121_Operation_Button();

    GFX_Print_1();
}

void Original_Test_Loop()
{
    GFX_Print_TEST("Touch Test");
    if (Skip_Current_Test == false)
    {
        Original_Test_1();

        while (1)
        {
            bool temp = false;

            // if (CST226SE->IIC_Interrupt_Flag == true)
            // {
            //     CST226SE->IIC_Interrupt_Flag = false;

            uint8_t fingers_number = CST226SE->IIC_Read_Device_Value(CST226SE->Arduino_IIC_Touch::Value_Information::TOUCH_FINGER_NUMBER);

            if (fingers_number > 0)
            {
                int32_t touch_x = CST226SE->IIC_Read_Device_Value(CST226SE->Arduino_IIC_Touch::Value_Information::TOUCH_COORDINATE_X);
                int32_t touch_y = CST226SE->IIC_Read_Device_Value(CST226SE->Arduino_IIC_Touch::Value_Information::TOUCH_COORDINATE_Y);

                GFX_Print_Touch_Info_Loop();
                if (touch_x > 20 && touch_x < 100 &&
                    touch_y > (LCD_HEIGHT - (LCD_HEIGHT / 4)) && touch_y < (LCD_HEIGHT - (LCD_HEIGHT / 4) + 40))
                {
                    Vibration_Start();
                    GFX_Print_TEST("Touch Test");
                    Original_Test_1();
                    if (Skip_Current_Test == true)
                    {
                        temp = true;
                    }
                }
                if (touch_x > 120 && touch_x < 200 &&
                    touch_y > (LCD_HEIGHT - (LCD_HEIGHT / 4)) && touch_y < (LCD_HEIGHT - (LCD_HEIGHT / 4) + 40))
                {
                    Vibration_Start();
                    temp = true;
                }
            }
            // }

            if (temp == true)
            {
                break;
            }
        }
    }

    GFX_Print_TEST("Vibration Motor Test");
    if (Skip_Current_Test == false)
    {
        Original_Test_7();
        while (1)
        {
            bool temp = false;

            // if (CST226SE->IIC_Interrupt_Flag == true)
            // {
            //     CST226SE->IIC_Interrupt_Flag = false;

            uint8_t fingers_number = CST226SE->IIC_Read_Device_Value(CST226SE->Arduino_IIC_Touch::Value_Information::TOUCH_FINGER_NUMBER);

            if (fingers_number > 0)
            {
                int32_t touch_x = CST226SE->IIC_Read_Device_Value(CST226SE->Arduino_IIC_Touch::Value_Information::TOUCH_COORDINATE_X);
                int32_t touch_y = CST226SE->IIC_Read_Device_Value(CST226SE->Arduino_IIC_Touch::Value_Information::TOUCH_COORDINATE_Y);

                if (touch_x > 20 && touch_x < 100 &&
                    touch_y > (LCD_HEIGHT - (LCD_HEIGHT / 4)) && touch_y < (LCD_HEIGHT - (LCD_HEIGHT / 4) + 40))
                {
                    Vibration_Start();
                    GFX_Print_TEST("Vibration Motor Test");
                    Original_Test_7();
                    if (Skip_Current_Test == true)
                    {
                        temp = true;
                    }
                }
                if (touch_x > 120 && touch_x < 200 &&
                    touch_y > (LCD_HEIGHT - (LCD_HEIGHT / 4)) && touch_y < (LCD_HEIGHT - (LCD_HEIGHT / 4) + 40))
                {
                    Vibration_Start();
                    temp = true;
                }

                if (touch_x > 30 && touch_x < 80 &&
                    touch_y > LCD_HEIGHT / 3 && touch_y < LCD_HEIGHT / 3 + 50)
                {
                    Strength_Value--;
                    if (Strength_Value < 0)
                    {
                        Strength_Value = 0;
                    }
                }
                if (touch_x > 140 && touch_x < 190 &&
                    touch_y > LCD_HEIGHT / 3 && touch_y < LCD_HEIGHT / 3 + 50)
                {
                    Strength_Value++;
                    if (Strength_Value > 20)
                    {
                        Strength_Value = 20;
                    }
                }

                ledcWrite(2, Strength_Value * 10);
                GFX_Print_Motor_Strength_Value();
                delay(200);
            }
            // }

            if (temp == true)
            {
                ledcWrite(2, 0);
                break;
            }
        }
    }

    GFX_Print_TEST("RTC Test");
    if (Skip_Current_Test == false)
    {
        Original_Test_2();
        while (1)
        {
            bool temp = false;

            if (millis() > CycleTime)
            {
                GFX_Print_RTC_Info_Loop();
                CycleTime = millis() + 1000;
            }

            // if (CST226SE->IIC_Interrupt_Flag == true)
            // {
            //     CST226SE->IIC_Interrupt_Flag = false;

            uint8_t fingers_number = CST226SE->IIC_Read_Device_Value(CST226SE->Arduino_IIC_Touch::Value_Information::TOUCH_FINGER_NUMBER);

            if (fingers_number > 0)
            {
                int32_t touch_x = CST226SE->IIC_Read_Device_Value(CST226SE->Arduino_IIC_Touch::Value_Information::TOUCH_COORDINATE_X);
                int32_t touch_y = CST226SE->IIC_Read_Device_Value(CST226SE->Arduino_IIC_Touch::Value_Information::TOUCH_COORDINATE_Y);

                if (touch_x > 20 && touch_x < 100 &&
                    touch_y > (LCD_HEIGHT - (LCD_HEIGHT / 4)) && touch_y < (LCD_HEIGHT - (LCD_HEIGHT / 4) + 40))
                {
                    Vibration_Start();
                    GFX_Print_TEST("RTC Test");
                    Original_Test_2();
                    if (Skip_Current_Test == true)
                    {
                        temp = true;
                    }
                }
                if (touch_x > 120 && touch_x < 200 &&
                    touch_y > (LCD_HEIGHT - (LCD_HEIGHT / 4)) && touch_y < (LCD_HEIGHT - (LCD_HEIGHT / 4) + 40))
                {
                    Vibration_Start();
                    temp = true;
                }

                if (touch_x > (LCD_WIDTH / 4) && touch_x < (LCD_WIDTH / 4 + LCD_WIDTH / 2) &&
                    touch_y > (LCD_HEIGHT - (LCD_HEIGHT / 3) - 30) && touch_y < (LCD_HEIGHT - (LCD_HEIGHT / 3) - 30 + 50))
                {
                    // 重置时间
                    //  关闭RTC
                    PCF85063->IIC_Write_Device_State(PCF85063->Arduino_IIC_RTC::Device::RTC_CLOCK_RTC,
                                                     PCF85063->Arduino_IIC_RTC::Device_State::RTC_DEVICE_OFF);
                    // 时钟传感器设置秒
                    PCF85063->IIC_Write_Device_Value(PCF85063->Arduino_IIC_RTC::Device_Value::RTC_SET_SECOND_DATA,
                                                     58);
                    // 时钟传感器设置分
                    PCF85063->IIC_Write_Device_Value(PCF85063->Arduino_IIC_RTC::Device_Value::RTC_SET_MINUTE_DATA,
                                                     59);
                    // 时钟传感器设置时
                    PCF85063->IIC_Write_Device_Value(PCF85063->Arduino_IIC_RTC::Device_Value::RTC_SET_HOUR_DATA,
                                                     23);
                    // 时钟传感器设置天
                    PCF85063->IIC_Write_Device_Value(PCF85063->Arduino_IIC_RTC::Device_Value::RTC_SET_DAY_DATA,
                                                     31);
                    // 时钟传感器设置月
                    PCF85063->IIC_Write_Device_Value(PCF85063->Arduino_IIC_RTC::Device_Value::RTC_SET_MONTH_DATA,
                                                     12);
                    // 时钟传感器设置
                    PCF85063->IIC_Write_Device_Value(PCF85063->Arduino_IIC_RTC::Device_Value::RTC_SET_YEAR_DATA,
                                                     99);
                    // 开启RTC
                    PCF85063->IIC_Write_Device_State(PCF85063->Arduino_IIC_RTC::Device::RTC_CLOCK_RTC,
                                                     PCF85063->Arduino_IIC_RTC::Device_State::RTC_DEVICE_ON);

                    Vibration_Start();
                }
            }
            // }

            if (temp == true)
            {
                break;
            }
        }
    }

    GFX_Print_TEST("Microphone Test");
    if (Skip_Current_Test == false)
    {
        Original_Test_3();
        while (1)
        {
            bool temp = false;

            if (IIS->IIS_Read_Data(IIS_Read_Buff, 100) == true)
            {
                gfx->fillRect(0, 70, 222, 110, WHITE);

                gfx->setCursor(20, 85);
                gfx->setTextColor(BLUE);
                gfx->printf("Left Data:%d", (int16_t)((int16_t)IIS_Read_Buff[0] | (int16_t)IIS_Read_Buff[1] << 8));

                gfx->setCursor(20, 100);
                gfx->printf("Right Data:%d", (int16_t)((int16_t)IIS_Read_Buff[2] | (int16_t)IIS_Read_Buff[3] << 8));
                delay(100);
            }
            else
            {
                gfx->setTextColor(RED);
                gfx->fillRect(0, 70, 222, 110, WHITE);

                gfx->setCursor(20, 100);
                gfx->printf("Microphone Data: fail");
                delay(100);
            }

            // if (CST226SE->IIC_Interrupt_Flag == true)
            // {
            //     CST226SE->IIC_Interrupt_Flag = false;

            uint8_t fingers_number = CST226SE->IIC_Read_Device_Value(CST226SE->Arduino_IIC_Touch::Value_Information::TOUCH_FINGER_NUMBER);

            if (fingers_number > 0)
            {
                int32_t touch_x = CST226SE->IIC_Read_Device_Value(CST226SE->Arduino_IIC_Touch::Value_Information::TOUCH_COORDINATE_X);
                int32_t touch_y = CST226SE->IIC_Read_Device_Value(CST226SE->Arduino_IIC_Touch::Value_Information::TOUCH_COORDINATE_Y);

                if (touch_x > 20 && touch_x < 100 &&
                    touch_y > (LCD_HEIGHT - (LCD_HEIGHT / 4)) && touch_y < (LCD_HEIGHT - (LCD_HEIGHT / 4) + 40))
                {
                    Vibration_Start();
                    GFX_Print_TEST("Microphone Test");
                    Original_Test_3();
                    if (Skip_Current_Test == true)
                    {
                        temp = true;
                    }
                }
                if (touch_x > 120 && touch_x < 200 &&
                    touch_y > (LCD_HEIGHT - (LCD_HEIGHT / 4)) && touch_y < (LCD_HEIGHT - (LCD_HEIGHT / 4) + 40))
                {
                    Vibration_Start();
                    temp = true;
                }
            }
            // }

            if (temp == true)
            {
                IIS->end();
                break;
            }
        }
    }

    GFX_Print_TEST("SD Test");
    if (Skip_Current_Test == false)
    {
        Original_Test_4();
        while (1)
        {
            bool temp = false;

            if (millis() > CycleTime)
            {
                SD_Test_Loop();
                CycleTime = millis() + 1000;
            }

            // if (CST226SE->IIC_Interrupt_Flag == true)
            // {
            //     CST226SE->IIC_Interrupt_Flag = false;

            uint8_t fingers_number = CST226SE->IIC_Read_Device_Value(CST226SE->Arduino_IIC_Touch::Value_Information::TOUCH_FINGER_NUMBER);

            if (fingers_number > 0)
            {
                int32_t touch_x = CST226SE->IIC_Read_Device_Value(CST226SE->Arduino_IIC_Touch::Value_Information::TOUCH_COORDINATE_X);
                int32_t touch_y = CST226SE->IIC_Read_Device_Value(CST226SE->Arduino_IIC_Touch::Value_Information::TOUCH_COORDINATE_Y);

                if (touch_x > 20 && touch_x < 100 &&
                    touch_y > (LCD_HEIGHT - (LCD_HEIGHT / 4)) && touch_y < (LCD_HEIGHT - (LCD_HEIGHT / 4) + 40))
                {
                    Vibration_Start();
                    GFX_Print_TEST("SD Test");
                    Original_Test_4();
                    if (Skip_Current_Test == true)
                    {
                        temp = true;
                    }
                }
                if (touch_x > 120 && touch_x < 200 &&
                    touch_y > (LCD_HEIGHT - (LCD_HEIGHT / 4)) && touch_y < (LCD_HEIGHT - (LCD_HEIGHT / 4) + 40))
                {
                    Vibration_Start();
                    temp = true;
                }
            }
            // }

            if (temp == true)
            {
                break;
            }
        }
    }

    GFX_Print_TEST("WIFI STA Test");
    if (Skip_Current_Test == false)
    {
        Original_Test_5();
        while (1)
        {
            bool temp = false;

            // if (CST226SE->IIC_Interrupt_Flag == true)
            // {
            //     CST226SE->IIC_Interrupt_Flag = false;

            uint8_t fingers_number = CST226SE->IIC_Read_Device_Value(CST226SE->Arduino_IIC_Touch::Value_Information::TOUCH_FINGER_NUMBER);

            if (fingers_number > 0)
            {
                int32_t touch_x = CST226SE->IIC_Read_Device_Value(CST226SE->Arduino_IIC_Touch::Value_Information::TOUCH_COORDINATE_X);
                int32_t touch_y = CST226SE->IIC_Read_Device_Value(CST226SE->Arduino_IIC_Touch::Value_Information::TOUCH_COORDINATE_Y);

                if (touch_x > 20 && touch_x < 100 &&
                    touch_y > (LCD_HEIGHT - (LCD_HEIGHT / 4)) &&
                    touch_y < (LCD_HEIGHT - (LCD_HEIGHT / 4) + 40))
                {
                    Vibration_Start();
                    GFX_Print_TEST("WIFI STA Test");
                    Original_Test_5();
                    if (Skip_Current_Test == true)
                    {
                        temp = true;
                    }
                }
                if (touch_x > 120 && touch_x < 200 &&
                    touch_y > (LCD_HEIGHT - (LCD_HEIGHT / 4)) &&
                    touch_y < (LCD_HEIGHT - (LCD_HEIGHT / 4) + 40))
                {
                    Vibration_Start();
                    temp = true;
                }
            }
            // }

            if (temp == true)
            {
                break;
            }
        }
    }

    GFX_Print_TEST("SD Music Test");
    if (Skip_Current_Test == false)
    {
        Original_Test_6();
        while (1)
        {
            bool temp = false;

            if (Music_Start_Playing_Flag == true)
            {
                if (millis() > CycleTime)
                {
                    GFX_Print_Play_Progress();
                    CycleTime = millis() + 1000;
                }

                audio.loop();
            }

            // if (CST226SE->IIC_Interrupt_Flag == true)
            // {
            //     CST226SE->IIC_Interrupt_Flag = false;

            uint8_t fingers_number = CST226SE->IIC_Read_Device_Value(CST226SE->Arduino_IIC_Touch::Value_Information::TOUCH_FINGER_NUMBER);

            if (fingers_number > 0)
            {
                int32_t touch_x = CST226SE->IIC_Read_Device_Value(CST226SE->Arduino_IIC_Touch::Value_Information::TOUCH_COORDINATE_X);
                int32_t touch_y = CST226SE->IIC_Read_Device_Value(CST226SE->Arduino_IIC_Touch::Value_Information::TOUCH_COORDINATE_Y);

                if (touch_x > 20 && touch_x < 100 &&
                    touch_y > (LCD_HEIGHT - (LCD_HEIGHT / 4)) &&
                    touch_y < (LCD_HEIGHT - (LCD_HEIGHT / 4) + 40))
                {
                    Vibration_Start();
                    GFX_Print_TEST("SD Music Test");
                    Original_Test_6();
                    if (Skip_Current_Test == true)
                    {
                        temp = true;
                    }
                }
                if (touch_x > 120 && touch_x < 200 &&
                    touch_y > (LCD_HEIGHT - (LCD_HEIGHT / 4)) &&
                    touch_y < (LCD_HEIGHT - (LCD_HEIGHT / 4) + 40))
                {
                    Vibration_Start();
                    temp = true;
                }

                if (Music_Start_Playing_Flag == true)
                {
                    if (touch_x > 30 && touch_x < 80 &&
                        touch_y > LCD_HEIGHT / 3 && touch_y < LCD_HEIGHT / 3 + 50)
                    {
                        Volume_Value--;
                        Vibration_Start();
                        if (Volume_Value < 0)
                        {
                            Volume_Value = 0;
                        }
                    }
                    if (touch_x > 140 && touch_x < 190 &&
                        touch_y > LCD_HEIGHT / 3 && touch_y < LCD_HEIGHT / 3 + 50)
                    {
                        Volume_Value++;
                        Vibration_Start();
                        if (Volume_Value > 21)
                        {
                            Volume_Value = 21;
                        }
                    }

                    audio.setVolume(Volume_Value);
                    GFX_Print_Music_Volume_Value();
                }
            }
            // }

            if (temp == true)
            {
                break;
            }
        }
    }

    GFX_Print_TEST("LR1121 Lora Test");
    if (Skip_Current_Test == false)
    {
        Original_Test_8();
        while (1)
        {
            bool temp = false;

            GFX_Print_LR1121_Info_Loop();

            // if (CST226SE->IIC_Interrupt_Flag == true)
            // {
            //     CST226SE->IIC_Interrupt_Flag = false;

            uint8_t fingers_number = CST226SE->IIC_Read_Device_Value(CST226SE->Arduino_IIC_Touch::Value_Information::TOUCH_FINGER_NUMBER);

            if (fingers_number > 0)
            {
                int32_t touch_x = CST226SE->IIC_Read_Device_Value(CST226SE->Arduino_IIC_Touch::Value_Information::TOUCH_COORDINATE_X);
                int32_t touch_y = CST226SE->IIC_Read_Device_Value(CST226SE->Arduino_IIC_Touch::Value_Information::TOUCH_COORDINATE_Y);

                if (touch_x > 20 && touch_x < 100 &&
                    touch_y > (LCD_HEIGHT - (LCD_HEIGHT / 4)) &&
                    touch_y < (LCD_HEIGHT - (LCD_HEIGHT / 4) + 40))
                {
                    Vibration_Start();
                    delay(500);
                    GFX_Print_TEST("LR1121 Lora Test");
                    Original_Test_8();
                    if (Skip_Current_Test == true)
                    {
                        temp = true;
                    }
                }
                if (touch_x > 120 && touch_x < 200 &&
                    touch_y > (LCD_HEIGHT - (LCD_HEIGHT / 4)) &&
                    touch_y < (LCD_HEIGHT - (LCD_HEIGHT / 4) + 40))
                {
                    Vibration_Start();
                    temp = true;
                }

                if (touch_x > 20 && touch_x < 100 &&
                    touch_y > (LCD_HEIGHT - (LCD_HEIGHT / 3) - 30) && touch_y < (LCD_HEIGHT - (LCD_HEIGHT / 3) - 30 + 50))
                {
                    Vibration_Start();
                    Lora_OP.device_1.send_flag = true;
                    Lora_OP.device_1.connection_flag = Lora_OP.state::CONNECTING;
                    // 清除错误计数看门狗
                    Lora_OP.device_1.error_count = 0;
                    CycleTime_2 = millis() + 1000;
                }
                if (touch_x > 120 && touch_x < 200 &&
                    touch_y > (LCD_HEIGHT - (LCD_HEIGHT / 3) - 30) && touch_y < (LCD_HEIGHT - (LCD_HEIGHT / 3) - 30 + 50))
                {
                    if (millis() > CycleTime_4)
                    {
                        Vibration_Start();
                        Lora_OP.device_1.send_flag = false;
                        Lora_OP.device_1.error_count = 11;
                        Lora_OP.device_1.send_data = 0;
                        Lora_OP.device_1.connection_flag = Lora_OP.state::UNCONNECTED;

                        Lora_Value_Change_Mode = !Lora_Value_Change_Mode;
                        if (Lora_Value_Change_Mode == 1) // 2.4GHz模式
                        {
                            Lora_OP.frequency.value = 2400.1;
                            Lora_OP.output_power.value = 13;
                        }
                        else
                        {
                            Lora_OP.frequency.value = 868.1;
                            Lora_OP.output_power.value = 22;
                        }
                        String temp_str;
                        if (Lora_Configuration_Default_Value_Parameters(&temp_str) == false)
                        {
                            Serial.printf("LR1121 Failed to set default parameters\n");
                            Serial.printf("LR1121 assertion: %s\n", temp_str.c_str());
                            // Lora_OP.initialization_flag = false;

                            if (Lora_Value_Change_Mode == 1) // 2.4GHz模式
                            {
                                Lora_OP.frequency.value = 868.1;
                                Lora_OP.output_power.value = 22;
                            }
                            else
                            {
                                Lora_OP.frequency.value = 2400.1;
                                Lora_OP.output_power.value = 13;
                            }
                        }

                        CycleTime_4 = millis() + 500;
                    }
                }
            }
            // }

            if (temp == true)
            {
                radio.sleep();
                break;
            }
        }
    }
}

void setup()
{
    Serial.begin(115200);
    Serial.println("Ciallo");
    Serial.println("[T-Display-S3-Pro-MVSRLora_" + (String)BOARD_VERSION "][" + (String)SOFTWARE_NAME +
                   "]_firmware_" + (String)SOFTWARE_LASTEDITTIME);

    pinMode(RT9080_EN, OUTPUT);
    digitalWrite(RT9080_EN, HIGH);

    pinMode(LR1121_CS, OUTPUT);
    digitalWrite(LR1121_CS, HIGH);
    pinMode(SD_CS, OUTPUT);
    digitalWrite(SD_CS, HIGH);
    pinMode(LCD_CS, OUTPUT);
    digitalWrite(LCD_CS, HIGH);

    pinMode(MP34DT05TR_EN, OUTPUT);
    digitalWrite(MP34DT05TR_EN, LOW);

    pinMode(MAX98357A_EN, OUTPUT);
    digitalWrite(MAX98357A_EN, HIGH);

    ledcAttachPin(LCD_BL, 1);
    ledcSetup(1, 2000, 8);
    ledcWrite(1, 255);

    ledcAttachPin(VIBRATINO_MOTOR_PWM, 2);
    ledcSetup(2, 12000, 8);
    ledcWrite(2, 0);

    if (SY6970->begin() == false)
    {
        Serial.println("SY6970 initialization fail");
        delay(2000);
    }
    else
    {
        Serial.println("SY6970 initialization successfully");
    }

    // 开启ADC测量功能
    if (SY6970->IIC_Write_Device_State(SY6970->Arduino_IIC_Power::Device::POWER_DEVICE_ADC_MEASURE,
                                       SY6970->Arduino_IIC_Power::Device_State::POWER_DEVICE_ON) == false)
    {
        Serial.println("SY6970 ADC Measure ON fail");
        delay(2000);
    }
    else
    {
        Serial.println("SY6970 ADC Measure ON successfully");
    }

    // 禁用看门狗定时器喂狗功能
    SY6970->IIC_Write_Device_Value(SY6970->Arduino_IIC_Power::Device_Value::POWER_DEVICE_WATCHDOG_TIMER, 0);
    // 热调节阈值设置为60度
    SY6970->IIC_Write_Device_Value(SY6970->Arduino_IIC_Power::Device_Value::POWER_DEVICE_THERMAL_REGULATION_THRESHOLD, 60);
    // 充电目标电压电压设置为4224mV
    SY6970->IIC_Write_Device_Value(SY6970->Arduino_IIC_Power::Device_Value::POWER_DEVICE_CHARGING_TARGET_VOLTAGE_LIMIT, 4224);
    // 最小系统电压限制为3600mA
    SY6970->IIC_Write_Device_Value(SY6970->Arduino_IIC_Power::Device_Value::POWER_DEVICE_MINIMUM_SYSTEM_VOLTAGE_LIMIT, 3600);
    // 设置OTG电压为5062mV
    SY6970->IIC_Write_Device_Value(SY6970->Arduino_IIC_Power::Device_Value::POWER_DEVICE_OTG_VOLTAGE_LIMIT, 5062);
    // 输入电流限制设置为2100mA
    SY6970->IIC_Write_Device_Value(SY6970->Arduino_IIC_Power::Device_Value::POWER_DEVICE_INPUT_CURRENT_LIMIT, 2100);
    // 快速充电电流限制设置为2112mA
    SY6970->IIC_Write_Device_Value(SY6970->Arduino_IIC_Power::Device_Value::POWER_DEVICE_FAST_CHARGING_CURRENT_LIMIT, 2112);
    // 预充电电流限制设置为192mA
    SY6970->IIC_Write_Device_Value(SY6970->Arduino_IIC_Power::Device_Value::POWER_DEVICE_PRECHARGE_CHARGING_CURRENT_LIMIT, 192);
    // 终端充电电流限制设置为320mA
    SY6970->IIC_Write_Device_Value(SY6970->Arduino_IIC_Power::Device_Value::POWER_DEVICE_TERMINATION_CHARGING_CURRENT_LIMIT, 320);
    // OTG电流限制设置为500mA
    SY6970->IIC_Write_Device_Value(SY6970->Arduino_IIC_Power::Device_Value::POWER_DEVICE_OTG_CHARGING_LIMIT, 500);

    if (CST226SE->begin(400000UL) == false)
    {
        Serial.println("CST226SE initialization fail");
        delay(2000);
    }
    else
    {
        Serial.println("CST226SE initialization successfully");
    }

    // 目前休眠功能只能进入不能退出 要退出只能系统重置
    // CST226SE->IIC_Write_Device_State(CST226SE->Arduino_IIC_Touch::Device::TOUCH_DEVICE_SLEEP_MODE,
    //                                 CST226SE->Arduino_IIC_Touch::Device_State::TOUCH_DEVICE_ON);

    if (PCF85063->begin() == false)
    {
        Serial.println("PCF85063 initialization fail");
        delay(2000);
    }
    else
    {
        Serial.println("PCF85063 initialization successfully");
    }

    // 设置时间格式为24小时制
    PCF85063->IIC_Write_Device_State(PCF85063->Arduino_IIC_RTC::Device::RTC_CLOCK_TIME_FORMAT,
                                     PCF85063->Arduino_IIC_RTC::Device_Mode::RTC_CLOCK_TIME_FORMAT_24);

    // 关闭时钟输出
    PCF85063->IIC_Write_Device_State(PCF85063->Arduino_IIC_RTC::Device::RTC_CLOCK_OUTPUT_VALUE,
                                     PCF85063->Arduino_IIC_RTC::Device_Mode::RTC_CLOCK_OUTPUT_OFF);

    // 开启RTC
    PCF85063->IIC_Write_Device_State(PCF85063->Arduino_IIC_RTC::Device::RTC_CLOCK_RTC,
                                     PCF85063->Arduino_IIC_RTC::Device_State::RTC_DEVICE_ON);

    Volume_Value = 3;
    audio.setVolume(Volume_Value); // 0...21,Volume setting

    gfx->begin();
    gfx->fillScreen(WHITE);

    Original_Test_Loop();

    gfx->setTextSize(1);
    gfx->fillScreen(PALERED);
}

void loop()
{
    if (millis() > CycleTime)
    {
        GFX_Print_Time_Info_Loop();
        CycleTime = millis() + 1000;
    }

    // if (CST226SE->IIC_Interrupt_Flag == true)
    // {
    //     CST226SE->IIC_Interrupt_Flag = false;

    uint8_t fingers_number = CST226SE->IIC_Read_Device_Value(CST226SE->Arduino_IIC_Touch::Value_Information::TOUCH_FINGER_NUMBER);

    if (fingers_number > 0)
    {
        Vibration_Start();

        int32_t touch_x = CST226SE->IIC_Read_Device_Value(CST226SE->Arduino_IIC_Touch::Value_Information::TOUCH_COORDINATE_X);
        int32_t touch_y = CST226SE->IIC_Read_Device_Value(CST226SE->Arduino_IIC_Touch::Value_Information::TOUCH_COORDINATE_Y);

        switch (Image_Flag)
        {
        case 0:
            gfx->draw16bitRGBBitmap(0, 0, (uint16_t *)gImage_1, LCD_WIDTH, LCD_HEIGHT); // RGB
            break;
        case 1:
            gfx->draw16bitRGBBitmap(0, 0, (uint16_t *)gImage_2, LCD_WIDTH, LCD_HEIGHT); // RGB
            break;

        default:
            break;
        }

        Image_Flag++;

        if (Image_Flag > 1)
        {
            Image_Flag = 0;
        }

        Serial.printf("[1] point x: %d  point y: %d \r\n", touch_x, touch_y);

        gfx->setCursor(touch_x, touch_y);
        gfx->setTextColor(RED);
        gfx->printf("[1] point x: %d  point y: %d \r\n", touch_x, touch_y);
    }
    // }
}

// optional
void audio_info(const char *info)
{
    Serial.print("info        ");
    Serial.println(info);
}
void audio_id3data(const char *info)
{ // id3 metadata
    Serial.print("id3data     ");
    Serial.println(info);
}
void audio_eof_mp3(const char *info)
{ // end of file
    Serial.print("eof_mp3     ");
    Serial.println(info);
}
void audio_showstation(const char *info)
{
    Serial.print("station     ");
    Serial.println(info);
}
void audio_showstreamtitle(const char *info)
{
    Serial.print("streamtitle ");
    Serial.println(info);
}
void audio_bitrate(const char *info)
{
    Serial.print("bitrate     ");
    Serial.println(info);
}
void audio_commercial(const char *info)
{ // duration in sec
    Serial.print("commercial  ");
    Serial.println(info);
}
void audio_icyurl(const char *info)
{ // homepage
    Serial.print("icyurl      ");
    Serial.println(info);
}
void audio_lasthost(const char *info)
{ // stream URL played
    Serial.print("lasthost    ");
    Serial.println(info);
}

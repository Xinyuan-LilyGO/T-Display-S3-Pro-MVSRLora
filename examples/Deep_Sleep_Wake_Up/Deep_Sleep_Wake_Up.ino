/*
 * @Description: Sleep
 * @Author: LILYGO_L
 * @Date: 2024-03-11 10:05:32
 * @LastEditTime: 2025-02-06 10:30:55
 * @License: GPL 3.0
 */
#include "Arduino.h"
#include "pin_config.h"
#include "Arduino_GFX_Library.h"
#include "Arduino_DriveBus_Library.h"
#include "RadioLib.h"

#define SLEEP_WAKE_UP_INT GPIO_NUM_0

enum lora_configuration_default_value_mode_t
{
    LORA_1GHZ,
    LORA_2_4GHZ,
};

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

size_t CycleTime = 0;

volatile bool Lora_Transmission_Flag = false;

Arduino_DataBus *bus = new Arduino_HWSPI(
    LCD_DC /* DC */, LCD_CS /* CS */, LCD_SCLK /* SCK */, LCD_MOSI /* MOSI */, LCD_MISO /* MISO */);

Arduino_GFX *gfx = new Arduino_ST7796(
    bus, LCD_RST /* RST */, 0 /* rotation */, true /* IPS */,
    LCD_WIDTH /* width */, LCD_HEIGHT /* height */,
    49 /* col offset 1 */, 0 /* row offset 1 */, 0 /* col_offset2 */, 0 /* row_offset2 */);

std::shared_ptr<Arduino_IIC_DriveBus> IIC_Bus =
    std::make_shared<Arduino_HWIIC>(IIC_SDA, IIC_SCL, &Wire);

void Arduino_IIC_Touch_Interrupt(void);

std::unique_ptr<Arduino_IIC> CST226SE(new Arduino_CST2xxSE(IIC_Bus, CST226SE_DEVICE_ADDRESS,
                                                           TOUCH_RST, TOUCH_INT, Arduino_IIC_Touch_Interrupt));

LR1121 radio = new Module(LR1121_CS, LR1121_INT, LR1121_RST, LR1121_BUSY, SPI);

void Arduino_IIC_Touch_Interrupt(void)
{
    CST226SE->IIC_Interrupt_Flag = true;
}

void Lora_Transmission_Interrupt(void)
{
    Lora_Transmission_Flag = true;
}

int16_t Lora_Configuration_Default_Value(lora_configuration_default_value_mode_t mode)
{
    int16_t state = RADIOLIB_ERR_NONE;

    switch (mode)
    {
    case LORA_1GHZ:
        state = radio.setFrequency(868.1);
        state = radio.setBandwidth(125.0, false);
        state = radio.setOutputPower(22);
        break;
    case LORA_2_4GHZ:
        state = radio.setFrequency(2400.1);
        state = radio.setBandwidth(812.5, true);
        state = radio.setOutputPower(13);
        break;

    default:
        break;
    }

    state = radio.setSpreadingFactor(12);
    state = radio.setCodingRate(6);
    state = radio.setSyncWord(0xAB);
    state = radio.setPreambleLength(16);
    state = radio.setCRC(false);

    if (state != RADIOLIB_ERR_NONE)
    {
        Serial.printf("Lora configuration default value fail\nCode: %hu\n", state);
    }

    return state;
}

void setup()
{
    Serial.begin(115200);
    Serial.println("Ciallo");

    pinMode(SLEEP_WAKE_UP_INT, INPUT_PULLUP);

    pinMode(RT9080_EN, OUTPUT);
    digitalWrite(RT9080_EN, HIGH);
    pinMode(MP34DT05TR_EN, OUTPUT);
    digitalWrite(MP34DT05TR_EN, LOW);
    pinMode(MAX98357A_EN, OUTPUT);
    digitalWrite(MAX98357A_EN, HIGH);

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
        // while (true)
        //     ;
    }

    Lora_Configuration_Default_Value(LORA_1GHZ);

    radio.setPacketReceivedAction(Lora_Transmission_Interrupt);

    radio.startReceive();

    Serial.println("Enter deep sleep in 5 seconds");

    CycleTime = millis() + 5000;
}

void loop()
{
    if (millis() > CycleTime)
    {
        delay(300);

        // gfx->displayOff();
        // ledcWrite(1, 0);

        // CST226SE->IIC_Write_Device_State(CST226SE->Arduino_IIC_Touch::Device::TOUCH_DEVICE_SLEEP_MODE,
        //                                  CST226SE->Arduino_IIC_Touch::Device_State::TOUCH_DEVICE_ON);

        digitalWrite(MP34DT05TR_EN, HIGH);
        delay(1000);
        digitalWrite(MAX98357A_EN, LOW);
        delay(1000);
        radio.sleep();
        delay(1000);

        pinMode(LR1121_RST, INPUT_PULLUP);
        pinMode(LR1121_CS, INPUT_PULLUP);

        Serial.println("Enter deep sleep");
        gpio_hold_en(SLEEP_WAKE_UP_INT);
        gpio_hold_en((gpio_num_t)LR1121_RST);
        gpio_hold_en((gpio_num_t)LR1121_CS);
        esp_sleep_enable_ext0_wakeup(SLEEP_WAKE_UP_INT, LOW);
        esp_deep_sleep_start();
    }
}

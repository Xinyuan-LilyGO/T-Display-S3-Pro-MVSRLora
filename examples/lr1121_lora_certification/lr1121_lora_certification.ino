/*
 * @Description: LR1121_PingPong_2 test
 * @Author: LILYGO_L
 * @Date: 2024-12-09 10:43:42
 * @LastEditTime: 2026-03-20 11:48:36
 * @License: GPL 3.0
 */
#include "RadioLib.h"
#include "Arduino_DriveBus_Library.h"
#include "pin_config.h"

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
    {LR11x0::MODE_TX, {LOW, HIGH}},
    {LR11x0::MODE_TX_HP, {LOW, HIGH}},
    {LR11x0::MODE_TX_HF, {LOW, LOW}},
    {LR11x0::MODE_GNSS, {LOW, LOW}},
    {LR11x0::MODE_WIFI, {LOW, LOW}},
    END_OF_MODE_TABLE,
};

const uint64_t Local_MAC = ESP.getEfuseMac();

size_t CycleTime = 0;

uint32_t Receive_Data = 0;

uint8_t Send_Package[16] = {'M', 'A', 'C', ':',
                            (uint8_t)(Local_MAC >> 56), (uint8_t)(Local_MAC >> 48),
                            (uint8_t)(Local_MAC >> 40), (uint8_t)(Local_MAC >> 32),
                            (uint8_t)(Local_MAC >> 24), (uint8_t)(Local_MAC >> 16),
                            (uint8_t)(Local_MAC >> 8), (uint8_t)Local_MAC,
                            0, 0, 0, 0};

uint32_t Send_Data = 0;
bool Send_Flag = 1;

volatile bool Lora_Transmission_Flag = false;

LR1121 radio = new Module(LR1121_CS, LR1121_INT, LR1121_RST, LR1121_BUSY, SPI);

std::shared_ptr<Arduino_IIC_DriveBus> IIC_Bus =
    std::make_shared<Arduino_HWIIC>(IIC_SDA, IIC_SCL, &Wire);

std::unique_ptr<Arduino_IIC> SY6970(new Arduino_SY6970(IIC_Bus, SY6970_DEVICE_ADDRESS,
                                                       DRIVEBUS_DEFAULT_VALUE, DRIVEBUS_DEFAULT_VALUE));

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
        state = radio.setFrequency(868.0);
        state = radio.setBandwidth(125.0, false);
        state = radio.setOutputPower(8, true);
        break;
    case LORA_2_4GHZ:
        state = radio.setFrequency(2200.0);
        state = radio.setBandwidth(125.0, false);
        state = radio.setOutputPower(13);
        break;

    default:
        break;
    }

    state = radio.setSpreadingFactor(12);
    state = radio.setCodingRate(8);
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

    pinMode(RT9080_EN, OUTPUT);
    digitalWrite(RT9080_EN, HIGH);

    pinMode(LR1121_CS, OUTPUT);
    digitalWrite(LR1121_CS, HIGH);
    pinMode(SD_CS, OUTPUT);
    digitalWrite(SD_CS, HIGH);
    pinMode(LCD_CS, OUTPUT);
    digitalWrite(LCD_CS, HIGH);

    while (SY6970->begin() == false)
    {
        Serial.println("SY6970 initialization fail");
        delay(2000);
    }
    Serial.println("SY6970 initialization successfully");

    // 开启ADC测量功能
    while (SY6970->IIC_Write_Device_State(SY6970->Arduino_IIC_Power::Device::POWER_DEVICE_ADC_MEASURE,
                                          SY6970->Arduino_IIC_Power::Device_State::POWER_DEVICE_ON) == false)
    {
        Serial.println("SY6970 ADC Measure ON fail");
        delay(2000);
    }
    Serial.println("SY6970 ADC Measure ON successfully");

    SPI.begin(LR1121_SCLK, LR1121_MISO, LR1121_MOSI);

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
        while (true)
            ;
    }

    // The line radio.setRfSwitchTable(rfswitch_dio_pins, rfswitch_table); must be placed after radio.begin();
    radio.setRfSwitchTable(rfswitch_dio_pins, rfswitch_table);

    // LR1121 TCXO Voltage 2.85~3.15V
    radio.setTCXO(3.0);

    Lora_Configuration_Default_Value(LORA_1GHZ);

    radio.setPacketReceivedAction(Lora_Transmission_Interrupt);

    // radio.transmitDirect();
}

void loop()
{
    radio.setTx(0xFFFFFF);
    delay(500);
    radio.standby();
    delay(500);
}

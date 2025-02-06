/*
 * @Description: LR1121_PingPong_2 test
 * @Author: LILYGO_L
 * @Date: 2024-12-09 10:43:42
 * @LastEditTime: 2025-02-06 10:36:33
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
    {LR11x0::MODE_TX, {HIGH, HIGH}},
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

    pinMode(0, INPUT_PULLUP);

    pinMode(RT9080_EN, OUTPUT);
    digitalWrite(RT9080_EN, HIGH);

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
        while (true)
            ;
    }

    Lora_Configuration_Default_Value(LORA_1GHZ);

    radio.setPacketReceivedAction(Lora_Transmission_Interrupt);

    radio.startReceive();
}

void loop()
{
    if (digitalRead(0) == LOW)
    {
        delay(300);
        Send_Flag = true;
        CycleTime = millis() + 3000;
    }

    if (Send_Flag == true)
    {
        if (millis() > CycleTime)
        {
            Send_Flag = false;
            // send another one
            Serial.println("[SX1262] Sending another packet ... ");

            Send_Package[12] = (uint8_t)(Send_Data >> 24);
            Send_Package[13] = (uint8_t)(Send_Data >> 16);
            Send_Package[14] = (uint8_t)(Send_Data >> 8);
            Send_Package[15] = (uint8_t)Send_Data;

            radio.transmit(Send_Package, 16);
            radio.startReceive();
        }
    }

    if (Lora_Transmission_Flag)
    {
        Lora_Transmission_Flag = false;

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
                    Receive_Data =
                        ((uint32_t)receive_package[12] << 24) |
                        ((uint32_t)receive_package[13] << 16) |
                        ((uint32_t)receive_package[14] << 8) |
                        (uint32_t)receive_package[15];

                    // packet was successfully received
                    Serial.println("[LR1121] Received packet!");

                    // print data of the packet
                    for (int i = 0; i < 16; i++)
                    {
                        Serial.printf("[LR1121] Data[%d]: %#X\n", i, receive_package[i]);
                    }

                    // uint32_t temp_mac_2[2];
                    // temp_mac_2[0] =
                    //     ((uint32_t)Send_Package[8] << 24) |
                    //     ((uint32_t)Send_Package[9] << 16) |
                    //     ((uint32_t)Send_Package[10] << 8) |
                    //     (uint32_t)Send_Package[11];
                    // temp_mac_2[1] =
                    //     ((uint32_t)Send_Package[4] << 24) |
                    //     ((uint32_t)Send_Package[5] << 16) |
                    //     ((uint32_t)Send_Package[6] << 8) |
                    //     (uint32_t)Send_Package[7];

                    // uint32_t temp_mac_3[2];
                    // temp_mac_3[0] =
                    //     ((uint32_t)receive_package[8] << 24) |
                    //     ((uint32_t)receive_package[9] << 16) |
                    //     ((uint32_t)receive_package[10] << 8) |
                    //     (uint32_t)receive_package[11];
                    // temp_mac_3[1] =
                    //     ((uint32_t)receive_package[4] << 24) |
                    //     ((uint32_t)receive_package[5] << 16) |
                    //     ((uint32_t)receive_package[6] << 8) |
                    //     (uint32_t)receive_package[7];

                    // print data of the packet
                    Serial.print("[LR1121] Data:\t\t");
                    Serial.println(Receive_Data);

                    // print RSSI (Received Signal Strength Indicator)
                    Serial.print("[LR1121] RSSI:\t\t");
                    Serial.print(radio.getRSSI());
                    Serial.println(" dBm");

                    // print SNR (Signal-to-Noise Ratio)
                    Serial.print("[LR1121] SNR:\t\t");
                    Serial.print(radio.getSNR());
                    Serial.println(" dB");

                    memset(receive_package, 0, sizeof(receive_package));

                    Send_Data = Receive_Data + 1;

                    Send_Flag = true;
                    CycleTime = millis() + 3000;
                }
            }
        }
    }
}

/*
 * @Description: LR1121_Walkie_Talkie test
 * @Author: LILYGO_L
 * @Date: 2024-12-12 14:07:33
 * @LastEditTime: 2025-02-06 10:39:10
 * @License: GPL 3.0
 */
#include "RadioLib.h"
#include "pin_config.h"
#include "Arduino_DriveBus_Library.h"
#include "codec2.h"

#define IIS_SAMPLE_RATE 8000 // 采样速率
#define IIS_DATA_BIT 16      // 数据位数

#define LORA_TRANSMISSION_HEAD_SIZE 12
#define LORA_TRANSMISSION_DATA_SIZE 200

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

std::vector<short> IIS_Read_Data_Stream;
std::vector<short> IIS_Write_Data_Stream;
std::vector<unsigned char> Lora_Send_Data_Stream;
std::vector<unsigned char> Lora_Receive_Data_Stream;

bool Lora_Transmission_Mode = 0; // 默认为接收模式

uint8_t Send_Package[LORA_TRANSMISSION_HEAD_SIZE + LORA_TRANSMISSION_DATA_SIZE] = {
    'M',
    'A',
    'C',
    ':',
    (uint8_t)(Local_MAC >> 56),
    (uint8_t)(Local_MAC >> 48),
    (uint8_t)(Local_MAC >> 40),
    (uint8_t)(Local_MAC >> 32),
    (uint8_t)(Local_MAC >> 24),
    (uint8_t)(Local_MAC >> 16),
    (uint8_t)(Local_MAC >> 8),
    (uint8_t)Local_MAC,
};

volatile bool Boot_Key_Flag = false;

volatile bool Lora_Transmission_Flag = false;

std::shared_ptr<Arduino_IIS_DriveBus> IIS_Bus =
    std::make_shared<Arduino_HWIIS>(I2S_NUM_0, -1, MP34DT05TR_LRCLK, MP34DT05TR_DATA);

std::unique_ptr<Arduino_IIS> IIS(new Arduino_MEMS(IIS_Bus));

std::shared_ptr<Arduino_IIS_DriveBus> IIS_Bus_1 =
    std::make_shared<Arduino_HWIIS>(I2S_NUM_1, MAX98357A_BCLK, MAX98357A_LRCLK,
                                    MAX98357A_DATA);

std::unique_ptr<Arduino_IIS> MAX98357A(new Arduino_Amplifier(IIS_Bus_1));

LR1121 radio = new Module(LR1121_CS, LR1121_INT, LR1121_RST, LR1121_BUSY, SPI);

void Lora_Transmission_Interrupt(void)
{
    Lora_Transmission_Flag = true;
}

/**
 * @brief 输入双声道输出单声道
 * @return
 * @Date 2024-10-23 11:27:04
 */
bool IIS_Mono_Conversion(const int16_t *input_data, std::vector<int16_t> *output_data,
                         size_t input_data_size, bool extract_channel)
{
    if (input_data == nullptr || input_data_size < 1)
    {
        log_w("The input data is incorrect.");
        return false;
    }

    if ((input_data_size % 2) != 0)
    {
        log_w("The input data is not an even number and will result in the loss of one data point.");
        input_data_size--;
    }

    // 清空
    output_data->clear();

    if (extract_channel == 0) // 提取左声道
    {
        // 遍历输入 PCM 数据，提取左声道样本
        for (size_t i = 0; i < input_data_size; i += 2)
        {
            output_data->push_back(input_data[i]);
        }
    }
    else // 提取右声道
    {
        // 遍历输入 PCM 数据，提取右声道样本
        for (size_t i = 0; i < input_data_size; i += 2)
        {
            output_data->push_back(input_data[i + 1]);
        }
    }

    return true;
}

/**
 * @brief 输入单声道输出双声道
 * @return
 * @Date 2024-10-23 11:27:04
 */
bool IIS_Dual_Conversion(const int16_t *input_data, std::vector<int16_t> *output_data,
                         size_t input_data_size, float decibel_multiplier = 1.0)
{
    if (input_data == nullptr || input_data_size == 0)
    {
        log_w("The input data is incorrect.");

        return false;
    }

    // 清空
    output_data->clear();

    // 遍历输入样本
    for (size_t i = 0; i < input_data_size; i++)
    {
        // 构建双声道
        output_data->push_back((short)((float)input_data[i] * decibel_multiplier));
        output_data->push_back((short)((float)input_data[i] * decibel_multiplier));
    }

    return true;
}

/**
 * @brief 输入单声道输出双声道
 * @return
 * @Date 2024-12-13 09:08:24
 */
// bool IIS_Dual_Conversion(const int16_t *input_data, std::vector<int16_t> *output_data,
//                          size_t input_data_size, float decibel_multiplier = 1.0)
// {
//     if (input_data == nullptr || input_data_size == 0)
//     {
//         log_w("The input data is incorrect.");
//         return false;
//     }

//     // 清空
//     output_data->clear();

//     // 确保输出数据有足够的空间
//     output_data->reserve(input_data_size * 2);

//     // 遍历输入样本
//     for (size_t i = 0; i < input_data_size; i++)
//     {
//         // 计算增益后的值
//         int32_t gain_value = static_cast<int32_t>(input_data[i]) * static_cast<int32_t>(decibel_multiplier);

//         // 防止溢出
//         if (gain_value > INT16_MAX)
//         {
//             gain_value = INT16_MAX;
//         }
//         else if (gain_value < INT16_MIN)
//         {
//             gain_value = INT16_MIN;
//         }

//         // 转换为 short 并添加到输出数据中
//         output_data->push_back(static_cast<int16_t>(gain_value));
//         output_data->push_back(static_cast<int16_t>(gain_value));
//     }

//     return true;
// }

/**
 * @brief 输入双声道处理左右声道数据
 * @return
 * @Date 2024-10-23 11:27:04
 */
bool IIS_Channel_Operation(int16_t *input_data, size_t input_data_size, size_t operation_mode)
{
    if (input_data == nullptr || input_data_size < 1)
    {
        log_w("The input data is incorrect.");

        return false;
    }

    if ((input_data_size % 2) != 0)
    {
        log_w("The input data is not an even number and will result in the loss of one data point.");

        input_data_size--;
    }

    switch (operation_mode)
    {
    case 1: // 将右声道的值复制到左声道
        for (size_t i = 0; i < input_data_size; i++)
        {
            input_data[2 * i] = input_data[2 * i + 1];
        }
        break;
    case 2: // 将左声道的值复制到右声道
        for (size_t i = 0; i < input_data_size; i++)
        {
            input_data[2 * i + 1] = input_data[2 * i];
        }
        break;
    case 3: // 清空左声道值
        for (size_t i = 0; i < input_data_size; i++)
        {
            input_data[2 * i] = 0;
        }
        break;
    case 4: // 清空右声道值
        for (size_t i = 0; i < input_data_size; i++)
        {
            input_data[2 * i + 1] = 0;
        }
        break;

    default:
        break;
    }

    return true;
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

void Codec2_Task(void *parameter)
{
    struct CODEC2 *codec2_state;
    // 麦克风采样次数 1次有8个uint8_t的数据 5次就是40个uint8_t数据
    const uint8_t msm_sample_frequency = LORA_TRANSMISSION_DATA_SIZE / 8;

    // construct and configure codec2
    codec2_state = codec2_create(CODEC2_MODE_3200);
    if (codec2_state == NULL)
    {
        Serial.println("Failed to create Codec2");
        return;
    }

    // 设置LPC后置滤波器 提高解码后的语音清晰度
    codec2_set_lpc_post_filter(codec2_state, 1, 0, 0.8, 0.2);

    // 获取Codec2处理每帧音频数据所需要的采样数
    const auto codec2_sample_size = codec2_samples_per_frame(codec2_state);
    // 经过Codec2压缩后的数据量
    const auto codec2_compress_size = codec2_bytes_per_frame(codec2_state);

    // const auto c2_sample_buf_size = sizeof(short) * c2_samples_per_frame_ * sampling_multiplier;
    // auto c2_sample_buf = _make_unique<short[]>(c2_sample_buf_size);

    // auto codec2_compress_buf = _make_unique<unsigned char[]>(codec2_compress_size);

    // 麦克风（单声道）需要采集数据的数量
    const auto msm_sample_buf_size = sizeof(short) * codec2_sample_size;
    auto msm_sample_buf = (short *)malloc(msm_sample_buf_size);

    auto codec2_buf = (short *)malloc(msm_sample_buf_size);

    // 扬声器（双声道）需要播放的数据量
    // const auto max_sample_buf_size = sizeof(short) * codec2_sample_size * 2;

    auto codec2_compress_buf = (unsigned char *)malloc(sizeof(unsigned char) * codec2_compress_size);

    Serial.printf("Codec2 initialization successful\n   codec2_sample_size: %d\n    codec2_compress_size: %d\n",
                  codec2_sample_size, codec2_compress_size);

    // run loopback record-encode-decode-playback loop
    Serial.println("Audio task started");

    while (1)
    {
        delay(1);

        if (Lora_Transmission_Mode == 1) // 编码发送
        {
            for (int i = 0; i < msm_sample_frequency; i++)
            {
                if (IIS->IIS_Read_Data(msm_sample_buf, msm_sample_buf_size) == true)
                {
                    // for (int i = 0; i < 60; i++)
                    // {
                    //     Serial.printf("debug1: %d\n", c2_samples_[i]);
                    // }
                    // Serial.println();

                    // uint32_t startTimeEncode = millis();
                    codec2_encode(codec2_state, codec2_compress_buf, msm_sample_buf);
                    // Serial.println("Done encoding, took ms: " + String(millis() - startTimeEncode));

                    // delay(1);

                    // 写入Lora流中
                    const auto current_buf_size = Lora_Send_Data_Stream.size();

                    // 调整容量
                    Lora_Send_Data_Stream.resize(current_buf_size + codec2_compress_size);

                    // 存储数据
                    memcpy(Lora_Send_Data_Stream.data() + current_buf_size, codec2_compress_buf,
                           codec2_compress_size);
                }
            }
            // Serial.printf("size1: %d\n", IIS_Read_Data_Stream.size());
        }

        if (Lora_Receive_Data_Stream.size() >= codec2_compress_size) // 流读取判断
        {
            unsigned char lora_data_buf[codec2_compress_size];
            std::vector<short> output_buf;

            // 存储数据
            memcpy(lora_data_buf, Lora_Receive_Data_Stream.data(), codec2_compress_size);

            // 删除已经存储的数据
            Lora_Receive_Data_Stream.erase(Lora_Receive_Data_Stream.begin(),
                                           Lora_Receive_Data_Stream.begin() + codec2_compress_size);

            codec2_decode(codec2_state, codec2_buf, lora_data_buf);
            // Serial.println("Done decoding, took ms: " + String(millis() - startTimeDecode));

            // 使用高功率扬声器时候需要降低音量或者接外部5V2A以上电源，或者接电池
            // 电脑接口只有500mA电流，高功率扬声器最大音量会使电流抬升到1.8A
            IIS_Dual_Conversion(codec2_buf, &output_buf, codec2_sample_size, 1.0);

            // if (MAX98357A->IIS_Write_Data(output_buf.data(), max_sample_buf_size) == true)
            // {
            //     // Serial.printf("MAX98357A played successfully\n");

            //     // for (int i = 0; i < 25; i++)
            //     // {
            //     //     Serial.printf("debug: %d\n", (int16_t)(temp[i + 2] | temp[i + 3] << 8));
            //     // }
            // }

            // for (int i = 0; i < 25; i++)
            // {
            //     Serial.printf("debug: %d\n", output_buf[i]);
            // }

            const auto current_buf_size = IIS_Write_Data_Stream.size();

            // 调整容量
            IIS_Write_Data_Stream.resize(current_buf_size + output_buf.size());

            // 存储数据
            memcpy(IIS_Write_Data_Stream.data() + current_buf_size, output_buf.data(),
                   sizeof(short) * output_buf.size());
        }
    }
}

void MAX_Play_Task(void *parameter)
{
    const short max_play_size = 320;

    while (1)
    {
        delay(1);

        if (IIS_Write_Data_Stream.size() >= max_play_size) // 流读取判断
        {
            short iis_data_buf[sizeof(short) * max_play_size];

            // 存储数据
            memcpy(iis_data_buf, IIS_Write_Data_Stream.data(), sizeof(short) * max_play_size);

            // 删除已经存储的数据
            IIS_Write_Data_Stream.erase(IIS_Write_Data_Stream.begin(),
                                        IIS_Write_Data_Stream.begin() + max_play_size);

            if (MAX98357A->IIS_Write_Data(iis_data_buf, sizeof(short) * max_play_size) == true)
            {
                // Serial.printf("MAX98357A played successfully\n");

                // for (int i = 0; i < 25; i++)
                // {
                //     Serial.printf("debug: %d\n", (int16_t)(IIS_Write_Buf[i + 2] | IIS_Write_Buf[i + 3] << 8));
                // }
                // delay(5);
            }
            else
            {
            }
        }
    }
}

void setup()
{
    Serial.begin(115200);

    pinMode(0, INPUT_PULLUP);

    pinMode(RT9080_EN, OUTPUT);
    digitalWrite(RT9080_EN, HIGH);

    pinMode(MAX98357A_EN, OUTPUT);
    digitalWrite(MAX98357A_EN, HIGH);

    attachInterrupt(
        0,
        []
        {
            Boot_Key_Flag = true;
        },
        FALLING); // Triggered every 1ms

    pinMode(MP34DT05TR_EN, OUTPUT);
    digitalWrite(MP34DT05TR_EN, LOW);

    // 单声道采集麦克风数据
    while (IIS->begin(I2S_MODE_PDM, AD_IIS_DATA_IN, I2S_CHANNEL_FMT_ONLY_LEFT,
                      IIS_DATA_BIT, IIS_SAMPLE_RATE) == false)
    {
        Serial.println("MP34DT05TR initialization fail");
        delay(2000);
    }
    Serial.println("MP34DT05TR initialization successfully");

    // 扬声器左和右一半所以输出双声道
    while (MAX98357A->begin(I2S_MODE_MASTER, AD_IIS_DATA_OUT, I2S_CHANNEL_FMT_RIGHT_LEFT,
                            IIS_DATA_BIT, IIS_SAMPLE_RATE) == false)
    {
        Serial.println("MAX98357A initialization fail");
        delay(2000);
    }
    Serial.println("MAX98357A initialization successfully");

    SPI.begin(LR1121_SCLK, LR1121_MISO, LR1121_MOSI);

    // set RF switch control configuration
    // this has to be done prior to calling begin()
    radio.setRfSwitchTable(rfswitch_dio_pins, rfswitch_table);

    // initialize LR1121 with default settings
    Serial.println("[LR1121] Initializing ... ");

    // int16_t state = radio.begin();
    int16_t state = radio.beginGFSK();
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

    Lora_Configuration_Default_Value(LORA_2_4GHZ);

    radio.setPacketReceivedAction(Lora_Transmission_Interrupt);

    xTaskCreate(&Codec2_Task, "Codec2_Task", 32000, NULL, 5, NULL);

    xTaskCreate(&MAX_Play_Task, "MAX_Play_Task", 30000, NULL, 5, NULL);

    radio.startReceive();
}

void loop()
{
    if (Boot_Key_Flag == true)
    {
        delay(500);

        Lora_Transmission_Mode = !Lora_Transmission_Mode;
        if (Lora_Transmission_Mode == 0)
        {
            radio.startReceive();
        }
        IIS_Read_Data_Stream.clear();
        IIS_Write_Data_Stream.clear();
        Lora_Send_Data_Stream.clear();
        Lora_Receive_Data_Stream.clear();

        Boot_Key_Flag = false;
    }

    if (Lora_Send_Data_Stream.size() >= LORA_TRANSMISSION_DATA_SIZE) // 流读取判断
    {
        // 存储数据
        memcpy(&Send_Package[LORA_TRANSMISSION_HEAD_SIZE], Lora_Send_Data_Stream.data(),
               LORA_TRANSMISSION_DATA_SIZE);

        // 删除已经存储的数据
        Lora_Send_Data_Stream.erase(Lora_Send_Data_Stream.begin(),
                                    Lora_Send_Data_Stream.begin() + LORA_TRANSMISSION_DATA_SIZE);

        Serial.println("[LR1121] Sending another packet ... ");

        radio.transmit(Send_Package,
                       LORA_TRANSMISSION_HEAD_SIZE + LORA_TRANSMISSION_DATA_SIZE);
    }

    // check if the previous operation finished
    if (Lora_Transmission_Flag)
    {
        Lora_Transmission_Flag = false;

        uint8_t receive_package[LORA_TRANSMISSION_HEAD_SIZE + LORA_TRANSMISSION_DATA_SIZE] = {0};

        if (radio.readData(receive_package,
                           LORA_TRANSMISSION_HEAD_SIZE + LORA_TRANSMISSION_DATA_SIZE) == RADIOLIB_ERR_NONE)
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
                    // packet was successfully received
                    Serial.println("[LR1121] Received packet!");

                    // print data of the packet
                    // for (int i = 0; i < LORA_TRANSMISSION_HEAD_SIZE; i++)
                    // {
                    //     Serial.printf("receive_package[%d]: %#X\n", i, receive_package[i]);
                    // }

                    // for (int i = 0; i < LORA_TRANSMISSION_HEAD_SIZE; i++)
                    // {
                    //     Serial.printf("Send_Package[%d]: %#X\n", i, Send_Package[i]);
                    // }

                    // print RSSI (Received Signal Strength Indicator)
                    Serial.print("[LR1121] RSSI:\t\t");
                    Serial.print(radio.getRSSI());
                    Serial.println(" dBm");

                    // print SNR (Signal-to-Noise Ratio)
                    Serial.print("[LR1121] SNR:\t\t");
                    Serial.print(radio.getSNR());
                    Serial.println(" dB");
                }

                // 写入Lora流中
                const auto current_buf_size = Lora_Receive_Data_Stream.size();

                // 调整容量
                Lora_Receive_Data_Stream.resize(current_buf_size + LORA_TRANSMISSION_DATA_SIZE);

                // 存储数据
                memcpy(Lora_Receive_Data_Stream.data() + current_buf_size,
                       &receive_package[LORA_TRANSMISSION_HEAD_SIZE], LORA_TRANSMISSION_DATA_SIZE);
            }
        }
    }

    // Serial.printf("IIS_Read_Data_Stream Size: %d\n", IIS_Read_Data_Stream.size());
    // Serial.printf("IIS_Write_Data_Stream Size: %d\n", IIS_Write_Data_Stream.size());
    // Serial.printf("Lora_Send_Data_Stream Size: %d\n", Lora_Send_Data_Stream.size());
    // Serial.printf("Lora_Receive_Data_Stream Size: %d\n", Lora_Receive_Data_Stream.size());
    delay(2); // xTask time
}

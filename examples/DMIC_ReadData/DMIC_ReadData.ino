/*
 * @Description: DMIC_ReadData test
 * @Author: LILYGO_L
 * @Date: 2023-12-20 16:24:06
 * @LastEditTime: 2025-02-06 10:33:12
 * @License: GPL 3.0
 */
#include "Arduino_DriveBus_Library.h"
#include "pin_config.h"

#define IIS_SAMPLE_RATE 44100 // 采样速率
#define IIS_DATA_BIT 16       // 数据位数

std::shared_ptr<Arduino_IIS_DriveBus> IIS_Bus =
    std::make_shared<Arduino_HWIIS>(I2S_NUM_0, -1, MP34DT05TR_LRCLK, MP34DT05TR_DATA);

std::unique_ptr<Arduino_IIS> IIS(new Arduino_MEMS(IIS_Bus));

short IIS_Read_Buff[100];

void setup()
{
    Serial.begin(115200);

    pinMode(RT9080_EN, OUTPUT);
    digitalWrite(RT9080_EN, HIGH);

    pinMode(MP34DT05TR_EN, OUTPUT);
    digitalWrite(MP34DT05TR_EN, LOW);

    while (IIS->begin(I2S_MODE_PDM, AD_IIS_DATA_IN, I2S_CHANNEL_FMT_RIGHT_LEFT,
                      IIS_DATA_BIT, IIS_SAMPLE_RATE) == false)
    {
        Serial.println("MP34DT05TR initialization fail");
        delay(2000);
    }
    Serial.println("MP34DT05TR initialization successfully");
}

void loop()
{
    if (IIS->IIS_Read_Data(IIS_Read_Buff, sizeof(IIS_Read_Buff)) == true)
    {
        // 输出左右声道数据
        Serial.printf("Left: %d\n", IIS_Read_Buff[0]);
        Serial.printf("Right: %d\n", IIS_Read_Buff[1]);

        // Arduino
        // Serial.println(IIS_Read_Buff[0]);
        // Serial.print(",");
        // Serial.print(IIS_Read_Buff[1]);

        // for (int i = 0; i < 25; i++)
        // {
        //     Serial.printf("debug[%d]: %d\n", i, IIS_Read_Buff[i]);
        // }
    }
    else
    {
        Serial.printf("Failed to read iis data");
    }

    delay(50);
}

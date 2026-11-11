#include <Arduino.h>
#include "WS2812.h"

// 配置
#define LED_PIN 10
#define LED_COUNT 64
MyWS2812 strip(LED_COUNT, LED_PIN);

// 彩虹配色辅助函数
uint32_t Wheel(byte WheelPos)
{
    WheelPos = 255 - WheelPos;
    if (WheelPos < 85)
    {
        return strip.Color(255 - WheelPos * 3, 0, WheelPos * 3);
    }
    if (WheelPos < 170)
    {
        WheelPos -= 85;
        return strip.Color(0, WheelPos * 3, 255 - WheelPos * 3);
    }
    WheelPos -= 170;
    return strip.Color(WheelPos * 3, 255 - WheelPos * 3, 0);
}

void rainbowDemo()
{
    static uint16_t j = 0;
    for (uint16_t i = 0; i < LED_COUNT; i++)
    {
        strip.setRawColor(i, (Wheel((i + j) & 255) >> 16) & 0xFF,
                          (Wheel((i + j) & 255) >> 8) & 0xFF,
                          Wheel((i + j) & 255) & 0xFF);
    }
    j += 2;
    strip.show();
}

void clearAll()
{
    strip.setAllRawColor(0, 0, 0);
    strip.show();
}

void lightOne(uint16_t idx, uint8_t r = 255, uint8_t g = 255, uint8_t b = 255)
{
    strip.setAllRawColor(0, 0, 0);
    if (idx < LED_COUNT)
    {
        strip.setRawColor(idx, r, g, b);
    }
    strip.show();
}

void setup()
{
    Serial.begin(115200);
    Serial.println("===== MyWS2812 Non-block Demo =====");
    Serial.println("cmd:");
    Serial.println("  c = clear");
    Serial.println("  r = rainbow");
    Serial.println("  0~63 = light single led");
    strip.begin();
    clearAll();
}

void loop()
{
    // 【必须调用】后台发送状态机，非阻塞核心
    strip.service();

    // 串口命令解析
    if (Serial.available() > 0)
    {
        char ch = Serial.read();
        if (ch == 'c')
        {
            clearAll();
            Serial.println("-> clear all");
        }
        else if (ch == 'r')
        {
            Serial.println("-> rainbow start");
        }
        else if (ch >= '0' && ch <= '9')
        {
            uint16_t num = ch - '0';
            lightOne(num, 0, 255, 0);
            Serial.printf("-> light idx %d\n", num);
        }
    }

    // 注释掉下面这行，就不会自动跑彩虹，只响应串口命令
    // rainbowDemo();

    delay(10);
}

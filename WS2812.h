#ifndef __WS2812_H__
#define __WS2812_H__

#include <Arduino.h>

/**
 * MyWS2812 —— ESP32 全系列通用 WS2812(NeoPixel) 驱动
 *
 * 适用芯片：ESP32 / ESP32-S2 / ESP32-S3 / ESP32-C3 / ESP32-C6 / ESP32-H2 等
 * 适用框架：Arduino-ESP32 2.x（IDF 4.x 旧 RMT 驱动）与 3.x（IDF 5.x 新 RMT 驱动）
 *
 * 实现要点：
 *  - 用 RMT 硬件外设产生精确 800kHz 波形，不依赖 CPU 空转、不受任务切换/中断影响
 *  - 时钟分频与位时序按实际 APB 频率动态计算，不写死 tick 数
 *  - RMT 内存块数量按芯片能力宏取值，兼容只有 2 个 TX 通道的 C3
 *  - 任意 GPIO（含 S3/C3 的 32~48 号）都可用，走 GPIO 矩阵，不碰寄存器
 *
 * 用法：
 *   MyWS2812 strip(64, 48);        // 64 颗，GPIO48
 *   strip.begin();
 *   strip.setAllRawColor(255, 0, 0);
 *   strip.show();                  // 非阻塞，立即返回
 *
 * 多灯带时可用第三个参数指定不同的 RMT 通道：
 *   MyWS2812 strip2(30, 5, 1);
 */
class MyWS2812
{
public:
    MyWS2812(uint16_t numPixels, uint8_t pin, uint8_t rmtChannel = 0);
    ~MyWS2812();

    // 初始化 RMT 通道与发送任务，可重复调用
    void begin();
    // 提交当前颜色，非阻塞
    void show();
    // 兼容旧接口：发送已由内部任务完成，无需再调用
    void service();

    bool isBusy();
    bool hasPendingFrame();
    // 初始化是否成功
    bool isReady() const { return _inited; }

    void clear();

    void setPixelColor(uint16_t index, uint8_t r, uint8_t g, uint8_t b);
    void setPixelColor(uint16_t index, uint32_t color);
    uint32_t getPixelColor(uint16_t index);
    uint16_t numPixels() const { return _pixelCount; }
    uint8_t channel() const { return _channel; }

    static uint32_t Color(uint8_t r, uint8_t g, uint8_t b);

    void setRawColor(uint16_t index, uint8_t r, uint8_t g, uint8_t b);
    void setBrightness(uint16_t index, uint8_t br);
    void setGlobalBrightness(uint8_t br);
    void setAllRawColor(uint8_t r, uint8_t g, uint8_t b);
    void setAllBrightness(uint8_t br);
    uint8_t getGlobalBrightness() { return _globalBright; }

private:
    uint16_t _pixelCount;
    uint8_t _pin;
    uint8_t _channel;
    uint8_t _globalBright;

    uint8_t *_rawR;
    uint8_t *_rawG;
    uint8_t *_rawB;
    uint8_t *_pixelBr;

    uint8_t *_frame;   // 上层写入的显示缓冲（GRB）
    uint8_t *_sendBuf; // 发送缓冲，发送任务独占
    void *_items;      // RMT 符号数组，长度 _pixelCount * 24

    uint16_t _clkDiv; // RMT 时钟分频
    uint32_t _sym1;   // "1" 的 RMT 符号（32 位打包）
    uint32_t _sym0;   // "0" 的 RMT 符号（32 位打包）

    void *_rmtChan;  // IDF5: rmt_channel_handle_t
    void *_rmtEnc;   // IDF5: rmt_encoder_handle_t

    bool _inited;
    volatile bool _pending;
    volatile bool _sending;
    SemaphoreHandle_t _lock;
    TaskHandle_t _task;

    bool hwInit();
    void hwDeinit();
    void hwTransmit();
    void computeTiming();

    static void taskEntry(void *arg);
    void txLoop();
    void calcBuffer(uint8_t *dst);
    void buildItems();
};

#endif

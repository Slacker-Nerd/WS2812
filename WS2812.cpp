#include <Arduino.h>
#include "WS2812.h"

#include <soc/soc_caps.h>

/* ==================================================================
 * RMT 驱动后端选择
 *   IDF 5.x（Arduino-ESP32 3.x）：新驱动 driver/rmt_tx.h
 *   IDF 4.x（Arduino-ESP32 2.x）：旧驱动 driver/rmt.h
 * ================================================================== */
#if __has_include(<driver/rmt_tx.h>)
#define WS_RMT_V2 1
#include <driver/rmt_tx.h>
#include <driver/rmt_encoder.h>
#if __has_include(<driver/rmt_common.h>)
#include <driver/rmt_common.h>
#endif
#if __has_include(<driver/rmt_simple_encoder.h>)
#include <driver/rmt_simple_encoder.h>
#endif
#else
#define WS_RMT_V2 0
#include <driver/rmt.h>
#endif

/* ---- 芯片能力宏兜底（不同型号数值不同，必须按宏取值） ---- */
#ifndef SOC_RMT_MEM_WORDS_PER_CHANNEL
#define SOC_RMT_MEM_WORDS_PER_CHANNEL 64
#endif
#ifndef SOC_RMT_CHANNELS_PER_GROUP
#define SOC_RMT_CHANNELS_PER_GROUP 8
#endif
#ifndef SOC_RMT_TX_CANDIDATES_PER_GROUP
#define SOC_RMT_TX_CANDIDATES_PER_GROUP SOC_RMT_CHANNELS_PER_GROUP
#endif

/* ---- 时序常量 ---- */
#define WS_TICK_HZ 20000000UL // 目标 tick 频率 20MHz → 50ns
#define WS_T1H_NS 800         // 逻辑 1 高电平
#define WS_T1L_NS 450         // 逻辑 1 低电平
#define WS_T0H_NS 400         // 逻辑 0 高电平
#define WS_T0L_NS 850         // 逻辑 0 低电平
#define WS_RESET_US 300       // 帧末低电平，保证锁存（要求 >50us）
#define WS_TASK_STACK 4096

// RMT 符号 32 位布局：duration0(bit0-14) level0(bit15) duration1(bit16-30) level1(bit31)
// 新旧两代驱动的符号结构位域一致，可直接用同一个 32 位值
#define WS_SYM(high, low) ((uint32_t)(1u << 15) | (uint32_t)(high) | ((uint32_t)(low) << 16))

static uint16_t nsToTicks(uint32_t tickHz, uint32_t ns)
{
    uint64_t t = ((uint64_t)ns * tickHz + 500000000ULL) / 1000000000ULL;
    if (t < 1)
    {
        t = 1;
    }
    if (t > 32767)
    {
        t = 32767;
    }
    return (uint16_t)t;
}

/* ============================ 构造/析构 ============================ */

MyWS2812::MyWS2812(uint16_t numPixels, uint8_t pin, uint8_t rmtChannel)
{
    _pixelCount = numPixels;
    _pin = pin;
    _channel = rmtChannel;
    _globalBright = 255;

    _rawR = new uint8_t[_pixelCount];
    _rawG = new uint8_t[_pixelCount];
    _rawB = new uint8_t[_pixelCount];
    _pixelBr = new uint8_t[_pixelCount];

    _frame = new uint8_t[_pixelCount * 3];
    _sendBuf = new uint8_t[_pixelCount * 3];
    _items = nullptr;

    _clkDiv = 4;
    _sym1 = 0;
    _sym0 = 0;
    _rmtChan = nullptr;
    _rmtEnc = nullptr;

    _inited = false;
    _pending = false;
    _sending = false;
    _lock = nullptr;
    _task = nullptr;
}

MyWS2812::~MyWS2812()
{
    if (_task != nullptr)
    {
        vTaskDelete(_task);
        _task = nullptr;
    }
    hwDeinit();
    if (_lock != nullptr)
    {
        vSemaphoreDelete(_lock);
        _lock = nullptr;
    }
    delete[] _rawR;
    delete[] _rawG;
    delete[] _rawB;
    delete[] _pixelBr;
    delete[] _frame;
    delete[] _sendBuf;
    if (_items != nullptr)
    {
#if WS_RMT_V2
        delete[](rmt_symbol_word_t *) _items;
#else
        delete[](rmt_item32_t *) _items;
#endif
        _items = nullptr;
    }
}

/* ============================ 时序计算 ============================ */

void MyWS2812::computeTiming()
{
    uint32_t apb = getApbFrequency();
    if (apb == 0)
    {
        apb = 80000000UL;
    }

    uint32_t div = apb / WS_TICK_HZ;
    if (div < 1)
    {
        div = 1;
    }
    if (div > 255)
    {
        div = 255;
    }
    _clkDiv = (uint16_t)div;

    uint32_t tickHz = apb / _clkDiv;

    uint16_t t1h = nsToTicks(tickHz, WS_T1H_NS);
    uint16_t t1l = nsToTicks(tickHz, WS_T1L_NS);
    uint16_t t0h = nsToTicks(tickHz, WS_T0H_NS);
    uint16_t t0l = nsToTicks(tickHz, WS_T0L_NS);

    _sym1 = WS_SYM(t1h, t1l);
    _sym0 = WS_SYM(t0h, t0l);
}

/* ============================ 硬件初始化 ============================ */

bool MyWS2812::hwInit()
{
    // 通道号按芯片能力校验（C3 只有 2 个 TX，S2/S3 有 4 个，ESP32 有 8 个）
    if (_channel >= SOC_RMT_TX_CANDIDATES_PER_GROUP)
    {
        Serial.printf("[WS2812] RMT 通道 %u 不可用（本芯片 TX 通道数 %u）\n",
                      _channel, (unsigned)SOC_RMT_TX_CANDIDATES_PER_GROUP);
        return false;
    }

#if WS_RMT_V2
    rmt_tx_channel_config_t cfg = {};
    cfg.gpio_num = (gpio_num_t)_pin;
    cfg.clk_src = RMT_CLK_SRC_DEFAULT;
    cfg.resolution_hz = getApbFrequency() / (_clkDiv ? _clkDiv : 1);
    cfg.mem_block_symbols = SOC_RMT_MEM_WORDS_PER_CHANNEL; // 按芯片实际块大小申请
    cfg.trans_queue_depth = 4;
    cfg.intr_priority = 0;

    if (rmt_new_tx_channel(&cfg, (rmt_channel_handle_t *)&_rmtChan) != ESP_OK)
    {
        Serial.printf("[WS2812] RMT 发送通道创建失败（GPIO %u）\n", _pin);
        _rmtChan = nullptr;
        return false;
    }

    rmt_copy_encoder_config_t encCfg = {};
    if (rmt_new_copy_encoder(&encCfg, (rmt_encoder_handle_t *)&_rmtEnc) != ESP_OK)
    {
        Serial.println("[WS2812] RMT 拷贝编码器创建失败");
        rmt_del_channel((rmt_channel_handle_t)_rmtChan);
        _rmtChan = nullptr;
        return false;
    }

    if (rmt_enable((rmt_channel_handle_t)_rmtChan) != ESP_OK)
    {
        Serial.println("[WS2812] RMT 通道使能失败");
        rmt_del_encoder((rmt_encoder_handle_t)_rmtEnc);
        rmt_del_channel((rmt_channel_handle_t)_rmtChan);
        _rmtEnc = nullptr;
        _rmtChan = nullptr;
        return false;
    }
#else
    rmt_config_t cfg = {};
    cfg.rmt_mode = RMT_MODE_TX;
    cfg.channel = (rmt_channel_t)_channel;
    cfg.gpio_num = (gpio_num_t)_pin;
    cfg.clk_div = (uint8_t)_clkDiv;
    // 只申请 1 个内存块：C3 一共只有 2 块，申请多了会失败；rmt_write_items 会自动续发
    cfg.mem_block_num = 1;
    cfg.flags = 0;
    cfg.tx_config.carrier_en = false;
    cfg.tx_config.carrier_freq_hz = 38000;
    cfg.tx_config.carrier_level = RMT_CARRIER_LEVEL_HIGH;
    cfg.tx_config.carrier_duty_percent = 33;
    cfg.tx_config.loop_en = false;
    cfg.tx_config.idle_level = RMT_IDLE_LEVEL_LOW;
    cfg.tx_config.idle_output_en = true;

    if (rmt_config(&cfg) != ESP_OK)
    {
        Serial.printf("[WS2812] RMT 配置失败（GPIO %u，通道 %u）\n", _pin, _channel);
        return false;
    }
    if (rmt_driver_install((rmt_channel_t)_channel, 0, 0) != ESP_OK)
    {
        Serial.println("[WS2812] RMT 驱动安装失败");
        return false;
    }
#endif

    return true;
}

void MyWS2812::hwDeinit()
{
#if WS_RMT_V2
    if (_rmtChan != nullptr)
    {
        rmt_disable((rmt_channel_handle_t)_rmtChan);
        if (_rmtEnc != nullptr)
        {
            rmt_del_encoder((rmt_encoder_handle_t)_rmtEnc);
            _rmtEnc = nullptr;
        }
        rmt_del_channel((rmt_channel_handle_t)_rmtChan);
        _rmtChan = nullptr;
    }
#else
    if (_inited)
    {
        rmt_driver_uninstall((rmt_channel_t)_channel);
    }
#endif
}

void MyWS2812::hwTransmit()
{
#if WS_RMT_V2
    rmt_transmit_config_t txCfg = {};
    txCfg.loop_count = 0;
    rmt_transmit((rmt_channel_handle_t)_rmtChan,
                 (rmt_encoder_handle_t)_rmtEnc,
                 _items,
                 (size_t)(_pixelCount * 24) * sizeof(rmt_symbol_word_t),
                 &txCfg);
    rmt_tx_wait_all_done((rmt_channel_handle_t)_rmtChan, 1000);
#else
    rmt_write_items((rmt_channel_t)_channel,
                    (rmt_item32_t *)_items,
                    (int)(_pixelCount * 24),
                    true);
#endif
}

/* ============================ 对外接口 ============================ */

void MyWS2812::begin()
{
    if (_inited)
    {
        return;
    }

    pinMode(_pin, OUTPUT);
    digitalWrite(_pin, LOW);

    for (uint16_t i = 0; i < _pixelCount; i++)
    {
        _rawR[i] = 0;
        _rawG[i] = 0;
        _rawB[i] = 0;
        _pixelBr[i] = 255;
    }
    memset(_frame, 0, _pixelCount * 3);
    memset(_sendBuf, 0, _pixelCount * 3);

#if WS_RMT_V2
    _items = (void *)new rmt_symbol_word_t[_pixelCount * 24];
    const char *driverName = "IDF5 新 RMT 驱动";
#else
    _items = (void *)new rmt_item32_t[_pixelCount * 24];
    const char *driverName = "IDF4 旧 RMT 驱动";
#endif
    if (_items == nullptr)
    {
        Serial.println("[WS2812] RMT 符号缓冲申请失败");
        return;
    }

    _lock = xSemaphoreCreateMutex();
    if (_lock == nullptr)
    {
        Serial.println("[WS2812] 互斥量创建失败");
        return;
    }

    computeTiming();

    if (!hwInit())
    {
        return;
    }

    _inited = true;

    if (xTaskCreate(MyWS2812::taskEntry, "ws2812_tx", WS_TASK_STACK, this, 4, &_task) != pdPASS)
    {
        _task = nullptr;
        Serial.println("[WS2812] 发送任务创建失败");
        hwDeinit();
        _inited = false;
        return;
    }

    Serial.printf("[WS2812] 就绪：%s / GPIO %u / %u 颗 / 通道 %u / clkDiv %u / 驱动 %s\n",
                  ESP.getChipModel(), _pin, _pixelCount, _channel, _clkDiv, driverName);
}

void MyWS2812::taskEntry(void *arg)
{
    static_cast<MyWS2812 *>(arg)->txLoop();
}

void MyWS2812::txLoop()
{
    for (;;)
    {
        // 等新帧通知；超时也回头检查一次，避免漏帧
        ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(200));

        while (_pending)
        {
            xSemaphoreTake(_lock, portMAX_DELAY);
            memcpy(_sendBuf, _frame, _pixelCount * 3);
            _pending = false;
            xSemaphoreGive(_lock);

            buildItems();

            _sending = true;
            hwTransmit();
            _sending = false;

            // 帧末保持低电平，让灯珠锁存
            delayMicroseconds(WS_RESET_US);
        }
    }
}

void MyWS2812::buildItems()
{
#if WS_RMT_V2
    rmt_symbol_word_t *items = (rmt_symbol_word_t *)_items;
#else
    rmt_item32_t *items = (rmt_item32_t *)_items;
#endif

    uint32_t idx = 0;
    for (uint16_t i = 0; i < _pixelCount; i++)
    {
        for (uint8_t byteIndex = 0; byteIndex < 3; byteIndex++)
        {
            uint8_t value = _sendBuf[i * 3 + byteIndex];
            for (int bit = 7; bit >= 0; bit--)
            {
                items[idx].val = (value & (1u << bit)) ? _sym1 : _sym0;
                idx++;
            }
        }
    }
}

void MyWS2812::calcBuffer(uint8_t *dst)
{
    for (uint16_t i = 0; i < _pixelCount; i++)
    {
        uint8_t br = (uint16_t)_pixelBr[i] * _globalBright / 255;
        dst[i * 3 + 0] = (uint16_t)_rawG[i] * br / 255; // G
        dst[i * 3 + 1] = (uint16_t)_rawR[i] * br / 255; // R
        dst[i * 3 + 2] = (uint16_t)_rawB[i] * br / 255; // B
    }
}

void MyWS2812::show()
{
    if (!_inited || _lock == nullptr)
    {
        return;
    }

    xSemaphoreTake(_lock, portMAX_DELAY);
    calcBuffer(_frame);
    _pending = true;
    xSemaphoreGive(_lock);

    if (_task != nullptr)
    {
        xTaskNotifyGive(_task);
    }
}

void MyWS2812::service()
{
    // 发送已由内部任务完成，保留该接口仅为兼容老代码
}

bool MyWS2812::isBusy()
{
    return _pending || _sending;
}

bool MyWS2812::hasPendingFrame()
{
    return _pending;
}

void MyWS2812::clear()
{
    for (uint16_t i = 0; i < _pixelCount; i++)
    {
        _rawR[i] = 0;
        _rawG[i] = 0;
        _rawB[i] = 0;
    }
}

uint32_t MyWS2812::Color(uint8_t r, uint8_t g, uint8_t b)
{
    return ((uint32_t)g << 16) | ((uint32_t)r << 8) | b;
}

void MyWS2812::setPixelColor(uint16_t index, uint8_t r, uint8_t g, uint8_t b)
{
    if (index >= _pixelCount)
    {
        return;
    }
    _rawR[index] = r;
    _rawG[index] = g;
    _rawB[index] = b;
}

void MyWS2812::setPixelColor(uint16_t index, uint32_t color)
{
    if (index >= _pixelCount)
    {
        return;
    }
    _rawR[index] = (color >> 8) & 0xFF;
    _rawG[index] = (color >> 16) & 0xFF;
    _rawB[index] = color & 0xFF;
}

uint32_t MyWS2812::getPixelColor(uint16_t index)
{
    if (index >= _pixelCount)
    {
        return 0;
    }
    return Color(_rawR[index], _rawG[index], _rawB[index]);
}

void MyWS2812::setRawColor(uint16_t index, uint8_t r, uint8_t g, uint8_t b)
{
    if (index >= _pixelCount)
    {
        return;
    }
    _rawR[index] = r;
    _rawG[index] = g;
    _rawB[index] = b;
}

void MyWS2812::setBrightness(uint16_t index, uint8_t br)
{
    if (index >= _pixelCount)
    {
        return;
    }
    _pixelBr[index] = br;
}

void MyWS2812::setGlobalBrightness(uint8_t br)
{
    _globalBright = br;
}

void MyWS2812::setAllRawColor(uint8_t r, uint8_t g, uint8_t b)
{
    for (uint16_t i = 0; i < _pixelCount; i++)
    {
        _rawR[i] = r;
        _rawG[i] = g;
        _rawB[i] = b;
    }
}

void MyWS2812::setAllBrightness(uint8_t br)
{
    for (uint16_t i = 0; i < _pixelCount; i++)
    {
        _pixelBr[i] = br;
    }
}

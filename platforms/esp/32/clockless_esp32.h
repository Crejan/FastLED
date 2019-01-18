/*
 * Integration into FastLED ClocklessController
 * Copyright (c) 2018 Samuel Z. Guyer
 * Copyright (c) 2017 Thomas Basler
 * Copyright (c) 2017 Martin F. Falatic
 *
 * ESP32 support is provided using the RMT peripheral device -- a unit
 * on the chip designed specifically for generating (and receiving)
 * precisely-timed digital signals. Nominally for use in infrared
 * remote controls, we use it to generate the signals for clockless
 * LED strips. The main advantage of using the RMT device is that,
 * once programmed, it generates the signal asynchronously, allowing
 * the CPU to continue executing other code. It is also not vulnerable
 * to interrupts or other timing problems that could disrupt the signal.
 *
 * The implementation strategy is borrowed from previous work and from
 * the RMT support built into the ESP32 IDF. The RMT device has 8
 * channels, which can be programmed independently to send sequences
 * of high/low bits. Memory for each channel is limited, however, so
 * in order to send a long sequence of bits, we need to continuously
 * refill the buffer until all the data is sent. To do this, we fill
 * half the buffer and then set an interrupt to go off when that half
 * is sent. Then we refill that half while the second half is being
 * sent. This strategy effectively overlaps computation (by the CPU)
 * and communication (by the RMT).
 *
 * Since the RMT device only has 8 channels, we need a strategy to
 * allow more than 8 LED controllers. Our driver assigns controllers
 * to channels on the fly, queuing up controllers as necessary until a
 * channel is free. The main showPixels routine just fires off the
 * first 8 controllers; the interrupt handler starts new controllers
 * asynchronously as previous ones finish. So, for example, it can
 * send the data for 8 controllers simultaneously, but 16 controllers
 * would take approximately twice as much time.
 *
 * There is a #define that allows a program to control the total
 * number of channels that the driver is allowed to use. It defaults
 * to 8 -- use all the channels. Setting it to 1, for example, results
 * in fully serial output:
 *
 *     #define FASTLED_RMT_MAX_CHANNELS 1
 *
 * OTHER RMT APPLICATIONS
 *
 * The default FastLED driver takes over control of the RMT interrupt
 * handler, making it hard to use the RMT device for other
 * (non-FastLED) purposes. You can change it's behavior to use the ESP
 * core driver instead, allowing other RMT applications to
 * co-exist. To switch to this mode, add the following directive
 * before you include FastLED.h:
 *
 *      #define FASTLED_RMT_BUILTIN_DRIVER
 *
 * There may be a performance penalty for using this mode. We need to
 * compute the RMT signal for the entire LED strip ahead of time,
 * rather than overlapping it with communication. We also need a large
 * buffer to hold the signal specification. Each bit of pixel data is
 * represented by a 32-bit pulse specification, so it is a 32X blow-up
 * in memory use.
 *
 *
 * Based on public domain code created 19 Nov 2016 by Chris Osborn <fozztexx@fozztexx.com>
 * http://insentricity.com *
 *
 */
/*
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

#pragma once

FASTLED_NAMESPACE_BEGIN

#ifdef __cplusplus
extern "C" {
#endif

#include <SPI.h>

#ifdef __cplusplus
}
#endif

#define FASTLED_HAS_CLOCKLESS 1


template<int DATA_PIN, int T1, int T2, int T3, EOrder RGB_ORDER = RGB, int XTRA0 = 0, bool FLIP = false, int WAIT_TIME = 5>
class ClocklessController : public CPixelLEDController<RGB_ORDER> {

    SPIClass * hspi = nullptr;
    uint8_t *led_data = nullptr;
    // H:875 L:375
    static const uint8_t mOne = 0xf8;
    // H:250 L:1000
    static const uint8_t mZero = 0x80;
    int mSize = 0;

public:

    void init() {
        hspi = new SPIClass(HSPI);
        hspi->begin();
    }

    virtual uint16_t getMaxRefreshRate() const {
        return 400;
    }

protected:

    virtual void showPixels(PixelController<RGB_ORDER> &pixels) { ;
        const int RESET_LENGTH = 10;
        int size_needed = pixels.size() * 3 * 8  + RESET_LENGTH;
        if (size_needed != mSize) {
            if (led_data != nullptr)
                free(led_data);
            mSize = size_needed;
            led_data = (uint8_t*) malloc(mSize * sizeof(rmt_data_t));
            for(int i=size_needed-RESET_LENGTH; i<size_needed; ++i){
                led_data[i] = 0;
            }
        }

        int cur = 0;
        while (pixels.has(1)) {
            uint8_t byte = pixels.loadAndScale0();
            led_data[cur++] = byte & 0x80 ? mOne : mZero;
            led_data[cur++] = byte & 0x40 ? mOne : mZero;
            led_data[cur++] = byte & 0x20 ? mOne : mZero;
            led_data[cur++] = byte & 0x10 ? mOne : mZero;
            led_data[cur++] = byte & 0x08 ? mOne : mZero;
            led_data[cur++] = byte & 0x04 ? mOne : mZero;
            led_data[cur++] = byte & 0x02 ? mOne : mZero;
            led_data[cur++] = byte & 0x01 ? mOne : mZero;
            byte = pixels.loadAndScale1();
            led_data[cur++] = byte & 0x80 ? mOne : mZero;
            led_data[cur++] = byte & 0x40 ? mOne : mZero;
            led_data[cur++] = byte & 0x20 ? mOne : mZero;
            led_data[cur++] = byte & 0x10 ? mOne : mZero;
            led_data[cur++] = byte & 0x08 ? mOne : mZero;
            led_data[cur++] = byte & 0x04 ? mOne : mZero;
            led_data[cur++] = byte & 0x02 ? mOne : mZero;
            led_data[cur++] = byte & 0x01 ? mOne : mZero;
            byte = pixels.loadAndScale2();
            led_data[cur++] = byte & 0x80 ? mOne : mZero;
            led_data[cur++] = byte & 0x40 ? mOne : mZero;
            led_data[cur++] = byte & 0x20 ? mOne : mZero;
            led_data[cur++] = byte & 0x10 ? mOne : mZero;
            led_data[cur++] = byte & 0x08 ? mOne : mZero;
            led_data[cur++] = byte & 0x04 ? mOne : mZero;
            led_data[cur++] = byte & 0x02 ? mOne : mZero;
            led_data[cur++] = byte & 0x01 ? mOne : mZero;
            pixels.advanceData();
            pixels.stepDithering();
        }
        const SPISettings settings(6400000, MSBFIRST, SPI_MODE0);
        hspi->beginTransaction(settings);
        hspi->writeBytes(led_data, cur);
        hspi->endTransaction();
    }

};

FASTLED_NAMESPACE_END

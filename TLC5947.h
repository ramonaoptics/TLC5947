/*
   TLC5947 Control Library
   Used to control the TI TLC5947 LED driver chip
   Zack Phillips - zkphil@berkeley.edu
   Product Page: http://www.ti.com/product/tlc5947

   Copyright (c) 2015, Zachary F. Phillips
   All rights reserved.

   Redistribution and use in source and binary forms, with or without
   modification, are permitted provided that the following conditions are met:
 * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.
 * Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in the
      documentation and/or other materials provided with the distribution.
 * Neither the name of Zack Phillips / UC Berkeley nor the
      names of its contributors may be used to endorse or promote products
      derived from this software without specific prior written permission.

   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
   ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
   WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
   DISCLAIMED. IN NO EVENT SHALL Z. PHILLIPS BE LIABLE FOR ANY
   DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
   (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
   LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
   ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
   (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
   SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include <math.h>

#ifndef TLC5947_H
#define TLC5947_H

//#include <Arduino.h>
#include <stdint.h>
#include <SPI.h>

class TLC5947
{
public:

/* Initialization */
void init(int8_t num_latches, int8_t num_tlc_one_row, uint8_t use_2D,
          uint8_t spi_mosi, uint8_t spi_clk, uint8_t blank);
void deallocate();

/* Setting individual LED intensities */
void setAllLed(uint16_t gsvalue);
void setAllLedRgb(uint16_t red, uint16_t green, uint16_t blue);
void setLed(uint16_t led_number, uint16_t red, uint16_t green, uint16_t blue);
void setLed(uint16_t led_number, uint16_t rgb);
void setLedAppend(uint16_t led_number, uint16_t red, uint16_t green, uint16_t blue);
void setChannel(uint16_t channel_number, uint16_t value);

/* Get LED Intensities */
uint16_t getChannelValue(uint16_t channelNum, int color_channel_index);
uint16_t getLEDValuePerChip(uint16_t chip, int led_number);

/* Control Mode Parameters */
void setBrightnessCurrent(uint8_t global);
void setBrightnessCurrent(uint8_t red, uint8_t green, uint8_t blue);

void setRgbPinOrder(uint8_t rPos, uint8_t grPos, uint8_t bPos);
void setPinOrderSingle(uint16_t channel, uint8_t color_channel_index, uint8_t position);
void setRgbPinOrderSingle(uint16_t channel, uint8_t rPos, uint8_t grPos, uint8_t bPos);

int updateLeds(double* output_current);
void clearLeds();
void latch(int latch_index);
void setSpiBaudRate(uint32_t new_baud_rate);
uint32_t getSpiBaudRate();

// illuminate might be calling these grayscale freq even though not used.
void setGsclkFreq(uint32_t new_gsclk_frequency);
uint32_t getGsclkFreq();

static const int _tlc_count; //
static const int latch_index;
static const int num_tlc_one_row;
static const uint8_t COLOR_CHANNEL_COUNT = 3;
static const uint8_t LEDS_PER_CHIP = 8;
static bool enforce_max_current;
static float max_current_amps;

static uint8_t _rgb_order[][LEDS_PER_CHIP][COLOR_CHANNEL_COUNT];
static uint16_t _grayscale_data[][LEDS_PER_CHIP][COLOR_CHANNEL_COUNT];
static uint8_t _latches[];
uint8_t rgb_order_default[3] = {0, 1, 2};

private:
  void updateLeds_2D(uint16_t * const_value=nullptr);
  void updateLeds_1D(uint16_t * const_value=nullptr);
  void updateChip_Leds(uint8_t chip, uint16_t * const_value);
  int enforceMaxCurrent(uint32_t * output_counts_ptr=nullptr);
  int _num_latches;
  int _num_tlc_one_row;
  uint8_t _use_2D;
  uint8_t _spi_mosi;
  uint8_t _spi_clk;
  uint8_t _blank;

  /* SPI */
  uint8_t _buffer;
  int8_t _buffer_count = 7;
  uint32_t spi_baud_rate = 1000000;

  const double maxCurrentValue = 0.02;
  SPISettings mSettings;
};

#endif

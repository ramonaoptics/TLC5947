/*
   TLC5947 Control Library
   Used to control the TI TLC5947 LED driver chip
   Zack Phillips - zkphil@berkeley.edu
   Product Page: http://www.ti.com/product/tlc5947
   Copyright (c) 2018, Zachary F. Phillips
   All rights reserved.
   Redistribution and use in source and binary forms, with or without
   modification, are permitted provided that the following conditions are met:
   Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.
   Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in the
      documentation and/or other materials provided with the distribution.
   Neither the name of the <organization> nor the
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

#include "TLC5947.h"

void TLC5947::init(uint8_t lat, uint8_t spi_mosi, uint8_t spi_clk, uint8_t blank)
{

  _lat = lat;
  _spi_clk = spi_clk;
  _spi_mosi = spi_mosi;
  _blank = blank;

  // Initialize SPI library
  SPI.setMOSI(_spi_mosi);
  SPI.begin();

  // Set up latch
  pinMode(_lat, OUTPUT);
  digitalWrite(_lat, LOW);

  // set up blank
  pinMode(_blank, OUTPUT);
  digitalWrite(_blank, HIGH);

  // Define baud rate
  SPISettings mSettings(spi_baud_rate, MSBFIRST, SPI_MODE0);

  // Set default color channel indicies
  setRgbPinOrder(rgb_order_default[0], rgb_order_default[1], rgb_order_default[2]);
}

void TLC5947::setSpiBaudRate(uint32_t new_baud_rate)
{
  // Store old baud rate
  spi_baud_rate = new_baud_rate;

  // Define baud rate
  SPISettings mSettings(spi_baud_rate, MSBFIRST, SPI_MODE0);
}

uint32_t TLC5947::getSpiBaudRate()
{
  // Return current baud rate
  return spi_baud_rate;
}

void TLC5947::setGsclkFreq(uint32_t new_gsclk_frequency)
{
  //not used. only kept for compatibility with TLC5955
}

uint32_t TLC5947::getGsclkFreq()
{
  return 0;
}

void TLC5947::setRgbPinOrder(uint8_t rPos, uint8_t grPos, uint8_t bPos)
{
  if (COLOR_CHANNEL_COUNT == 3)
  {
    for (int8_t chip = _tlc_count - 1; chip >= 0; chip--)
    {
      for (int8_t channel = 0; channel < LEDS_PER_CHIP; channel++)
      {
        _rgb_order[chip][channel][0] = rPos;
        _rgb_order[chip][channel][1] = grPos;
        _rgb_order[chip][channel][2] = bPos;
      }
    }
  } else
    Serial.println(F("ERROR (TLC5947::setRgbPinOrder): Color channel count is not 3"));
}

void TLC5947::setPinOrderSingle(uint16_t led_number, uint8_t color_channel_index, uint8_t position)
{
  uint8_t chip = (uint16_t)floor(led_number / LEDS_PER_CHIP);
  uint8_t channel = (uint8_t)(led_number - LEDS_PER_CHIP * chip);        // Turn that LED on
  _rgb_order[chip][channel][color_channel_index] = position;
}

void TLC5947::setRgbPinOrderSingle(uint16_t led_number, uint8_t rPos, uint8_t grPos, uint8_t bPos)
{
  uint8_t chip = (uint16_t)floor(led_number / LEDS_PER_CHIP);
  uint8_t channel = (uint8_t)round(led_number - LEDS_PER_CHIP * chip);        // Turn that LED on
  _rgb_order[chip][channel][0] = rPos;
  _rgb_order[chip][channel][1] = grPos;
  _rgb_order[chip][channel][2] = bPos;
}

void TLC5947::printByte(byte my_byte)
{
  for (byte mask = 0x80; mask; mask >>= 1)
  {
    if (mask  & my_byte)
      Serial.print('1');
    else
      Serial.print('0');
  }
}

void TLC5947::setAllLed(uint16_t gsvalue)
{
  for (int8_t chip = _tlc_count - 1; chip >= 0; chip--)
  {
    for (int8_t a = 0; a < LEDS_PER_CHIP; a++)
    {
      for (int8_t b = 0; b < COLOR_CHANNEL_COUNT; b++)
        _grayscale_data[chip][a][b] = gsvalue;
    }
  }
}

void TLC5947::setAllLedRgb(uint16_t red, uint16_t green, uint16_t blue)
{
  if (COLOR_CHANNEL_COUNT == 3)
  {
    for (int8_t chip = _tlc_count - 1; chip >= 0; chip--)
    {
      for (int8_t channel = 0; channel < LEDS_PER_CHIP; channel++)
      {
        _grayscale_data[chip][channel][2] = blue;
        _grayscale_data[chip][channel][1] = green;
        _grayscale_data[chip][channel][0] = red;
      }
    }
  } else
    Serial.println(F("ERROR (TLC5947::setAllLedRgb): Color channel count is not 3"));
}



void TLC5947::updateLeds()
{
  if (enforce_max_current)
  {
    // Get number of counts for current pattern
    uint32_t power_output_counts = 0;
    for (int16_t chip = (int8_t)_tlc_count - 1; chip >= 0; chip--)
      for (int8_t led_channel_index = (int8_t)LEDS_PER_CHIP - 1; led_channel_index >= 0; led_channel_index--)
        for (int8_t color_channel_index = (int8_t)COLOR_CHANNEL_COUNT - 1; color_channel_index >= 0; color_channel_index--)
          power_output_counts += _grayscale_data[chip][led_channel_index][color_channel_index];

    double power_output_amps = ((double)power_output_counts / (double)UINT16_MAX) * LED_CURRENT_AMPS;
    if (power_output_amps > max_current_amps)
    {
      Serial.print(F("Current output ("));
      Serial.print(power_output_amps);
      Serial.print(F(") exceeds maximum current output ("));
      Serial.print(max_current_amps);
	  Serial.println(')');
      return;
    }
  }
  if (debug >= 2)
  {
    Serial.println(F("Begin LED Update String (All Chips)..."));
  }

  // uint32_t power_output_counts = 0;
  for (int16_t chip = (int8_t)_tlc_count - 1; chip >= 0; chip--)
  {
    SPI.beginTransaction(mSettings);
    uint8_t color_channel_ordered;
    for (int8_t led_channel_index = (int8_t)LEDS_PER_CHIP - 1; led_channel_index >= 0; led_channel_index--)
    {
      for (int8_t color_channel_index = (int8_t)COLOR_CHANNEL_COUNT - 1; color_channel_index >= 0; color_channel_index--)
      {
        color_channel_ordered = _rgb_order[chip][led_channel_index][(uint8_t) color_channel_index];

        SPI.transfer((char)(_grayscale_data[chip][led_channel_index][color_channel_ordered] >> 8)); // Output MSB first
        SPI.transfer((char)(_grayscale_data[chip][led_channel_index][color_channel_ordered] & 0xFF)); // Followed by LSB
      }
    }
    SPI.endTransaction();
  }

  if (debug >= 2)
  {
    Serial.println(F("End LED Update String (All Chips)"));
  }
  latch();
}

void TLC5947::setChannel(uint16_t channel_number, uint16_t value)
{
  // Change to multi-channel indexing
  int16_t chip_number = floor(channel_number / (COLOR_CHANNEL_COUNT * LEDS_PER_CHIP));
  int16_t channel_number_new = (int16_t) floor((channel_number - chip_number * LEDS_PER_CHIP * COLOR_CHANNEL_COUNT) / COLOR_CHANNEL_COUNT);
  int16_t color_channel_number = (int16_t) (channel_number - chip_number * LEDS_PER_CHIP * COLOR_CHANNEL_COUNT) % COLOR_CHANNEL_COUNT;

  // Set grayscale data
  _grayscale_data[chip_number][channel_number_new][color_channel_number] = value;
}

void TLC5947::setLed(uint16_t led_number, uint16_t red, uint16_t green, uint16_t blue)
{
  uint8_t chip = (uint16_t)floor(led_number / LEDS_PER_CHIP);
  uint8_t channel = (uint8_t)(led_number - LEDS_PER_CHIP * chip);        // Turn that LED on
  _grayscale_data[chip][channel][2] = blue;
  _grayscale_data[chip][channel][1] = green;
  _grayscale_data[chip][channel][0] = red;
}

void TLC5947::setLedAppend(uint16_t led_number, uint16_t red, uint16_t green, uint16_t blue)
{
  uint8_t chip = (uint16_t)floor(led_number / LEDS_PER_CHIP);
  uint8_t channel = (uint8_t)(led_number - LEDS_PER_CHIP * chip);        // Turn that LED on

  if (((uint32_t)blue + (uint32_t) _grayscale_data[chip][channel][2]) > (uint32_t)UINT16_MAX)
    _grayscale_data[chip][channel][2] = UINT16_MAX;
  else
    _grayscale_data[chip][channel][2] = blue  + _grayscale_data[chip][channel][2];

  if (((uint32_t)green + (uint32_t) _grayscale_data[chip][channel][1]) > (uint32_t)UINT16_MAX)
    _grayscale_data[chip][channel][1] = UINT16_MAX;
  else
    _grayscale_data[chip][channel][1] = green  + _grayscale_data[chip][channel][1];

  if (((uint32_t)red + (uint32_t) _grayscale_data[chip][channel][0]) > (uint32_t)UINT16_MAX)
    _grayscale_data[chip][channel][0] = UINT16_MAX;
  else
    _grayscale_data[chip][channel][0] = red  + _grayscale_data[chip][channel][0];
}

void TLC5947::setLed(uint16_t led_number, uint16_t rgb)
{
  uint8_t chip = (uint16_t)floor(led_number / LEDS_PER_CHIP);
  uint8_t channel = (uint8_t)(led_number - LEDS_PER_CHIP * chip);        // Turn that LED on
  _grayscale_data[chip][channel][2] = rgb;
  _grayscale_data[chip][channel][1] = rgb;
  _grayscale_data[chip][channel][0] = rgb;
}





void TLC5947::latch()
{
  delayMicroseconds(LATCH_DELAY);
  digitalWrite(_lat, HIGH);
  delayMicroseconds(LATCH_DELAY);
  digitalWrite(_lat, LOW);
  delayMicroseconds(LATCH_DELAY);
}

// Get a single channel's current values
uint16_t TLC5947::getChannelValue(uint16_t channel_number, int color_channel_index)
{
  if (color_channel_index >= COLOR_CHANNEL_COUNT)
    return 0;

  uint8_t chip = (uint16_t)floor(channel_number / LEDS_PER_CHIP);
  uint8_t channel = (uint8_t)(channel_number - LEDS_PER_CHIP * chip);
  return _grayscale_data[chip][channel][color_channel_index];
}

// SPI interface - accumulates single bits, then sends over SPI
// interface once we accumulate 8 bits
void TLC5947::setBuffer(uint8_t bit)
{
  bitWrite(_buffer, _buffer_count, bit);
  _buffer_count--;
  SPI.beginTransaction(mSettings);
  if (_buffer_count == -1)
  {
    if (debug >= 2)
      printByte(_buffer);

    SPI.transfer(_buffer);
    _buffer_count = 7;
    _buffer = 0;
  }
  SPI.endTransaction();
}

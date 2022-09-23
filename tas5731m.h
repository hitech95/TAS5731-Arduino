/*
 * TAS573x Class D Amp register header
 *
 * Copyright (C) 2021 Nicolò Veronese
 *
 * Author: Nicolò Veronese <nicveronese@gmail.com>
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 */
#ifndef __TAS5731_H_
#define __TAS5731_H_

#include <Arduino.h>
#include <ArduinoLog.h>
#include <Adafruit_I2CDevice.h>

#include "tas573xx_reg.h"

// TODO - Default address
#define TAS5731_I2CADDR 0x1B

enum TAS5731_AUDIO_FORMATS
{
    TAS5731_FMT_I2S,     /* I2S mode */
    TAS5731_FMT_RIGHT_J, /* Right Justified mode */
    TAS5731_FMT_LEFT_J,  /* Left Justified mode */
};

enum TAS5731_DAI_FORMATS
{
    TAS5731_DAI_RATE_8000 = 8000,
    TAS5731_DAI_RATE_11025 = 11025,
    TAS5731_DAI_RATE_12000 = 12000,
    TAS5731_DAI_RATE_16000 = 16000,
    TAS5731_DAI_RATE_22050 = 22050,
    TAS5731_DAI_RATE_24000 = 24000,
    TAS5731_DAI_RATE_32000 = 32000,
    TAS5731_DAI_RATE_44100 = 44100,
    TAS5731_DAI_RATE_48000 = 48000,
};

enum TAS5731_MODE
{
    TAS5731_MODE_20,
    TAS5731_MODE_21,
    TAS5731_MODE_PBTL,
};

enum TAS5731_INPUT_MUX
{
    TAS5731_INPUT_MUX_SDIN_L,
    TAS5731_INPUT_MUX_SDIN_R,
    TAS5731_INPUT_MUX_GROUND,
    TAS5731_INPUT_MUX_LR_HALF,
    TAS5731_INPUT_MUX_LEFT_BQ
};

enum TAS5731_CHANNELS
{
    TAS5731_CHANNEL_1,
    TAS5731_CHANNEL_2,
    TAS5731_CHANNEL_3,
    TAS5731_CHANNEL_4,
};

static const unsigned int tas571x_mclk_ratios[] = {64, 128, 192, 256, 384, 512};
#define TAS571X_NUM_MCLK_RATIOS 6

class TAS5731
{
public:
    union packed32
    {
        uint32_t value;
        byte buffer[4];
    };

public:
    typedef void (*TAS5731CallbackFunction)(uint8_t npcm, uint8_t dtscd);
    typedef void (*TAS5731FunctionWithSender)(const void *sender, uint8_t npcm, uint8_t dtscd);

    // Library Functions
    TAS5731(int8_t reset_pin, uint8_t addr = TAS5731_I2CADDR, TwoWire *theWire = &Wire,
            int8_t fault_pin = -1, int8_t shutdown_pin = -1, int8_t pbtl_pin = -1,
            void (*isrCB)() = nullptr);
    ~TAS5731(void);

    bool begin(uint8_t mode, uint8_t format = TAS5731_FMT_I2S, uint8_t width = 24, bool bdMod = false);
    bool reset();
    int shutdown(bool status = false);
    int powerDown(bool fast = false);

    int16_t errors();
    int16_t clockStatus();

    int mute(bool mute = false);
    int softMute(uint8_t channel, bool mute = false);

    int setMasterVolumeDB(int16_t db)
    {
        return _setMasterVolume(_mapDb(db, -10350, 2400, 0xFE, 0x00));
    }
    int setChannelVolumeDB(uint8_t channel, int16_t db)
    {
        return _setChannelVolume(channel, _mapDb(db, -10350, 2400, 0xFE, 0x00));
    }

    int setInputMux(uint8_t channel1, uint8_t channel2, uint8_t channel4);

private:
    int _init(uint8_t mode, uint8_t format, uint8_t width, bool bdMod);
    int _isPBTL();
    int _setFormat(uint8_t format, uint8_t width);
    int _setDelay(bool modulation);
    int _setMasterVolume(uint8_t value);
    int _setChannelVolume(uint8_t channel, uint8_t value);

    uint8_t _mapDb(int16_t x, int16_t min, uint16_t max, uint8_t minOff, uint8_t maxOff)
    {
        return (x - min) * (maxOff - minOff) / (max - min) + minOff;
    }

    int updateRegister8(uint8_t reg, uint8_t mask, uint8_t value, uint8_t *buffer = nullptr);
    int read8(uint8_t addr, uint8_t *buffer);
    int write8(uint8_t addr, uint8_t data);

    int updateRegister32(uint8_t reg, uint32_t mask, uint32_t value, uint32_t *buffer = nullptr);
    int read32(uint8_t addr, uint32_t *buffer);
    int write32(uint8_t addr, uint32_t data);

    int readArray(uint8_t addr, uint8_t *buffer, size_t length);
    int writeArray(uint8_t addr, uint8_t *buffer, size_t length);

private:
    Adafruit_I2CDevice *i2c_dev = nullptr; ///< Pointer to I2C bus interface

    void (*isrCB)();
    bool isInitialized;

    uint8_t _masterVolume;

    int8_t _rstPin;   ///< The Arduino pin connected to reset (-1 if unused)
    int8_t _faultPin; ///< The Arduino pin connected to error (-1 if unused)
    int8_t _pdnPin;   ///< The Arduino pin connected to pdn (-1 if unused)
    int8_t _pbtlPin;  ///< The Arduino pin connected to pbtl (-1 if unused)
};

#endif
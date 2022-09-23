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
#include "tas5731m.h"

/*
 * TAS5731 Library stuff
 */
TAS5731::TAS5731(int8_t reset_pin, uint8_t addr, TwoWire *theWire,
                 int8_t fault_pin, int8_t shutdown_pin, int8_t pbtl_pin,
                 void (*isrCB)())
{
    i2c_dev = new Adafruit_I2CDevice(addr, theWire);

    this->_faultPin = fault_pin;
    this->_pdnPin = shutdown_pin;
    this->_rstPin = reset_pin;

    this->isInitialized = false;
}

TAS5731::~TAS5731(void)
{
    if (i2c_dev)
        delete i2c_dev;
}

bool TAS5731::begin(uint8_t mode, uint8_t format, uint8_t width, bool bdMod)
{
    i2c_dev->begin();
    // {
    //     Log.error(F("Did not find device at %X\n"), i2c_dev->address());
    //     return false;
    // }
    Log.info(F("Device on address %X\n"), i2c_dev->address());

    this->isInitialized = _init(mode, format, width, bdMod) == 0;
    // this->isInitialized = false;

    Log.info(F("Device initialized on address %X: %d\n"), i2c_dev->address(), this->isInitialized);
    return this->isInitialized;
}

bool TAS5731::reset()
{
    digitalWrite(_rstPin, LOW);
    delay(5);
    digitalWrite(_rstPin, HIGH);

    // Wait at least 13.5ms
    delay(15);

    return true;
}

int TAS5731::shutdown(bool status)
{
    uint8_t sysctl2;
    int ret;

    sysctl2 = status ? TAS571X_SYS_CTRL_2_SDN_MASK : 0;

    ret = updateRegister8(TAS571X_SYS_CTRL_2_REG,
                          TAS571X_SYS_CTRL_2_SDN_MASK, 0x00);
    if (ret)
    {
        Log.error(F("Could not set VOL_CFG: %d\n"), ret);
        return ret;
    }

    delay(50);

    return 0;
}

int TAS5731::powerDown(bool fast)
{
    if (!fast)
    {
        // Enter shutdown
        shutdown(true);

        // Lower PDN pin
        if (this->_pdnPin > 0)
        {
            digitalWrite(_pdnPin, LOW);
        }

        // Wait at least 2ms
        delay(5);
    }

    // Enter reset state
    digitalWrite(_rstPin, LOW);

    return 0;
}

int16_t TAS5731::errors()
{
    int ret;
    uint8_t value;

    ret = read8(TAS571X_ERR_STATUS_REG, &value);
    if (ret)
    {
        Log.error(F("Could not read ERR_STATUS_REG: %d\n"), ret);
        return ret;
    }

    return value;
}

int16_t TAS5731::clockStatus()
{
    int ret;
    uint8_t value;

    ret = read8(TAS571X_CLK_CTRL_REG, &value);
    if (ret)
    {
        Log.error(F("Could not readCLK_CTRL_REG: %d\n"), ret);
        return ret;
    }

    return value;
}

int TAS5731::mute(bool mute)
{
    uint8_t value;
    int ret;

    value = mute ? TAS571X_SYS_CTRL_2_SDN_MASK : this->_masterVolume;

    ret = write8(TAS571X_MVOL_REG, value);
    if (ret)
    {
        Log.error(F("Could not update Master Volume: %d\n"), ret);
        return ret;
    }
    delay(1);

    return ret;
}

int TAS5731::softMute(uint8_t channel, bool mute)
{
    uint8_t mask;
    int res;

    switch (channel)
    {
    case TAS5731_CHANNEL_1:
        mask = TAS571X_SOFT_MUTE_CH1_MASK;
        break;
    case TAS5731_CHANNEL_2:
        mask = TAS571X_SOFT_MUTE_CH2_MASK;
        break;
    case TAS5731_CHANNEL_3:
        mask = TAS571X_SOFT_MUTE_CH3_MASK;
        break;
    default:
        Log.error(F("Invalid channel to mute for: %X\n"), channel);
        return -EINVAL;
    }

    res = updateRegister8(TAS571X_SOFT_MUTE_REG, mask, mute ? mask : 0x00);
    if (res)
    {
        Log.error(F("Unable to mute channel(%X): %d\n"), channel, res);
        return res;
    }
    return res;
}

int TAS5731::setInputMux(uint8_t chan1, uint8_t chan2, uint8_t chan4)
{
    uint8_t chan1Value, chan2Value, chan4Value;
    int ret;

    switch (chan1)
    {
    case TAS5731_INPUT_MUX_SDIN_L:
        chan1Value = 0x00;
        break;
    case TAS5731_INPUT_MUX_SDIN_R:
        chan1Value = 0x01;
        break;
    case TAS5731_INPUT_MUX_GROUND:
        chan1Value = 0x06;
        break;
    default:
        Log.error(F("Invalid Input Mux value (%X) for chan1\n"), chan1);
        return -EINVAL;
    }

    switch (chan2)
    {
    case TAS5731_INPUT_MUX_SDIN_L:
        chan2Value = 0x00;
        break;
    case TAS5731_INPUT_MUX_SDIN_R:
        chan2Value = 0x01;
        break;
    case TAS5731_INPUT_MUX_GROUND:
        chan2Value = 0x06;
        break;
    default:
        Log.error(F("Invalid Input Mux value (%X) for chan2\n"), chan2);
        return -EINVAL;
    }

    ret = updateRegister32(TAS571X_INPUT_MUX_REG,
                           TAS571X_INPUT_MUX_CHAN1_ROUTEMASK | TAS571X_INPUT_MUX_CHAN2_ROUTEMASK,
                           (chan1Value << TAS571X_INPUT_MUX_CHAN1_ROUTE_SHIFT) |
                               (chan2Value << TAS571X_INPUT_MUX_CHAN2_ROUTE_SHIFT));
    if (ret)
    {
        Log.error(F("Unable to ser CHAN_1_2 Input Mux: %d\n"), ret);
        return ret;
    }

    switch (chan4)
    {
    case TAS5731_INPUT_MUX_LR_HALF:
        chan4Value = 0x00;
        break;
    case TAS5731_INPUT_MUX_LEFT_BQ:
        chan4Value = 0x01;
        break;
    default:
        Log.error(F("Invalid Input Mux value (%X) for chan4\n"), chan4);
        return -EINVAL;
    }

    ret = updateRegister32(TAS571X_CH4_SRC_SELECT_REG,
                           TAS571X_CH4_SRC_SELECT_MASK,
                           chan4Value << TAS571X_CH4_SRC_SELECT_SHIFT);
    if (ret)
    {
        Log.error(F("Unable to ser CHAN_4 Input Mux: %d\n"), ret);
        return ret;
    }
    return ret;
}

int TAS5731::_init(uint8_t mode, uint8_t format, uint8_t width, bool bdMod)
{
    uint8_t reg_value;
    int i, ret;

    if (!this->isInitialized)
    {
        if (this->_rstPin == -1)
        {
            Log.error(F("Reset pin in requred, specified (%d) is not valid %u\n"),
                      this->_rstPin);
            return -EINVAL;
        }

        // Setup pins
        if (this->_pbtlPin > 0)
        {
            pinMode(this->_pbtlPin, INPUT_PULLUP);
        }

        if (this->_faultPin > 0)
        {
            pinMode(this->_faultPin, INPUT_PULLUP);
        }

        // Reset the chip and start the configuration

        if (this->_pdnPin > 0)
        {
            pinMode(_pdnPin, OUTPUT);
        }

        pinMode(_rstPin, OUTPUT);
    }

    // Enter RESET STATE
    if (this->_pdnPin > 0)
    {
        digitalWrite(_pdnPin, LOW);
    }
    digitalWrite(_rstPin, LOW);

    // Await 15 ms before exit reset
    delay(15);

    // Exit SD & RESET status
    if (this->_pdnPin > 0)
    {
        digitalWrite(_pdnPin, HIGH);
        delay(1);
    }
    digitalWrite(_rstPin, HIGH);

    // Await 15 ms before trim
    delay(15);

    // Run a factory OSC TRIM as stated in datasheet
    ret = write8(TAS571X_OSC_TRIM_REG, 0);
    if (ret)
    {
        Log.error(F("Could not set OSC_TRIM: %d\n"), ret);
        return ret;
    }

    // Lets wait 50ms
    delay(50);

    // Soft mute channnels 1, 2, 3 (4) (chan 3 is chan 4 in DAP)
    ret = write8(TAS571X_SOFT_MUTE_REG, 0x07);
    if (ret)
    {
        Log.error(F("Could not set SOFT_MUTE: %d\n"), ret);
        return ret;
    }

    if (this->_faultPin > 0)
    {
        ret = updateRegister8(TAS571X_SYS_CTRL_2_REG,
                              TAS571X_SYS_CTRL_2_ADR_FAULT_MASK, TAS571X_SYS_CTRL_2_ADR_FAULT_MASK);
        if (ret)
        {
            Log.error(F("Could not set SYS_CTRL_2: %d\n"), ret);
            return ret;
        }
    }

    // If we want to use BD modulation update configuration registers
    if (bdMod)
    {
        // Configure channel 1 & 2
        ret = updateRegister32(TAS571X_INPUT_MUX_REG,
                               TAS571X_INPUT_MUX_CHAN1_MODE_MASK | TAS571X_INPUT_MUX_CHAN2_MODE_MASK,
                               TAS571X_INPUT_MUX_CHAN1_MODE_MASK | TAS571X_INPUT_MUX_CHAN2_MODE_MASK);
        if (ret)
        {
            Log.error(F("Could not set BD Mode on chan 1/2 (INPUT_MUX): %d\n"), ret);
            return ret;
        }

        // Configure channel 4
        ret = updateRegister8(TAS571X_SYS_CTRL_2_REG,
                              TAS571X_SYS_CTRL_2_SUB_MODE_MASK, TAS571X_SYS_CTRL_2_SUB_MODE_MASK);
        if (ret)
        {
            Log.error(F("Could not set BD Mode on chan 3 (INPUT_MUX): %d\n"), ret);
            return ret;
        }
    }

    // Configure Amplifier Stage
    switch (mode)
    {
    case TAS5731_MODE_PBTL:
        reg_value = 0x0;

        // Check if PBTL pin is set else throw -ENXIO
        if (this->_pbtlPin > 0 && !digitalRead(this->_pbtlPin))
        {
            Log.error(F("Cannot not configure PBTL mode on a non PBTL board!\n"));
            return -ENXIO;
        }

        // Shutdown unused PWM channels
        ret = write8(TAS571X_PWM_CH_SDN_GROUP_REG, 0X3A);
        if (ret)
        {
            Log.error(F("Could not set PWM_CH_SDN_GROUP: %d\n"), ret);
            return ret;
        }

        // Set PWM generator to correct channel (PBTL)
        ret = write32(TAS571X_PWM_MUX_REG, 0x01103245);
        if (ret)
        {
            Log.error(F("Could not set PWM_MUX_REG: %d\n"), ret);
            return ret;
        }
        break;
    case TAS5731_MODE_20:
        reg_value = 0x00;
        break;
    case TAS5731_MODE_21:
        reg_value = TAS571X_SYS_CTRL_2_CH_MODE_MASK;
        break;
    default:
        return -EINVAL;
    }

    ret = updateRegister8(TAS571X_SYS_CTRL_2_REG,
                          TAS571X_SYS_CTRL_2_CH_MODE_MASK, reg_value);
    if (ret)
    {
        Log.error(F("Could not set SYS_CTRL_2: %d\n"), ret);
        return ret;
    }

    // Lower PWM modulation since we have 33nF boost caps
    // This is not necessary if we are using 10nF caps
    // TI's DS says nothing about this. Found in EE forum
    ret = write8(TAS571X_MODULATION_LIMIT_REG, 0X07);
    if (ret)
    {
        Log.error(F("Could not set MODULATION_LIMIT: %d\n"), ret);
        return ret;
    }

    // Set DAI Format
    ret = this->_setFormat(format, width);
    if (ret)
    {
        Log.error(F("Could not configure DAI format: %d\n"), ret);
        return ret;
    }

    // Set chan 1, 2, 3 (4) delays to default values
    ret = this->_setDelay(bdMod);
    if (ret)
    {
        return ret;
    }

    // Set the default Input Mux Routing
    ret = this->setInputMux(
        TAS5731_INPUT_MUX_SDIN_L,
        TAS5731_INPUT_MUX_SDIN_R,
        TAS5731_INPUT_MUX_LR_HALF);
    if (ret)
    {
        Log.error(F("Could not configure DAP Input Mux: %d\n"), ret);

        return ret;
    }

    // Assign DAP ch4 to 3rd volume control
    ret = updateRegister8(TAS571X_VOL_CFG_REG,
                          TAS571X_VOL_CFG_REG_CH4_MASK, TAS571X_VOL_CFG_REG_CH4_MASK);
    if (ret)
    {
        Log.error(F("Could not set VOL_CFG: %d\n"), ret);
        return ret;
    }

    // Set chan 1, 2, 3 (4) volumes to 0dB
    ret = write8(TAS571X_CH1_VOL_REG, 0x30);
    if (ret)
    {
        Log.error(F("Could not CH1_VOL: %d\n"), ret);
        return ret;
    }
    ret = write8(TAS571X_CH2_VOL_REG, 0x30);
    if (ret)
    {
        Log.error(F("Could not CH2_VOL: %d\n"), ret);
        return ret;
    }
    ret = write8(TAS571X_CH3_VOL_REG, 0x30);
    if (ret)
    {
        Log.error(F("Could not CH3_VOL: %d\n"), ret);
        return ret;
    }

    // Exit Hard Mute (shutdown)
    ret = this->shutdown(false);
    if (ret)
    {
        Log.error(F("Could not EXIT SD Mode: %d\n"), ret);
        return ret;
    }

    // Set Master Volume
    ret = this->_setMasterVolume(0xFF);
    if (ret)
    {
        Log.error(F("Could not Master Volume to MUTE: %d\n"), ret);
        return ret;
    }

    // Soft unmute channnels 1, 2, 3 (4) (chan 3 is chan 4 in DAP)
    ret = write8(TAS571X_SOFT_MUTE_REG, 0x00);
    if (ret)
    {
        Log.error(F("Could not set SOFT_MUTE: %d\n"), ret);
        return ret;
    }

    return ret;
}

int TAS5731::_isPBTL()
{
    int ret;
    if (this->_pbtlPin == -1)
    {
        return -ENXIO;
    }

    return digitalRead(this->_pbtlPin);
}

int TAS5731::_setFormat(uint8_t format, uint8_t width)
{
    uint8_t val;
    int res;

    switch (format)
    {
    case TAS5731_FMT_RIGHT_J:
        val = 0x00;
        break;
    case TAS5731_FMT_I2S:
        val = 0x03;
        break;
    case TAS5731_FMT_LEFT_J:
        val = 0x06;
        break;
    default:
        return -EINVAL;
    }

    if (width >= 24)
        val += 2;
    else if (width >= 20)
        val += 1;

    res = updateRegister8(TAS571X_SDI_REG, TAS571X_SDI_FMT_MASK, val);
    if (res)
    {
        Log.error(F("Cannot set SDI_REG: %d\n"), res);
        return res;
    }

    return res;
}

int TAS5731::_setDelay(bool bdMod)
{
    int res;

    if (bdMod)
    {
        res = write8(TAS571X_IC_DELAY_PWM_A_REG, 0xB8);
        res = write8(TAS571X_IC_DELAY_PWM_B_REG, 0x60);
        res = write8(TAS571X_IC_DELAY_PWM_C_REG, 0xA0);
        res = write8(TAS571X_IC_DELAY_PWM_D_REG, 0x48);
    }
    else
    {
        res = write8(TAS571X_IC_DELAY_PWM_A_REG, 0xAC);
        res = write8(TAS571X_IC_DELAY_PWM_B_REG, 0x54);
        res = write8(TAS571X_IC_DELAY_PWM_C_REG, 0xAC);
        res = write8(TAS571X_IC_DELAY_PWM_D_REG, 0x54);
    }

    return res;
}

int TAS5731::_setMasterVolume(uint8_t value)
{
    int ret;

    this->_masterVolume = value;

    ret = write8(TAS571X_MVOL_REG, value);
    if (ret)
    {
        Log.error(F("Could not set master volume: %d\n"), ret);
        return ret;
    }

    return ret;
}

int TAS5731::_setChannelVolume(uint8_t channel, uint8_t value)
{
    uint8_t reg;
    int ret;

    switch (channel)
    {
    case TAS5731_CHANNEL_1:
        reg = TAS571X_CH1_VOL_REG;
        break;
    case TAS5731_CHANNEL_2:
        reg = TAS571X_CH2_VOL_REG;

        break;
    case TAS5731_CHANNEL_3:
        reg = TAS571X_CH3_VOL_REG;
        break;
    default:
        Log.error(F("Invalid channel to set volume for: %X\n"), channel);
        return -EINVAL;
    }

    ret = write8(reg, value);
    if (ret)
    {
        Log.error(F("Could not set channel (%X) volume: %d\n"), channel, ret);
        return ret;
    }

    return ret;
}

/*
 * I2C helper methods
 */
int TAS5731::updateRegister8(uint8_t reg, uint8_t mask, uint8_t value, uint8_t *buffer)
{
    uint8_t data;
    int ret;

    ret = read8(reg, &data);
    if (ret)
    {
        Log.error(F("Could not read from device: %d\n"), ret);
        return ret;
    }

    data = (data & ~mask) | (value & mask);

    // Store the new value into the assigned variable
    if (buffer != nullptr)
    {
        *buffer = data;
    }
    ret = write8(reg, data);
    if (ret)
    {
        Log.error(F("Could not write from device: %d\n"), ret);
        return ret;
    }

    return ret;
}

int TAS5731::read8(uint8_t reg, uint8_t *buffer)
{
    // send 1 byte, read 1 byte
    return readArray(reg, buffer, 1);
}

int TAS5731::write8(uint8_t reg, uint8_t val)
{
    return writeArray(reg, &val, 1);
}

int TAS5731::updateRegister32(uint8_t reg, uint32_t mask, uint32_t value, uint32_t *buffer)
{
    uint32_t data;
    int ret;

    ret = read32(reg, &data);
    if (ret)
    {
        Log.error(F("Could not read from device: %d\n"), ret);
        return ret;
    }

    data = (data & ~mask) | (value & mask);

    // Store the new value into the assigned variable
    if (buffer != nullptr)
    {
        *buffer = data;
    }
    ret = write32(reg, data);
    if (ret)
    {
        Log.error(F("Could not write to device: %d\n"), ret);
        return ret;
    }

    return ret;
}

int TAS5731::read32(uint8_t reg, uint32_t *buffer)
{
    int i, res;
    uint8_t readBuffer[4];

    packed32 readPacket;

    // Send 1 byte, read 1 byte
    res = readArray(reg, readBuffer, 4);

    // Reverse byte order
    for (i = 0; i < 4; i++)
    {
        readPacket.buffer[i] = readBuffer[3 - i];
    }

    if (res)
    {
        Log.error(F("Could not perform read32()\n"), res);
        return res;
    }

    *buffer = readPacket.value;
    return res;
}

int TAS5731::write32(uint8_t reg, uint32_t val)
{
    int i;
    uint8_t writeBuffer[4];
    packed32 writePacket;
    writePacket.value = val;

    for (i = 0; i < 4; i++)
    {
        writeBuffer[i] = writePacket.buffer[3 - i];
    }

    return writeArray(reg, writeBuffer, 4);
}

int TAS5731::readArray(uint8_t reg, uint8_t *buffer, size_t length)
{
    // send 1 byte, reset i2c, read n bytes
    bool success = i2c_dev->write_then_read(&reg, 1, buffer, length, true);
    return success ? 0 : -EIO;
}

int TAS5731::writeArray(uint8_t reg, uint8_t *buffer, size_t length)
{
    // send 1 byte, reset i2c, read n bytes
    bool success = i2c_dev->write(buffer, length, true, &reg, 1);
    return success ? 0 : -EIO;
}

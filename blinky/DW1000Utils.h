#pragma once

#include <mbed.h>
#include "DW1000.h"

class DW1000Utils
{
public:
    // Recommended settings:
    // Data rate 110 kbps - Preamble length 2048 or 4096.
    // Data rate 850 kbps - Preamble length 256, 512 or 1024.
    // Data rate 6.8 Mbps - Preamble length 64.

    const static uint32_t PREAMBLE_LENGTH_64 = (1 << 18);
    const static uint32_t PREAMBLE_LENGTH_128 = (1 << 18) | (1 << 20);
    const static uint32_t PREAMBLE_LENGTH_256 = (1 << 18) | (1 << 21);
    const static uint32_t PREAMBLE_LENGTH_512 = (1 << 18) | (1 << 20) | (1 << 21);
    const static uint32_t PREAMBLE_LENGTH_1024 = (1 << 19);
    const static uint32_t PREAMBLE_LENGTH_2048 = (1 << 19) | (1 << 21);
    const static uint32_t PREAMBLE_LENGTH_4096 = (1 << 18) | (1 << 19);

    enum PrfSetting
    {
        PRF_16_MHz = 1,
        PRF_64_MHz,
    };

    enum DataRateSetting
    {
        RATE_110_kbps = 1,
        RATE_850_kbps,
        RATE_6_8_Mbps,
    };

    // Set pulse repetition frequency
    static void setPulseRepetitionFrequency(DW1000* dw_ptr, PrfSetting prf_setting)
    {
        // Transmit PRF setting (see page 75 of user manual)
        uint32_t prf_value;
        if (prf_setting == PRF_16_MHz)
        {
            prf_value = (1 << 16);
        }
        else
        {
            prf_value = (1 << 17);
        }
        uint32_t prf_mask = (1 << 16) | (1 << 17);
        uint32_t tx_ctrl = dw_ptr->readRegister32(DW1000_TX_FCTRL, 0x00);
        tx_ctrl &= ~prf_mask;
        tx_ctrl |= (prf_value & prf_mask);
        dw_ptr->writeRegister32(DW1000_TX_FCTRL, 0x00, tx_ctrl);

        // Receive PRF setting (see page 109 and of user manual)
        if (prf_setting == PRF_16_MHz)
        {
            prf_value = (1 << 18);
        }
        else
        {
            prf_value = (1 << 19);
        }
        prf_mask = (1 << 18) | (1 << 19);
        uint32_t chan_ctrl = dw_ptr->readRegister32(DW1000_CHAN_CTRL, 0x00);
        chan_ctrl &= ~prf_mask;
        chan_ctrl |= (prf_value & prf_mask);
        dw_ptr->writeRegister32(DW1000_CHAN_CTRL, 0x00, chan_ctrl);
    }

    // Set preamble length (see page 76 of user manual)
    static void setPreambleLength(DW1000* dw_ptr, uint32_t preamble_setting)
    {
        uint32_t preamble_mask = (1 << 18) | (1 << 19) | (1 << 20) | (1 << 21);
        uint32_t tx_ctrl = dw_ptr->readRegister32(DW1000_TX_FCTRL, 0x00);
        tx_ctrl &= ~preamble_mask;
        tx_ctrl |= (preamble_setting & preamble_mask);
        dw_ptr->writeRegister32(DW1000_TX_FCTRL, 0x00, tx_ctrl);
    }

    // Set data rate
    static void setDataRate(DW1000* dw_ptr, DataRateSetting rate_setting)
    {
        // Transmit data rate (see page 73 of user manual)
        uint32_t rate_value;
        if (rate_setting == RATE_110_kbps)
        {
            rate_value = 0;
        }
        else if (rate_setting == RATE_850_kbps)
        {
            rate_value = (1 << 13);
        }
        else
        {
            rate_value = (1 << 14);
        }
        uint32_t rate_mask = (1 << 13) | (1 << 14);
        uint32_t tx_ctrl = dw_ptr->readRegister32(DW1000_TX_FCTRL, 0x00);
        tx_ctrl &= ~rate_mask;
        tx_ctrl |= (rate_value & rate_mask);
        dw_ptr->writeRegister32(DW1000_TX_FCTRL, 0x00, tx_ctrl);

        // Receive data rate (see page 72 of user manual)
        if (rate_setting == RATE_110_kbps)
        {
            rate_value = (1 << 18);
            rate_value = (1 << 22);
        }
        else if (rate_setting == RATE_850_kbps)
        {
            rate_value = (1 << 18);
        }
        else
        {
            rate_value = 0;
        }
        rate_mask = (1 << 18) | (1 << 22);
        uint32_t sys_cfg = dw_ptr->readRegister32(DW1000_SYS_CFG, 0x00);
        sys_cfg &= ~rate_mask;
        sys_cfg |= (rate_value & rate_mask);
        dw_ptr->writeRegister32(DW1000_SYS_CFG, 0x00, sys_cfg);

        if (rate_setting == RATE_110_kbps)
        {
            dw_ptr->writeRegister16(DW1000_DRX_CONF, 0x02, 0x000A);     // DRX_TUNE0b for 110 kbps
            dw_ptr->writeRegister16(DW1000_DRX_CONF, 0x06, 0x0064);     // DRX_TUNE1b for 110 kbps & > 1024 symbols
        }
        else if (rate_setting == RATE_850_kbps)
        {
            dw_ptr->writeRegister16(DW1000_DRX_CONF, 0x02, 0x0001);     // DRX_TUNE0b for 850 kbps
            dw_ptr->writeRegister16(DW1000_DRX_CONF, 0x06, 0x0020);     // DRX_TUNE1b for 850 kbps & 128 - 1024 symbols
        }
        else
        {
            dw_ptr->writeRegister16(DW1000_DRX_CONF, 0x02, 0x0001);     // DRX_TUNE0b for 6.8 Mbps
            dw_ptr->writeRegister16(DW1000_DRX_CONF, 0x06, 0x0010);     // DRX_TUNE1b for 6.8 Mbps & 64 symbols
        }
    }

    // Improved settings for direct path detection in non-line-of-sight environments.
    // See DecaWave Application Note APS006.
    static void setNLOSSettings(DW1000* dw_ptr, DataRateSetting rate_setting = RATE_850_kbps, PrfSetting prf_setting = PRF_16_MHz, uint32_t preamble_setting = PREAMBLE_LENGTH_1024)
    {
        setDataRate(dw_ptr, rate_setting);
        setPulseRepetitionFrequency(dw_ptr, prf_setting);
        setPreambleLength(dw_ptr, preamble_setting);

        // Setting for Noise Threshold Multiplier 1
        dw_ptr->writeRegister8(DW1000_LDE_CTRL, 0x0806, 0x07);                  // LDE_CFG1
        // Setting for Noise Threshold Multiplier 2
        if (prf_setting == PRF_16_MHz)
        {
            dw_ptr->writeRegister16(DW1000_LDE_CTRL, 0x1806, 0x0003);           // LDE_CFG2 for 16 MHz PRF
        }
        else
        {
            dw_ptr->writeRegister16(DW1000_LDE_CTRL, 0x1806, 0x1603);           // LDE_CFG2 for 64 MHz PRF
        }
    }

    // Default settings for line-of-sight environments
    static void setLOSSettings(DW1000* dw_ptr, DataRateSetting rate_setting = RATE_850_kbps, PrfSetting prf_setting = PRF_16_MHz, uint32_t preamble_setting = PREAMBLE_LENGTH_1024)
    {
        setDataRate(dw_ptr, rate_setting);
        setPulseRepetitionFrequency(dw_ptr, prf_setting);
        setPreambleLength(dw_ptr, preamble_setting);

        // Setting for Noise Threshold Multiplier 1
            dw_ptr->writeRegister8(DW1000_LDE_CTRL, 0x0806, 0x0c);              // LDE_CFG1
//        dw_ptr->writeRegister8(DW1000_LDE_CTRL, 0x0806, 0x0d);                // LDE_CFG1
        // Setting for Noise Threshold Multiplier 2
        if (prf_setting == PRF_16_MHz)
        {
            dw_ptr->writeRegister16(DW1000_LDE_CTRL, 0x1806, 0x1607);           // LDE_CFG2 for 16 MHz PRF
        }
        else
        {
            dw_ptr->writeRegister16(DW1000_LDE_CTRL, 0x1806, 0x0607);           // LDE_CFG2 for 64 MHz PRF
        }
    }
};

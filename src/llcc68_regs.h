/**
 * @file      llcc68_regs.h
 *
 * @brief     LLCC68 register definitions
 *
 * The Clear BSD License
 * Copyright Semtech Corporation 2021. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted (subject to the limitations in the disclaimer
 * below) provided that the following conditions are met:
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Semtech corporation nor the
 *       names of its contributors may be used to endorse or promote products
 *       derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY
 * THIS LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
 * CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT
 * NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
 * PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL SEMTECH CORPORATION BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef LLCC68_REGS_H
#define LLCC68_REGS_H

/*
 * -----------------------------------------------------------------------------
 * --- DEPENDENCIES ------------------------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC MACROS -----------------------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC CONSTANTS --------------------------------------------------------
 */

/**
 * @brief The address of the register holding the first byte defining the CRC seed
 */
#define LLCC68_REG_CRCSEEDBASEADDRESS 0x06BC

/**
 * @brief The address of the register holding the first byte defining the CRC polynomial
 */
#define LLCC68_REG_CRCPOLYBASEADDRESS 0x06BE

/**
 * @brief The address of the register holding the first byte defining the whitening seed
 */
#define LLCC68_REG_WHITSEEDBASEADDRESS 0x06B8

/**
 * @brief The addresses of the registers holding SyncWords values
 */
#define LLCC68_REG_SYNCWORDBASEADDRESS 0x06C0

/**
 * @brief The addresses of the register holding LoRa Modem SyncWord value
 *        0x1424: LoRaWAN private network,
 *        0x3444: LoRaWAN public network
 */
#define LLCC68_REG_LR_SYNCWORD 0x0740

/**
 * @brief The address of the register holding the coding rate configuration extracted from a received LoRa header
 */
#define LLCC68_REG_LR_HEADER_CR 0x0749
#define LLCC68_REG_LR_HEADER_CR_POS ( 4U )
#define LLCC68_REG_LR_HEADER_CR_MASK ( 0x07UL << LLCC68_REG_LR_HEADER_CR_POS )

/**
 * @brief The address of the register holding the CRC configuration extracted from a received LoRa header
 */
#define LLCC68_REG_LR_HEADER_CRC 0x076B
#define LLCC68_REG_LR_HEADER_CRC_POS ( 4U )
#define LLCC68_REG_LR_HEADER_CRC_MASK ( 0x01UL << LLCC68_REG_LR_HEADER_CRC_POS )

/*!
 * The address of the register giving a 32-bit random number
 */
#define LLCC68_REG_RNGBASEADDRESS 0x0819

/*!
 * The address of the register used to disable the LNA
 */
#define LLCC68_REG_ANA_LNA 0x08E2

/*!
 * The address of the register used to disable the mixer
 */
#define LLCC68_REG_ANA_MIXER 0x08E5

/*!
 * The address of the register holding RX Gain value
 *     0x94: power saving,
 *     0x96: rx boosted
 */
#define LLCC68_REG_RXGAIN 0x08AC

/**
 * @brief Change the value on the device internal trimming capacitor
 */
#define LLCC68_REG_XTATRIM 0x0911

/**
 * @brief Set the current max value in the over current protection
 */
#define LLCC68_REG_OCP 0x08E7

/**
 * @brief WORKAROUND - Optimizing the Inverted IQ Operation, see DS_LLCC68_V1.0 datasheet chapter 15.4
 */
#define LLCC68_REG_IQ_POLARITY 0x0736

/**
 * @brief WORKAROUND - Modulation Quality with 500 kHz LoRa Bandwidth, see DS_LLCC68_V1.0 datasheet chapter 15.1
 */
#define LLCC68_REG_TX_MODULATION 0x0889

/**
 * @brief WORKAROUND - Better resistance to antenna mismatch, see DS_LLCC68_V1.0 datasheet chapter 15.2
 */
#define LLCC68_REG_TX_CLAMP_CFG 0x08D8
#define LLCC68_REG_TX_CLAMP_CFG_POS ( 1U )
#define LLCC68_REG_TX_CLAMP_CFG_MASK ( 0x0FUL << LLCC68_REG_TX_CLAMP_CFG_POS )

/**
 * @brief RTC control
 */
#define LLCC68_REG_RTC_CTRL 0x0902

/**
 * @brief Event clear
 */
#define LLCC68_REG_EVT_CLR 0x0944
#define LLCC68_REG_EVT_CLR_TIMEOUT_POS ( 1U )
#define LLCC68_REG_EVT_CLR_TIMEOUT_MASK ( 0x01UL << LLCC68_REG_EVT_CLR_TIMEOUT_POS )

/**
 * @brief RX address pointer
 */
#define LLCC68_REG_RX_ADDRESS_POINTER 0x0803

/**
 * @brief RX/TX payload length
 */
#define LLCC68_REG_RXTX_PAYLOAD_LEN 0x06BB

/**
 * @brief Output disable
 */
#define LLCC68_REG_OUT_DIS_REG 0x0580
#define LLCC68_REG_OUT_DIS_REG_DIO3_POS ( 3U )
#define LLCC68_REG_OUT_DIS_REG_DIO3_MASK ( 0x01UL << LLCC68_REG_OUT_DIS_REG_DIO3_POS )

/**
 * @brief Input enable
 */
#define LLCC68_REG_IN_EN_REG 0x0583
#define LLCC68_REG_IN_EN_REG_DIO3_POS ( 3U )
#define LLCC68_REG_IN_EN_REG_DIO3_MASK ( 0x01UL << LLCC68_REG_IN_EN_REG_DIO3_POS )

/**
 * @brief TX bitbang A
 */
#define LLCC68_REG_BITBANG_A_REG 0x0680
#define LLCC68_REG_BITBANG_A_REG_ENABLE_POS ( 4U )
#define LLCC68_REG_BITBANG_A_REG_ENABLE_MASK ( 0x07UL << LLCC68_REG_BITBANG_A_REG_ENABLE_POS )
#define LLCC68_REG_BITBANG_A_REG_ENABLE_VAL ( 0x01UL << LLCC68_REG_BITBANG_A_REG_ENABLE_POS )

/**
 * @brief TX bitbang B
 */
#define LLCC68_REG_BITBANG_B_REG 0x0587
#define LLCC68_REG_BITBANG_B_REG_ENABLE_POS ( 0U )
#define LLCC68_REG_BITBANG_B_REG_ENABLE_MASK ( 0x0FUL << LLCC68_REG_BITBANG_B_REG_ENABLE_POS )
#define LLCC68_REG_BITBANG_B_REG_ENABLE_VAL ( 0x0CUL << LLCC68_REG_BITBANG_B_REG_ENABLE_POS )

/**
 * @brief Number of symbols given as LLCC68_REG_LR_SYNCH_TIMEOUT[7:3] * 2 ^ (2*LLCC68_REG_LR_SYNCH_TIMEOUT[2:0] + 1)
 */
#define LLCC68_REG_LR_SYNCH_TIMEOUT 0x0706

/**
 * @brief Base address of the register retention list
 */
#define LLCC68_REG_RETENTION_LIST_BASE_ADDRESS 0x029F

/**
 * @brief GFSK node address
 *
 * @remark Reset value is 0x00
 */
#define LLCC68_REG_GFSK_NODE_ADDRESS 0x06CD

/**
 * @brief GFSK broadcast address
 *
 * @remark Reset value is 0x00
 */
#define LLCC68_REG_GFSK_BROADCAST_ADDRESS 0x06CE

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC TYPES ------------------------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC FUNCTIONS PROTOTYPES ---------------------------------------------
 */

#endif  // LLCC68_REGS_H

/* --- EOF ------------------------------------------------------------------ */

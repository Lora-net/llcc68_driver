/**
 * @file      llcc68.h
 *
 * @brief     LLCC68 radio driver definition
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

#ifndef LLCC68_H
#define LLCC68_H

#ifdef __cplusplus
extern "C" {
#endif

/*
 * -----------------------------------------------------------------------------
 * --- DEPENDENCIES ------------------------------------------------------------
 */

#include <stdint.h>
#include <stdbool.h>

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC MACROS -----------------------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC CONSTANTS --------------------------------------------------------
 */

/**
 * @brief Maximum value for parameter timeout_in_rtc_step in both functions @ref llcc68_set_rx_with_timeout_in_rtc_step
 * and @ref llcc68_set_tx_with_timeout_in_rtc_step
 */
#define LLCC68_MAX_TIMEOUT_IN_RTC_STEP 0x00FFFFFE

/**
 * @brief  Maximum value for parameter timeout_in_ms in both functions @ref llcc68_set_rx and @ref llcc68_set_tx
 */
#define LLCC68_MAX_TIMEOUT_IN_MS ( LLCC68_MAX_TIMEOUT_IN_RTC_STEP / 64 )

/**
 * @brief Timeout parameter in \ref llcc68_set_rx_with_timeout_in_rtc_step to set the chip in reception until a
 * reception occurs
 */
#define LLCC68_RX_SINGLE_MODE 0x00000000

/**
 * @brief Timeout parameter in @ref llcc68_set_rx_with_timeout_in_rtc_step to launch a continuous reception
 */
#define LLCC68_RX_CONTINUOUS 0x00FFFFFF

/**
 * @brief Over-current protection default value after @ref llcc68_set_pa_cfg is called with @ref device_sel set to 1
 */
#define LLCC68_OCP_PARAM_VALUE_60_MA 0x18

/**
 * @brief Over-current protection default value after @ref llcc68_set_pa_cfg is called with @ref device_sel set to 0
 */
#define LLCC68_OCP_PARAM_VALUE_140_MA 0x38

/**
 * @brief XTA and XTB trimming capacitor default value after the chip entered @ref LLCC68_STANDBY_CFG_XOSC mode
 */
#define LLCC68_XTAL_TRIMMING_CAPACITOR_DEFAULT_VALUE_STDBY_XOSC 0x12

/**
 * @brief  Maximum value for parameter nb_of_symbs in @ref llcc68_set_lora_symb_nb_timeout
 */
#define LLCC68_MAX_LORA_SYMB_NUM_TIMEOUT 248

/**
 * @brief Maximum number of register that can be added to the retention list
 */
#define LLCC68_MAX_NB_REG_IN_RETENTION 4

/*!
 * @brief Frequency step in MHz used to compute the image calibration parameter
 *
 * @see llcc68_cal_img_in_mhz
 */
#define LLCC68_IMAGE_CALIBRATION_STEP_IN_MHZ 4

#define LLCC68_CHIP_MODES_POS ( 4U )
#define LLCC68_CHIP_MODES_MASK ( 0x07UL << LLCC68_CHIP_MODES_POS )

#define LLCC68_CMD_STATUS_POS ( 1U )
#define LLCC68_CMD_STATUS_MASK ( 0x07UL << LLCC68_CMD_STATUS_POS )

#define LLCC68_GFSK_RX_STATUS_PKT_SENT_POS ( 0U )
#define LLCC68_GFSK_RX_STATUS_PKT_SENT_MASK ( 0x01UL << LLCC68_GFSK_RX_STATUS_PKT_SENT_POS )

#define LLCC68_GFSK_RX_STATUS_PKT_RECEIVED_POS ( 1U )
#define LLCC68_GFSK_RX_STATUS_PKT_RECEIVED_MASK ( 0x01UL << LLCC68_GFSK_RX_STATUS_PKT_RECEIVED_POS )

#define LLCC68_GFSK_RX_STATUS_ABORT_ERROR_POS ( 2U )
#define LLCC68_GFSK_RX_STATUS_ABORT_ERROR_MASK ( 0x01UL << LLCC68_GFSK_RX_STATUS_ABORT_ERROR_POS )

#define LLCC68_GFSK_RX_STATUS_LENGTH_ERROR_POS ( 3U )
#define LLCC68_GFSK_RX_STATUS_LENGTH_ERROR_MASK ( 0x01UL << LLCC68_GFSK_RX_STATUS_LENGTH_ERROR_POS )

#define LLCC68_GFSK_RX_STATUS_CRC_ERROR_POS ( 4U )
#define LLCC68_GFSK_RX_STATUS_CRC_ERROR_MASK ( 0x01UL << LLCC68_GFSK_RX_STATUS_CRC_ERROR_POS )

#define LLCC68_GFSK_RX_STATUS_ADRS_ERROR_POS ( 5U )
#define LLCC68_GFSK_RX_STATUS_ADRS_ERROR_MASK ( 0x01UL << LLCC68_GFSK_RX_STATUS_ADRS_ERROR_POS )

/*!
 * \brief Ramp-up delay for the power amplifier
 *
 * This parameter configures the delay to fine tune the ramp-up time of the power amplifier for BPSK operation.
 */
enum
{
    LLCC68_SIGFOX_DBPSK_RAMP_UP_TIME_DEFAULT = 0x0000,  //!< No optimization
    LLCC68_SIGFOX_DBPSK_RAMP_UP_TIME_100_BPS = 0x370F,  //!< Ramp-up optimization for 100bps
    LLCC68_SIGFOX_DBPSK_RAMP_UP_TIME_600_BPS = 0x092F,  //!< Ramp-up optimization for 600bps
};

/*!
 * \brief Ramp-down delay for the power amplifier
 *
 * This parameter configures the delay to fine tune the ramp-down time of the power amplifier for BPSK operation.
 */
enum
{
    LLCC68_SIGFOX_DBPSK_RAMP_DOWN_TIME_DEFAULT = 0x0000,  //!< No optimization
    LLCC68_SIGFOX_DBPSK_RAMP_DOWN_TIME_100_BPS = 0x1D70,  //!< Ramp-down optimization for 100bps
    LLCC68_SIGFOX_DBPSK_RAMP_DOWN_TIME_600_BPS = 0x04E1,  //!< Ramp-down optimization for 600bps
};

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC TYPES ------------------------------------------------------------
 */

/**
 * @brief LLCC68 APIs return status enumeration definition
 */
typedef enum llcc68_status_e
{
    LLCC68_STATUS_OK = 0,
    LLCC68_STATUS_UNSUPPORTED_FEATURE,
    LLCC68_STATUS_UNKNOWN_VALUE,
    LLCC68_STATUS_ERROR,
} llcc68_status_t;

/**
 * @brief LLCC68 sleep mode configurations definition
 */
typedef enum llcc68_sleep_cfgs_e
{
    LLCC68_SLEEP_CFG_COLD_START = ( 0 << 2 ),
    LLCC68_SLEEP_CFG_WARM_START = ( 1 << 2 ),
} llcc68_sleep_cfgs_t;

/**
 * @brief LLCC68 standby modes enumeration definition
 */
typedef enum llcc68_standby_cfgs_e
{
    LLCC68_STANDBY_CFG_RC   = 0x00,
    LLCC68_STANDBY_CFG_XOSC = 0x01,
} llcc68_standby_cfgs_t;

typedef uint8_t llcc68_standby_cfg_t;

/**
 * @brief LLCC68 power regulator modes enumeration definition
 */
typedef enum llcc68_reg_mods_e
{
    LLCC68_REG_MODE_LDO  = 0x00,  // default
    LLCC68_REG_MODE_DCDC = 0x01,
} llcc68_reg_mod_t;

/**
 * @brief LLCC68 power amplifier configuration parameters structure definition
 */
typedef struct llcc68_pa_cfg_params_s
{
    uint8_t pa_duty_cycle;
    uint8_t hp_max;
    uint8_t device_sel;
    uint8_t pa_lut;
} llcc68_pa_cfg_params_t;

/**
 * @brief LLCC68 fallback modes enumeration definition
 */
typedef enum llcc68_fallback_modes_e
{
    LLCC68_FALLBACK_STDBY_RC   = 0x20,
    LLCC68_FALLBACK_STDBY_XOSC = 0x30,
    LLCC68_FALLBACK_FS         = 0x40,
} llcc68_fallback_modes_t;

/**
 * @brief LLCC68 interrupt masks enumeration definition
 */
enum llcc68_irq_masks_e
{
    LLCC68_IRQ_NONE              = ( 0 << 0 ),
    LLCC68_IRQ_TX_DONE           = ( 1 << 0 ),
    LLCC68_IRQ_RX_DONE           = ( 1 << 1 ),
    LLCC68_IRQ_PREAMBLE_DETECTED = ( 1 << 2 ),
    LLCC68_IRQ_SYNC_WORD_VALID   = ( 1 << 3 ),
    LLCC68_IRQ_HEADER_VALID      = ( 1 << 4 ),
    LLCC68_IRQ_HEADER_ERROR      = ( 1 << 5 ),
    LLCC68_IRQ_CRC_ERROR         = ( 1 << 6 ),
    LLCC68_IRQ_CAD_DONE          = ( 1 << 7 ),
    LLCC68_IRQ_CAD_DETECTED      = ( 1 << 8 ),
    LLCC68_IRQ_TIMEOUT           = ( 1 << 9 ),
    LLCC68_IRQ_ALL               = LLCC68_IRQ_TX_DONE | LLCC68_IRQ_RX_DONE | LLCC68_IRQ_PREAMBLE_DETECTED |
                     LLCC68_IRQ_SYNC_WORD_VALID | LLCC68_IRQ_HEADER_VALID | LLCC68_IRQ_HEADER_ERROR |
                     LLCC68_IRQ_CRC_ERROR | LLCC68_IRQ_CAD_DONE | LLCC68_IRQ_CAD_DETECTED | LLCC68_IRQ_TIMEOUT,
};

typedef uint16_t llcc68_irq_mask_t;

/**
 * @brief Calibration settings
 */
enum llcc68_cal_mask_e
{
    LLCC68_CAL_RC64K      = ( 1 << 0 ),
    LLCC68_CAL_RC13M      = ( 1 << 1 ),
    LLCC68_CAL_PLL        = ( 1 << 2 ),
    LLCC68_CAL_ADC_PULSE  = ( 1 << 3 ),
    LLCC68_CAL_ADC_BULK_N = ( 1 << 4 ),
    LLCC68_CAL_ADC_BULK_P = ( 1 << 5 ),
    LLCC68_CAL_IMAGE      = ( 1 << 6 ),
    LLCC68_CAL_ALL        = LLCC68_CAL_RC64K | LLCC68_CAL_RC13M | LLCC68_CAL_PLL | LLCC68_CAL_ADC_PULSE |
                     LLCC68_CAL_ADC_BULK_N | LLCC68_CAL_ADC_BULK_P | LLCC68_CAL_IMAGE,
};

typedef uint8_t llcc68_cal_mask_t;

/**
 * @brief LLCC68 TCXO control voltages enumeration definition
 */
typedef enum llcc68_tcxo_ctrl_voltages_e
{
    LLCC68_TCXO_CTRL_1_6V = 0x00,
    LLCC68_TCXO_CTRL_1_7V = 0x01,
    LLCC68_TCXO_CTRL_1_8V = 0x02,
    LLCC68_TCXO_CTRL_2_2V = 0x03,
    LLCC68_TCXO_CTRL_2_4V = 0x04,
    LLCC68_TCXO_CTRL_2_7V = 0x05,
    LLCC68_TCXO_CTRL_3_0V = 0x06,
    LLCC68_TCXO_CTRL_3_3V = 0x07,
} llcc68_tcxo_ctrl_voltages_t;

/**
 * @brief LLCC68 packet types enumeration definition
 */
typedef enum llcc68_pkt_types_e
{
    LLCC68_PKT_TYPE_GFSK    = 0x00,
    LLCC68_PKT_TYPE_LORA    = 0x01,
    LLCC68_PKT_TYPE_BPSK    = 0x02,
} llcc68_pkt_type_t;

/**
 * @brief LLCC68 power amplifier ramp-up timings enumeration definition
 */
typedef enum llcc68_ramp_time_e
{
    LLCC68_RAMP_10_US   = 0x00,
    LLCC68_RAMP_20_US   = 0x01,
    LLCC68_RAMP_40_US   = 0x02,
    LLCC68_RAMP_80_US   = 0x03,
    LLCC68_RAMP_200_US  = 0x04,
    LLCC68_RAMP_800_US  = 0x05,
    LLCC68_RAMP_1700_US = 0x06,
    LLCC68_RAMP_3400_US = 0x07,
} llcc68_ramp_time_t;

/**
 * @brief LLCC68 GFSK modulation shaping enumeration definition
 */
typedef enum llcc68_gfsk_pulse_shape_e
{
    LLCC68_GFSK_PULSE_SHAPE_OFF   = 0x00,
    LLCC68_GFSK_PULSE_SHAPE_BT_03 = 0x08,
    LLCC68_GFSK_PULSE_SHAPE_BT_05 = 0x09,
    LLCC68_GFSK_PULSE_SHAPE_BT_07 = 0x0A,
    LLCC68_GFSK_PULSE_SHAPE_BT_1  = 0x0B,
} llcc68_gfsk_pulse_shape_t;

/**
 * @brief LLCC68 BPSK modulation shaping enumeration definition
 */
typedef enum
{
    LLCC68_DBPSK_PULSE_SHAPE = 0x16,  //!< Double OSR / RRC / BT 0.7
} llcc68_bpsk_pulse_shape_t;

/**
 * @brief LLCC68 GFSK Rx bandwidth enumeration definition
 */
typedef enum llcc68_gfsk_bw_e
{
    LLCC68_GFSK_BW_4800   = 0x1F,
    LLCC68_GFSK_BW_5800   = 0x17,
    LLCC68_GFSK_BW_7300   = 0x0F,
    LLCC68_GFSK_BW_9700   = 0x1E,
    LLCC68_GFSK_BW_11700  = 0x16,
    LLCC68_GFSK_BW_14600  = 0x0E,
    LLCC68_GFSK_BW_19500  = 0x1D,
    LLCC68_GFSK_BW_23400  = 0x15,
    LLCC68_GFSK_BW_29300  = 0x0D,
    LLCC68_GFSK_BW_39000  = 0x1C,
    LLCC68_GFSK_BW_46900  = 0x14,
    LLCC68_GFSK_BW_58600  = 0x0C,
    LLCC68_GFSK_BW_78200  = 0x1B,
    LLCC68_GFSK_BW_93800  = 0x13,
    LLCC68_GFSK_BW_117300 = 0x0B,
    LLCC68_GFSK_BW_156200 = 0x1A,
    LLCC68_GFSK_BW_187200 = 0x12,
    LLCC68_GFSK_BW_234300 = 0x0A,
    LLCC68_GFSK_BW_312000 = 0x19,
    LLCC68_GFSK_BW_373600 = 0x11,
    LLCC68_GFSK_BW_467000 = 0x09,
} llcc68_gfsk_bw_t;

/**
 * @brief LLCC68 GFSK modulation parameters structure definition
 */
typedef struct llcc68_mod_params_gfsk_s
{
    uint32_t                  br_in_bps;
    uint32_t                  fdev_in_hz;
    llcc68_gfsk_pulse_shape_t pulse_shape;
    llcc68_gfsk_bw_t          bw_dsb_param;
} llcc68_mod_params_gfsk_t;

/**
 * @brief Modulation configuration for BPSK packet
 */
typedef struct llcc68_mod_params_bpsk_s
{
    uint32_t                  br_in_bps;    //!< BPSK bitrate [bit/s]
    llcc68_bpsk_pulse_shape_t pulse_shape;  //!< BPSK pulse shape
} llcc68_mod_params_bpsk_t;

/**
 * @brief LLCC68 LoRa spreading factor enumeration definition
 */
typedef enum llcc68_lora_sf_e
{
    LLCC68_LORA_SF5  = 0x05,
    LLCC68_LORA_SF6  = 0x06,
    LLCC68_LORA_SF7  = 0x07,
    LLCC68_LORA_SF8  = 0x08,
    LLCC68_LORA_SF9  = 0x09,
    LLCC68_LORA_SF10 = 0x0A,
    LLCC68_LORA_SF11 = 0x0B,
} llcc68_lora_sf_t;

/**
 * @brief LLCC68 LoRa bandwidth enumeration definition
 */
typedef enum llcc68_lora_bw_e
{
    LLCC68_LORA_BW_500 = 6,
    LLCC68_LORA_BW_250 = 5,
    LLCC68_LORA_BW_125 = 4,
    LLCC68_LORA_BW_062 = 3,
    LLCC68_LORA_BW_041 = 10,
    LLCC68_LORA_BW_031 = 2,
    LLCC68_LORA_BW_020 = 9,
    LLCC68_LORA_BW_015 = 1,
    LLCC68_LORA_BW_010 = 8,
    LLCC68_LORA_BW_007 = 0,
} llcc68_lora_bw_t;

/**
 * @brief LLCC68 LoRa coding rate enumeration definition
 */
typedef enum llcc68_lora_cr_e
{
    LLCC68_LORA_CR_4_5 = 0x01,
    LLCC68_LORA_CR_4_6 = 0x02,
    LLCC68_LORA_CR_4_7 = 0x03,
    LLCC68_LORA_CR_4_8 = 0x04,
} llcc68_lora_cr_t;

/**
 * @brief LLCC68 LoRa modulation parameters structure definition
 */
typedef struct llcc68_mod_params_lora_s
{
    llcc68_lora_sf_t sf;    //!< LoRa Spreading Factor
    llcc68_lora_bw_t bw;    //!< LoRa Bandwidth
    llcc68_lora_cr_t cr;    //!< LoRa Coding Rate
    uint8_t          ldro;  //!< Low DataRate Optimization configuration
} llcc68_mod_params_lora_t;

/**
 * @brief LLCC68 GFSK preamble length Rx detection size enumeration definition
 */
typedef enum llcc68_gfsk_preamble_detector_e
{
    LLCC68_GFSK_PREAMBLE_DETECTOR_OFF        = 0x00,
    LLCC68_GFSK_PREAMBLE_DETECTOR_MIN_8BITS  = 0x04,
    LLCC68_GFSK_PREAMBLE_DETECTOR_MIN_16BITS = 0x05,
    LLCC68_GFSK_PREAMBLE_DETECTOR_MIN_24BITS = 0x06,
    LLCC68_GFSK_PREAMBLE_DETECTOR_MIN_32BITS = 0x07,
} llcc68_gfsk_preamble_detector_t;

/**
 * @brief LLCC68 GFSK address filtering configuration enumeration definition
 */
typedef enum llcc68_gfsk_address_filtering_e
{
    LLCC68_GFSK_ADDRESS_FILTERING_DISABLE                      = 0x00,
    LLCC68_GFSK_ADDRESS_FILTERING_NODE_ADDRESS                 = 0x01,
    LLCC68_GFSK_ADDRESS_FILTERING_NODE_AND_BROADCAST_ADDRESSES = 0x02,
} llcc68_gfsk_address_filtering_t;

/**
 * @brief LLCC68 GFSK packet length enumeration definition
 */
typedef enum llcc68_gfsk_pkt_len_modes_e
{
    LLCC68_GFSK_PKT_FIX_LEN = 0x00,  //!< The packet length is known on both sides, no header included
    LLCC68_GFSK_PKT_VAR_LEN = 0x01,  //!< The packet length is variable, header included
} llcc68_gfsk_pkt_len_modes_t;

/**
 * @brief LLCC68 GFSK CRC type enumeration definition
 */
typedef enum llcc68_gfsk_crc_types_e
{
    LLCC68_GFSK_CRC_OFF         = 0x01,
    LLCC68_GFSK_CRC_1_BYTE      = 0x00,
    LLCC68_GFSK_CRC_2_BYTES     = 0x02,
    LLCC68_GFSK_CRC_1_BYTE_INV  = 0x04,
    LLCC68_GFSK_CRC_2_BYTES_INV = 0x06,
} llcc68_gfsk_crc_types_t;

/**
 * @brief LLCC68 GFSK whitening control enumeration definition
 */
typedef enum llcc68_gfsk_dc_free_e
{
    LLCC68_GFSK_DC_FREE_OFF       = 0x00,
    LLCC68_GFSK_DC_FREE_WHITENING = 0x01,
} llcc68_gfsk_dc_free_t;

/**
 * @brief LLCC68 LoRa packet length enumeration definition
 */
typedef enum llcc68_lora_pkt_len_modes_e
{
    LLCC68_LORA_PKT_EXPLICIT = 0x00,  //!< Header included in the packet
    LLCC68_LORA_PKT_IMPLICIT = 0x01,  //!< Header not included in the packet
} llcc68_lora_pkt_len_modes_t;

/**
 * @brief LLCC68 LoRa packet parameters structure definition
 */
typedef struct llcc68_pkt_params_lora_s
{
    uint16_t                    preamble_len_in_symb;  //!< Preamble length in symbols
    llcc68_lora_pkt_len_modes_t header_type;           //!< Header type
    uint8_t                     pld_len_in_bytes;      //!< Payload length in bytes
    bool                        crc_is_on;             //!< CRC activation
    bool                        invert_iq_is_on;       //!< IQ polarity setup
} llcc68_pkt_params_lora_t;

/**
 * @brief LLCC68 GFSK packet parameters structure definition
 */
typedef struct llcc68_pkt_params_gfsk_s
{
    uint16_t                        preamble_len_in_bits;   //!< Preamble length in bits
    llcc68_gfsk_preamble_detector_t preamble_detector;      //!< Preamble detection length
    uint8_t                         sync_word_len_in_bits;  //!< Sync word length in bits
    llcc68_gfsk_address_filtering_t address_filtering;      //!< Address filtering configuration
    llcc68_gfsk_pkt_len_modes_t     header_type;            //!< Header type
    uint8_t                         pld_len_in_bytes;       //!< Payload length in bytes
    llcc68_gfsk_crc_types_t         crc_type;               //!< CRC type configuration
    llcc68_gfsk_dc_free_t           dc_free;                //!< Whitening configuration
} llcc68_pkt_params_gfsk_t;

/**
 * @brief LLCC68 BPSK packet parameters structure definition
 */
typedef struct llcc68_pkt_params_bpsk_s
{
    uint8_t  pld_len_in_bytes;  //!< Payload length [bytes]
    uint16_t ramp_up_delay;     //!< Delay to fine tune ramp-up time, if non-zero
    uint16_t ramp_down_delay;   //!< Delay to fine tune ramp-down time, if non-zero
    uint16_t pld_len_in_bits;   //!< If non-zero, used to ramp down PA before end of a payload with length that is not a
                                //!< multiple of 8
} llcc68_pkt_params_bpsk_t;

/**
 * @brief LLCC68 LoRa CAD number of symbols enumeration definition
 *
 * @note Represents the number of symbols to be used for a CAD operation
 */
typedef enum llcc68_cad_symbs_e
{
    LLCC68_CAD_01_SYMB = 0x00,
    LLCC68_CAD_02_SYMB = 0x01,
    LLCC68_CAD_04_SYMB = 0x02,
    LLCC68_CAD_08_SYMB = 0x03,
    LLCC68_CAD_16_SYMB = 0x04,
} llcc68_cad_symbs_t;

/**
 * @brief LLCC68 LoRa CAD exit modes enumeration definition
 *
 * @note Represents the action to be performed after a CAD is done
 */
typedef enum llcc68_cad_exit_modes_e
{
    LLCC68_CAD_ONLY = 0x00,
    LLCC68_CAD_RX   = 0x01,
    LLCC68_CAD_LBT  = 0x10,
} llcc68_cad_exit_modes_t;

/**
 * @brief LLCC68 CAD parameters structure definition
 */
typedef struct llcc68_cad_param_s
{
    llcc68_cad_symbs_t      cad_symb_nb;      //!< CAD number of symbols
    uint8_t                 cad_detect_peak;  //!< CAD peak detection
    uint8_t                 cad_detect_min;   //!< CAD minimum detection
    llcc68_cad_exit_modes_t cad_exit_mode;    //!< CAD exit mode
    uint32_t                cad_timeout;      //!< CAD timeout value
} llcc68_cad_params_t;

/**
 * @brief LLCC68 chip mode enumeration definition
 */
typedef enum llcc68_chip_modes_e
{
    LLCC68_CHIP_MODE_UNUSED    = 0,
    LLCC68_CHIP_MODE_RFU       = 1,
    LLCC68_CHIP_MODE_STBY_RC   = 2,
    LLCC68_CHIP_MODE_STBY_XOSC = 3,
    LLCC68_CHIP_MODE_FS        = 4,
    LLCC68_CHIP_MODE_RX        = 5,
    LLCC68_CHIP_MODE_TX        = 6,
} llcc68_chip_modes_t;

/**
 * @brief LLCC68 command status enumeration definition
 */
typedef enum llcc68_cmd_status_e
{
    LLCC68_CMD_STATUS_RESERVED          = 0,
    LLCC68_CMD_STATUS_RFU               = 1,
    LLCC68_CMD_STATUS_DATA_AVAILABLE    = 2,
    LLCC68_CMD_STATUS_CMD_TIMEOUT       = 3,
    LLCC68_CMD_STATUS_CMD_PROCESS_ERROR = 4,
    LLCC68_CMD_STATUS_CMD_EXEC_FAILURE  = 5,
    LLCC68_CMD_STATUS_CMD_TX_DONE       = 6,
} llcc68_cmd_status_t;

/**
 * @brief LLCC68 chip status structure definition
 */
typedef struct llcc68_chip_status_s
{
    llcc68_cmd_status_t cmd_status;  //!< Previous command status
    llcc68_chip_modes_t chip_mode;   //!< Current chip mode
} llcc68_chip_status_t;

/**
 * @brief LLCC68 RX buffer status structure definition
 */
typedef struct llcc68_rx_buffer_status_s
{
    uint8_t pld_len_in_bytes;      //!< Number of bytes available in the buffer
    uint8_t buffer_start_pointer;  //!< Position of the first byte in the buffer
} llcc68_rx_buffer_status_t;

typedef struct llcc68_rx_status_gfsk_s
{
    bool pkt_sent;
    bool pkt_received;
    bool abort_error;
    bool length_error;
    bool crc_error;
    bool adrs_error;
} llcc68_rx_status_gfsk_t;

/**
 * @brief LLCC68 GFSK packet status structure definition
 */
typedef struct llcc68_pkt_status_gfsk_s
{
    llcc68_rx_status_gfsk_t rx_status;
    int8_t                  rssi_sync;  //!< The RSSI measured on last packet
    int8_t                  rssi_avg;   //!< The averaged RSSI
} llcc68_pkt_status_gfsk_t;

/**
 * @brief LLCC68 LoRa packet status structure definition
 */
typedef struct llcc68_pkt_status_lora_s
{
    int8_t rssi_pkt_in_dbm;         //!< RSSI of the last packet
    int8_t snr_pkt_in_db;           //!< SNR of the last packet
    int8_t signal_rssi_pkt_in_dbm;  //!< Estimation of RSSI (after despreading)
} llcc68_pkt_status_lora_t;

/**
 * @brief LLCC68 GFSK reception statistics structure definition
 */
typedef struct llcc68_stats_gfsk_s
{
    uint16_t nb_pkt_received;
    uint16_t nb_pkt_crc_error;
    uint16_t nb_pkt_len_error;
} llcc68_stats_gfsk_t;

/**
 * @brief LLCC68 LoRa reception statistics structure definition
 */
typedef struct llcc68_stats_lora_s
{
    uint16_t nb_pkt_received;
    uint16_t nb_pkt_crc_error;
    uint16_t nb_pkt_header_error;
} llcc68_stats_lora_t;

/**
 * @brief LLCC68 errors enumeration definition
 */
enum llcc68_errors_e
{
    LLCC68_ERRORS_RC64K_CALIBRATION = ( 1 << 0 ),
    LLCC68_ERRORS_RC13M_CALIBRATION = ( 1 << 1 ),
    LLCC68_ERRORS_PLL_CALIBRATION   = ( 1 << 2 ),
    LLCC68_ERRORS_ADC_CALIBRATION   = ( 1 << 3 ),
    LLCC68_ERRORS_IMG_CALIBRATION   = ( 1 << 4 ),
    LLCC68_ERRORS_XOSC_START        = ( 1 << 5 ),
    LLCC68_ERRORS_PLL_LOCK          = ( 1 << 6 ),
    LLCC68_ERRORS_PA_RAMP           = ( 1 << 8 ),
};

typedef uint16_t llcc68_errors_mask_t;

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC FUNCTIONS PROTOTYPES ---------------------------------------------
 */

//
// Operational Modes Functions
//

/**
 * @brief Set the chip in sleep mode
 *
 * @param [in]  context Chip implementation context
 * @param [in]  cfg Sleep mode configuration
 *
 * @returns Operation status
 */
llcc68_status_t llcc68_set_sleep( const void* context, const llcc68_sleep_cfgs_t cfg );

/**
 * @brief Set the chip in stand-by mode
 *
 * @param [in]  context Chip implementation context
 * @param [in]  cfg Stand-by mode configuration
 *
 * @returns Operation status
 */
llcc68_status_t llcc68_set_standby( const void* context, const llcc68_standby_cfg_t cfg );

/**
 * @brief Set the chip in frequency synthesis mode
 *
 * @param [in] context Chip implementation context
 *
 * @returns Operation status
 */
llcc68_status_t llcc68_set_fs( const void* context );

/**
 * @brief Set the chip in transmission mode
 *
 * @remark The packet type shall be configured with @ref llcc68_set_pkt_type before using this command.
 *
 * @remark By default, the chip returns automatically to standby RC mode as soon as the packet is sent or if the packet
 * has not been completely transmitted before the timeout. This behavior can be altered by @ref
 * llcc68_set_rx_tx_fallback_mode.
 *
 * @remark If the timeout argument is 0, then no timeout is used.
 *
 * @param [in] context Chip implementation context
 * @param [in] timeout_in_ms The timeout configuration in millisecond for Tx operation
 *
 * @returns Operation status
 */
llcc68_status_t llcc68_set_tx( const void* context, const uint32_t timeout_in_ms );

/**
 * @brief Set the chip in transmission mode
 *
 * @remark The packet type shall be configured with @ref llcc68_set_pkt_type before using this command.
 *
 * @remark By default, the chip returns automatically to standby RC mode as soon as the packet is sent or if the packet
 * has not been completely transmitted before the timeout. This behavior can be altered by @ref
 * llcc68_set_rx_tx_fallback_mode.
 *
 * @remark The timeout duration can be computed with the formula:
 * \f$ timeout\_duration\_ms = timeout_in_rtc_step \times * \frac{1}{64} \f$
 *
 * @remark Maximal value is LLCC68_MAX_TIMEOUT_IN_RTC_STEP (i.e. 262 143 ms)
 *
 * @remark If the timeout argument is 0, then no timeout is used.
 *
 * @param [in] context Chip implementation context
 * @param [in] timeout_in_rtc_step The timeout configuration for Tx operation
 *
 * @returns Operation status
 */
llcc68_status_t llcc68_set_tx_with_timeout_in_rtc_step( const void* context, const uint32_t timeout_in_rtc_step );

/**
 * @brief Set the chip in reception mode
 *
 * @remark The packet type shall be configured with @ref llcc68_set_pkt_type before using this command.
 *
 * @remark By default, the chip returns automatically to standby RC mode as soon as a packet is received
 * or if no packet has been received before the timeout. This behavior can be altered by @ref
 * llcc68_set_rx_tx_fallback_mode.
 *
 * @remark The timeout argument can have the following special values:
 *
 * | Special values        | Meaning                                                                               |
 * | ----------------------| --------------------------------------------------------------------------------------|
 * | LLCC68_RX_SINGLE_MODE | Single: the chip stays in RX mode until a reception occurs, then switch to standby RC |
 *
 * @remark Refer to @ref llcc68_handle_rx_done
 *
 * @param [in] context Chip implementation context
 * @param [in] timeout_in_ms The timeout configuration in millisecond for Rx operation
 *
 * @returns Operation status
 */
llcc68_status_t llcc68_set_rx( const void* context, const uint32_t timeout_in_ms );

/**
 * @brief Set the chip in reception mode
 *
 * @remark The packet type shall be configured with @ref llcc68_set_pkt_type before using this command.
 *
 * @remark By default, the chip returns automatically to standby RC mode as soon as a packet is received
 * or if no packet has been received before the timeout. This behavior can be altered by @ref
 * llcc68_set_rx_tx_fallback_mode.
 *
 * @remark The timeout duration is obtained by:
 * \f$ timeout\_duration\_ms = timeout_in_rtc_step \times \frac{1}{64} \f$
 *
 * @remark Maximal timeout value is LLCC68_MAX_TIMEOUT_IN_RTC_STEP (i.e. 262 143 ms).
 *
 * @remark The timeout argument can have the following special values:
 *
 * | Special values        | Meaning                                                                               |
 * | ----------------------| --------------------------------------------------------------------------------------|
 * | LLCC68_RX_SINGLE_MODE | Single: the chip stays in RX mode until a reception occurs, then switch to standby RC |
 * | LLCC68_RX_CONTINUOUS  | Continuous: the chip stays in RX mode even after reception of a packet                |
 *
 * @remark Refer to @ref llcc68_handle_rx_done
 *
 * @param [in] context Chip implementation context
 * @param [in] timeout_in_rtc_step The timeout configuration for Rx operation
 *
 * @returns Operation status
 */
llcc68_status_t llcc68_set_rx_with_timeout_in_rtc_step( const void* context, const uint32_t timeout_in_rtc_step );

/**
 * @brief Configure the event on which the Rx timeout is stopped
 *
 * @remark The two options are:
 *   - Syncword / Header detection (default)
 *   - Preamble detection
 *
 * @param [in] context Chip implementation context
 * @param [in] enable If true, the timer stops on Syncword / Header detection
 *
 * @returns Operation status
 */
llcc68_status_t llcc68_stop_timer_on_preamble( const void* context, const bool enable );

/**
 * @brief Set the chip in reception mode with duty cycling
 *
 * @param [in] context Chip implementation context
 * @param [in] rx_time_in_ms The timeout of Rx period - in millisecond
 * @param [in] sleep_time_in_ms The length of sleep period - in millisecond
 *
 * @returns Operation status
 */
llcc68_status_t llcc68_set_rx_duty_cycle( const void* context, const uint32_t rx_time_in_ms,
                                          const uint32_t sleep_time_in_ms );

/**
 * @brief Set the chip in reception mode with duty cycling
 *
 * @remark The Rx mode duration is defined by:
 * \f$ rx\_duration\_ms = rx_time \times \frac{1}{64} \f$
 *
 * @remark The sleep mode duration is defined by:
 * \f$ sleep\_duration\_ms = sleep_time \times \frac{1}{64} \f$
 *
 * @remark Maximal timeout value is 0xFFFFFF (i.e. 511 seconds).
 *
 * @param [in] context Chip implementation context
 * @param [in] rx_time The timeout of Rx period
 * @param [in] sleep_time The length of sleep period
 *
 * @returns Operation status
 */
llcc68_status_t llcc68_set_rx_duty_cycle_with_timings_in_rtc_step( const void*    context,
                                                                   const uint32_t rx_time_in_rtc_step,
                                                                   const uint32_t sleep_time_in_rtc_step );

/**
 * @brief Set the chip in CAD (Channel Activity Detection) mode
 *
 * @remark The LoRa packet type shall be selected with @ref llcc68_set_pkt_type before this function is called.
 *
 * @remark The fallback mode is configured with @ref llcc68_set_cad_params.
 *
 * @param [in] context Chip implementation context
 *
 * @returns Operation status
 */
llcc68_status_t llcc68_set_cad( const void* context );

/**
 * @brief Set the chip in Tx continuous wave (RF tone).
 *
 * @remark The packet type shall be configured with @ref llcc68_set_pkt_type before using this command.
 *
 * @param [in] context Chip implementation context
 *
 * @returns Operation status
 */
llcc68_status_t llcc68_set_tx_cw( const void* context );

/**
 * @brief Set the chip in Tx infinite preamble (modulated signal).
 *
 * @remark The packet type shall be configured with @ref llcc68_set_pkt_type before using this command.
 *
 * @param [in] context Chip implementation context
 *
 * @returns Operation status
 */
llcc68_status_t llcc68_set_tx_infinite_preamble( const void* context );

/**
 * @brief Configure the regulator mode to be used
 *
 * @remark This function shall be called to set the correct regulator mode, depending on the usage of LDO or DC/DC on
 * the PCB implementation.
 *
 * @param [in] context Chip implementation context
 * @param [in] mode Regulator mode configuration
 *
 * @returns Operation status
 */
llcc68_status_t llcc68_set_reg_mode( const void* context, const llcc68_reg_mod_t mode );

/**
 * @brief Perform the calibration of the requested blocks
 *
 * @remark This function shall only be called in stand-by RC mode
 *
 * @remark The chip will return to stand-by RC mode on exit. Potential calibration issues can be read out with @ref
 * llcc68_get_device_errors command.
 *
 * @param [in] context Chip implementation context
 * @param [in] param Mask holding the blocks to be calibrated
 *
 * @returns Operation status
 */
llcc68_status_t llcc68_cal( const void* context, const llcc68_cal_mask_t param );

/**
 * @brief Launch an image calibration valid for all frequencies inside an interval, in steps
 *
 * @param [in] context Chip implementation context
 * @param [in] freq1 Image calibration interval lower bound, in steps
 * @param [in] freq2 Image calibration interval upper bound, in steps
 *
 * @remark freq1 must be less than or equal to freq2
 *
 * @returns Operation status
 */
llcc68_status_t llcc68_cal_img( const void* context, const uint8_t freq1, const uint8_t freq2 );

/**
 * @brief Launch an image calibration valid for all frequencies inside an interval, in MHz
 *
 * @param [in] context Chip implementation context
 * @param [in] freq1_in_mhz Image calibration interval lower bound, in MHz
 * @param [in] freq2_in_mhz Image calibration interval upper bound, in MHz
 *
 * @remark freq1_in_mhz must be less than or equal to freq2_in_mhz
 *
 * @returns Operation status
 */
llcc68_status_t llcc68_cal_img_in_mhz( const void* context, const uint16_t freq1_in_mhz, const uint16_t freq2_in_mhz );

/**
 * @brief Configure the PA (Power Amplifier)
 *
 * @remark The parameters depend on the chip being used
 *
 * @param [in] context Chip implementation context
 * @param [in] params Power amplifier configuration parameters
 *
 * @returns Operation status
 */
llcc68_status_t llcc68_set_pa_cfg( const void* context, const llcc68_pa_cfg_params_t* params );

/**
 * @brief Set chip mode to be used after successful transmission or reception.
 *
 * @remark This setting is not taken into account during Rx Duty Cycle mode or Auto TxRx.
 *
 * @param [in] context Chip implementation context
 * @param [in] fallback_mode Selected fallback mode
 *
 * @returns Operation status
 */
llcc68_status_t llcc68_set_rx_tx_fallback_mode( const void* context, const llcc68_fallback_modes_t fallback_mode );

//
// Registers and Buffer Access
//

/**
 * @brief Write data into register memory space.
 *
 * @param [in] context Chip implementation context
 * @param [in] address The register memory address to start writing operation
 * @param [in] buffer The buffer of bytes to write into memory
 * @param [in] size Number of bytes to write into memory, starting from address
 *
 * @returns Operation status
 *
 * @see llcc68_read_register
 */
llcc68_status_t llcc68_write_register( const void* context, const uint16_t address, const uint8_t* buffer,
                                       const uint8_t size );

/**
 * @brief Read data from register memory space.
 *
 * @param [in] context Chip implementation context
 * @param [in] address The register memory address to start reading operation
 * @param [in] buffer The buffer of bytes to be filled with data from registers
 * @param [in] size Number of bytes to read from memory, starting from address
 *
 * @returns Operation status
 *
 * @see llcc68_write_register
 */
llcc68_status_t llcc68_read_register( const void* context, const uint16_t address, uint8_t* buffer,
                                      const uint8_t size );

/**
 * @brief Write data into radio Tx buffer memory space.
 *
 * @param [in] context Chip implementation context
 * @param [in] offset Start address in the Tx buffer of the chip
 * @param [in] buffer The buffer of bytes to write into radio buffer
 * @param [in] size The number of bytes to write into Tx radio buffer
 *
 * @returns Operation status
 *
 * @see llcc68_read_buffer
 */
llcc68_status_t llcc68_write_buffer( const void* context, const uint8_t offset, const uint8_t* buffer,
                                     const uint8_t size );

/**
 * @brief Read data from radio Rx buffer memory space.
 *
 * @param [in] context Chip implementation context
 * @param [in] offset Start address in the Rx buffer of the chip
 * @param [in] buffer The buffer of bytes to be filled with content from Rx radio buffer
 * @param [in] size The number of bytes to read from the Rx radio buffer
 *
 * @returns Operation status
 *
 * @see llcc68_write_buffer
 */
llcc68_status_t llcc68_read_buffer( const void* context, const uint8_t offset, uint8_t* buffer, const uint8_t size );

//
// DIO and IRQ Control Functions
//

/**
 * @brief Set which interrupt signals are redirected to the dedicated DIO pin
 *
 * @remark By default, no interrupt signal is redirected.
 *
 * @remark An interrupt will not occur until it is enabled system-wide, even if it is redirected to a specific DIO.
 *
 * @remark The DIO pin will remain asserted until all redirected interrupt signals are cleared with a call to @ref
 * llcc68_clear_irq_status.
 *
 * @remark DIO2 and DIO3 are shared with other features. See @ref llcc68_set_dio2_as_rf_sw_ctrl and @ref
 * llcc68_set_dio3_as_tcxo_ctrl
 *
 * @param [in] context Chip implementation context
 * @param [in] irq_mask Variable that holds the system interrupt mask
 * @param [in] dio1_mask Variable that holds the interrupt mask for dio1
 * @param [in] dio2_mask Variable that holds the interrupt mask for dio2
 * @param [in] dio3_mask Variable that holds the interrupt mask for dio3
 *
 * @returns Operation status
 *
 * @see llcc68_clear_irq_status, llcc68_get_irq_status, llcc68_set_dio2_as_rf_sw_ctrl, llcc68_set_dio3_as_tcxo_ctrl
 */
llcc68_status_t llcc68_set_dio_irq_params( const void* context, const uint16_t irq_mask, const uint16_t dio1_mask,
                                           const uint16_t dio2_mask, const uint16_t dio3_mask );

/**
 * @brief Get system interrupt status
 *
 * @param [in] context Chip implementation context
 * @param [out] irq Pointer to a variable for holding the system interrupt status
 *
 * @returns Operation status
 *
 * @see llcc68_clear_irq_status
 */
llcc68_status_t llcc68_get_irq_status( const void* context, llcc68_irq_mask_t* irq );

/**
 * @brief Clear selected system interrupts
 *
 * @param [in] context Chip implementation context
 * @param [in] irq_mask Variable that holds the system interrupt to be cleared
 *
 * @returns Operation status
 *
 * @see llcc68_get_irq_status
 */
llcc68_status_t llcc68_clear_irq_status( const void* context, const llcc68_irq_mask_t irq_mask );

/**
 * @brief Clears any radio irq status flags that are set and returns the flags that
 * were cleared.
 *
 * @param [in] context Chip implementation context
 * @param [out] irq Pointer to a variable for holding the system interrupt status; can be NULL
 *
 * @returns Operation status
 */
llcc68_status_t llcc68_get_and_clear_irq_status( const void* context, llcc68_irq_mask_t* irq );

/**
 * @brief Configure the embedded RF switch control
 *
 * @param [in] context Chip implementation context
 * @param [in] enable Enable this feature if set to true
 *
 * @returns Operation status
 */
llcc68_status_t llcc68_set_dio2_as_rf_sw_ctrl( const void* context, const bool enable );

/**
 * @brief Configure the embedded TCXO switch control
 *
 * @remark This function shall only be called in standby RC mode.
 *
 * @remark The chip will wait for the timeout to happen before starting any operation that requires the TCXO.
 *
 * @param [in] context Chip implementation context
 * @param [in] tcxo_voltage Voltage used to power the TCXO
 * @param [in] timeout Time needed for the TCXO to be stable
 *
 * @returns Operation status
 *
 */
llcc68_status_t llcc68_set_dio3_as_tcxo_ctrl( const void* context, const llcc68_tcxo_ctrl_voltages_t tcxo_voltage,
                                              const uint32_t timeout );

//
// RF Modulation and Packet-Related Functions
//

/**
 * @brief Set the RF frequency for future radio operations.
 *
 * @remark This commands shall be called only after a packet type is selected.
 *
 * @param [in] context Chip implementation context
 * @param [in] freq_in_hz The frequency in Hz to set for radio operations
 *
 * @returns Operation status
 */
llcc68_status_t llcc68_set_rf_freq( const void* context, const uint32_t freq_in_hz );

/**
 * @brief Set the RF frequency for future radio operations - parameter in PLL steps
 *
 * @remark This commands shall be called only after a packet type is selected.
 *
 * @param [in] context Chip implementation context
 * @param [in] freq The frequency in PLL steps to set for radio operations
 *
 * @returns Operation status
 */
llcc68_status_t llcc68_set_rf_freq_in_pll_steps( const void* context, const uint32_t freq );

/**
 * @brief Set the packet type
 *
 * @param [in] context Chip implementation context
 *
 * @param [in] pkt_type Packet type to set
 *
 * @returns Operation status
 */
llcc68_status_t llcc68_set_pkt_type( const void* context, const llcc68_pkt_type_t pkt_type );

/**
 * @brief Get the current packet type
 *
 * @param [in] context Chip implementation context
 * @param [out] pkt_type Pointer to a variable holding the packet type
 *
 * @returns Operation status
 */
llcc68_status_t llcc68_get_pkt_type( const void* context, llcc68_pkt_type_t* pkt_type );

/**
 * @brief Set the parameters for TX power and power amplifier ramp time
 *
 * @param [in] context Chip implementation context
 * @param [in] pwr_in_dbm The Tx output power in dBm
 * @param [in] ramp_time The ramping time configuration for the PA
 *
 * @returns Operation status
 */
llcc68_status_t llcc68_set_tx_params( const void* context, const int8_t pwr_in_dbm,
                                      const llcc68_ramp_time_t ramp_time );

/**
 * @brief Set the modulation parameters for GFSK packets
 *
 * @remark The command @ref llcc68_set_pkt_type must be called prior to this
 * one.
 *
 * @param [in] context Chip implementation context
 * @param [in] params The structure of GFSK modulation configuration
 *
 * @returns Operation status
 */
llcc68_status_t llcc68_set_gfsk_mod_params( const void* context, const llcc68_mod_params_gfsk_t* params );

/**
 * @brief Set the modulation parameters for BPSK packets
 *
 * @remark The command @ref llcc68_set_pkt_type must be called prior to this
 * one.
 *
 * @param [in] context Chip implementation context
 * @param [in] params The structure of BPSK modulation configuration
 *
 * @returns Operation status
 */
llcc68_status_t llcc68_set_bpsk_mod_params( const void* context, const llcc68_mod_params_bpsk_t* params );

/**
 * @brief Set the modulation parameters for LoRa packets
 *
 * @remark The command @ref llcc68_set_pkt_type must be called prior to this one.
 *
 * @param [in] context Chip implementation context
 * @param [in] params The structure of LoRa modulation configuration
 *
 * @returns Operation status
 */
llcc68_status_t llcc68_set_lora_mod_params( const void* context, const llcc68_mod_params_lora_t* params );

/**
 * @brief Set the packet parameters for GFSK packets
 *
 * @remark The command @ref llcc68_set_pkt_type must be called prior to this
 * one.
 *
 * @param [in] context Chip implementation context
 * @param [in] params The structure of GFSK packet configuration
 *
 * @returns Operation status
 */
llcc68_status_t llcc68_set_gfsk_pkt_params( const void* context, const llcc68_pkt_params_gfsk_t* params );

/**
 * @brief Set the packet parameters for BPSK packets
 *
 * @remark The command @ref llcc68_set_pkt_type must be called prior to this
 * one.
 *
 * @param [in] context Chip implementation context
 * @param [in] params The structure of BPSK packet configuration
 *
 * @returns Operation status
 */
llcc68_status_t llcc68_set_bpsk_pkt_params( const void* context, const llcc68_pkt_params_bpsk_t* params );

/**
 * @brief Set the packet parameters for LoRa packets
 *
 * @remark The command @ref llcc68_set_pkt_type must be called prior to this one.
 *
 * @param [in] context Chip implementation context
 * @param [in] params The structure of LoRa packet configuration
 *
 * @returns Operation status
 */
llcc68_status_t llcc68_set_lora_pkt_params( const void* context, const llcc68_pkt_params_lora_t* params );

/*!
 * @brief Set the Node and Broadcast address used for GFSK
 *
 * This setting is used only when filtering is enabled.
 *
 * @param [in] context Chip implementation context
 * @param [in] node_address The node address used as filter
 * @param [in] broadcast_address The broadcast address used as filter
 *
 * @returns Operation status
 */
llcc68_status_t llcc68_set_gfsk_pkt_address( const void* context, const uint8_t node_address,
                                             const uint8_t broadcast_address );

/**
 * @brief Set the parameters for CAD operation
 *
 * @remark The command @ref llcc68_set_pkt_type must be called prior to this one.
 *
 * @param [in] context Chip implementation context
 * @param [in] params The structure of CAD configuration
 *
 * @returns Operation status
 */
llcc68_status_t llcc68_set_cad_params( const void* context, const llcc68_cad_params_t* params );

/**
 * @brief Set buffer start addresses for both Tx and Rx operations
 *
 * @param [in] context Chip implementation context
 * @param [in] tx_base_address The start address used for Tx operations
 * @param [in] rx_base_address The start address used for Rx operations
 *
 * @returns Operation status
 */
llcc68_status_t llcc68_set_buffer_base_address( const void* context, const uint8_t tx_base_address,
                                                const uint8_t rx_base_address );

/**
 * @brief Set the timeout to be used when the chip is configured in Rx mode (only in LoRa)
 *
 * @remark The maximum timeout is \ref LLCC68_MAX_LORA_SYMB_NUM_TIMEOUT
 * @remark The function is disabled if the timeout is set to 0
 *
 * @param [in] context Chip implementation context
 * @param [in] nb_of_symbs Timeout in number of symbol
 *
 * @returns Operation status
 */
llcc68_status_t llcc68_set_lora_symb_nb_timeout( const void* context, const uint8_t nb_of_symbs );

//
// Communication Status Information
//

/**
 * @brief Get the chip status
 *
 * @param [in] context Chip implementation context
 * @param [out] radio_status Pointer to a structure holding the radio status
 *
 * @returns Operation status
 */
llcc68_status_t llcc68_get_status( const void* context, llcc68_chip_status_t* radio_status );

/**
 * @brief Get the current Rx buffer status for both LoRa and GFSK Rx operations
 *
 * @details This function is used to get the length of the received payload and the start address to be used when
 * reading data from the Rx buffer.
 *
 * @param [in] context Chip implementation context
 * @param [out] rx_buffer_status Pointer to a structure to store the current status
 *
 * @returns Operation status
 */
llcc68_status_t llcc68_get_rx_buffer_status( const void* context, llcc68_rx_buffer_status_t* rx_buffer_status );

/**
 * @brief Get the status of the last GFSK packet received
 *
 * @param [in] context Chip implementation context
 * @param [out] pkt_status Pointer to a structure to store the packet status
 *
 * @returns Operation status
 */
llcc68_status_t llcc68_get_gfsk_pkt_status( const void* context, llcc68_pkt_status_gfsk_t* pkt_status );

/**
 * @brief Get the status of the last LoRa packet received
 *
 * @param [in] context Chip implementation context
 * @param [out] pkt_status Pointer to a structure to store the packet status
 *
 * @returns Operation status
 */
llcc68_status_t llcc68_get_lora_pkt_status( const void* context, llcc68_pkt_status_lora_t* pkt_status );

/**
 * @brief Get the instantaneous RSSI value.
 *
 * @remark This function shall be called when in Rx mode.
 *
 * @param [in] context Chip implementation context
 * @param [out] rssi_in_dbm Pointer to a variable to store the RSSI value in dBm
 *
 * @returns Operation status
 *
 * @see llcc68_set_rx
 */
llcc68_status_t llcc68_get_rssi_inst( const void* context, int16_t* rssi_in_dbm );

/**
 * @brief Get the statistics about GFSK communication
 *
 * @param [in] context Chip implementation context
 * @param [out] stats Pointer to a structure to store GFSK-related statistics
 *
 * @returns Operation status
 */
llcc68_status_t llcc68_get_gfsk_stats( const void* context, llcc68_stats_gfsk_t* stats );

/**
 * @brief Get the statistics about LoRa communication
 *
 * @param [in] context Chip implementation context
 * @param [out] stats Pointer to a structure to store LoRa-related statistics
 *
 * @returns Operation status
 */
llcc68_status_t llcc68_get_lora_stats( const void* context, llcc68_stats_lora_t* stats );

/**
 * @brief Reset all the statistics for both Lora and GFSK communications
 *
 * @param [in] context Chip implementation context
 *
 * @returns Operation status
 */
llcc68_status_t llcc68_reset_stats( const void* context );

//
// Miscellaneous
//

/**
 * @brief Perform a hard reset of the chip
 *
 * @param [in] context Chip implementation context
 *
 * @returns Operation status
 */
llcc68_status_t llcc68_reset( const void* context );

/**
 * @brief Wake the radio up from sleep mode.
 *
 * @param [in]  context Chip implementation context
 *
 * @returns Operation status
 */
llcc68_status_t llcc68_wakeup( const void* context );

/**
 * @brief Get the list of all active errors
 *
 * @param [in] context Chip implementation context
 * @param [out] errors Pointer to a variable to store the error list
 *
 * @returns Operation status
 */
llcc68_status_t llcc68_get_device_errors( const void* context, llcc68_errors_mask_t* errors );

/**
 * @brief Clear all active errors
 *
 * @param [in] context Chip implementation context
 *
 * @returns Operation status
 */
llcc68_status_t llcc68_clear_device_errors( const void* context );

/**
 * @brief Get the parameter corresponding to a GFSK Rx bandwith immediately above the minimum requested one.
 *
 * @param [in] bw Minimum required bandwith in Hz
 * @param [out] param Pointer to a value to store the parameter
 *
 * @returns Operation status
 */
llcc68_status_t llcc68_get_gfsk_bw_param( const uint32_t bw, uint8_t* param );

/**
 * @brief Get the actual value in Hertz of a given LoRa bandwidth
 *
 * @param [in] bw LoRa bandwidth parameter
 *
 * @returns Actual LoRa bandwidth in Hertz
 */
uint32_t llcc68_get_lora_bw_in_hz( llcc68_lora_bw_t bw );

/**
 * @brief Compute the numerator for LoRa time-on-air computation.
 *
 * @remark To get the actual time-on-air in second, this value has to be divided by the LoRa bandwidth in Hertz.
 *
 * @param [in] pkt_p Pointer to the structure holding the LoRa packet parameters
 * @param [in] mod_p Pointer to the structure holding the LoRa modulation parameters
 *
 * @returns LoRa time-on-air numerator
 */
uint32_t llcc68_get_lora_time_on_air_numerator( const llcc68_pkt_params_lora_t* pkt_p,
                                                const llcc68_mod_params_lora_t* mod_p );

/**
 * @brief Get the time on air in ms for LoRa transmission
 *
 * @param [in] pkt_p Pointer to a structure holding the LoRa packet parameters
 * @param [in] mod_p Pointer to a structure holding the LoRa modulation parameters
 *
 * @returns Time-on-air value in ms for LoRa transmission
 */
uint32_t llcc68_get_lora_time_on_air_in_ms( const llcc68_pkt_params_lora_t* pkt_p,
                                            const llcc68_mod_params_lora_t* mod_p );

/**
 * @brief Compute the numerator for GFSK time-on-air computation.
 *
 * @remark To get the actual time-on-air in second, this value has to be divided by the GFSK bitrate in bits per
 * second.
 *
 * @param [in] pkt_p Pointer to the structure holding the GFSK packet parameters
 *
 * @returns GFSK time-on-air numerator
 */
uint32_t llcc68_get_gfsk_time_on_air_numerator( const llcc68_pkt_params_gfsk_t* pkt_p );

/**
 * @brief Get the time on air in ms for GFSK transmission
 *
 * @param [in] pkt_p Pointer to a structure holding the GFSK packet parameters
 * @param [in] mod_p Pointer to a structure holding the GFSK modulation parameters
 *
 * @returns Time-on-air value in ms for GFSK transmission
 */
uint32_t llcc68_get_gfsk_time_on_air_in_ms( const llcc68_pkt_params_gfsk_t* pkt_p,
                                            const llcc68_mod_params_gfsk_t* mod_p );

/**
 * @brief Generate one or more 32-bit random numbers.
 *
 * @remark A valid packet type must have been configured with @ref llcc68_set_pkt_type
 *         before using this command.
 *
 * @param [in]  context Chip implementation context
 * @param [out] numbers Array where numbers will be stored
 * @param [in]  n Number of desired random numbers
 *
 * @returns Operation status
 *
 * This code can potentially result in interrupt generation. It is the responsibility of
 * the calling code to disable radio interrupts before calling this function,
 * and re-enable them afterwards if necessary, or be certain that any interrupts
 * generated during this process will not cause undesired side-effects in the software.
 *
 * Please note that the random numbers produced by the generator do not have a uniform or Gaussian distribution. If
 * uniformity is needed, perform appropriate software post-processing.
 */
llcc68_status_t llcc68_get_random_numbers( const void* context, uint32_t* numbers, unsigned int n );

/**
 * @brief Get the number of PLL steps for a given frequency in Hertz
 *
 * @param [in] freq_in_hz Frequency in Hertz
 *
 * @returns Number of PLL steps
 */
uint32_t llcc68_convert_freq_in_hz_to_pll_step( uint32_t freq_in_hz );

/**
 * @brief Get the number of RTC steps for a given timeout in millisecond
 *
 * @param [in] timeout_in_ms Timeout in millisecond
 *
 * @returns Number of RTC steps
 */
uint32_t llcc68_convert_timeout_in_ms_to_rtc_step( uint32_t timeout_in_ms );

/**
 * @brief Generic finalizing function after reception
 *
 * @remark This function can be called after any reception sequence and must be called after any reception with timeout
 * active sequence.
 *
 * @param [in] context Chip implementation context
 *
 * @returns Operation status
 */
llcc68_status_t llcc68_handle_rx_done( const void* context );

//
// Registers access
//

/**
 * @brief Configure the boost mode in reception
 *
 * @remark This configuration is not kept in the retention memory. Rx boosted mode shall be enabled each time the chip
 * leaves sleep mode.
 *
 * @param [in] context Chip implementation context
 * @param [in] state Boost mode activation
 *
 * @returns Operation status
 */
llcc68_status_t llcc68_cfg_rx_boosted( const void* context, const bool state );

/**
 * @brief Configure the sync word used in GFSK packet
 *
 * @param [in] context Chip implementation context
 * @param [in] sync_word Buffer holding the sync word to be configured
 * @param [in] sync_word_len Sync word length in byte
 *
 * @returns Operation status
 */
llcc68_status_t llcc68_set_gfsk_sync_word( const void* context, const uint8_t* sync_word, const uint8_t sync_word_len );

/**
 * @brief Configure the sync word used in LoRa packet
 *
 * @remark In the case of a LoRaWAN use case, the two following values are specified:
 *   - 0x12 for a private LoRaWAN network (default)
 *   - 0x34 for a public LoRaWAN network
 *
 * @param [in] context Chip implementation context
 * @param [in] sync_word Sync word to be configured
 *
 * @returns Operation status
 */
llcc68_status_t llcc68_set_lora_sync_word( const void* context, const uint8_t sync_word );

/**
 * @brief Configure the seed used to compute CRC in GFSK packet
 *
 * @param [in] context Chip implementation context
 * @param [in] seed Seed value used to compute the CRC value
 *
 * @returns Operation status
 */
llcc68_status_t llcc68_set_gfsk_crc_seed( const void* context, uint16_t seed );

/**
 * @brief Configure the polynomial used to compute CRC in GFSK packet
 *
 * @param [in] context Chip implementation context
 * @param [in] polynomial Polynomial value used to compute the CRC value
 *
 * @returns Operation status
 */
llcc68_status_t llcc68_set_gfsk_crc_polynomial( const void* context, const uint16_t polynomial );

/**
 * @brief Configure the whitening seed used in GFSK packet
 *
 * @param [in] context Chip implementation context
 * @param [in] seed Seed value used in data whitening
 *
 * @returns Operation status
 */
llcc68_status_t llcc68_set_gfsk_whitening_seed( const void* context, const uint16_t seed );

/**
 * @brief Configure the Tx PA clamp
 *
 * @remark Workaround - With a LLCC68, during the chip initialization, calling this function optimizes the PA clamping
 * threshold. The call must be done after a Power On Reset or a wake-up from cold start (see DS_LLCC68_V1.0 datasheet
 * chapter 15.2)
 *
 * @param [in] context Chip implementation context
 *
 * @returns Operation status
 */
llcc68_status_t llcc68_cfg_tx_clamp( const void* context );

/**
 * @brief Stop the RTC and clear the related event
 *
 * @remark Workaround - It is advised to call this function after ANY reception with timeout active sequence, which
 * stop the RTC and clear the timeout event, if any (see DS_LLCC68_V1.0 datasheet chapter 15.4)
 *
 * @param [in] context Chip implementation context
 *
 * @returns Operation status
 */
llcc68_status_t llcc68_stop_rtc( const void* context );

/**
 * @brief Configure the Over Current Protection (OCP) value
 *
 * @remark The maximum value that can be configured is 63 (i.e. 157.5 mA)
 *
 * @param [in] context Chip implementation context
 * @param [in] ocp_in_step_of_2_5_ma OCP value given in steps of 2.5 mA
 *
 * @returns Operation status
 */
llcc68_status_t llcc68_set_ocp_value( const void* context, const uint8_t ocp_in_step_of_2_5_ma );

/**
 * @brief Configure the internal trimming capacitor values
 *
 * @remark The device is fitted with internal programmable capacitors connected independently to the pins XTA and XTB of
 * the device. Each capacitor can be controlled independently in steps of 0.47 pF added to the minimal value 11.3pF.
 *
 * @param [in] context Chip implementation context
 * @param [in] trimming_cap_xta Value for the trimming capacitor connected to XTA pin
 * @param [in] trimming_cap_xtb Value for the trimming capacitor connected to XTB pin
 *
 * @returns Operation status
 */
llcc68_status_t llcc68_set_trimming_capacitor_values( const void* context, const uint8_t trimming_cap_xta,
                                                      const uint8_t trimming_cap_xtb );

/**
 * @brief Add registers to the retention list
 *
 * @remark Up to 4 registers can be added to the retention list
 * @remark This function actually appends registers to the list until it is full
 * @remark Registers already added to the list cannot be removed unless the chip goes in sleep mode without retention or
 * a reset is issued
 *
 * @param [in] context Chip implementation context
 * @param [in] register_address The array with addresses of the register to be kept in retention
 * @param [in] register_nb The number of register to be kept in retention
 *
 * @returns Operation status
 */
llcc68_status_t llcc68_add_registers_to_retention_list( const void* context, const uint16_t* register_addr,
                                                        uint8_t register_nb );

/**
 * @brief Add LLCC68_REG_RXGAIN, LLCC68_REG_TX_MODULATION and LLCC68_REG_IQ_POLARITY registers to the retention list
 *
 * @remark These registers are used in workarounds implemented in this driver
 * @remark This function adds 3 registers out of the 4 available slots to the retention list
 * @remark It is recommended to call this function once during initialization phase if the application requires the chip
 * to enter sleep mode without retention
 *
 * @param [in] context Chip implementation context
 *
 * @returns Operation status
 *
 * @see llcc68_add_registers_to_retention_list
 */
llcc68_status_t llcc68_init_retention_list( const void* context );

/**
 * @brief Get LoRa coding rate and CRC configurations from received header
 *
 * @remark The output of this function is only valid if the field header_type of pkt_params is equal to @ref
 * LLCC68_LORA_PKT_EXPLICIT when calling @ref llcc68_set_lora_pkt_params()
 * @remark The values for cr and crc_is_on are extracted from the header of the received LoRa packet
 *
 * @param [in]  context    Chip implementation context
 * @param [out] cr         LoRa coding rate
 * @param [out] crc_is_on  LoRa CRC configuration
 *
 * @returns Operation status
 */
llcc68_status_t llcc68_get_lora_params_from_header( const void* context, llcc68_lora_cr_t* cr, bool* crc_is_on );

#ifdef __cplusplus
}
#endif

#endif  // LLCC68_H

/* --- EOF ------------------------------------------------------------------ */

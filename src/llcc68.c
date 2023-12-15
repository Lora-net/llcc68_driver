/**
 * @file      llcc68.c
 *
 * @brief     LLCC68 radio driver implementation
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

/*
 * -----------------------------------------------------------------------------
 * --- DEPENDENCIES ------------------------------------------------------------
 */
#include <stddef.h>
#include "llcc68.h"
#include "llcc68_hal.h"
#include "llcc68_regs.h"

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE MACROS-----------------------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE CONSTANTS -------------------------------------------------------
 */

/**
 * @brief Internal frequency of the radio
 */
#define LLCC68_XTAL_FREQ 32000000UL

/**
 * @brief Internal frequency of the radio
 */
#define LLCC68_RTC_FREQ_IN_HZ 64000UL

/**
 * @brief Scaling factor used to perform fixed-point operations
 */
#define LLCC68_PLL_STEP_SHIFT_AMOUNT ( 14 )

/**
 * @brief PLL step - scaled with LLCC68_PLL_STEP_SHIFT_AMOUNT
 */
#define LLCC68_PLL_STEP_SCALED ( LLCC68_XTAL_FREQ >> ( 25 - LLCC68_PLL_STEP_SHIFT_AMOUNT ) )

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE TYPES -----------------------------------------------------------
 */

/**
 * Commands Interface
 */
typedef enum llcc68_commands_e
{
    // Operational Modes Functions
    LLCC68_SET_SLEEP                  = 0x84,
    LLCC68_SET_STANDBY                = 0x80,
    LLCC68_SET_FS                     = 0xC1,
    LLCC68_SET_TX                     = 0x83,
    LLCC68_SET_RX                     = 0x82,
    LLCC68_SET_STOP_TIMER_ON_PREAMBLE = 0x9F,
    LLCC68_SET_RX_DUTY_CYCLE          = 0x94,
    LLCC68_SET_CAD                    = 0xC5,
    LLCC68_SET_TX_CONTINUOUS_WAVE     = 0xD1,
    LLCC68_SET_TX_INFINITE_PREAMBLE   = 0xD2,
    LLCC68_SET_REGULATOR_MODE         = 0x96,
    LLCC68_CALIBRATE                  = 0x89,
    LLCC68_CALIBRATE_IMAGE            = 0x98,
    LLCC68_SET_PA_CFG                 = 0x95,
    LLCC68_SET_RX_TX_FALLBACK_MODE    = 0x93,
    // Registers and buffer Access
    LLCC68_WRITE_REGISTER = 0x0D,
    LLCC68_READ_REGISTER  = 0x1D,
    LLCC68_WRITE_BUFFER   = 0x0E,
    LLCC68_READ_BUFFER    = 0x1E,
    // DIO and IRQ Control Functions
    LLCC68_SET_DIO_IRQ_PARAMS         = 0x08,
    LLCC68_GET_IRQ_STATUS             = 0x12,
    LLCC68_CLR_IRQ_STATUS             = 0x02,
    LLCC68_SET_DIO2_AS_RF_SWITCH_CTRL = 0x9D,
    LLCC68_SET_DIO3_AS_TCXO_CTRL      = 0x97,
    // RF Modulation and Packet-Related Functions
    LLCC68_SET_RF_FREQUENCY          = 0x86,
    LLCC68_SET_PKT_TYPE              = 0x8A,
    LLCC68_GET_PKT_TYPE              = 0x11,
    LLCC68_SET_TX_PARAMS             = 0x8E,
    LLCC68_SET_MODULATION_PARAMS     = 0x8B,
    LLCC68_SET_PKT_PARAMS            = 0x8C,
    LLCC68_SET_CAD_PARAMS            = 0x88,
    LLCC68_SET_BUFFER_BASE_ADDRESS   = 0x8F,
    LLCC68_SET_LORA_SYMB_NUM_TIMEOUT = 0xA0,
    // Communication Status Information
    LLCC68_GET_STATUS           = 0xC0,
    LLCC68_GET_RX_BUFFER_STATUS = 0x13,
    LLCC68_GET_PKT_STATUS       = 0x14,
    LLCC68_GET_RSSI_INST        = 0x15,
    LLCC68_GET_STATS            = 0x10,
    LLCC68_RESET_STATS          = 0x00,
    // Miscellaneous
    LLCC68_GET_DEVICE_ERRORS = 0x17,
    LLCC68_CLR_DEVICE_ERRORS = 0x07,
} llcc68_commands_t;

/**
 * Commands Interface buffer sizes
 */
typedef enum llcc68_commands_size_e
{
    // Operational Modes Functions
    LLCC68_SIZE_SET_SLEEP                  = 2,
    LLCC68_SIZE_SET_STANDBY                = 2,
    LLCC68_SIZE_SET_FS                     = 1,
    LLCC68_SIZE_SET_TX                     = 4,
    LLCC68_SIZE_SET_RX                     = 4,
    LLCC68_SIZE_SET_STOP_TIMER_ON_PREAMBLE = 2,
    LLCC68_SIZE_SET_RX_DUTY_CYCLE          = 7,
    LLCC68_SIZE_SET_CAD                    = 1,
    LLCC68_SIZE_SET_TX_CONTINUOUS_WAVE     = 1,
    LLCC68_SIZE_SET_TX_INFINITE_PREAMBLE   = 1,
    LLCC68_SIZE_SET_REGULATOR_MODE         = 2,
    LLCC68_SIZE_CALIBRATE                  = 2,
    LLCC68_SIZE_CALIBRATE_IMAGE            = 3,
    LLCC68_SIZE_SET_PA_CFG                 = 5,
    LLCC68_SIZE_SET_RX_TX_FALLBACK_MODE    = 2,
    // Registers and buffer Access
    // Full size: this value plus buffer size
    LLCC68_SIZE_WRITE_REGISTER = 3,
    // Full size: this value plus buffer size
    LLCC68_SIZE_READ_REGISTER = 4,
    // Full size: this value plus buffer size
    LLCC68_SIZE_WRITE_BUFFER = 2,
    // Full size: this value plus buffer size
    LLCC68_SIZE_READ_BUFFER = 3,
    // DIO and IRQ Control Functions
    LLCC68_SIZE_SET_DIO_IRQ_PARAMS         = 9,
    LLCC68_SIZE_GET_IRQ_STATUS             = 2,
    LLCC68_SIZE_CLR_IRQ_STATUS             = 3,
    LLCC68_SIZE_SET_DIO2_AS_RF_SWITCH_CTRL = 2,
    LLCC68_SIZE_SET_DIO3_AS_TCXO_CTRL      = 5,
    // RF Modulation and Packet-Related Functions
    LLCC68_SIZE_SET_RF_FREQUENCY           = 5,
    LLCC68_SIZE_SET_PKT_TYPE               = 2,
    LLCC68_SIZE_GET_PKT_TYPE               = 2,
    LLCC68_SIZE_SET_TX_PARAMS              = 3,
    LLCC68_SIZE_SET_MODULATION_PARAMS_GFSK = 9,
    LLCC68_SIZE_SET_MODULATION_PARAMS_BPSK = 5,
    LLCC68_SIZE_SET_MODULATION_PARAMS_LORA = 5,
    LLCC68_SIZE_SET_PKT_PARAMS_GFSK        = 10,
    LLCC68_SIZE_SET_PKT_PARAMS_BPSK        = 2,
    LLCC68_SIZE_SET_PKT_PARAMS_LORA        = 7,
    LLCC68_SIZE_SET_CAD_PARAMS             = 8,
    LLCC68_SIZE_SET_BUFFER_BASE_ADDRESS    = 3,
    LLCC68_SIZE_SET_LORA_SYMB_NUM_TIMEOUT  = 2,
    // Communication Status Information
    LLCC68_SIZE_GET_STATUS           = 1,
    LLCC68_SIZE_GET_RX_BUFFER_STATUS = 2,
    LLCC68_SIZE_GET_PKT_STATUS       = 2,
    LLCC68_SIZE_GET_RSSI_INST        = 2,
    LLCC68_SIZE_GET_STATS            = 2,
    LLCC68_SIZE_RESET_STATS          = 7,
    // Miscellaneous
    LLCC68_SIZE_GET_DEVICE_ERRORS = 2,
    LLCC68_SIZE_CLR_DEVICE_ERRORS = 3,
    LLCC68_SIZE_MAX_BUFFER        = 255,
    LLCC68_SIZE_DUMMY_BYTE        = 1,
} llcc68_commands_size_t;

typedef struct
{
    uint32_t bw;
    uint8_t  param;
} gfsk_bw_t;

gfsk_bw_t gfsk_bw[] = {
    { 4800, LLCC68_GFSK_BW_4800 },     { 5800, LLCC68_GFSK_BW_5800 },     { 7300, LLCC68_GFSK_BW_7300 },
    { 9700, LLCC68_GFSK_BW_9700 },     { 11700, LLCC68_GFSK_BW_11700 },   { 14600, LLCC68_GFSK_BW_14600 },
    { 19500, LLCC68_GFSK_BW_19500 },   { 23400, LLCC68_GFSK_BW_23400 },   { 29300, LLCC68_GFSK_BW_29300 },
    { 39000, LLCC68_GFSK_BW_39000 },   { 46900, LLCC68_GFSK_BW_46900 },   { 58600, LLCC68_GFSK_BW_58600 },
    { 78200, LLCC68_GFSK_BW_78200 },   { 93800, LLCC68_GFSK_BW_93800 },   { 117300, LLCC68_GFSK_BW_117300 },
    { 156200, LLCC68_GFSK_BW_156200 }, { 187200, LLCC68_GFSK_BW_187200 }, { 234300, LLCC68_GFSK_BW_234300 },
    { 312000, LLCC68_GFSK_BW_312000 }, { 373600, LLCC68_GFSK_BW_373600 }, { 467000, LLCC68_GFSK_BW_467000 },
};

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE VARIABLES -------------------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DECLARATION -------------------------------------------
 */

/**
 * @brief 15.1.2 Workaround
 *
 * @remark Before any packet transmission, bit #2 of LLCC68_REG_TX_MODULATION shall be set to:
 * 0 if the LoRa BW = 500 kHz
 * 1 for any other LoRa BW
 * 1 for any (G)FSK configuration
 *
 * @param [in] context Chip implementation context.
 * @param [in] pkt_type The modulation type (G)FSK/LoRa
 * @param [in] bw In case of LoRa modulation the bandwith must be specified
 *
 * @returns Operation status
 */
static llcc68_status_t llcc68_tx_modulation_workaround( const void* context, llcc68_pkt_type_t pkt_type,
                                                        llcc68_lora_bw_t bw );

static inline uint32_t llcc68_get_gfsk_crc_len_in_bytes( llcc68_gfsk_crc_types_t crc_type );

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC FUNCTIONS DEFINITION ---------------------------------------------
 */

llcc68_status_t llcc68_set_sleep( const void* context, const llcc68_sleep_cfgs_t cfg )
{
    const uint8_t buf[LLCC68_SIZE_SET_SLEEP] = {
        LLCC68_SET_SLEEP,
        ( uint8_t ) cfg,
    };

    return ( llcc68_status_t ) llcc68_hal_write( context, buf, LLCC68_SIZE_SET_SLEEP, 0, 0 );
}

llcc68_status_t llcc68_set_standby( const void* context, const llcc68_standby_cfg_t cfg )
{
    const uint8_t buf[LLCC68_SIZE_SET_STANDBY] = {
        LLCC68_SET_STANDBY,
        ( uint8_t ) cfg,
    };

    return ( llcc68_status_t ) llcc68_hal_write( context, buf, LLCC68_SIZE_SET_STANDBY, 0, 0 );
}

llcc68_status_t llcc68_set_fs( const void* context )
{
    const uint8_t buf[LLCC68_SIZE_SET_FS] = {
        LLCC68_SET_FS,
    };

    return ( llcc68_status_t ) llcc68_hal_write( context, buf, LLCC68_SIZE_SET_FS, 0, 0 );
}

llcc68_status_t llcc68_set_tx( const void* context, const uint32_t timeout_in_ms )
{
    if( timeout_in_ms > LLCC68_MAX_TIMEOUT_IN_MS )
    {
        return LLCC68_STATUS_UNKNOWN_VALUE;
    }

    const uint32_t timeout_in_rtc_step = llcc68_convert_timeout_in_ms_to_rtc_step( timeout_in_ms );

    return llcc68_set_tx_with_timeout_in_rtc_step( context, timeout_in_rtc_step );
}

llcc68_status_t llcc68_set_tx_with_timeout_in_rtc_step( const void* context, const uint32_t timeout_in_rtc_step )
{
    const uint8_t buf[LLCC68_SIZE_SET_TX] = {
        LLCC68_SET_TX,
        ( uint8_t )( timeout_in_rtc_step >> 16 ),
        ( uint8_t )( timeout_in_rtc_step >> 8 ),
        ( uint8_t )( timeout_in_rtc_step >> 0 ),
    };

    return ( llcc68_status_t ) llcc68_hal_write( context, buf, LLCC68_SIZE_SET_TX, 0, 0 );
}

llcc68_status_t llcc68_set_rx( const void* context, const uint32_t timeout_in_ms )
{
    if( timeout_in_ms > LLCC68_MAX_TIMEOUT_IN_MS )
    {
        return LLCC68_STATUS_UNKNOWN_VALUE;
    }

    const uint32_t timeout_in_rtc_step = llcc68_convert_timeout_in_ms_to_rtc_step( timeout_in_ms );

    return llcc68_set_rx_with_timeout_in_rtc_step( context, timeout_in_rtc_step );
}

llcc68_status_t llcc68_set_rx_with_timeout_in_rtc_step( const void* context, const uint32_t timeout_in_rtc_step )
{
    const uint8_t buf[LLCC68_SIZE_SET_RX] = {
        LLCC68_SET_RX,
        ( uint8_t )( timeout_in_rtc_step >> 16 ),
        ( uint8_t )( timeout_in_rtc_step >> 8 ),
        ( uint8_t )( timeout_in_rtc_step >> 0 ),
    };

    return ( llcc68_status_t ) llcc68_hal_write( context, buf, LLCC68_SIZE_SET_RX, 0, 0 );
}

llcc68_status_t llcc68_stop_timer_on_preamble( const void* context, const bool enable )
{
    const uint8_t buf[LLCC68_SIZE_SET_STOP_TIMER_ON_PREAMBLE] = {
        LLCC68_SET_STOP_TIMER_ON_PREAMBLE,
        ( enable == true ) ? 1 : 0,
    };

    return ( llcc68_status_t ) llcc68_hal_write( context, buf, LLCC68_SIZE_SET_STOP_TIMER_ON_PREAMBLE, 0, 0 );
}

llcc68_status_t llcc68_set_rx_duty_cycle( const void* context, const uint32_t rx_time_in_ms,
                                          const uint32_t sleep_time_in_ms )
{
    const uint32_t rx_time_in_rtc_step    = llcc68_convert_timeout_in_ms_to_rtc_step( rx_time_in_ms );
    const uint32_t sleep_time_in_rtc_step = llcc68_convert_timeout_in_ms_to_rtc_step( sleep_time_in_ms );

    return llcc68_set_rx_duty_cycle_with_timings_in_rtc_step( context, rx_time_in_rtc_step, sleep_time_in_rtc_step );
}

llcc68_status_t llcc68_set_rx_duty_cycle_with_timings_in_rtc_step( const void*    context,
                                                                   const uint32_t rx_time_in_rtc_step,
                                                                   const uint32_t sleep_time_in_rtc_step )
{
    const uint8_t buf[LLCC68_SIZE_SET_RX_DUTY_CYCLE] = {
        LLCC68_SET_RX_DUTY_CYCLE,
        ( uint8_t )( rx_time_in_rtc_step >> 16 ),
        ( uint8_t )( rx_time_in_rtc_step >> 8 ),
        ( uint8_t )( rx_time_in_rtc_step >> 0 ),
        ( uint8_t )( sleep_time_in_rtc_step >> 16 ),
        ( uint8_t )( sleep_time_in_rtc_step >> 8 ),
        ( uint8_t )( sleep_time_in_rtc_step >> 0 ),
    };

    return ( llcc68_status_t ) llcc68_hal_write( context, buf, LLCC68_SIZE_SET_RX_DUTY_CYCLE, 0, 0 );
}

llcc68_status_t llcc68_set_cad( const void* context )
{
    const uint8_t buf[LLCC68_SIZE_SET_CAD] = {
        LLCC68_SET_CAD,
    };

    return ( llcc68_status_t ) llcc68_hal_write( context, buf, LLCC68_SIZE_SET_CAD, 0, 0 );
}

llcc68_status_t llcc68_set_tx_cw( const void* context )
{
    const uint8_t buf[LLCC68_SIZE_SET_TX_CONTINUOUS_WAVE] = {
        LLCC68_SET_TX_CONTINUOUS_WAVE,
    };

    return ( llcc68_status_t ) llcc68_hal_write( context, buf, LLCC68_SIZE_SET_TX_CONTINUOUS_WAVE, 0, 0 );
}

llcc68_status_t llcc68_set_tx_infinite_preamble( const void* context )
{
    const uint8_t buf[LLCC68_SIZE_SET_TX_INFINITE_PREAMBLE] = {
        LLCC68_SET_TX_INFINITE_PREAMBLE,
    };

    return ( llcc68_status_t ) llcc68_hal_write( context, buf, LLCC68_SIZE_SET_TX_INFINITE_PREAMBLE, 0, 0 );
}

llcc68_status_t llcc68_set_reg_mode( const void* context, const llcc68_reg_mod_t mode )
{
    const uint8_t buf[LLCC68_SIZE_SET_REGULATOR_MODE] = {
        LLCC68_SET_REGULATOR_MODE,
        ( uint8_t ) mode,
    };

    return ( llcc68_status_t ) llcc68_hal_write( context, buf, LLCC68_SIZE_SET_REGULATOR_MODE, 0, 0 );
}

llcc68_status_t llcc68_cal( const void* context, const llcc68_cal_mask_t param )
{
    const uint8_t buf[LLCC68_SIZE_CALIBRATE] = {
        LLCC68_CALIBRATE,
        ( uint8_t ) param,
    };

    return ( llcc68_status_t ) llcc68_hal_write( context, buf, LLCC68_SIZE_CALIBRATE, 0, 0 );
}

llcc68_status_t llcc68_cal_img( const void* context, const uint8_t freq1, const uint8_t freq2 )
{
    const uint8_t buf[LLCC68_SIZE_CALIBRATE_IMAGE] = {
        LLCC68_CALIBRATE_IMAGE,
        freq1,
        freq2,
    };

    return ( llcc68_status_t ) llcc68_hal_write( context, buf, LLCC68_SIZE_CALIBRATE_IMAGE, 0, 0 );
}

llcc68_status_t llcc68_cal_img_in_mhz( const void* context, const uint16_t freq1_in_mhz, const uint16_t freq2_in_mhz )
{
    // Perform a floor() to get a value for freq1 corresponding to a frequency lower than or equal to freq1_in_mhz
    const uint8_t freq1 = freq1_in_mhz / LLCC68_IMAGE_CALIBRATION_STEP_IN_MHZ;

    // Perform a ceil() to get a value for freq2 corresponding to a frequency higher than or equal to freq2_in_mhz
    const uint8_t freq2 =
        ( freq2_in_mhz + LLCC68_IMAGE_CALIBRATION_STEP_IN_MHZ - 1 ) / LLCC68_IMAGE_CALIBRATION_STEP_IN_MHZ;

    return llcc68_cal_img( context, freq1, freq2 );
}

llcc68_status_t llcc68_set_pa_cfg( const void* context, const llcc68_pa_cfg_params_t* params )
{
    const uint8_t buf[LLCC68_SIZE_SET_PA_CFG] = {
        LLCC68_SET_PA_CFG, params->pa_duty_cycle, params->hp_max, params->device_sel, params->pa_lut,
    };

    return ( llcc68_status_t ) llcc68_hal_write( context, buf, LLCC68_SIZE_SET_PA_CFG, 0, 0 );
}

llcc68_status_t llcc68_set_rx_tx_fallback_mode( const void* context, const llcc68_fallback_modes_t fallback_mode )
{
    const uint8_t buf[LLCC68_SIZE_SET_RX_TX_FALLBACK_MODE] = {
        LLCC68_SET_RX_TX_FALLBACK_MODE,
        ( uint8_t ) fallback_mode,
    };

    return ( llcc68_status_t ) llcc68_hal_write( context, buf, LLCC68_SIZE_SET_RX_TX_FALLBACK_MODE, 0, 0 );
}

//
// Registers and buffer Access
//

llcc68_status_t llcc68_write_register( const void* context, const uint16_t address, const uint8_t* buffer,
                                       const uint8_t size )
{
    const uint8_t buf[LLCC68_SIZE_WRITE_REGISTER] = {
        LLCC68_WRITE_REGISTER,
        ( uint8_t )( address >> 8 ),
        ( uint8_t )( address >> 0 ),
    };

    return ( llcc68_status_t ) llcc68_hal_write( context, buf, LLCC68_SIZE_WRITE_REGISTER, buffer, size );
}

llcc68_status_t llcc68_read_register( const void* context, const uint16_t address, uint8_t* buffer, const uint8_t size )
{
    const uint8_t buf[LLCC68_SIZE_READ_REGISTER] = {
        LLCC68_READ_REGISTER,
        ( uint8_t )( address >> 8 ),
        ( uint8_t )( address >> 0 ),
        LLCC68_NOP,
    };

    return ( llcc68_status_t ) llcc68_hal_read( context, buf, LLCC68_SIZE_READ_REGISTER, buffer, size );
}

llcc68_status_t llcc68_write_buffer( const void* context, const uint8_t offset, const uint8_t* buffer,
                                     const uint8_t size )
{
    const uint8_t buf[LLCC68_SIZE_WRITE_BUFFER] = {
        LLCC68_WRITE_BUFFER,
        offset,
    };

    return ( llcc68_status_t ) llcc68_hal_write( context, buf, LLCC68_SIZE_WRITE_BUFFER, buffer, size );
}

llcc68_status_t llcc68_read_buffer( const void* context, const uint8_t offset, uint8_t* buffer, const uint8_t size )
{
    const uint8_t buf[LLCC68_SIZE_READ_BUFFER] = {
        LLCC68_READ_BUFFER,
        offset,
        LLCC68_NOP,
    };

    return ( llcc68_status_t ) llcc68_hal_read( context, buf, LLCC68_SIZE_READ_BUFFER, buffer, size );
}

//
// DIO and IRQ Control Functions
//
llcc68_status_t llcc68_set_dio_irq_params( const void* context, const uint16_t irq_mask, const uint16_t dio1_mask,
                                           const uint16_t dio2_mask, const uint16_t dio3_mask )
{
    const uint8_t buf[LLCC68_SIZE_SET_DIO_IRQ_PARAMS] = {
        LLCC68_SET_DIO_IRQ_PARAMS,     ( uint8_t )( irq_mask >> 8 ),  ( uint8_t )( irq_mask >> 0 ),
        ( uint8_t )( dio1_mask >> 8 ), ( uint8_t )( dio1_mask >> 0 ), ( uint8_t )( dio2_mask >> 8 ),
        ( uint8_t )( dio2_mask >> 0 ), ( uint8_t )( dio3_mask >> 8 ), ( uint8_t )( dio3_mask >> 0 ),
    };

    return ( llcc68_status_t ) llcc68_hal_write( context, buf, LLCC68_SIZE_SET_DIO_IRQ_PARAMS, 0, 0 );
}

llcc68_status_t llcc68_get_irq_status( const void* context, llcc68_irq_mask_t* irq )
{
    const uint8_t buf[LLCC68_SIZE_GET_IRQ_STATUS] = {
        LLCC68_GET_IRQ_STATUS,
        LLCC68_NOP,
    };
    uint8_t irq_local[sizeof( llcc68_irq_mask_t )] = { 0x00 };

    const llcc68_status_t status = ( llcc68_status_t ) llcc68_hal_read( context, buf, LLCC68_SIZE_GET_IRQ_STATUS,
                                                                        irq_local, sizeof( llcc68_irq_mask_t ) );

    if( status == LLCC68_STATUS_OK )
    {
        *irq = ( ( llcc68_irq_mask_t ) irq_local[0] << 8 ) + ( ( llcc68_irq_mask_t ) irq_local[1] << 0 );
    }

    return status;
}

llcc68_status_t llcc68_clear_irq_status( const void* context, const llcc68_irq_mask_t irq_mask )
{
    const uint8_t buf[LLCC68_SIZE_CLR_IRQ_STATUS] = {
        LLCC68_CLR_IRQ_STATUS,
        ( uint8_t )( irq_mask >> 8 ),
        ( uint8_t )( irq_mask >> 0 ),
    };

    return ( llcc68_status_t ) llcc68_hal_write( context, buf, LLCC68_SIZE_CLR_IRQ_STATUS, 0, 0 );
}

llcc68_status_t llcc68_get_and_clear_irq_status( const void* context, llcc68_irq_mask_t* irq )
{
    llcc68_irq_mask_t llcc68_irq_mask = LLCC68_IRQ_NONE;

    llcc68_status_t status = llcc68_get_irq_status( context, &llcc68_irq_mask );

    if( ( status == LLCC68_STATUS_OK ) && ( llcc68_irq_mask != 0 ) )
    {
        status = llcc68_clear_irq_status( context, llcc68_irq_mask );
    }
    if( ( status == LLCC68_STATUS_OK ) && ( irq != NULL ) )
    {
        *irq = llcc68_irq_mask;
    }
    return status;
}

llcc68_status_t llcc68_set_dio2_as_rf_sw_ctrl( const void* context, const bool enable )
{
    const uint8_t buf[LLCC68_SIZE_SET_DIO2_AS_RF_SWITCH_CTRL] = {
        LLCC68_SET_DIO2_AS_RF_SWITCH_CTRL,
        ( enable == true ) ? 1 : 0,
    };

    return ( llcc68_status_t ) llcc68_hal_write( context, buf, LLCC68_SIZE_SET_DIO2_AS_RF_SWITCH_CTRL, 0, 0 );
}

llcc68_status_t llcc68_set_dio3_as_tcxo_ctrl( const void* context, const llcc68_tcxo_ctrl_voltages_t tcxo_voltage,
                                              const uint32_t timeout )
{
    const uint8_t buf[LLCC68_SIZE_SET_DIO3_AS_TCXO_CTRL] = {
        LLCC68_SET_DIO3_AS_TCXO_CTRL, ( uint8_t ) tcxo_voltage,    ( uint8_t )( timeout >> 16 ),
        ( uint8_t )( timeout >> 8 ),  ( uint8_t )( timeout >> 0 ),
    };

    return ( llcc68_status_t ) llcc68_hal_write( context, buf, LLCC68_SIZE_SET_DIO3_AS_TCXO_CTRL, 0, 0 );
}

//
// RF Modulation and Packet-Related Functions
//

llcc68_status_t llcc68_set_rf_freq( const void* context, const uint32_t freq_in_hz )
{
    const uint32_t freq = llcc68_convert_freq_in_hz_to_pll_step( freq_in_hz );

    return llcc68_set_rf_freq_in_pll_steps( context, freq );
}

llcc68_status_t llcc68_set_rf_freq_in_pll_steps( const void* context, const uint32_t freq )
{
    const uint8_t buf[LLCC68_SIZE_SET_RF_FREQUENCY] = {
        LLCC68_SET_RF_FREQUENCY,  ( uint8_t )( freq >> 24 ), ( uint8_t )( freq >> 16 ),
        ( uint8_t )( freq >> 8 ), ( uint8_t )( freq >> 0 ),
    };

    return ( llcc68_status_t ) llcc68_hal_write( context, buf, LLCC68_SIZE_SET_RF_FREQUENCY, 0, 0 );
}

llcc68_status_t llcc68_set_pkt_type( const void* context, const llcc68_pkt_type_t pkt_type )
{
    const uint8_t buf[LLCC68_SIZE_SET_PKT_TYPE] = {
        LLCC68_SET_PKT_TYPE,
        ( uint8_t ) pkt_type,
    };

    return ( llcc68_status_t ) llcc68_hal_write( context, buf, LLCC68_SIZE_SET_PKT_TYPE, 0, 0 );
}

llcc68_status_t llcc68_get_pkt_type( const void* context, llcc68_pkt_type_t* pkt_type )
{
    const uint8_t buf[LLCC68_SIZE_GET_PKT_TYPE] = {
        LLCC68_GET_PKT_TYPE,
        LLCC68_NOP,
    };

    return ( llcc68_status_t ) llcc68_hal_read( context, buf, LLCC68_SIZE_GET_PKT_TYPE, ( uint8_t* ) pkt_type, 1 );
}

llcc68_status_t llcc68_set_tx_params( const void* context, const int8_t pwr_in_dbm, const llcc68_ramp_time_t ramp_time )
{
    const uint8_t buf[LLCC68_SIZE_SET_TX_PARAMS] = {
        LLCC68_SET_TX_PARAMS,
        ( uint8_t ) pwr_in_dbm,
        ( uint8_t ) ramp_time,
    };

    return ( llcc68_status_t ) llcc68_hal_write( context, buf, LLCC68_SIZE_SET_TX_PARAMS, 0, 0 );
}

llcc68_status_t llcc68_set_gfsk_mod_params( const void* context, const llcc68_mod_params_gfsk_t* params )
{
    const uint32_t bitrate = ( uint32_t )( 32 * LLCC68_XTAL_FREQ / params->br_in_bps );
    const uint32_t fdev    = llcc68_convert_freq_in_hz_to_pll_step( params->fdev_in_hz );
    const uint8_t  buf[LLCC68_SIZE_SET_MODULATION_PARAMS_GFSK] = {
        LLCC68_SET_MODULATION_PARAMS, ( uint8_t )( bitrate >> 16 ),       ( uint8_t )( bitrate >> 8 ),
        ( uint8_t )( bitrate >> 0 ),  ( uint8_t )( params->pulse_shape ), params->bw_dsb_param,
        ( uint8_t )( fdev >> 16 ),    ( uint8_t )( fdev >> 8 ),           ( uint8_t )( fdev >> 0 ),
    };

    llcc68_status_t status =
        ( llcc68_status_t ) llcc68_hal_write( context, buf, LLCC68_SIZE_SET_MODULATION_PARAMS_GFSK, 0, 0 );

    if( status == LLCC68_STATUS_OK )
    {
        // WORKAROUND - Modulation Quality with 500 kHz LoRa Bandwidth, see DS_LLCC68_V1.0 datasheet chapter 15.1
        status = llcc68_tx_modulation_workaround( context, LLCC68_PKT_TYPE_GFSK, ( llcc68_lora_bw_t ) 0 );
        // WORKAROUND END
    }
    return status;
}

llcc68_status_t llcc68_set_bpsk_mod_params( const void* context, const llcc68_mod_params_bpsk_t* params )
{
    const uint32_t bitrate = ( uint32_t )( 32 * LLCC68_XTAL_FREQ / params->br_in_bps );

    const uint8_t buf[LLCC68_SIZE_SET_MODULATION_PARAMS_BPSK] = {
        LLCC68_SET_MODULATION_PARAMS, ( uint8_t )( bitrate >> 16 ),       ( uint8_t )( bitrate >> 8 ),
        ( uint8_t )( bitrate >> 0 ),  ( uint8_t )( params->pulse_shape ),
    };

    return ( llcc68_status_t ) llcc68_hal_write( context, buf, LLCC68_SIZE_SET_MODULATION_PARAMS_BPSK, 0, 0 );
}

llcc68_status_t llcc68_set_lora_mod_params( const void* context, const llcc68_mod_params_lora_t* params )
{
    llcc68_status_t status;

    if( ( ( params->bw == LLCC68_LORA_BW_250 ) && ( params->sf == LLCC68_LORA_SF11 ) ) ||
        ( ( params->bw == LLCC68_LORA_BW_125 ) &&
          ( ( params->sf == LLCC68_LORA_SF11 ) || ( params->sf == LLCC68_LORA_SF10 ) ) ) )
    {
        status = LLCC68_STATUS_UNSUPPORTED_FEATURE;
    }
    else
    {
        const uint8_t buf[LLCC68_SIZE_SET_MODULATION_PARAMS_LORA] = {
            LLCC68_SET_MODULATION_PARAMS, ( uint8_t )( params->sf ), ( uint8_t )( params->bw ),
            ( uint8_t )( params->cr ),    params->ldro & 0x01,
        };

        status = ( llcc68_status_t ) llcc68_hal_write( context, buf, LLCC68_SIZE_SET_MODULATION_PARAMS_LORA, 0, 0 );

        if( status == LLCC68_STATUS_OK )
        {
            // WORKAROUND - Modulation Quality with 500 kHz LoRa Bandwidth, see datasheet DS_LLCC68_V1.0 §15.1
            status = llcc68_tx_modulation_workaround( context, LLCC68_PKT_TYPE_LORA, params->bw );
            // WORKAROUND END
        }
    }

    return status;
}

llcc68_status_t llcc68_set_gfsk_pkt_params( const void* context, const llcc68_pkt_params_gfsk_t* params )
{
    const uint8_t buf[LLCC68_SIZE_SET_PKT_PARAMS_GFSK] = {
        LLCC68_SET_PKT_PARAMS,
        ( uint8_t )( params->preamble_len_in_bits >> 8 ),
        ( uint8_t )( params->preamble_len_in_bits >> 0 ),
        ( uint8_t )( params->preamble_detector ),
        params->sync_word_len_in_bits,
        ( uint8_t )( params->address_filtering ),
        ( uint8_t )( params->header_type ),
        params->pld_len_in_bytes,
        ( uint8_t )( params->crc_type ),
        ( uint8_t )( params->dc_free ),
    };

    return ( llcc68_status_t ) llcc68_hal_write( context, buf, LLCC68_SIZE_SET_PKT_PARAMS_GFSK, 0, 0 );
}

llcc68_status_t llcc68_set_bpsk_pkt_params( const void* context, const llcc68_pkt_params_bpsk_t* params )
{
    const uint8_t buf[LLCC68_SIZE_SET_PKT_PARAMS_BPSK] = {
        LLCC68_SET_PKT_PARAMS,
        params->pld_len_in_bytes,
    };

    llcc68_status_t status =
        ( llcc68_status_t ) llcc68_hal_write( context, buf, LLCC68_SIZE_SET_PKT_PARAMS_BPSK, 0, 0 );
    if( status != LLCC68_STATUS_OK )
    {
        return status;
    }

    const uint8_t buf2[] = {
        ( uint8_t )( params->ramp_up_delay >> 8 ),   ( uint8_t )( params->ramp_up_delay >> 0 ),
        ( uint8_t )( params->ramp_down_delay >> 8 ), ( uint8_t )( params->ramp_down_delay >> 0 ),
        ( uint8_t )( params->pld_len_in_bits >> 8 ), ( uint8_t )( params->pld_len_in_bits >> 0 ),
    };

    return llcc68_write_register( context, 0x00F0, buf2, sizeof( buf2 ) );
}

llcc68_status_t llcc68_set_lora_pkt_params( const void* context, const llcc68_pkt_params_lora_t* params )
{
    const uint8_t buf[LLCC68_SIZE_SET_PKT_PARAMS_LORA] = {
        LLCC68_SET_PKT_PARAMS,
        ( uint8_t )( params->preamble_len_in_symb >> 8 ),
        ( uint8_t )( params->preamble_len_in_symb >> 0 ),
        ( uint8_t )( params->header_type ),
        params->pld_len_in_bytes,
        ( uint8_t )( params->crc_is_on ? 1 : 0 ),
        ( uint8_t )( params->invert_iq_is_on ? 1 : 0 ),
    };

    llcc68_status_t status =
        ( llcc68_status_t ) llcc68_hal_write( context, buf, LLCC68_SIZE_SET_PKT_PARAMS_LORA, 0, 0 );

    // WORKAROUND - Optimizing the Inverted IQ Operation, see datasheet DS_LLCC68_V1.0 §15.4
    if( status == LLCC68_STATUS_OK )
    {
        uint8_t reg_value = 0;

        status = llcc68_read_register( context, LLCC68_REG_IQ_POLARITY, &reg_value, 1 );
        if( status == LLCC68_STATUS_OK )
        {
            if( params->invert_iq_is_on == true )
            {
                reg_value &= ~( 1 << 2 );  // Bit 2 set to 0 when using inverted IQ polarity
            }
            else
            {
                reg_value |= ( 1 << 2 );  // Bit 2 set to 1 when using standard IQ polarity
            }
            status = llcc68_write_register( context, LLCC68_REG_IQ_POLARITY, &reg_value, 1 );
        }
    }
    // WORKAROUND END

    return status;
}

llcc68_status_t llcc68_set_gfsk_pkt_address( const void* context, const uint8_t node_address,
                                             const uint8_t broadcast_address )
{
    const uint8_t addresses[2] = { node_address, broadcast_address };

    return llcc68_write_register( context, LLCC68_REG_GFSK_NODE_ADDRESS, addresses, 2 );
}

llcc68_status_t llcc68_set_cad_params( const void* context, const llcc68_cad_params_t* params )
{
    const uint8_t buf[LLCC68_SIZE_SET_CAD_PARAMS] = {
        LLCC68_SET_CAD_PARAMS,
        ( uint8_t ) params->cad_symb_nb,
        params->cad_detect_peak,
        params->cad_detect_min,
        ( uint8_t ) params->cad_exit_mode,
        ( uint8_t )( params->cad_timeout >> 16 ),
        ( uint8_t )( params->cad_timeout >> 8 ),
        ( uint8_t )( params->cad_timeout >> 0 ),
    };

    return ( llcc68_status_t ) llcc68_hal_write( context, buf, LLCC68_SIZE_SET_CAD_PARAMS, 0, 0 );
}

llcc68_status_t llcc68_set_buffer_base_address( const void* context, const uint8_t tx_base_address,
                                                const uint8_t rx_base_address )
{
    const uint8_t buf[LLCC68_SIZE_SET_BUFFER_BASE_ADDRESS] = {
        LLCC68_SET_BUFFER_BASE_ADDRESS,
        tx_base_address,
        rx_base_address,
    };

    return ( llcc68_status_t ) llcc68_hal_write( context, buf, LLCC68_SIZE_SET_BUFFER_BASE_ADDRESS, 0, 0 );
}

llcc68_status_t llcc68_set_lora_symb_nb_timeout( const void* context, const uint8_t nb_of_symbs )
{
    uint8_t exp = 0;
    uint8_t mant =
        ( ( ( nb_of_symbs > LLCC68_MAX_LORA_SYMB_NUM_TIMEOUT ) ? LLCC68_MAX_LORA_SYMB_NUM_TIMEOUT : nb_of_symbs ) +
          1 ) >>
        1;

    while( mant > 31 )
    {
        mant = ( mant + 3 ) >> 2;
        exp++;
    }

    const uint8_t buf[LLCC68_SIZE_SET_LORA_SYMB_NUM_TIMEOUT] = {
        LLCC68_SET_LORA_SYMB_NUM_TIMEOUT,
        mant << ( 2 * exp + 1 ),
    };

    llcc68_status_t status =
        ( llcc68_status_t ) llcc68_hal_write( context, buf, LLCC68_SIZE_SET_LORA_SYMB_NUM_TIMEOUT, 0, 0 );

    if( ( status == LLCC68_STATUS_OK ) && ( nb_of_symbs > 0 ) )
    {
        uint8_t reg = exp + ( mant << 3 );
        status      = llcc68_write_register( context, LLCC68_REG_LR_SYNCH_TIMEOUT, &reg, 1 );
    }

    return status;
}

//
// Communication Status Information
//

llcc68_status_t llcc68_get_status( const void* context, llcc68_chip_status_t* radio_status )
{
    const uint8_t buf[LLCC68_SIZE_GET_STATUS] = {
        LLCC68_GET_STATUS,
    };
    uint8_t status_local = 0;

    const llcc68_status_t status =
        ( llcc68_status_t ) llcc68_hal_read( context, buf, LLCC68_SIZE_GET_STATUS, &status_local, 1 );

    if( status == LLCC68_STATUS_OK )
    {
        radio_status->cmd_status =
            ( llcc68_cmd_status_t )( ( status_local & LLCC68_CMD_STATUS_MASK ) >> LLCC68_CMD_STATUS_POS );
        radio_status->chip_mode =
            ( llcc68_chip_modes_t )( ( status_local & LLCC68_CHIP_MODES_MASK ) >> LLCC68_CHIP_MODES_POS );
    }

    return status;
}

llcc68_status_t llcc68_get_rx_buffer_status( const void* context, llcc68_rx_buffer_status_t* rx_buffer_status )
{
    const uint8_t buf[LLCC68_SIZE_GET_RX_BUFFER_STATUS] = {
        LLCC68_GET_RX_BUFFER_STATUS,
        LLCC68_NOP,
    };
    uint8_t status_local[sizeof( llcc68_rx_buffer_status_t )] = { 0x00 };

    const llcc68_status_t status = ( llcc68_status_t ) llcc68_hal_read(
        context, buf, LLCC68_SIZE_GET_RX_BUFFER_STATUS, status_local, sizeof( llcc68_rx_buffer_status_t ) );

    if( status == LLCC68_STATUS_OK )
    {
        rx_buffer_status->pld_len_in_bytes     = status_local[0];
        rx_buffer_status->buffer_start_pointer = status_local[1];
    }

    return status;
}

llcc68_status_t llcc68_get_gfsk_pkt_status( const void* context, llcc68_pkt_status_gfsk_t* pkt_status )
{
    const uint8_t buf[LLCC68_SIZE_GET_PKT_STATUS] = {
        LLCC68_GET_PKT_STATUS,
        LLCC68_NOP,
    };
    uint8_t pkt_status_local[3] = { 0x00 };

    const llcc68_status_t status =
        ( llcc68_status_t ) llcc68_hal_read( context, buf, LLCC68_SIZE_GET_PKT_STATUS, pkt_status_local, 3 );

    if( status == LLCC68_STATUS_OK )
    {
        pkt_status->rx_status.pkt_sent =
            ( ( pkt_status_local[0] & LLCC68_GFSK_RX_STATUS_PKT_SENT_MASK ) != 0 ) ? true : false;
        pkt_status->rx_status.pkt_received =
            ( ( pkt_status_local[0] & LLCC68_GFSK_RX_STATUS_PKT_RECEIVED_MASK ) != 0 ) ? true : false;
        pkt_status->rx_status.abort_error =
            ( ( pkt_status_local[0] & LLCC68_GFSK_RX_STATUS_ABORT_ERROR_MASK ) != 0 ) ? true : false;
        pkt_status->rx_status.length_error =
            ( ( pkt_status_local[0] & LLCC68_GFSK_RX_STATUS_LENGTH_ERROR_MASK ) != 0 ) ? true : false;
        pkt_status->rx_status.crc_error =
            ( ( pkt_status_local[0] & LLCC68_GFSK_RX_STATUS_CRC_ERROR_MASK ) != 0 ) ? true : false;
        pkt_status->rx_status.adrs_error =
            ( ( pkt_status_local[0] & LLCC68_GFSK_RX_STATUS_ADRS_ERROR_MASK ) != 0 ) ? true : false;

        pkt_status->rssi_sync = ( int8_t )( -pkt_status_local[1] >> 1 );
        pkt_status->rssi_avg  = ( int8_t )( -pkt_status_local[2] >> 1 );
    }

    return status;
}

llcc68_status_t llcc68_get_lora_pkt_status( const void* context, llcc68_pkt_status_lora_t* pkt_status )
{
    const uint8_t buf[LLCC68_SIZE_GET_PKT_STATUS] = {
        LLCC68_GET_PKT_STATUS,
        LLCC68_NOP,
    };
    uint8_t pkt_status_local[sizeof( llcc68_pkt_status_lora_t )] = { 0x00 };

    const llcc68_status_t status = ( llcc68_status_t ) llcc68_hal_read(
        context, buf, LLCC68_SIZE_GET_PKT_STATUS, pkt_status_local, sizeof( llcc68_pkt_status_lora_t ) );

    if( status == LLCC68_STATUS_OK )
    {
        pkt_status->rssi_pkt_in_dbm        = ( int8_t )( -pkt_status_local[0] >> 1 );
        pkt_status->snr_pkt_in_db          = ( ( ( int8_t ) pkt_status_local[1] ) + 2 ) >> 2;
        pkt_status->signal_rssi_pkt_in_dbm = ( int8_t )( -pkt_status_local[2] >> 1 );
    }

    return status;
}

llcc68_status_t llcc68_get_rssi_inst( const void* context, int16_t* rssi_in_dbm )
{
    const uint8_t buf[LLCC68_SIZE_GET_RSSI_INST] = {
        LLCC68_GET_RSSI_INST,
        LLCC68_NOP,
    };
    uint8_t rssi_local = 0x00;

    const llcc68_status_t status =
        ( llcc68_status_t ) llcc68_hal_read( context, buf, LLCC68_SIZE_GET_RSSI_INST, &rssi_local, 1 );

    if( status == LLCC68_STATUS_OK )
    {
        *rssi_in_dbm = ( int8_t )( -rssi_local >> 1 );
    }

    return status;
}

llcc68_status_t llcc68_get_gfsk_stats( const void* context, llcc68_stats_gfsk_t* stats )
{
    const uint8_t buf[LLCC68_SIZE_GET_STATS] = {
        LLCC68_GET_STATS,
        LLCC68_NOP,
    };
    uint8_t stats_local[sizeof( llcc68_stats_gfsk_t )] = { 0 };

    const llcc68_status_t status = ( llcc68_status_t ) llcc68_hal_read( context, buf, LLCC68_SIZE_GET_STATS,
                                                                        stats_local, sizeof( llcc68_stats_gfsk_t ) );

    if( status == LLCC68_STATUS_OK )
    {
        stats->nb_pkt_received  = ( ( uint16_t ) stats_local[0] << 8 ) + ( uint16_t ) stats_local[1];
        stats->nb_pkt_crc_error = ( ( uint16_t ) stats_local[2] << 8 ) + ( uint16_t ) stats_local[3];
        stats->nb_pkt_len_error = ( ( uint16_t ) stats_local[4] << 8 ) + ( uint16_t ) stats_local[5];
    }

    return status;
}

llcc68_status_t llcc68_get_lora_stats( const void* context, llcc68_stats_lora_t* stats )
{
    const uint8_t buf[LLCC68_SIZE_GET_STATS] = {
        LLCC68_GET_STATS,
        LLCC68_NOP,
    };
    uint8_t stats_local[sizeof( llcc68_stats_lora_t )] = { 0 };

    const llcc68_status_t status = ( llcc68_status_t ) llcc68_hal_read( context, buf, LLCC68_SIZE_GET_STATS,
                                                                        stats_local, sizeof( llcc68_stats_lora_t ) );

    if( status == LLCC68_STATUS_OK )
    {
        stats->nb_pkt_received     = ( ( uint16_t ) stats_local[0] << 8 ) + ( uint16_t ) stats_local[1];
        stats->nb_pkt_crc_error    = ( ( uint16_t ) stats_local[2] << 8 ) + ( uint16_t ) stats_local[3];
        stats->nb_pkt_header_error = ( ( uint16_t ) stats_local[4] << 8 ) + ( uint16_t ) stats_local[5];
    }
    return status;
}

llcc68_status_t llcc68_reset_stats( const void* context )
{
    const uint8_t buf[LLCC68_SIZE_RESET_STATS] = {
        LLCC68_RESET_STATS, LLCC68_NOP, LLCC68_NOP, LLCC68_NOP, LLCC68_NOP, LLCC68_NOP, LLCC68_NOP,
    };

    return ( llcc68_status_t ) llcc68_hal_write( context, buf, LLCC68_SIZE_RESET_STATS, 0, 0 );
}

//
// Miscellaneous
//

llcc68_status_t llcc68_reset( const void* context )
{
    return ( llcc68_status_t ) llcc68_hal_reset( context );
}

llcc68_status_t llcc68_wakeup( const void* context )
{
    return ( llcc68_status_t ) llcc68_hal_wakeup( context );
}

llcc68_status_t llcc68_get_device_errors( const void* context, llcc68_errors_mask_t* errors )
{
    const uint8_t buf[LLCC68_SIZE_GET_DEVICE_ERRORS] = {
        LLCC68_GET_DEVICE_ERRORS,
        LLCC68_NOP,
    };
    uint8_t errors_local[sizeof( llcc68_errors_mask_t )] = { 0x00 };

    const llcc68_status_t status = ( llcc68_status_t ) llcc68_hal_read( context, buf, LLCC68_SIZE_GET_DEVICE_ERRORS,
                                                                        errors_local, sizeof( llcc68_errors_mask_t ) );

    if( status == LLCC68_STATUS_OK )
    {
        *errors = ( ( llcc68_errors_mask_t ) errors_local[0] << 8 ) + ( ( llcc68_errors_mask_t ) errors_local[1] << 0 );
    }

    return status;
}

llcc68_status_t llcc68_clear_device_errors( const void* context )
{
    const uint8_t buf[LLCC68_SIZE_CLR_DEVICE_ERRORS] = {
        LLCC68_CLR_DEVICE_ERRORS,
        LLCC68_NOP,
        LLCC68_NOP,
    };

    return ( llcc68_status_t ) llcc68_hal_write( context, buf, LLCC68_SIZE_CLR_DEVICE_ERRORS, 0, 0 );
}

llcc68_status_t llcc68_get_gfsk_bw_param( const uint32_t bw, uint8_t* param )
{
    llcc68_status_t status = LLCC68_STATUS_ERROR;

    if( bw != 0 )
    {
        status = LLCC68_STATUS_UNKNOWN_VALUE;
        for( uint8_t i = 0; i < ( sizeof( gfsk_bw ) / sizeof( gfsk_bw_t ) ); i++ )
        {
            if( bw <= gfsk_bw[i].bw )
            {
                *param = gfsk_bw[i].param;
                status = LLCC68_STATUS_OK;
                break;
            }
        }
    }

    return status;
}

uint32_t llcc68_get_lora_bw_in_hz( llcc68_lora_bw_t bw )
{
    uint32_t bw_in_hz = 0;

    switch( bw )
    {
    case LLCC68_LORA_BW_007:
        bw_in_hz = 7812UL;
        break;
    case LLCC68_LORA_BW_010:
        bw_in_hz = 10417UL;
        break;
    case LLCC68_LORA_BW_015:
        bw_in_hz = 15625UL;
        break;
    case LLCC68_LORA_BW_020:
        bw_in_hz = 20833UL;
        break;
    case LLCC68_LORA_BW_031:
        bw_in_hz = 31250UL;
        break;
    case LLCC68_LORA_BW_041:
        bw_in_hz = 41667UL;
        break;
    case LLCC68_LORA_BW_062:
        bw_in_hz = 62500UL;
        break;
    case LLCC68_LORA_BW_125:
        bw_in_hz = 125000UL;
        break;
    case LLCC68_LORA_BW_250:
        bw_in_hz = 250000UL;
        break;
    case LLCC68_LORA_BW_500:
        bw_in_hz = 500000UL;
        break;
    }

    return bw_in_hz;
}

uint32_t llcc68_get_lora_time_on_air_numerator( const llcc68_pkt_params_lora_t* pkt_p,
                                                const llcc68_mod_params_lora_t* mod_p )
{
    const int32_t pld_len_in_bytes = pkt_p->pld_len_in_bytes;
    const int32_t sf               = mod_p->sf;
    const bool    pld_is_fix       = pkt_p->header_type == LLCC68_LORA_PKT_IMPLICIT;
    const int32_t cr_denom         = mod_p->cr + 4;

    int32_t ceil_denominator;
    int32_t ceil_numerator =
        ( pld_len_in_bytes << 3 ) + ( pkt_p->crc_is_on ? 16 : 0 ) - ( 4 * sf ) + ( pld_is_fix ? 0 : 20 );

    if( sf <= 6 )
    {
        ceil_denominator = 4 * sf;
    }
    else
    {
        ceil_numerator += 8;

        if( mod_p->ldro )
        {
            ceil_denominator = 4 * ( sf - 2 );
        }
        else
        {
            ceil_denominator = 4 * sf;
        }
    }

    if( ceil_numerator < 0 )
    {
        ceil_numerator = 0;
    }

    // Perform integral ceil()
    int32_t intermed =
        ( ( ceil_numerator + ceil_denominator - 1 ) / ceil_denominator ) * cr_denom + pkt_p->preamble_len_in_symb + 12;

    if( sf <= 6 )
    {
        intermed += 2;
    }

    return ( uint32_t )( ( 4 * intermed + 1 ) * ( 1 << ( sf - 2 ) ) );
}

uint32_t llcc68_get_lora_time_on_air_in_ms( const llcc68_pkt_params_lora_t* pkt_p,
                                            const llcc68_mod_params_lora_t* mod_p )
{
    uint32_t numerator   = 1000U * llcc68_get_lora_time_on_air_numerator( pkt_p, mod_p );
    uint32_t denominator = llcc68_get_lora_bw_in_hz( mod_p->bw );
    // Perform integral ceil()
    return ( numerator + denominator - 1 ) / denominator;
}

uint32_t llcc68_get_gfsk_time_on_air_numerator( const llcc68_pkt_params_gfsk_t* pkt_p )
{
    return pkt_p->preamble_len_in_bits + ( pkt_p->header_type == LLCC68_GFSK_PKT_VAR_LEN ? 8 : 0 ) +
           pkt_p->sync_word_len_in_bits +
           ( ( pkt_p->pld_len_in_bytes + ( pkt_p->address_filtering == LLCC68_GFSK_ADDRESS_FILTERING_DISABLE ? 0 : 1 ) +
               llcc68_get_gfsk_crc_len_in_bytes( pkt_p->crc_type ) )
             << 3 );
}

uint32_t llcc68_get_gfsk_time_on_air_in_ms( const llcc68_pkt_params_gfsk_t* pkt_p,
                                            const llcc68_mod_params_gfsk_t* mod_p )
{
    uint32_t numerator   = 1000U * llcc68_get_gfsk_time_on_air_numerator( pkt_p );
    uint32_t denominator = mod_p->br_in_bps;

    // Perform integral ceil()
    return ( numerator + denominator - 1 ) / denominator;
}

llcc68_status_t llcc68_get_random_numbers( const void* context, uint32_t* numbers, unsigned int n )
{
    llcc68_status_t status;

    uint8_t tmp_ana_lna   = 0x00;
    uint8_t tmp_ana_mixer = 0x00;
    uint8_t tmp           = 0x00;

    // Configure for random number generation
    status = llcc68_read_register( context, LLCC68_REG_ANA_LNA, &tmp_ana_lna, 1 );
    if( status != LLCC68_STATUS_OK )
    {
        return status;
    }
    tmp    = tmp_ana_lna & ~( 1 << 0 );
    status = llcc68_write_register( context, LLCC68_REG_ANA_LNA, &tmp, 1 );
    if( status != LLCC68_STATUS_OK )
    {
        return status;
    }

    status = llcc68_read_register( context, LLCC68_REG_ANA_MIXER, &tmp_ana_mixer, 1 );
    if( status != LLCC68_STATUS_OK )
    {
        return status;
    }
    tmp    = tmp_ana_mixer & ~( 1 << 7 );
    status = llcc68_write_register( context, LLCC68_REG_ANA_MIXER, &tmp, 1 );
    if( status != LLCC68_STATUS_OK )
    {
        return status;
    }

    // Start RX continuous
    status = llcc68_set_rx_with_timeout_in_rtc_step( context, LLCC68_RX_CONTINUOUS );
    if( status != LLCC68_STATUS_OK )
    {
        return status;
    }

    // Store values
    for( unsigned int i = 0; i < n; i++ )
    {
        status = llcc68_read_register( context, LLCC68_REG_RNGBASEADDRESS, ( uint8_t* ) &numbers[i], 4 );
        if( status != LLCC68_STATUS_OK )
        {
            return status;
        }
    }

    status = llcc68_set_standby( context, LLCC68_STANDBY_CFG_RC );
    if( status != LLCC68_STATUS_OK )
    {
        return status;
    }

    // Restore registers
    status = llcc68_write_register( context, LLCC68_REG_ANA_LNA, &tmp_ana_lna, 1 );
    if( status != LLCC68_STATUS_OK )
    {
        return status;
    }
    status = llcc68_write_register( context, LLCC68_REG_ANA_MIXER, &tmp_ana_mixer, 1 );

    return status;
}

uint32_t llcc68_convert_freq_in_hz_to_pll_step( uint32_t freq_in_hz )
{
    uint32_t steps_int;
    uint32_t steps_frac;

    // Get integer and fractional parts of the frequency computed with a PLL step scaled value
    steps_int  = freq_in_hz / LLCC68_PLL_STEP_SCALED;
    steps_frac = freq_in_hz - ( steps_int * LLCC68_PLL_STEP_SCALED );

    // Apply the scaling factor to retrieve a frequency in Hz (+ ceiling)
    return ( steps_int << LLCC68_PLL_STEP_SHIFT_AMOUNT ) +
           ( ( ( steps_frac << LLCC68_PLL_STEP_SHIFT_AMOUNT ) + ( LLCC68_PLL_STEP_SCALED >> 1 ) ) /
             LLCC68_PLL_STEP_SCALED );
}

uint32_t llcc68_convert_timeout_in_ms_to_rtc_step( uint32_t timeout_in_ms )
{
    return ( uint32_t )( timeout_in_ms * ( LLCC68_RTC_FREQ_IN_HZ / 1000 ) );
}

llcc68_status_t llcc68_handle_rx_done( const void* context )
{
    return llcc68_stop_rtc( context );
}

//
// Registers access
//

llcc68_status_t llcc68_cfg_rx_boosted( const void* context, const bool state )
{
    if( state == true )
    {
        return llcc68_write_register( context, LLCC68_REG_RXGAIN, ( const uint8_t[] ){ 0x96 }, 1 );
    }
    else
    {
        return llcc68_write_register( context, LLCC68_REG_RXGAIN, ( const uint8_t[] ){ 0x94 }, 1 );
    }
}

llcc68_status_t llcc68_set_gfsk_sync_word( const void* context, const uint8_t* sync_word, const uint8_t sync_word_len )
{
    llcc68_status_t status = LLCC68_STATUS_ERROR;

    if( sync_word_len <= 8 )
    {
        uint8_t buf[8] = { 0 };

        for( int i = 0; i < sync_word_len; i++ )
        {
            buf[i] = sync_word[i];
        }
        status = llcc68_write_register( context, LLCC68_REG_SYNCWORDBASEADDRESS, buf, 8 );
    }

    return status;
}

llcc68_status_t llcc68_set_lora_sync_word( const void* context, const uint8_t sync_word )
{
    uint8_t buffer[2] = { 0x00 };

    llcc68_status_t status = llcc68_read_register( context, LLCC68_REG_LR_SYNCWORD, buffer, 2 );

    if( status == LLCC68_STATUS_OK )
    {
        buffer[0] = ( buffer[0] & ~0xF0 ) + ( sync_word & 0xF0 );
        buffer[1] = ( buffer[1] & ~0xF0 ) + ( ( sync_word & 0x0F ) << 4 );

        status = llcc68_write_register( context, LLCC68_REG_LR_SYNCWORD, buffer, 2 );
    }

    return status;
}

llcc68_status_t llcc68_set_gfsk_crc_seed( const void* context, uint16_t seed )
{
    uint8_t s[] = { ( uint8_t )( seed >> 8 ), ( uint8_t ) seed };

    return llcc68_write_register( context, LLCC68_REG_CRCSEEDBASEADDRESS, s, sizeof( s ) );
}

llcc68_status_t llcc68_set_gfsk_crc_polynomial( const void* context, const uint16_t polynomial )
{
    uint8_t poly[] = { ( uint8_t )( polynomial >> 8 ), ( uint8_t ) polynomial };

    return llcc68_write_register( context, LLCC68_REG_CRCPOLYBASEADDRESS, poly, sizeof( poly ) );
}

llcc68_status_t llcc68_set_gfsk_whitening_seed( const void* context, const uint16_t seed )
{
    uint8_t reg_value = 0;

    // The LLCC68_REG_WHITSEEDBASEADDRESS @ref LSBit is used for the seed value. The 7 MSBits must not be modified.
    // Thus, we first need to read the current value and then change the LSB according to the provided seed @ref value.
    llcc68_status_t status = llcc68_read_register( context, LLCC68_REG_WHITSEEDBASEADDRESS, &reg_value, 1 );
    if( status == LLCC68_STATUS_OK )
    {
        reg_value = ( reg_value & 0xFE ) | ( ( uint8_t )( seed >> 8 ) & 0x01 );
        status    = llcc68_write_register( context, LLCC68_REG_WHITSEEDBASEADDRESS, &reg_value, 1 );
        if( status == LLCC68_STATUS_OK )
        {
            reg_value = ( uint8_t ) seed;
            status    = llcc68_write_register( context, LLCC68_REG_WHITSEEDBASEADDRESS + 1, &reg_value, 1 );
        }
    }

    return status;
}

llcc68_status_t llcc68_cfg_tx_clamp( const void* context )
{
    uint8_t reg_value = 0x00;

    llcc68_status_t status = llcc68_read_register( context, LLCC68_REG_TX_CLAMP_CFG, &reg_value, 1 );

    if( status == LLCC68_STATUS_OK )
    {
        reg_value |= LLCC68_REG_TX_CLAMP_CFG_MASK;
        status = llcc68_write_register( context, LLCC68_REG_TX_CLAMP_CFG, &reg_value, 1 );
    }

    return status;
}

llcc68_status_t llcc68_stop_rtc( const void* context )
{
    uint8_t reg_value = 0;

    llcc68_status_t status = llcc68_write_register( context, LLCC68_REG_RTC_CTRL, &reg_value, 1 );

    if( status == LLCC68_STATUS_OK )
    {
        status = llcc68_read_register( context, LLCC68_REG_EVT_CLR, &reg_value, 1 );

        if( status == LLCC68_STATUS_OK )
        {
            reg_value |= LLCC68_REG_EVT_CLR_TIMEOUT_MASK;
            status = llcc68_write_register( context, LLCC68_REG_EVT_CLR, &reg_value, 1 );
        }
    }

    return status;
}

llcc68_status_t llcc68_set_ocp_value( const void* context, const uint8_t ocp_in_step_of_2_5_ma )
{
    return ( llcc68_status_t ) llcc68_write_register( context, LLCC68_REG_OCP, &ocp_in_step_of_2_5_ma, 1 );
}

llcc68_status_t llcc68_set_trimming_capacitor_values( const void* context, const uint8_t trimming_cap_xta,
                                                      const uint8_t trimming_cap_xtb )
{
    uint8_t trimming_capacitor_values[2] = { trimming_cap_xta, trimming_cap_xtb };

    return ( llcc68_status_t ) llcc68_write_register( context, LLCC68_REG_XTATRIM, trimming_capacitor_values, 2 );
}

llcc68_status_t llcc68_add_registers_to_retention_list( const void* context, const uint16_t* register_addr,
                                                        uint8_t register_nb )
{
    uint8_t buffer[9] = {0};

    llcc68_status_t status = llcc68_read_register( context, LLCC68_REG_RETENTION_LIST_BASE_ADDRESS, buffer, 9 );

    if( status == LLCC68_STATUS_OK )
    {
        const uint8_t initial_nb_of_registers = buffer[0];
        uint8_t*      register_list           = &buffer[1];

        for( uint8_t index = 0; index < register_nb; index++ )
        {
            bool register_has_to_be_added = true;

            // Check if the current register is already added to the list
            for( uint8_t i = 0; i < buffer[0]; i++ )
            {
                if( register_addr[index] == ( ( uint16_t ) register_list[2 * i] << 8 ) + register_list[2 * i + 1] )
                {
                    register_has_to_be_added = false;
                    break;
                }
            }

            if( register_has_to_be_added == true )
            {
                if( buffer[0] < LLCC68_MAX_NB_REG_IN_RETENTION )
                {
                    register_list[2 * buffer[0]]     = ( uint8_t )( register_addr[index] >> 8 );
                    register_list[2 * buffer[0] + 1] = ( uint8_t )( register_addr[index] >> 0 );
                    buffer[0] += 1;
                }
                else
                {
                    return LLCC68_STATUS_ERROR;
                }
            }
        }

        if( buffer[0] != initial_nb_of_registers )
        {
            status = llcc68_write_register( context, LLCC68_REG_RETENTION_LIST_BASE_ADDRESS, buffer, 9 );
        }
    }

    return status;
}

llcc68_status_t llcc68_init_retention_list( const void* context )
{
    const uint16_t list_of_registers[3] = { LLCC68_REG_RXGAIN, LLCC68_REG_TX_MODULATION, LLCC68_REG_IQ_POLARITY };

    return llcc68_add_registers_to_retention_list( context, list_of_registers,
                                                   sizeof( list_of_registers ) / sizeof( list_of_registers[0] ) );
}

llcc68_status_t llcc68_get_lora_params_from_header( const void* context, llcc68_lora_cr_t* cr, bool* crc_is_on )
{
    uint8_t buffer_cr = 0;
    uint8_t buffer_crc = 0;

    llcc68_status_t status = llcc68_read_register( context, LLCC68_REG_LR_HEADER_CR, &buffer_cr, 1 );

    if( status == LLCC68_STATUS_OK )
    {
        status = llcc68_read_register( context, LLCC68_REG_LR_HEADER_CRC, &buffer_crc, 1 );

        if( status == LLCC68_STATUS_OK )
        {
            *cr = ( llcc68_lora_cr_t )( ( buffer_cr & LLCC68_REG_LR_HEADER_CR_MASK ) >> LLCC68_REG_LR_HEADER_CR_POS );
            *crc_is_on = ( ( buffer_crc & LLCC68_REG_LR_HEADER_CRC_MASK ) != 0 ) ? true : false;
        }
    }

    return status;
}

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DEFINITION --------------------------------------------
 */

static llcc68_status_t llcc68_tx_modulation_workaround( const void* context, llcc68_pkt_type_t pkt_type,
                                                        llcc68_lora_bw_t bw )
{
    uint8_t reg_value = 0;

    llcc68_status_t status = llcc68_read_register( context, LLCC68_REG_TX_MODULATION, &reg_value, 1 );

    if( status == LLCC68_STATUS_OK )
    {
        if( pkt_type == LLCC68_PKT_TYPE_LORA )
        {
            if( bw == LLCC68_LORA_BW_500 )
            {
                reg_value &= ~( 1 << 2 );  // Bit 2 set to 0 if the LoRa BW = 500 kHz
            }
            else
            {
                reg_value |= ( 1 << 2 );  // Bit 2 set to 1 for any other LoRa BW
            }
        }
        else
        {
            reg_value |= ( 1 << 2 );  // Bit 2 set to 1 for any (G)FSK configuration
        }

        status = llcc68_write_register( context, LLCC68_REG_TX_MODULATION, &reg_value, 1 );
    }
    return status;
}

static inline uint32_t llcc68_get_gfsk_crc_len_in_bytes( llcc68_gfsk_crc_types_t crc_type )
{
    switch( crc_type )
    {
    case LLCC68_GFSK_CRC_OFF:
        return 0;
    case LLCC68_GFSK_CRC_1_BYTE:
        return 1;
    case LLCC68_GFSK_CRC_2_BYTES:
        return 2;
    case LLCC68_GFSK_CRC_1_BYTE_INV:
        return 1;
    case LLCC68_GFSK_CRC_2_BYTES_INV:
        return 2;
    }

    return 0;
}

/* --- EOF ------------------------------------------------------------------ */

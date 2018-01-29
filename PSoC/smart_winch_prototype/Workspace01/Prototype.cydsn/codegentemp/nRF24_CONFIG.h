/****************************************************************************
* This file is part of the nRF24 Custom Component for PSoC Devices.
*
* Copyright (C) 2017 Carlos Diaz <carlos.santiago.diaz@gmail.com>
*
* Permission to use, copy, modify, and/or distribute this software for any
* purpose with or without fee is hereby granted, provided that the above
* copyright notice and this permission notice appear in all copies.
*
* THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
* WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
* MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
* ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
* WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
* ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF
* OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.
****************************************************************************/

#ifndef nRF24_CONFIG_H
#define nRF24_CONFIG_H

#include "project.h"

// GPIO headers
#if defined(CY_GPIO_H)
    #include "gpio/cy_gpio.h"
#else
    #include "CE.h"
    #include "SS.h"
#endif

// SPI headers
#include "SPI.h"
#if defined(CY_SCB_SPI_H)
    #if !defined(SPI_CY_SCB_SPI_PDL_H)
        #include "SPI_SPI_UART.h"
    #endif
#endif

#if 0
#ifdef (CY_SCB_SPI_H)
#define NRF_PSOC4_SCB   1
#endif

#ifdef (SPI_CY_SCB_SPI_PDL_H)
#define NRF_PSOC6       1
#endif
#endif

// Component version info
#define nRF24_VERSION_MAJOR  1
#define nRF24_VERSION_MINOR  6
#define nRF24_VERSION        1.6

// Data from customizer
#define ENABLE_PIPE0    1
#define ENABLE_PIPE1    0
#define ENABLE_PIPE2    0
#define ENABLE_PIPE3    0
#define ENABLE_PIPE4    0
#define ENABLE_PIPE5    0

#define CUSTOMIZER_EN_AA        ((0 << NRF_EN_AA_ENAA_P5) | (0 << NRF_EN_AA_ENAA_P4) | \
                                (0 << NRF_EN_AA_ENAA_P3) | (0 << NRF_EN_AA_ENAA_P2) | \
                                (0 << NRF_EN_AA_ENAA_P1) | (0 << NRF_EN_AA_ENAA_P0))
#define CUSTOMIZER_EN_RXADDR    ((0 << NRF_EN_RXADDR_ERX_P5) | (0 << NRF_EN_RXADDR_ERX_P4) | \
                                (0 << NRF_EN_RXADDR_ERX_P3) | (0 << NRF_EN_RXADDR_ERX_P2) | \
                                (0 << NRF_EN_RXADDR_ERX_P1) | (1 << NRF_EN_RXADDR_ERX_P0))
#define CUSTOMIZER_SETUP_AW     (3)
#define CUSTOMIZER_SETUP_RETR   ((1 << NRF_SETUP_RETR_ARD) | (8 << NRF_SETUP_RETR_ARC))
#define CUSTOMIZER_RF_CH        (120)
#define CUSTOMIZER_RF_SETUP     ((0 << NRF_RF_SETUP_CONT_WAVE) | (1 << NRF_RF_SETUP_RF_DR) | \
                                (3 << NRF_RF_SETUP_RF_PWR))
#define CUSTOMIZER_DYNPD        ((0 << NRF_DYNPD_DPL_P5) | (0 << NRF_DYNPD_DPL_P4) | \
                                (0 << NRF_DYNPD_DPL_P3) | (0 << NRF_DYNPD_DPL_P2) | \
                                (0 << NRF_DYNPD_DPL_P1) | (0 << NRF_DYNPD_DPL_P0))
#define CUSTOMIZER_FEATURE      ((0 << NRF_FEATURE_EN_DPL) | (0 << NRF_FEATURE_EN_ACK_PAY) | \
                                (1 << NRF_FEATURE_EN_DYN_ACK))
#define CUSTOMIZER_CONFIG       ((1 << NRF_CONFIG_MASK_RX_DR) | (1 << NRF_CONFIG_MASK_TX_DS) | \
                                (1 << NRF_CONFIG_MASK_MAX_RT) | (1 << NRF_CONFIG_EN_CRC) | \
                                (1 << NRF_CONFIG_CRCO) | (1<< NRF_CONFIG_PWR_UP) | (1 << NRF_CONFIG_PRIM_RX))
#define CUSTOMIZER_RX_PW_P0     (4)
#define CUSTOMIZER_RX_PW_P1     (0)
#define CUSTOMIZER_RX_PW_P2     (0)
#define CUSTOMIZER_RX_ADDR_P2   (195)
#define CUSTOMIZER_RX_PW_P3     (0)
#define CUSTOMIZER_RX_ADDR_P3   (196)
#define CUSTOMIZER_RX_PW_P4     (0)
#define CUSTOMIZER_RX_ADDR_P4   (197)
#define CUSTOMIZER_RX_PW_P5     (0)
#define CUSTOMIZER_RX_ADDR_P5   (198)
    
#endif /* nRF24_CONFIG_H */

/* [] END OF FILE */

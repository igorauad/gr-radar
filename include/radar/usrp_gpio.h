/* -*- c++ -*- */
/*
 * Copyright (c) 2021 Igor Freire (Blockstream Corp)
 *
 * This file is part of gr-radar.
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#ifndef INCLUDED_RADAR_USRP_GPIO_H
#define INCLUDED_RADAR_USRP_GPIO_H

#include <radar/api.h>
#include <uhd/usrp/multi_usrp.hpp>
#include <uhd/version.hpp>

namespace gr {
namespace radar {

#if UHD_VERSION >= 4000000

/**
 * @brief Configure USRP GPIO pin in automatic transmit/receive (ATR) mode.
 *
 * Configures the GPIO pin as an output defined automatically based on the
 * radio's transmit/receive state.
 *
 * @param usrp Shared pointer to a multi-USRP object.
 * @param pin Target GPIO pin.
 * @param high_idle Output a high value when the radio is in idle state.
 * @param high_tx_only Output a high value when the radio is in Tx-only state.
 * @param high_rx_only Output a high value when the radio is in Rx-only state.
 * @param high_full_duplex Output a high value when the radio is in full-duplex state.
 * @param bank GPIO bank index.
 * @param chan RF channel index.
 * @param mboard Motherboard index.
 */
RADAR_API void usrp_gpio_configure_atr_output(uhd::usrp::multi_usrp::sptr usrp,
                                              int pin,
                                              bool high_idle,
                                              bool high_tx_only,
                                              bool high_rx_only,
                                              bool high_full_duplex,
                                              size_t bank = 0,
                                              size_t chan = 0,
                                              size_t mboard = 0);


/**
 * @brief Configure USRP GPIO pin(s) in manual output mode.
 *
 * Configures one or multiple GPIO pins as manually-controlled output pins
 * driven by UHD.
 *
 * @param usrp Shared pointer to a multi-USRP object.
 * @param pins Target GPIO pins.
 * @param vals Manual high/low output values for each pin.
 * @param bank GPIO bank index.
 * @param chan RF channel index.
 * @param mboard Motherboard index.
 *
 * @return GPIO bank name to facilitate future changes to the manually-controlled output.
 */
RADAR_API std::string usrp_gpio_configure_manual_output(uhd::usrp::multi_usrp::sptr usrp,
                                                        const std::vector<int>& pins,
                                                        const std::vector<bool>& vals,
                                                        size_t bank = 0,
                                                        size_t chan = 0,
                                                        size_t mboard = 0);

/**
 * @brief Configure USRP GPIO pin(s) in input mode.
 *
 * @param usrp Shared pointer to a multi-USRP object.
 * @param pins Target GPIO pins.
 * @param bank GPIO bank index.
 * @param chan RF channel index.
 * @param mboard Motherboard index.
 * @return GPIO bank name to facilitate future calls to read the input pin(s).
 */
RADAR_API std::string usrp_gpio_configure_input(uhd::usrp::multi_usrp::sptr usrp,
                                                const std::vector<int>& pins,
                                                size_t bank = 0,
                                                size_t chan = 0,
                                                size_t mboard = 0);

/**
 * @brief Dump the current USRP GPIO configuration.
 *
 * @param usrp Shared pointer to a multi-USRP object.
 * @param bank GPIO bank index.
 * @param chan RF channel index.
 * @param mboard Motherboard index.
 */
RADAR_API void usrp_gpio_dump_config(uhd::usrp::multi_usrp::sptr usrp,
                                     size_t bank = 0,
                                     size_t chan = 0,
                                     size_t mboard = 0);

#else
#warning "Compiling without GPIO support"
#endif

} // namespace radar
} // namespace gr
#endif
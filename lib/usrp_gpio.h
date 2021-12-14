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

#include <uhd/usrp/multi_usrp.hpp>
#include <uhd/version.hpp>

namespace gr {
namespace radar {

#if UHD_VERSION >= 4000000
void usrp_configure_gpio(uhd::usrp::multi_usrp::sptr usrp,
                         int pin,
                         bool high_on_tx,
                         bool manual = false,
                         bool manual_val = false,
                         size_t bank = 0,
                         size_t chan = 0,
                         size_t mboard = 0);

void usrp_dump_gpio_config(uhd::usrp::multi_usrp::sptr usrp,
                           size_t bank = 0,
                           size_t chan = 0,
                           size_t mboard = 0);

#else
#warning "Compiling echotimer without GPIO support"
#endif

} // namespace radar
} // namespace gr
#endif
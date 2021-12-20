/* -*- c++ -*- */
/*
 * Copyright 2021 Igor Freire (Blockstream Corp).
 *
 * This is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 3, or (at your option)
 * any later version.
 *
 * This software is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this software; see the file COPYING.  If not, write to
 * the Free Software Foundation, Inc., 51 Franklin Street,
 * Boston, MA 02110-1301, USA.
 */

#ifndef INCLUDED_RADAR_USRP_BURST_SOURCE_C_IMPL_H
#define INCLUDED_RADAR_USRP_BURST_SOURCE_C_IMPL_H

#include <radar/usrp_burst_source_c.h>
#include <uhd/usrp/multi_usrp.hpp>

namespace gr {
namespace radar {

class usrp_burst_source_c_impl : public usrp_burst_source_c
{
private:
    double d_samp_rate;
    double d_burst_duration;              //!< Burst duration in secs
    double d_burst_period;                //!< Inter-burst interval (periodicity) in secs
    size_t d_burst_len;                   //!< Burst length in samples
    size_t d_n_sent;                      //!< Samples sent so far in the current burst
    uint64_t d_n_burst;                   //!< Burst count
    int d_gpio_pin;                       //!< GPIO pin asserted on Tx
    bool d_gpio_manual;                   //!< Whether controlling the GPIO manually
    bool d_gpio_manual_assertion_pending; //!< Whether a manual GPIO assertion is pending
    double d_gpio_guard_period;           //!< Guard period for GPIO assertion/deassertion
    std::string d_gpio_bank;              //!< GPIO bank name
    uhd::usrp::multi_usrp::sptr d_usrp;   //!< Pointer to multi-USRP object
    uhd::tx_streamer::sptr d_tx_streamer; //!< Pointer to UHD Tx streamer object
    uhd::tx_metadata_t d_tx_metadata;     //!< Tx packet metadata
    uhd::time_spec_t d_next_tx;           //!< Timestamp for the next burst Tx

public:
    usrp_burst_source_c_impl(float samp_rate,
                             float center_freq,
                             float duty_cycle,
                             float period,
                             float gain,
                             std::string args,
                             std::string wire,
                             std::string clock_source,
                             std::string time_source,
                             std::string antenna,
                             int gpio_pin,
                             float gpio_guard_period);
    ~usrp_burst_source_c_impl();

    int work(int noutput_items,
             gr_vector_const_void_star& input_items,
             gr_vector_void_star& output_items);
    void set_tx_gain(float gain);
    void set_gpio_timed(const uhd::time_spec_t& time, bool val);
};

} // namespace radar
} // namespace gr

#endif /* INCLUDED_RADAR_USRP_BURST_SOURCE_C_IMPL_H */

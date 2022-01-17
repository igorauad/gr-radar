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

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include "usrp_burst_tx_c_impl.h"
#include <gnuradio/io_signature.h>
#include <radar/usrp_gpio.h>
#include <uhd/version.hpp>
#include <algorithm>
#include <chrono>
#include <thread>

namespace gr {
namespace radar {

usrp_burst_tx_c::sptr usrp_burst_tx_c::make(float samp_rate,
                                            float center_freq,
                                            float duty_cycle,
                                            float period,
                                            float gain,
                                            std::string args,
                                            std::string wire,
                                            std::string clock_source,
                                            std::string time_source,
                                            std::string antenna,
                                            int in_gpio_pin,
                                            int out_gpio_pin,
                                            float gpio_guard_period,
                                            bool in_gpio_wait_val,
                                            bool out_gpio_tx_val)
{
    return gnuradio::get_initial_sptr(new usrp_burst_tx_c_impl(samp_rate,
                                                               center_freq,
                                                               duty_cycle,
                                                               period,
                                                               gain,
                                                               args,
                                                               wire,
                                                               clock_source,
                                                               time_source,
                                                               antenna,
                                                               in_gpio_pin,
                                                               out_gpio_pin,
                                                               gpio_guard_period,
                                                               in_gpio_wait_val,
                                                               out_gpio_tx_val));
}


/*
 * The private constructor
 */
usrp_burst_tx_c_impl::usrp_burst_tx_c_impl(float samp_rate,
                                           float center_freq,
                                           float duty_cycle,
                                           float period,
                                           float gain,
                                           std::string args,
                                           std::string wire,
                                           std::string clock_source,
                                           std::string time_source,
                                           std::string antenna,
                                           int in_gpio_pin,
                                           int out_gpio_pin,
                                           float gpio_guard_period,
                                           bool in_gpio_wait_val,
                                           bool out_gpio_tx_val)
    : gr::sync_block("usrp_burst_tx_c",
                     gr::io_signature::make(1, 1, sizeof(gr_complex)),
                     gr::io_signature::make(0, 0, 0)),
      d_samp_rate(samp_rate),
      d_burst_duration(duty_cycle * period),
      d_burst_period(period),
      d_burst_len(d_burst_duration * samp_rate),
      d_n_sent(0),
      d_n_burst(0),
      d_in_gpio_pin(in_gpio_pin),
      d_out_gpio_pin(out_gpio_pin),
      d_gpio_manual(out_gpio_pin != -1 && gpio_guard_period > 0.0),
      d_gpio_manual_change_pending(d_gpio_manual), // start pending in manual mode
      d_gpio_guard_period(gpio_guard_period),
      d_in_gpio_wait_val(in_gpio_wait_val),
      d_out_gpio_tx_val(out_gpio_tx_val)
{
    if (duty_cycle > 1.0 || duty_cycle < 0.0) {
        throw std::runtime_error("Invalid duty cycle (not in [0,1] range)");
    }
    if (gpio_guard_period < 0.0) {
        throw std::runtime_error("GPIO guard period must be non-negative");
    }
    const double idle_time = d_burst_period - d_burst_duration;
    if ((2 * gpio_guard_period) >= 0.9 * idle_time) {
        throw std::runtime_error("Insufficient idle time for the GPIO guard periods");
    }

    // USRP Configuration
    d_usrp = uhd::usrp::multi_usrp::make(args);
    std::cout << "Using USRP Device (TX): " << std::endl
              << d_usrp->get_pp_string() << std::endl;

    std::cout << "Setting TX Rate: " << d_samp_rate << std::endl;
    d_usrp->set_tx_rate(d_samp_rate);
    std::cout << "Actual TX Rate: " << d_usrp->get_tx_rate() << std::endl;
    uhd::tune_request_t tune_request(center_freq);
    set_tx_gain(gain);
    d_usrp->set_tx_freq(tune_request);
    d_usrp->set_tx_antenna(antenna);
    d_usrp->set_clock_source(clock_source);
    d_usrp->set_time_source(time_source);

    // Reset the time notion
    if (time_source == "internal") {
        d_usrp->set_time_now(uhd::time_spec_t(0.0));
    } else {
        d_usrp->set_time_unknown_pps(uhd::time_spec_t(0.0));
    }

    uhd::stream_args_t stream_args("fc32", wire); // complex floats
    d_tx_streamer = d_usrp->get_tx_stream(stream_args);

// GPIO configuration
#if UHD_VERSION < 4000000
    if (in_gpio_pin != -1 || out_gpio_pin != -1)
        throw std::runtime_error("Block compiled without GPIO support");
#else
    if (in_gpio_pin != -1)
        d_in_gpio_bank = usrp_gpio_configure_input(d_usrp, { in_gpio_pin });

    if (out_gpio_pin != -1) {
        // Configure the GPIO output value manually when the GPIO has to be
        // asserted with a time advance relative to when the Tx starts and
        // deasserted with a delay relative to when the Tx ends. Otherwise, tie
        // the output value to the radio's Tx state via the ATR mechanism.
        //
        // When controlling the output value manually, make sure to start with
        // the value that indicates the USRP is not transmitting. This value
        // will be flipped when the first burst transmission starts.
        if (d_gpio_manual) {
            d_out_gpio_bank = usrp_gpio_configure_manual_output(
                d_usrp, { out_gpio_pin }, { !d_out_gpio_tx_val });
        } else {
            usrp_gpio_configure_atr_output(
                d_usrp, out_gpio_pin, false, true /* Tx only */, false, false);
        }
    }

    if (in_gpio_pin != -1 || out_gpio_pin != -1)
        usrp_gpio_dump_config(d_usrp);
#endif

    // Dump settings:
    std::cout << "Burst settings:" << std::endl;
    std::cout << "- Period: " << d_burst_period << " secs" << std::endl;
    std::cout << "- Duration: " << d_burst_duration << " secs" << std::endl;
    std::cout << "- Length: " << d_burst_len << " samples" << std::endl;

    // Schedule the first transmission
    d_next_tx = d_usrp->get_time_now() + d_burst_period;
    std::cout << "Burst " << d_n_burst << std::endl;
}

void usrp_burst_tx_c_impl::set_tx_gain(float gain) { d_usrp->set_tx_gain(gain); }

void usrp_burst_tx_c_impl::wait_gpio_in(bool expected_val,
                                        double timeout_ms,
                                        int sleep_ms)
{
    double elapsed_ms = 0;
    while (elapsed_ms < timeout_ms) {
        uint32_t reg_val = d_usrp->get_gpio_attr(d_in_gpio_bank, "READBACK");
        bool bit_val = (reg_val >> d_in_gpio_pin) & 0x01;
        if (bit_val == expected_val)
            break;
        std::this_thread::sleep_for(std::chrono::milliseconds(sleep_ms));
        elapsed_ms += sleep_ms;
    }
    if (elapsed_ms >= timeout_ms) {
        throw std::runtime_error(
            "Timed out while waiting for val=" + std::to_string(expected_val) +
            " on input pin " + std::to_string(d_in_gpio_pin));
    }
}

void usrp_burst_tx_c_impl::set_gpio_timed(const uhd::time_spec_t& time, bool val)
{
#if UHD_VERSION >= 4000000
    uint32_t mask = 1 << d_out_gpio_pin;
    d_usrp->set_command_time(time);
    d_usrp->set_gpio_attr(d_out_gpio_bank, "OUT", val << d_out_gpio_pin, mask);
#endif
}

/*
 * Our virtual destructor.
 */
usrp_burst_tx_c_impl::~usrp_burst_tx_c_impl() {}

int usrp_burst_tx_c_impl::work(int noutput_items,
                               gr_vector_const_void_star& input_items,
                               gr_vector_void_star& output_items)
{
    const gr_complex* in = (const gr_complex*)input_items[0];
    const int ninput_items = noutput_items; // sync block
    int n_consumed = 0;

    while (n_consumed < ninput_items) {
        if (d_gpio_manual_change_pending) {
            set_gpio_timed(d_next_tx - d_gpio_guard_period, d_out_gpio_tx_val);
            d_gpio_manual_change_pending = false;
        }

        while (d_n_sent < d_burst_len && n_consumed < ninput_items) {
            const size_t samps_available = ninput_items - n_consumed;
            const size_t samps_remaining = d_burst_len - d_n_sent;
            const size_t samps_to_send = std::min(samps_remaining, samps_available);
            const bool start_of_burst = d_n_sent == 0;

            // If required, wait for a high reading on the input GPIO pin before
            // initiating a burst
            if (start_of_burst && d_in_gpio_pin != -1)
                wait_gpio_in(d_in_gpio_wait_val);

            // Update the Tx metadata and the timeout value
            //
            // Note only the first packet of the burst needs a time_spec (see
            // the tx_bursts.cpp example in the UHD repository).
            d_tx_metadata.start_of_burst = start_of_burst;
            d_tx_metadata.end_of_burst = samps_to_send == samps_remaining;
            d_tx_metadata.has_time_spec = start_of_burst;
            if (start_of_burst)
                d_tx_metadata.time_spec = d_next_tx;

            // Timeout value when waiting for the blocking send call: delay
            // before Tx starts + padding
            const float timeout = d_burst_period + 0.1;

            // Send data to USRP
            size_t sent_samps = d_tx_streamer->send(
                in + n_consumed, samps_to_send, d_tx_metadata, timeout);
            d_n_sent += sent_samps;
            n_consumed += sent_samps;

            // Get timeout
            if (sent_samps < samps_to_send)
                std::cerr << "Tx timeout..." << std::endl;
        }

        // If the loop has stopped before completing the burst transmission, do
        // not prepare for the next burst just yet.
        if (d_n_sent < d_burst_len)
            continue;

        // Check the burst ACK
        uhd::async_metadata_t async_md;
        size_t acks = 0;
        while (acks == 0 and d_tx_streamer->recv_async_msg(async_md, 0.1)) {
            if (async_md.event_code == uhd::async_metadata_t::EVENT_CODE_BURST_ACK) {
                acks++;
            } else {
                std::cout << "WARNING: Got non-ack async event: code "
                          << async_md.event_code << std::endl;
            }
        }
        if (acks == 0)
            std::cout << "ERROR: burst ACK not received" << std::endl;

        // In manual mode, the output GPIO value should be reverted with a guard
        // period after the end of the burst transmission
        if (d_gpio_manual) {
            set_gpio_timed(d_next_tx + d_burst_duration + d_gpio_guard_period,
                           !d_out_gpio_tx_val);
            d_gpio_manual_change_pending = true; // prepare for the next burst
        }

        // If required, wait for a low reading on the input GPIO pin before
        // proceeding to the next burst
        if (d_in_gpio_pin != -1)
            wait_gpio_in(!d_in_gpio_wait_val);

        d_next_tx += d_burst_period;
        d_n_sent = 0;
        d_n_burst++;
        std::cout << "Burst " << d_n_burst << std::endl;
    }

    return n_consumed;
}

} /* namespace radar */
} /* namespace gr */

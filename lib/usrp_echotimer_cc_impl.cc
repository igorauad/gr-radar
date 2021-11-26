/* -*- c++ -*- */
/*
 * Copyright 2014 Communications Engineering Lab, KIT.
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

#include "usrp_echotimer_cc_impl.h"
#include <gnuradio/io_signature.h>
#include <uhd/version.hpp>
#include <iostream>

namespace gr {
namespace radar {

usrp_echotimer_cc::sptr usrp_echotimer_cc::make(int samp_rate,
                                                float center_freq,
                                                int num_delay_samps,
                                                std::string args_tx,
                                                std::string wire_tx,
                                                std::string clock_source_tx,
                                                std::string time_source_tx,
                                                std::string antenna_tx,
                                                float gain_tx,
                                                float timeout_tx,
                                                float wait_tx,
                                                float lo_offset_tx,
                                                std::string args_rx,
                                                std::string wire_rx,
                                                std::string clock_source_rx,
                                                std::string time_source_rx,
                                                std::string antenna_rx,
                                                float gain_rx,
                                                float timeout_rx,
                                                float wait_rx,
                                                float lo_offset_rx,
                                                const std::string& len_key,
                                                int gpio_tx_pin,
                                                int gpio_rx_pin)
{
    return gnuradio::get_initial_sptr(new usrp_echotimer_cc_impl(samp_rate,
                                                                 center_freq,
                                                                 num_delay_samps,
                                                                 args_tx,
                                                                 wire_tx,
                                                                 clock_source_tx,
                                                                 time_source_tx,
                                                                 antenna_tx,
                                                                 gain_tx,
                                                                 timeout_tx,
                                                                 wait_tx,
                                                                 lo_offset_tx,
                                                                 args_rx,
                                                                 wire_rx,
                                                                 clock_source_rx,
                                                                 time_source_rx,
                                                                 antenna_rx,
                                                                 gain_rx,
                                                                 timeout_rx,
                                                                 wait_rx,
                                                                 lo_offset_rx,
                                                                 len_key,
                                                                 gpio_tx_pin,
                                                                 gpio_rx_pin));
}

#if UHD_VERSION >= 4000000
void configure_gpio(uhd::usrp::multi_usrp::sptr usrp,
                    int pin,
                    bool high_on_tx,
                    size_t bank = 0,
                    size_t chan = 0,
                    size_t mboard = 0)
{
    auto src_banks = usrp->get_gpio_src_banks();
    if (bank >= src_banks.size()) {
        throw std::runtime_error("Source-controlled GPIO bank " + std::to_string(bank) +
                                 " is out of range");
    }

    auto gpio_src = usrp->get_gpio_src(src_banks[bank]);
    if (pin < 0 || pin > 31 || pin >= gpio_src.size()) {
        throw std::runtime_error("GPIO pin " + std::to_string(pin) + " is out of range");
    }

    // Configure pin to be controlled by UHD:
    gpio_src[pin] = "RF" + std::to_string(chan);
    usrp->set_gpio_src(src_banks[bank], gpio_src);

    // Configure the GPIO pin
    auto banks = usrp->get_gpio_banks(mboard);
    if (bank >= banks.size()) {
        throw std::runtime_error("GPIO bank " + std::to_string(bank) +
                                 " is out of range");
    }

    // Set it in automatic transmit-receive mode:
    uint32_t mask = 1 << pin;
    usrp->set_gpio_attr(banks[bank], "CTRL", mask, mask);
    // Configure it as an output pin
    usrp->set_gpio_attr(banks[bank], "DDR", mask, mask);
    // Set it high in either transmit-only or receive-only states.
    // Off in idle or full-duplex state.
    usrp->set_gpio_attr(banks[bank], "ATR_0X", 0x0, mask);
    usrp->set_gpio_attr(banks[bank], "ATR_XX", 0x0, mask);
    if (high_on_tx) {
        usrp->set_gpio_attr(banks[bank], "ATR_TX", mask, mask);
        usrp->set_gpio_attr(banks[bank], "ATR_RX", 0x0, mask);
    } else {
        usrp->set_gpio_attr(banks[bank], "ATR_TX", 0x0, mask);
        usrp->set_gpio_attr(banks[bank], "ATR_RX", mask, mask);
    }
}
#else
#warning "Compiling echotimer without GPIO support"
#endif

/*
 * The private constructor
 */
usrp_echotimer_cc_impl::usrp_echotimer_cc_impl(int samp_rate,
                                               float center_freq,
                                               int num_delay_samps,
                                               std::string args_tx,
                                               std::string wire_tx,
                                               std::string clock_source_tx,
                                               std::string time_source_tx,
                                               std::string antenna_tx,
                                               float gain_tx,
                                               float timeout_tx,
                                               float wait_tx,
                                               float lo_offset_tx,
                                               std::string args_rx,
                                               std::string wire_rx,
                                               std::string clock_source_rx,
                                               std::string time_source_rx,
                                               std::string antenna_rx,
                                               float gain_rx,
                                               float timeout_rx,
                                               float wait_rx,
                                               float lo_offset_rx,
                                               const std::string& len_key,
                                               int gpio_tx_pin,
                                               int gpio_rx_pin)
    : gr::tagged_stream_block("usrp_echotimer_cc",
                              gr::io_signature::make(1, 1, sizeof(gr_complex)),
                              gr::io_signature::make(1, 1, sizeof(gr_complex)),
                              len_key),
      d_samp_rate(samp_rate),
      d_samp_period(1.0 / (double)samp_rate),
      d_center_freq(center_freq),
      d_num_delay_samps(num_delay_samps),
      d_timeout_tx(timeout_tx),
      d_timeout_rx(timeout_rx),
      d_wait_tx(wait_tx),
      d_wait_rx(wait_rx),
      d_n_ready_send(0),
      d_n_ready_recv(0),
      d_stop_send(false),
      d_stop_recv(false)
{
    //***** Setup USRP TX *****//

    // Setup USRP TX: args (addr,...)
    d_usrp_tx = uhd::usrp::multi_usrp::make(args_tx);
    std::cout << "Using USRP Device (TX): " << std::endl
              << d_usrp_tx->get_pp_string() << std::endl;

    // Setup USRP TX: sample rate
    std::cout << "Setting TX Rate: " << d_samp_rate << std::endl;
    d_usrp_tx->set_tx_rate(d_samp_rate);
    std::cout << "Actual TX Rate: " << d_usrp_tx->get_tx_rate() << std::endl;

    uhd::tune_request_t tune_request_tx(
        d_center_freq, lo_offset_tx); // FIXME: add alternative tune requests

    set_tx_gain(gain_tx);
    d_usrp_tx->set_tx_freq(tune_request_tx);
    d_usrp_tx->set_tx_antenna(antenna_tx);
    d_usrp_tx->set_clock_source(clock_source_tx); // Set TX clock, TX is master
    d_usrp_tx->set_time_source(time_source_tx);   // Set TX time, TX is master

    if (time_source_tx != "gpsdo") {
        d_usrp_tx->set_time_now(
            uhd::time_spec_t(0.0)); // Do set time on startup if not gpsdo is activated.
    }

    // Setup transmit streamer
    uhd::stream_args_t stream_args_tx("fc32", wire_tx); // complex floats
    d_tx_stream = d_usrp_tx->get_tx_stream(stream_args_tx);

    //***** Setup USRP RX *****//

    // Setup USRP RX: args (addr,...)
    d_usrp_rx = uhd::usrp::multi_usrp::make(args_rx);
    std::cout << "Using USRP Device (RX): " << std::endl
              << d_usrp_rx->get_pp_string() << std::endl;

    // Setup USRP RX: sample rate
    std::cout << "Setting RX Rate: " << d_samp_rate << std::endl;
    d_usrp_rx->set_rx_rate(d_samp_rate);
    std::cout << "Actual RX Rate: " << d_usrp_rx->get_rx_rate() << std::endl;

    uhd::tune_request_t tune_request_rx(
        d_center_freq, lo_offset_rx); // FIXME: add alternative tune requests

    set_rx_gain(gain_rx);
    d_usrp_rx->set_rx_freq(tune_request_rx);
    d_usrp_rx->set_rx_antenna(antenna_rx);
    d_usrp_rx->set_clock_source(clock_source_rx); // RX is slave, clock is set on TX
    d_usrp_rx->set_time_source(time_source_rx);

    // Setup receive streamer
    uhd::stream_args_t stream_args_rx("fc32", wire_rx); // complex floats
    std::vector<size_t> channel_nums;
    channel_nums.push_back(0); // define channel!
    stream_args_rx.channels = channel_nums;
    d_rx_stream = d_usrp_rx->get_rx_stream(stream_args_rx);

    //***** Misc *****//

    // Pins indicating Tx and Rx state
#if UHD_VERSION < 4000000
    if (gpio_tx_pin != -1 || gpio_rx_pin != -1)
        throw std::runtime_error("Block compiled without GPIO support");
#else
    if (gpio_tx_pin != -1)
        configure_gpio(d_usrp_tx, gpio_tx_pin, true /* Tx */);

    if (gpio_rx_pin != -1)
        configure_gpio(d_usrp_rx, gpio_rx_pin, false /* Rx */);
#endif

    // Setup rx_time pmt
    d_time_key = pmt::string_to_symbol("rx_time");
    d_srcid = pmt::string_to_symbol("usrp_echotimer");

    // Setup thread priority
    // uhd::set_thread_priority_safe(); // necessary? doesnt work...

    // Sleep to get sync done
    boost::this_thread::sleep(boost::posix_time::milliseconds(1000)); // FIXME: necessary?

    // Kick off the send/receive threads
    d_thread_send =
        gr::thread::thread(boost::bind(&usrp_echotimer_cc_impl::send_loop, this));
    d_thread_recv =
        gr::thread::thread(boost::bind(&usrp_echotimer_cc_impl::receive_loop, this));
}

/*
 * Our virtual destructor.
 */
usrp_echotimer_cc_impl::~usrp_echotimer_cc_impl()
{
    {
        gr::thread::scoped_lock lock(d_mutex_send);
        d_stop_send = true;
    }
    d_thread_send.join();

    {
        gr::thread::scoped_lock lock(d_mutex_recv);
        d_stop_recv = true;
    }
    d_thread_recv.join();
}

int usrp_echotimer_cc_impl::calculate_output_stream_length(
    const gr_vector_int& ninput_items)
{
    int noutput_items = ninput_items[0];
    return noutput_items;
}

void usrp_echotimer_cc_impl::set_num_delay_samps(int num_samps)
{
    d_num_delay_samps = num_samps;
}

void usrp_echotimer_cc_impl::set_rx_gain(float gain) { d_usrp_rx->set_rx_gain(gain); }

void usrp_echotimer_cc_impl::set_tx_gain(float gain) { d_usrp_tx->set_tx_gain(gain); }

void usrp_echotimer_cc_impl::send()
{
    // Setup metadata for first package
    d_metadata_tx.start_of_burst = true;
    d_metadata_tx.end_of_burst = false;
    d_metadata_tx.has_time_spec = true;
    d_metadata_tx.time_spec =
        d_time_now_tx + uhd::time_spec_t(d_wait_tx); // Timespec needed?

    // Send input buffer
    size_t num_acc_samps = 0; // Number of accumulated samples
    double expected_duration = d_n_ready_send * d_samp_period;
    // Data to USRP
    size_t num_tx_samps = d_tx_stream->send(
        d_in_send, d_n_ready_send, d_metadata_tx, expected_duration + d_timeout_tx);
    // Get timeout
    if (num_tx_samps < d_n_ready_send)
        std::cerr << "Send timeout..." << std::endl;

    // send a mini EOB packet
    d_metadata_tx.start_of_burst = false;
    d_metadata_tx.end_of_burst = true;
    d_metadata_tx.has_time_spec = false;
    d_tx_stream->send("", 0, d_metadata_tx);

    d_n_ready_send = 0; // flag as done
}

void usrp_echotimer_cc_impl::receive()
{
    // Setup RX streaming
    uhd::stream_cmd_t stream_cmd(uhd::stream_cmd_t::STREAM_MODE_NUM_SAMPS_AND_DONE);
    stream_cmd.num_samps = d_n_ready_recv;
    stream_cmd.stream_now = false;
    stream_cmd.time_spec = d_time_now_rx + uhd::time_spec_t(d_wait_rx);
    d_rx_stream->issue_stream_cmd(stream_cmd);

    // Receive a packet
    double expected_duration = d_n_ready_recv * d_samp_period;
    size_t num_rx_samps = d_rx_stream->recv(
        d_out_recv, d_n_ready_recv, d_metadata_rx, expected_duration + d_timeout_rx);

    // Save timestamp
    d_time_val =
        pmt::make_tuple(pmt::from_uint64(d_metadata_rx.time_spec.get_full_secs()),
                        pmt::from_double(d_metadata_rx.time_spec.get_frac_secs()));

    // Handle the error code
    if (d_metadata_rx.error_code != uhd::rx_metadata_t::ERROR_CODE_NONE) {
        throw std::runtime_error(
            str(boost::format("Receiver error %s") % d_metadata_rx.strerror()));
    }

    if (num_rx_samps < d_n_ready_recv)
        std::cerr << "Receive timeout before all samples received..." << std::endl;

    d_n_ready_recv = 0; // flag as done
}

void usrp_echotimer_cc_impl::send_loop()
{
    while (true) {
        gr::thread::scoped_lock lock(d_mutex_send);
        d_cv_send.wait(lock, [this]() { return d_n_ready_send > 0 || d_stop_send; });
        if (d_stop_send)
            break;
        send();
    }
}

void usrp_echotimer_cc_impl::receive_loop()
{
    while (true) {
        gr::thread::scoped_lock lock(d_mutex_recv);
        d_cv_recv.wait(lock, [this]() { return d_n_ready_recv > 0 || d_stop_recv; });
        if (d_stop_recv)
            break;
        receive();
    }
}

int usrp_echotimer_cc_impl::work(int noutput_items,
                                 gr_vector_int& ninput_items,
                                 gr_vector_const_void_star& input_items,
                                 gr_vector_void_star& output_items)
{
    gr_complex* in = (gr_complex*)input_items[0]; // remove const
    gr_complex* out = (gr_complex*)output_items[0];

    // Set output items on packet length
    noutput_items = ninput_items[0];

    // Resize output buffer
    if (d_out_buffer.size() != noutput_items)
        d_out_buffer.resize(noutput_items);

    // Get time from USRP TX
    d_time_now_tx = d_usrp_tx->get_time_now();
    d_time_now_rx = d_time_now_tx;

    // Send thread
    {
        gr::thread::scoped_lock lock(d_mutex_send);
        d_in_send = in;
        d_n_ready_send = noutput_items;
    }
    d_cv_send.notify_one();

    // Receive thread
    {
        gr::thread::scoped_lock lock(d_mutex_recv);
        d_out_recv = &d_out_buffer[0];
        d_n_ready_recv = noutput_items;
    }
    d_cv_recv.notify_one();

    // Wait for the thread iterations to complete
    while (true) {
        gr::thread::scoped_lock lock(d_mutex_send);
        if (d_n_ready_send == 0)
            break;
    }
    while (true) {
        gr::thread::scoped_lock lock(d_mutex_recv);
        if (d_n_ready_recv == 0)
            break;
    }

    // Shift of number delay samples (fill with zeros)
    memcpy(out,
           &d_out_buffer[0] + d_num_delay_samps,
           (noutput_items - d_num_delay_samps) *
               sizeof(gr_complex)); // push buffer to output
    memset(out + (noutput_items - d_num_delay_samps),
           0,
           d_num_delay_samps * sizeof(gr_complex)); // set zeros

    // Setup rx_time tag
    add_item_tag(0, nitems_written(0), d_time_key, d_time_val, d_srcid);

    // Tell runtime system how many output items we produced.
    return noutput_items;
}

} /* namespace radar */
} /* namespace gr */

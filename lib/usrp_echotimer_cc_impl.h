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

#ifndef INCLUDED_RADAR_USRP_ECHOTIMER_CC_IMPL_H
#define INCLUDED_RADAR_USRP_ECHOTIMER_CC_IMPL_H

#include <radar/usrp_echotimer_cc.h>
#include <uhd/usrp/multi_usrp.hpp>
#include <uhd/version.hpp>
// clang-format off
#if UHD_VERSION >= 3110000
#include <uhd/utils/thread.hpp>
#else
#include <uhd/utils/thread_priority.hpp>
#endif
// clang-format on

namespace gr {
namespace radar {

using worker_callback = boost::function<void(gr_complex*, size_t)>;
class echotimer_worker
{
private:
    bool d_stop = false;
    size_t d_pending_samples = 0;
    gr_complex* d_buffer;
    gr::thread::thread d_thread;
    gr::thread::mutex d_mutex;
    gr::thread::condition_variable d_cv;
    worker_callback d_callback;

public:
    echotimer_worker(worker_callback callback) : d_callback(callback){};
    void loop();
    void provide(gr_complex* buf, size_t n_samples);
    void wait();
    void start();
    void stop();
};

class usrp_echotimer_cc_impl : public usrp_echotimer_cc
{
private:
    int d_samp_rate;
    double d_samp_period;
    float d_center_freq;
    int d_num_delay_samps;
    float d_timeout_tx, d_timeout_rx;      // timeout for sending/receiving
    float d_wait_tx, d_wait_rx;            // secs to wait before sending/receiving
    size_t d_n_ready_send, d_n_ready_recv; // samples ready to send/receive
    bool d_stop_send, d_stop_recv;         // stop sending/receiving

    std::vector<gr_complex> d_out_buffer;

    uhd::usrp::multi_usrp::sptr d_usrp_tx, d_usrp_rx;
    uhd::tx_streamer::sptr d_tx_stream;
    uhd::rx_streamer::sptr d_rx_stream;
    uhd::tx_metadata_t d_metadata_tx;
    uhd::rx_metadata_t d_metadata_rx;
    uhd::time_spec_t d_time_now_tx, d_time_now_rx;
    pmt::pmt_t d_time_key, d_time_val, d_srcid;

    echotimer_worker d_worker_send, d_worker_recv;

    void send(gr_complex* send_buf, size_t num_samps);
    void receive(gr_complex* recv_buf, size_t num_samps);

protected:
    int calculate_output_stream_length(const gr_vector_int& ninput_items);

public:
    usrp_echotimer_cc_impl(int samp_rate,
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
                           int gpio_rx_pin);

    ~usrp_echotimer_cc_impl();

    int work(int noutput_items,
             gr_vector_int& ninput_items,
             gr_vector_const_void_star& input_items,
             gr_vector_void_star& output_items);

    void set_num_delay_samps(int num_samps);
    void set_rx_gain(float gain);
    void set_tx_gain(float gain);
};

} // namespace radar
} // namespace gr

#endif /* INCLUDED_RADAR_USRP_ECHOTIMER_CC_IMPL_H */

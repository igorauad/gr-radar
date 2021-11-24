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

#include "signal_generator_sync_pulse_c_impl.h"
#include <gnuradio/io_signature.h>

namespace gr {
namespace radar {

signal_generator_sync_pulse_c::sptr
signal_generator_sync_pulse_c::make(int packet_len,
                                    std::vector<int> pulse_len,
                                    std::vector<int> pulse_pause,
                                    float pulse_amplitude,
                                    const std::string len_key)
{
    return gnuradio::get_initial_sptr(new signal_generator_sync_pulse_c_impl(
        packet_len, pulse_len, pulse_pause, pulse_amplitude, len_key));
}

/*
 * The private constructor
 */
signal_generator_sync_pulse_c_impl::signal_generator_sync_pulse_c_impl(
    int packet_len,
    std::vector<int> pulse_len,
    std::vector<int> pulse_pause,
    float pulse_amplitude,
    const std::string len_key)
    : gr::sync_block("signal_generator_sync_pulse_c",
                     gr::io_signature::make(0, 0, 0),
                     gr::io_signature::make(1, 1, sizeof(gr_complex))),
    d_packet_len(packet_len),
    d_counter(packet_len),
    d_key(pmt::string_to_symbol(len_key)), // set tag identifier for tagged stream
    d_value(pmt::from_long(packet_len)),   // set length of 1 cw packet as tagged stream
    d_srcid(pmt::string_to_symbol("sig_gen_sync")) // set block identifier
{
    // Setup output buffer
    // Use alternating pulse_pause (zeros) and pulse_len (amplitude), fill with zeros at
    // the end up to packet_len
    d_out_buffer.resize(packet_len);
    int i_len = 0;
    int i_pause = 0;
    int k = 0;
    while (i_pause < pulse_pause.size() || i_len < pulse_len.size()) {
        if (i_pause < pulse_pause.size()) { // Setup wait samples
            for (int p = 0; p < pulse_pause[i_pause]; p++)
                d_out_buffer[k + p] = gr_complex(0, 0);
            k += pulse_pause[i_pause];
            i_pause++;
        }
        if (i_len < pulse_len.size()) { // Setup pulse samples
            for (int p = 0; p < pulse_len[i_len]; p++)
                d_out_buffer[k + p] = gr_complex(pulse_amplitude, 0);
            k += pulse_len[i_len];
            i_len++;
        }
    }
    if (k < packet_len) { // fill with zeros
        for (int p = 0; p < packet_len - k; p++) {
            d_out_buffer[k + p] = gr_complex(0, 0);
        }
    }
}

/*
 * Our virtual destructor.
 */
signal_generator_sync_pulse_c_impl::~signal_generator_sync_pulse_c_impl() {}

int signal_generator_sync_pulse_c_impl::work(int noutput_items,
                                             gr_vector_const_void_star& input_items,
                                             gr_vector_void_star& output_items)
{
    gr_complex* out = (gr_complex*)output_items[0];

    // Push output buffer to out
    for (int i = 0; i < noutput_items; i++) {
        if (d_counter == d_packet_len) {
            add_item_tag(0, nitems_written(0) + i, d_key, d_value, d_srcid);
            d_counter = 0;
        }

        // Write sample
        out[i] = d_out_buffer[d_counter];
        d_counter++;
    }

    // Tell runtime system how many output items we produced.
    return noutput_items;
}

} /* namespace radar */
} /* namespace gr */

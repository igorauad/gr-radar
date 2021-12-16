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

#ifndef INCLUDED_RADAR_USRP_BURST_SOURCE_C_H
#define INCLUDED_RADAR_USRP_BURST_SOURCE_C_H

#include <gnuradio/sync_block.h>
#include <radar/api.h>

namespace gr {
namespace radar {

/*!
 * \brief <+description of block+>
 * \ingroup radar
 *
 */
class RADAR_API usrp_burst_source_c : virtual public gr::sync_block
{
public:
    typedef boost::shared_ptr<usrp_burst_source_c> sptr;

    /*!
     * \brief Return a shared_ptr to a new instance of radar::usrp_burst_source_c.
     *
     * To avoid accidental use of raw pointers, radar::usrp_burst_source_c's
     * constructor is in a private implementation
     * class. radar::usrp_burst_source_c::make is the public interface for
     * creating new instances.
     */
    static sptr make(float samp_rate,
                     float center_freq,
                     float duty_cycle,
                     float period,
                     float gain,
                     std::string args,
                     std::string wire,
                     std::string clock_source,
                     std::string time_source,
                     std::string antenna,
                     int gpio_pin = -1);

    virtual void set_tx_gain(float gain) = 0;
};

} // namespace radar
} // namespace gr

#endif /* INCLUDED_RADAR_USRP_BURST_SOURCE_C_H */

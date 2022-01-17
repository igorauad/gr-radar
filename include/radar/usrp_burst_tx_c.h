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

#ifndef INCLUDED_RADAR_USRP_BURST_TX_C_H
#define INCLUDED_RADAR_USRP_BURST_TX_C_H

#include <gnuradio/sync_block.h>
#include <radar/api.h>

namespace gr {
namespace radar {

/*!
 * \brief USRP Burst Transmitter
 *
 * Implements periodic burst transmissions via a USRP device with a specified burst
 * duration (duty cycle) in each period. Optionally asserts a GPIO when transmitting and
 * optionally introduces a guard period before and after the GPIO assertion.
 *
 * \ingroup radar
 *
 */
class RADAR_API usrp_burst_tx_c : virtual public gr::sync_block
{
public:
    typedef boost::shared_ptr<usrp_burst_tx_c> sptr;

    /**
     * @brief Make USRP Burst Tx object.
     *
     * @param samp_rate Sample rate.
     * @param center_freq Center frequency.
     * @param duty_cycle Burst duty cycle.
     * @param period Burst transmission period in seconds.
     * @param gain USRP Tx gain in dB.
     * @param args USRP device address args.
     * @param wire Wire format
     * @param clock_source USRP clock source.
     * @param time_source USRP time source.
     * @param antenna USRP Tx antenna.
     * @param in_gpio_pin Input GPIO pin to poll before and after each burst transmission.
     * @param out_gpio_pin Output GPIO pin to control when transmitting.
     * @param gpio_guard_period GPIO assertion/deassertion guard period in seconds. The
     * GPIO will be changed this many seconds in advance of the actual transmission start
     * and reverted back to the original value with this many seconds delay after the
     * burst transmission ends. If the input GPIO pin is polled before/after each burst
     * transmission, the GPIO value wait will timeout as soon as the guard period expires.
     * @param in_gpio_wait_val Value to wait on the input GPIO pin before each burst
     * transmission if parameter in_gpio_pin is defined. By default, the input value
     * should be high for Tx to start and low after Tx ends.
     * @param out_gpio_tx_val Output GPIO pin value to set when transmitting if parameter
     * out_gpio_pin is defined. Only supported in manual mode (when gpio_guard_period is
     * non-zero). By default, the GPIO is set to high when transmitting. In automatic mode
     * (when gpio_guard_period is zero), the GPIO is always set to high when transmitting.
     * @return sptr Shared pointer to the created usrp_burst_tx_c object.
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
                     int in_gpio_pin = -1,
                     int out_gpio_pin = -1,
                     float gpio_guard_period = 0.0,
                     bool in_gpio_wait_val = true,
                     bool out_gpio_tx_val = true);

    virtual void set_tx_gain(float gain) = 0;
};

} // namespace radar
} // namespace gr

#endif /* INCLUDED_RADAR_USRP_BURST_TX_C_H */

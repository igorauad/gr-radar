/* -*- c++ -*- */
/*
 * Copyright (c) 2021 Igor Freire (Blockstream Corp)
 *
 * This file is part of gr-radar.
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */
#include "usrp_gpio.h"
#include <iostream>

namespace gr {
namespace radar {

#if UHD_VERSION >= 4000000
void usrp_configure_gpio(uhd::usrp::multi_usrp::sptr usrp,
                         int pin,
                         bool high_on_tx,
                         bool manual,
                         bool manual_val,
                         size_t bank,
                         size_t chan,
                         size_t mboard)
{
    auto src_banks = usrp->get_gpio_src_banks();
    if (bank >= src_banks.size()) {
        throw std::runtime_error("Source-controlled GPIO bank " + std::to_string(bank) +
                                 " is out of range");
    }

    std::cout << "Configuring GPIO pin " << pin << std::endl;
    std::cout << "- Source bank " << src_banks[bank] << std::endl;

    auto gpio_src = usrp->get_gpio_src(src_banks[bank]);
    if (pin < 0 || pin >= gpio_src.size()) {
        throw std::runtime_error("GPIO pin " + std::to_string(pin) +
                                 " is out of range (" + std::to_string(gpio_src.size()) +
                                 " pins available)");
    }

    // Configure pin to be controlled by UHD:
    gpio_src[pin] = "RF" + std::to_string(chan);
    std::cout << "- Set control to " << gpio_src[pin] << std::endl;
    usrp->set_gpio_src(src_banks[bank], gpio_src);

    // Configure the GPIO pin
    auto banks = usrp->get_gpio_banks(mboard);
    if (bank >= banks.size()) {
        throw std::runtime_error("GPIO bank " + std::to_string(bank) +
                                 " is out of range");
    }

    std::cout << "- GPIO bank " << banks[bank] << std::endl;

    // Set it in automatic transmit-receive mode:
    uint32_t mask = 1 << pin;
    uint32_t atr_control = !manual << pin;
    usrp->set_gpio_attr(banks[bank], "CTRL", atr_control, mask);
    // Configure it as an output pin
    usrp->set_gpio_attr(banks[bank], "DDR", mask, mask);
    // Output value
    if (manual) {
        uint32_t out_val = manual_val << pin;
        std::cout << "- Manual output value: " << manual_val << " (mask 0x" << out_val
                  << ")" << std::endl;
        usrp->set_gpio_attr(banks[bank], "OUT", out_val, mask);
    } else {
        // Set it high in either transmit-only or receive-only states.
        // Off in idle or full-duplex state.
        usrp->set_gpio_attr(banks[bank], "ATR_0X", 0x0, mask);
        usrp->set_gpio_attr(banks[bank], "ATR_XX", 0x0, mask);
        if (high_on_tx) {
            std::cout << "- High on Tx" << std::endl;
            usrp->set_gpio_attr(banks[bank], "ATR_TX", mask, mask);
            usrp->set_gpio_attr(banks[bank], "ATR_RX", 0x0, mask);
        } else {
            std::cout << "- High on Rx" << std::endl;
            usrp->set_gpio_attr(banks[bank], "ATR_TX", 0x0, mask);
            usrp->set_gpio_attr(banks[bank], "ATR_RX", mask, mask);
        }
    }
}

void usrp_dump_gpio_config(uhd::usrp::multi_usrp::sptr usrp,
                           size_t bank,
                           size_t chan,
                           size_t mboard)
{
    auto src_banks = usrp->get_gpio_src_banks();
    std::cout << "Available GPIO source banks:" << std::endl;
    for (const auto& bank : src_banks) {
        std::cout << "- " << bank << std::endl;
    }

    std::cout << "Current GPIO sources:" << std::endl;
    auto gpio_src = usrp->get_gpio_src(src_banks[bank]);
    size_t pin = 0;
    for (const auto& src : gpio_src) {
        std::cout << "- Pin " << pin << ": " << src << std::endl;
        pin++;
    }

    auto banks = usrp->get_gpio_banks(mboard);
    std::cout << "Available GPIO banks:" << std::endl;
    for (const auto& bank : banks) {
        std::cout << "- " << bank << std::endl;
    }

    std::cout << "Bank " << bank << "'s attributes:" << std::endl;
    std::cout << "- CTRL: 0x" << usrp->get_gpio_attr(banks[bank], "CTRL") << std::endl;
    std::cout << "- DDR: 0x" << usrp->get_gpio_attr(banks[bank], "DDR") << std::endl;
    std::cout << "- OUT: 0x" << usrp->get_gpio_attr(banks[bank], "OUT") << std::endl;
    std::cout << "- ATR_0X: 0x" << usrp->get_gpio_attr(banks[bank], "ATR_0X")
              << std::endl;
    std::cout << "- ATR_XX: 0x" << usrp->get_gpio_attr(banks[bank], "ATR_XX")
              << std::endl;
    std::cout << "- ATR_TX: 0x" << usrp->get_gpio_attr(banks[bank], "ATR_TX")
              << std::endl;
    std::cout << "- ATR_RX: 0x" << usrp->get_gpio_attr(banks[bank], "ATR_RX")
              << std::endl;
}
#else
#warning "Compiling echotimer without GPIO support"
#endif

} // namespace radar
} // namespace gr
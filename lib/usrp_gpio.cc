/* -*- c++ -*- */
/*
 * Copyright (c) 2021 Igor Freire (Blockstream Corp)
 *
 * This file is part of gr-radar.
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */
#include <radar/usrp_gpio.h>
#include <iostream>

namespace gr {
namespace radar {

#if UHD_VERSION >= 4000000

std::string get_and_configure_gpio_bank(uhd::usrp::multi_usrp::sptr usrp,
                                        const std::vector<int>& pins,
                                        size_t bank,
                                        size_t chan,
                                        size_t mboard)
{
    // Source-controlled GPIO banks
    auto src_banks = usrp->get_gpio_src_banks();
    if (bank >= src_banks.size()) {
        throw std::runtime_error("Source-controlled GPIO bank " + std::to_string(bank) +
                                 " is out of range");
    }

    // Get current GPIO sources
    auto gpio_src = usrp->get_gpio_src(src_banks[bank]);

    // Update GPIO sources assigned for each of the target pins. Set them to UHD.
    for (const int& pin : pins) {
        if (pin < 0 || pin >= gpio_src.size()) {
            throw std::runtime_error(
                "GPIO pin " + std::to_string(pin) + " is out of range (" +
                std::to_string(gpio_src.size()) + " pins available)");
        }
        gpio_src[pin] = "RF" + std::to_string(chan);
    }
    usrp->set_gpio_src(src_banks[bank], gpio_src);

    // Get and return GPIO bank
    auto banks = usrp->get_gpio_banks(mboard);
    if (bank >= banks.size()) {
        throw std::runtime_error("GPIO bank " + std::to_string(bank) +
                                 " is out of range");
    }
    return banks[bank];
}

void usrp_gpio_configure_atr_output(uhd::usrp::multi_usrp::sptr usrp,
                                    int pin,
                                    bool high_idle,
                                    bool high_tx_only,
                                    bool high_rx_only,
                                    bool high_full_duplex,
                                    size_t bank,
                                    size_t chan,
                                    size_t mboard)
{
    std::string bank_name =
        get_and_configure_gpio_bank(usrp, { pin }, bank, chan, mboard);

    // Set it in automatic transmit-receive mode:
    uint32_t mask = 1 << pin;
    usrp->set_gpio_attr(bank_name, "CTRL", mask, mask);
    // Configure it as an output pin
    usrp->set_gpio_attr(bank_name, "DDR", mask, mask);
    // Configure the ATR output
    usrp->set_gpio_attr(bank_name, "ATR_0X", high_idle ? mask : 0x0, mask);
    usrp->set_gpio_attr(bank_name, "ATR_TX", high_tx_only ? mask : 0x0, mask);
    usrp->set_gpio_attr(bank_name, "ATR_RX", high_rx_only ? mask : 0x0, mask);
    usrp->set_gpio_attr(bank_name, "ATR_XX", high_full_duplex ? mask : 0x0, mask);
}

void usrp_gpio_configure_manual_output(uhd::usrp::multi_usrp::sptr usrp,
                                       const std::vector<int>& pins,
                                       const std::vector<bool>& vals,
                                       size_t bank,
                                       size_t chan,
                                       size_t mboard)
{
    if (pins.size() != vals.size())
        throw std::runtime_error("Pins and values vectors must have the same length");

    std::string bank_name = get_and_configure_gpio_bank(usrp, pins, bank, chan, mboard);

    uint32_t mask = 0;
    uint32_t out_val = 0;
    for (size_t i = 0; i < pins.size(); i++) {
        mask |= 1 << pins[i];
        out_val |= vals[i] << pins[i];
    }

    // Set pin(s) to manual mode and output direction:
    usrp->set_gpio_attr(bank_name, "CTRL", 0x0, mask);
    usrp->set_gpio_attr(bank_name, "DDR", mask, mask);
    // Define their output values
    usrp->set_gpio_attr(bank_name, "OUT", out_val, mask);
}

void usrp_gpio_dump_config(uhd::usrp::multi_usrp::sptr usrp,
                           size_t bank,
                           size_t chan,
                           size_t mboard)
{
    auto src_banks = usrp->get_gpio_src_banks();
    std::cout << "Source-controllable GPIO banks:" << std::endl;
    for (const auto& bank : src_banks) {
        std::cout << "- " << bank << std::endl;
    }

    std::cout << "Current GPIO sources:" << std::endl;
    auto gpio_src = usrp->get_gpio_src(src_banks[bank]);

    std::cout << "- Pin:     ";
    for (int pin = 0; pin < gpio_src.size(); pin++)
        std::cout << boost::format("|  %2d ") % pin;
    std::cout << "|" << std::endl;

    std::cout << "- Source:  ";
    for (const auto& src : gpio_src)
        std::cout << boost::format("| %3s ") % src;
    std::cout << "|" << std::endl;

    auto banks = usrp->get_gpio_banks(mboard);
    std::cout << "GPIO banks:" << std::endl;
    for (const auto& bank : banks) {
        std::cout << "- " << bank << std::endl;
    }

    std::cout << "Bank " << bank << "'s attributes:" << std::endl;
    std::vector<std::string> attributes(
        { "CTRL", "DDR", "OUT", "ATR_0X", "ATR_TX", "ATR_RX", "ATR_XX" });

    for (const auto& attr : attributes)
        std::cout << boost::format("| %6s ") % attr;
    std::cout << "|" << std::endl;

    for (const auto& attr : attributes)
        std::cout << boost::format("|   0x%02x ") %
                         usrp->get_gpio_attr(banks[bank], attr);
    std::cout << "|" << std::endl;
}
#else
#warning "Compiling echotimer without GPIO support"
#endif

} // namespace radar
} // namespace gr
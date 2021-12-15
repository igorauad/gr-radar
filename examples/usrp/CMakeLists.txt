#
# Copyright (c) 2021 Igor Freire (Blockstream Corp)
#
# This file is part of gr-radar.
#
# SPDX-License-Identifier: GPL-3.0-or-later
#
########################################################################
# Build executable
########################################################################
add_executable(usrp_gpio_ctrl usrp_gpio_ctrl.cc)
target_link_libraries(usrp_gpio_ctrl PUBLIC gnuradio-radar)

INSTALL(TARGETS
    usrp_gpio_ctrl
    DESTINATION bin)

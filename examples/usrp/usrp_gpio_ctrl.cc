#include <radar/usrp_gpio.h>
#include <boost/program_options.hpp>
#include <iostream>

namespace po = boost::program_options;

int _main(int argc, const char* argv[])
{
    std::string args;
    std::string subdev_spec;
    std::vector<int> pins;
    std::vector<bool> vals;
    po::options_description desc{ "Options" };
    // clang-format off
    desc.add_options()
        ("help,h", "help message")
        ("args,a", po::value<std::string>(&args)->default_value(""),
         "Multi UHD device address args")
        ("subdev,s", po::value<std::string>(&subdev_spec)->default_value(""),
         "Subdevice Specification")
        ("pin,p", po::value<std::vector<int>>(&pins), "GPIO Pins")
        ("val,v", po::value<std::vector<bool>>(&vals), "GPIO High/Low Value")
    ;
    // clang-format on
    po::variables_map vm;
    po::store(po::parse_command_line(argc, argv, desc), vm);
    po::notify(vm);

    if (vm.count("help")) {
        std::cout << desc << '\n';
        return 0;
    }

    if (pins.size() != vals.size()) {
        throw std::runtime_error("Pin and value lists must have the same size");
    }

    auto p_usrp = uhd::usrp::multi_usrp::make(args);
    std::cout << "USRP Device: " << std::endl << p_usrp->get_pp_string() << std::endl;
    gr::radar::usrp_gpio_configure_manual_output(p_usrp, pins, vals);
    gr::radar::usrp_gpio_dump_config(p_usrp);

    return 0;
}

int main(int argc, const char* argv[])
{
    try {
        return _main(argc, argv);
    } catch (const std::exception& ex) {
        std::cerr << ex.what() << '\n';
    }
    return ~0;
}
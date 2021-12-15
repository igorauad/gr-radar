FROM igorfreire/gnuradio-oot-dev:3.8.2-ubuntu-bionic

# Dependencies
RUN apt update && apt install -y \
    gdb \
    gir1.2-gtk-3.0 \
    libqwt-qt5-dev \
    qtbase5-dev
RUN add-apt-repository -y ppa:ettusresearch/uhd && \
    apt update && \
    apt install -y libuhd-dev libuhd4.1.0 uhd-host

# gr-radar
RUN mkdir -p /src/ && cd /src/ && \
	git clone https://github.com/kit-cel/gr-radar.git && \
	cd gr-radar && mkdir build && cd build && \
	cmake -DCMAKE_BUILD_TYPE=Release -DPYTHON_EXECUTABLE=/usr/bin/python3 ../ \
	&& make -j$(nproc) && make install && ldconfig

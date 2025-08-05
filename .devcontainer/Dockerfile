FROM archlinux:base-devel

RUN pacman -Syu --noconfirm \
        bash \
        git \
        wget \
        python \
        python-pip \
        arm-none-eabi-binutils \
        arm-none-eabi-gcc \
        arm-none-eabi-gdb \
        arm-none-eabi-newlib

RUN wget https://github.com/Kitware/CMake/releases/download/v3.31.8/cmake-3.31.8-linux-x86_64.sh -O cmake.sh && \
    sudo sh cmake.sh --skip-license --prefix=/usr/local && \
    rm cmake.sh

ARG SDK_PATH=/usr/share/pico-sdk
RUN git clone --depth 1 --branch 1.5.1 https://github.com/raspberrypi/pico-sdk $SDK_PATH && \
    cd $SDK_PATH && \
    git submodule update --init --depth 1

ENV PICO_SDK_PATH=$SDK_PATH

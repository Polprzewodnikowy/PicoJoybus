FROM archlinux:base-devel

RUN pacman -Syu --noconfirm \
        bash \
        git \
        python \
        python-pip \
        cmake \
        arm-none-eabi-binutils \
        arm-none-eabi-gcc \
        arm-none-eabi-gdb \
        arm-none-eabi-newlib

ARG SDK_PATH=/usr/share/pico-sdk
RUN git clone --depth 1 --branch 1.5.0 https://github.com/raspberrypi/pico-sdk $SDK_PATH && \
    cd $SDK_PATH && \
    git submodule update --init

ENV PICO_SDK_PATH=$SDK_PATH

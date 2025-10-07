FROM ubuntu:jammy
# Use arm64 base image if building on x86 host for Raspberry Pi
# FROM arm64v8/ubuntu:jammy

ENV DEBIAN_FRONTEND=noninteractive
ENV TZ=Europe/Copenhagen

# Install all dependencies and cleanup in one step
RUN apt-get update && apt-get install -y \
    python3-pip \
    git \
    python3-jinja2 \
    libboost-dev \
    libgnutls28-dev \
    openssl \
    libtiff5-dev \
    pybind11-dev \
    qtbase5-dev \
    libqt5core5a \
    libqt5gui5 \
    libqt5widgets5 \
    cmake \
    python3-yaml \
    python3-ply \
    libglib2.0-dev \
    libgstreamer-plugins-base1.0-dev \
    tzdata \
    build-essential \
    ninja-build \
    && rm -rf /var/lib/apt/lists/*

# Install Meson via pip
RUN pip3 install meson

# Clone libcamera
WORKDIR /
RUN git clone https://github.com/raspberrypi/libcamera.git

WORKDIR /libcamera

# Setup and build
RUN meson setup build --buildtype=release \
    -Dpipelines=rpi/vc4,rpi/pisp \
    -Dipas=rpi/vc4,rpi/pisp \
    -Dv4l2=true \
    -Dgstreamer=enabled \
    -Dtest=false \
    -Dlc-compliance=disabled \
    -Dcam=disabled \
    -Dqcam=disabled \
    -Ddocumentation=disabled \
    -Dpycamera=enabled && \
    ninja -C build

# =========================================================
# Standalone Trakstar Dockerfile (No ROS2)
# =========================================================
FROM ubuntu:22.04

ENV DEBIAN_FRONTEND=noninteractive

# ---------------------------------------------------------
# Install system dependencies
# ---------------------------------------------------------
RUN apt-get update && apt-get install -y \
    build-essential \
    cmake \
    git \
    libusb-0.1-4 \
    libusb-dev \
    python3-dev \
    python3-pip \
    python3-pybind11 \
    udev \
    usbutils \
    && rm -rf /var/lib/apt/lists/*

# Install Python dependencies
RUN pip3 install numpy

# ---------------------------------------------------------
# Set the working directory
# ---------------------------------------------------------
WORKDIR /pytrak

# Copy source code
COPY . /pytrak/

# ---------------------------------------------------------
# Udev rules
# ---------------------------------------------------------
COPY 99-trakstar.rules /etc/udev/rules.d/99-libusb.rules
RUN udevadm control --reload-rules && udevadm trigger || true

# ---------------------------------------------------------
# Build and install
# ---------------------------------------------------------
RUN rm -rf build && mkdir -p build && cd build && \
    cmake .. -DCMAKE_BUILD_TYPE=Release && \
    make -j$(nproc) && \
    make install

# Set up environment
ENV PYTHONPATH=/usr/local/lib/python3/dist-packages:$PYTHONPATH

CMD ["/bin/bash"]

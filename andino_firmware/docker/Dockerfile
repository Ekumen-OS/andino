FROM python:3.11-slim-bookworm

ENV DEBIAN_FRONTEND=noninteractive

# Install system dependencies.
RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential \
    gcc \
    g++ \
    make \
    git \
    curl \
    udev \
    libusb-1.0-0 \
    libusb-1.0-0-dev \
    usbutils \
    doxygen \
    && rm -rf /var/lib/apt/lists/*

# Install PlatformIO and gcovr.
RUN pip install --no-cache-dir -U platformio gcovr

# Pre-create the PlatformIO cache directory with open permissions so that the
# named volume mounted here is writable by any host UID/GID at runtime.
RUN mkdir -p /tmp/.platformio && chmod 777 /tmp/.platformio

WORKDIR /workspace
CMD ["/bin/bash"]

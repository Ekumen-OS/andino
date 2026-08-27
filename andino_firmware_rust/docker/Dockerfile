FROM ubuntu:24.04 AS base

ENV USER ubuntu 

RUN apt-get update \
  && apt-get install -y --no-install-recommends \
  curl \
  avr-libc \
  gcc-avr \
  pkg-config \
  avrdude \
  libudev-dev \
  build-essential \
  sudo \
  ca-certificates \
  && rm -rf /var/lib/apt/lists/*

WORKDIR /home/${USER}
USER ${USER}

RUN curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | bash -s -- -y
ENV PATH="/home/${USER}/.cargo/bin:${PATH}"

RUN cargo +stable install ravedude

# RUN --mount=type=bind,source=./,target=/home/${USER}/andino_firmware_temp cp -r andino_firmware_temp andino_firmware_t && cd andino_firmware_t && cargo update
WORKDIR /home/${USER}/andino_firmware
COPY --chown=${USER}:${USER} . . 
RUN cargo build --release

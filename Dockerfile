FROM ubuntu:20.04

ENV DEBIAN_FRONTEND=noninteractive
WORKDIR /opt/formant-spot-adapter

RUN apt-get update && apt-get install -y \
  build-essential \
  cmake \
  ninja-build \
  ccache \
  pkg-config \
  git \
  curl \
  unzip \
  python3 \
  libopencv-dev \
  libeigen3-dev \
  libprotobuf-dev \
  protobuf-compiler \
  protobuf-compiler-grpc \
  libgrpc++-dev \
  && rm -rf /var/lib/apt/lists/*

COPY . .

RUN ./scripts/build.sh

ENV CONFIG_PATH=/opt/formant-spot-adapter/config/formant-spot-adapter.json

ENTRYPOINT ["./scripts/docker_run.sh"]

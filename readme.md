## Prerequisites

1. Install necessary libraries

```bash
# for macos
brew install sdl2 premake

# for debian-based linux
sudo apt install libsdl2-dev libglm-dev libegl-mesa0
```

2. Install [docker](https://www.docker.com)

```bash
# On linux, can run the install script
./linux_install_docker.sh
```

## Build C++ Locally

1. Populate submodules

```bash
git submodule update --init --recursive
```

2. Initialize python venv

```bash
python3 -m venv .venv
source ./.venv/bin/activate
```

2. Install required python libs

```bash
python3 -m pip install -r requirements.txt
```

3. Build C++ code

```
cd cpp
make
```

## Build docker image and container

1. Populate submodules

```
git submodule update --init --recursive
```

2. Do the docker build

```bash
./docker_utils.sh build

# See list of commands for docker_utils.sh
./docker_utils.sh help
```

## Visualization Script

It will connect to the debug client hosted on the server and visualize the iPhone
in 2D space.

```bash
python3 ./tools/visualize_iphone.py
```

## Telemetry Record and Replay

Utility scripts to record all telemetry data from iPhone and anchors. Record test data
once and use it to test without HW.

```bash
# Record some telemetry data, recording will stop when you press ctrl+c
python3 ./tools/telem_capture.py

# Replay the recorded telemetry
python3 tools/telem_replay.py --uid 69 data/walking_path_3.csv
```

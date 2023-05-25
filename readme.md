### Prerequisites

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

### Build C++ Locally

1. Populate submodules

```
git submodule update --init --recursive
```

2. Install required python libs

```
python3 -m pip install -r requirements.txt
```

3. Build C++ code

```
cd cpp
make
```

### Build docker image and container

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

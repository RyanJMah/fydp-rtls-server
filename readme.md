### Prerequisites

```bash
# for macos
brew install sdl2 premake

# for debian-based linux
sudo apt install libsdl2-dev libglm-dev libegl-mesa0
```

### Build

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

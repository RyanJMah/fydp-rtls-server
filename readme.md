### Prerequisites

TODO: I wrote the makefile for macos, need to port to linux probably later

1. Install SDL2 (needed for recast navigation demo)

```
# for macos
brew install sdl2
```

2. Install premake (needed for recast navigation demo)

```
# for macos
brew install premake
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

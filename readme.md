### Prerequisites

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

2. Build C++ code

```
cd cpp
make
```

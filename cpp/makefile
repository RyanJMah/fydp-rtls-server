TARGET     = main
SHARED_LIB = lib_pathfinding
BUILD_DIR  = build

SHARED_LIB_EXT := $(shell python3-config --extension-suffix)

DEBUG = 1
OPT = -O3

CPP = g++ -std=c++11
CC = gcc
CP = objcopy
SZ = size

CPP_SOURCES += $(wildcard src/*.cpp)
CPP_SOURCES += $(wildcard recast/src/*.cpp)

C_SOURCES += fastlz/fastlz.c

CPP_INCLUDES += -I./inc
CPP_INCLUDES += -I./recast/inc
CPP_INCLUDES += -I./fastlz
CPP_INCLUDES += -I/opt/homebrew/include
CPP_INCLUDES += -I../recastnavigation/Recast/Include
CPP_INCLUDES += -I../recastnavigation/Detour/Include
CPP_INCLUDES += -I../recastnavigation/DetourCrowd/Include
CPP_INCLUDES += -I../recastnavigation/DetourTileCache/Include
CPP_INCLUDES += $(shell python3 -m pybind11 --includes)

CPP_DEFINES += -D PY_MODULE_NAME=$(SHARED_LIB)

CPP_FLAGS = $(CPP_INCLUDES) $(OPT) -Wall -fPIC -MMD -MP -MF"$(@:%.o=%.d)"

ifeq ($(DEBUG), 1)
	CPP_FLAGS += -ggdb
endif

RECAST_DIR = ../recastnavigation
RECAST_BUILD_DIR = $(RECAST_DIR)/build

STATIC_LIBS += ../recastnavigation/build/Recast/libRecast.a
STATIC_LIBS += ../recastnavigation/build/Detour/libDetour.a
STATIC_LIBS += ../recastnavigation/build/DetourCrowd/libDetourCrowd.a
STATIC_LIBS += ../recastnavigation/build/DetourTileCache/libDetourTileCache.a

LD_FLAGS := $(shell python3-config --ldflags --embed)

OBJECTS = $(addprefix $(BUILD_DIR)/,$(notdir $(CPP_SOURCES:.cpp=.o)))
vpath %.cpp $(sort $(dir $(CPP_SOURCES)))

OBJECTS += $(addprefix $(BUILD_DIR)/,$(notdir $(C_SOURCES:.c=.o)))
vpath %.c $(sort $(dir $(C_SOURCES)))

.PHONY: all
all: $(BUILD_DIR)/$(TARGET) $(BUILD_DIR)/$(SHARED_LIB)$(SHARED_LIB_EXT)

.PHONY: run
run: all
	@echo "running $(BUILD_DIR)/$(TARGET)\n"
	@$(BUILD_DIR)/$(TARGET)

$(STATIC_LIBS):
	mkdir -p $(RECAST_BUILD_DIR)
	cd $(RECAST_BUILD_DIR) && cmake -DCMAKE_CXX_FLAGS=-I/opt/homebrew/include/SDL2 ..
	cd $(RECAST_BUILD_DIR) && make

$(BUILD_DIR)/%.o: %.cpp | $(BUILD_DIR) 
	$(CPP) -c $(CPP_FLAGS) $< -o $@

$(BUILD_DIR)/%.o: %.c | $(BUILD_DIR) 
	$(CC) -c $(CPP_FLAGS) $< -o $@

$(BUILD_DIR)/$(TARGET): $(OBJECTS) $(STATIC_LIBS)
	@echo "\nlinking c++ executable $@\n"
	$(CPP) -shared $(LD_FLAGS) $(OBJECTS) $(STATIC_LIBS) -o $@
	@echo ""
	@$(SZ) $@
	@echo ""

$(BUILD_DIR)/$(SHARED_LIB)$(SHARED_LIB_EXT): $(OBJECTS) $(STATIC_LIBS)
	@echo "linking shared library $@\n"
	$(CPP) -shared $(LD_FLAGS) $(OBJECTS) $(STATIC_LIBS) -o $@
	@echo ""
	@$(SZ) $@
	@echo ""

$(BUILD_DIR):
	mkdir $(BUILD_DIR)
	@echo ""

.PHONY: clean
clean:
	rm -rf $(BUILD_DIR)

.PHONY: clean_recast
clean_recast:
	rm -rf $(RECAST_BUILD_DIR)

.PHONY: clean_all
clean_all:
	rm -rf $(BUILD_DIR)
	rm -rf $(RECAST_BUILD_DIR)

TARGET = test
BUILD_DIR = build

DEBUG = 1
OPT = -O2

CPP = g++
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

CPP_FLAGS = $(CPP_INCLUDES) $(OPT) -Wall -MMD -MP -MF"$(@:%.o=%.d)"

ifeq ($(DEBUG), 1)
	CPP_FLAGS += -ggdb
endif

STATIC_LIBS += ../recastnavigation/build/Recast/libRecast.a
STATIC_LIBS += ../recastnavigation/build/Detour/libDetour.a
STATIC_LIBS += ../recastnavigation/build/DetourCrowd/libDetourCrowd.a
STATIC_LIBS += ../recastnavigation/build/DetourTileCache/libDetourTileCache.a

OBJECTS = $(addprefix $(BUILD_DIR)/,$(notdir $(CPP_SOURCES:.cpp=.o)))
vpath %.cpp $(sort $(dir $(CPP_SOURCES)))

OBJECTS += $(addprefix $(BUILD_DIR)/,$(notdir $(C_SOURCES:.c=.o)))
vpath %.c $(sort $(dir $(C_SOURCES)))

.PHONY: all
all: $(BUILD_DIR)/$(TARGET)

.PHONY: run
run: all
	@echo "running $(BUILD_DIR)/$(TARGET)\n"
	@$(BUILD_DIR)/$(TARGET)

$(BUILD_DIR)/%.o: %.cpp | $(BUILD_DIR) 
	$(CPP) -c $(CPP_FLAGS) $< -o $@

$(BUILD_DIR)/%.o: %.c | $(BUILD_DIR) 
	$(CC) -c $(CPP_FLAGS) $< -o $@

$(BUILD_DIR)/$(TARGET): $(OBJECTS)
	@echo ""
	$(CPP) $(OBJECTS) $(STATIC_LIBS) -o $@
	@echo ""
	@$(SZ) $@
	@echo ""

$(BUILD_DIR):
	mkdir $(BUILD_DIR)
	@echo ""

.PHONY: clean
clean:
	rm -rf $(BUILD_DIR)

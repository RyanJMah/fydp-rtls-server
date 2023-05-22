TARGET = test
BUILD_DIR = build

DEBUG = 1
OPT = -O2

CC = g++ -std=c++11
CP = objcopy
SZ = size

CPP_SOURCES = $(wildcard src/*.cpp)

CPP_INCLUDES = -I./inc
CPP_INCLUDES += -I/opt/homebrew/include
CPP_INCLUDES += -I../recastnavigation/Recast/Include
CPP_INCLUDES += -I../recastnavigation/Detour/Include

CPP_FLAGS = $(CPP_INCLUDES) $(OPT) -Wall -MMD -MP -MF"$(@:%.o=%.d)"

ifeq ($(DEBUG), 1)
	CPP_FLAGS += -ggdb
endif

RECAST_LIB = ../recastnavigation/build/Recast/libRecast.a
DETOUR_LIB = ../recastnavigation/build/Detour/libDetour.a

OBJECTS = $(addprefix $(BUILD_DIR)/,$(notdir $(CPP_SOURCES:.cpp=.o)))
vpath %.cpp $(sort $(dir $(CPP_SOURCES)))

.PHONY: all
all: $(BUILD_DIR)/$(TARGET)

.PHONY: run
run: all
	@echo "running $(BUILD_DIR)/$(TARGET)\n"

	@$(BUILD_DIR)/$(TARGET)

$(BUILD_DIR)/%.o: %.cpp | $(BUILD_DIR) 
	$(CC) -c $(CPP_FLAGS) $< -o $@

$(BUILD_DIR)/$(TARGET): $(OBJECTS)
	@echo ""
	$(CC) $(OBJECTS) $(RECAST_LIB) $(DETOUR_LIB) -o $@
	@echo ""
	@$(SZ) $@
	@echo ""

$(BUILD_DIR):
	mkdir $(BUILD_DIR)
	@echo ""

.PHONY: clean
clean:
	rm -rf $(BUILD_DIR)

#pragma once

#include <string>

extern "C" {

bool generate_navmesh(float tile_size, std::string in_file, std::string out_file);

}

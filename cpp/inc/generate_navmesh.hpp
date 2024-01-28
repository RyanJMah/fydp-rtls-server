#pragma once

#include <string>

extern "C" {

bool generate_tiled_navmesh( float tile_size,
                             float agent_radius,
                             std::string in_file,
                             std::string out_file );

bool generate_solo_navmesh( float cell_size,
                            float agent_radius,
                            std::string in_file,
                            std::string out_file );

}

#pragma once

#include <string>
#include <vector>
#include <array>
#include <stdint.h>

#include "DetourStatus.h"
#include "DetourNavMeshQuery.h"

struct GL_Coordinate
{
    float x;
    float y;
    float z;

    GL_Coordinate(void)
    {
        this->x = 0;
        this->y = 0;
        this->z = 0;
    }
};

class GL_Pathfinder
{
    public:
        GL_Pathfinder(void);
        ~GL_Pathfinder(void);

        /**
         * @brief   Load a navmesh (from a file) into the pathfinder
         * 
         * @param navmesh_filepath[in]      path to navmesh file
         */
        void load_navmesh( std::string navmesh_filepath );

        /**
         * @brief   Set the maximum amount of polygons for GL_Pathfinder::find_path
         * 
         * @param max_polygons[in]      number to set max polygons to, -1 means no limit
         * 
         * @return                      error code 
         */
        void set_max_output_path_polygons( int32_t max_polygons );

        /**
         * @brief   Finds the shortest traversible path from the start point
         *          to the end point
         * 
         * @param start[in]     Starting (x,y,z) of path
         * @param end[in]       Ending (x,y,z) of path
         * 
         * @return              Output path, empty if error occured
         */
        std::vector< std::array<float, 3> > find_path( GL_Coordinate start, GL_Coordinate end );

    private:
        bool    _is_initialized;
        int32_t _max_output_path_polygons;

        NavmeshHandle* _p_navmesh_handle;
        dtQueryFilter* _p_filter;
};
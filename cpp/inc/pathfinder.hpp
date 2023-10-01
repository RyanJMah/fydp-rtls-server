#pragma once

#include <string>
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
         * @brief   Finds the shortest traversible path from the start point
         *          to the end point
         * 
         * @param start[in]         awef
         * @param end[in]           awef
         * @param out_path[out]     awef
         * 
         * @return                  error code 
         */
        dtStatus find_path( GL_Coordinate start, GL_Coordinate end );

        /**
         * @brief   Set the maximum amount of polygons for GL_Pathfinder::find_path
         * 
         * @param max_polygons[in]      number to set max polygons to, -1 means no limit
         * 
         * @return                      error code 
         */
        void set_max_output_path_polygons( int32_t max_polygons );

    private:
        bool    _is_initialized;
        int32_t _max_output_path_polygons;



        dtQueryFilter*  _p_filter;
        dtNavMeshQuery* _p_navmesh_query;
};
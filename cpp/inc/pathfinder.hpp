#pragma once

#include <string>
#include <vector>
#include <array>
#include <stdint.h>
#include <stddef.h>

#include "Navmesh_Handle.h"
#include "DetourStatus.h"
#include "DetourNavMeshQuery.h"

#include "GL_RecastContext.hpp"

/*
 * NOTE: some potentially useful functions to look into if needed
 *
 * - DetourNavMeshQuery::findDistanceToWall
 */

typedef std::array<float, 3>  GL_Point;
typedef std::vector<GL_Point> GL_Path;

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
    void load_navmesh(std::string navmesh_filepath);

    /**
     * @brief   Finds the shortest traversible path from the start point
     *          to the end point
     * 
     * @param start[in]     Starting (x,y,z) of path
     * @param end[in]       Ending (x,y,z) of path
     * 
     * @return              Output path, empty if error occured
     */
    GL_Path find_path(GL_Point start, GL_Point end);

    /**
     * @brief   Set the radius of the pathfinding "agent" (i.e., the person
     *          who is using the pathfinding), default is (0.6, 0.6, 0)
     * 
     * @param r[in]     Agent radius to be set
     */
    void set_agent_xyz_radius(float x, float y, float z);

    /**
     * @brief   Set the max amount of polygons that can exist within
     *          a path, default is 2048
     * 
     * @param n[in]     Number of polygons
     */
    void set_path_max_polygons(size_t n);

    /**
     * @brief   Set the scaling factor for the navmesh (i.e., navmesh coords
     *          to real life coords), default is 1.0
     * 
     * @param scale[in]     Scaling factor
     */
    void set_navmesh_scale(float scale);

    /**
     * @brief   Set the smoothing factor for the path, default is 0.5
     * 
     * @param factor[in]    Smoothing factor
     */
    void set_smoothing_factor(float factor);
    
private:
    bool _is_initialized;

    // (x ,y, z) distances for agent radius
    float _half_extents[3];

    float _scale;
    float _smoothing_factor;

    dtPolyRef* _path;
    size_t     _path_max_len;

    GL_RecastContext* _p_ctx;
    NavmeshHandle*    _p_navmesh_handle;
    dtQueryFilter*    _p_filter;

    /*
     * Convenience pointers for accessing navmesh pointers
     * in _p_navmesh_handle
     * 
     * IMPORTANT: DO NOT DELETE/FREE THESE POINTERS
     */
    dtNavMesh*      _p_navmesh;
    dtNavMeshQuery* _p_nav_query;

    void _assign_convenience_ptrs(void);
};
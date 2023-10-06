#include <stdlib.h>
#include <string.h>

#include "Navmesh_Handle.h"
#include "macros.h"
#include "pathfinder.hpp"

#define MAX_NAVQUERY_SEARCH_NODES       ( 65535 )

// -1 ==> no limit
#define MAX_OUTPUT_PATH_POLYGONS        ( -1 )

/*
 * IMPORTANT: READ ME BEFORE WRITING ANY CODE
 *
 * Recast navigation treats the "y" direction as the "up" vector (for some reason???)
 * 
 * So, in recast navigation's eyes, the coordinate system looks like this...
 * 
 *          y
 *          ^
 *          |
 *          |
 *          |
 *          |
 *          |
 *          |- - - - - - - - - - - -> z
 *         |
 *        |
 *       |
 *      |
 *     v
 *    x
 * 
 * So we don't lose our mind, we will swap the y and z coordinate systems
 * in public functions, so we can use the "normal" coordinate system everywhere
 * else in our codebase.
 * 
 * The function "fix_coordinate_system" should be used for this
 */

static ALWAYS_INLINE void fix_coordinate_system(float xyz[3])
{
    // Swap y and z coordinates...
    std::swap( xyz[1], xyz[2] );
}

static void getPolyCenter(dtNavMesh* navMesh, dtPolyRef ref, float center[3])
{
    center[0] = 0;
    center[1] = 0;
    center[2] = 0;
    
    const dtMeshTile* tile = 0;
    const dtPoly* poly = 0;
    dtStatus status = navMesh->getTileAndPolyByRef(ref, &tile, &poly);
    if (dtStatusFailed(status))
        return;
        
    for (int i = 0; i < (int)poly->vertCount; ++i)
    {
        const float* v = &tile->verts[poly->verts[i]*3];
        center[0] += v[0];
        center[1] += v[1];
        center[2] += v[2];
    }
    const float s = 1.0f / poly->vertCount;
    center[0] *= s;
    center[1] *= s;
    center[2] *= s;
}

ALWAYS_INLINE void GL_Pathfinder::_assign_convenience_ptrs(void)
{
    this->_p_navmesh   = this->_p_navmesh_handle->m_navMesh;
    this->_p_nav_query = this->_p_navmesh_handle->m_navQuery;
}

GL_Pathfinder::GL_Pathfinder(void)
{
    this->_is_initialized = false;

    this->_half_extents[0] = 2;
    this->_half_extents[1] = 2;
    this->_half_extents[2] = 100;   // Overkill serach range for z axis, treats navmesh like 2-D

    this->_scale = 1.0F;

    fix_coordinate_system( this->_half_extents );

    this->_path_max_len = 2048;

    this->_path = (dtPolyRef* )malloc( this->_path_max_len*sizeof(dtPolyRef) );

    this->_p_ctx            = new GL_RecastContext();
    this->_p_filter         = new dtQueryFilter();
    this->_p_navmesh_handle = new NavmeshHandle();

    this->_p_navmesh_handle->setContext( this->_p_ctx );
}

GL_Pathfinder::~GL_Pathfinder(void)
{
    delete this->_p_ctx;
    delete this->_p_filter;
    delete this->_p_navmesh_handle;

    free(this->_path);
}

void GL_Pathfinder::load_navmesh(std::string navmesh_filepath)
{
    // Load navmesh handle
    this->_p_navmesh_handle->loadAll( navmesh_filepath.c_str() );

    this->_p_navmesh_handle->m_navQuery->init( this->_p_navmesh_handle->m_navMesh,
                                               MAX_NAVQUERY_SEARCH_NODES );

    this->_assign_convenience_ptrs();

    this->_is_initialized = true;
}

GL_Path GL_Pathfinder::find_path(GL_Point start_, GL_Point end_)
{
    dtStatus err_code = DT_FAILURE;
    GL_Path  ret;

    /*
     * Get C-style array pointers to the std::array inputs
     *
     * They are only std:array's for pybind11's sake
     */
    float* start = start_.data();
    float* end   = end_.data();

    fix_coordinate_system(start);
    fix_coordinate_system(end);


    // Immediately exit if un-initialized
    require(this->_is_initialized, exit);


    // Find the polygons closest to the start and end points
    dtPolyRef start_poly, end_poly;
    float start_pos[3];
    float end_pos[3];

    err_code = this->_p_nav_query->findNearestPoly( start,
                                                    this->_half_extents,
                                                    this->_p_filter,
                                                    &start_poly,
                                                    &start_pos[0] );
    require_dt_success(err_code, exit);
    require(start_poly != 0, exit);

    err_code = this->_p_nav_query->findNearestPoly( end,
                                                    this->_half_extents,
                                                    this->_p_filter,
                                                    &end_poly,
                                                    &end_pos[0] );
    require_dt_success(err_code, exit);
    require(end_poly != 0, exit);


    // Kick off pathfinding
    int path_len;
    err_code = this->_p_nav_query->findPath( start_poly,
                                             end_poly,
                                             start_pos,
                                             end_pos,
                                             this->_p_filter,
                                             &this->_path[0],
                                             &path_len,
                                             this->_path_max_len );
    require_dt_success(err_code, exit);

    // Put (x,y,z) of return polygons into return vector
    ret.resize(path_len);

    for (int i = 0; i < path_len; i++)
    {
        float xyz[3];
        getPolyCenter(this->_p_navmesh, this->_path[i], &xyz[0]);

        fix_coordinate_system(xyz);

        ret[i][0] = this->_scale * xyz[0];
        ret[i][1] = this->_scale * xyz[1];
        ret[i][2] = this->_scale * xyz[2];
    }

exit:
    if (err_code != DT_SUCCESS)
    {
        ret.clear();
    }

    return ret;
}

void GL_Pathfinder::set_agent_xyz_radius(float x, float y, float z)
{
    this->_half_extents[0] = x;
    this->_half_extents[1] = y;
    this->_half_extents[2] = z;

    fix_coordinate_system( this->_half_extents );
}

void GL_Pathfinder::set_path_max_polygons(size_t n)
{
    this->_path_max_len = n;

    this->_path = (dtPolyRef* )realloc( this->_path, n*sizeof(dtPolyRef) );
}

void GL_Pathfinder::set_navmesh_scale(float scale)
{
    this->_scale = scale;
}

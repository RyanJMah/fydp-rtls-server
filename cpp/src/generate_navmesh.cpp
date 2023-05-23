#include <iostream>
#include <stdint.h>
#include <string.h>
#include "Recast.h"
#include "generate_navmesh.hpp"

#if 0
static rcConfig g_build_cfg =
{
    .tileSize = 32,
    .cs = 2.0F,
    .ch = 0.20F,
    .bmin = {1, 1, 1},
    .bmax = {10, 10, 10},
};

bool GenerateNavMesh(
        int width,
        int height,
        const float* vertices,
        const int    num_vertices,
        const int*   triangles,
        const int    num_triangles,
        const char*  outputPath )
{
    rcContext ctx;

    ////////////////////////////////////////////////////////////////
    // 1. Initialize build config
    g_build_cfg.width = width;
    g_build_cfg.height = height;
    ////////////////////////////////////////////////////////////////

    ////////////////////////////////////////////////////////////////
    // Rasterize input polygon soup

    // Create a heightfield object
    rcHeightfield* hf = rcAllocHeightfield();

    if ( !rcCreateHeightfield(
                &ctx,
                *hf,
                g_build_cfg.width,
                g_build_cfg.height,
                g_build_cfg.bmin,
                g_build_cfg.bmax,
                g_build_cfg.cs,
                g_build_cfg.ch ) )
    {
        // Handle heightfield creation failure
        std::cerr << "failed to create height field" << std::endl;
        return false;
    }

    // Allocate array that can hold triangle area types.
    uint8_t* tri_areas = new uint8_t[num_triangles];
    if ( !tri_areas )
    {
        std::cerr << "failed to allocate buffer for triangle areas" << std::endl;
        return false;
    }

	// Find triangles which are walkable based on their slope and rasterize them.
    memset( tri_areas, 0, num_triangles*sizeof(uint8_t) );
    rcMarkWalkableTriangles(
            ctx,
            g_build_cfg.walkableSlopeAngle,
            vertices,
            num_vertices,
            triangles,
            num_triangles,
            tri_areas );

    if ( !rcRasterizeTriangles(
                m_ctx,
                vertices,
                num_vertices,
                triangles,
                num_triangles,
                hf,
                g_build_cfg.walkableClimb) )
    {
        return false;
    }
    ////////////////////////////////////////////////////////////////

    ////////////////////////////////////////////////////////////////
    // Filter walkable surfaces
    rcFilterLowHangingWalkableObstacles(
                m_ctx, g_build_cfg.walkableClimb, hf );

    rcFilterLedgeSpans(
                m_ctx,
                g_build_cfg.walkableHeight,
                g_build_cfg.walkableClimb,
                hf );

    rcFilterWalkableLowHeightSpans(
                ctx, g_build_cfg.walkableHeight, hf);
    ////////////////////////////////////////////////////////////////

    ////////////////////////////////////////////////////////////////
    // Partition walkable surface to simple regions
    rcCompactHeightfield* chf = rcAllocCompactHeightfield();
    if (!chf)
    {
        return false;
    }

    if ( !rcBuildCompactHeightfield(
                ctx,
                g_build_cfg.walkableHeight,
                g_build_cfg.walkableClimb,
                hf,
                chf) )
    {
        return false;
    }

    if ()

    ////////////////////////////////////////////////////////////////


    // clean up
    rcFreeHeightField(hf);
    delete[] tri_areas;
    rcFreeCompactHeightfield(chf);

    return true;
}
#endif

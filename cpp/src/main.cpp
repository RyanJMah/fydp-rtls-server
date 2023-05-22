#include <iostream>
#include <stdint.h>
#include <Recast.h>
#include <DetourNavMeshBuilder.h>

static rcConfig g_build_cfg =
{
    .width = 200,
    .height = 200,
    .cs = 10,
    .ch = 10,
    .bmin = {1, 1, 1},
    .bmax = {10, 10, 10},
};

void GenerateNavMesh(
        const unsigned char* heightmapData,
        const char* outputPath );

int main();

void GenerateNavMesh(
        const unsigned char* heightmapData,
        const char* outputPath )
{
    rcContext ctx;

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
        return;
    }



    // clean up
    rcFreeHeightField(hf);

#if (0)
    // Set the heightfield data
    for (int y = 0; y < height; ++y) {
        for (int x = 0; x < width; ++x) {
            unsigned char value = heightmapData[y * width + x];
            hf.spans[y * width + x].y = value * cellHeight;
        }
    }

    // Filter the heightfield to remove obstacles
    rcFilterLowHangingWalkableObstacles(&ctx, hf);
    rcFilterLedgeSpans(&ctx, hf);
    rcFilterWalkableLowHeightSpans(&ctx, hf, 1.0f);

    // Create a compact heightfield object
    rcCompactHeightfield chf;
    if (!rcBuildCompactHeightfield(&ctx, 1, hf)) {
        // Handle compact heightfield generation failure
        return;
    }

    // Create a contour set object
    rcContourSet cset;
    if (!rcBuildContours(&ctx, chf, 2.0f, 10, cset)) {
        // Handle contour set generation failure
        return;
    }

    // Create a polygon mesh object
    rcPolyMesh pmesh;
    if (!rcBuildPolyMesh(&ctx, cset, 6, pmesh)) {
        // Handle polygon mesh generation failure
        return;
    }

    // Create a detail mesh object
    rcPolyMeshDetail dmesh;
    if (!rcBuildPolyMeshDetail(&ctx, pmesh, chf, 6, 12, dmesh)) {
        // Handle detail mesh generation failure
        return;
    }

    // Create a navigation mesh object
    dtNavMeshCreateParams params;
    memset(&params, 0, sizeof(params));
    params.verts = pmesh.verts;
    params.vertCount = pmesh.nverts;
    params.polys = pmesh.polys;
    params.polyAreas = pmesh.areas;
    params.polyFlags = pmesh.flags;
    params.polyCount = pmesh.npolys;
    params.nvp = pmesh.nvp;
    params.detailMeshes = dmesh.meshes;
    params.detailVerts = dmesh.verts;
    params.detailVertsCount = dmesh.nverts;
    params.detailTris = dmesh.tris;
    params.detailTriCount = dmesh.ntris;

    // Create the navigation mesh
    dtNavMesh* navMesh = nullptr;
    dtStatus status = dtCreateNavMeshData(&params, &navMesh);
    if (dtStatusFailed(status)) {
        // Handle navigation mesh creation failure
        return;
    }

    // Save the navigation mesh to a file
    status = navMesh->saveNavMeshBin(outputPath);
    if (dtStatusFailed(status)) {
        // Handle navigation mesh save failure
        return;
    }

    // Clean up resources
    dtFreeNavMesh(navMesh);
#endif
}

int main()
{
    std::cout << "calling GenerateNavMesh()..." << std::endl;

    GenerateNavMesh(NULL, NULL);

    std::cout << "done GenerateNavMesh()..." << std::endl;

    return 0;
}


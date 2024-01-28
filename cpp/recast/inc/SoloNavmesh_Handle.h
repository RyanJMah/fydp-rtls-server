#pragma once

#include "BaseBuilder.h"
#include "DetourNavMesh.h"
#include "Recast.h"

class SoloNavmeshHandle : public Builder
{
protected:
    bool m_keepInterResults;
    float m_totalBuildTimeMs;

    unsigned char* m_triareas;
    rcHeightfield* m_solid;
    rcCompactHeightfield* m_chf;
    rcContourSet* m_cset;
    rcPolyMesh* m_pmesh;
    rcConfig m_cfg;    
    rcPolyMeshDetail* m_dmesh;

    enum SamplePartitionType
    {
        PARTITION_WATERSHED,
        PARTITION_MONOTONE,
        PARTITION_LAYERS
    };
    
    enum DrawMode
    {
        DRAWMODE_NAVMESH,
        DRAWMODE_NAVMESH_TRANS,
        DRAWMODE_NAVMESH_BVTREE,
        DRAWMODE_NAVMESH_NODES,
        DRAWMODE_NAVMESH_INVIS,
        DRAWMODE_MESH,
        DRAWMODE_VOXELS,
        DRAWMODE_VOXELS_WALKABLE,
        DRAWMODE_COMPACT,
        DRAWMODE_COMPACT_DISTANCE,
        DRAWMODE_COMPACT_REGIONS,
        DRAWMODE_REGION_CONNECTIONS,
        DRAWMODE_RAW_CONTOURS,
        DRAWMODE_BOTH_CONTOURS,
        DRAWMODE_CONTOURS,
        DRAWMODE_POLYMESH,
        DRAWMODE_POLYMESH_DETAIL,
        MAX_DRAWMODE
    };
    
    DrawMode m_drawMode;
    
    void cleanup();

public:
    SoloNavmeshHandle();
    virtual ~SoloNavmeshHandle();
    
    virtual bool handleBuild();

private:
    // Explicitly disabled copy constructor and copy assignment operator.
    SoloNavmeshHandle(const SoloNavmeshHandle&);
    SoloNavmeshHandle& operator=(const SoloNavmeshHandle&);
};

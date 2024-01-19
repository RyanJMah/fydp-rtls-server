//
// Copyright (c) 2009-2010 Mikko Mononen memon@inside.org
//
// This software is provided 'as-is', without any express or implied
// warranty.  In no event will the authors be held liable for any damages
// arising from the use of this software.
// Permission is granted to anyone to use this software for any purpose,
// including commercial applications, and to alter it and redistribute it
// freely, subject to the following restrictions:
// 1. The origin of this software must not be misrepresented; you must not
//    claim that you wrote the original software. If you use this software
//    in a product, an acknowledgment in the product documentation would be
//    appreciated but is not required.
// 2. Altered source versions must be plainly marked as such, and must not be
//    misrepresented as being the original software.
// 3. This notice may not be removed or altered from any source distribution.
//

#ifndef RECASTBUILDER_H
#define RECASTBUILDER_H

#include <iostream>
#include "Recast.h"

/// These are just sample areas to use consistent values across the samples.
/// The use should specify these base on his needs.
enum BuilderPolyAreas
{
	BUILDER_POLYAREA_GROUND,
	BUILDER_POLYAREA_WATER,
	BUILDER_POLYAREA_ROAD,
	BUILDER_POLYAREA_DOOR,
	BUILDER_POLYAREA_GRASS,
	BUILDER_POLYAREA_JUMP
};
enum BuilderPolyFlags
{
	BUILDER_POLYFLAGS_WALK		= 0x01,		// Ability to walk (ground, grass, road)
	BUILDER_POLYFLAGS_SWIM		= 0x02,		// Ability to swim (water).
	BUILDER_POLYFLAGS_DOOR		= 0x04,		// Ability to move through doors.
	BUILDER_POLYFLAGS_JUMP		= 0x08,		// Ability to jump.
	BUILDER_POLYFLAGS_DISABLED	= 0x10,		// Disabled polygon
	BUILDER_POLYFLAGS_ALL		= 0xffff	// All abilities.
};

enum BuilderPartitionType
{
	BUILDER_PARTITION_WATERSHED,
	BUILDER_PARTITION_MONOTONE,
	BUILDER_PARTITION_LAYERS
};

class Builder
{
public:
	class dtNavMesh* m_navMesh;
	class dtNavMeshQuery* m_navQuery;

protected:
	class InputGeom* m_geom;
	class dtCrowd* m_crowd;

	float m_cellSize;
	float m_cellHeight;
	float m_agentHeight;
	float m_agentRadius;
	float m_agentMaxClimb;
	float m_agentMaxSlope;
	float m_regionMinSize;
	float m_regionMergeSize;
	float m_edgeMaxLen;
	float m_edgeMaxError;
	float m_vertsPerPoly;
	float m_detailSampleDist;
	float m_detailSampleMaxError;
	int m_partitionType;

	bool m_filterLowHangingObstacles;
	bool m_filterLedgeSpans;
	bool m_filterWalkableLowHeightSpans;
	
	rcContext* m_ctx;

	dtNavMesh* loadAll(const char* path);
	void saveAll(const char* path, const dtNavMesh* mesh);

public:
	Builder();
	virtual ~Builder();
	
	void setContext(rcContext* ctx) { m_ctx = ctx; }
	
	virtual void handleMeshChanged(class InputGeom* geom);
	virtual bool handleBuild();
	virtual void collectSettings(struct BuildSettings& settings);

	virtual class InputGeom* getInputGeom() { return m_geom; }
	virtual class dtNavMesh* getNavMesh() { return m_navMesh; }
	virtual class dtNavMeshQuery* getNavMeshQuery() { return m_navQuery; }
	virtual class dtCrowd* getCrowd() { return m_crowd; }
	virtual float getAgentRadius() { return m_agentRadius; }
	virtual float getAgentHeight() { return m_agentHeight; }
	virtual float getAgentClimb() { return m_agentMaxClimb; }
	
    void setTileSize(float tileSize);
    void setAgentRadius(float agentRadius);
    
	void resetCommonSettings();
	void handleCommonSettings();

	void save(const char* path);

private:
	// Explicitly disabled copy constructor and copy assignment operator.
	Builder(const Builder&);
	Builder& operator=(const Builder&);
};


#endif // RECASTBUILDER_H

#pragma once

// workaround to get all the recast functions to compile with C linkage

extern "C" {

#include "BaseBuilder.h"
#include "ChunkyTriMesh.h"
#include "InputGeom.h"
#include "MeshLoaderObj.h"
#include "Navmesh_Builder.h"

}

#include "Recast.h"
#include "InputGeom.h"
#include "Sample_TempObstacles.h"
#include "generate_navmesh.hpp"

bool generate_navmesh(const char* in_file, const char* out_file)
{
    rcContext ctx;

    bool ret_code = true;

    InputGeom* input_geom = new InputGeom();
    Sample_TempObstacles* navmesh_builder = new Sample_TempObstacles();

    if ( !input_geom->load( &ctx, in_file ) )
    {
        ret_code = false;
        goto exit;
    }

    navmesh_builder->setContext( &ctx );
    navmesh_builder->resetCommonSettings();
    navmesh_builder->handleMeshChanged( input_geom );
    navmesh_builder->handleSettings();

    if ( !navmesh_builder->handleBuild() )
    {
        ret_code = false;
        goto exit;
    }

    navmesh_builder->save( out_file );

exit:
    delete input_geom;
    delete navmesh_builder;

    return ret_code;
}

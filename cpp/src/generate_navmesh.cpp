#include <iostream>
#include "Recast.h"
#include "InputGeom.h"
#include "Navmesh_Builder.h"
#include "generate_navmesh.hpp"

bool generate_navmesh(std::string in_file, std::string out_file)
{
    rcContext ctx;

    bool ret_code = true;

    InputGeom* input_geom = new InputGeom();
    NavmeshBuilder* navmesh_builder = new NavmeshBuilder();

    if ( !input_geom->load( &ctx, in_file.c_str() ) )
    {
        ret_code = false;
        goto cleanup;
    }

    navmesh_builder->setContext( &ctx );
    navmesh_builder->resetCommonSettings();
    navmesh_builder->handleMeshChanged( input_geom );
    navmesh_builder->handleSettings();

    if ( !navmesh_builder->handleBuild() )
    {
        ret_code = false;
        goto cleanup;
    }

    navmesh_builder->save( out_file.c_str() );

cleanup:
    delete input_geom;
    delete navmesh_builder;

exit:
    return ret_code;
}

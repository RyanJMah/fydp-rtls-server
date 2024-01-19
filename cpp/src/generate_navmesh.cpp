#include "Recast.h"
#include "InputGeom.h"
#include "Navmesh_Handle.h"

#include "macros.h"
#include "GL_RecastContext.hpp"
#include "generate_navmesh.hpp"

bool generate_navmesh( float tile_size,
                       float agent_radius,
                       std::string in_file,
                       std::string out_file )
{
    GL_RecastContext ctx;

    bool ret_code = true;

    InputGeom* input_geom = new InputGeom();
    NavmeshHandle* navmesh_builder = new NavmeshHandle();

    if ( !input_geom->load( &ctx, in_file.c_str() ) )
    {
        ret_code = false;
        goto_error(exit, "Failed to load input geometry");
    }

    navmesh_builder->setContext( &ctx );

    navmesh_builder->resetCommonSettings();
    navmesh_builder->setTileSize( tile_size );
    navmesh_builder->setAgentRadius( agent_radius );

    navmesh_builder->handleMeshChanged( input_geom );
    navmesh_builder->handleSettings();

    if ( !navmesh_builder->handleBuild() )
    {
        ret_code = false;
        goto_error(exit, "Failed to build navmesh");
    }

    navmesh_builder->saveAll( out_file.c_str() );

exit:
    delete input_geom;
    delete navmesh_builder;

    return ret_code;
}

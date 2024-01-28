#include "Recast.h"
#include "InputGeom.h"
#include "SoloNavmesh_Handle.h"
#include "TiledNavmesh_Handle.h"

#include "macros.h"
#include "GL_RecastContext.hpp"
#include "generate_navmesh.hpp"

bool generate_tiled_navmesh( float tile_size,
                       float agent_radius,
                       std::string in_file,
                       std::string out_file )
{
    GL_RecastContext ctx;

    bool ret_code = true;

    InputGeom* input_geom = new InputGeom();
    TiledNavmeshHandle* navmesh_builder = new TiledNavmeshHandle();

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

bool generate_solo_navmesh( float cell_size,
                            float agent_radius,
                            std::string in_file,
                            std::string out_file )
{
    GL_RecastContext ctx;

    bool ret_code = true;

    InputGeom* input_geom = new InputGeom();
    SoloNavmeshHandle* navmesh_builder = new SoloNavmeshHandle();

    if ( !input_geom->load( &ctx, in_file.c_str() ) )
    {
        ret_code = false;
        goto_error(exit, "Failed to load input geometry");
    }

    navmesh_builder->setContext( &ctx );

    navmesh_builder->resetCommonSettings();
    navmesh_builder->setTileSize( cell_size );
    navmesh_builder->setAgentRadius( agent_radius );

    navmesh_builder->handleMeshChanged( input_geom );

    if ( !navmesh_builder->handleBuild() )
    {
        ret_code = false;
        goto_error(exit, "Failed to build navmesh");
    }

    navmesh_builder->saveAll( out_file.c_str(), navmesh_builder->getNavMesh() );

exit:
    delete input_geom;
    delete navmesh_builder;

    return ret_code;
}

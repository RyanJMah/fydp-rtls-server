#include <iostream>
#include <stdarg.h>

#include "Recast.h"
#include "InputGeom.h"
#include "Navmesh_Handle.h"

#include "macros.h"
#include "generate_navmesh.hpp"

class GL_BuildContext : public rcContext
{
    public:
        // override to print to stdout
	    virtual void doLog(const rcLogCategory category, const char* msg, const int len)
        {
            switch ( category )
            {
                case RC_LOG_PROGRESS:
                {
                    std::cout << "RECAST-INFO: ";
                    break;
                }

                case RC_LOG_WARNING:
                {
                    std::cout << "RECAST-WARNING: ";
                    break;
                }

                case RC_LOG_ERROR:
                {
                    std::cout << "RECAST-ERROR: ";
                    break;
                }

                default:
                {
                    std::cout << "INVALID LOG CATEGORY, ABORTING!!!!" << std::endl;
                    exit(1);
                }
            }

            std::cout << msg << std::endl;
        }
};

bool generate_navmesh(std::string in_file, std::string out_file)
{
    GL_BuildContext ctx;

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
    navmesh_builder->handleMeshChanged( input_geom );
    navmesh_builder->handleSettings();

    if ( !navmesh_builder->handleBuild() )
    {
        ret_code = false;
        goto_error(exit, "Failed to build navmesh");
    }

    navmesh_builder->save( out_file.c_str() );

exit:
    delete input_geom;
    delete navmesh_builder;

    return ret_code;
}

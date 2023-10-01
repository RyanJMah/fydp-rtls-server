#include "Navmesh_Handle.h"
#include "pathfinder.hpp"

GL_Pathfinder::GL_Pathfinder(void)
{
    this->_p_filter = new dtQueryFilter();

    this->_is_initialized = false;
    this->_max_output_path_polygons = -1;   // -1 ==> no limit
}

GL_Pathfinder::~GL_Pathfinder(void)
{
    delete this->_p_filter;

    if ( this->_is_initialized )
    {
        delete this->_p_navmesh_query;
    }
}

void GL_Pathfinder::load_navmesh( std::string navmesh_filepath )
{
//     NavmeshBuilder* builder = new NavmeshBuilder();

//     // Load into builder object
//     builder->loadAll( navmesh_filepath.c_str() );



// exit:
    // delete builder;
}
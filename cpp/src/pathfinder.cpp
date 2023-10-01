#include "Navmesh_Handle.h"
#include "pathfinder.hpp"

GL_Pathfinder::GL_Pathfinder(void)
{
    this->_p_filter = new dtQueryFilter();
    this->_p_navmesh_handle = new NavmeshHandle();

    this->_is_initialized = false;
    this->_max_output_path_polygons = -1;   // -1 ==> no limit
}

GL_Pathfinder::~GL_Pathfinder(void)
{
    delete this->_p_filter;
    delete this->_p_navmesh_handle;
}

void GL_Pathfinder::load_navmesh( std::string navmesh_filepath )
{
    // Load navmesh handle
    this->_p_navmesh_handle->loadAll( navmesh_filepath.c_str() );
}

void GL_Pathfinder::set_max_output_path_polygons( int32_t max_polygons )
{
    this->_max_output_path_polygons = max_polygons;
}

std::vector< std::array<float, 3> > find_path( GL_Coordinate start, GL_Coordinate end )
{

}
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include "generate_navmesh.hpp"
#include "pathfinder.hpp"

namespace py = pybind11;

PYBIND11_MODULE(lib_pathfinding, m) {
    m.doc() = "C++ Pathfinding Library"; // optional module docstring

    m.def("generate_navmesh", &generate_navmesh, "Generate a navmesh given an OBJ file");

    py::class_<GL_Pathfinder>(m, "Pathfinder")
        .def(py::init<>())
        .def("load_navmesh",          &GL_Pathfinder::load_navmesh)
        .def("find_path",             &GL_Pathfinder::find_path)
        .def("set_agent_xyzradius",   &GL_Pathfinder::set_agent_xyz_radius)
        .def("set_path_max_polygons", &GL_Pathfinder::set_path_max_polygons)
        .def("set_navmesh_scale",     &GL_Pathfinder::set_navmesh_scale);
}

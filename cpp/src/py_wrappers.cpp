#include <pybind11/pybind11.h>

#include "generate_navmesh.hpp"
#include "pathfinder.hpp"

namespace py = pybind11;

PYBIND11_MODULE(lib_pathfinding, m) {
    m.doc() = "C++ Pathfinding Library"; // optional module docstring

    m.def("generate_navmesh", &generate_navmesh, "Generate a navmesh given an OBJ file");

    py::class_<GL_Pathfinder>(m, "Pathfinder").def("find_path", &GL_Pathfinder::find_path);
}

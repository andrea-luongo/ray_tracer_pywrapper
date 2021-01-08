#include <pybind11.h>
#include <pybind11/stl.h>
//#include <BVH.h>
//#include <WindowsNumerics.h>
//#include <vector>

namespace py = pybind11;

class PyBindBVH {

//private:
    //BVH* bvh = nullptr;
public:
    PyBindBVH() { };
    //PyBindBVH(std::vector<float>& vertices, int maxPrimsInNode)//, SplitMethod splitMethod)
    //{
    // /*   std::vector<std::shared_ptr<Primitive>> primitives;
    //    for (int i = 0; i < (int)(vertices.size() / 9); i++)
    //    {
    //        
    //        float3 p0(vertices[i*9], vertices[i*9+1], vertices[i*9+2]);
    //        float3 p1(vertices[i*9+3], vertices[i*9+4], vertices[i*9+5]);
    //        float3 p2(vertices[i*6], vertices[i*9+7], vertices[i*9+8]);
    //        std::shared_ptr<Primitive> primitive = std::shared_ptr<Triangle>(new Triangle(p0, p1, p2));
    //        primitives.push_back(primitive);
    //    }
    //    bvh = new BVH(primitives, splitMethod, maxPrimsInNode);*/
    //}

    ~PyBindBVH() {
        //delete bvh;
    }
};


int add(int i, int j) {
    return i + j;
}

PYBIND11_MODULE(ray_tracer_pywrapper, m) {
    m.doc() = "pybind11 example plugin"; // optional module docstring

    m.def("add", &add, "A function which adds two numbers");
}

//PYBIND11_MODULE(ray_tracer_pywrapper, m) {
//    m.doc() = R"pbdoc(
//        Pybind wrapper for BVH build and traversal
//    )pbdoc";
//    py::class_<PyBindBVH> bvh(m, "PyBindBVH");
//    bvh.def(py::init());// , SplitMethod > ());
//    //bvh.def(py::init < std::vector<float>&, int>());// , SplitMethod > ());
//  /*  py::enum_ <SplitMethod> (bvh, "SplitMethod")
//        .value("SAH", SplitMethod::SAH)
//        .value("HLBVH", SplitMethod::HLBVH)
//        .value("Middle", SplitMethod::Middle)
//        .value("EqualCounts", SplitMethod::EqualCounts)
//        .export_values();*/
//#ifdef VERSION_INFO
//    m.attr("__version__") = VERSION_INFO;
//#else
//    m.attr("__version__") = "dev";
//#endif
//}




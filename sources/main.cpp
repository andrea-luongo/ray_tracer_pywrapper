#include <pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/numpy.h>
#include <BVH.h>
//#include <vector>
#include <time.h>

namespace py = pybind11;

class PyBindPlane
{
public:
    Plane* plane = nullptr;
public:
    PyBindPlane()
    {
        plane = new Plane();
    }
    PyBindPlane(py::array_t<float> x, py::array_t<float> n)
    {
        float3 x0(x.at(0), x.at(1), x.at(2));
        float3 normal(n.at(0), n.at(1), n.at(2));
        plane = new Plane(x0, normal);
    }
    py::array_t<float> GetX() const {
        float3 x = plane->GetX();
        auto result = py::array(py::buffer_info(
            nullptr,            /* Pointer to data (nullptr -> ask NumPy to allocate!) */
            sizeof(float),     /* Size of one item */
            py::format_descriptor<float>::value, /* Buffer format */
            1,          /* How many dimensions? */
            { 3 },  /* Number of elements for each dimension */
            { sizeof(float) }  /* Strides for each dimension */
        ));
        auto buf = result.request();
        float* ptr = (float*)buf.ptr;
        ptr[0] = x.x;
        ptr[1] = x.y;
        ptr[2] = x.z;
        return result;
    };
    py::array_t<float> GetNormal() const {
        float3 n = plane->GetNormal();
        auto result = py::array(py::buffer_info(
            nullptr,            /* Pointer to data (nullptr -> ask NumPy to allocate!) */
            sizeof(float),     /* Size of one item */
            py::format_descriptor<float>::value, /* Buffer format */
            1,          /* How many dimensions? */
            { 3 },  /* Number of elements for each dimension */
            { sizeof(float) }  /* Strides for each dimension */
        ));
        auto buf = result.request();
        float* ptr = (float*)buf.ptr;
        ptr[0] = n.x;
        ptr[1] = n.y;
        ptr[2] = n.z;
        return result;
    };
};

class PyBindPlaneInfo
{
public:
    PlaneIntersectionInfo* planeInfo = nullptr;
public:
    PyBindPlaneInfo()
    {
        planeInfo = new PlaneIntersectionInfo();
    };
    std::vector<py::array_t<float>> GetHits()
    { 
        std::vector<float3> hits = *planeInfo->GetHits();
        std::vector<py::array_t<float>> result;
        for (int i = 0; i < planeInfo->GetHitsSize(); i++)
        {
            float3 value = hits[i];
            auto tmp = py::array(py::buffer_info(
                nullptr,            /* Pointer to data (nullptr -> ask NumPy to allocate!) */
                sizeof(float),     /* Size of one item */
                py::format_descriptor<float>::value, /* Buffer format */
                1,          /* How many dimensions? */
                { 3 },  /* Number of elements for each dimension */
                { sizeof(float) }  /* Strides for each dimension */
            ));
            auto buf = tmp.request();
            float* ptr = (float*)buf.ptr;
            ptr[0] = value.x;
            ptr[1] = value.y;
            ptr[2] = value.z;
            result.push_back(tmp);
        }
        return result;
    };
    void AddHit(py::array_t<float> t)
    { 
        float3 x0(t.at(0), t.at(1), t.at(2));
        planeInfo->AddHit(x0);
    };
    int GetHitsSize() { return planeInfo->GetHitsSize(); };

};

class PyBindRay {
public:
    Ray* ray = nullptr;
public:
    PyBindRay()
    {
        float3 origin(0, 0, 0);
        float3 direction(0, 0, 1);
        ray = new Ray(origin, direction, 0, std::numeric_limits<float>::max(), 0, 0);
    }

    PyBindRay(py::array_t<float> o, py::array_t<float> dir, float min, float max, int d, int s)
    {
        float3 origin(o.at(0), o.at(1), o.at(2));
        float3 direction(dir.at(0), dir.at(1), dir.at(2));
        ray = new Ray(origin, direction, min, max, d, s);
    };

    py::array_t<float> GetDirection() const { 
        float3 d = ray->GetDirection();
        auto result = py::array(py::buffer_info(
            nullptr,            /* Pointer to data (nullptr -> ask NumPy to allocate!) */
            sizeof(float),     /* Size of one item */
            py::format_descriptor<float>::value, /* Buffer format */
            1,          /* How many dimensions? */
            { 3 },  /* Number of elements for each dimension */
            { sizeof(float) }  /* Strides for each dimension */
        ));
        auto buf = result.request();
        float * ptr = (float*)buf.ptr;
        ptr[0] = d.x;
        ptr[1] = d.y;
        ptr[2] = d.z;
        return result;
    };
    py::array_t<float> GetOrigin() const { 
        float3 o = ray->GetOrigin();
        auto result = py::array(py::buffer_info(
            nullptr,            /* Pointer to data (nullptr -> ask NumPy to allocate!) */
            sizeof(float),     /* Size of one item */
            py::format_descriptor<float>::value, /* Buffer format */
            1,          /* How many dimensions? */
            { 3 },  /* Number of elements for each dimension */
            { sizeof(float) }  /* Strides for each dimension */
        ));
        auto buf = result.request();
        float* ptr = (float*)buf.ptr;
        ptr[0] = o.x;
        ptr[1] = o.y;
        ptr[2] = o.z;
        return result;
    };
    float GetMin() const { return ray->GetMin(); };
    float GetMax() const { return ray->GetMax(); };
    void SetMin(float t) { ray->SetMin(t); };
    void SetMax(float t) { ray->SetMax(t); };
};

class PyBindRayInfo {
public:
    RayIntersectionInfo* rayInfo = nullptr;
public:
    PyBindRayInfo()
    {
        rayInfo = new RayIntersectionInfo();
    };
    std::vector<float> GetHits() { return *rayInfo->GetHits(); };
    void SetNormal(float3 n) { rayInfo->SetNormal(n); };
    float3 GetNormal() { return rayInfo->GetNormal(); };
    void AddHit(float t) { rayInfo->AddHit(t); };
    void AddClosestHit(float t) { rayInfo->AddClosestHit(t); };
    int GetHitsSize() { return rayInfo->GetHitsSize(); };
};

class PyBindBVH {

private:
    BVH* bvh = nullptr;
public:
    //PyBindBVH() { };
    PyBindBVH(std::vector<float>& vertices, SplitMethod splitMethod, int maxPrimsInNode=255)
    {
        std::vector<std::shared_ptr<Primitive>> primitives;
        for (int i = 0; i < (int)(vertices.size() / 9); i++)
        {
            float3 p0(vertices[i*9], vertices[i*9+1], vertices[i*9+2]);
            float3 p1(vertices[i*9+3], vertices[i*9+4], vertices[i*9+5]);
            float3 p2(vertices[i*9+6], vertices[i*9+7], vertices[i*9+8]);
            std::shared_ptr<Primitive> primitive = std::shared_ptr<Triangle>(new Triangle(p0, p1, p2));
            primitives.push_back(primitive);
        }
        bvh = new BVH(primitives, splitMethod, maxPrimsInNode);
    }

    ~PyBindBVH() {
        //delete bvh;
    }

    py::tuple MultiRayIntersect(std::vector<py::array_t<float>> origins, std::vector<py::array_t<float>> directions, float min, float max)
    {
        //clock_t tStart = clock();
        std::vector<PyBindRay> rays(origins.size());
        std::vector<PyBindRayInfo> infos(origins.size());
        for (int idx = 0; idx < origins.size(); idx++)
        {
            float3 origin(origins[idx].at(0), origins[idx].at(1), origins[idx].at(2));
            float3 direction(directions[idx].at(0), directions[idx].at(1), directions[idx].at(2));
            rays[idx].ray->SetOrigin(origin);
            rays[idx].ray->SetDirection(direction);
            rays[idx].SetMax(max);
            rays[idx].SetMin(min);
            
        }
        //printf("(c++) Multiray construction: %fs\n", (double)(clock() - tStart) / CLOCKS_PER_SEC);
        //tStart = clock();
        for (int idx = 0; idx < origins.size(); idx++)
        {
            bvh->intersect(*rays[idx].ray, *infos[idx].rayInfo);
        }
        //printf("(c++) Multiray intesection time: %fs\n", (double)(clock() - tStart) / CLOCKS_PER_SEC);
        return py::make_tuple(rays, infos);
    };

    py::tuple MultiRayAllIntersects(std::vector<py::array_t<float>> origins, std::vector<py::array_t<float>> directions, float min, float max)
    {
        //clock_t tStart = clock();
        std::vector<PyBindRay> rays(origins.size());
        std::vector<PyBindRayInfo> infos(origins.size());
        for (int idx = 0; idx < origins.size(); idx++)
        {
            float3 origin(origins[idx].at(0), origins[idx].at(1), origins[idx].at(2));
            float3 direction(directions[idx].at(0), directions[idx].at(1), directions[idx].at(2));
            rays[idx].ray->SetOrigin(origin);
            rays[idx].ray->SetDirection(direction);
            rays[idx].SetMax(max);
            rays[idx].SetMin(min);

        }
        //printf("(c++) Multiray construction: %fs\n", (double)(clock() - tStart) / CLOCKS_PER_SEC);
        //tStart = clock();
        for (int idx = 0; idx < origins.size(); idx++)
        {
            bvh->all_intersects(*rays[idx].ray, *infos[idx].rayInfo);
        }
        //printf("(c++) Multiray intesection time: %fs\n", (double)(clock() - tStart) / CLOCKS_PER_SEC);
        return py::make_tuple(rays, infos);
    };

    bool Intersect(PyBindRay& ray, PyBindRayInfo& info) 
    { 
        bool result = bvh->intersect(*ray.ray, *info.rayInfo);
        return result;
    };
    bool AnyIntersect(PyBindRay& ray) 
    { 
        return bvh->any_intersect(*ray.ray); 
    };
    bool AllIntersects(PyBindRay& ray, PyBindRayInfo& info) 
    { 
        bool result = bvh->all_intersects(*ray.ray, *info.rayInfo);
        return result;
    };
    bool PlaneAllIntersects(PyBindPlane& plane, PyBindPlaneInfo& info)
    {
        bool result = bvh->plane_all_intersects(*plane.plane, *info.planeInfo);
        return result;
    }
};
PYBIND11_MODULE(rayTracerPyWrapper, m) {
    m.doc() = R"pbdoc(
        Pybind wrapper for BVH build and traversal
    )pbdoc";
    py::class_<PyBindBVH> bvh(m, "PyBindBVH");
    //bvh.def(py::init());// , SplitMethod > ());
    bvh.def(py::init < std::vector<float>&, SplitMethod, int> ());
    bvh.def("Intersect", &PyBindBVH::Intersect);
    bvh.def("AnyIntersect", &PyBindBVH::AnyIntersect);
    bvh.def("AllIntersects", &PyBindBVH::AllIntersects);
    bvh.def("MultiRayIntersect", &PyBindBVH::MultiRayIntersect);
    bvh.def("MultiRayAllIntersects", &PyBindBVH::MultiRayAllIntersects);
    bvh.def("PlaneAllIntersects", &PyBindBVH::PlaneAllIntersects);
    py::enum_ <SplitMethod> (bvh, "SplitMethod")
        .value("SAH", SplitMethod::SAH)
        .value("HLBVH", SplitMethod::HLBVH)
        .value("Middle", SplitMethod::Middle)
        .value("EqualCounts", SplitMethod::EqualCounts)
        .export_values();
    py::class_<PyBindRay> ray(m, "PyBindRay");
    ray.def(py::init<py::array_t<float>, py::array_t<float>, float, float, int, int>());
    ray.def(py::init<>());
    ray.def("GetDirection", &PyBindRay::GetDirection);
    ray.def("GetOrigin", &PyBindRay::GetOrigin);
    ray.def("GetMin", &PyBindRay::GetMin);
    ray.def("GetMax", &PyBindRay::GetMax);
    ray.def("SetMin", &PyBindRay::SetMin);
    ray.def("SetMax", &PyBindRay::SetMax);
    py::class_<PyBindRayInfo> rayInfo(m, "PyBindRayInfo");
    rayInfo.def(py::init<>());
    rayInfo.def("GetHits", &PyBindRayInfo::GetHits);
    rayInfo.def("SetNormal", &PyBindRayInfo::SetNormal);
    rayInfo.def("GetNormal", &PyBindRayInfo::GetNormal);
    rayInfo.def("AddHit", &PyBindRayInfo::AddHit);
    rayInfo.def("AddClosestHit", &PyBindRayInfo::AddClosestHit);
    rayInfo.def("GetHitsSize", &PyBindRayInfo::GetHitsSize);
    py::class_<PyBindPlane> plane(m, "PyBindPlane");
    plane.def(py::init<py::array_t<float>, py::array_t<float>>());
    plane.def(py::init<>());
    plane.def("GetX", &PyBindPlane::GetX);
    plane.def("GetNormal", &PyBindPlane::GetNormal);
    py::class_<PyBindPlaneInfo> planeInfo(m, "PyBindPlaneInfo");
    planeInfo.def(py::init<>());
    planeInfo.def("GetHits", &PyBindPlaneInfo::GetHits);
    planeInfo.def("AddHit", &PyBindPlaneInfo::AddHit);
    planeInfo.def("GetHitsSize", &PyBindPlaneInfo::GetHitsSize);
    
#ifdef VERSION_INFO
    m.attr("__version__") = VERSION_INFO;
#else
    m.attr("__version__") = "dev";
#endif
}




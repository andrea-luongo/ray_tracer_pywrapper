#include <pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/numpy.h>
#include <BVH.h>
//#include <vector>
#include <time.h>
#include <ppl.h>
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
        std::vector<PyBindRay> rays(origins.size());
        std::vector<PyBindRayInfo> infos(origins.size());
        concurrency::parallel_for(size_t(0), origins.size(), [&](size_t idx)
            {
                float3 origin(origins[idx].at(0), origins[idx].at(1), origins[idx].at(2));
                float3 direction(directions[idx].at(0), directions[idx].at(1), directions[idx].at(2));
                rays[idx].ray->SetOrigin(origin);
                rays[idx].ray->SetDirection(direction);
                rays[idx].SetMax(max);
                rays[idx].SetMin(min);
            });
        concurrency::parallel_for(size_t(0), origins.size(), [&](size_t idx)
            {
            bvh->all_intersects(*rays[idx].ray, *infos[idx].rayInfo);
            });
        return py::make_tuple(rays, infos);
    };

    std::vector<std::vector<py::array_t<float>>> MultiRayAllIntersectsHits(std::vector<py::array_t<float>> origins, std::vector<py::array_t<float>> directions, float min, float max)
    {
        std::vector<PyBindRay> rays(origins.size());
        std::vector<PyBindRayInfo> infos(origins.size());
        concurrency::parallel_for(size_t(0), origins.size(), [&](size_t idx)
            {
                float3 origin(origins[idx].at(0), origins[idx].at(1), origins[idx].at(2));
                float3 direction(directions[idx].at(0), directions[idx].at(1), directions[idx].at(2));
                rays[idx].ray->SetOrigin(origin);
                rays[idx].ray->SetDirection(direction);
                rays[idx].SetMax(max);
                rays[idx].SetMin(min);
            });
        concurrency::parallel_for(size_t(0), origins.size(), [&](size_t idx)
            {
                bvh->all_intersects(*rays[idx].ray, *infos[idx].rayInfo);
            });
        std::vector<std::vector<py::array_t<float>>> hit_points(origins.size());
        try {
            for(int ray_idx = 0; ray_idx < origins.size(); ray_idx++)
                {
                    std::vector<float> t_hits = infos[ray_idx].GetHits();
                    for (int hit_idx = 0; hit_idx < t_hits.size(); hit_idx++)
                    {
                        float3 hit_point = rays[ray_idx].ray->GetOrigin() + t_hits[hit_idx] * rays[ray_idx].ray->GetDirection();
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
                        ptr[0] = hit_point.x;
                        ptr[1] = hit_point.y;
                        ptr[2] = hit_point.z;
                        hit_points[ray_idx].push_back(tmp);
                    }
                }
        }
        catch(...)
        {
            std::cout << "MEGA EXCEPTION" << std::endl;
        }
        return hit_points;
    };

    std::vector<std::vector<py::array_t<float>>> SuperMultiRayAllIntersectsHits(py::array_t<float>& origin, py::array_t<float>& direction, float min, float max, int number_of_rays, py::array_t<float>& bbox_center, float ray_offset, py::array_t<float>& rotation_matrix, py::array_t<float>& inverse_matrix)
    {
        std::vector<PyBindRay> rays(number_of_rays);
        std::vector<PyBindRayInfo> infos(number_of_rays);
        float3 o(origin.at(0), origin.at(1), origin.at(2));
        float3 ray_d(direction.at(0), direction.at(1), direction.at(2));
        float3 b_center(bbox_center.at(0), bbox_center.at(1), bbox_center.at(2));
        py::buffer_info rot_buf = rotation_matrix.request();
        py::buffer_info inv_buf = inverse_matrix.request();
        float* rot_ptr = (float*)rot_buf.ptr;
        float* inv_ptr = (float*)inv_buf.ptr;
        std::array<float, 16> rot_el = {rot_ptr[0], rot_ptr[4], rot_ptr[8], rot_ptr[12], rot_ptr[1], rot_ptr[5], rot_ptr[9], rot_ptr[13], rot_ptr[2], rot_ptr[6], rot_ptr[10], rot_ptr[14], rot_ptr[3], rot_ptr[7], rot_ptr[11], rot_ptr[15]};
        std::array<float, 16> inv_el = {inv_ptr[0], inv_ptr[4], inv_ptr[8], inv_ptr[12], inv_ptr[1], inv_ptr[5], inv_ptr[9], inv_ptr[13], inv_ptr[2], inv_ptr[6], inv_ptr[10], inv_ptr[14], inv_ptr[3], inv_ptr[7], inv_ptr[11], inv_ptr[15]};
        Matrix4x4 rot_matrix(rot_el);
        Matrix4x4 inv_matrix(inv_el);
        //float3 ray_d = inv_matrix * (rot_matrix * d);
        concurrency::parallel_for(int(0), number_of_rays, [&](int idx)
            {
                //float4 ray_o(o.x + ray_offset * idx, o.y, o.z, 1.0f);
                float3 ray_o = inv_matrix * (rot_matrix * float4(o.x + ray_offset * idx, o.y, o.z, 1.0f) + float4(b_center.x, 0, b_center.z, 0));
    /*            if (idx == 0) {
                    std::cout << ray_o << std::endl;
                    std::cout << ray_d << std::endl;
                    std::cout << inv_matrix << std::endl;
                    std::cout << rot_matrix << std::endl;
                }*/
                rays[idx].ray->SetOrigin(ray_o);
                rays[idx].ray->SetDirection(ray_d);
                rays[idx].SetMax(max);
                rays[idx].SetMin(min);
            });
        concurrency::parallel_for(int(0), number_of_rays, [&](int idx)
            {
                bvh->all_intersects(*rays[idx].ray, *infos[idx].rayInfo);
            });
        std::vector<std::vector<py::array_t<float>>> hit_points(number_of_rays);
        try {
            for (int ray_idx = 0; ray_idx < number_of_rays; ray_idx++)
            {
                std::vector<float> t_hits = infos[ray_idx].GetHits();
                for (int hit_idx = 0; hit_idx < t_hits.size(); hit_idx++)
                {
                    float3 hit_point = rays[ray_idx].ray->GetOrigin() + t_hits[hit_idx] * rays[ray_idx].ray->GetDirection();
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
                    ptr[0] = hit_point.x;
                    ptr[1] = hit_point.y;
                    ptr[2] = hit_point.z;
                    hit_points[ray_idx].push_back(tmp);
                }
            }
        }
        catch (...)
        {
            std::cout << "MEGA EXCEPTION" << std::endl;
        }
        return hit_points;
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
    std::vector<py::array_t<float>> PlaneAllIntersectsHits(PyBindPlane& plane, PyBindPlaneInfo& info)
    {
        bool result = bvh->plane_all_intersects(*plane.plane, *info.planeInfo);
        return info.GetHits();
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
    bvh.def("MultiRayAllIntersectsHits", &PyBindBVH::MultiRayAllIntersectsHits);
    bvh.def("SuperMultiRayAllIntersectsHits", &PyBindBVH::SuperMultiRayAllIntersectsHits);
    bvh.def("PlaneAllIntersects", &PyBindBVH::PlaneAllIntersects);
    bvh.def("PlaneAllIntersectsHits", &PyBindBVH::PlaneAllIntersectsHits);
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




#include <pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/numpy.h>
#include <BVH.h>
#include <Contour.h>
//#include <vector>
#include <time.h>
#include <ppl.h>
namespace py = pybind11;

enum class PrimitiveType { SEGMENT, SPHERE, TRIANGLE, CONTOUR };

inline Matrix4x4 reinterpret_matrix(py::array_t<float>& pd)
{
    py::buffer_info pd_buf = pd.request();
    float* pd_ptr = (float*)pd_buf.ptr;
    std::array<float, 16> el = { pd_ptr[0], pd_ptr[4], pd_ptr[8], pd_ptr[12], pd_ptr[1], pd_ptr[5], pd_ptr[9], pd_ptr[13], pd_ptr[2], pd_ptr[6], pd_ptr[10], pd_ptr[14], pd_ptr[3], pd_ptr[7], pd_ptr[11], pd_ptr[15] };
    return Matrix4x4(el);
}

inline py::array_t<float> reinterpret_float3(float3 f)
{
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
    ptr[0] = f.x;
    ptr[1] = f.y;
    ptr[2] = f.z;
    return tmp;
}

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
        return reinterpret_float3(x);
    };
    py::array_t<float> GetNormal() const {
        float3 n = plane->GetNormal();
        return reinterpret_float3(n);
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
            result.push_back(reinterpret_float3(value));
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
        return reinterpret_float3(d);
    };
    py::array_t<float> GetOrigin() const { 
        float3 o = ray->GetOrigin();
        return reinterpret_float3(o);
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

class PyContour{

public:
    Contour* contour= nullptr;
public:

    PyContour(std::vector<float>& vertices)
    {
        try {
            std::cout << vertices.size() << std::endl;
            std::vector<std::shared_ptr<Segment>> primitives((int)(vertices.size() / 6));
            for (int i = 0; i < (int)(vertices.size() / 6); i++)
            {
                float3 p0(vertices[i * 6], vertices[i * 6 + 1], vertices[i * 6 + 2]);
                float3 p1(vertices[i * 6 + 3], vertices[i * 6 + 4], vertices[i * 6 + 5]);
                //std::shared_ptr<Primitive> primitive = std::shared_ptr<Segment>(new Segment(p0, p1));
                std::cout << p0 << ' ' << p1 << std::endl;
                primitives[i] = std::shared_ptr<Segment>(new Segment(p0, p1));
            }
            std::cout << "segments created" << std::endl;
            contour = new Contour(primitives);
        }
        catch (const std::exception & exc)
        {
            std::cerr << "exception" << std::endl;
            //std::cout << vertices.size() << std::endl;
            //for (int i = 0; i < vertices.size(); i++)
            //{
            //    /*float3 p0(vertices[i * 6], vertices[i * 6 + 1], vertices[i * 6 + 2]);
            //    float3 p1(vertices[i * 6 + 3], vertices[i * 6 + 4], vertices[i * 6 + 5]);
            //}
            // catch anything thrown within try block that derives from std::exception
        }
    };

    ~PyContour() {
        //delete bvh;
    }

};

class PyContourTree {

public:
    ContourTree* contour_tree = nullptr;
public:

    PyContourTree(std::vector<PyContour>& py_contours)
    {
        std::vector<std::shared_ptr<Contour>> contours(py_contours.size());
        for (int i = 0; i < contours.size(); i++)
        {
            contours[i] = std::shared_ptr<Contour>(py_contours[i].contour);
        }
        contour_tree = new ContourTree(contours);
    };

    ~PyContourTree() {
        //delete bvh;
    }

};

class PyBindBVH {

private:
    BVH* bvh = nullptr;
public:
    //PyBindBVH() { };
    PyBindBVH(std::vector<float>& vertices, SplitMethod splitMethod, int maxPrimsInNode=255, PrimitiveType primitive_type=PrimitiveType::TRIANGLE)
    {
        std::vector<std::shared_ptr<Primitive>> primitives;
        if (primitive_type == PrimitiveType::TRIANGLE)
        {
            for (int i = 0; i < (int)(vertices.size() / 9); i++)
            {
                float3 p0(vertices[i*9], vertices[i*9+1], vertices[i*9+2]);
                float3 p1(vertices[i*9+3], vertices[i*9+4], vertices[i*9+5]);
                float3 p2(vertices[i*9+6], vertices[i*9+7], vertices[i*9+8]);
                std::shared_ptr<Primitive> primitive = std::shared_ptr<Triangle>(new Triangle(p0, p1, p2));
                primitives.push_back(primitive);
            }
        }
        else if (primitive_type == PrimitiveType::SEGMENT)
        {

        }

        bvh = new BVH(primitives, splitMethod, maxPrimsInNode);
    }

    ~PyBindBVH() {
        //delete bvh;
    }

    py::tuple MultiRayIntersect(std::vector<py::array_t<float>> origins, std::vector<py::array_t<float>> directions, float min, float max)
    {
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
        for (int idx = 0; idx < origins.size(); idx++)
        {
            bvh->intersect(*rays[idx].ray, *infos[idx].rayInfo);
        }
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

    std::vector<std::vector<py::array_t<float>>> MultiRayAllIntersectsHits(std::vector<py::array_t<float>> origins, std::vector<py::array_t<float>> directions)
    {
        std::vector<PyBindRay> rays(origins.size());
        std::vector<PyBindRayInfo> infos(origins.size());
        float min = 0;
        float max = std::numeric_limits<float>::infinity();
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
                        hit_points[ray_idx].push_back(reinterpret_float3(hit_point));
                    }
                }
        }
        catch(...)
        {
            std::cout << "Ray Intersection Exception" << std::endl;
        }
        return hit_points;
    };

    std::vector<std::vector<py::array_t<float>>> MultiRayAllIntersectsTransformHits(py::array_t<float>& origin, py::array_t<float>& direction, int number_of_rays, py::array_t<float>& bbox_center, float ray_offset, py::array_t<float>& rotation_matrix, py::array_t<float>& inverse_matrix)
    {
        std::vector<PyBindRay> rays(number_of_rays);
        std::vector<PyBindRayInfo> infos(number_of_rays);
        float3 o(origin.at(0), origin.at(1), origin.at(2));
        float3 ray_d(direction.at(0), direction.at(1), direction.at(2));
        float3 b_center(bbox_center.at(0), bbox_center.at(1), bbox_center.at(2));
        Matrix4x4 rot_matrix = reinterpret_matrix(rotation_matrix);
        Matrix4x4 inv_matrix = reinterpret_matrix(inverse_matrix);
        float min = 0;
        float max = std::numeric_limits<float>::infinity();
        concurrency::parallel_for(int(0), number_of_rays, [&](int idx)
            {
                float3 ray_o = inv_matrix * (rot_matrix * float4(o.x + ray_offset * idx, o.y, o.z, 1.0f) + float4(b_center.x, 0, b_center.z, 0));
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
                    hit_points[ray_idx].push_back(reinterpret_float3(hit_point));
                }
            }
        }
        catch (...)
        {
            std::cout << "Ray Intersection Exception" << std::endl;
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
    py::enum_ <PrimitiveType>(m, "PrimitiveType")
        .value("SEGMENT", PrimitiveType::SEGMENT)
        .value("SPHERE", PrimitiveType::SPHERE)
        .value("TRIANGLE", PrimitiveType::TRIANGLE)
        .value("CONTOUR", PrimitiveType::CONTOUR)
        .export_values();
    py::class_<PyBindBVH> bvh(m, "PyBindBVH");
    //bvh.def(py::init());// , SplitMethod > ());
    bvh.def(py::init < std::vector<float>&, SplitMethod, int> ());
    bvh.def("Intersect", &PyBindBVH::Intersect);
    bvh.def("AnyIntersect", &PyBindBVH::AnyIntersect);
    bvh.def("AllIntersects", &PyBindBVH::AllIntersects);
    bvh.def("MultiRayIntersect", &PyBindBVH::MultiRayIntersect);
    bvh.def("MultiRayAllIntersects", &PyBindBVH::MultiRayAllIntersects);
    bvh.def("MultiRayAllIntersectsHits", &PyBindBVH::MultiRayAllIntersectsHits);
    bvh.def("MultiRayAllIntersectsTransformHits", &PyBindBVH::MultiRayAllIntersectsTransformHits);
    bvh.def("PlaneAllIntersects", &PyBindBVH::PlaneAllIntersects);
    bvh.def("PlaneAllIntersectsHits", &PyBindBVH::PlaneAllIntersectsHits);
    py::enum_ <SplitMethod> (bvh, "SplitMethod")
        .value("SAH", SplitMethod::SAH)
        .value("HLBVH", SplitMethod::HLBVH)
        .value("Middle", SplitMethod::Middle)
        .value("EqualCounts", SplitMethod::EqualCounts)
        .export_values();


    py::class_<PyContour> contour(m, "PyContour");
    contour.def(py::init<std::vector<float>&>());

    py::class_<PyContourTree> contourtree(m, "PyContourTree");
    contourtree.def(py::init<std::vector<PyContour>&>());


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




#include <pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/numpy.h>
#include <BVH.h>
#include <Contour.h>
//#include <vector>
#include <time.h>
#include <ppl.h>
#include "pystructs.h"
namespace py = pybind11;


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


PyBindPlane::PyBindPlane()
{
    plane = new Plane();
}
PyBindPlane::PyBindPlane(py::array_t<float> x, py::array_t<float> n)
{
    float3 x0(x.at(0), x.at(1), x.at(2));
    float3 normal(n.at(0), n.at(1), n.at(2));
    plane = new Plane(x0, normal);
}
py::array_t<float> PyBindPlane::GetX() const {
    float3 x = plane->GetX();
    return reinterpret_float3(x);
};
py::array_t<float> PyBindPlane::GetNormal() const {
    float3 n = plane->GetNormal();
    return reinterpret_float3(n);
};


PyBindPlaneInfo::PyBindPlaneInfo()
{
    planeInfo = new PlaneIntersectionInfo();
};
std::vector<py::array_t<float>> PyBindPlaneInfo::GetHits()
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
void PyBindPlaneInfo::AddHit(py::array_t<float> t)
{ 
    float3 x0(t.at(0), t.at(1), t.at(2));
    planeInfo->AddHit(x0);
};
int PyBindPlaneInfo::GetHitsSize() { return planeInfo->GetHitsSize(); };



PyBindRay::PyBindRay()
{
    float3 origin(0, 0, 0);
    float3 direction(0, 0, 1);
    ray = new Ray(origin, direction, 0, std::numeric_limits<float>::max(), 0, 0);
}

PyBindRay::PyBindRay(py::array_t<float> o, py::array_t<float> dir, float min, float max, int d, int s)
{
    float3 origin(o.at(0), o.at(1), o.at(2));
    float3 direction(dir.at(0), dir.at(1), dir.at(2));
    ray = new Ray(origin, direction, min, max, d, s);
};

py::array_t<float> PyBindRay::GetDirection() const {
    float3 d = ray->GetDirection();
    return reinterpret_float3(d);
};
py::array_t<float> PyBindRay::GetOrigin() const {
    float3 o = ray->GetOrigin();
    return reinterpret_float3(o);
};
float PyBindRay::GetMin() const { return ray->GetMin(); };
float PyBindRay::GetMax() const { return ray->GetMax(); };
void PyBindRay::SetMin(float t) { ray->SetMin(t); };
void PyBindRay::SetMax(float t) { ray->SetMax(t); };


PyBindRayInfo::PyBindRayInfo()
{
    rayInfo = new RayIntersectionInfo();
};
std::vector<float> PyBindRayInfo::GetHits() { return *rayInfo->GetHits(); };
void PyBindRayInfo::SetNormal(float3 n) { rayInfo->SetNormal(n); };
float3 PyBindRayInfo::GetNormal() { return rayInfo->GetNormal(); };
void PyBindRayInfo::AddHit(float t) { rayInfo->AddHit(t); };
void PyBindRayInfo::AddClosestHit(float t) { rayInfo->AddClosestHit(t); };
int PyBindRayInfo::GetHitsSize() { return rayInfo->GetHitsSize(); };




    //PyBindBVH() { };
PyBindBVH::PyBindBVH(std::vector<float>& vertices, SplitMethod splitMethod, int maxPrimsInNode, PrimitiveType primitive_type)
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

    bvh = std::make_shared<BVH>(primitives, splitMethod, maxPrimsInNode);
}

py::array_t<float> PyBindBVH::GetBBoxMin()
{
    float3 x = bvh->getBVHBBox().GetpMin();
    return reinterpret_float3(x);
}

py::array_t<float> PyBindBVH::GetBBoxMax()
{
    float3 x = bvh->getBVHBBox().GetpMax();
    return reinterpret_float3(x);
}

PyBindBVH::PyBindBVH(std::shared_ptr<BVH> b)
{
    bvh = b;
}

py::tuple PyBindBVH::MultiRayIntersect(std::vector<py::array_t<float>> origins, std::vector<py::array_t<float>> directions, float min, float max)
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

py::tuple PyBindBVH::MultiRayAllIntersects(std::vector<py::array_t<float>> origins, std::vector<py::array_t<float>> directions, float min, float max)
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

std::vector<std::vector<py::array_t<float>>> PyBindBVH::MultiRayAllIntersectsHits(std::vector<py::array_t<float>> origins, std::vector<py::array_t<float>> directions)
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

std::vector<std::vector<py::array_t<float>>> PyBindBVH::MultiRayAllIntersectsTransformHits(py::array_t<float>& origin, py::array_t<float>& direction, int number_of_rays, py::array_t<float>& bbox_center, float ray_offset, py::array_t<float>& rotation_matrix, py::array_t<float>& inverse_matrix)
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

bool PyBindBVH::Intersect(PyBindRay& ray, PyBindRayInfo& info)
{ 
    bool result = bvh->intersect(*ray.ray, *info.rayInfo);
    return result;
};
bool PyBindBVH::AnyIntersect(PyBindRay& ray)
{ 
    return bvh->any_intersect(*ray.ray); 
};
bool PyBindBVH::AllIntersects(PyBindRay& ray, PyBindRayInfo& info)
{ 
    bool result = bvh->all_intersects(*ray.ray, *info.rayInfo);
    return result;
};
bool PyBindBVH::PlaneAllIntersects(PyBindPlane& plane, PyBindPlaneInfo& info)
{
    bool result = bvh->plane_all_intersects(*plane.plane, *info.planeInfo);
    return result;
}
std::vector<py::array_t<float>> PyBindBVH::PlaneAllIntersectsHits(PyBindPlane& plane, PyBindPlaneInfo& info)
{
    bool result = bvh->plane_all_intersects(*plane.plane, *info.planeInfo);
    return info.GetHits();
}
std::vector<PyBindContour> PyBindBVH::PlaneAllIntersectsContours(PyBindPlane& plane, PyBindPlaneInfo& info, py::array_t<float>& transformation_matrix, float const geometry_scaling, float const segment_min_length, bool verbose)
{

    Matrix4x4 tr_matrix = reinterpret_matrix(transformation_matrix);
    Matrix4x4 tr_matrix_transposed = tr_matrix.Transpose();
    bool result = bvh->plane_all_intersects(*plane.plane, *info.planeInfo);

    std::vector<float3> hits = *(info.planeInfo->GetHits());
    if (verbose)
    {
        std::cout << "%hits: " << hits.size() << std::endl;
    }
    std::vector<float3> transformed_hits(hits.size());
    for (int idx = 0; idx < hits.size(); idx++)
    {
        float4 t_hit = (float4(hits[idx][0], hits[idx][1], hits[idx][2], 1) * tr_matrix_transposed) * geometry_scaling;
        if (geometry_scaling != 1)
            transformed_hits[idx] = float3(int(t_hit[0]), int(t_hit[1]), int(t_hit[2]));
        else
            transformed_hits[idx] = float3(t_hit[0], t_hit[1], t_hit[2]);

    }
    if (verbose)
    {
        std::cout << "%Transformed hits" << std::endl;
    }
    std::vector<std::shared_ptr<Segment>> segment_primitives;
    if (verbose)
    {
        std::cout << "primitives" << "=[";
    }
    for (int i = 0; i < (int)(transformed_hits.size() / 2); i++)
    {
        float3 p0 = transformed_hits[i * 2];
        float3 p1 = transformed_hits[i * 2 + 1];
        if (p0 == float3(-24869, 429300, 24869) || p1 == float3(-24869, 429300, 24869))
        {
            std::cout << "OCIO" << p0 << " " << p1 << " " << float3::length(p0 - p1) <<std::endl;
        }
        if (float3::length(p0 - p1) == 0)
        {
            continue;
        }
        if (verbose)
        {
            std::cout << "[" << p0 << "]\n[" << p1 << "]" << std::endl;

        }
        segment_primitives.push_back(std::shared_ptr<Segment>(new Segment(p0, p1)));
    }
    if (verbose)
    {
        std::cout << "];" << std::endl;;
        std::cout << "%created primitives " << segment_primitives.size() << std::endl;
    }

    float epsilon = 0.001 * geometry_scaling;
    float alignment_epsilon = 1e-3;
    bool remove_aligned_segments = true;
    bool remove_short_segments = true;
    if (verbose)
    {
        std::cout << "%epsilon " << epsilon << " remove aligned " << remove_aligned_segments << " " << alignment_epsilon << " remove short " << remove_short_segments << " " << segment_min_length << std::endl;
    }
    auto sorted_segments = Segment::SortSegments(segment_primitives, epsilon, remove_aligned_segments, alignment_epsilon, remove_short_segments, segment_min_length);
    
    if (verbose)
    {
        int sorted_primitives_counter = 0;
        for (auto sc : sorted_segments)
            sorted_primitives_counter += sc.size();
        std::cout << "%sorted primitives: " << sorted_primitives_counter << std::endl;

        int sc_counter = 0;
        for (auto c : sorted_segments)
        {
            std::cout << "sc" << sc_counter++ << "=[";
            for (auto s : c)
            {
                std::cout << "[" << s->v0 << "]\n[" << s->v1 << "]" << std::endl;
            }
            std::cout << "];" << std::endl;;
        }
        std::cout << "sc_contours={";
        for (int idx = 0; idx < sc_counter; idx++)
            std::cout << "sc" << idx << ", ";
        std::cout << "};" << std::endl;;
    }

    std::vector<PyBindContour> sorted_contours;
    int discarded_contours = 0;
    int contour_counter = 0;
    for (int i = 0; i < sorted_segments.size(); i++) {
        Contour c(sorted_segments[i], plane.plane->GetNormal());
    
        if (!c.is_valid) {
            if (verbose)
            {
                std::cout << "%Discarded" << std::endl;
            }
            discarded_contours++;
            continue;
        }

        if (remove_short_segments)
        {
            c.RemoveShortSegments(segment_min_length);
            if (!c.is_valid) {
                if (verbose)
                {
                    std::cout << "%Discarded" << std::endl;
                }
                discarded_contours++;
                continue;
            }
        }
        if (remove_aligned_segments)
        {
            c.RemoveAlignedSegments(alignment_epsilon);
            if (!c.is_valid) {
                if (verbose)
                {
                    std::cout << "%Discarded" << std::endl;
                }
                discarded_contours++;
                continue;
            }
        }

        if (verbose)
        {
            std::cout << "%REMOVED ALIGNED CONTOUR" << std::endl;
            std::cout << "cc" << contour_counter++ << "=[";
            for (auto ss : c.segments)
                std::cout << "[" << ss->v0 << "]\n[" << ss->v1 << "]" << std::endl;
            std::cout << "];" << std::endl;
        }

        sorted_contours.push_back(PyBindContour(c));
    }
    if (verbose)
    {
        std::cout << "cleaned_contours={";
        for (int idx = 0; idx < contour_counter; idx++)
            std::cout << "cc" << idx << ", ";
        std::cout << "};" << std::endl;;
        std::cout << "%Discarded contours: " << discarded_contours << std::endl;
    }
    return sorted_contours;
}



PyBindContour::PyBindContour(std::vector<float>& vertices, py::array_t<float> n)
{
    bool verbose = false;
    float3 normal(n.at(0), n.at(1), n.at(2));
    try {
        std::vector<std::shared_ptr<Segment>> primitives;
        for (int i = 0; i < (int)(vertices.size() / 6); i++)
        {
            float3 p0(vertices[i * 6], vertices[i * 6 + 1], vertices[i * 6 + 2]);
            float3 p1(vertices[i * 6 + 3], vertices[i * 6 + 4], vertices[i * 6 + 5]);
            if (float3::length(p0 - p1) == 0)
            {
                continue;
            }
            primitives.push_back(std::shared_ptr<Segment>(new Segment(p0, p1)));
        }
        contour = std::make_shared<Contour>(primitives, normal);
    }
    catch (const std::exception& exc)
    {
        std::cerr << "exception" << std::endl;
    }
};

PyBindContour::PyBindContour(Contour &c)
{
    contour = std::make_shared<Contour>(c);
}

bool PyBindContour::IsValid()
{
    return contour->CheckValidity();
}

py::tuple PyBindContour::IsContained(PyBindContour& contour_b)
{
    float t_hit;
    bool result = contour->IsContained(*contour_b.contour, t_hit);
    return py::make_tuple(result, t_hit);
}

py::tuple PyBindContour::Contains(PyBindContour& contour_b)
{
    float t_hit;
    bool result = contour->Contains(*contour_b.contour, t_hit);
    return py::make_tuple(result, t_hit);
}
py::tuple PyBindContour::EvaluateContoursRelationship(PyBindContour& contour_b)
{
    float t_hit;
    int result = Contour::EvaluateContoursRelationship(*contour, *contour_b.contour, t_hit);
    return py::make_tuple(result, t_hit);
}

py::array_t<float> PyBindContour::GetBBoxMin()
{
    float3 x = contour->GetBBox().GetpMin();
    return reinterpret_float3(x);
}
    
py::array_t<float> PyBindContour::GetBBoxMax()
{
    float3 x = contour->GetBBox().GetpMax();
    return reinterpret_float3(x);
}

bool PyBindContour::Intersect(PyBindRay& ray, PyBindRayInfo& info)
{
    bool result = contour->Intersect(*ray.ray, *info.rayInfo);
    return result;
}

bool PyBindContour::AnyIntersect(PyBindRay& ray)
{
    bool result = contour->AnyIntersect(*ray.ray);
    return result;
}

bool PyBindContour::AllIntersects(PyBindRay& ray, PyBindRayInfo& info)
{
    bool result = contour->AllIntersect(*ray.ray, *info.rayInfo);
    return result;
}

std::vector<std::vector<py::array_t<float>>> PyBindContour::MultiRayAllIntersects(float laser_width_microns, float density, float overlap, float rot_angle, bool verbose)
{

    std::vector<std::vector<float3>> individual_hit_points = contour->MultiRayAllIntersects(laser_width_microns, density, overlap, rot_angle, verbose);

    if (verbose)
    {
        std::cout << "REINTERPRETING INTERSECTIONS" << std::endl;
    }
    std::vector<std::vector<py::array_t<float>>> reinterpreted_individual_hit_points(individual_hit_points.size());
    try {

        for (int ray_idx = 0; ray_idx < individual_hit_points.size(); ray_idx++)
        {
            std::vector<float3> ray_hits = individual_hit_points[ray_idx];
            for (int hit_idx = 0; hit_idx < ray_hits.size(); hit_idx++)
            {
                reinterpreted_individual_hit_points[ray_idx].push_back(reinterpret_float3(ray_hits[hit_idx]));
            }
        }
    }
    catch (...)
    {
        std::cout << "Ray Intersection Exception" << std::endl;
    }
    return reinterpreted_individual_hit_points;
};

py::tuple PyBindContour::OffsetContour(float offset)
{
    Contour offset_contour;
    bool result = contour->OffsetContour(offset, offset_contour);

    //return PyBindContour(offset_contour);
    return py::make_tuple(result, PyBindContour(offset_contour));
}

std::vector<PyBindContour> PyBindContour::RemoveSelfIntersections()
{
    std::vector<std::shared_ptr<Contour>> new_contours;
    std::vector<std::shared_ptr<ContourSelfIntersectionPoint>> intersection_points;
    std::map<int, std::vector<std::shared_ptr<ContourSelfIntersectionPoint>>> intersections_dict;
    bool is_self_intersecting = contour->FindSelfIntersections(intersection_points, intersections_dict);
    bool self_intersect = contour->RemoveSelfIntersections(new_contours, intersection_points, intersections_dict, contour->contour_orientation);
    std::vector<PyBindContour> result;
    if (self_intersect)
    {
        for(auto c : new_contours)
            result.push_back(PyBindContour(*c));;
    }
    else
    {
        result.push_back(PyBindContour(*contour));
    }

    return result;
}


std::vector<py::array_t<float>> PyBindContour::GetSegments()
{

    std::vector<py::array_t<float>> result(contour->segments.size() * 2);
    for (int i = 0; i < contour->segments.size(); i++)
    {
        float3 p0 = (*contour->segments[i])[0];
        float3 p1 = (*contour->segments[i])[1];
        result[i*2] = (reinterpret_float3(p0));
        result[i*2+1] = (reinterpret_float3(p1));
    }
    return result;
}


PyBindContourTree::PyBindContourTree(std::vector<PyBindContour>& py_contours)
{
    std::vector<std::shared_ptr<Contour>> contours;
    for (int i = 0; i < py_contours.size(); i++)
    {
        if (py_contours[i].IsValid())
            contours.push_back(py_contours[i].contour);
    }
    contour_tree = std::make_shared<ContourTree>(contours);
};

PyBindContourTree::PyBindContourTree(ContourTree& c)
{
    contour_tree = std::make_shared<ContourTree>(c);
}

py::array_t<float> PyBindContourTree::GetBBoxMin()
{
    float3 x = contour_tree->GetBBox().GetpMin();
    return reinterpret_float3(x);
}

py::array_t<float> PyBindContourTree::GetBBoxMax()
{
    float3 x = contour_tree->GetBBox().GetpMax();
    return reinterpret_float3(x);
}

bool PyBindContourTree::Intersect(PyBindRay& ray, PyBindRayInfo& info)
{
    bool result = contour_tree->Intersect(*ray.ray, *info.rayInfo);
    return result;
}
    
bool PyBindContourTree::AnyIntersect(PyBindRay& ray)
{
    bool result = contour_tree->AnyIntersect(*ray.ray);
    return result;
}
    
bool PyBindContourTree::AllIntersects(PyBindRay& ray, PyBindRayInfo& info)
{
    bool result = contour_tree->AllIntersect(*ray.ray, *info.rayInfo);
    return result;
}

std::vector< std::vector<std::vector<py::array_t<float>>>> PyBindContourTree::MultiRayAllIntersects(float laser_width_microns, float density, float overlap, float rot_angle, bool verbose)
{

    if (verbose)
    {
        std::cout << "laser width " << laser_width_microns << std::endl;
        std::cout << "density " << density << std::endl;
        std::cout << "overlap " << overlap << std::endl;
        std::cout << "rot_angle " << rot_angle << std::endl;

        std::cout << "COMPUTING INTERSECTIONS" << std::endl;
    }

    std::vector < std::vector<std::vector<float3>>> individual_hit_points = contour_tree->MultiRayAllIntersects(laser_width_microns, density, overlap, rot_angle, verbose);

    if (verbose)
    {
        std::cout << "REINTERPRETING INTERSECTIONS" << std::endl;
    }
    std::vector <std::vector<std::vector<py::array_t<float>>>> reinterpreted_contour_tree_hit_points(individual_hit_points.size());
    try {
        for (int bvh_idx = 0; bvh_idx < individual_hit_points.size(); bvh_idx++)
        {
            auto bvh_hits = individual_hit_points[bvh_idx];
            std::vector<std::vector<py::array_t<float>>> contour_hit_points(bvh_hits.size());
            for (int ray_idx = 0; ray_idx < bvh_hits.size(); ray_idx++)
            {
                std::vector<float3> ray_hits = bvh_hits[ray_idx];
                for (int hit_idx = 0; hit_idx < ray_hits.size(); hit_idx++)
                {
                    contour_hit_points[ray_idx].push_back(reinterpret_float3(ray_hits[hit_idx]));
                }
            }
            reinterpreted_contour_tree_hit_points[bvh_idx] = contour_hit_points;
        }
    }
    catch (...)
    {
        std::cout << "Ray Intersection Exception" << std::endl;
    }
    return reinterpreted_contour_tree_hit_points;
};

py::tuple PyBindContourTree::OffsetContourTree(float offset, bool verbose)
{
    ContourTree tree;
    bool succesful_offset = contour_tree->OffsetContourTree(offset, tree, verbose);
    //return PyBindContourTree(result);
    //std::cout << "succesful offset " << succesful_offset << std::endl;
    PyBindContourTree pytree(tree);
    return py::make_tuple(succesful_offset, pytree);
}

std::vector<py::array_t<float>> PyBindContourTree::GetAllSegments()
{

    std::vector<py::array_t<float>> result;
    for (auto c : contour_tree->contours)
    {
        for (int i = 0; i < c->segments.size(); i++)
        {
            float3 p0 = (*c->segments[i])[0];
            float3 p1 = (*c->segments[i])[1];
            result.push_back(reinterpret_float3(p0));
            result.push_back(reinterpret_float3(p1));
        }

    }
    return result;
}


PYBIND11_MODULE(rayTracerPyWrapper, m) {
    m.doc() = R"pbdoc(
        Pybind wrapper for BVH build and traversal
    )pbdoc";

    py::enum_ <PrimitiveType>(m, "PrimitiveType")
        .value("SEGMENT", PrimitiveType::SEGMENT)
        .value("SPHERE", PrimitiveType::SPHERE)
        .value("TRIANGLE", PrimitiveType::TRIANGLE)
        .value("INT_TRIANGLE", PrimitiveType::INT_TRIANGLE)
        .value("CONTOUR", PrimitiveType::CONTOUR)
        .export_values();

    py::class_<PyBindBVH> bvh(m, "PyBindBVH");
    //bvh.def(py::init());// , SplitMethod > ());
    bvh.def(py::init < std::vector<float>&, SplitMethod, int> ());
    bvh.def(py::init < std::vector<float>&, SplitMethod, int, PrimitiveType> ());
    //bvh.def(py::init < std::vector<int32_t>&, SplitMethod, int, PrimitiveType> ());
    bvh.def("GetBBoxMin", &PyBindBVH::GetBBoxMin);
    bvh.def("GetBBoxMax", &PyBindBVH::GetBBoxMax);
    bvh.def("Intersect", &PyBindBVH::Intersect);
    bvh.def("AnyIntersect", &PyBindBVH::AnyIntersect);
    bvh.def("AllIntersects", &PyBindBVH::AllIntersects);
    bvh.def("MultiRayIntersect", &PyBindBVH::MultiRayIntersect);
    bvh.def("MultiRayAllIntersects", &PyBindBVH::MultiRayAllIntersects);
    bvh.def("MultiRayAllIntersectsHits", &PyBindBVH::MultiRayAllIntersectsHits);
    bvh.def("MultiRayAllIntersectsTransformHits", &PyBindBVH::MultiRayAllIntersectsTransformHits);
    bvh.def("PlaneAllIntersects", &PyBindBVH::PlaneAllIntersects);
    bvh.def("PlaneAllIntersectsHits", &PyBindBVH::PlaneAllIntersectsHits);
    bvh.def("PlaneAllIntersectsContours", &PyBindBVH::PlaneAllIntersectsContours);
    py::enum_ <SplitMethod>(bvh, "SplitMethod")
        .value("SAH", SplitMethod::SAH)
        .value("HLBVH", SplitMethod::HLBVH)
        .value("Middle", SplitMethod::Middle)
        .value("EqualCounts", SplitMethod::EqualCounts)
        .export_values();

    py::class_<PyBindContour> contour(m, "PyBindContour");
    contour.def(py::init<std::vector<float>&, py::array_t<float>>());
    contour.def("IsValid", &PyBindContour::IsValid);
    contour.def("IsContained", &PyBindContour::IsContained);
    contour.def("Contains", &PyBindContour::Contains);
    contour.def("EvaluateContoursRelationship", &PyBindContour::EvaluateContoursRelationship);
    contour.def("GetBBoxMin", &PyBindContour::GetBBoxMin);
    contour.def("GetBBoxMax", &PyBindContour::GetBBoxMax);
    contour.def("AllIntersects", &PyBindContour::AllIntersects);
    contour.def("AnyIntersect", &PyBindContour::AnyIntersect);
    contour.def("Intersect", &PyBindContour::Intersect);
    contour.def("MultiRayAllIntersects", &PyBindContour::MultiRayAllIntersects);
    contour.def("OffsetContour", &PyBindContour::OffsetContour);
    contour.def("RemoveSelfIntersections", &PyBindContour::RemoveSelfIntersections);
    contour.def("GetSegments", &PyBindContour::GetSegments);

    py::class_<PyBindContourTree> contourtree(m, "PyBindContourTree");
    contourtree.def(py::init<std::vector<PyBindContour>&>());
    //contourtree.def("GetTreeInternalPyBVHs", &PyBindContourTree::GetTreeInternalPyBVHs);
    contourtree.def("AllIntersects", &PyBindContourTree::AllIntersects);
    contourtree.def("AnyIntersect", &PyBindContourTree::AnyIntersect);
    contourtree.def("Intersect", &PyBindContourTree::Intersect);
    contourtree.def("OffsetContourTree", &PyBindContourTree::OffsetContourTree);
    contourtree.def("MultiRayAllIntersects", &PyBindContourTree::MultiRayAllIntersects);
    contourtree.def("GetBBoxMin", &PyBindContourTree::GetBBoxMin);
    contourtree.def("GetBBoxMax", &PyBindContourTree::GetBBoxMax);
    contourtree.def("GetAllSegments", &PyBindContourTree::GetAllSegments);

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




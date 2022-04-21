#include "structs.h"
#include <mutex>
#include <pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/numpy.h>
#include <BVH.h>
#include <Contour.h>
//#include <vector>
#include <time.h>
#include <ppl.h>
namespace py = pybind11;
enum class PrimitiveType { SEGMENT, SPHERE, TRIANGLE, INT_TRIANGLE, CONTOUR };

class PyBindPlane
{
public:
    Plane* plane = nullptr;
public:
    PyBindPlane();
    
    PyBindPlane(py::array_t<float> x, py::array_t<float> n);
    py::array_t<float> GetX() const;
    py::array_t<float> GetNormal() const;
};

class PyBindPlaneInfo
{
public:
    PlaneIntersectionInfo* planeInfo = nullptr;
public:
    PyBindPlaneInfo();
    std::vector<py::array_t<float>> GetHits();
    void AddHit(py::array_t<float> t);
    int GetHitsSize();
};

class PyBindRay {
public:
    Ray* ray = nullptr;
public:
    PyBindRay();

    PyBindRay(py::array_t<float> o, py::array_t<float> dir, float min, float max, int d, int s);

    py::array_t<float> GetDirection() const ;
    py::array_t<float> GetOrigin() const ;
    float GetMin() const;
    float GetMax() const;
    void SetMin(float t);
    void SetMax(float t);
};

class PyBindRayInfo {
public:
    RayIntersectionInfo* rayInfo = nullptr;
public:
    PyBindRayInfo();
    std::vector<float> GetHits() ;
    void SetNormal(float3 n);
    float3 GetNormal();
    void AddHit(float t);
    void AddClosestHit(float t);
    int GetHitsSize();
};

class PyBindContour;

class PyBindBVH {

private:
    std::shared_ptr<BVH> bvh = nullptr;
public:
    //PyBindBVH() { };
    PyBindBVH(std::vector<float>& vertices, SplitMethod splitMethod, int maxPrimsInNode = 255, PrimitiveType primitive_type = PrimitiveType::TRIANGLE);
    py::array_t<float> GetBBoxMin();
    py::array_t<float> GetBBoxMax();
    PyBindBVH(std::shared_ptr<BVH> b);
    ~PyBindBVH() { }
    py::tuple MultiRayIntersect(std::vector<py::array_t<float>> origins, std::vector<py::array_t<float>> directions, float min, float max);
    py::tuple MultiRayAllIntersects(std::vector<py::array_t<float>> origins, std::vector<py::array_t<float>> directions, float min, float max);
    std::vector<std::vector<py::array_t<float>>> MultiRayAllIntersectsHits(std::vector<py::array_t<float>> origins, std::vector<py::array_t<float>> directions);
    std::vector<std::vector<py::array_t<float>>> MultiRayAllIntersectsTransformHits(py::array_t<float>& origin, py::array_t<float>& direction, int number_of_rays, py::array_t<float>& bbox_center, float ray_offset, py::array_t<float>& rotation_matrix, py::array_t<float>& inverse_matrix);
    bool Intersect(PyBindRay& ray, PyBindRayInfo& info);
    bool AnyIntersect(PyBindRay& ray);
    bool AllIntersects(PyBindRay& ray, PyBindRayInfo& info);
    bool PlaneAllIntersects(PyBindPlane& plane, PyBindPlaneInfo& info);
    std::vector<py::array_t<float>> PlaneAllIntersectsHits(PyBindPlane& plane, PyBindPlaneInfo& info);
    std::vector<PyBindContour> PlaneAllIntersectsContours(PyBindPlane& plane, PyBindPlaneInfo& info, py::array_t<float>& transformation_matrix, float const geometry_scaling, float const segment_min_length);
};

class PyBindContour {
public:
    std::shared_ptr<Contour> contour = nullptr;
public:
    PyBindContour() {};
    PyBindContour(std::vector<float>& vertices, py::array_t<float> n);
    PyBindContour(Contour& c);
    ~PyBindContour() {
    }
    bool IsValid();
    py::tuple IsContained(PyBindContour& contour_b);
    py::tuple Contains(PyBindContour& contour_b);
    py::tuple EvaluateContoursRelationship(PyBindContour& contour_b);
    py::array_t<float> GetBBoxMin();
    py::array_t<float> GetBBoxMax();
    bool Intersect(PyBindRay& ray, PyBindRayInfo& info);
    bool AnyIntersect(PyBindRay& ray);
    bool AllIntersects(PyBindRay& ray, PyBindRayInfo& info);
    std::vector<std::vector<py::array_t<float>>> MultiRayAllIntersects(float laser_width_microns, float density, float overlap, float rot_angle, bool verbose = false);
    py::tuple OffsetContour(float offset);
    std::vector<PyBindContour> RemoveSelfIntersections();
    std::vector<py::array_t<float>> GetSegments();

};

class PyBindContourTree {

public:
    std::shared_ptr<ContourTree> contour_tree = nullptr;
public:

    PyBindContourTree(std::vector<PyBindContour>& py_contours);
    PyBindContourTree(ContourTree& c);
    ~PyBindContourTree() {
        //delete bvh;
    }
    py::array_t<float> GetBBoxMin();
    py::array_t<float> GetBBoxMax();
    std::vector<py::array_t<float>> GetAllSegments();
    bool Intersect(PyBindRay& ray, PyBindRayInfo& info);
    bool AnyIntersect(PyBindRay& ray);
    bool AllIntersects(PyBindRay& ray, PyBindRayInfo& info);
    //PyBindContourTree OffsetContourTree(float offset);
    py::tuple OffsetContourTree(float offset);
    std::vector< std::vector<std::vector<py::array_t<float>>>> MultiRayAllIntersects(float laser_width_microns, float density, float overlap, float rot_angle, bool verbose = false);
};
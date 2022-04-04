#pragma once
#ifdef RAYTRACERDLL_EXPORTS
#define RAYTRACERDLL_API __declspec(dllexport)
#else
#define RAYTRACERDLL_API __declspec(dllimport)
#endif

#include <WindowsNumerics.h>
#include <vector>
#include "MyFloat3.h"
#include "MyDouble3.h"
#include "MyFloat4.h"
#include "MyInt3.h"
#include "MyMatrix4x4.h"
namespace num = Windows::Foundation::Numerics;

extern "C" RAYTRACERDLL_API inline constexpr float gamma_error(int n);
class Segment;


class Plane
{
private:
	float3 x_0;
	float3 normal;
public:
	RAYTRACERDLL_API Plane();
	RAYTRACERDLL_API Plane(const float3& x, const float3& n);
	RAYTRACERDLL_API float3 GetX() const { return x_0; };
	RAYTRACERDLL_API float3 GetNormal() const { return normal; };
	RAYTRACERDLL_API float DistFromPlane(const float3& x) const ;
	RAYTRACERDLL_API bool OnPlane(const float3& x) const;
	RAYTRACERDLL_API bool PlaneSegmentIntersection(const float3& p_0, const float3& p_1, float3& p) const;
};

struct PlaneIntersectionInfo
{
private:
	std::vector<float3> t_hits;
public:
	RAYTRACERDLL_API void AddHit(float3 t) { t_hits.push_back(t); };
	RAYTRACERDLL_API int GetHitsSize() { return t_hits.size(); };
	RAYTRACERDLL_API std::vector<float3>* GetHits() { return &t_hits; };
	RAYTRACERDLL_API std::vector<std::shared_ptr<Segment>> GetSegments();
	};

struct Ray
{
private:
	float3 origin;
	float3 direction;
	float t_min;
	float t_max;
	int depth;
	int seed;
public:
	RAYTRACERDLL_API Ray()
	{
		origin = float3(0.0);
		direction = float3(1.0);
		t_min = 0;
		t_max = std::numeric_limits<float>::infinity();
		depth = 0;
		seed = 0;
	};
	RAYTRACERDLL_API Ray(float3 o, float3 dir, float min, float max, int d, int s) 
	{
		origin = o;
		direction = dir;
		t_min = min;
		t_max = max;
		depth = d;
		seed = s;
	};
	RAYTRACERDLL_API float3 GetDirection() const { return direction; };
	RAYTRACERDLL_API float3 GetOrigin() const { return origin; };
	RAYTRACERDLL_API void SetOrigin(float3 o) { origin = o; };
	RAYTRACERDLL_API void SetDirection(float3 d) { direction = d; };
	RAYTRACERDLL_API float GetMin() const { return t_min; };
	RAYTRACERDLL_API float GetMax() const{ return t_max; };
	RAYTRACERDLL_API void SetMin(float t) { t_min = t; };
	RAYTRACERDLL_API void SetMax(float t) { t_max = t; };
};

struct RayIntersectionInfo
{
private:
	float3 normal;
	std::vector<float> t_hits;
public:
	RAYTRACERDLL_API RayIntersectionInfo() {};
	RAYTRACERDLL_API void SetNormal(float3 n) { normal = n; };
	RAYTRACERDLL_API float3 GetNormal() { return normal; };
	RAYTRACERDLL_API std::vector<float>* GetHits() { return &t_hits; };
	RAYTRACERDLL_API void AddHit(float t) { 
		if (std::find(t_hits.begin(), t_hits.end(), t) == t_hits.end())
			t_hits.push_back(t); 
	};
	RAYTRACERDLL_API void AddClosestHit(float t) {
		if (t_hits.size() > 0)
			t_hits[0] = t;
		else
			t_hits.insert(t_hits.begin(), t);
	};
	RAYTRACERDLL_API int GetHitsSize() { return t_hits.size(); };
};

class BBox
{
private:
	float3 pMin;
	float3 pMax;
public:
	RAYTRACERDLL_API BBox();
	RAYTRACERDLL_API BBox(const float3& p);
	RAYTRACERDLL_API BBox(const float3& p0, const float3& p1);
	RAYTRACERDLL_API float3 GetpMax();
	RAYTRACERDLL_API float3 GetpMax() const;
	RAYTRACERDLL_API float3 GetpMin();
	RAYTRACERDLL_API float3 GetpMin() const;
	RAYTRACERDLL_API const float3& operator[](const int i) const;
	RAYTRACERDLL_API float3& operator[](const int i);
	RAYTRACERDLL_API float3 Corner(int corner) const;
	RAYTRACERDLL_API static BBox Union(const BBox& b, const float3& p);
	RAYTRACERDLL_API static BBox Union(const BBox& b0, const BBox& b1);
	RAYTRACERDLL_API static BBox Intersect(const BBox& b0, const BBox& b1);
	RAYTRACERDLL_API static bool Overlaps(const BBox& b0, const BBox& b1);
	RAYTRACERDLL_API static bool Inside(const float3& p, const BBox& b);
	RAYTRACERDLL_API static BBox Expand(const BBox& b, float delta);
	RAYTRACERDLL_API float3 Diagonal() const;
	RAYTRACERDLL_API float SurfaceArea() const;
	RAYTRACERDLL_API float Volume() const;
	RAYTRACERDLL_API int MaximumExtent() const;
	RAYTRACERDLL_API float3 Offset(const float3& p) const;
	RAYTRACERDLL_API bool Intersect(const Ray& ray, float* hit_t0, float* hit_t1) const;
	RAYTRACERDLL_API bool AnyIntersect(const Ray& ray, const float3& invDir, const int dirIsNeg[3]) const;
	RAYTRACERDLL_API bool PlaneAnyIntersect(const Plane& plane) const;
};

class Primitive
{
protected:
	BBox bbox;
public:
	RAYTRACERDLL_API Primitive() { bbox = BBox(); }
	RAYTRACERDLL_API BBox GetBBox() { return bbox; };
	RAYTRACERDLL_API virtual void ComputeBBox() = 0;
	RAYTRACERDLL_API virtual bool Intersect(Ray& ray, RayIntersectionInfo& info) = 0;
	RAYTRACERDLL_API virtual bool AnyIntersect(Ray& ray) = 0;
	RAYTRACERDLL_API virtual bool AllIntersect(Ray& ray, RayIntersectionInfo& info) = 0;
	RAYTRACERDLL_API virtual bool PlaneIntersect(Plane& plane, PlaneIntersectionInfo& info) = 0;
};


class Segment : public Primitive
{
public:
	float3 v0;
	float3 v1;
public:
	RAYTRACERDLL_API Segment() {};
	RAYTRACERDLL_API Segment(const float3& p0, const float3& p1);
	RAYTRACERDLL_API void ComputeBBox();
	RAYTRACERDLL_API bool Intersect(Ray& ray, RayIntersectionInfo& info);
	RAYTRACERDLL_API bool AnyIntersect(Ray& ray);
	RAYTRACERDLL_API bool AllIntersect(Ray& ray, RayIntersectionInfo& info);
	RAYTRACERDLL_API bool PlaneIntersect(Plane& plane, PlaneIntersectionInfo& info);
	RAYTRACERDLL_API float3 operator[](int i);
	RAYTRACERDLL_API float3 operator[](int i) const;
	RAYTRACERDLL_API void FlipSegment() { float3 tmp = v0; v0 = v1; v1 = tmp; };
	RAYTRACERDLL_API static Segment FlipSegment(Segment& const s) { return Segment(s.v1, s.v0); };
	RAYTRACERDLL_API bool IntersectSegment(Segment& const s, float3& hit_point, float& t_hit);

	RAYTRACERDLL_API static std::vector<std::vector<std::shared_ptr<Segment>>> SortSegments(std::vector<std::shared_ptr<Segment>>& segments, float const epsilon, bool remove_aligned_segments, float const alignment_epsilon, bool remove_short_segments, float const min_segment_length);
private:

	RAYTRACERDLL_API bool static CompareSegments(Segment& s0, Segment& s1, float const epsilon);
	RAYTRACERDLL_API bool static CheckAlignment(Segment& s0, Segment& s1, float const alignment_epsilon);
	RAYTRACERDLL_API bool static CheckMinLength(Segment& s0, Segment& s1, float const min_segment_length);
	RAYTRACERDLL_API bool static MergeSegments(std::vector<std::shared_ptr<Segment>>& s0, std::vector<std::shared_ptr<Segment>>& s1, float const epsilon, bool remove_aligned_segments, float const alignment_epsilon, bool remove_short_segments, float const min_segment_length);
};
RAYTRACERDLL_API std::ostream& operator<<(std::ostream& os, Segment const& s);



class Sphere : public Primitive
{
public:
	float radius;
	float3 center;
public:
	RAYTRACERDLL_API Sphere(const float r, const float3& c);
	RAYTRACERDLL_API void ComputeBBox();
	RAYTRACERDLL_API bool Intersect(Ray& ray, RayIntersectionInfo& info);
	RAYTRACERDLL_API bool AnyIntersect(Ray& ray);
	RAYTRACERDLL_API bool AllIntersect(Ray& ray, RayIntersectionInfo& info);
	RAYTRACERDLL_API bool PlaneIntersect(Plane& plane, PlaneIntersectionInfo& info);
};


class Triangle : public Primitive
{
public:
	float3 v0;
	float3 v1;
	float3 v2;
public:
	RAYTRACERDLL_API Triangle(const float3 p0, const float3 p1, const float3 p2);
	RAYTRACERDLL_API void ComputeBBox();
	RAYTRACERDLL_API bool Intersect(Ray& ray, RayIntersectionInfo& info);
	RAYTRACERDLL_API bool AnyIntersect(Ray& ray);
	RAYTRACERDLL_API bool AllIntersect(Ray& ray, RayIntersectionInfo& info);
	RAYTRACERDLL_API bool PlaneIntersect(Plane& plane, PlaneIntersectionInfo& info);
	RAYTRACERDLL_API float3 operator[](int i);
	RAYTRACERDLL_API float3 operator[](int i) const;
};

//
//class IntTriangle : public Primitive
//{
//public:
//	int3 v0;
//	int3 v1;
//	int3 v2;
//public:
//	RAYTRACERDLL_API IntTriangle(const int3 p0, const int3 p1, const int3 p2);
//	RAYTRACERDLL_API void ComputeBBox();
//	RAYTRACERDLL_API bool Intersect(Ray& ray, RayIntersectionInfo& info);
//	RAYTRACERDLL_API bool AnyIntersect(Ray& ray);
//	RAYTRACERDLL_API bool AllIntersect(Ray& ray, RayIntersectionInfo& info);
//	RAYTRACERDLL_API bool PlaneIntersect(Plane& plane, PlaneIntersectionInfo& info);
//};

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
#include "MyMatrix4x4.h"
namespace num = Windows::Foundation::Numerics;

extern "C" RAYTRACERDLL_API inline constexpr float gamma_error(int n);

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
	RAYTRACERDLL_API void AddHit(float t) { t_hits.push_back(t); };
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
	float3 pMax;
	float3 pMin;
public:
	RAYTRACERDLL_API BBox();
	RAYTRACERDLL_API BBox(const float3& p);
	RAYTRACERDLL_API BBox(const float3& p0, const float3& p1);
	RAYTRACERDLL_API float3 GetpMax();
	RAYTRACERDLL_API float3 GetpMin();
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
	RAYTRACERDLL_API virtual bool PlaneIntersect(const Plane& plane, PlaneIntersectionInfo& info) = 0;
};

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
	RAYTRACERDLL_API bool PlaneIntersect(const Plane& plane, PlaneIntersectionInfo& info);
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
	RAYTRACERDLL_API bool PlaneIntersect(const Plane& plane, PlaneIntersectionInfo& info);
};

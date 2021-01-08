#pragma once
#ifdef RAYTRACER_EXPORTS
#define RAYTRACER_API __declspec(dllexport)
#else
#define RAYTRACER_API __declspec(dllimport)
#endif

#include <WindowsNumerics.h>
#include <vector>
#include "MyFloat3.h"
namespace num = Windows::Foundation::Numerics;

extern "C" RAYTRACER_API inline constexpr float gamma(int n);

class Plane
{
private:
	float3 x_0;
	float3 normal;
public:
	RAYTRACER_API Plane(const float3& x, const float3& n);
	RAYTRACER_API float DistFromPlane(const float3& x) const ;
	RAYTRACER_API bool OnPlane(const float3& x) const;
	RAYTRACER_API bool PlaneSegmentIntersection(const float3& p_0, const float3& p_1, float3& p) const;
};

struct PlaneIntersectionInfo
{
private:
	std::vector<float3> t_hits;
public:
	RAYTRACER_API void AddHit(float3 t) { t_hits.push_back(t); };
	RAYTRACER_API int GetHitsSize() { return t_hits.size(); };
	RAYTRACER_API std::vector<float3>* GetHits() { return &t_hits; };
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
	RAYTRACER_API Ray(float3 o, float3 dir, float min, float max, int d, int s) 
	{
		origin = o;
		direction = dir;
		t_min = min;
		t_max = max;
		depth = d;
		seed = s;
	};
	RAYTRACER_API float3 GetDirection() const { return direction; };
	RAYTRACER_API float3 GetOrigin() const { return origin; };
	RAYTRACER_API float GetMin() const { return t_min; };
	RAYTRACER_API float GetMax() const{ return t_max; };
	RAYTRACER_API void SetMin(float t) { t_min = t; };
	RAYTRACER_API void SetMax(float t) { t_max = t; };
};

struct RayIntersectionInfo
{
private:
	float3 normal;
	std::vector<float> t_hits;
public:
	RAYTRACER_API RayIntersectionInfo() {};
	RAYTRACER_API void SetNormal(float3 n) { normal = n; };
	RAYTRACER_API float3 GetNormal() { return normal; };
	RAYTRACER_API std::vector<float>* GetHits() { return &t_hits; };
	RAYTRACER_API void AddHit(float t) { t_hits.push_back(t); };
	RAYTRACER_API void AddClosestHit(float t) {
		if (t_hits.size() > 0)
			t_hits[0] = t;
		else
			t_hits.insert(t_hits.begin(), t);
	};
	RAYTRACER_API int GetHitsSize() { return t_hits.size(); };
};

class BBox
{
private:
	float3 pMax;
	float3 pMin;
public:
	RAYTRACER_API BBox();
	RAYTRACER_API BBox(const float3& p);
	RAYTRACER_API BBox(const float3& p0, const float3& p1);
	RAYTRACER_API float3 GetpMax();
	RAYTRACER_API float3 GetpMin();
	RAYTRACER_API const float3& operator[](const int i) const;
	RAYTRACER_API float3& operator[](const int i);
	RAYTRACER_API float3 Corner(int corner) const;
	RAYTRACER_API static BBox Union(const BBox& b, const float3& p);
	RAYTRACER_API static BBox Union(const BBox& b0, const BBox& b1);
	RAYTRACER_API static BBox Intersect(const BBox& b0, const BBox& b1);
	RAYTRACER_API static bool Overlaps(const BBox& b0, const BBox& b1);
	RAYTRACER_API static bool Inside(const float3& p, const BBox& b);
	RAYTRACER_API static BBox Expand(const BBox& b, float delta);
	RAYTRACER_API float3 Diagonal() const;
	RAYTRACER_API float SurfaceArea() const;
	RAYTRACER_API float Volume() const;
	RAYTRACER_API int MaximumExtent() const;
	RAYTRACER_API float3 Offset(const float3& p) const;
	RAYTRACER_API bool Intersect(const Ray& ray, float* hit_t0, float* hit_t1) const;
	RAYTRACER_API bool AnyIntersect(const Ray& ray, const float3& invDir, const int dirIsNeg[3]) const;
	RAYTRACER_API bool PlaneAnyIntersect(const Plane& plane) const;
};

class Primitive
{
protected:
	BBox bbox;
public:
	RAYTRACER_API Primitive() { bbox = BBox(); }
	RAYTRACER_API BBox GetBBox() { return bbox; };
	RAYTRACER_API virtual void ComputeBBox() = 0;
	RAYTRACER_API virtual bool Intersect(Ray& ray, RayIntersectionInfo& info) = 0;
	RAYTRACER_API virtual bool AnyIntersect(Ray& ray) = 0;
	RAYTRACER_API virtual bool AllIntersect(Ray& ray, RayIntersectionInfo& info) = 0;
	RAYTRACER_API virtual bool PlaneIntersect(const Plane& plane, PlaneIntersectionInfo& info) = 0;
};

class Sphere : public Primitive
{
public:
	float radius;
	float3 center;
public:
	RAYTRACER_API Sphere(const float r, const float3& c);
	RAYTRACER_API void ComputeBBox();
	RAYTRACER_API bool Intersect(Ray& ray, RayIntersectionInfo& info);
	RAYTRACER_API bool AnyIntersect(Ray& ray);
	RAYTRACER_API bool AllIntersect(Ray& ray, RayIntersectionInfo& info);
	RAYTRACER_API bool PlaneIntersect(const Plane& plane, PlaneIntersectionInfo& info);
};


class Triangle : public Primitive
{
public:
	float3 v0;
	float3 v1;
	float3 v2;
public:
	RAYTRACER_API Triangle(const float3 p0, const float3 p1, const float3 p2);
	RAYTRACER_API void ComputeBBox();
	RAYTRACER_API bool Intersect(Ray& ray, RayIntersectionInfo& info);
	RAYTRACER_API bool AnyIntersect(Ray& ray);
	RAYTRACER_API bool AllIntersect(Ray& ray, RayIntersectionInfo& info);
	RAYTRACER_API bool PlaneIntersect(const Plane& plane, PlaneIntersectionInfo& info);
};

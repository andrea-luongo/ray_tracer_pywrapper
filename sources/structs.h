#pragma once
#ifdef RAYTRACER_EXPORTS
#define RAYTRACER_API __declspec(dllexport)
#else
#define RAYTRACER_API __declspec(dllimport)
#endif

#include <WindowsNumerics.h>
#include <vector>

namespace num = Windows::Foundation::Numerics;

extern "C" RAYTRACER_API bool CompareFloat3(const num::float3 & p0, const num::float3 & p1, int dim);

extern "C" RAYTRACER_API float GetFloat3Component(const num::float3 & p, const int dim);

extern "C" RAYTRACER_API inline constexpr float gamma(int n);


class Plane
	{
	private:
		num::float3 x_0;
		num::float3 normal;
	public:
		RAYTRACER_API Plane(const num::float3& x, const num::float3& n);
		RAYTRACER_API float DistFromPlane(const num::float3& x);
		RAYTRACER_API const float DistFromPlane(const num::float3& x) const ;
		RAYTRACER_API bool OnPlane(const num::float3& x) ;
		RAYTRACER_API const bool OnPlane(const num::float3& x) const;
		RAYTRACER_API 	bool PlaneSegmentIntersection(const num::float3& p_0, const num::float3& p_1, num::float3& p) ;
		RAYTRACER_API 	const bool PlaneSegmentIntersection(const num::float3& p_0, const num::float3& p_1, num::float3& p) const;
	};

struct PlaneIntersectionInfo
{
private:
	std::vector<num::float3> t_hits;
public:
	RAYTRACER_API void AddHit(num::float3 t) { t_hits.push_back(t); };
	RAYTRACER_API int GetHitsSize() { return t_hits.size(); };
};

struct Ray
{
private:
	num::float3 origin;
	num::float3 direction;
	float t_min;
	float t_max;
	int depth;
	int seed;
public:
	RAYTRACER_API Ray(num::float3 o, num::float3 dir, float min, float max, int d, int s) 
	{
		origin = o;
		direction = dir;
		t_min = min;
		t_max = max;
		depth = d;
		seed = s;
	};
	RAYTRACER_API num::float3 GetOrigin() { return origin; };
	RAYTRACER_API num::float3 GetOrigin() const { return origin; };
	RAYTRACER_API num::float3 GetDirection() { return direction; };
	RAYTRACER_API num::float3 GetDirection() const { return direction; };
	RAYTRACER_API float GetMin() { return t_min; };
	RAYTRACER_API float GetMin() const { return t_min; };
	RAYTRACER_API float GetMax() { return t_max; };
	RAYTRACER_API float GetMax() const{ return t_max; };
	RAYTRACER_API void SetMin(float t) { t_min = t; };
	RAYTRACER_API void SetMax(float t) { t_max = t; };
};

struct RayIntersectionInfo
{
private:
	num::float3 normal;
	std::vector<float> t_hits;
public:
	RAYTRACER_API RayIntersectionInfo();
	RAYTRACER_API void SetNormal(num::float3 n) { normal = n; };
	RAYTRACER_API num::float3 GetNormal() { return normal; };
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
	num::float3 pMax;
	num::float3 pMin;
public:
	RAYTRACER_API BBox();
	RAYTRACER_API BBox(const num::float3& p);
	RAYTRACER_API BBox(const num::float3& p0, const num::float3& p1);
	RAYTRACER_API num::float3 GetpMax();
	RAYTRACER_API num::float3 GetpMin();
	RAYTRACER_API const num::float3& operator[](const int i) const;
	RAYTRACER_API num::float3& operator[](const int i);
	RAYTRACER_API num::float3 Corner(int corner) const;
	RAYTRACER_API static BBox Union(const BBox& b, const num::float3& p);
	RAYTRACER_API static BBox Union(const BBox& b0, const BBox& b1);
	RAYTRACER_API static BBox Intersect(const BBox& b0, const BBox& b1);
	RAYTRACER_API static bool Overlaps(const BBox& b0, const BBox& b1);
	RAYTRACER_API static bool Inside(const num::float3& p, const BBox& b);
	RAYTRACER_API static BBox Expand(const BBox& b, float delta);
	RAYTRACER_API num::float3 Diagonal() const;
	RAYTRACER_API float SurfaceArea() const;
	RAYTRACER_API float Volume() const;
	RAYTRACER_API int MaximumExtent() const;
	RAYTRACER_API num::float3 Offset(const num::float3& p) const;
	RAYTRACER_API bool Intersect(const Ray& ray, float* hit_t0, float* hit_t1) const;
	RAYTRACER_API bool AnyIntersect(const Ray& ray, const num::float3& invDir, const int dirIsNeg[3]) const;
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
private:
	float radius;
	num::float3 center;
public:
	RAYTRACER_API Sphere(const float r, const num::float3& c);
	RAYTRACER_API void ComputeBBox();
	RAYTRACER_API bool Intersect(Ray& ray, RayIntersectionInfo& info);
	RAYTRACER_API bool AnyIntersect(Ray& ray);
	RAYTRACER_API bool AllIntersect(Ray& ray, RayIntersectionInfo& info);
	RAYTRACER_API bool PlaneIntersect(const Plane& plane, PlaneIntersectionInfo& info);
};


class Triangle : public Primitive
{
private:
	num::float3 v0;
	num::float3 v1;
	num::float3 v2;
public:
	RAYTRACER_API Triangle(const num::float3 p0, const num::float3 p1, const num::float3 p2);
	RAYTRACER_API void ComputeBBox();
	RAYTRACER_API bool Intersect(Ray& ray, RayIntersectionInfo& info);
	RAYTRACER_API bool AnyIntersect(Ray& ray);
	RAYTRACER_API bool AllIntersect(Ray& ray, RayIntersectionInfo& info);
	RAYTRACER_API bool PlaneIntersect(const Plane& plane, PlaneIntersectionInfo& info);
};

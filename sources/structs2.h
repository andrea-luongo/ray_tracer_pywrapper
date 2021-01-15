#pragma once

#include <WindowsNumerics.h>
#include <vector>
#include "MyFloat32.h"
namespace num = Windows::Foundation::Numerics;

extern "C" inline constexpr float gamma(int n);

class Plane
{
private:
	float3 x_0;
	float3 normal;
public:
	 Plane(const float3& x, const float3& n);
	 float DistFromPlane(const float3& x) const ;
	 bool OnPlane(const float3& x) const;
	 bool PlaneSegmentIntersection(const float3& p_0, const float3& p_1, float3& p) const;
};

struct PlaneIntersectionInfo
{
private:
	std::vector<float3> t_hits;
public:
	 void AddHit(float3 t) { t_hits.push_back(t); };
	 int GetHitsSize() { return t_hits.size(); };
	 std::vector<float3>* GetHits() { return &t_hits; };
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
	 Ray(float3 o, float3 dir, float min, float max, int d, int s) 
	{
		origin = o;
		direction = dir;
		t_min = min;
		t_max = max;
		depth = d;
		seed = s;
	};
	 float3 GetDirection() const { return direction; };
	 float3 GetOrigin() const { return origin; };
	 void SetOrigin(float3 o) { origin = o; };
	 void SetDirection(float3 d) { direction = d; };
	 float GetMin() const { return t_min; };
	 float GetMax() const{ return t_max; };
	 void SetMin(float t) { t_min = t; };
	 void SetMax(float t) { t_max = t; };
};

struct RayIntersectionInfo
{
private:
	float3 normal;
	std::vector<float> t_hits;
public:
	 RayIntersectionInfo() {};
	 void SetNormal(float3 n) { normal = n; };
	 float3 GetNormal() { return normal; };
	 std::vector<float>* GetHits() { return &t_hits; };
	 void AddHit(float t) { t_hits.push_back(t); };
	 void AddClosestHit(float t) {
		if (t_hits.size() > 0)
			t_hits[0] = t;
		else
			t_hits.insert(t_hits.begin(), t);
	};
	 int GetHitsSize() { return t_hits.size(); };
};

class BBox
{
private:
	float3 pMax;
	float3 pMin;
public:
	 BBox();
	 BBox(const float3& p);
	 BBox(const float3& p0, const float3& p1);
	 float3 GetpMax();
	 float3 GetpMin();
	 const float3& operator[](const int i) const;
	 float3& operator[](const int i);
	 float3 Corner(int corner) const;
	 static BBox Union(const BBox& b, const float3& p);
	 static BBox Union(const BBox& b0, const BBox& b1);
	 static BBox Intersect(const BBox& b0, const BBox& b1);
	 static bool Overlaps(const BBox& b0, const BBox& b1);
	 static bool Inside(const float3& p, const BBox& b);
	 static BBox Expand(const BBox& b, float delta);
	 float3 Diagonal() const;
	 float SurfaceArea() const;
	 float Volume() const;
	 int MaximumExtent() const;
	 float3 Offset(const float3& p) const;
	 bool Intersect(const Ray& ray, float* hit_t0, float* hit_t1) const;
	 bool AnyIntersect(const Ray& ray, const float3& invDir, const std::vector<int> dirIsNeg) const;
	 bool PlaneAnyIntersect(const Plane& plane) const;
};

class Primitive
{
protected:
	BBox bbox;
public:
	 Primitive() { bbox = BBox(); }
	 BBox GetBBox() { return bbox; };
	 virtual void ComputeBBox() = 0;
	 virtual bool Intersect(Ray& ray, RayIntersectionInfo& info) = 0;
	 virtual bool AnyIntersect(Ray& ray) = 0;
	 virtual bool AllIntersect(Ray& ray, RayIntersectionInfo& info) = 0;
	 virtual bool PlaneIntersect(const Plane& plane, PlaneIntersectionInfo& info) = 0;
};

class Sphere : public Primitive
{
public:
	float radius;
	float3 center;
public:
	 Sphere(const float r, const float3& c);
	 void ComputeBBox();
	 bool Intersect(Ray& ray, RayIntersectionInfo& info);
	 bool AnyIntersect(Ray& ray);
	 bool AllIntersect(Ray& ray, RayIntersectionInfo& info);
	 bool PlaneIntersect(const Plane& plane, PlaneIntersectionInfo& info);
};


class Triangle : public Primitive
{
public:
	float3 v0;
	float3 v1;
	float3 v2;
public:
	 Triangle(const float3 p0, const float3 p1, const float3 p2);
	 void ComputeBBox();
	 bool Intersect(Ray& ray, RayIntersectionInfo& info);
	 bool AnyIntersect(Ray& ray);
	 bool AllIntersect(Ray& ray, RayIntersectionInfo& info);
	 bool PlaneIntersect(const Plane& plane, PlaneIntersectionInfo& info);
};

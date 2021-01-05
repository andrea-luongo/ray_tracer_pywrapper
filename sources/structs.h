#pragma once
#include <WindowsNumerics.h>
#include <vector>

namespace num = Windows::Foundation::Numerics;


const float machine_epsilon = std::numeric_limits<float>::epsilon() * 0.5;

bool CompareFloat3(const num::float3& p0, const num::float3& p1, int dim) 
{
	if (dim == 0)
		return p0.x == p1.x;
	if (dim == 1)
		return p0.y == p1.y;
	if (dim == 2)
		return p0.z == p1.z;
	return false;
}

float GetFloat3Component(const num::float3& p, const int dim)
{
	if (dim == 0)
		return p.x;
	else if (dim == 1)
		return p.y;
	else
		return p.z;
}

inline constexpr float gamma(int n)
{
	return (n * machine_epsilon) / (1 - n * machine_epsilon);
}

namespace MyStructures
{
	class Plane
	{
	private:
		num::float3 x_0;
		num::float3 normal;
	public:
		Plane(const num::float3& x, const num::float3& n) { x_0 = x; normal = num::normalize(n); };
		float DistFromPlane(const num::float3& x) { return num::dot(normal, x - x_0); };
		const float DistFromPlane(const num::float3& x) const { return num::dot(normal, x - x_0); };
		bool OnPlane(const num::float3& x) { return abs(DistFromPlane(x)) < machine_epsilon; };
		const bool OnPlane(const num::float3& x) const{ return abs(DistFromPlane(x)) < machine_epsilon; };
		bool PlaneSegmentIntersection(const num::float3& p_0, const num::float3& p_1, num::float3& p) 
		{
			float d_0 = DistFromPlane(p_0);
			float d_1 = DistFromPlane(p_1);
			if (d_0 * d_1 > 0)
			{
				return false;
			}
			float t = d_0 / (d_0 - d_1);
			p = p_0 + t * (p_1 - p_0);
			return true;
		};
		const bool PlaneSegmentIntersection(const num::float3& p_0, const num::float3& p_1, num::float3& p) const 
		{
			float d_0 = DistFromPlane(p_0);
			float d_1 = DistFromPlane(p_1);
			if (d_0 * d_1 > 0)
			{
				return false;
			}
			float t = d_0 / (d_0 - d_1);
			p = p_0 + t * (p_1 - p_0);
			return true;
		};
	};

	struct PlaneIntersectionInfo
	{
		std::vector<num::float3> t_hits;
	};

	struct Ray
	{
		num::float3 origin;
		num::float3 direction;
		float t_min;
		float t_max;
		int depth;
		int seed;
	};

	struct RayIntersectionInfo
	{
		num::float3 normal;
		std::vector<float> t_hits;
	};

	class BBox
	{
	public:
		num::float3 pMin;
		num::float3 pMax;
		BBox()
		{
			pMin = num::float3(std::numeric_limits<float>::min());
			pMax = num::float3(std::numeric_limits<float>::max());
		};

		BBox(const num::float3& p) : pMin(p), pMax(p) { };

		BBox(const num::float3& p0, const num::float3& p1)
		{
			pMin = num::min(p0, p1);
			pMax = num::max(p0, p1);
		}

		const num::float3& operator[](const int i) const { if (i == 0) return pMin; else return pMax; };

		num::float3& operator[](const int i) { if (i == 0) return pMin; else return pMax; };

		num::float3 Corner(int corner) const
		{
			return num::float3((*this)[(corner & 1)].x, (*this)[(corner & 2) ? 1 : 0].y, (*this)[(corner & 4) ? 1 : 0].z);
		}

		static BBox Union(const BBox& b, const num::float3& p)
		{
			return BBox(num::min(b.pMin, p), num::max(b.pMax, p));
		}

		static BBox Union(const BBox& b0, const BBox& b1)
		{
			return BBox(num::min(b0.pMin, b1.pMin), num::max(b0.pMax, b1.pMax));
		}

		static BBox Intersect(const BBox& b0, const BBox& b1)
		{
			return BBox(num::max(b0.pMin, b1.pMin), num::min(b0.pMax, b1.pMax));
		}

		static bool Overlaps(const BBox& b0, const BBox& b1)
		{
			bool x = (b0.pMax.x >= b1.pMin.x) && (b0.pMin.x <= b1.pMax.x);
			bool y = (b0.pMax.y >= b1.pMin.y) && (b0.pMin.y <= b1.pMax.y);
			bool z = (b0.pMax.z >= b1.pMin.z) && (b0.pMin.z <= b1.pMax.z);
			return (x && y && z);
		}

		static bool Inside(const num::float3& p, const BBox& b)
		{
			return (p.x >= b.pMin.x && p.x <= b.pMax.x && p.y >= b.pMin.y && p.y <= b.pMax.y && p.z >= b.pMin.z && p.z <= b.pMax.z);
		}

		static BBox Expand(const BBox& b, float delta)
		{
			return BBox(b.pMin - num::float3(delta), b.pMax + num::float3(delta));
		}

		num::float3 Diagonal() const { return pMax - pMin; }

		float SurfaceArea() const { num::float3 d = Diagonal(); return 2 * (d.x * d.y + d.x * d.z + d.y * d.z); }

		float Volume() const { num::float3 d = Diagonal(); return d.x * d.y * d.z; }

		int MaximumExtent() const
		{
			num::float3 d = Diagonal();
			if (d.x > d.y && d.x > d.z)
				return 0;
			else if (d.y > d.z)
				return 1;
			else
				return 2;
		}

		num::float3 Offset(const num::float3& p) const 
		{
			num::float3 o = p - pMin;
			if (pMax.x > pMin.x) o.x /= pMax.x - pMin.x;
			if (pMax.y > pMin.y) o.y /= pMax.y - pMin.y;
			if (pMax.z > pMin.z) o.z /= pMax.z - pMin.z;
			return o;
		}

		bool Intersect(const Ray& ray, float* hit_t0, float* hit_t1) const
		{
			float t0 = 0, t1 = ray.t_max;
			for (int i = 0; i < 3; ++i)
			{
				//update interval for ith bounding box slab
				float invRayDir = 1 / GetFloat3Component(ray.direction, i);
				float tNear = (GetFloat3Component(pMin, i) - GetFloat3Component(ray.origin, i)) * invRayDir;
				float tFar = (GetFloat3Component(pMax, i) - GetFloat3Component(ray.origin, i)) * invRayDir;
				//update parameteric interval from slab intersection t values
				if (tNear > tFar) std::swap(tNear, tFar);
				//update tFar to ensure robust ray-bounds intersection
				tFar *= 1 + 2 * gamma(3);
				t0 = tNear > t0 ? tNear : t0;
				t0 = tFar < t1 ? tFar : t1;
				if (t0 > t1) return false;
			}
			if(hit_t0)* hit_t0 = t0;
			if(hit_t1)* hit_t1 = t1;
			return true;
		}

		bool AnyIntersect(const Ray& ray, const num::float3& invDir, const int dirIsNeg[3]) const
		{
			const MyStructures::BBox& bounds = *this;
			float tMin = (bounds[dirIsNeg[0]].x - ray.origin.x) * invDir.x;
			float tMax = (bounds[1 - dirIsNeg[0]].x - ray.origin.x) * invDir.x;
			float tyMin = (bounds[dirIsNeg[1]].y - ray.origin.y) * invDir.y;
			float tyMax = (bounds[1 - dirIsNeg[1]].y - ray.origin.y) * invDir.y;
			tMax *= 1 + 2 * gamma(3);
			tyMax *= 1 + 2 * gamma(3);
			if (tMin > tyMax || tyMin > tMax)
				return false;
			if (tyMin > tMin) tMin = tyMin;
			if (tyMax < tMax) tMax = tyMax;
			float tzMin = (bounds[dirIsNeg[2]].z - ray.origin.z) * invDir.z;
			float tzMax = (bounds[1 - dirIsNeg[2]].z - ray.origin.z) * invDir.z;
			tzMax *= 1 + 2 * gamma(3);
			if (tMin > tzMax || tzMin > tMax)
				return false;
			if (tzMin > tMin) tMin = tzMin;
			if (tzMax < tMax) tMax = tzMax;
			return (tMin < ray.t_max) && (tMax > 0);
		}

		bool PlaneAnyIntersect(const Plane& plane) const
		{
			float d_0 = plane.DistFromPlane(pMin);
			float d_1 = plane.DistFromPlane(pMax);
			if (d_1 * d_0 < 0.0)
				return true;
			num::float3 p_2(pMin.x, pMin.y, pMax.z);
			num::float3 p_3(pMax.x, pMax.y, pMin.z);
			float d_2 = plane.DistFromPlane(p_2);
			float d_3 = plane.DistFromPlane(p_3);
			if (d_2 * d_3 < 0.0)
				return true;
			num::float3 p_4(pMin.x, pMax.y, pMax.z);
			num::float3 p_5(pMax.x, pMin.y, pMin.z);
			float d_4 = plane.DistFromPlane(p_4);
			float d_5 = plane.DistFromPlane(p_5);
			if (d_4 * d_5 < 0.0)
				return true;
			num::float3 p_6(pMin.x, pMax.y, pMin.z);
			num::float3 p_7(pMax.x, pMin.y, pMax.z);
			float d_6 = plane.DistFromPlane(p_6);
			float d_7 = plane.DistFromPlane(p_7);
			if (d_6 * d_7 < 0.0)
				return true;
		}
	};

	class Primitive
	{
	protected:
		MyStructures::BBox bbox;
	public:
		Primitive() { bbox = MyStructures::BBox(); }
		void ComputeBBox() {}
		MyStructures::BBox BBox() { return bbox; }
		bool Intersect(MyStructures::Ray& ray, MyStructures::RayIntersectionInfo& info) {
			return false;
		}
		bool AnyIntersect(MyStructures::Ray& ray) {
			return false;
		}
		bool AllIntersect(MyStructures::Ray& ray, MyStructures::RayIntersectionInfo& info) {
			return false;
		}
		bool PlaneIntersect(const MyStructures::Plane& plane, MyStructures::PlaneIntersectionInfo& info) {
			return false;
		}
	};

	class Sphere : public Primitive
	{
	protected:
		float radius;
		num::float3 center;
	public:
		Sphere(const float r, const num::float3& c) { radius = r; center = c; ComputeBBox(); }
		
		void ComputeBBox()
		{
			bbox = MyStructures::BBox(center - num::float3(radius), center + num::float3(radius));
			return;
		};

		bool Intersect(MyStructures::Ray& ray, MyStructures::RayIntersectionInfo& info) {
			num::float3 O = ray.origin - center;
			float b = num::dot(O, ray.direction);
			float c = num::dot(O, O) - radius * radius;
			float disc = b * b - c;
			if (disc > 0.0)
			{
				float s_disc = sqrt(disc);
				float t = -b - s_disc;
				if (ray.t_min < t && t < ray.t_max)
				{
					num::float3 n = (O + t * ray.direction) / radius;
					ray.t_max = t;
					info.normal = num::normalize(n);
					if (info.t_hits.size() > 0)
						info.t_hits[0] = t;
					else
						info.t_hits.insert(info.t_hits.begin(), t);
					return true;
				}
				t = -b + s_disc;
				if (ray.t_min < t && t < ray.t_max)
				{
					num::float3 n = (O + t * ray.direction) / radius;
					ray.t_max = t;
					info.normal = num::normalize(n);
					if (info.t_hits.size() > 0)
						info.t_hits[0] = t;
					else
						info.t_hits.insert(info.t_hits.begin(), t);
					return true;
				}
			}
			return false;
		}

		bool AnyIntersect(MyStructures::Ray& ray) {
			num::float3 O = ray.origin - center;
			float b = num::dot(O, ray.direction);
			float c = num::dot(O, O) - radius * radius;
			float disc = b * b - c;
			if (disc > 0.0)
			{
				float s_disc = sqrt(disc);
				float t = -b - s_disc;
				if (ray.t_min < t && t < ray.t_max)
				{
					return true;
				}
				t = -b + s_disc;
				if (ray.t_min < t && t < ray.t_max)
				{
					return true;
				}
			}
			return false;
		}

		bool AllIntersect(MyStructures::Ray& ray, MyStructures::RayIntersectionInfo& info) {
			num::float3 O = ray.origin - center;
			float b = num::dot(O, ray.direction);
			float c = num::dot(O, O) - radius * radius;
			float disc = b * b - c;
			bool hit = false;
			if (disc > 0.0)
			{
				float s_disc = sqrt(disc);
				float t = -b - s_disc;
				if (ray.t_min < t && t < ray.t_max)
				{
					info.t_hits.push_back(t);
					hit = true;
				}
				t = -b + s_disc;
				if (ray.t_min < t && t < ray.t_max)
				{
					info.t_hits.push_back(t);
					hit = true;
				}
			}
			return hit;
		}
	};


	class Triangle : public Primitive
	{
	protected:
		num::float3 v0;
		num::float3 v1;
		num::float3 v2;
	public:
		Triangle(const num::float3 p0, const num::float3 p1, const num::float3 p2)
		{
			v0 = p0; v1 = p1; v2 = p2;
			ComputeBBox();
		}

		void ComputeBBox()
		{
			float area = num::length(num::cross(v1 - v0, v2 - v0));
			if (area > 0.0 && isfinite(area))
			{
				num::float3 b_min = num::min(num::min(v0, v1), v2);
				num::float3 b_max = num::max(num::max(v0, v1), v2);
				bbox = MyStructures::BBox(b_min, b_max);
			}
		}

		bool Intersect(MyStructures::Ray& ray, MyStructures::RayIntersectionInfo& info) {
			num::float3 e0 = v1 - v0;
			num::float3 e1 = v0 - v2;
			num::float3 n = num::cross(e1, e0);
			num::float3 e2 = 1.0 / num::dot(n, ray.direction) * (v0 - ray.origin);
			num::float3 i = num::cross(ray.direction, e2);
			float beta = num::dot(i, e1);
			float gamma = num::dot(i, e0);
			float t = num::dot(n, e2);
			if (ray.t_max > t && t > ray.t_min && beta > 0.0 && gamma >= 0.0 && beta + gamma <= 1)
			{
				ray.t_max = t;
				info.normal = num::normalize(n);
				if (info.t_hits.size() > 0)
					info.t_hits[0] = t;
				else
					info.t_hits.insert(info.t_hits.begin(), t);
				return true;
			}
			return false;
		}

		bool AnyIntersect(MyStructures::Ray& ray) {
			num::float3 e0 = v1 - v0;
			num::float3 e1 = v0 - v2;
			num::float3 n = num::cross(e1, e0);
			num::float3 e2 = 1.0 / num::dot(n, ray.direction) * (v0 - ray.origin);
			num::float3 i = num::cross(ray.direction, e2);
			float beta = num::dot(i, e1);
			float gamma = num::dot(i, e0);
			float t = num::dot(n, e2);
			if (ray.t_max > t && t > ray.t_min && beta > 0.0 && gamma >= 0.0 && beta + gamma <= 1)
			{
				return true;
			}
			return false;
		}

		bool AllIntersect(MyStructures::Ray& ray, MyStructures::RayIntersectionInfo& info) {
			num::float3 e0 = v1 - v0;
			num::float3 e1 = v0 - v2;
			num::float3 n = num::cross(e1, e0);
			num::float3 e2 = 1.0 / num::dot(n, ray.direction) * (v0 - ray.origin);
			num::float3 i = num::cross(ray.direction, e2);
			float beta = num::dot(i, e1);
			float gamma = num::dot(i, e0);
			float t = num::dot(n, e2);
			if (ray.t_max > t && t > ray.t_min && beta > 0.0 && gamma >= 0.0 && beta + gamma <= 1)
			{
				info.t_hits.push_back(t);
				return true;
			}
			return false;
		}

		bool PlaneIntersect(const MyStructures::Plane& plane, MyStructures::PlaneIntersectionInfo& info) {
			bool hit = false;
			num::float3 p0, p1, p2;
			bool intersects = plane.PlaneSegmentIntersection(v0, v1, p0);
			if (intersects)
			{
				info.t_hits.push_back(p0);
				hit = true;

			}
			intersects = plane.PlaneSegmentIntersection(v1, v2, p1);
			if (intersects)
			{
				info.t_hits.push_back(p1);
				hit = true;
			}
			intersects = plane.PlaneSegmentIntersection(v2, v0, p2);
			if (intersects)
			{
				info.t_hits.push_back(p2);
				hit = true;
			}
			return hit;
		}
	};
}
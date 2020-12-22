#pragma once
#include <WindowsNumerics.h>
#include <vector>

namespace num = Windows::Foundation::Numerics;


const float machine_epsilon = std::numeric_limits<float>::epsilon() * 0.5;

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

		const num::float3& operator[](int i) const { if (i == 0) return pMin; else return pMax; };

		num::float3& operator[](int i) { if (i == 0) return pMin; else return pMax; };

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
	};

	class Primitive
	{
	protected:
		MyStructures::BBox bbox;
	public:
		Primitive() { bbox = MyStructures::BBox(); }
		void ComputeBBox() {}
		MyStructures::BBox BBox() { return bbox; }
		bool CloseIntersect(MyStructures::Ray& ray, MyStructures::RayIntersectionInfo& info) {
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

	class Sphere : Primitive
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

		bool CloseIntersect(MyStructures::Ray& ray, MyStructures::RayIntersectionInfo& info) {
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
					info.t_hits.insert(info.t_hits.begin(), t);
					return true;
				}
				t = -b + s_disc;
				if (ray.t_min < t && t < ray.t_max)
				{
					num::float3 n = (O + t * ray.direction) / radius;
					ray.t_max = t;
					info.normal = num::normalize(n);
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
					ray.t_max = t;
					info.t_hits.push_back(t);
					hit = true;
				}
			}
			return hit;
		}
	};


	class Triangle : Primitive
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

		bool CloseIntersect(MyStructures::Ray& ray, MyStructures::RayIntersectionInfo& info) {
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
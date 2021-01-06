#include "structs.h"

const static float machine_epsilon = std::numeric_limits<float>::epsilon() * 0.5;

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

Plane::Plane(const num::float3& x, const num::float3& n) 
{ 
	x_0 = x; 
	normal = num::normalize(n); 
};

float Plane::DistFromPlane(const num::float3& x) { return num::dot(normal, x - x_0); };

const float Plane::DistFromPlane(const num::float3& x) const { return num::dot(normal, x - x_0); };

bool Plane::OnPlane(const num::float3& x) { return abs(DistFromPlane(x)) < machine_epsilon; };

const bool Plane::OnPlane(const num::float3& x) const { return abs(DistFromPlane(x)) < machine_epsilon; };

bool Plane::PlaneSegmentIntersection(const num::float3& p_0, const num::float3& p_1, num::float3& p)
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

const bool Plane::PlaneSegmentIntersection(const num::float3& p_0, const num::float3& p_1, num::float3& p) const
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

BBox::BBox()
{
	pMin = num::float3(std::numeric_limits<float>::min());
	pMax = num::float3(std::numeric_limits<float>::max());
};

BBox::BBox(const num::float3& p) : pMin(p), pMax(p) { };

BBox::BBox(const num::float3& p0, const num::float3& p1)
{
	pMin = num::min(p0, p1);
	pMax = num::max(p0, p1);
}

num::float3 BBox::GetpMax()
{
	return pMax;
}

num::float3 BBox::GetpMin()
{
	return pMin;
}

const num::float3& BBox::operator[](const int i) const { if (i == 0) return pMin; else return pMax; };

num::float3& BBox::operator[](const int i) { if (i == 0) return pMin; else return pMax; };

num::float3 BBox::Corner(int corner) const
{
	return num::float3((*this)[(corner & 1)].x, (*this)[(corner & 2) ? 1 : 0].y, (*this)[(corner & 4) ? 1 : 0].z);
}

BBox BBox::Union(const BBox& b, const num::float3& p)
{
	return BBox(num::min(b.pMin, p), num::max(b.pMax, p));
}

BBox BBox::Union(const BBox& b0, const BBox& b1)
{
	return BBox(num::min(b0.pMin, b1.pMin), num::max(b0.pMax, b1.pMax));
}

BBox BBox::Intersect(const BBox& b0, const BBox& b1)
{
	return BBox(num::max(b0.pMin, b1.pMin), num::min(b0.pMax, b1.pMax));
}

bool BBox::Overlaps(const BBox& b0, const BBox& b1)
{
	bool x = (b0.pMax.x >= b1.pMin.x) && (b0.pMin.x <= b1.pMax.x);
	bool y = (b0.pMax.y >= b1.pMin.y) && (b0.pMin.y <= b1.pMax.y);
	bool z = (b0.pMax.z >= b1.pMin.z) && (b0.pMin.z <= b1.pMax.z);
	return (x && y && z);
}

bool BBox::Inside(const num::float3& p, const BBox& b)
{
	return (p.x >= b.pMin.x && p.x <= b.pMax.x && p.y >= b.pMin.y && p.y <= b.pMax.y && p.z >= b.pMin.z && p.z <= b.pMax.z);
}

BBox BBox::Expand(const BBox& b, float delta)
{
	return BBox(b.pMin - num::float3(delta), b.pMax + num::float3(delta));
}

num::float3 BBox::Diagonal() const { return pMax - pMin; }

float BBox::SurfaceArea() const { num::float3 d = Diagonal(); return 2 * (d.x * d.y + d.x * d.z + d.y * d.z); }

float BBox::Volume() const { num::float3 d = Diagonal(); return d.x * d.y * d.z; }

int BBox::MaximumExtent() const
{
	num::float3 d = Diagonal();
	if (d.x > d.y && d.x > d.z)
		return 0;
	else if (d.y > d.z)
		return 1;
	else
		return 2;
}

num::float3 BBox::Offset(const num::float3& p) const
{
	num::float3 o = p - pMin;
	if (pMax.x > pMin.x) o.x /= pMax.x - pMin.x;
	if (pMax.y > pMin.y) o.y /= pMax.y - pMin.y;
	if (pMax.z > pMin.z) o.z /= pMax.z - pMin.z;
	return o;
}

bool BBox::Intersect(const Ray& ray, float* hit_t0, float* hit_t1) const
{
	float t0 = 0, t1 = ray.GetMax();
	for (int i = 0; i < 3; ++i)
	{
		//update interval for ith bounding box slab
		float invRayDir = 1 / GetFloat3Component(ray.GetDirection(), i);
		float tNear = (GetFloat3Component(pMin, i) - GetFloat3Component(ray.GetOrigin(), i)) * invRayDir;
		float tFar = (GetFloat3Component(pMax, i) - GetFloat3Component(ray.GetOrigin(), i)) * invRayDir;
		//update parameteric interval from slab intersection t values
		if (tNear > tFar) std::swap(tNear, tFar);
		//update tFar to ensure robust ray-bounds intersection
		tFar *= 1 + 2 * gamma(3);
		t0 = tNear > t0 ? tNear : t0;
		t0 = tFar < t1 ? tFar : t1;
		if (t0 > t1) return false;
	}
	if (hit_t0)*hit_t0 = t0;
	if (hit_t1)*hit_t1 = t1;
	return true;
}

bool BBox::AnyIntersect(const Ray& ray, const num::float3& invDir, const int dirIsNeg[3]) const
{
	num::float3 ray_origin = ray.GetOrigin();
	const BBox& bounds = *this;
	float tMin = (bounds[dirIsNeg[0]].x - ray_origin.x) * invDir.x;
	float tMax = (bounds[1 - dirIsNeg[0]].x - ray_origin.x) * invDir.x;
	float tyMin = (bounds[dirIsNeg[1]].y - ray_origin.y) * invDir.y;
	float tyMax = (bounds[1 - dirIsNeg[1]].y - ray_origin.y) * invDir.y;
	tMax *= 1 + 2 * gamma(3);
	tyMax *= 1 + 2 * gamma(3);
	if (tMin > tyMax || tyMin > tMax)
		return false;
	if (tyMin > tMin) tMin = tyMin;
	if (tyMax < tMax) tMax = tyMax;
	float tzMin = (bounds[dirIsNeg[2]].z - ray_origin.z) * invDir.z;
	float tzMax = (bounds[1 - dirIsNeg[2]].z - ray_origin.z) * invDir.z;
	tzMax *= 1 + 2 * gamma(3);
	if (tMin > tzMax || tzMin > tMax)
		return false;
	if (tzMin > tMin) tMin = tzMin;
	if (tzMax < tMax) tMax = tzMax;
	return (tMin < ray.GetMax()) && (tMax > 0);
}

bool BBox::PlaneAnyIntersect(const Plane& plane) const
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
	return false;
}


Sphere::Sphere(const float r, const num::float3& c) { radius = r; center = c; ComputeBBox(); }
		
void Sphere::ComputeBBox()
{
	bbox = BBox(center - num::float3(radius), center + num::float3(radius));
	return;
};

bool Sphere::Intersect(Ray& ray, RayIntersectionInfo& info) {
	num::float3 ray_origin = ray.GetOrigin();
	num::float3 ray_direction = ray.GetDirection();
	num::float3 O = ray_origin - center;
	float b = num::dot(O, ray_direction);
	float c = num::dot(O, O) - radius * radius;
	float disc = b * b - c;
	if (disc > 0.0)
	{
		float s_disc = sqrt(disc);
		float t = -b - s_disc;
		if (ray.GetMin() < t && t < ray.GetMax())
		{
			num::float3 n = (O + t * ray_direction) / radius;
			ray.SetMax(t);
			info.SetNormal(num::normalize(n));
			info.AddClosestHit(t);
			return true;
		}
		t = -b + s_disc;
		if (ray.GetMin() < t && t < ray.GetMax())
		{
			num::float3 n = (O + t * ray_direction) / radius;
			ray.SetMax(t);
			info.SetNormal(num::normalize(n));
			info.AddClosestHit(t);
			return true;
		}
	}
	return false;
}

bool Sphere::AnyIntersect(Ray& ray) {
	num::float3 ray_origin = ray.GetOrigin();
	num::float3 ray_direction = ray.GetDirection();
	num::float3 O = ray_origin - center;
	float b = num::dot(O, ray_direction);
	float c = num::dot(O, O) - radius * radius;
	float disc = b * b - c;
	if (disc > 0.0)
	{
		float s_disc = sqrt(disc);
		float t = -b - s_disc;
		if (ray.GetMin() < t && t < ray.GetMax())
		{
			return true;
		}
		t = -b + s_disc;
		if (ray.GetMin() < t && t < ray.GetMax())
		{
			return true;
		}
	}
	return false;
}

bool Sphere::AllIntersect(Ray& ray, RayIntersectionInfo& info) {
	num::float3 ray_origin = ray.GetOrigin();
	num::float3 ray_direction = ray.GetDirection();
	num::float3 O = ray_origin - center;
	float b = num::dot(O, ray_direction);
	float c = num::dot(O, O) - radius * radius;
	float disc = b * b - c;
	bool hit = false;
	if (disc > 0.0)
	{
		float s_disc = sqrt(disc);
		float t = -b - s_disc;
		if (ray.GetMin() < t && t < ray.GetMax())
		{
			info.AddHit(t);
			hit = true;
		}
		t = -b + s_disc;
		if (ray.GetMin() < t && t < ray.GetMax())
		{
			info.AddHit(t);
			hit = true;
		}
	}
	return hit;
}


bool Sphere::PlaneIntersect(const Plane& plane, PlaneIntersectionInfo& info)
{
	return false;
}

Triangle::Triangle(const num::float3 p0, const num::float3 p1, const num::float3 p2)
{
	v0 = p0; v1 = p1; v2 = p2;
	ComputeBBox();
}

void Triangle::ComputeBBox()
{
	float area = num::length(num::cross(v1 - v0, v2 - v0));
	if (area > 0.0 && isfinite(area))
	{
		num::float3 b_min = num::min(num::min(v0, v1), v2);
		num::float3 b_max = num::max(num::max(v0, v1), v2);
		bbox = BBox(b_min, b_max);
	}
}

bool Triangle::Intersect(Ray& ray, RayIntersectionInfo& info) {
	num::float3 ray_origin = ray.GetOrigin();
	num::float3 ray_direction = ray.GetDirection();
	num::float3 e0 = v1 - v0;
	num::float3 e1 = v0 - v2;
	num::float3 n = num::cross(e1, e0);
	num::float3 e2 = 1.0 / num::dot(n, ray_direction) * (v0 - ray_origin);
	num::float3 i = num::cross(ray_direction, e2);
	float beta = num::dot(i, e1);
	float gamma = num::dot(i, e0);
	float t = num::dot(n, e2);
	if (ray.GetMax() > t && t > ray.GetMin() && beta > 0.0 && gamma >= 0.0 && beta + gamma <= 1)
	{
		ray.SetMax(t);
		info.SetNormal(num::normalize(n));
		info.AddClosestHit(t);
		return true;
	}
	return false;
}

bool Triangle::AnyIntersect(Ray& ray) {
	num::float3 ray_origin = ray.GetOrigin();
	num::float3 ray_direction = ray.GetDirection();
	num::float3 e0 = v1 - v0;
	num::float3 e1 = v0 - v2;
	num::float3 n = num::cross(e1, e0);
	num::float3 e2 = 1.0 / num::dot(n, ray_direction) * (v0 - ray_origin);
	num::float3 i = num::cross(ray_direction, e2);
	float beta = num::dot(i, e1);
	float gamma = num::dot(i, e0);
	float t = num::dot(n, e2);
	if (ray.GetMax() > t && t > ray.GetMin() && beta > 0.0 && gamma >= 0.0 && beta + gamma <= 1)
	{
		return true;
	}
	return false;
}

bool Triangle::AllIntersect(Ray& ray, RayIntersectionInfo& info) {
	num::float3 ray_origin = ray.GetOrigin();
	num::float3 ray_direction = ray.GetDirection();
	num::float3 e0 = v1 - v0;
	num::float3 e1 = v0 - v2;
	num::float3 n = num::cross(e1, e0);
	num::float3 e2 = 1.0 / num::dot(n, ray_direction) * (v0 - ray_origin);
	num::float3 i = num::cross(ray_direction, e2);
	float beta = num::dot(i, e1);
	float gamma = num::dot(i, e0);
	float t = num::dot(n, e2);
	if (ray.GetMax() > t && t > ray.GetMin() && beta > 0.0 && gamma >= 0.0 && beta + gamma <= 1)
	{
		info.AddHit(t);
		return true;
	}
	return false;
}

bool Triangle::PlaneIntersect(const Plane& plane, PlaneIntersectionInfo& info) {
	bool hit = false;
	num::float3 p0, p1, p2;
	bool intersects = plane.PlaneSegmentIntersection(v0, v1, p0);
	if (intersects)
	{
		info.AddHit(p0);
		hit = true;

	}
	intersects = plane.PlaneSegmentIntersection(v1, v2, p1);
	if (intersects)
	{
		info.AddHit(p1);
		hit = true;
	}
	intersects = plane.PlaneSegmentIntersection(v2, v0, p2);
	if (intersects)
	{
		info.AddHit(p2);
		hit = true;
	}
	return hit;
}
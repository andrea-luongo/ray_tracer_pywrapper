#include "structs.h"

const static float machine_epsilon = std::numeric_limits<float>::epsilon() * 0.5;

inline constexpr float gamma_error(int n)
{
	return (n * machine_epsilon) / (1 - n * machine_epsilon);
}

template <typename T> constexpr int sign(T x)
{
	return (T(0) < x) - (x < T(0));
}


// PLANE
Plane::Plane() 
{ 
	x_0 = float3(0); 
	normal = float3(0, 0, 1); 
};

Plane::Plane(const float3& x, const float3& n) 
{ 
	x_0 = x; 
	normal = float3::normalize(n); 
};

float Plane::DistFromPlane(const float3& x) const { return float3::dot(normal, x - x_0); };

bool Plane::OnPlane(const float3& x) const { return abs(DistFromPlane(x)) < machine_epsilon; };

bool Plane::PlaneSegmentIntersection(const float3& p_0, const float3& p_1, float3& p) const
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


//PLANE INTERSECTION INFO
std::vector<std::shared_ptr<Segment>> PlaneIntersectionInfo::GetSegments()
{
	std::vector<std::shared_ptr<Segment>> segments(t_hits.size() / 2);
	for (int idx = 0; idx < t_hits.size(); idx++)
	{
		segments[idx] = std::shared_ptr<Segment>(new Segment(t_hits[idx*2], t_hits[idx*2+1]));
	}
	return segments;
}


// BBOX
BBox::BBox()
{
	pMin = float3(std::numeric_limits<float>::max());
	pMax = -1 * float3(std::numeric_limits<float>::max());
};

BBox::BBox(const float3& p) : pMin(p), pMax(p) { };

BBox::BBox(const float3& p0, const float3& p1)
{
	pMin = float3::min(p0, p1);
	pMax = float3::max(p0, p1);
}

float3 BBox::GetpMax()
{
	return pMax;
}

float3 BBox::GetpMin()
{
	return pMin;
}

const float3& BBox::operator[](const int i) const { if (i == 0) return pMin; else return pMax; };

float3& BBox::operator[](const int i) { if (i == 0) return pMin; else return pMax; };

float3 BBox::Corner(int corner) const
{
	return float3((*this)[(corner & 1)].x, (*this)[(corner & 2) ? 1 : 0].y, (*this)[(corner & 4) ? 1 : 0].z);
}

BBox BBox::Union(const BBox& b, const float3& p)
{
	return BBox(float3::min(b.pMin, p), float3::max(b.pMax, p));
}

BBox BBox::Union(const BBox& b0, const BBox& b1)
{
	return BBox(float3::min(b0.pMin, b1.pMin), float3::max(b0.pMax, b1.pMax));
}

BBox BBox::Intersect(const BBox& b0, const BBox& b1)
{
	return BBox(float3::max(b0.pMin, b1.pMin), float3::min(b0.pMax, b1.pMax));
}

bool BBox::Overlaps(const BBox& b0, const BBox& b1)
{
	bool x = (b0.pMax.x >= b1.pMin.x) && (b0.pMin.x <= b1.pMax.x);
	bool y = (b0.pMax.y >= b1.pMin.y) && (b0.pMin.y <= b1.pMax.y);
	bool z = (b0.pMax.z >= b1.pMin.z) && (b0.pMin.z <= b1.pMax.z);
	return (x && y && z);
}

bool BBox::Inside(const float3& p, const BBox& b)
{
	return (p.x >= b.pMin.x && p.x <= b.pMax.x && p.y >= b.pMin.y && p.y <= b.pMax.y && p.z >= b.pMin.z && p.z <= b.pMax.z);
}

BBox BBox::Expand(const BBox& b, float delta)
{
	return BBox(b.pMin - float3(delta), b.pMax + float3(delta));
}

float3 BBox::Diagonal() const { return pMax - pMin; }

float BBox::SurfaceArea() const { float3 d = Diagonal(); return 2 * (d.x * d.y + d.x * d.z + d.y * d.z); }

float BBox::Volume() const { float3 d = Diagonal(); return d.x * d.y * d.z; }

int BBox::MaximumExtent() const
{
	float3 d = Diagonal();
	if (d.x >= d.y && d.x >= d.z)
		return 0;
	else if (d.y >= d.z)
		return 1;
	else
		return 2;
}

float3 BBox::Offset(const float3& p) const
{
	float3 o = p - pMin;
	if (pMax.x > pMin.x) o.x /= pMax.x - pMin.x;
	if (pMax.y > pMin.y) o.y /= pMax.y - pMin.y;
	if (pMax.z > pMin.z) o.z /= pMax.z - pMin.z;
	return o;
}

bool BBox::Intersect(const Ray& ray, float* hit_t0, float* hit_t1) const
{
	float t0 = 0, t1 = ray.GetMax();
	float3 ray_direction = ray.GetDirection();
	float3 ray_origin = ray.GetOrigin();
	for (int i = 0; i < 3; ++i)
	{
		//update interval for ith bounding box slab
		float invRayDir = 1 / ray_direction[i];
		float tNear = (pMin[i] - ray_origin[i]) * invRayDir;
		float tFar = (pMax[i] - ray_origin[i]) * invRayDir;
		//update parameteric interval from slab intersection t values
		if (tNear > tFar) std::swap(tNear, tFar);
		//update tFar to ensure robust ray-bounds intersection
		tFar *= 1 + 2 * gamma_error(3);
		t0 = tNear > t0 ? tNear : t0;
		t0 = tFar < t1 ? tFar : t1;
		if (t0 > t1) return false;
	}
	if (hit_t0)*hit_t0 = t0;
	if (hit_t1)*hit_t1 = t1;
	return true;
}

bool BBox::AnyIntersect(const Ray& ray, const float3& invDir, const int dirIsNeg[3]) const
{
	float3 ray_origin = ray.GetOrigin();
	const BBox& bounds = *this;
	float epsilon = 1e-6;
	//float epsilon = 0;
	float tMin = (bounds[dirIsNeg[0]].x - ray_origin.x - epsilon) * invDir.x;
	float tMax = (bounds[1 - dirIsNeg[0]].x - ray_origin.x + epsilon) * invDir.x;
	float tyMin = (bounds[dirIsNeg[1]].y - ray_origin.y - epsilon) * invDir.y;
	float tyMax = (bounds[1 - dirIsNeg[1]].y - ray_origin.y + epsilon) * invDir.y;
	tMax *= 1 + 2 * gamma_error(3);
	tyMax *= 1 + 2 * gamma_error(3);
	if (tMin > tyMax || tyMin > tMax)
		return false;
	if (tyMin > tMin) tMin = tyMin;
	if (tyMax < tMax) tMax = tyMax;
	float tzMin = (bounds[dirIsNeg[2]].z - ray_origin.z - epsilon) * invDir.z;
	float tzMax = (bounds[1 - dirIsNeg[2]].z - ray_origin.z + epsilon) * invDir.z;
	tzMax *= 1 + 2 * gamma_error(3);
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
	float3 p_2(pMin.x, pMin.y, pMax.z);
	float3 p_3(pMax.x, pMax.y, pMin.z);
	float d_2 = plane.DistFromPlane(p_2);
	float d_3 = plane.DistFromPlane(p_3);
	if (d_2 * d_3 < 0.0)
		return true;
	float3 p_4(pMin.x, pMax.y, pMax.z);
	float3 p_5(pMax.x, pMin.y, pMin.z);
	float d_4 = plane.DistFromPlane(p_4);
	float d_5 = plane.DistFromPlane(p_5);
	if (d_4 * d_5 < 0.0)
		return true;
	float3 p_6(pMin.x, pMax.y, pMin.z);
	float3 p_7(pMax.x, pMin.y, pMax.z);
	float d_6 = plane.DistFromPlane(p_6);
	float d_7 = plane.DistFromPlane(p_7);
	if (d_6 * d_7 < 0.0)
		return true;
	return false;
}

//SEGMENT
Segment::Segment(const float3& p0, const float3& p1) { v0 = p0; v1= p1; ComputeBBox(); }

void Segment::ComputeBBox()
{
	bbox = BBox(float3::min(v0, v1), float3::max(v0,v1));
	return;
}

bool Segment::Intersect(Ray& ray, RayIntersectionInfo& info)
{
	double3 s = v1 - v0;
	double3 e = v0 - ray.GetOrigin();
	double3 cross_dir_s = double3::cross(ray.GetDirection(), s);
	double3 cross_e_dir = double3::cross(e, ray.GetDirection());
	double x = double3::length(cross_dir_s);
	//double t = double3::length(double3::cross(e, s)) / x;
	double t = double3::dot(double3::cross(e, s), cross_dir_s) / (x*x);
	double u = double3::length(cross_e_dir) / x * sign(double3::dot(cross_dir_s, cross_e_dir));
	if (ray.GetMax() > t && t > ray.GetMin() && 1 > u && u >= 0.0f)
	{
		ray.SetMax(t);
		double3 n = double3::cross(double3::cross(ray.GetDirection(), s), s);
		info.SetNormal(double3::normalize(n));
		info.AddClosestHit(t);
		return true;
	}
	return false;
}

bool Segment::AnyIntersect(Ray& ray)
{
	double3 s = v1 - v0;
	double3 e = v0 - ray.GetOrigin();
	double3 cross_dir_s = double3::cross(ray.GetDirection(), s);
	double3 cross_e_dir = double3::cross(e, ray.GetDirection());
	double x = double3::length(cross_dir_s);
	//double t = double3::length(double3::cross(e, s)) / x;
	double t = double3::dot(double3::cross(e, s), cross_dir_s) / (x * x);
	double u = double3::length(cross_e_dir) / x * sign(double3::dot(cross_dir_s, cross_e_dir));
	if (ray.GetMax() > t && t > ray.GetMin() && 1 > u && u >= 0.0f)
	{
		return true;
	}
	return false;
}

bool Segment::AllIntersect(Ray& ray, RayIntersectionInfo& info)
{
	double epsilon = 1e-8;
	epsilon = 0.0;
	double3 s = v1 - v0;
	double3 e = v0 - ray.GetOrigin();
	double3 cross_dir_s = double3::cross(ray.GetDirection(), s);
	double3 cross_e_dir = double3::cross(e, ray.GetDirection());
	double x = double3::length(cross_dir_s);
	//double t = double3::length(double3::cross(e, s)) / x;
	double t = double3::dot(double3::cross(e, s), cross_dir_s) / (x * x);
	double u = double3::length(cross_e_dir) / x * sign(double3::dot(cross_dir_s, cross_e_dir));
	if (ray.GetMax() > t && t > ray.GetMin() && 1.0-epsilon > u && u >= 0.0f)
	{
		info.AddHit(t);
		if (info.GetHits()->size() == 3) {
			int tmp = 0;
		}
		return true;
	}
	return false;
	/*double epsilon = 1e-5;
	epsilon = 0.0;
	float3 s = v1 - v0;
	float3 e = v0 - ray.GetOrigin();
	float3 cross_dir_s = float3::cross(ray.GetDirection(), s);
	float3 cross_e_dir = float3::cross(e, ray.GetDirection());
	float x = float3::length(cross_dir_s);
	float t = float3::length(float3::cross(e, s)) / x;
	float u = double3::length(cross_e_dir) / x * sign(float3::dot(cross_dir_s, cross_e_dir));
	if (ray.GetMax() > t && t > ray.GetMin() && 1.0-epsilon > u && u >= 0.0f)
	{
		info.AddHit(t);
		if (info.GetHits()->size() == 3) {
			int tmp = 0;
		}
		return true;
	}
	return false;*/
}

bool Segment::PlaneIntersect(Plane& plane, PlaneIntersectionInfo& info)
{
	bool hit = false;
	float3 p0;
	bool intersects = plane.PlaneSegmentIntersection(v0, v1, p0);
	if (intersects)
	{
		info.AddHit(p0);
		hit = true;

	}
	return hit;
}

float3 Segment::operator[](int i) {
	if (i == 0)
		return v0;
	else if (i == 1)
		return v1;
	else
		return float3();
}

float3 Segment::operator[](int i) const {
	if (i == 0)
		return v0;
	else if (i == 1)
		return v1;
	else
		return float3();
}

std::ostream& operator<<(std::ostream& os, Segment const& s)
{
	os << "v0 " << s.v0 << " v1 " << s.v1;
	return os;
};

bool Segment::IntersectSegment(Segment& const s, float3& hit_point, float& t_hit)
{
	float3 ray_dir = v1 - v0;
	float3 ray_o = v0;
	Ray ray(ray_o, ray_dir, 0, 1, 0, 0);
	RayIntersectionInfo ray_info;
	bool hit = s.Intersect(ray, ray_info);
	if (hit)
	{
		t_hit = (*ray_info.GetHits())[0];
		hit_point = ray_o + t_hit * ray_dir;
	}
	return hit;
}

bool Segment::CompareSegments(Segment& s0, Segment& s1, float epsilon)
{
	bool result = false;
	float3 s_dist = float3::abs(s0.v1 - s1.v0);
	if (s_dist.length() <= epsilon)
	{
		result = true;
		//s0.v1 = s1.v0;
	}
	return result;
}

std::vector<std::vector<std::shared_ptr<Segment>>> Segment::SortSegments(std::vector<std::shared_ptr<Segment>>& segments, float const epsilon, bool remove_aligned_segments)
{
	std::vector<std::vector<std::shared_ptr<Segment>>> sorted_segments;
	if (segments.size() == 1)
	{
		sorted_segments.push_back(segments);
	}
	else
	{
		int half_idx = int(segments.size() * 0.5);
		std::vector<std::shared_ptr<Segment>> left_segments(segments.begin(), segments.begin() + half_idx);
		std::vector<std::shared_ptr<Segment>> right_segments(segments.begin() + half_idx, segments.end());
		auto left_loop = Segment::SortSegments(left_segments, epsilon, remove_aligned_segments);
		auto right_loop = Segment::SortSegments(right_segments, epsilon, remove_aligned_segments);
		std::vector<int> left_merged_idxs;
		std::vector<int> right_merged_idxs;
		//check if left loops can be connected with right loops
		for (int l_idx = 0; l_idx < left_loop.size(); l_idx++)
		{
			for (int r_idx = 0; r_idx < right_loop.size(); r_idx++)
			{
				if (std::find(right_merged_idxs.begin(), right_merged_idxs.end(), r_idx) != right_merged_idxs.end())
					continue;
				
				bool is_merged = Segment::MergeSegments(left_loop[l_idx], right_loop[r_idx], epsilon, remove_aligned_segments);
				if (is_merged)
				{
					left_merged_idxs.push_back(l_idx);
					right_merged_idxs.push_back(r_idx);
				}
			}
		}
		//add unconnected right loops to sorted segments
		for (int r_idx = 0; r_idx < right_loop.size(); r_idx++)
		{
			if (std::find(right_merged_idxs.begin(), right_merged_idxs.end(), r_idx) != right_merged_idxs.end())
				continue;
			sorted_segments.push_back(right_loop[r_idx]);
		}
		//check if left loops can be connected
		std::vector<int> merged_idxs;
		for (int idx_1 = 0; idx_1 < left_loop.size(); idx_1++)
		{
			for (int idx_2 = idx_1+1; idx_2 < left_loop.size(); idx_2++)
			{
				bool is_merged = Segment::MergeSegments(left_loop[idx_2], left_loop[idx_1], epsilon, remove_aligned_segments);
				if (is_merged)
				{
					merged_idxs.push_back(idx_1);
					break;
				}
			}
		}
		//add unconnected left loops to sorted segments
		for (int l_idx = 0; l_idx < left_loop.size(); l_idx++)
		{
			if (std::find(merged_idxs.begin(), merged_idxs.end(), l_idx) != merged_idxs.end())
				continue;
			sorted_segments.push_back(left_loop[l_idx]);
		}
	}
	return sorted_segments;
}

bool Segment::CheckAlignment(Segment& s0, Segment& s1, float const epsilon)
{
	float3 s_0_dir = (s0.v1 - s0.v0).normalize();
	float3 s_1_dir = (s1.v1 - s1.v0).normalize();
	float dot_pr = abs(float3::dot(s_0_dir, s_1_dir));
	bool is_aligned = false;
	if (dot_pr > 1 - epsilon)
	{
		is_aligned = true;
	}
	return is_aligned;
}

bool Segment::MergeSegments(std::vector<std::shared_ptr<Segment>>& s0, std::vector<std::shared_ptr<Segment>>& s1, float const epsilon, bool remove_aligned_segments)
{
	bool is_merged = false;
	std::shared_ptr<Segment> start_0 = (*s0.begin());
	std::shared_ptr<Segment> end_0 = (*(s0.end()-1));
	std::shared_ptr<Segment> start_1 = (*s1.begin());
	std::shared_ptr<Segment> end_1 = (*(s1.end() - 1));
	Segment start_1_flipped = Segment::FlipSegment(*start_1);
	Segment end_1_flipped = Segment::FlipSegment(*end_1);
	float alignment_epsilon = 1e-4;
	if (Segment::CompareSegments(*end_0, *start_1, epsilon))
	{
		end_0->v1 = start_1->v0;
		end_0->ComputeBBox();
		if (remove_aligned_segments && CheckAlignment(*end_0, *start_1, alignment_epsilon))
		{
			
			end_0->v1 = start_1->v1;
			end_0->ComputeBBox();
			s0.insert(s0.end(), s1.begin() + 1, s1.end());
		}
		else
		{
		s0.insert(s0.end(), s1.begin(), s1.end());
		}
		is_merged = true;
	}
	else if (Segment::CompareSegments(*end_1, *start_0, epsilon))
	{
		end_1->v1 = start_0->v0;
		end_1->ComputeBBox();
		if (remove_aligned_segments && CheckAlignment(*end_1, *start_0, alignment_epsilon))
		{
			start_0->v0 = end_1->v0;
			start_0->ComputeBBox();
			s0.insert(s0.begin(), s1.begin(), s1.end() - 1);
		}
		else
		{
			s0.insert(s0.begin(), s1.begin(), s1.end());
		}
		is_merged = true;
	}
	else if (Segment::CompareSegments(*end_0, end_1_flipped, epsilon))
	{
		end_0->v1 = end_1_flipped.v0;
		end_0->ComputeBBox();
		std::rotate(s1.begin(), s1.end() - 1, s1.end());
		for (auto s : s1)
			s->FlipSegment();
		if (remove_aligned_segments && CheckAlignment(*end_0, end_1_flipped, alignment_epsilon))
		{
			end_0->v1 = end_1_flipped.v1;
			end_0->ComputeBBox();
			s0.insert(s0.end(), s1.begin() + 1, s1.end());
		}
		else
		{
			s0.insert(s0.end(), s1.begin(), s1.end());
		}
		is_merged = true;
	}
	else if (Segment::CompareSegments(start_1_flipped, *start_0, epsilon))
	{
		start_1->v0 = start_0->v0;
		start_1->ComputeBBox();
		std::rotate(s1.begin(), s1.end() - 1, s1.end());
		for (auto s : s1)
			s->FlipSegment();

		if (remove_aligned_segments && CheckAlignment(start_1_flipped, *start_0, alignment_epsilon))
		{
			start_0->v0 = start_1_flipped.v0;
			start_0->ComputeBBox();
			s0.insert(s0.begin(), s1.begin(), s1.end() - 1);
		}
		else
		{
			s0.insert(s0.begin(), s1.begin(), s1.end());
		}
		is_merged = true;
	}
	//check if start and end are aligned, if yes, remove last segment
	if (remove_aligned_segments)
	{
		if (is_merged && s0.size() > 1 && (*s0.begin())->v0 == (*(s0.end() - 1))->v1)
		{
			if (CheckAlignment(*(*s0.begin()), *(*(s0.end() - 1)), alignment_epsilon))
			{
				(*s0.begin())->v0 = (*(s0.end() - 1))->v0;
				(*s0.begin())->ComputeBBox();
				s0.pop_back();
			}
		}
	}
	return is_merged;
}



//SPHERE
Sphere::Sphere(const float r, const float3& c) { radius = r; center = c; ComputeBBox(); }
		
void Sphere::ComputeBBox()
{
	bbox = BBox(center - float3(radius), center + float3(radius));
	return;
};

bool Sphere::Intersect(Ray& ray, RayIntersectionInfo& info) {
	float3 ray_origin = ray.GetOrigin();
	float3 ray_direction = ray.GetDirection();
	float3 O = ray_origin - center;
	float b = float3::dot(O, ray_direction);
	float c = float3::dot(O, O) - radius * radius;
	float disc = b * b - c;
	if (disc > 0.0)
	{
		float s_disc = sqrt(disc);
		float t = -b - s_disc;
		if (ray.GetMin() < t && t < ray.GetMax())
		{
			float3 n = (O + t * ray_direction) / radius;
			ray.SetMax(t);
			info.SetNormal(float3::normalize(n));
			info.AddClosestHit(t);
			return true;
		}
		t = -b + s_disc;
		if (ray.GetMin() < t && t < ray.GetMax())
		{
			float3 n = (O + t * ray_direction) / radius;
			ray.SetMax(t);
			info.SetNormal(float3::normalize(n));
			info.AddClosestHit(t);
			return true;
		}
	}
	return false;
}

bool Sphere::AnyIntersect(Ray& ray) {
	float3 ray_origin = ray.GetOrigin();
	float3 ray_direction = ray.GetDirection();
	float3 O = ray_origin - center;
	float b = float3::dot(O, ray_direction);
	float c = float3::dot(O, O) - radius * radius;
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
	float3 ray_origin = ray.GetOrigin();
	float3 ray_direction = ray.GetDirection();
	float3 O = ray_origin - center;
	float b = float3::dot(O, ray_direction);
	float c = float3::dot(O, O) - radius * radius;
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


bool Sphere::PlaneIntersect(Plane& plane, PlaneIntersectionInfo& info)
{
	return false;
}

//TRIANGLE
Triangle::Triangle(const float3 p0, const float3 p1, const float3 p2)
{
	v0 = p0; v1 = p1; v2 = p2;
	ComputeBBox();
}

void Triangle::ComputeBBox()
{
	float area = float3::length(float3::cross(v1 - v0, v2 - v0));
	if (area > 0.0 && isfinite(area))
	{
		float3 b_min = float3::min(float3::min(v0, v1), v2);
		float3 b_max = float3::max(float3::max(v0, v1), v2);
		bbox = BBox(b_min, b_max);
	}
}

bool Triangle::Intersect(Ray& ray, RayIntersectionInfo& info) {
	double3 ray_origin = ray.GetOrigin();
	double3 ray_direction = ray.GetDirection();
	double3 e0 = v1 - v0;
	double3 e1 = v0 - v2;
	double3 n = double3::cross(e1, e0);
	double3 e2 = 1.0 / double3::dot(n, ray_direction) * (double3(v0) - ray_origin);
	double3 i = double3::cross(ray_direction, e2);
	double beta = double3::dot(i, e1);
	double gamma = double3::dot(i, e0);
	float t = float3::dot(n, e2);
	if (ray.GetMax() > t && t > ray.GetMin() && beta > 0.0 && gamma >= 0.0 && beta + gamma <= 1)
	{
		ray.SetMax(t);
		info.SetNormal(float3::normalize(n));
		info.AddClosestHit(t);
		return true;
	}
	return false;
}

bool Triangle::AnyIntersect(Ray& ray) {
	double3 ray_origin = ray.GetOrigin();
	double3 ray_direction = ray.GetDirection();
	double3 e0 = v1 - v0;
	double3 e1 = v0 - v2;
	double3 n = double3::cross(e1, e0);
	double3 e2 = 1.0 / double3::dot(n, ray_direction) * (double3(v0) - ray_origin);
	double3 i = double3::cross(ray_direction, e2);
	double beta = double3::dot(i, e1);
	double gamma = double3::dot(i, e0);
	float t = float3::dot(n, e2);
	if (ray.GetMax() > t && t > ray.GetMin() && beta > 0.0 && gamma >= 0.0 && beta + gamma <= 1)
	{
		return true;
	}
	return false;
}

bool Triangle::AllIntersect(Ray& ray, RayIntersectionInfo& info) {
	double3 ray_origin = ray.GetOrigin();
	double3 ray_direction = ray.GetDirection();
	double3 e0 = v1 - v0;
	double3 e1 = v0 - v2;
	double3 n = double3::cross(e1, e0);
	double3 e2 = 1.0 / double3::dot(n, ray_direction) * (double3(v0) - ray_origin);
	double3 i = double3::cross(ray_direction, e2);
	double beta = double3::dot(i, e1);
	double gamma = double3::dot(i, e0);
	float t = float3::dot(n, e2);
	if (ray.GetMax() > t && t > ray.GetMin() && beta > 0.0 && gamma >= 0.0 && beta + gamma <= 1)
	{
		info.AddHit(t);
		return true;
	}
	return false;
}

bool Triangle::PlaneIntersect(Plane& plane, PlaneIntersectionInfo& info) {
	bool hit = false;
	float3 p0, p1, p2;
	std::vector<float3> t_hits;
	bool intersects = plane.PlaneSegmentIntersection(v0, v1, p0);
	if (intersects)
	{
		t_hits.push_back(p0);
		hit = true;

	}
	intersects = plane.PlaneSegmentIntersection(v1, v2, p1);
	if (intersects)
	{
		t_hits.push_back(p1);
		hit = true;
	}
	intersects = plane.PlaneSegmentIntersection(v2, v0, p2);
	if (intersects)
	{
		t_hits.push_back(p2);
		hit = true;
	}
	if (hit)
	{
		float3 dir = t_hits[1] - t_hits[0];
		if (float3::length(dir) == 0)
		{
			return false;
		}
		double3 e0 = v1 - v0;
		double3 e1 = v0 - v2;
		double3 n = double3::cross(e1, e0);
		float s = sign(float3::dot(float3::cross(n, dir), plane.GetNormal()));
		if (s < 0) 
		{
			info.AddHit(t_hits[1]);
			info.AddHit(t_hits[0]);
		}
		else
		{
			info.AddHit(t_hits[0]);
			info.AddHit(t_hits[1]);
		}
	
	}
	return hit;
}

float3 Triangle::operator[](int i) {
	if (i == 0)
		return v0;
	else if (i == 1)
		return v1;
	else
		return v2;
}

float3 Triangle::operator[](int i) const{
	if (i == 0)
		return v0;
	else if (i == 1)
		return v1;
	else
		return v2;
}

//
//IntTriangle::IntTriangle(const int3 p0, const int3 p1, const int3 p2)
//{
//	v0 = p0; v1 = p1; v2 = p2;
//	ComputeBBox();
//}
//
//void IntTriangle::ComputeBBox()
//{
//	float area = float3::length(float3::cross(v1 - v0, v2 - v0));
//	if (area > 0.0 && isfinite(area))
//	{
//		float3 b_min = float3::min(float3::min(v0, v1), v2);
//		float3 b_max = float3::max(float3::max(v0, v1), v2);
//		bbox = BBox(b_min, b_max);
//	}
//}
//
//bool IntTriangle::Intersect(Ray& ray, RayIntersectionInfo& info) {
//	double3 ray_origin = ray.GetOrigin();
//	double3 ray_direction = ray.GetDirection();
//	double3 e0 = v1 - v0;
//	double3 e1 = v0 - v2;
//	double3 n = double3::cross(e1, e0);
//	double3 e2 = 1.0 / double3::dot(n, ray_direction) * (double3(v0) - ray_origin);
//	double3 i = double3::cross(ray_direction, e2);
//	double beta = double3::dot(i, e1);
//	double gamma = double3::dot(i, e0);
//	float t = float3::dot(n, e2);
//	if (ray.GetMax() > t&& t > ray.GetMin() && beta > 0.0 && gamma >= 0.0 && beta + gamma <= 1)
//	{
//		ray.SetMax(t);
//		info.SetNormal(float3::normalize(n));
//		info.AddClosestHit(t);
//		return true;
//	}
//	return false;
//}
//
//bool IntTriangle::AnyIntersect(Ray& ray) {
//	double3 ray_origin = ray.GetOrigin();
//	double3 ray_direction = ray.GetDirection();
//	double3 e0 = v1 - v0;
//	double3 e1 = v0 - v2;
//	double3 n = double3::cross(e1, e0);
//	double3 e2 = 1.0 / double3::dot(n, ray_direction) * (double3(v0) - ray_origin);
//	double3 i = double3::cross(ray_direction, e2);
//	double beta = double3::dot(i, e1);
//	double gamma = double3::dot(i, e0);
//	float t = float3::dot(n, e2);
//	if (ray.GetMax() > t&& t > ray.GetMin() && beta > 0.0 && gamma >= 0.0 && beta + gamma <= 1)
//	{
//		return true;
//	}
//	return false;
//}
//
//bool IntTriangle::AllIntersect(Ray& ray, RayIntersectionInfo& info) {
//	double3 ray_origin = ray.GetOrigin();
//	double3 ray_direction = ray.GetDirection();
//	double3 e0 = v1 - v0;
//	double3 e1 = v0 - v2;
//	double3 n = double3::cross(e1, e0);
//	double3 e2 = 1.0 / double3::dot(n, ray_direction) * (double3(v0) - ray_origin);
//	double3 i = double3::cross(ray_direction, e2);
//	double beta = double3::dot(i, e1);
//	double gamma = double3::dot(i, e0);
//	float t = float3::dot(n, e2);
//	if (ray.GetMax() > t&& t > ray.GetMin() && beta > 0.0 && gamma >= 0.0 && beta + gamma <= 1)
//	{
//		info.AddHit(t);
//		return true;
//	}
//	return false;
//}
//
//bool IntTriangle::PlaneIntersect(Plane& plane, PlaneIntersectionInfo& info) {
//	bool hit = false;
//	float3 p0, p1, p2;
//	std::vector<float3> t_hits;
//	bool intersects = plane.PlaneSegmentIntersection(v0, v1, p0);
//	if (intersects)
//	{
//		t_hits.push_back(p0);
//		hit = true;
//
//	}
//	intersects = plane.PlaneSegmentIntersection(v1, v2, p1);
//	if (intersects)
//	{
//		t_hits.push_back(p1);
//		hit = true;
//	}
//	intersects = plane.PlaneSegmentIntersection(v2, v0, p2);
//	if (intersects)
//	{
//		t_hits.push_back(p2);
//		hit = true;
//	}
//	if (hit)
//	{
//		float3 dir = t_hits[1] - t_hits[0];
//		double3 e0 = v1 - v0;
//		double3 e1 = v0 - v2;
//		double3 n = double3::cross(e1, e0);
//		float s = sign(float3::dot(float3::cross(n, dir), plane.GetNormal()));
//		if (s < 0)
//		{
//			info.AddHit(t_hits[1]);
//			info.AddHit(t_hits[0]);
//		}
//		else
//		{
//			info.AddHit(t_hits[0]);
//			info.AddHit(t_hits[1]);
//		}
//
//	}
//
//	//sign(float3::dot(cross_dir_s, cross_e_dir));
//	return hit;
//}
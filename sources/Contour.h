#pragma once
#ifdef RAYTRACERDLL_EXPORTS
#define RAYTRACERDLL_API __declspec(dllexport)
#else
#define RAYTRACERDLL_API __declspec(dllimport)
#endif
#include "structs.h"
#include "BVH.h"
#include <mutex>
#include <set>
#include <map>


struct ContourSelfIntersectionPoint
{
	float3 hit_point;
	float t_hit;
	int idx_0;
	int idx_1;
	int id;

	ContourSelfIntersectionPoint()
	{
		hit_point = float3();
		t_hit = -1;
		idx_0 = -1;
		idx_1 = -1;
		id = -1;
	}

	ContourSelfIntersectionPoint(float3 p, float t, int i0, int i1, int i)
	{
		hit_point = p;
		t_hit = t;
		idx_0 = i0;
		idx_1 = i1;
		id = i;
		
	}
	bool operator< (const ContourSelfIntersectionPoint& p)
	{
		//if (idx_0 == p.idx_0)
			return t_hit < p.t_hit;
	/*	else
			return idx_0 < p.idx_0;*/
	}
	bool operator== (const ContourSelfIntersectionPoint& p)
	{
		//bool result = true;
		//if (hit_point != p.hit_point || t_hit != p.t_hit || idx_0 != p.idx_0 || idx_1 != p.idx_1)
		//	result = false;
		//return result;
		return id == p.id;
	}
};

class Contour : public Primitive
{
public:
	std::vector<std::shared_ptr<Segment>> segments;
	BVH* bvh = nullptr;
	bool is_valid = true;
	float3 contour_normal;
	bool contour_orientation;

public:
	RAYTRACERDLL_API Contour();
	RAYTRACERDLL_API Contour(const std::vector<std::shared_ptr<Segment>>& p, const float3 n);
	RAYTRACERDLL_API void ComputeBBox();
	RAYTRACERDLL_API void UpdateSegments(std::vector<std::shared_ptr<Segment>> new_segments);
	RAYTRACERDLL_API bool CheckValidity();
	RAYTRACERDLL_API bool Intersect(Ray& ray, RayIntersectionInfo& info);
	RAYTRACERDLL_API bool AnyIntersect(Ray& ray);
	RAYTRACERDLL_API bool AllIntersect(Ray& ray, RayIntersectionInfo& info); 
	RAYTRACERDLL_API bool PlaneIntersect(Plane& plane, PlaneIntersectionInfo& info);
	RAYTRACERDLL_API bool IsContained(Contour& contour_b, float& t_hit);
	RAYTRACERDLL_API bool Contains(Contour& contour_b, float& t_hit);
	RAYTRACERDLL_API static int EvaluateContoursRelationship(Contour& contour_a, Contour& contour_b, float& t_hit);
	RAYTRACERDLL_API bool OffsetContour(float offset, Contour& new_c);
	RAYTRACERDLL_API bool RemoveSelfIntersections(std::vector<std::shared_ptr<Contour>>& new_contours, bool keep_clockwise);
	RAYTRACERDLL_API void RemoveAlignedSegments(float alignment_epsilon);
	RAYTRACERDLL_API void RemoveShortSegments(float min_length);
	RAYTRACERDLL_API std::vector<std::vector<float3>> MultiRayAllIntersects(float laser_width_microns, float density, float overlap, float rot_angle_deg, bool verbose);
	RAYTRACERDLL_API bool FindSelfIntersections(std::vector<ContourSelfIntersectionPoint>& contour_intersection_points, std::map<int, std::vector<ContourSelfIntersectionPoint>>& contour_intersection_dict);
private:
	RAYTRACERDLL_API void ComputeContourOrientation();


};
RAYTRACERDLL_API std::ostream& operator<<(std::ostream& os, Contour const& c);


class ContourNode : public std::enable_shared_from_this<ContourNode>
{
public:
	std::shared_ptr<Contour> contour = nullptr;
	std::set<std::shared_ptr<ContourNode>> children_set;
	std::weak_ptr<ContourNode> parent;
	int node_id = 0;
	int depth = 0;
public:
	RAYTRACERDLL_API ~ContourNode();
	RAYTRACERDLL_API ContourNode();
	RAYTRACERDLL_API ContourNode(std::shared_ptr<Contour> c);
	RAYTRACERDLL_API ContourNode(int id);
	RAYTRACERDLL_API ContourNode(std::shared_ptr<Contour> c, int id);

	RAYTRACERDLL_API void SetNodeID(int id) { node_id = id; };
	RAYTRACERDLL_API void AddChild(std::shared_ptr<ContourNode> c);
	RAYTRACERDLL_API void RemoveChild(std::shared_ptr<ContourNode> c);
	RAYTRACERDLL_API void UpdateChildrenDepth();

	RAYTRACERDLL_API std::vector<std::shared_ptr<Contour>> GetChildrenContours();
	RAYTRACERDLL_API std::vector<std::shared_ptr<ContourNode>> GetDescendants();
	RAYTRACERDLL_API std::vector<std::shared_ptr<ContourNode>> GetAncestors();
	RAYTRACERDLL_API std::vector<std::shared_ptr<ContourNode>> GetChildren();

	RAYTRACERDLL_API bool operator==(const ContourNode& other) {
		return node_id == other.node_id;
	};
	RAYTRACERDLL_API bool operator!=(const ContourNode& other) {
		return !(*this == other);
	};
	RAYTRACERDLL_API bool operator<(const ContourNode& other) {
		return node_id < other.node_id;
	};
};


class ContourTree
{
public:
	std::vector<std::shared_ptr<Contour>> contours;
	std::shared_ptr<ContourNode> tree_root;
	BBox bbox;
	BVH* root_bvh;
	std::vector<std::shared_ptr<BVH>> tree_global_bvsh;
	std::vector<std::shared_ptr<BVH>> internal_bvhs;
	int node_id_counter = 0;

public:
	RAYTRACERDLL_API ContourTree(std::vector<std::shared_ptr<Contour>> c);
	RAYTRACERDLL_API ContourTree();
	RAYTRACERDLL_API bool Intersect(Ray& ray, RayIntersectionInfo& info);
	RAYTRACERDLL_API bool AnyIntersect(Ray& ray);
	RAYTRACERDLL_API bool AllIntersect(Ray& ray, RayIntersectionInfo& info);
	RAYTRACERDLL_API std::vector<std::vector<std::vector<float3>>> MultiRayAllIntersects(float laser_width_microns, float density, float overlap, float rot_angle_deg, bool verbose);
	RAYTRACERDLL_API BBox GetBBox();
	//RAYTRACERDLL_API ContourTree OffsetContourTree(float offset);
	RAYTRACERDLL_API bool OffsetContourTree(float offset, ContourTree& new_tree);
private:
	RAYTRACERDLL_API void BuildTree();
	RAYTRACERDLL_API void CheckChildren(std::shared_ptr<ContourNode> n, std::vector<std::shared_ptr<ContourNode>> children);
	RAYTRACERDLL_API void CheckParents(std::shared_ptr<ContourNode>& n, std::shared_ptr<ContourNode>& p);
	RAYTRACERDLL_API void BuildRootBVH();
	RAYTRACERDLL_API void BuildInternalBVHs();
	RAYTRACERDLL_API void BuildTreeGlobalBVH();
};


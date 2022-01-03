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


class Contour : public Primitive
{
public:
	std::vector<std::shared_ptr<Segment>> segments;
	BVH* bvh;
	bool is_valid = true;
	float3 contour_normal;
public:
	RAYTRACERDLL_API Contour();
	RAYTRACERDLL_API Contour(const std::vector<std::shared_ptr<Segment>>& p, const float3 n);
	RAYTRACERDLL_API void ComputeBBox();
	RAYTRACERDLL_API bool Intersect(Ray& ray, RayIntersectionInfo& info);
	RAYTRACERDLL_API bool AnyIntersect(Ray& ray);
	RAYTRACERDLL_API bool AllIntersect(Ray& ray, RayIntersectionInfo& info);
	RAYTRACERDLL_API bool PlaneIntersect(Plane& plane, PlaneIntersectionInfo& info);
	RAYTRACERDLL_API bool IsContained(Contour& contour_b, float& t_hit);
	RAYTRACERDLL_API bool Contains(Contour& contour_b, float& t_hit);
	RAYTRACERDLL_API static int EvaluateContoursRelationship(Contour& contour_a, Contour& contour_b, float& t_hit);
	RAYTRACERDLL_API Contour OffsetContour(float offset);
};


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
	BVH* root_bvh;
	std::vector<std::shared_ptr<BVH>> tree_global_bvsh;
	std::vector<std::vector<std::shared_ptr<BVH>>> tree_individual_bvhs;
	int node_id_counter = 0;

public:
	RAYTRACERDLL_API ContourTree(std::vector<std::shared_ptr<Contour>> c);
private:
	RAYTRACERDLL_API void BuildTree();
	RAYTRACERDLL_API void CheckChildren(std::shared_ptr<ContourNode> n, std::vector<std::shared_ptr<ContourNode>> children);
	RAYTRACERDLL_API void CheckParents(std::shared_ptr<ContourNode>& n, std::shared_ptr<ContourNode>& p);
	RAYTRACERDLL_API void BuildRootBVH();
	RAYTRACERDLL_API void BuildTreeIndividualBVH();
	RAYTRACERDLL_API void BuildTreeGlobalBVH();
};


class TestPointer
{
public:
	std::shared_ptr<TestPointer> little_brother;
	std::weak_ptr<TestPointer> big_brother;
	bool is_big_brother = true;

public:
	RAYTRACERDLL_API TestPointer() { };
	RAYTRACERDLL_API ~TestPointer() {
		big_brother.reset(); 
	};
	RAYTRACERDLL_API void SetBigBrother(std::shared_ptr<TestPointer> bb)
	{
		is_big_brother = false;
		//bb->little_brother = std::shared_ptr<TestPointer>(this);
		big_brother = bb;
		
	};
	RAYTRACERDLL_API void SetLittleBrother(std::shared_ptr<TestPointer> lb)
	{
		is_big_brother = true;
		little_brother = lb;
	};
};
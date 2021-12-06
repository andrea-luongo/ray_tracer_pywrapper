#pragma once
#ifdef RAYTRACERDLL_EXPORTS
#define RAYTRACERDLL_API __declspec(dllexport)
#else
#define RAYTRACERDLL_API __declspec(dllimport)
#endif
#include "structs.h"
#include "BVH.h"
#include <mutex>

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

struct ContourNode
{
public:
	std::shared_ptr<Contour> contour;
	std::vector<std::shared_ptr<ContourNode>> children;
	std::shared_ptr<ContourNode> parent;
	int depth = 0;
public:
	ContourNode();
	ContourNode(std::shared_ptr<Contour> c);
	ContourNode(std::shared_ptr<Contour> c, std::shared_ptr<ContourNode> p);
	void SetChildren(std::vector<std::shared_ptr<ContourNode>> c);
	void SetParent(std::shared_ptr<ContourNode> p);
	void AddChild(std::shared_ptr<ContourNode> c);
	std::vector<std::shared_ptr<Contour>> GetChildrenContours();
	std::vector<std::shared_ptr<ContourNode>> GetDescendants();
	std::vector<std::shared_ptr<ContourNode>> GetAncestors();
};

class ContourTree
{
public:
	std::vector<std::shared_ptr<Contour>> contours;
	std::shared_ptr<ContourNode> tree_root;
	BVH* root_bvh;
	std::vector<std::shared_ptr<BVH>> tree_global_bvsh;
	std::vector<std::vector<std::shared_ptr<BVH>>> tree_individual_bvhs;

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

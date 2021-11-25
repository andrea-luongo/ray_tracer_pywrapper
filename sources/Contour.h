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
	BVH bvh;
	bool is_valid;
public:
	RAYTRACERDLL_API Contour(const std::vector<std::shared_ptr<Segment>>& p);
	RAYTRACERDLL_API void ComputeBBox();
	RAYTRACERDLL_API bool Intersect(Ray& ray, RayIntersectionInfo& info);
	RAYTRACERDLL_API bool AnyIntersect(Ray& ray);
	RAYTRACERDLL_API bool AllIntersect(Ray& ray, RayIntersectionInfo& info);
	RAYTRACERDLL_API bool PlaneIntersect(const Plane& plane, PlaneIntersectionInfo& info);
	RAYTRACERDLL_API bool IsContained(Contour contour_b);
	RAYTRACERDLL_API bool Contains(Contour contour_b);
	RAYTRACERDLL_API static int EvaluateContoursRelationship(Contour contour_a, Contour contour_b);
};

struct ContourNode
{
	std::shared_ptr<Contour> contour;
	std::vector<std::shared_ptr<ContourNode>> children;
	std::shared_ptr<ContourNode> parent;
};

class ContourTree
{
public:
	std::vector<std::shared_ptr<Contour>> contour;
	ContourNode tree_root;
	BVH root_bvh;
	BVH tree_global_bvsh;
	BVH tree_individual_bvhs;

private:
	RAYTRACERDLL_API ContourTree(std::vector<std::shared_ptr<Contour>> c);
	RAYTRACERDLL_API void BuildTree();
	RAYTRACERDLL_API void CheckChildren();
	RAYTRACERDLL_API void CheckParents();
	RAYTRACERDLL_API void BuildRootBVH();
	RAYTRACERDLL_API void BuildTreeIndividualBVH();
	RAYTRACERDLL_API void BuildTreeGlobalBVH();
};

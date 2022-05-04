#pragma once
#ifdef RAYTRACERDLL_EXPORTS
#define RAYTRACERDLL_API __declspec(dllexport)
#else
#define RAYTRACERDLL_API __declspec(dllimport)
#endif
#include "structs.h"
#include <mutex>

enum class RAYTRACERDLL_API SplitMethod{ SAH, HLBVH, Middle, EqualCounts};

struct BVHPrimitiveInfo
{
	size_t primitiveNumber;
	BBox bounds;
	float3 centroid;
	BVHPrimitiveInfo() {};
	BVHPrimitiveInfo(size_t pn, const BBox& b)
	{
		primitiveNumber = pn;
		bounds = b;
		centroid = 0.5f * bounds.GetpMin() + 0.5f * bounds.GetpMax();
	};
};

struct BVHBuildNode
{
	BBox bounds;
	BVHBuildNode* children[2];
	int splitAxis, firstPrimOffset, nPrimitives;
	void initLeaf(int first, int n, const BBox& b)
	{
		firstPrimOffset = first;
		nPrimitives = n;
		bounds = b;
		children[0] = children[1] = nullptr;
	}
	void initInterior(int axis, BVHBuildNode* c0, BVHBuildNode* c1)
	{
		children[0] = c0;
		children[1] = c1;
		bounds = BBox::Union(c0->bounds, c1->bounds);
		splitAxis = axis;
		nPrimitives = 0;
	}
};

struct LinearBVHNode
{
	BBox bounds;
	union {
		int primitivesOffset;
		int secondChildOffset;
	};
	uint32_t nPrimitives;
	uint8_t axis;
	uint8_t pad[3];
};

class BVH
{
private:
	const int maxPrimsInNode;
	const SplitMethod splitMethod;
	LinearBVHNode* nodes = nullptr;

public:
	std::vector<std::shared_ptr<Primitive>> primitives;
	RAYTRACERDLL_API BVH(const std::vector<std::shared_ptr<Primitive>>& p, SplitMethod splitMethod, int maxPrims =255);
	RAYTRACERDLL_API ~BVH() {  };
	RAYTRACERDLL_API bool intersect(Ray& ray, RayIntersectionInfo& info);
	RAYTRACERDLL_API bool any_intersect(Ray& ray);
	RAYTRACERDLL_API bool all_intersects(Ray& ray, RayIntersectionInfo& info);
	RAYTRACERDLL_API bool plane_all_intersects(Plane& plane, PlaneIntersectionInfo& info);
	RAYTRACERDLL_API BBox getBVHBBox() { return nodes[0].bounds; };
protected:
	//BVHBuildNode* HLBVHBuild(const std::vector<BVHPrimitiveInfo>& primitiveInfo, int* totalNodes, std::vector<std::shared_ptr<Primitive>>& orderedPrims);
	BVHBuildNode* recursiveBuild(std::vector<BVHPrimitiveInfo>& primitiveInfo, int start, int end, int* totalNodes, std::vector<std::shared_ptr<Primitive>>& orderedPrims);
	BVHBuildNode* createLeafBVHNode(std::vector<BVHPrimitiveInfo>& primitiveInfo, int start, int end, std::vector<std::shared_ptr<Primitive>>& orderedPrims, BBox& bounds);
	bool middlePointSplit(std::vector<BVHPrimitiveInfo>& primitiveInfo, int start, int end, int dim, BBox& centroidBounds, int& mid);
	bool equalCountsSplit(std::vector<BVHPrimitiveInfo>& primitiveInfo, int start, int end, int dim, int& mid);
	bool SAHSplit(std::vector<BVHPrimitiveInfo>& primitiveInfo, int start, int end, int dim, BBox& bounds, BBox& centroidBounds, int& mid);
	int flattenBVHTree(BVHBuildNode* node, int* offset);
};
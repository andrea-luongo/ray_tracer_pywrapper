#pragma once
#include "structs.h"
#include <mutex>

enum class SplitMethod{ SAH, HLBVH, Middle, EqualCounts};

struct BVHPrimitiveInfo
{
	size_t primitiveNumber;
	MyStructures::BBox bounds;
	num::float3 centroid;
	BVHPrimitiveInfo(size_t primitiveNumber, const MyStructures::BBox& bounds) : primitiveNumber(primitiveNumber), bounds(bounds), centroid(0.5f * bounds.pMin + 0.5f * bounds.pMax) {};
};

struct BVHBuildNode
{
	MyStructures::BBox bounds;
	BVHBuildNode* children[2];
	int splitAxis, firstPrimOffset, nPrimitives;
	void initLeaf(int first, int n, const MyStructures::BBox& b)
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
		bounds = MyStructures::BBox::Union(c0->bounds, c1->bounds);
		splitAxis = axis;
		nPrimitives = 0;
	}
};

class BVH
{
private:
	std::vector<std::shared_ptr<MyStructures::Primitive>> primitives;
	const int maxPrimsInNode;
	const SplitMethod splitMethod;
public:
	BVH(const std::vector<std::shared_ptr<MyStructures::Primitive>>& p, int maxPrimsInNode, SplitMethod splitMethod);

protected:
	BVHBuildNode* HLBVHBuild(const std::vector<BVHPrimitiveInfo>& primitiveInfo, int* totalNodes, std::vector<std::shared_ptr<MyStructures::Primitive>>& orderedPrims);
	BVHBuildNode* recursiveBuild(std::vector<BVHPrimitiveInfo>& primitiveInfo, int start, int end, int* totalNodes, std::vector<std::shared_ptr<MyStructures::Primitive>>& orderedPrims);
	BVHBuildNode* createLeafBVHNode(std::vector<BVHPrimitiveInfo>& primitiveInfo, int start, int end, std::vector<std::shared_ptr<MyStructures::Primitive>>& orderedPrims, MyStructures::BBox& bounds);
	bool middlePointSplit(std::vector<BVHPrimitiveInfo>& primitiveInfo, int start, int end, int dim, MyStructures::BBox& bounds, int& mid);
	bool equalCountsSplit(std::vector<BVHPrimitiveInfo>& primitiveInfo, int start, int end, int dim, int& mid);
};
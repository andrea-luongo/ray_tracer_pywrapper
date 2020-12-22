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

class BVH
{
private:
	std::vector<std::shared_ptr<MyStructures::Primitive>> primitives;
	const int maxPrimsInNode;
	const SplitMethod splitMethod;
public:
	BVH(const std::vector<std::shared_ptr<MyStructures::Primitive>>& p, int maxPrimsInNode, SplitMethod splitMethod);
};
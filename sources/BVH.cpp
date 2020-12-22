#include "BVH.h"


BVH::BVH(const std::vector<std::shared_ptr<MyStructures::Primitive>>& p, int maxPrimsInNode, SplitMethod splitMethod): maxPrimsInNode(std::min(255, maxPrimsInNode)), primitives(p), splitMethod(splitMethod)
{
	if (primitives.size() == 0)
		return;
	std::vector<BVHPrimitiveInfo> primitiveInfo(primitives.size());
	for (size_t i = 0; i < primitives.size(); i++)
		primitiveInfo[i] = {i, primitives[i]->BBox()};
}
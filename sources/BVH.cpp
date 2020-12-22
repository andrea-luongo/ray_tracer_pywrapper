#include "BVH.h"


BVH::BVH(const std::vector<std::shared_ptr<MyStructures::Primitive>>& p, int maxPrimsInNode, SplitMethod splitMethod): maxPrimsInNode(std::min(255, maxPrimsInNode)), primitives(p), splitMethod(splitMethod)
{
	if (primitives.size() == 0)
		return;
	std::vector<BVHPrimitiveInfo> primitiveInfo(primitives.size());
	for (size_t i = 0; i < primitives.size(); i++)
		primitiveInfo[i] = {i, primitives[i]->BBox()};
	int totalNodes = 0;
	std::vector<std::shared_ptr<MyStructures::Primitive>> orderedPrims;
	BVHBuildNode* root;
	if (splitMethod == SplitMethod::HLBVH)
		root = HLBVHBuild(primitiveInfo, &totalNodes, orderedPrims);
	else
		root = recursiveBuild(primitiveInfo, 0, primitives.size(), &totalNodes, orderedPrims);
}

BVHBuildNode* BVH::recursiveBuild(std::vector<BVHPrimitiveInfo>& primitiveInfo, int start, int end, int* totalNodes, std::vector<std::shared_ptr<MyStructures::Primitive>>& orderedPrims)
{
	BVHBuildNode* node = new BVHBuildNode();
	(*totalNodes)++;
	MyStructures::BBox bounds;
	for (int i = start; i < end; i++) {
		bounds = MyStructures::BBox::Union(bounds, primitiveInfo[i].bounds);
	}
	int nPrimitives = end - start;
	if (nPrimitives == 1){
		// create leaf bvhbuildnode
		int firstPrimOffset = orderedPrims.size();
		for (int i = start; i < end; i++) {
			int primNum = primitiveInfo[i].primitiveNumber;
			orderedPrims.push_back(primitives[primNum]);
		}
		node->initLeaf(firstPrimOffset, nPrimitives, bounds);
		return node;
	}
	else {
		// compute bound of primitive centroid, choose split dimension
		MyStructures::BBox centroidBounds;
		for (int i = start; i < end; i++) {
			centroidBounds = MyStructures::BBox::Union(centroidBounds, primitiveInfo[i].centroid);
		}
		int dim = centroidBounds.MaximumExtent();
		// partition primitives into two sets and build children
	}
	return node;
}
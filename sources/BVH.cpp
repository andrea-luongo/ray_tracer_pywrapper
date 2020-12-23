#include "BVH.h"
#include <algorithm>


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
		node = createLeafBVHNode(primitiveInfo, start, end, orderedPrims, bounds);
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
		int mid = (start + end) / 2;
		if (CompareFloat3(centroidBounds.pMax, centroidBounds.pMin, dim)) {
			// create leaf bvhbuildnode
			node = createLeafBVHNode(primitiveInfo, start, end, orderedPrims, bounds);
			return node;
		}
		else {
			// partition primitives based on splitmethod
			if (splitMethod == SplitMethod::Middle)
			{
				bool success = middlePointSplit(primitiveInfo, start, end, dim, centroidBounds, mid);
				if (!success)
					equalCountsSplit(primitiveInfo, start, end, dim, mid);

			}
			else if (splitMethod == SplitMethod::EqualCounts)
			{
				equalCountsSplit(primitiveInfo, start, end, dim, mid);
			}
			else if (splitMethod == SplitMethod::SAH)
			{
				//TODO implement;
			}
			node->initInterior(dim, recursiveBuild(primitiveInfo, start, mid, totalNodes, orderedPrims), recursiveBuild(primitiveInfo, mid, end, totalNodes, orderedPrims));
		}
	}
	return node;
}

BVHBuildNode* BVH::createLeafBVHNode(std::vector<BVHPrimitiveInfo>& primitiveInfo, int start, int end, std::vector<std::shared_ptr<MyStructures::Primitive>>& orderedPrims, MyStructures::BBox& bounds)
{
	BVHBuildNode* node = new BVHBuildNode();
	int nPrimitives = end - start;
	int firstPrimOffset = orderedPrims.size();
	for (int i = start; i < end; i++) {
		int primNum = primitiveInfo[i].primitiveNumber;
		orderedPrims.push_back(primitives[primNum]);
	}
	node->initLeaf(firstPrimOffset, nPrimitives, bounds);
	return node;
}

bool BVH::middlePointSplit(std::vector<BVHPrimitiveInfo>& primitiveInfo, int start, int end, int dim, MyStructures::BBox& bounds, int& mid)
{
	float pmid = (GetFloat3Component(bounds.pMin, dim) + GetFloat3Component(bounds.pMax, dim)) * 0.5;
	BVHPrimitiveInfo* midPtr = std::partition(&primitiveInfo[start], &primitiveInfo[end-1] + 1, [dim, pmid](const BVHPrimitiveInfo& pi) {
		return GetFloat3Component(pi.centroid, dim) < pmid; });
	mid = midPtr - &primitiveInfo[0];
	if (mid != start && mid != end)
		return true;
	return false;

}

bool BVH::equalCountsSplit(std::vector<BVHPrimitiveInfo>& primitiveInfo, int start, int end, int dim, int& mid)
{
	mid = (start + end) / 2;
	std::nth_element(&primitiveInfo[start], &primitiveInfo[mid], &primitiveInfo[end - 1] + 1, [dim](const BVHPrimitiveInfo& a, 
		const BVHPrimitiveInfo& b) {return GetFloat3Component(a.centroid, dim) < GetFloat3Component(b.centroid, dim); });
	return true;
}
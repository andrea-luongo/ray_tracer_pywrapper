#include "BVH.h"
#include <algorithm>

BVH::BVH(const std::vector<std::shared_ptr<Primitive>>& p, int maxPrimsInNode, SplitMethod splitMethod): maxPrimsInNode(std::min(255, maxPrimsInNode)), primitives(p), splitMethod(splitMethod)
{
	if (primitives.size() == 0)
		return;
	//initialize primitiveInfo array for primitives
	std::vector<BVHPrimitiveInfo> primitiveInfo(primitives.size());
	for (size_t i = 0; i < primitives.size(); i++)
		primitiveInfo[i] = {i, primitives[i]->GetBBox()};
	//build BVH tree for primitives using primitiveInfo
	int totalNodes = 0;
	std::vector<std::shared_ptr<Primitive>> orderedPrims;
	BVHBuildNode* root;
	if (splitMethod == SplitMethod::HLBVH)
		//root = HLBVHBuild(primitiveInfo, &totalNodes, orderedPrims);
		root = recursiveBuild(primitiveInfo, 0, primitives.size(), &totalNodes, orderedPrims);
	else
		root = recursiveBuild(primitiveInfo, 0, primitives.size(), &totalNodes, orderedPrims);
	primitives.swap(orderedPrims);
	// Compute representation of depth-first traversal of BVH tree
	nodes = new LinearBVHNode[totalNodes];
	int offset = 0;
	flattenBVHTree(root, &offset);
}

int BVH::flattenBVHTree(BVHBuildNode* node, int* offset)
{
	LinearBVHNode* linearNode = &nodes[*offset];
	linearNode->bounds = node->bounds;
	int myOffset = (*offset)++;
	if (node->nPrimitives > 0)
	{
		linearNode->primitivesOffset = node->firstPrimOffset;
		linearNode->nPrimitives = node->nPrimitives;
	}
	else
	{
		linearNode->axis = node->splitAxis;
		linearNode->nPrimitives = 0;
		flattenBVHTree(node->children[0], offset);
		linearNode->secondChildOffset = flattenBVHTree(node->children[1], offset);
	}
	return myOffset;
}


BVHBuildNode* BVH::recursiveBuild(std::vector<BVHPrimitiveInfo>& primitiveInfo, int start, int end, int* totalNodes, std::vector<std::shared_ptr<Primitive>>& orderedPrims)
{
	BVHBuildNode* node = new BVHBuildNode();
	(*totalNodes)++;
	BBox bounds;
	for (int i = start; i < end; i++) {
		bounds = BBox::Union(bounds, primitiveInfo[i].bounds);
	}
	int nPrimitives = end - start;
	if (nPrimitives == 1){
		// create leaf bvhbuildnode
		node = createLeafBVHNode(primitiveInfo, start, end, orderedPrims, bounds);
		return node;
	}
	else {
		// compute bound of primitive centroid, choose split dimension
		BBox centroidBounds;
		for (int i = start; i < end; i++) {
			centroidBounds = BBox::Union(centroidBounds, primitiveInfo[i].centroid);
		}
		int dim = centroidBounds.MaximumExtent();
		// partition primitives into two sets and build children
		int mid = (start + end) / 2;
		if (CompareFloat3(centroidBounds.GetpMax(), centroidBounds.GetpMin(), dim)) {
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
				bool success = SAHSplit(primitiveInfo, start, end, dim, bounds, centroidBounds, mid);
				if (!success)
				{
					node = createLeafBVHNode(primitiveInfo, start, end, orderedPrims, bounds);
					return node;
				}
			}
			node->initInterior(dim, recursiveBuild(primitiveInfo, start, mid, totalNodes, orderedPrims), recursiveBuild(primitiveInfo, mid, end, totalNodes, orderedPrims));
		}
	}
	return node;
}

BVHBuildNode* BVH::createLeafBVHNode(std::vector<BVHPrimitiveInfo>& primitiveInfo, int start, int end, std::vector<std::shared_ptr<Primitive>>& orderedPrims, BBox& bounds)
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

bool BVH::middlePointSplit(std::vector<BVHPrimitiveInfo>& primitiveInfo, int start, int end, int dim, BBox& centroidBounds, int& mid)
{
	float pmid = (GetFloat3Component(centroidBounds.GetpMin(), dim) + GetFloat3Component(centroidBounds.GetpMax(), dim)) * 0.5;
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

bool BVH::SAHSplit(std::vector<BVHPrimitiveInfo>& primitiveInfo, int start, int end, int dim, BBox& bounds, BBox& centroidBounds, int& mid)
{
	int nPrimitives = end - start;
	if (nPrimitives < 4) 
	{
		//partition primitives into equally syzed subsets
	} 
	else 
	{
		//allocate bucketinfo for sah partition buckets
		constexpr int nBuckets = 12;
		struct BucketInfo 
		{
			int count = 0;
			BBox bounds;
		};
		BucketInfo buckets[nBuckets];
		//initialize bucketinfo for sah partition buckets
		for (int i = start; i < end; ++i)
		{
			int b = nBuckets * GetFloat3Component(centroidBounds.Offset(primitiveInfo[i].centroid), dim);
			if (b == nBuckets) b = nBuckets - 1;
			buckets[b].count++;
			buckets[b].bounds = BBox::Union(buckets[b].bounds, primitiveInfo[i].bounds);
		}
		//compute costs for splitting after each bucket
		float cost[nBuckets-1];
		for(int i=0; i<nBuckets-1; ++i)
		{
			BBox b0, b1;
			int count0 = 0, count1 = 0;
			for (int j = 0; j <= i; ++j)
			{
				b0 = BBox::Union(b0, buckets[j].bounds);
				count0 += buckets[j].count;
			}
			for (int j = i + 1; j < nBuckets; ++j)
			{
				b1 = BBox::Union(b1, buckets[j].bounds);
				count1 += buckets[j].count;
			}
			cost[i] = .125f + (count0 * b0.SurfaceArea() + count1 * b1.SurfaceArea()) / bounds.SurfaceArea();
		}
		//find bucket to split at that minimizes sah metric
		float minCost = cost[0];
		int minCostSplitBucket = 0;
		for (int i = 1; i < nBuckets - 1; ++i)
		{
			if (cost[i] < minCost)
			{
				minCost = cost[i];
				minCostSplitBucket = i;
			}
		}
		//either create leaf or split primitives at selected sah bucket
		float leafCost = nPrimitives;
		if (nPrimitives > maxPrimsInNode || minCost < leafCost)
		{
			BVHPrimitiveInfo* pmid = std::partition(&primitiveInfo[start], &primitiveInfo[end - 1], 
				[=](const BVHPrimitiveInfo& pi) 
				{
					int b = nBuckets * GetFloat3Component(centroidBounds.Offset(pi.centroid), dim);
					if (b == nBuckets) b = nBuckets - 1;
					return b <= minCostSplitBucket;
				});
			mid = pmid - &primitiveInfo[0];
			return true;
		}
		else
		{
			return false;
		}
	}
}

bool BVH::intersect(Ray& ray, RayIntersectionInfo& info)
{
	bool hit = false;
	num::float3 invDir(num::float3(1) / ray.GetDirection());
	int dirIsNeg[3] = {invDir.x < 0, invDir.y < 0, invDir.z < 0};
	//follow ray through BVH nodes to find primitive intersections
	int toVisitOffset = 0, currentNodeIndex = 0;
	int nodesToVisit[1024];
	while (true)
	{
		const LinearBVHNode* node = &nodes[currentNodeIndex];
		//check ray against BVH node
		if (node->bounds.AnyIntersect(ray, invDir, dirIsNeg))
		{
			if (node->nPrimitives > 0)
			{
				//intersect ray with primitives in leaf bvh node
				for (int i = 0; i < node->nPrimitives; ++i)
				{
					if (primitives[node->primitivesOffset + i]->Intersect(ray, info))
						hit = true;
				}
				if (toVisitOffset == 0) break;
				currentNodeIndex = nodesToVisit[--toVisitOffset];
			}
			else
			{
				//put far bvh node on nodestovisit stack, advance to near node
				if (dirIsNeg[node->axis])
				{
					nodesToVisit[toVisitOffset++] = currentNodeIndex + 1;
					currentNodeIndex = node->secondChildOffset;
				}
				else
				{
					nodesToVisit[toVisitOffset++] = node->secondChildOffset;
					currentNodeIndex = currentNodeIndex + 1;
				}
			}
		}
		else
		{
			if (toVisitOffset == 0) break;
			currentNodeIndex = nodesToVisit[--toVisitOffset];
		}
	}
	return hit;
}

bool BVH::any_intersect(Ray& ray)
{
	bool hit = false;
	num::float3 invDir(num::float3(1) / ray.GetDirection());
	int dirIsNeg[3] = { invDir.x < 0, invDir.y < 0, invDir.z < 0 };
	//follow ray through BVH nodes to find primitive intersections
	int toVisitOffset = 0, currentNodeIndex = 0;
	int nodesToVisit[1024];
	while (true)
	{
		const LinearBVHNode* node = &nodes[currentNodeIndex];
		//check ray against BVH node
		if (node->bounds.AnyIntersect(ray, invDir, dirIsNeg))
		{
			if (node->nPrimitives > 0)
			{
				//intersect ray with primitives in leaf bvh node
				for (int i = 0; i < node->nPrimitives; ++i)
				{
					if (primitives[node->primitivesOffset + i]->AnyIntersect(ray)) {
						hit = true;
						return hit;
					}
				}
				if (toVisitOffset == 0) break;
				currentNodeIndex = nodesToVisit[--toVisitOffset];
			}
			else
			{
				//put far bvh node on nodestovisit stack, advance to near node
				if (dirIsNeg[node->axis])
				{
					nodesToVisit[toVisitOffset++] = currentNodeIndex + 1;
					currentNodeIndex = node->secondChildOffset;
				}
				else
				{
					nodesToVisit[toVisitOffset++] = node->secondChildOffset;
					currentNodeIndex = currentNodeIndex + 1;
				}
			}
		}
		else
		{
			if (toVisitOffset == 0) break;
			currentNodeIndex = nodesToVisit[--toVisitOffset];
		}
	}
	return hit;
}

bool BVH::all_intersects(Ray& ray, RayIntersectionInfo& info)
{
	bool hit = false;
	num::float3 invDir(num::float3(1) / ray.GetDirection());
	int dirIsNeg[3] = { invDir.x < 0, invDir.y < 0, invDir.z < 0 };
	//follow ray through BVH nodes to find primitive intersections
	int toVisitOffset = 0, currentNodeIndex = 0;
	int nodesToVisit[1024];
	while (true)
	{
		const LinearBVHNode* node = &nodes[currentNodeIndex];
		//check ray against BVH node
		if (node->bounds.AnyIntersect(ray, invDir, dirIsNeg))
		{
			if (node->nPrimitives > 0)
			{
				//intersect ray with primitives in leaf bvh node
				for (int i = 0; i < node->nPrimitives; ++i)
				{
					if (primitives[node->primitivesOffset + i]->AllIntersect(ray, info))
						hit = true;
				}
				if (toVisitOffset == 0) break;
				currentNodeIndex = nodesToVisit[--toVisitOffset];
			}
			else
			{
				//put far bvh node on nodestovisit stack, advance to near node
				if (dirIsNeg[node->axis])
				{
					nodesToVisit[toVisitOffset++] = currentNodeIndex + 1;
					currentNodeIndex = node->secondChildOffset;
				}
				else
				{
					nodesToVisit[toVisitOffset++] = node->secondChildOffset;
					currentNodeIndex = currentNodeIndex + 1;
				}
			}
		}
		else
		{
			if (toVisitOffset == 0) break;
			currentNodeIndex = nodesToVisit[--toVisitOffset];
		}
	}
	return hit;
}

bool BVH::plane_all_intersects(Plane& plane, PlaneIntersectionInfo& info)
{
	bool hit = false;
	int toVisitOffset = 0, currentNodeIndex = 0;
	int nodesToVisit[1024*1024];
	while (true)
	{
		const LinearBVHNode* node = &nodes[currentNodeIndex];
		//check ray against BVH node
		if (node->bounds.PlaneAnyIntersect(plane))
		{
			if (node->nPrimitives > 0)
			{
				//intersect ray with primitives in leaf bvh node
				for (int i = 0; i < node->nPrimitives; ++i)
				{
					if (primitives[node->primitivesOffset + i]->PlaneIntersect(plane, info))
						hit = true;
				}
				if (toVisitOffset == 0) break;
				currentNodeIndex = nodesToVisit[--toVisitOffset];
			}
			else
			{
				//put far bvh node on nodestovisit stack, advance to near node
					nodesToVisit[toVisitOffset++] = node->secondChildOffset;
					currentNodeIndex = currentNodeIndex + 1;
			}
		}
		else
		{
			if (toVisitOffset == 0) break;
			currentNodeIndex = nodesToVisit[--toVisitOffset];
		}
	}
	return hit;
}
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
public:
	RAYTRACERDLL_API Contour(const std::vector<std::shared_ptr<Segment>>& p);
	RAYTRACERDLL_API void ComputeBBox();
	RAYTRACERDLL_API bool Intersect(Ray& ray, RayIntersectionInfo& info);
	RAYTRACERDLL_API bool AnyIntersect(Ray& ray);
	RAYTRACERDLL_API bool AllIntersect(Ray& ray, RayIntersectionInfo& info);
	RAYTRACERDLL_API bool PlaneIntersect(Plane& plane, PlaneIntersectionInfo& info);
	RAYTRACERDLL_API bool IsContained(Contour& contour_b, float& t_hit);
	RAYTRACERDLL_API bool Contains(Contour& contour_b, float& t_hit);
	RAYTRACERDLL_API static int EvaluateContoursRelationship(Contour& contour_a, Contour& contour_b, float& t_hit);
};

struct ContourNode
{
public:
	std::shared_ptr<Contour> contour;
	std::vector<std::shared_ptr<ContourNode>> children;
	std::shared_ptr<ContourNode> parent;
	int depth = 0;
public:
	ContourNode() { contour = nullptr; parent = nullptr; };
	ContourNode(std::shared_ptr<Contour> c) { contour = c; parent = nullptr; };
	ContourNode(std::shared_ptr<Contour> c, std::shared_ptr<ContourNode> p) { contour = c; p->AddChild(std::shared_ptr<ContourNode>(this)); };
	void SetChildren(std::vector<std::shared_ptr<ContourNode>> c) 
	{ 
		children = c;
		for (std::shared_ptr<ContourNode> n : children) 
		{
			n->parent = std::shared_ptr<ContourNode>(this);
			n->depth = depth + 1;
		}
	};
	void AddChild(std::shared_ptr<ContourNode> c) 
	{ 
		children.push_back(c);
		c->parent = std::shared_ptr<ContourNode>(this);
		c->depth = depth + 1; 
	};
	
	std::vector<std::shared_ptr<Contour>> GetChildrenContours() 
	{
		std::vector<std::shared_ptr<Contour>> children_contour(children.size());
		for (int idx = 0; idx < children.size(); idx++)
		{
			children_contour[idx] = children[idx]->contour;
		}
		return children_contour;
	};

	std::vector<std::shared_ptr<ContourNode>> GetDescendants() 
	{ 
		std::vector<std::shared_ptr<ContourNode>> descendants;

		descendants.insert(descendants.end(), children.begin(), children.end());
		for (int idx = 0; idx < children.size(); idx++)
		{
			std::vector<std::shared_ptr<ContourNode>> c_descend = children[idx]->GetDescendants();
			descendants.insert(descendants.end(), c_descend.begin(), c_descend.end());
		}
		return descendants;
	};

	std::vector<std::shared_ptr<ContourNode>> GetAncestors()
	{
		std::vector<std::shared_ptr<ContourNode>> ancestors;
		if (parent != nullptr)
		{
			ancestors.push_back(parent);
			std::vector<std::shared_ptr<ContourNode>> p_ancestors = parent->GetAncestors();
			ancestors.insert(ancestors.end(), p_ancestors.begin(), p_ancestors.end());
		}
	
		return ancestors;
	};


};

class ContourTree
{
public:
	std::vector<std::shared_ptr<Contour>> contours;
	ContourNode* tree_root;
	BVH* root_bvh;
	std::vector<std::shared_ptr<BVH>> tree_global_bvsh;
	std::vector<std::vector<std::shared_ptr<BVH>>> tree_individual_bvhs;

private:
	RAYTRACERDLL_API ContourTree(std::vector<std::shared_ptr<Contour>> c);
	RAYTRACERDLL_API void BuildTree();
	RAYTRACERDLL_API void CheckChildren(std::shared_ptr<ContourNode> n, std::vector<std::shared_ptr<ContourNode>> children);
	RAYTRACERDLL_API void CheckParents(std::shared_ptr<ContourNode>& n, std::shared_ptr<ContourNode>& p);
	RAYTRACERDLL_API void BuildRootBVH();
	RAYTRACERDLL_API void BuildTreeIndividualBVH();
	RAYTRACERDLL_API void BuildTreeGlobalBVH();
};

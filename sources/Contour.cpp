#include "Contour.h"
#include <algorithm>

///////IMPLEMENTING CONTOUR CLASS
Contour::Contour() 
{ 
	is_valid = false; 
	bbox = BBox();
}

Contour::Contour(const std::vector<std::shared_ptr<Segment>>& p, const float3 n) : segments(p), contour_normal(n)
{
	bvh = new BVH({segments.begin(), segments.end()}, SplitMethod::EqualCounts, 255);
	ComputeBBox();
	if (segments.size() < 3) {
		is_valid = false;
	}
}

void Contour::ComputeBBox()
{
	bbox = bvh->getBVHBBox();
}

bool Contour::Intersect(Ray& ray, RayIntersectionInfo& info)
{
	return bvh->intersect(ray, info);
}

bool Contour::AnyIntersect(Ray& ray)
{
	return bvh->any_intersect(ray);
}

bool Contour::AllIntersect(Ray& ray, RayIntersectionInfo& info)
{
	return bvh->all_intersects(ray, info);
}

bool Contour::PlaneIntersect(Plane& plane, PlaneIntersectionInfo& info)
{
	return bvh->plane_all_intersects(plane, info);
}

bool Contour::IsContained(Contour& contour_b, float& t_hit)
{
	t_hit = std::numeric_limits<float>::max();
	if (bbox.GetpMin()[0] > contour_b.bbox.GetpMax()[0] || contour_b.bbox.GetpMin()[0] > bbox.GetpMax()[0])
	{
		return false;
	}
	if (bbox.GetpMin()[2] > contour_b.bbox.GetpMax()[2] || contour_b.bbox.GetpMin()[2] > bbox.GetpMax()[2])
	{
		return false;
	}
	float3 origin = segments[0]->v1;
	float3 direction = float3::normalize(segments[0]->v1 - segments[0]->v0);
	Ray ray(origin, direction, 0, std::numeric_limits<float>::max(), 0, 0);
	RayIntersectionInfo *info = new RayIntersectionInfo();
	//RayIntersectionInfo info();
	contour_b.AllIntersect(ray, *info);
	std::vector<float> t_hits = *info->GetHits();
	if (t_hits.size() % 2 == 1)
	{
		t_hit = *std::min_element(t_hits.begin(), t_hits.end());
		return true;
	}
	else 
	{
		return false;
	}
}

bool Contour::Contains(Contour& contour_b, float& t_hit)
{
	t_hit = std::numeric_limits<float>::max();
	if (bbox.GetpMin()[0] > contour_b.bbox.GetpMax()[0] || contour_b.bbox.GetpMin()[0] > bbox.GetpMax()[0])
	{
		return false;
	}
	if (bbox.GetpMin()[2] > contour_b.bbox.GetpMax()[2] || contour_b.bbox.GetpMin()[2] > bbox.GetpMax()[2])
	{
		return false;
	}
	float3 origin = contour_b.segments[0]->v1;
	float3 direction = float3::normalize(contour_b.segments[0]->v1 - contour_b.segments[0]->v0);
	Ray ray(origin, direction, 0, std::numeric_limits<float>::max(), 0, 0);
	RayIntersectionInfo* info = new RayIntersectionInfo();
	//RayIntersectionInfo info();
	this->AllIntersect(ray, *info);
	std::vector<float> t_hits = *info->GetHits();
	if (t_hits.size() % 2 == 1)
	{
		t_hit = *std::min_element(t_hits.begin(), t_hits.end());
		return true;
	}
	else
	{
		return false;
	}
}

int Contour::EvaluateContoursRelationship(Contour& contour_a, Contour& contour_b, float& t_hit)
{
	t_hit = std::numeric_limits<float>::max();
	if (contour_a.Contains(contour_b, t_hit))
		return 1;
	else if (contour_a.IsContained(contour_b, t_hit))
		return -1;
	else
		return 0;

}

Contour Contour::OffsetContour(float offset)
{
	return Contour();
}

///////////////////////////////////////////////////////////////////////

///////IMPLEMENTING CONTOURNODE CLASS
ContourNode::ContourNode()
{ 
};

ContourNode::~ContourNode()
{
	children.clear();
}

ContourNode::ContourNode(std::shared_ptr<Contour> c)
{
	contour = c;
	parent = nullptr;
};

ContourNode::ContourNode(std::shared_ptr<Contour> c, std::shared_ptr<ContourNode> p)
{
	contour = c;
	p->AddChild(*this);
};

ContourNode::ContourNode(std::shared_ptr<Contour> c, std::shared_ptr<ContourNode> p, int i)
{
	contour = c;
	p->AddChild(*this);
	id = i;
};

//void ContourNode::SetChildren(std::vector<std::shared_ptr<ContourNode>> c)
//{
//	children = c;
//	for (std::shared_ptr<ContourNode> n : children)
//	{
//		n->parent = std::shared_ptr<ContourNode>(this);
//		n->depth = depth + 1;
//	}
//};
//
//void ContourNode::RemoveParent()
//{
//	if (parent != nullptr)
//	{
//		parent->RemoveChild(*this);
//	}
//	parent = nullptr;
//	depth = 0;
//}
//
//void ContourNode::SetParent(std::shared_ptr<ContourNode> p)
//{
//
//	if (parent != nullptr)
//	{
//		parent->RemoveChild(p);
//	}
//	p->AddChild(std::shared_ptr<ContourNode>(this));
//}

void ContourNode::AddChild(ContourNode& c)
{
	if (c.parent != nullptr)
	{
		c.parent->RemoveChild(c);
	}
	children.push_back(c);
	c.parent = std::shared_ptr<ContourNode>(this);
	c.depth = depth + 1;
	//c.UpdateChildrenDepth();
};
//
void ContourNode::RemoveChild(ContourNode& c)
{
	//std::vector<ContourNode> tmp;

	children.erase(std::remove(children.begin(), children.end(), c), children.end());

	//for (int idx = 0; idx < children.size(); idx++) {
	//	if (children[idx] != c)
	//		tmp.push_back(children[idx]);
	//}
	//children = tmp;
	c.parent = nullptr;
	c.depth = 0;
	//c.UpdateChildrenDepth();
	//c->parent = nullptr;
	//c->depth = 0;
};

void ContourNode::UpdateChildrenDepth()
{
	for (int idx = 0; idx < children.size(); idx++) {
		children[idx].depth = depth + 1;
		children[idx].UpdateChildrenDepth();
	}
};



//std::vector<Contour> ContourNode::GetChildrenContours()
//{
//	std::vector<Contour> children_contour(children.size());
//	for (int idx = 0; idx < children.size(); idx++)
//	{
//		children_contour[idx] = children[idx].contour;
//	}
//	return children_contour;
//};
//
//std::vector<ContourNode> ContourNode::GetDescendants()
//{
//	std::vector<ContourNode> descendants;
//
//	descendants.insert(descendants.end(), children.begin(), children.end());
//	for (int idx = 0; idx < children.size(); idx++)
//	{
//		std::vector<std::shared_ptr<ContourNode>> c_descend = children[idx]->GetDescendants();
//		descendants.insert(descendants.end(), c_descend.begin(), c_descend.end());
//	}
//	return descendants;
//};
//
//std::vector<std::shared_ptr<ContourNode>> ContourNode::GetAncestors()
//{
//	std::vector<std::shared_ptr<ContourNode>> ancestors;
//	if (parent != nullptr)
//	{
//		ancestors.push_back(parent);
//		std::vector<std::shared_ptr<ContourNode>> p_ancestors = parent->GetAncestors();
//		ancestors.insert(ancestors.end(), p_ancestors.begin(), p_ancestors.end());
//	}
//
//	return ancestors;
//};
//////////////////////////////////////////////////////////////////////////////

///////IMPLEMENTING CONTOURTREE CLASS
ContourTree::ContourTree(std::vector<std::shared_ptr<Contour>> c)
{
	contours = c;
	tree_root;
	BuildTree();
	BuildTreeGlobalBVH();
	BuildTreeIndividualBVH();
}

void ContourTree::BuildTree()
{
	std::vector<ContourNode> contour_nodes(contours.size());
	for (int idx = 0; idx < contour_nodes.size(); idx++)
	{
		contour_nodes[idx].SetNodeID(node_id++);
		tree_root.AddChild(contour_nodes[idx]);
	}

	for (int i_idx = 0; i_idx < contour_nodes.size() - 1; i_idx++)
	{
		float closest_hit = std::numeric_limits<float>::max();
		std::shared_ptr<ContourNode> possible_parent;
		std::vector<std::shared_ptr<ContourNode>> possible_children;
		std::shared_ptr<ContourNode> i_node = std::make_shared<ContourNode>(contour_nodes[i_idx]);
		for (int j_idx = 0; j_idx < contour_nodes.size() - 1; j_idx++)
		{
			float t_hit;
			std::shared_ptr<ContourNode> j_node = std::make_shared<ContourNode>(contour_nodes[j_idx]);
			int result = Contour::EvaluateContoursRelationship(*i_node->contour, *j_node->contour, t_hit);
			if (result == -1)
			{
				if (t_hit < closest_hit)
				{
					possible_parent = j_node;
					closest_hit = t_hit;
				}
			}
			else if (result == 1)
			{
				possible_children.push_back(j_node);
			}
		}
		CheckParents(i_node, possible_parent);
		CheckChildren(i_node, possible_children);
	}

}

void ContourTree::CheckChildren(std::shared_ptr<ContourNode> n, std::vector<std::shared_ptr<ContourNode>> children)
{
	if (children.size() == 0)
		return;

	std::vector<std::shared_ptr<ContourNode>> descendants = n->GetDescendants();
	for (std::shared_ptr<ContourNode> c : children) {
		if (std::find(descendants.begin(), descendants.end(), c) == descendants.end())
		{
			c->SetParent(n);
		}
	}

}

void ContourTree::CheckParents(std::shared_ptr<ContourNode>& n, std::shared_ptr<ContourNode>& p)
{
	if (p == nullptr)
	{
		return;
	}
	std::vector<std::shared_ptr<ContourNode>> ancestors = n->GetAncestors();
	if (std::find(ancestors.begin(), ancestors.end(), p) == ancestors.end())
	{
		n->SetParent(p);
	}
}

//build a global root bvh
void ContourTree::BuildRootBVH()
{
	std::vector<std::shared_ptr<Contour>> root_children_contours = tree_root->GetChildrenContours();
	root_bvh = new BVH({ root_children_contours.begin(), root_children_contours.end() }, SplitMethod::EqualCounts, 255);
}

//build a tree for each set of external-internal contours
void ContourTree::BuildTreeIndividualBVH()
{
	for (std::shared_ptr<ContourNode> c : tree_root->children)
	{
		std::vector<std::shared_ptr<ContourNode>> nodes = c->GetDescendants();
		nodes.push_back(c);
		std::vector<std::shared_ptr<BVH>> node_bvhs;

		for (int idx = 0; idx < nodes.size(); idx++)
		{
			if (nodes[idx]->depth % 2 == 1)
			{
				std::vector<std::shared_ptr<Contour>> contours = nodes[idx]->GetChildrenContours();
				contours.push_back(nodes[idx]->contour);
				BVH* bvh = new BVH({ contours.begin(), contours.end() }, SplitMethod::EqualCounts, 255);
				node_bvhs.push_back(std::shared_ptr<BVH>(bvh));
			}
			
		}

		tree_individual_bvhs.push_back(node_bvhs);
	}
}


//build a bvh for each tree_root children
void ContourTree::BuildTreeGlobalBVH()
{
	for (std::shared_ptr<ContourNode> c : tree_root->children) 
	{
		std::vector<std::shared_ptr<ContourNode>> nodes = c->GetDescendants();
		nodes.push_back(c);
		std::vector<std::shared_ptr<Contour>> contours(nodes.size());
		for (int idx = 0; idx < nodes.size(); idx++)
		{
			contours[idx] = nodes[idx]->contour;
		}
		//std::shared_ptr<BVH> a = std::make_shared<BVH>({ contours.begin(), contours.end() }, SplitMethod::EqualCounts, 255);
		BVH* bvh = new BVH({ contours.begin(), contours.end() }, SplitMethod::EqualCounts, 255);
		tree_global_bvsh.push_back(std::shared_ptr<BVH>(bvh));
	}
}
/////////////////////////////////////////////////////////////////////////
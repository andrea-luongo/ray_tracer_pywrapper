#include "Contour.h"


Contour::Contour(const std::vector<std::shared_ptr<Segment>>& p)
{
	segments = p;
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



ContourTree::ContourTree(std::vector<std::shared_ptr<Contour>> c)
{
	contours = c;
	tree_root = new ContourNode(nullptr);

}

void ContourTree::BuildTree()
{
	std::vector<std::shared_ptr<ContourNode>> contour_nodes(contours.size());
	for (int idx = 0; idx < contour_nodes.size(); idx++)
	{
		contour_nodes[idx] = std::shared_ptr<ContourNode>(contours[idx], tree_root);
	}

	for (int i_idx = 0; i_idx < contour_nodes.size() - 1; i_idx++)
	{
		float closest_hit = std::numeric_limits<float>::max();
		std::shared_ptr<ContourNode> possible_parent;
		std::vector<std::shared_ptr<ContourNode>> possible_children;
		std::shared_ptr<ContourNode> i_node = contour_nodes[i_idx];
		for (int j_idx = 0; j_idx < contour_nodes.size() - 1; j_idx++)
		{
			float t_hit;
			std::shared_ptr<ContourNode> j_node = contour_nodes[j_idx];
			bool result = Contour::EvaluateContoursRelationship(*i_node->contour, *j_node->contour, t_hit);
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
			c->parent = n;
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
		n->parent = p;
	}
}

void ContourTree::BuildRootBVH()
{

}

void ContourTree::BuildTreeIndividualBVH()
{

}

void ContourTree::BuildTreeGlobalBVH()
{

}

#include "Contour.h"
#include <algorithm>
#include <ppl.h>
#define _USE_MATH_DEFINES
#include <math.h>

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

}

ContourNode::ContourNode(std::shared_ptr<Contour> c)
{
	contour = c;
};

ContourNode::ContourNode(int id)
{
	node_id = id;
};

ContourNode::ContourNode(std::shared_ptr<Contour> c, int id)
{
	contour = c;
	node_id = id;
};

void ContourNode::AddChild(std::shared_ptr<ContourNode> c)
{
	if (!c->parent.expired())
	{
		c->parent.lock()->RemoveChild(c);
	}
	children_set.insert(c);
	c->parent = shared_from_this();
	c->depth = depth + 1;
};

void ContourNode::RemoveChild(std::shared_ptr<ContourNode> c)
{
	children_set.erase(c);
	c->parent.reset();
	c->depth = 0;
};

void ContourNode::UpdateChildrenDepth()
{
	for (auto child = children_set.begin(); child != children_set.end(); ++child) {
		(*child)->depth = depth + 1;
		(*child)->UpdateChildrenDepth();
	}
};

std::vector<std::shared_ptr<Contour>> ContourNode::GetChildrenContours()
{
	std::vector<std::shared_ptr<Contour>> children_contour(children_set.size());
	int idx = 0;
	for (auto child = children_set.begin(); child != children_set.end(); ++child) {
		children_contour[idx++] = (*child)->contour;
	}
	return children_contour;
};

std::vector<std::shared_ptr<ContourNode>> ContourNode::GetDescendants()
{
	std::vector<std::shared_ptr<ContourNode>> descendants;

	descendants.insert(descendants.end(), children_set.begin(), children_set.end());
	for (auto child = children_set.begin(); child != children_set.end(); ++child)
	{
		std::vector<std::shared_ptr<ContourNode>> c_descend = (*child)->GetDescendants();
		descendants.insert(descendants.end(), c_descend.begin(), c_descend.end());
	}
	return descendants;
};

std::vector<std::shared_ptr<ContourNode>> ContourNode::GetAncestors()
{
	std::vector<std::shared_ptr<ContourNode>> ancestors;
	if (!parent.expired())
	{

		ancestors.push_back(parent.lock());
		std::vector<std::shared_ptr<ContourNode>> p_ancestors = parent.lock()->GetAncestors();
		ancestors.insert(ancestors.end(), p_ancestors.begin(), p_ancestors.end());
	}

	return ancestors;
};

std::vector<std::shared_ptr<ContourNode>> ContourNode::GetChildren()
{
	std::vector<std::shared_ptr<ContourNode>> children;

	children.insert(children.end(), children_set.begin(), children_set.end());
	
	return children;
};
//////////////////////////////////////////////////////////////////////////////



///////IMPLEMENTING CONTOURTREE CLASS
ContourTree::ContourTree(std::vector<std::shared_ptr<Contour>> c)
{
	contours = c;
	tree_root = std::make_shared<ContourNode>(node_id_counter++);
	BuildTree();
	BuildRootBVH();
	//BuildTreeGlobalBVH();
	BuildInternalBVHs();
}

void ContourTree::BuildTree()
{

	std::vector<std::shared_ptr<ContourNode>> contour_nodes;
	for (int idx = 0; idx < contours.size(); idx++)
	{
		contour_nodes.push_back(std::make_shared<ContourNode>(contours[idx], node_id_counter++));
		//contour_nodes[idx]->SetNodeID(node_id_counter++);
		tree_root->AddChild(contour_nodes[idx]);
	}

	for (int i_idx = 0; i_idx < contour_nodes.size() - 1; i_idx++)
	{
		float closest_hit = std::numeric_limits<float>::max();
		std::shared_ptr<ContourNode> possible_parent;
		std::vector<std::shared_ptr<ContourNode>> possible_children;
		std::shared_ptr<ContourNode> i_node = contour_nodes[i_idx];
		for (int j_idx = i_idx + 1; j_idx < contour_nodes.size(); j_idx++)
		{
			std::shared_ptr<ContourNode> j_node = contour_nodes[j_idx];
			float t_hit;
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
			n->AddChild(c);
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
		p->AddChild(n);
	}
}

//build a global root bvh
void ContourTree::BuildRootBVH()
{
	std::vector<std::shared_ptr<Contour>> root_children_contours = tree_root->GetChildrenContours();
	root_bvh = new BVH({ root_children_contours.begin(), root_children_contours.end() }, SplitMethod::EqualCounts, 255);
	bbox = root_bvh->getBVHBBox();
}

//build a bvh for each set of external-internal contours
void ContourTree::BuildInternalBVHs()
{
	for (std::shared_ptr<ContourNode> c : tree_root->children_set)
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

		internal_bvhs.push_back(node_bvhs);
	}
}
//
//
////build a bvh for each tree_root children
void ContourTree::BuildTreeGlobalBVH()
{
	for (std::shared_ptr<ContourNode> c : tree_root->children_set) 
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

bool ContourTree::Intersect(Ray& ray, RayIntersectionInfo& info)
{
	bool result = false;
	for (auto branch_bvhs : internal_bvhs)
	{
		for (auto bvh : branch_bvhs)
		{
			result = bvh->intersect(ray, info);
			//result = bvh->all_intersects(*ray.ray, *info.rayInfo);
		}
	}
	return result;
}

bool ContourTree::AnyIntersect(Ray& ray)
{
	for (auto branch_bvhs : internal_bvhs)
	{
		for (auto bvh : branch_bvhs)
		{
			if (bvh->any_intersect(ray))
				return true;
		}
	}
	return false;
}

bool ContourTree::AllIntersect(Ray& ray, RayIntersectionInfo& info)
{
	bool result = false;
	for (auto branch_bvhs : internal_bvhs)
	{
		for (auto bvh : branch_bvhs)
		{
			result = bvh->all_intersects(ray, info);
			//result = bvh->all_intersects(*ray.ray, *info.rayInfo);
		}
	}
	return result;
}

std::vector < std::vector<std::vector<float3>>> ContourTree::MultiRayIndividualBVHsAllIntersects(float laser_width_microns, float layer_thickness_microns, float density, float overlap, float current_slice, float height_offset, float rot_angle_deg, Matrix4x4& const rot_matrix)
{
	bool verbose = false;
	//float3 ray_direction = rot_matrix * float4(0.0f, 0.0f, 1.0f, 0.0f);
	float rot_angle = fmod(rot_angle_deg * current_slice, 360) * M_PI / 180.0f;
	float3 ray_direction(sinf(rot_angle), 0.0f, cosf(rot_angle));
	//float const ray_min = 0;
	//float const ray_max = std::numeric_limits<float>::infinity();
	if (density < 1.0)
		overlap = 0.0;

	std::vector < std::vector<std::vector<float3>>> individual_hit_points(internal_bvhs.size());
	int bvh_idx = 0;
	for (auto branch_bvhs : internal_bvhs)
	{
		for (auto bvh : branch_bvhs)
		{
			float3 bbox_min = bvh->getBVHBBox().GetpMin();
			float3 bbox_max = bvh->getBVHBBox().GetpMax();
			float3 bbox_center = 0.5f * (bbox_min + bbox_max);
			float bbox_width = (bbox_max.x - bbox_min.x);
			float bbox_depth = (bbox_max.z - bbox_min.z);
			float bbox_diagonal = bbox.Diagonal().length();
			float bbox_max_width = bbox_width * fabsf(cosf(rot_angle)) + bbox_depth * fabsf(sinf(rot_angle));
			float bbox_max_depth = bbox_width * fabsf(sinf(rot_angle)) + bbox_depth * fabsf(cosf(rot_angle));
			//std::cout << "min " << bbox_min << " max " << bbox_max << " maxlength " << bbox_max_length << std::endl;
			int number_of_rays = ceil(bbox_max_width / (laser_width_microns - laser_width_microns * overlap) * 1000 * density);
			if (number_of_rays == 0)
				continue;
			//std::cout << "Rays: " << number_of_rays << std::endl;
			float rays_origin_offset = bbox_max_width / number_of_rays;
			float ray_origin_x = (-bbox_max_width * 0.5) + rays_origin_offset * 0.5;
			float ray_origin_y = layer_thickness_microns * (current_slice + height_offset) / 1000.0f;
			float ray_origin_z = -bbox_max_depth * 0.5 - 1.0;
			float3 ray_origin(ray_origin_x, ray_origin_y, ray_origin_z);
			if (verbose)
			{
			std::cout << "bbox " << bbox_min << " " << bbox_max << std::endl;
			std::cout << "rot angle rad " << rot_angle << std::endl;
			std::cout << "bbox max length " << bbox_max_width << std::endl;
			std::cout << "number of rays " << number_of_rays << " ray offset "<< rays_origin_offset << std::endl;
			std::cout << "ray_origin " << ray_origin << std::endl;
			std::cout << "ray_direction " << ray_direction << std::endl;

			}
			//std::cout << "Ray start origin " << ray_origin << std::endl;
			std::vector<Ray> rays(number_of_rays);
			std::vector<RayIntersectionInfo> infos(number_of_rays);

			concurrency::parallel_for(int(0), number_of_rays, [&](int idx)
				{
					float3 ray_o = (rot_matrix * float4(ray_origin.x + rays_origin_offset * idx, ray_origin.y, ray_origin.z, 1.0f) + float4(bbox_center.x, 0, bbox_center.z, 0));
					rays[idx].SetOrigin(ray_o);
					rays[idx].SetDirection(ray_direction);
					//rays[idx].SetMax(ray_max);
					//rays[idx].SetMin(ray_min);
				});
			concurrency::parallel_for(int(0), number_of_rays, [&](int idx)
				{
					bvh->all_intersects(rays[idx], infos[idx]);
				});

			std::vector<std::vector<float3>> hit_points(number_of_rays);
			try {
				for (int ray_idx = 0; ray_idx < number_of_rays; ray_idx++)
				{
					std::vector<float> t_hits = *infos[ray_idx].GetHits();
					for (int hit_idx = 0; hit_idx < t_hits.size(); hit_idx++)
					{
						float3 hit_point = rays[ray_idx].GetOrigin() + t_hits[hit_idx] * rays[ray_idx].GetDirection();
						hit_points[ray_idx].push_back(hit_point);
						//std::cout << hit_point << std::endl;
					}
				}
				individual_hit_points[bvh_idx++] = hit_points;
			}
			catch (...)
			{
				std::cout << "Ray Intersection Exception" << std::endl;
			}
		}
	}
	return individual_hit_points;
}

BBox ContourTree::GetBBox()
{
	return bbox;

}
/////////////////////////////////////////////////////////////////////////
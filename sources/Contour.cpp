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
	if (segments.size() < 3) {
		is_valid = false;
		return;
	}
	//bvh = new BVH({segments.begin(), segments.end()}, SplitMethod::EqualCounts, 255);
	ComputeBBox();
}

void Contour::ComputeBBox()
{
	bvh = new BVH({ segments.begin(), segments.end() }, SplitMethod::EqualCounts, 255);
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
	std::vector<float3> new_vertices(segments.size());
	std::vector<std::shared_ptr<Segment>> new_segments(segments.size());
	for (int idx = 0; idx < segments.size(); idx++)
	{
		Segment s_cur = *segments[idx];
		Segment s_prev;
		if (idx == 0)
			s_prev = *segments[segments.size()-1];
		else
			s_prev = *segments[idx-1];
		float3 s_normal = float3::cross((s_cur[1] - s_cur[0]), contour_normal).normalize();
		float3 s_prev_normal = float3::cross((s_prev[1] - s_prev[0]), contour_normal).normalize();
		float half_angle = 0.5 * (acosf(float3::dot(s_normal, s_prev_normal)));
		float3 half_normal = (s_normal + s_prev_normal).normalize();
		float3 v = s_cur[0] + offset / cosf(half_angle) * half_normal;
		new_vertices[idx] = v;
		if (idx > 0)
		{
			new_segments[idx] = std::shared_ptr<Segment>(new Segment(new_vertices[idx - 1], v));
		}
		if (idx == segments.size()-1)
		{
			new_segments[0] = std::shared_ptr<Segment>(new Segment(v, new_vertices[0]));
		}
	}
	return Contour(new_segments, contour_normal);
}

bool Contour::RemoveSelfIntersections(std::vector<std::shared_ptr<Contour>>& new_contours)
{
	std::vector<ContourSelfIntersectionPoint> contour_intersection_points;
	bool is_self_intersecting = false;
	for (int i = 0; i < segments.size() - 2; i++)
	{
		std::shared_ptr<Segment> s_i = segments[i];
		std::vector<ContourSelfIntersectionPoint> segment_intersection_points;
		for (int j = i + 2; j < segments.size(); j++)
		{
			if (i == 0 && j == segments.size() - 1)
				continue;
			std::shared_ptr<Segment> s_j = segments[j];
			float3 hit_point;
			float t_hit;
			if (s_i->IntersectSegment(*s_j, hit_point, t_hit))
			{
				segment_intersection_points.push_back(ContourSelfIntersectionPoint(hit_point, t_hit, i, j));
			}

		}
		std::sort(segment_intersection_points.begin(), segment_intersection_points.end());
		contour_intersection_points.insert(contour_intersection_points.end(), segment_intersection_points.begin(), segment_intersection_points.end());
	}

	if (contour_intersection_points.size() > 0)
	{
		is_self_intersecting = true;
		std::vector<std::shared_ptr<Segment>> starting_loop;
		std::vector<std::shared_ptr<Segment>> ending_loop;
		ContourSelfIntersectionPoint p_start = contour_intersection_points[0];
		if (p_start.idx_0 == 0)
		{
			starting_loop.push_back(std::make_shared<Segment>(p_start.hit_point, segments[p_start.idx_0]->v1));
		}
		else
		{

		}
	}


	return is_self_intersecting;

}

std::vector<std::vector<float3>> Contour::MultiRayAllIntersects(float laser_width_microns, float density, float overlap, float rot_angle_deg, bool verbose=false)
{
	float rot_angle = rot_angle_deg * M_PI / 180.0f;
	Matrix4x4 rot_matrix = Matrix4x4::Rotate(rot_angle, float3(0, 1, 0));
	float3 ray_direction(sinf(rot_angle), 0.0f, cosf(rot_angle));
	if (density < 1.0)
		overlap = 0.0;

	float3 bbox_min = bvh->getBVHBBox().GetpMin();
	float3 bbox_max = bvh->getBVHBBox().GetpMax();
	float3 bbox_center = 0.5f * (bbox_min + bbox_max);
	float bbox_width = (bbox_max.x - bbox_min.x);
	float bbox_depth = (bbox_max.z - bbox_min.z);
	float bbox_diagonal = bbox.Diagonal().length();
	float bbox_max_width = bbox_width * fabsf(cosf(rot_angle)) + bbox_depth * fabsf(sinf(rot_angle));
	float bbox_max_depth = bbox_width * fabsf(sinf(rot_angle)) + bbox_depth * fabsf(cosf(rot_angle));
	int number_of_rays = ceil(bbox_max_width / (laser_width_microns - laser_width_microns * overlap) * 1000 * density);
	std::vector<std::vector<float3>> individual_hit_points(number_of_rays);
	if (number_of_rays == 0)
		return individual_hit_points;
	float rays_origin_offset = bbox_max_width / number_of_rays;
	float ray_origin_x = (-bbox_max_width * 0.5) + rays_origin_offset * 0.5;
	float ray_origin_y = bbox_max[1];
	float ray_origin_z = -bbox_max_depth * 0.5 - 1.0;
	float3 ray_origin(ray_origin_x, ray_origin_y, ray_origin_z);
	
	if (verbose)
	{
		std::cout << "bbox " << bbox_min << " " << bbox_max << std::endl;
		std::cout << "rot angle rad " << rot_angle << std::endl;
		std::cout << "bbox max length " << bbox_max_width << std::endl;
		std::cout << "bbox max depth " << bbox_max_depth << std::endl;
		std::cout << "bbox_center " << bbox_center << std::endl;
		std::cout << "number of rays " << number_of_rays << " ray offset " << rays_origin_offset << std::endl;
		std::cout << "ray_origin " << ray_origin << std::endl;
		std::cout << "ray_direction " << ray_direction << std::endl;
	}
	std::vector<Ray> rays(number_of_rays);
	std::vector<RayIntersectionInfo> infos(number_of_rays);
	concurrency::parallel_for(int(0), number_of_rays, [&](int idx)
		{
			float3 ray_o = (rot_matrix * float4(ray_origin.x + rays_origin_offset * idx, ray_origin.y, ray_origin.z, 1.0f) + float4(bbox_center.x, 0, bbox_center.z, 0));
			rays[idx].SetOrigin(ray_o);
			rays[idx].SetDirection(ray_direction);
		});
	if (verbose)
	{
		for (int idx = 0; idx < number_of_rays; idx++)
			std::cout << "ray_origin " << rays[idx].GetOrigin() << std::endl;
	}
	concurrency::parallel_for(int(0), number_of_rays, [&](int idx)
		{
			bvh->all_intersects(rays[idx], infos[idx]);
		});

	try {
		for (int ray_idx = 0; ray_idx < number_of_rays; ray_idx++)
		{
			std::vector<float> t_hits = *infos[ray_idx].GetHits();
			for (int hit_idx = 0; hit_idx < t_hits.size(); hit_idx++)
			{
				float3 hit_point = rays[ray_idx].GetOrigin() + t_hits[hit_idx] * rays[ray_idx].GetDirection();
				if (verbose)
				{
					std::cout << "ray idx " << ray_idx << " hit " << hit_point << std::endl;
				}
				individual_hit_points[ray_idx].push_back(hit_point);
			}
		}
	}
	catch (...)
	{
		std::cout << "Ray Intersection Exception" << std::endl;
	}
	return individual_hit_points;
}


std::ostream& operator<<(std::ostream& os, Contour const& c)
{
	for (int i = 0; i < c.segments.size(); i++)
		std::cout << *c.segments[i] << std::endl;
	return os;
};

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
	/*c->depth = depth + 1;*/
	UpdateChildrenDepth();
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
	//std::cout << "building tree with " << c.size() << std::endl;
	for (int idx = 0; idx < c.size(); idx++)
	{
		if (c[idx]->is_valid)
			contours.push_back(c[idx]);
	}
	contours = c;
	tree_root = std::make_shared<ContourNode>(node_id_counter++);
	BuildTree();
	BuildRootBVH();
	BuildInternalBVHs();
	std::cout << "Tree components " << internal_bvhs.size() << " contours " << contours.size() << std::endl;
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
	std::vector<std::shared_ptr<ContourNode>> nodes = tree_root->GetDescendants();

	for (int idx = 0; idx < nodes.size(); idx++)
	{
		if (nodes[idx]->depth % 2 == 1)
		{
			std::vector<std::shared_ptr<Contour>> contours = nodes[idx]->GetChildrenContours();
			contours.push_back(nodes[idx]->contour);
			BVH* bvh = new BVH({ contours.begin(), contours.end() }, SplitMethod::EqualCounts, 255);
			internal_bvhs.push_back(std::shared_ptr<BVH>(bvh));
		}
			
	}
}

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
		BVH* bvh = new BVH({ contours.begin(), contours.end() }, SplitMethod::EqualCounts, 255);
		tree_global_bvsh.push_back(std::shared_ptr<BVH>(bvh));
	}
}

bool ContourTree::Intersect(Ray& ray, RayIntersectionInfo& info)
{
	bool result = false;
	for (auto bvh : internal_bvhs)
	{
		result = bvh->intersect(ray, info);
	}
	return result;
}

bool ContourTree::AnyIntersect(Ray& ray)
{
	for (auto bvh : internal_bvhs)
	{
		if (bvh->any_intersect(ray))
			return true;
	}
	return false;
}

bool ContourTree::AllIntersect(Ray& ray, RayIntersectionInfo& info)
{
	bool result = false;
	for (auto bvh : internal_bvhs)
	{
			result = bvh->all_intersects(ray, info);
	}
	return result;
}


std::vector<std::vector<std::vector<float3>>> ContourTree::MultiRayAllIntersects(float laser_width_microns, float density, float overlap, float rot_angle_deg, bool verbose=false)
{
	float rot_angle = rot_angle_deg * M_PI / 180.0f;
	Matrix4x4 rot_matrix = Matrix4x4::Rotate(rot_angle, float3(0, 1, 0));
	float3 ray_direction(sinf(rot_angle), 0.0f, cosf(rot_angle));
	if (density < 1.0)
		overlap = 0.0;

	std::vector < std::vector<std::vector<float3>>> contour_tree_hit_points(internal_bvhs.size());
	int bvh_idx = 0;
	if (verbose)
	{
		std::cout << "internal bvhs " << internal_bvhs.size() << std::endl;
	}
	for (auto bvh : internal_bvhs)
	{

		float3 bbox_min = bvh->getBVHBBox().GetpMin();
		float3 bbox_max = bvh->getBVHBBox().GetpMax();
		float3 bbox_center = 0.5f * (bbox_min + bbox_max);
		float bbox_width = (bbox_max.x - bbox_min.x);
		float bbox_depth = (bbox_max.z - bbox_min.z);
		float bbox_diagonal = bbox.Diagonal().length();
		float bbox_max_width = bbox_width * fabsf(cosf(rot_angle)) + bbox_depth * fabsf(sinf(rot_angle));
		float bbox_max_depth = bbox_width * fabsf(sinf(rot_angle)) + bbox_depth * fabsf(cosf(rot_angle));
		int number_of_rays = ceil(bbox_max_width / (laser_width_microns - laser_width_microns * overlap) * 1000 * density);
		if (number_of_rays == 0)
			continue;
		float rays_origin_offset = bbox_max_width / number_of_rays;
		float ray_origin_x = (-bbox_max_width * 0.5) + rays_origin_offset * 0.5;
		float ray_origin_y = bbox_max[1];
		float ray_origin_z = -bbox_max_depth * 0.5 - 1.0;
		float3 ray_origin(ray_origin_x, ray_origin_y, ray_origin_z);
		if (verbose)
		{
		std::cout << "bbox " << bbox_min << " " << bbox_max << std::endl;
		std::cout << "rot angle rad " << rot_angle << std::endl;
		std::cout << "bbox max length " << bbox_max_width << std::endl;
		std::cout << "bbox max depth " << bbox_max_depth << std::endl;
		std::cout << "bbox_center " << bbox_center << std::endl;
		std::cout << "number of rays " << number_of_rays << " ray offset "<< rays_origin_offset << std::endl;
		std::cout << "ray_origin " << ray_origin << std::endl;
		std::cout << "ray_direction " << ray_direction << std::endl;
		std::cout << "rot_matrix " << rot_matrix << std::endl;

		}

		std::vector<Ray> rays(number_of_rays);
		std::vector<RayIntersectionInfo> infos(number_of_rays);

		concurrency::parallel_for(int(0), number_of_rays, [&](int idx)
			{
				float3 ray_o = (rot_matrix * float4(ray_origin.x + rays_origin_offset * idx, ray_origin.y, ray_origin.z, 1.0f) + float4(bbox_center.x, 0, bbox_center.z, 0));
				rays[idx].SetOrigin(ray_o);
				rays[idx].SetDirection(ray_direction);
			});
		if (verbose)
		{
			for (int idx=0; idx < number_of_rays; idx++)
				std::cout << "ray_origin " << rays[idx].GetOrigin() << std::endl;
			
		}
		concurrency::parallel_for(int(0), number_of_rays, [&](int idx)
			{
				bvh->all_intersects(rays[idx], infos[idx]);
			});

		std::vector<std::vector<float3>> contour_hit_points;
		try {
			for (int ray_idx = 0; ray_idx < number_of_rays; ray_idx++)
			{
				std::vector<float> t_hits = *infos[ray_idx].GetHits();
				// DISCARD RAYS WITH ODD NUMBER OF INTERSECTIONS
				if (t_hits.size() % 2 != 0) 
				{
					if (verbose)
					{
						std::cout << "ray idx " << ray_idx << " DISCARDED " << "hits" << t_hits.size() << std::endl;
					}
					continue;
				}

				std::vector<float3> ray_hit_points(t_hits.size());
				for (int hit_idx = 0; hit_idx < t_hits.size(); hit_idx++)
				{
					float3 hit_point = rays[ray_idx].GetOrigin() + t_hits[hit_idx] * rays[ray_idx].GetDirection();
					if (verbose)
					{
						std::cout <<"ray idx " << ray_idx <<  " hit " << hit_point << std::endl;
					}
					ray_hit_points[hit_idx] = hit_point;
				}
				contour_hit_points.push_back(ray_hit_points);
			}
			if (verbose)
			{
				std::cout << "bvh idx " << bvh_idx << std::endl;
				std::cout << "ind hit points length " << contour_tree_hit_points.size() << std::endl;
			}
			contour_tree_hit_points[bvh_idx++] = contour_hit_points;
		}
		catch (...)
		{
			std::cout << "Ray Intersection Exception" << std::endl;
		}
	}
	if (verbose)
	{
		std::cout << "END" << std::endl;
	}
	return contour_tree_hit_points;
}

BBox ContourTree::GetBBox()
{
	return bbox;

}
/////////////////////////////////////////////////////////////////////////
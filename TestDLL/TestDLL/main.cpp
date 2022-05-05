#include <iostream>
#include <BVH.h>
#include <Contour.h>
#include <MyMatrix4x4.h>
#include <random>
#include <time.h>
#include <fstream>
#include <sstream>
#include <ppl.h>
#define _USE_MATH_DEFINES
#include <math.h>

bool load_obj(const std::string filename, float geometry_scaling, bool swap_yz, std::vector<float3> &out_vertices, float3 &b_min, float3 &b_max)
{
	std::vector< unsigned int > vertexIndices;
	std::vector<float3> temp_vertices;
	bool bbox_defined = false;
	std::ifstream file(filename);
	if (file.is_open()) {
		std::string line;
		while (std::getline(file, line)) {
			std::vector<std::string> parsed_line;
			std::string delimiter = " ";
			size_t pos = 0;
			std::string token;
			while ((pos = line.find(delimiter)) != std::string::npos) {
				token = line.substr(0, pos);
				parsed_line.push_back(token);
				line.erase(0, pos + delimiter.length());
			}
			parsed_line.push_back(line);
			if (parsed_line[0] == "v")
			{
				float3 vertex(std::stof(parsed_line[1]), std::stof(parsed_line[2]), std::stof(parsed_line[3]));
				vertex = vertex * geometry_scaling;
				if (swap_yz)
					vertex = float3(-vertex[0], vertex[2], vertex[1]);
				temp_vertices.push_back(vertex);
				if (!bbox_defined)
				{
					b_min = vertex;
					b_max = vertex;
					bbox_defined = true;
				}
				else
				{
					b_min = float3::min(b_min, vertex);
					b_max = float3::max(b_max, vertex);
				}
			}
			else if (parsed_line[0] == "f")
			{
				delimiter = "/";
				for (int i = 1; i < parsed_line.size(); i++)
				{
					pos = parsed_line[i].find(delimiter);
					token = parsed_line[i].substr(0, pos);
					vertexIndices.push_back(std::stoi(token));
				}
				
			}
		}
		file.close();
	}
	float3 bbox_center = 0.5f * (b_max + b_min);
	b_max = b_max - bbox_center;
	b_min = b_min - bbox_center;
	for (unsigned int i = 0; i < vertexIndices.size(); i++) {
		unsigned int vertexIndex = vertexIndices[i];
		float3 vertex = temp_vertices[vertexIndex - 1] - bbox_center;
		out_vertices.push_back(vertex);
	}

	return true;
}

std::shared_ptr<Primitive> triangle_generator()
{
	float3 x0(rand() % 10, rand() % 10, rand() % 10);
	float3 x1(rand() % 10, rand() % 10, rand() % 10);
	float3 x2(rand() % 10, rand() % 10, rand() % 10);
	std::shared_ptr<Primitive> t(new Triangle(x0, x1, x2));
	return t;
};

std::vector<std::shared_ptr<Primitive>> build_box(float l)
{
	float3 v0(l, -l, l);
	float3 v1(l, -l, -l);
	float3 v2(-l, -l, l);
	float3 v3(-l, -l, -l);
	float3 v4(l, l, l);
	float3 v5(l, l, -l);
	float3 v6(-l, l, l);
	float3 v7(-l, l, -l);
	std::shared_ptr<Primitive> t0(new Triangle(v0, v2, v1));
	std::shared_ptr<Primitive> t1(new Triangle(v3, v1, v2));
	std::shared_ptr<Primitive> t2(new Triangle(v1, v0, v5));
	std::shared_ptr<Primitive> t3(new Triangle(v5, v0, v4));
	std::shared_ptr<Primitive> t4(new Triangle(v1, v5, v3));
	std::shared_ptr<Primitive> t5(new Triangle(v3, v5, v7));
	std::shared_ptr<Primitive> t6(new Triangle(v5, v6, v4));
	std::shared_ptr<Primitive> t7(new Triangle(v6, v5, v7));
	std::shared_ptr<Primitive> t8(new Triangle(v6, v7, v2));
	std::shared_ptr<Primitive> t9(new Triangle(v2, v7, v3));
	std::shared_ptr<Primitive> t10(new Triangle(v2, v0, v6));
	std::shared_ptr<Primitive> t11(new Triangle(v0, v4, v6));
	//std::vector<std::shared_ptr<Primitive>> cube = { t0, t1, t2, t3, t4, t5, t6, t7, t8, t9, t10, t11 };
	std::vector<std::shared_ptr<Primitive>> cube = { t0, t1, t2, t3};
	return cube;
}


std::shared_ptr<ContourNode> generate_node() 
{
	return std::make_shared<ContourNode>();
};


void test_contour_intersection()
{
	int  decimals = 4;
	float vertices_scale = pow(10, decimals);
	int laser_width_microns = 600 * vertices_scale;
	std::vector<float> points_a = {
-12690.139, 1500.0, -5127.15, -11071.033, 1500.0, 6393.376, -11071.033, 1500.0, 6393.3765, -10785.309, 1500.0, 8426.41, -9167.514, 1500.0, 7162.4487, 0.0, 1500.0, 0.0, -10785.309, 1500.0, 8426.41, -9167.513, 1500.0, 7162.4487, 7263.993, 1500.0, -7931.5215, -12690.139, 1500.0, -5127.15, 10785.309, 1500.0, -8426.41, 7263.9917, 1500.0, -7931.521, 1903.5205, 1500.0, 769.07227, 12690.139, 1500.0, 5127.15, 0.0, 1500.0, 0.0, 1903.521, 1500.0, 769.07245, 11071.033, 1500.0, -6393.3765, 10785.309, 1500.0, -8426.41, 12690.139, 1500.0, 5127.15, 11071.033, 1500.0, -6393.376
	};

	std::vector<std::shared_ptr<Segment>> primitives_a((int)(points_a.size() / 6));
	for (int i = 0; i < (int)(points_a.size() / 6); i++)
	{
		float3 p0(points_a[i * 6], points_a[i * 6 + 1], points_a[i * 6 + 2]);
		float3 p1(points_a[i * 6 + 3], points_a[i * 6 + 4], points_a[i * 6 + 5]);
		primitives_a[i] = std::shared_ptr<Segment>(new Segment(p0, p1));
	}

	std::cout << "sorting Contour" << std::endl;
	float geometry_scaling = 10000;
	float epsilon = 0.0002 * geometry_scaling;
	clock_t tStart = clock();
	float alignment_epsilon = 1e-2;
	float min_length = laser_width_microns / 1000.0 * 0.5;
	auto sorted_segments = Segment::SortSegments(primitives_a, epsilon, true, alignment_epsilon, true, min_length);
	printf("Time taken: %fs\n", (double)(clock() - tStart) / CLOCKS_PER_SEC);

	/////////////////////BUILD Contour
	float3 n(0.0, 1.0, 0.0);
	std::cout << "Building Contour" << std::endl;
	tStart = clock();
	std::vector<std::shared_ptr<Contour>> sorted_contours(sorted_segments.size());
	for (int i = 0; i < sorted_segments.size(); i++)
		sorted_contours[i] = std::shared_ptr<Contour>(new Contour(primitives_a, n));
	//printf("Time taken: %fs\n", (double)(clock() - tStart) / CLOCKS_PER_SEC);

	/////////////////////BUILD ContourTree
	std::cout << "Building Tree Contour" << std::endl;
	tStart = clock();
	ContourTree contour_tree({ sorted_contours });
	//printf("Time taken: %fs\n", (double)(clock() - tStart) / CLOCKS_PER_SEC);

	std::cout << "Intersecting Contour" << std::endl;


	float rot_angle = 0;
	auto contour_hits = contour_tree.MultiRayAllIntersects(laser_width_microns, 1.0, 0.0, rot_angle, true);
	Contour offset_contour;
	bool is_valid= sorted_contours[0]->OffsetContour(-2 * vertices_scale, offset_contour);
	std::cout << offset_contour << std::endl;
	//auto contour_hits_2 = contour_tree.MultiRayAllIntersects(laser_width_microns, 1.0, 0.0, rot_angle, rot, true);
	std::cout << "DONE\n";

};

std::vector<std::shared_ptr<Contour>> OldMethod(BVH& bvh, Plane& plane, Matrix4x4& tr_matrix, float const geometry_scaling, bool check_min_length, float const segment_min_length, float const epsilon, bool check_alignment, float const alignment_epsilon, bool print_segments)
{
	PlaneIntersectionInfo info;
	bool result = bvh.plane_all_intersects(plane, info);

	std::vector<float3> hits = *info.GetHits();
	std::cout << "%hits: " << hits.size() << std::endl;
	std::vector<float3> transformed_hits(hits.size());
	for (int idx = 0; idx < hits.size(); idx++)
	{
		float4 t_hit = (float4(hits[idx][0], hits[idx][1], hits[idx][2], 1) * tr_matrix.Transpose()) * geometry_scaling;
		transformed_hits[idx] = float3(int(round(t_hit[0])), int(round(t_hit[1])), int(round(t_hit[2])));
	}
	std::cout << "%Trasnformed hits" << std::endl;

	std::vector<std::shared_ptr<Segment>> segment_primitives;
	std::cout << "primitives" << "=[";
	for (int i = 0; i < (int)(transformed_hits.size() / 2); i++)
	{
		float3 p0 = transformed_hits[i * 2];
		float3 p1 = transformed_hits[i * 2 + 1];
		if (float3::length(p0 - p1) == 0)
		{
			continue;
		}
	
		std::cout << "[" << p0 << "]\n[" << p1 << "]" << std::endl;

		segment_primitives.push_back(std::shared_ptr<Segment>(new Segment(p0, p1)));
	}
	std::cout << "];" << std::endl;;
	std::cout << "%created primitives " << segment_primitives.size() << std::endl;
	std::cout << "%epsilon " << epsilon << " remove aligned " << check_alignment << " " << alignment_epsilon << " remove short " << check_min_length << " " << segment_min_length << std::endl;

	auto sorted_segments = Segment::SortSegments(segment_primitives, epsilon, check_alignment, alignment_epsilon, check_min_length, segment_min_length);

	if (print_segments)
	{
		int sorted_primitives_counter = 0;
		for (auto sc : sorted_segments)
			sorted_primitives_counter += sc.size();
		std::cout << "%sorted primitives: " << sorted_primitives_counter << std::endl;

		int sc_counter = 0;
		for (auto c : sorted_segments)
		{
			std::cout << "sc" << sc_counter++ << "=[";
			for (auto s : c)
			{
				std::cout << "[" << s->v0 << "]\n[" << s->v1 << "]" << std::endl;
			}
			std::cout << "];" << std::endl;;
		}
		std::cout << "sc_contours={";
		for (int idx = 0; idx < sc_counter; idx++)
			std::cout << "sc" << idx << ", ";
		std::cout << "};" << std::endl;;
	}
	std::vector<std::shared_ptr<Contour>> sorted_contours;
	int discarded_contours = 0;
	int contour_counter = 0;
	for (int idx = 0; idx < sorted_segments.size(); idx++)
	{
		std::shared_ptr<Contour> c = std::make_shared<Contour>(sorted_segments[idx], plane.GetNormal());
		if (!c->is_valid) {
			if (print_segments)
			{
				std::cout << "%Discarded" << std::endl;
				std::cout << "%Discarded CONTOUR" << std::endl;
				std::cout << "dc" << discarded_contours << "=[";
				for (auto ss : c->segments)
					std::cout << "[" << ss->v0 << "]\n[" << ss->v1 << "]" << std::endl;
				std::cout << "];" << std::endl;
			}
			discarded_contours++;
			continue;
		}
		if (true)
		{
			c->RemoveShortSegments(segment_min_length);
			if (!c->is_valid) {
				if (print_segments)
				{
					std::cout << "%Discarded" << std::endl;
					std::cout << "%Discarded CONTOUR" << std::endl;
					std::cout << "dc" << discarded_contours << "=[";
					for (auto ss : c->segments)
						std::cout << "[" << ss->v0 << "]\n[" << ss->v1 << "]" << std::endl;
					std::cout << "];" << std::endl;
				}
				discarded_contours++;
				continue;
			}
		}
		if (true)
		{
			c->RemoveAlignedSegments(alignment_epsilon);
			if (!c->is_valid) {
				if (print_segments)
				{
					std::cout << "%Discarded" << std::endl;
					std::cout << "%Discarded CONTOUR" << std::endl;
					std::cout << "dc" << discarded_contours << "=[";
					for (auto ss : c->segments)
						std::cout << "[" << ss->v0 << "]\n[" << ss->v1 << "]" << std::endl;
					std::cout << "];" << std::endl;
				}
				discarded_contours++;
				continue;
			}
		}
		if (print_segments)
		{
			std::cout << "%REMOVED ALIGNED CONTOUR" << std::endl;
			std::cout << "cc" << contour_counter++ << "=[";
			for (auto ss : c->segments)
				std::cout << "[" << ss->v0 << "]\n[" << ss->v1 << "]" << std::endl;
			std::cout << "];" << std::endl;
		}

		sorted_contours.push_back(c);

	}
	std::cout << "cleaned_contours={";
	for (int idx = 0; idx < contour_counter; idx++)
		std::cout << "cc" << idx << ", ";
	std::cout << "};" << std::endl;;
	std::cout << "%Discarded contours: " << discarded_contours << std::endl;
	std::cout << "discarded_contours={";
	for (int idx = 0; idx < discarded_contours; idx++)
		std::cout << "dc" << idx << ", ";
	std::cout << "};" << std::endl;;
	return sorted_contours;
}


//std::vector<std::shared_ptr<Contour>> PlaneAllIntersectsContours(BVH& bvh, Plane& plane, Matrix4x4& tr_matrix, float const geometry_scaling, float const segment_min_length)
//{
//
//	bool verbose = false;
//	PlaneIntersectionInfo info;
//	Matrix4x4 tr_matrix_transposed = tr_matrix.Transpose();
//	bool result = bvh.plane_all_intersects(plane, info);
//
//	std::vector<float3> hits = *info.GetHits();
//
//	std::cout << "%hits: " << hits.size() << std::endl;
//	std::vector<float3> transformed_hits(hits.size());
//	for (int idx = 0; idx < hits.size(); idx++)
//	{
//		float4 t_hit = (float4(hits[idx][0], hits[idx][1], hits[idx][2], 1) * tr_matrix_transposed) * geometry_scaling;
//		if (geometry_scaling != 1)
//			transformed_hits[idx] = float3(int(t_hit[0]), int(t_hit[1]), int(t_hit[2]));
//		else
//			transformed_hits[idx] = float3(t_hit[0], t_hit[1], t_hit[2]);
//
//	}
//	if (verbose)
//	{
//		std::cout << "%Trasnformed hits" << std::endl;
//	}
//	std::vector<std::shared_ptr<Segment>> segment_primitives;
//
//	for (int i = 0; i < (int)(transformed_hits.size() / 2); i++)
//	{
//		float3 p0 = transformed_hits[i * 2];
//		float3 p1 = transformed_hits[i * 2 + 1];
//		if (float3::length(p0 - p1) == 0)
//		{
//			continue;
//		}
//		segment_primitives.push_back(std::shared_ptr<Segment>(new Segment(p0, p1)));
//	}
//	if (verbose)
//	{
//		std::cout << "%created primitives " << segment_primitives.size() << std::endl;
//	}
//	float epsilon = 0.001 * geometry_scaling;
//	float alignment_epsilon = 1e-3;
//	bool remove_aligned_segments = true;
//	bool remove_short_segments = false;
//
//	auto sorted_segments = Segment::SortSegments(segment_primitives, epsilon, remove_aligned_segments, alignment_epsilon, remove_short_segments, segment_min_length);
//	std::cout << "Sorted Segments " << sorted_segments.size() << std::endl;
//
//
//	std::vector<std::shared_ptr<Contour>> sorted_contours;
//	int discarded_contours = 0;
//	for (int i = 0; i < sorted_segments.size(); i++) 
//	{
//		Contour c(sorted_segments[i], plane.GetNormal());
//		if (verbose)
//		{
//			std::cout << "%contour idx " << i << std::endl;
//			std::cout << "%SORTED CONTOUR" << std::endl;
//			std::cout << "p1=[";
//			for (auto ss : c.segments)
//				std::cout << "[" << ss->v0 << "]\n[" << ss->v1 << "]" << std::endl;
//			std::cout << "];" << std::endl;;
//		}
//		if (!c.is_valid) {
//			std::cout << "%Discarded" << std::endl;
//			discarded_contours++;
//			continue;
//		}
//		if (c.segments[0]->v0.x == 12770)
//			int a = 0;
//		c.RemoveShortSegments(segment_min_length);
//		if (verbose)
//		{
//			//std::cout << "removed short primitives" << std::endl;
//			std::cout << "%REMOVED SHORT CONTOUR" << std::endl;
//			std::cout << "p2=[";
//			for (auto ss : c.segments)
//				std::cout << "[" << ss->v0 << "]\n[" << ss->v1 << "]" << std::endl;
//			std::cout << "];" << std::endl;
//		}
//		if (!c.is_valid) {
//			std::cout << "%Discarded" << std::endl;
//			discarded_contours++;
//			continue;
//		}
//
//		//c.RemoveAlignedSegments(alignment_epsilon);
//		//if (verbose)
//		//{
//		//	//std::cout << "removed aligned primitives" << std::endl;
//		//	std::cout << "%REMOVED ALIGNED CONTOUR" << std::endl;
//		//	std::cout << "p3=[";
//		//	for (auto ss : c.segments)
//		//		std::cout << "[" << ss->v0 << "]\n[" << ss->v1 << "]" << std::endl;
//		//	std::cout << "];" << std::endl;
//		//}
//		//if (!c.is_valid) {
//		//	std::cout << "%Discarded" << std::endl;
//		//	discarded_contours++;
//		//	continue;
//		//}
//
//
//		sorted_contours.push_back(std::make_shared<Contour>(c));
//
//	}
//	std::cout << "%Discarded contours: " << discarded_contours << std::endl;
//	return sorted_contours;
//};
//


void test_geometry_precision()
{
	//std::string filename("C:/Users/aluo/Documents/Repositories/3DOpenSource_development/resources/Bunny-LowPoly.obj");
	std::string filename("C:/Users/aluo/Documents/Repositories/3DOpenSource_development/resources/EiffelTower_fixed.obj");
	//std::string filename("C:/Users/aluo/Documents/Repositories/3DOpenSource_development/resources/coso.obj");
	//std::string filename("C:/Users/aluo/Documents/Repositories/3DOpenSource_development/resources/circles.obj");
	std::vector<float3> vertices;
	float3 b_min, b_max;
	float geometry_scaling = 10000;
	bool swap_yz = true;
	load_obj(filename, geometry_scaling, swap_yz, vertices, b_min, b_max);
	float4 r0(9.99999975e-05, 0.00000000e+00, 0.00000000e+00, 0.00000000e+00);
	float4 r1(0.00000000e+00, 9.99999975e-05, 0.00000000e+00, 6.04720039e+01);
	float4 r2(0.00000000e+00, 0.00000000e+00, 9.99999975e-05, 0.00000000e+00);
	float4 r3(0., 0., 0., 1.);
	Matrix4x4 t_matrix(r0, r1, r2, r3);
	float3 plane_x0(0, -417220, 0);
	float3 plane_n(0, 1e-4, 0);
	float laser_width_microns = 80;
	//float laser_width_microns = 10000;
	float epsilon = 0.0000 * geometry_scaling;
	float alignment_epsilon = 1e-3;
	bool check_alignment = true;
	bool check_min_length = true;
	bool print_segments = true;
	float min_length = (laser_width_microns * 1e-3 * 0.5) * geometry_scaling * 1 * 0.1;
	float contour_offset = laser_width_microns / 1000.0 * 0.5 * geometry_scaling;

	std::vector<std::shared_ptr<Primitive>> primitives;
	for (int i = 0; i < (int)(vertices.size() / 3); i++)
	{
		float3 p0(vertices[i * 3]);
		float3 p1(vertices[i * 3 + 1]);
		float3 p2(vertices[i * 3 + 2]);
		std::shared_ptr<Primitive> primitive = std::shared_ptr<Triangle>(new Triangle(p0, p1, p2));
		primitives.push_back(primitive);
	}

	BVH bvh(primitives, SplitMethod::EqualCounts, 255);


	Plane plane(plane_x0, plane_n);

	//std::vector<std::shared_ptr<Contour>> sorted_contours = PlaneAllIntersectsContours(bvh, plane, t_matrix, geometry_scaling, min_length);
	//std::cout << "BUILDING TREE CONTOURS 1" << std::endl;
	//	ContourTree sorted_tree(sorted_contours);
	//for (int idx = 0; idx < 1000; idx++)
	//{
	//	ContourTree sorted_tree(sorted_contours);
	//}
	//ContourTree offset_tree = sorted_tree.OffsetContourTree(contour_offset);

	std::ofstream out("out.txt");
	std::streambuf* coutbuf = std::cout.rdbuf(); //save old buf
	std::cout.rdbuf(out.rdbuf());
	bool verbose = true;
	std::vector<std::shared_ptr<Contour>> sorted_contours2 = OldMethod(bvh, plane, t_matrix, geometry_scaling, check_min_length, min_length, epsilon, check_alignment, alignment_epsilon, verbose);
	std::cout << "%BUILDING TREE CONTOURS 2" << std::endl;
	ContourTree sorted_tree2(sorted_contours2);
	ContourTree offset_tree2;
	bool succesful_offset = sorted_tree2.OffsetContourTree(contour_offset, offset_tree2, verbose); 

	
	//auto intersection_points = offset_tree2.MultiRayAllIntersects(laser_width_microns * geometry_scaling,
	//	1, 0, 0, true);

	std::cout << "%SUCCESS" << std::endl;
}

int main() {
	//test_contour_intersection();
	test_geometry_precision();
	return 0;
}

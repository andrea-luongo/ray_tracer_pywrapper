#include <iostream>
#include <BVH.h>
#include <Contour.h>
#include <MyMatrix4x4.h>
#include <random>
#include <time.h>
#include <fstream>
#include <sstream>
#include <ppl.h>

bool load_obj(const char* filename, std::vector<float3> &out_vertices, float3 &b_min, float3 &b_max)
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


//int main() {
//	// TESTING BVH CONSTRUCTION AND INTERSECTION
//	std::vector<std::shared_ptr<Primitive>> primitives;
//	clock_t tStart;
//	bool is_cube = false;
//	float height, width, depth;
//	
//	float4 r0(1, 2, 3, 4);
//	float4 r1(5, 6, 7, 8);
//	float4 r2(9, 10, 11, 12);
//	float4 r3(13, 14, 15, 16);
//	Matrix4x4 m(r0, r1, r2, r3);
//	std::cout << m << std::endl;
//	std::cout << m.Transpose() << std::endl;
//
//
//	////////////////////// CUBE 
//	width = 5;
//	depth = 5;
//	height = 5;
//	is_cube = true;
//	primitives = build_box(width);
//	///////////////////////
//
//	//////////////////////RANDOM PRIMITIVES
//	//generate "number_of_primitives" random triangles and store them in a vector
//	//srand((unsigned)time(NULL));
//	//int number_of_primitives = 65000;
//	//std::cout << "Generating " << number_of_primitives << " random primitives" << std::endl;
//	//tStart = clock();
//	//std::generate_n(std::back_inserter(primitives), number_of_primitives, triangle_generator);
//	//printf("Time taken: %fs\n", (double)(clock() - tStart) / CLOCKS_PER_SEC);
//	/////////////////////////////////////////////////////////
//
//	/////////////// LOAD OBJ
//	//std::cout << "LOAD GEOMETRY" << std::endl;
//	//is_cube = false;
//	//tStart = clock();
//	//std::string filename = "C:\\Users\\aluo\\Documents\\Repos\\3DOpenSource_development\\resources\\Bunny-LowPoly.obj";
//	//filename = "C:\\Users\\aluo\\Documents\\Repos\\3DOpenSource_development\\resources\\closed_bunny_vn_centered.obj";
//	//float3 b_min, b_max;
//	//std::vector<float3> vertices;
//	//load_obj(filename.c_str(), vertices, b_min, b_max);
//	//width = b_max.x - b_min.x;
//	//height = b_max.y - b_min.y;
//	//depth = b_max.z - b_min.z;
//	//for (int i = 0; i < (int)(vertices.size() / 3); i++)
//	//{
//	//	float3 p0 = vertices[i * 3];
//	//	float3 p1 = vertices[i * 3 +1 ];
//	//	float3 p2 = vertices[i * 3 + 2];
//	//	std::shared_ptr<Primitive> primitive = std::shared_ptr<Triangle>(new Triangle(p0, p1, p2));
//	//	primitives.push_back(primitive);
//	//}
//	//printf("Time taken: %fs\n", (double)(clock() - tStart) / CLOCKS_PER_SEC);
//	//////////////////////////////////
//
//	/////////////////////BUILD BVH
//	std::cout << "Building BVH" << std::endl;
//	tStart = clock();
//	BVH* bvh = new BVH(primitives, SplitMethod::EqualCounts);
//	printf("Time taken: %fs\n", (double)(clock() - tStart) / CLOCKS_PER_SEC);
//	/////////////////////////////
//
//	//// PARALLEL FOR TEST
//	tStart = clock();
//	int sum = 0;
//	concurrency::parallel_for(size_t(0), primitives.size(), [&](size_t idx)
//		{
//			sum += idx;
//		});
//	std::cout << sum << std::endl;
//	printf("Parallel Time taken: %fs\n", (double)(clock() - tStart) / CLOCKS_PER_SEC);
//	tStart = clock();
//	int sum2 = 0;
//	for (int idx = 0; idx < primitives.size(); idx++)
//		{
//			sum2 += idx;
//		}
//	std::cout << sum2 << std::endl;
//	printf("Sequential Time taken: %fs\n", (double)(clock() - tStart) / CLOCKS_PER_SEC);
//	////////////////////
//
//	////////////////RAY TRACING
//	int number_of_rays = 10000;
//	int number_of_layers = 10;
//	float w_offset, h_offset;
//	float x_start, y_start;
//	if (is_cube)
//	{
//		w_offset = 2 * width / number_of_rays;
//		h_offset = 2 * height / number_of_layers;
//		x_start = -width;
//		y_start = -height;
//	}
//	else
//	{
//		w_offset = width / number_of_rays;
//		h_offset = height / number_of_layers;
//		x_start = -0.5*width;
//		y_start = -0.5*height;
//	}
//	std::cout << "Testing ray intersection" << std::endl;
//	for (int l_idx = 0; l_idx < number_of_layers; l_idx++)
//	{
//		tStart = clock();
//		for (int ray_idx = 0; ray_idx < number_of_rays; ray_idx++) {
//			float3 o(x_start + ray_idx * w_offset, y_start + l_idx*h_offset, depth);
//			float3 d(0, 0, -1);
//			Ray ray(o, d, 0, 100000, 0, 0);
//			RayIntersectionInfo rinfo;
//			bvh->all_intersects(ray, rinfo);
//		}
//		printf("Time taken: %.2fs\n", (double)(clock() - tStart) / CLOCKS_PER_SEC);
//	}
//	///////////////////////////////
//
//
//	////////////// PLANE TRACING
//	std::cout << "Testing Plane intersection" << std::endl;
//	for (int l_idx = 0; l_idx < number_of_layers; l_idx++)
//	{
//		tStart = clock();
//		float3 x0(0.0, y_start + h_offset * l_idx, 0.0);
//		float3 n(0, 1, 0);
//		Plane plane(x0, n);
//		PlaneIntersectionInfo pinfo;
//		bvh->plane_all_intersects(plane, pinfo);
//		printf("Time taken: %.2fs\n", (double)(clock() - tStart) / CLOCKS_PER_SEC);
//		std::vector<float3> *hits = pinfo.GetHits();
//		//for (int i = 0; i < hits->size(); i++)
//		//{
//		//	std::cout << (*hits)[i] << std::endl;
//		//}
//	}
//	//////////////////
//	return 0;
//}

std::shared_ptr<ContourNode> generate_node() 
{
	return std::make_shared<ContourNode>();
};


int main() {
	// TESTING BVH CONSTRUCTION AND INTERSECTION
	/*std::vector<float> points_a = { 3.0902, 2.5, - 5 ,0.0095 ,2.5, - 5,0.0095, 2.5, - 5, - 3.0902, 2.5, - 5,- 3.0902, 2.5, - 5, - 3.0902, 2.5, 0.0153,- 3.0902, 2.5, 0.0153 ,- 3.0902, 2.5, 5,
			- 3.0902, 2.5, 5, - 0.0095, 2.5, 5, - 0.0095, 2.5, 5, 3.0902, 2.5, 5, 3.0902, 2.5, 5, 3.0902, 2.5, - 0.0153, 3.0902, 2.5, - 0.0153, 3.0902, 2.5, - 5 };*/

	std::vector<float> points_a = { 1, 0, 1 , 1, 0, -1, 1, 0, -1, -1, 0, -1, -1, 0, -1, -1, 0, 1, -1, 0, 1, 1, 0, 1};

	std::vector<std::shared_ptr<Segment>> primitives_a((int)(points_a.size() / 6));
	for (int i = 0; i < (int)(points_a.size() / 6); i++)
	{
		float3 p0(points_a[i * 6], points_a[i * 6 + 1], points_a[i * 6 + 2]);
		float3 p1(points_a[i * 6 + 3], points_a[i * 6 + 4], points_a[i * 6 + 5]);
		primitives_a[i] = std::shared_ptr<Segment>(new Segment(p0, p1));
	}


	std::vector<float> points_b(points_a.size());
	for (int i = 0; i < points_b.size(); i++)
	{
		float p = points_a[i];
		if (i % 3 != 1)
			p *= 0.5f;
		points_b[i] = p;
	}
	std::vector<std::shared_ptr<Segment>> primitives_b((int)(points_b.size() / 6));
	for (int i = 0; i < (int)(points_b.size() / 6); i++)
	{
		float3 p0(points_b[i * 6], points_b[i * 6 + 1], points_b[i * 6 + 2]);
		float3 p1(points_b[i * 6 + 3], points_b[i * 6 + 4], points_b[i * 6 + 5]);
		primitives_b[i] = std::shared_ptr<Segment>(new Segment(p0, p1));
	}

	std::vector<float> points_c(points_a.size());
	for (int i = 0; i < points_c.size(); i++)
	{
		float p = points_a[i];
		if (i % 3 != 1)
			p = p*1.5f + 20;
		points_c[i] = p;
	}
	std::vector<std::shared_ptr<Segment>> primitives_c((int)(points_c.size() / 6));
	for (int i = 0; i < (int)(points_c.size() / 6); i++)
	{
		float3 p0(points_c[i * 6], points_c[i * 6 + 1], points_c[i * 6 + 2]);
		float3 p1(points_c[i * 6 + 3], points_c[i * 6 + 4], points_c[i * 6 + 5]);
		primitives_c[i] = std::shared_ptr<Segment>(new Segment(p0, p1));
	}

	std::vector<float> points_d(points_a.size());
	for (int i = 0; i < points_d.size(); i++)
	{
		float p = points_a[i];
		if (i % 3 != 1)
			p = p*0.2f + 20;
		points_d[i] = p;
	}
	std::vector<std::shared_ptr<Segment>> primitives_d((int)(points_d.size() / 6));
	for (int i = 0; i < (int)(points_d.size() / 6); i++)
	{
		float3 p0(points_d[i * 6], points_d[i * 6 + 1], points_d[i * 6 + 2]);
		float3 p1(points_d[i * 6 + 3], points_d[i * 6 + 4], points_d[i * 6 + 5]);
		primitives_d[i] = std::shared_ptr<Segment>(new Segment(p0, p1));
	}


	clock_t tStart;
	

	/////////////////////BUILD Contour
	float3 n(0.0, 1.0, 0.0);
	std::cout << "Building Contour" << std::endl;
	tStart = clock();
	std::shared_ptr<Contour> contour_a(new Contour(primitives_a, n));
	std::shared_ptr<Contour> contour_b(new Contour(primitives_b, n));
	std::shared_ptr<Contour> contour_c(new Contour(primitives_c, n));
	std::shared_ptr<Contour> contour_d(new Contour(primitives_d, n));
	printf("Time taken: %fs\n", (double)(clock() - tStart) / CLOCKS_PER_SEC);


	/////////////TEST CONTOURS INCLUSION
	float t_hit;
	int r = Contour::EvaluateContoursRelationship(*contour_a, *contour_b, t_hit);
	std::cout << r << ' ' << t_hit << std::endl;
	r = Contour::EvaluateContoursRelationship(*contour_a, *contour_c, t_hit);
	std::cout << r << ' ' << t_hit << std::endl;
	r = Contour::EvaluateContoursRelationship(*contour_a, *contour_d, t_hit);
	std::cout << r << ' ' << t_hit << std::endl;
	r = Contour::EvaluateContoursRelationship(*contour_b, *contour_c, t_hit);
	std::cout << r << ' ' << t_hit << std::endl;
	r = Contour::EvaluateContoursRelationship(*contour_b, *contour_d, t_hit);
	std::cout << r << ' ' << t_hit << std::endl;
	r = Contour::EvaluateContoursRelationship(*contour_c, *contour_d, t_hit);
	std::cout << r << ' ' << t_hit << std::endl;


	/////////////////////BUILD ContourTree
	std::cout << "Building Tree Contour" << std::endl;
	tStart = clock();
	ContourTree contour_tree({ contour_a, contour_b, contour_c, contour_d });
	printf("Time taken: %fs\n", (double)(clock() - tStart) / CLOCKS_PER_SEC);

	std::shared_ptr<ContourNode> root = contour_tree.tree_root;
	std::vector<std::shared_ptr<ContourNode>> child = root->GetChildren();
	std::vector<std::shared_ptr<ContourNode>> desc = root->GetDescendants();

	for (auto branch_bvhs : contour_tree.tree_individual_bvhs) 
	{
			for (auto bvh : branch_bvhs) 
			{
				int number_of_rays = 10;
				BBox bbox = bvh->getBVHBBox();
				float3 pmin = bbox.GetpMin();
				float3 pmax = bbox.GetpMax();
				float width = pmax[0] - pmin[0];
				float3 origin_0(pmin[0], pmin[1], pmin[2] + 100.0);
				float3 direction(0.0, 0.0, -1.0);
				for (int idx = 0; idx < number_of_rays; idx++)
				{
					float3 origin = origin_0 + float3(width / number_of_rays * idx, 0.0, 0.0);
					Ray ray(origin, direction, 0, 1000, 0, 0);
					RayIntersectionInfo info;
					bvh->all_intersects(ray, info);
					auto hits = *info.GetHits();
					int a = 1;
					for (auto hit : hits)
					{
						std::cout << origin + hit * direction << std::endl;
					std::cout <<  hit  << std::endl;

					}
				std::cout << "\n" << std::endl;
				}
				std::cout << "\n" << std::endl;
			}
	}

	std::cout << "contours" << std::endl;
	for (auto c : contour_tree.contours)
	{
			int number_of_rays = 10;
			BBox bbox = c->bvh->getBVHBBox();
			float3 pmin = bbox.GetpMin();
			float3 pmax = bbox.GetpMax();
			float width = pmax[0] - pmin[0];
			float3 origin_0(pmin[0], pmin[1], pmin[2] + 100.0);
			float3 direction(0.0, 0.0, -1.0);
			for (int idx = 0; idx < number_of_rays; idx++)
			{
				float3 origin = origin_0 + float3(width / number_of_rays * idx, 0.0, 0.0);
				Ray ray(origin, direction, 0, 1000, 0, 0);
				RayIntersectionInfo info;
				c->AllIntersect(ray, info);
				auto hits = *info.GetHits();
				int a = 1;
				for (auto hit : hits)
				{
					std::cout << origin + hit * direction << std::endl;
					std::cout <<  hit  << std::endl;

				}
			std::cout << "\n" << std::endl;
			}
			std::cout << "\n" << std::endl;
	}

	return 0;
}

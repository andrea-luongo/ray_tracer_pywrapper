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


std::shared_ptr<ContourNode> generate_node() 
{
	return std::make_shared<ContourNode>();
};


void test_contour_intersection()
{
	std::vector<float> points_a = {
1.0596, 54.5, 0.6947, 0.8988, 54.5, 0.6949, 0.8988, 54.5, 0.6949, 0.6949, 54.5, 0.6949, 0.6949, 54.5, 0.6949, 0.6947, 54.5, 0.863, 0.6947, 54.5, 0.863, 0.6947, 54.5, 1.0596, 0.6947, 54.5, 1.0596, 0.863, 54.5, 1.0596, 0.863, 54.5, 1.0596, 1.0581, 54.5, 1.0581, 1.0581, 54.5, 1.0581, 1.0596, 54.5, 0.863, 1.0596, 54.5, 0.863, 1.0596, 54.5, 0.6947
	};

	std::vector<std::shared_ptr<Segment>> primitives_a((int)(points_a.size() / 6));
	for (int i = 0; i < (int)(points_a.size() / 6); i++)
	{
		float3 p0(points_a[i * 6], points_a[i * 6 + 1], points_a[i * 6 + 2]);
		float3 p1(points_a[i * 6 + 3], points_a[i * 6 + 4], points_a[i * 6 + 5]);
		primitives_a[i] = std::shared_ptr<Segment>(new Segment(p0, p1));
	}

	clock_t tStart;

	/////////////////////BUILD Contour
	float3 n(0.0, 1.0, 0.0);
	std::cout << "Building Contour" << std::endl;
	tStart = clock();
	std::shared_ptr<Contour> contour_a(new Contour(primitives_a, n));
	printf("Time taken: %fs\n", (double)(clock() - tStart) / CLOCKS_PER_SEC);

	/////////////////////BUILD ContourTree
	std::cout << "Building Tree Contour" << std::endl;
	tStart = clock();
	ContourTree contour_tree({ contour_a });
	printf("Time taken: %fs\n", (double)(clock() - tStart) / CLOCKS_PER_SEC);

	std::cout << "Intersecting Contour" << std::endl;
	int  decimals = 4;
	float vertices_scale = pow(10, decimals);
	int laser_width_microns = 600;
	float slice_height_offset = 0.5;
	float rot_angle = 45;
	auto contour_hits = contour_a->MultiRayAllIntersects(laser_width_microns, 1.0, 0.0, rot_angle, true);
	//auto contour_hits_2 = contour_tree.MultiRayAllIntersects(laser_width_microns, 1.0, 0.0, rot_angle, rot, true);
	std::cout << "DONE\n";
	
};

void test_geometry_precision()
{
	//std::string filename = "C:/Users/aluo/Documents/Repositories/3DOpenSource_development/resources/EiffelTower_fixed.obj";
	//load_obj(filename, std::vector<float3> &out_vertices, float3 & b_min, float3 & b_max)
}

int main() {
	test_contour_intersection();
	return 0;
}

#include <iostream>
#include <BVH.h>
#include <structs.h>
#include <MyFloat3.h>
#include <random>
#include <time.h>

std::shared_ptr<Primitive> triangle_generator()
{
	float3 x0(rand() % 100, rand() % 100, rand() % 100);
	float3 x1(rand() % 100, rand() % 100, rand() % 100);
	float3 x2(rand() % 100, rand() % 100, rand() % 100);
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
	std::vector<std::shared_ptr<Primitive>> cube = { t0, t1, t2, t3, t4, t5, t6, t7, t8, t9, t10, t11 };
	return cube;
}


int main() {
	// TESTING BVH CONSTRUCTION AND INTERSECTION

	//generate "number_of_primitives" random triangles and store them in a vector
	srand((unsigned)time(NULL));
	int number_of_primitives = 65000;
	std::vector<std::shared_ptr<Primitive>> primitives;
	std::cout << "Generating " << number_of_primitives << " random primitives" << std::endl;

	clock_t tStart = clock();
	float box_size = 5;
	primitives = build_box(box_size);
	//std::generate_n(std::back_inserter(primitives), number_of_primitives, triangle_generator);

	printf("Time taken: %.2fs\n", (double)(clock() - tStart) / CLOCKS_PER_SEC);

	//create BVH and time it
	std::cout << "Building BVH" << std::endl;

	tStart = clock();

	BVH* bvh = new BVH(primitives, SplitMethod::EqualCounts);

	printf("Time taken: %.2fs\n", (double)(clock() - tStart) / CLOCKS_PER_SEC);

	//test Ray BVH intersection
	int number_of_rays = 1000;
	float offset = 2.0 * box_size / number_of_rays;
	std::cout << "Testing ray intersection" << std::endl;
	tStart = clock();

	for (int i = 0; i < number_of_rays; i++) {
		float3 o(-box_size + i * offset, 0, box_size + 1);
		float3 d(0, 0, -1);
		Ray ray(o, d, 0, 1000, 0, 0);
		RayIntersectionInfo rinfo;
		bvh->all_intersects(ray, rinfo);
	}
	printf("Time taken: %.2fs\n", (double)(clock() - tStart) / CLOCKS_PER_SEC);
	std::cout << "Testing plane intersection" << std::endl;
	tStart = clock();
	Plane plane(float3(0, 0, 0), float3(0, 1, 0));
	PlaneIntersectionInfo pinfo;
	bvh->plane_all_intersects(plane, pinfo);
	//for (int i = 0; i < primitives.size(); i++)
	//{
	//	//PlaneIntersectionInfo pinfo;
	//	primitives[i]->PlaneIntersect(plane, pinfo);
	//	std::vector<float3>* hits = pinfo.GetHits();
	//	//for (int j = 0; j < hits->size(); j++)
	//	//	std::cout << (*hits)[j] << std::endl;
	//	//std::cout << std::endl;
	//}
	printf("Time taken: %.2fs\n", (double)(clock() - tStart) / CLOCKS_PER_SEC);
	
	return 0;
}


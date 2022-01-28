#pragma once
#ifndef RAYTRACERDLL_API
#ifdef RAYTRACERDLL_EXPORTS
#define RAYTRACERDLL_API __declspec(dllexport)
#else
#define RAYTRACERDLL_API __declspec(dllimport)
#endif
#endif
#ifndef MyDouble3_H
#define MyDouble3_H
#include <iostream>
//#include "MyFloat3.h"
class float3;
class int3;

class double3 {
public:
	double x;
	double y;
	double z;
	RAYTRACERDLL_API double3();
	RAYTRACERDLL_API double3(double x);
	RAYTRACERDLL_API double3(double x, double y, double z);
	RAYTRACERDLL_API double3(float3 d);
	RAYTRACERDLL_API double3(int3 d);
	RAYTRACERDLL_API double3(const double3 &d);
	RAYTRACERDLL_API double operator[](int i);
	RAYTRACERDLL_API double operator[](int i) const;
	RAYTRACERDLL_API void operator=(const double3& a);
	RAYTRACERDLL_API double length() const;
	RAYTRACERDLL_API static double length(const double3& a);
	RAYTRACERDLL_API static double3 normalize(const double3& a);
	RAYTRACERDLL_API static double dot(const double3& a, const double3& b);
	RAYTRACERDLL_API float dot(const double3& b) const;
	RAYTRACERDLL_API static double3 cross(const double3& a, const double3& b);
	RAYTRACERDLL_API static double3 abs(const double3& a);
	RAYTRACERDLL_API double min();
	RAYTRACERDLL_API static double3 min(const double3& a, const double3& b);
	RAYTRACERDLL_API double max();
	RAYTRACERDLL_API static double3 max(const double3& a, const double3& b);
};

RAYTRACERDLL_API double3 operator+(const double3& a, const double3& b);
RAYTRACERDLL_API double3 operator+(const double3& a, double c);
RAYTRACERDLL_API double3 operator+(double c, const double3& a);
RAYTRACERDLL_API double3 operator-(const double3& a, const double3& b);
RAYTRACERDLL_API double3 operator-(const double3& a, double c);
RAYTRACERDLL_API double3 operator-(double c, const double3& a);
RAYTRACERDLL_API double3 operator*(const double3& a, const double3& b);
RAYTRACERDLL_API double3 operator*(const double3& a, double c);
RAYTRACERDLL_API double3 operator*(double c, const double3& a);
RAYTRACERDLL_API double3 operator/(const double3& a, const double3& b);
RAYTRACERDLL_API double3 operator/(const double3& a, double c);
RAYTRACERDLL_API double3 operator/(double c, const double3& a);
RAYTRACERDLL_API std::ostream& operator<<(std::ostream& os, double3 const& v);

#endif 
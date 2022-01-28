#pragma once
#ifndef RAYTRACERDLL_API
#ifdef RAYTRACERDLL_EXPORTS
#define RAYTRACERDLL_API __declspec(dllexport)
#else
#define RAYTRACERDLL_API __declspec(dllimport)
#endif
#endif
#ifndef MyFloat3_H
#define MyFloat3_H
#include <iostream>

class double3;
class float4;
class int3;

class float3 {
public:
	float x;
	float y;
	float z;
	RAYTRACERDLL_API float3();
	RAYTRACERDLL_API float3(float x);
	RAYTRACERDLL_API float3(float x, float y, float z);
	RAYTRACERDLL_API float3(float4 d);
	RAYTRACERDLL_API float3(double3 d);
	RAYTRACERDLL_API float3(int3 d);
	RAYTRACERDLL_API float3(const float3&);
	RAYTRACERDLL_API float operator[](int i);
	RAYTRACERDLL_API float operator[](int i) const;
	RAYTRACERDLL_API void operator=(const float3& a);
	RAYTRACERDLL_API float length() const;
	RAYTRACERDLL_API static float length(const float3& a);
	RAYTRACERDLL_API static float3 normalize(const float3& a);
	RAYTRACERDLL_API static float dot(const float3& a, const float3& b);
	RAYTRACERDLL_API float dot(const float3& b) const;
	RAYTRACERDLL_API static float3 cross(const float3& a, const float3& b);
	RAYTRACERDLL_API static float3 abs(const float3& a);
	RAYTRACERDLL_API float min();
	RAYTRACERDLL_API static float3 min(const float3& a, const float3& b);
	RAYTRACERDLL_API float max();
	RAYTRACERDLL_API static float3 max(const float3& a, const float3& b);
};

RAYTRACERDLL_API float3 operator+(const float3& a, const float3& b);
RAYTRACERDLL_API float3 operator+(const float3& a, float c);
RAYTRACERDLL_API float3 operator+(float c, const float3& a);
RAYTRACERDLL_API float3 operator-(const float3& a, const float3& b);
RAYTRACERDLL_API float3 operator-(const float3& a, float c);
RAYTRACERDLL_API float3 operator-(float c, const float3& a);
RAYTRACERDLL_API float3 operator*(const float3& a, const float3& b);
RAYTRACERDLL_API float3 operator*(const float3& a, float c);
RAYTRACERDLL_API float3 operator*(float c, const float3& a);
RAYTRACERDLL_API float3 operator/(const float3& a, const float3& b);
RAYTRACERDLL_API float3 operator/(const float3& a, float c);
RAYTRACERDLL_API float3 operator/(float c, const float3& a);
RAYTRACERDLL_API std::ostream& operator<<(std::ostream& os, float3 const& v);

#endif
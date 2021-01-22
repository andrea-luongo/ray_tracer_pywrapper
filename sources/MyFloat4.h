#pragma once
#ifndef RAYTRACERDLL_API
#ifdef RAYTRACERDLL_EXPORTS
#define RAYTRACERDLL_API __declspec(dllexport)
#else
#define RAYTRACERDLL_API __declspec(dllimport)
#endif
#endif
#ifndef MyFloat4_H
#define MyFloat4_H
#include <iostream>

class double3;
class float3;

class float4 {
public:
	float x;
	float y;
	float z;
	float w;
	RAYTRACERDLL_API float4();
	RAYTRACERDLL_API float4(float x);
	RAYTRACERDLL_API float4(float x, float y, float z, float w);
	RAYTRACERDLL_API float4(double3 d);
	RAYTRACERDLL_API float4(float3 d);
	RAYTRACERDLL_API float4(const float4&);
	RAYTRACERDLL_API float operator[](int i);
	RAYTRACERDLL_API float operator[](int i) const;
	RAYTRACERDLL_API void operator=(const float4& a);
	RAYTRACERDLL_API float length() const;
	RAYTRACERDLL_API static float length(const float4& a);
	RAYTRACERDLL_API static float4 normalize(const float4& a);
	RAYTRACERDLL_API static float dot(const float4& a, const float4& b);
	RAYTRACERDLL_API float dot(const float4& b) const;
	RAYTRACERDLL_API static float4 abs(const float4& a);
	RAYTRACERDLL_API float min();
	RAYTRACERDLL_API static float4 min(const float4& a, const float4& b);
	RAYTRACERDLL_API float max();
	RAYTRACERDLL_API static float4 max(const float4& a, const float4& b);
};

RAYTRACERDLL_API float4 operator+(const float4& a, const float4& b);
RAYTRACERDLL_API float4 operator+(const float4& a, float c);
RAYTRACERDLL_API float4 operator+(float c, const float4& a);
RAYTRACERDLL_API float4 operator-(const float4& a, const float4& b);
RAYTRACERDLL_API float4 operator-(const float4& a, float c);
RAYTRACERDLL_API float4 operator-(float c, const float4& a);
RAYTRACERDLL_API float4 operator*(const float4& a, const float4& b);
RAYTRACERDLL_API float4 operator*(const float4& a, float c);
RAYTRACERDLL_API float4 operator*(float c, const float4& a);
RAYTRACERDLL_API float4 operator/(const float4& a, const float4& b);
RAYTRACERDLL_API float4 operator/(const float4& a, float c);
RAYTRACERDLL_API float4 operator/(float c, const float4& a);
RAYTRACERDLL_API std::ostream& operator<<(std::ostream& os, float4 const& v);

#endif
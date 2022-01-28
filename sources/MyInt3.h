#pragma once
#ifndef RAYTRACERDLL_API
#ifdef RAYTRACERDLL_EXPORTS
#define RAYTRACERDLL_API __declspec(dllexport)
#else
#define RAYTRACERDLL_API __declspec(dllimport)
#endif
#endif
#ifndef MyInt3_H
#define MyInt3_H
#include <iostream>

class double3;
class float4;
class float3;

class int3 {
public:
	int32_t x;
	int32_t y;
	int32_t z;
	RAYTRACERDLL_API int3();
	RAYTRACERDLL_API int3(int32_t x);
	RAYTRACERDLL_API int3(int32_t x, int32_t y, int32_t z);
	//RAYTRACERDLL_API int3(float3 d);
	//RAYTRACERDLL_API int3(double3 d);
	RAYTRACERDLL_API int3(const int3&);
	RAYTRACERDLL_API int32_t operator[](int i);
	RAYTRACERDLL_API int32_t operator[](int i) const;
	RAYTRACERDLL_API void operator=(const int3& a);
	RAYTRACERDLL_API float length() const;
	RAYTRACERDLL_API static float length(const int3& a);
	RAYTRACERDLL_API static float3 normalize(const int3& a);
	RAYTRACERDLL_API static float dot(const int3& a, const int3& b);
	RAYTRACERDLL_API float dot(const int3& b) const;
	RAYTRACERDLL_API static int3 cross(const int3& a, const int3& b);
	RAYTRACERDLL_API static int3 abs(const int3& a);
	RAYTRACERDLL_API int32_t min();
	RAYTRACERDLL_API static int3 min(const int3& a, const int3& b);
	RAYTRACERDLL_API int32_t max();
	RAYTRACERDLL_API static int3 max(const int3& a, const int3& b);
};

RAYTRACERDLL_API int3 operator+(const int3& a, const int3& b);
RAYTRACERDLL_API float3 operator+(const int3& a, float c);
RAYTRACERDLL_API float3 operator+(float c, const int3& a);
RAYTRACERDLL_API int3 operator+(const int3& a, int32_t c);
RAYTRACERDLL_API int3 operator+(int32_t c, const int3& a);
RAYTRACERDLL_API int3 operator-(const int3& a, const int3& b);
RAYTRACERDLL_API float3 operator-(const int3& a, float c);
RAYTRACERDLL_API float3 operator-(float c, const int3& a);
RAYTRACERDLL_API int3 operator-(const int3& a, int32_t c);
RAYTRACERDLL_API int3 operator-(int32_t c, const int3& a);
RAYTRACERDLL_API int3 operator*(const int3& a, const int3& b);
RAYTRACERDLL_API float3 operator*(const int3& a, float c);
RAYTRACERDLL_API float3 operator*(float c, const int3& a);
RAYTRACERDLL_API int3 operator*(const int3& a, int32_t c);
RAYTRACERDLL_API int3 operator*(int32_t c, const int3& a);
RAYTRACERDLL_API float3 operator/(const int3& a, const int3& b);
RAYTRACERDLL_API float3 operator/(const int3& a, float c);
RAYTRACERDLL_API float3 operator/(float c, const int3& a);
RAYTRACERDLL_API std::ostream& operator<<(std::ostream& os, int3 const& v);

#endif
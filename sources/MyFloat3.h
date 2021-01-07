#pragma once
#ifndef RAYTRACER_API
#ifdef RAYTRACER_EXPORTS
#define RAYTRACER_API __declspec(dllexport)
#else
#define RAYTRACER_API __declspec(dllimport)
#endif
#endif
#include <iostream>
#ifndef MyFloat_H
#define MyFloat_H


class float3 {
public:
	float x;
	float y;
	float z;
	RAYTRACER_API float3();
	RAYTRACER_API float3(float x);
	RAYTRACER_API float3(float x, float y, float z);
	RAYTRACER_API float operator[](int i);
	RAYTRACER_API float operator[](int i) const;
	RAYTRACER_API void operator=(const float3& a);
	RAYTRACER_API float length() const;
	RAYTRACER_API static float length(const float3& a);
	RAYTRACER_API static float3 normalize(const float3& a);
	RAYTRACER_API static float dot(const float3& a, const float3& b);
	RAYTRACER_API static float3 cross(const float3& a, const float3& b);
	RAYTRACER_API static float3 abs(const float3& a);
	RAYTRACER_API float min();
	RAYTRACER_API static float3 min(const float3& a, const float3& b);
	RAYTRACER_API float max();
	RAYTRACER_API static float3 max(const float3& a, const float3& b);
};

RAYTRACER_API float3 operator+(const float3& a, const float3& b);
RAYTRACER_API float3 operator+(const float3& a, float c);
RAYTRACER_API float3 operator+(float c, const float3& a);
RAYTRACER_API float3 operator-(const float3& a, const float3& b);
RAYTRACER_API float3 operator-(const float3& a, float c);
RAYTRACER_API float3 operator-(float c, const float3& a);
RAYTRACER_API float3 operator*(const float3& a, const float3& b);
RAYTRACER_API float3 operator*(const float3& a, float c);
RAYTRACER_API float3 operator*(float c, const float3& a);
RAYTRACER_API float3 operator/(const float3& a, const float3& b);
RAYTRACER_API float3 operator/(const float3& a, float c);
RAYTRACER_API float3 operator/(float c, const float3& a);
RAYTRACER_API std::ostream& operator<<(std::ostream& os, float3 const& v);

#endif // MyComplex_H
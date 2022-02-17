#pragma once
#ifndef RAYTRACERDLL_API
#ifdef RAYTRACERDLL_EXPORTS
#define RAYTRACERDLL_API __declspec(dllexport)
#else
#define RAYTRACERDLL_API __declspec(dllimport)
#endif
#endif
#ifndef MyMatrix4x4_H
#define MyMatrix4x4_H
#include <iostream>
#include <array>

class double3;
class float3;
class float4;

class Matrix4x4 {
public:
	std::array<float, 16> m_elements = {0.0f};

	RAYTRACERDLL_API Matrix4x4();
	RAYTRACERDLL_API Matrix4x4(const std::array<float, 16>& elements);
	RAYTRACERDLL_API Matrix4x4(const float4& r_0, const float4& r_1, const float4& r_2, const float4& r_3);
	RAYTRACERDLL_API Matrix4x4(const Matrix4x4&);
	RAYTRACERDLL_API float operator[](int i);
	RAYTRACERDLL_API float operator[](int i) const;
	RAYTRACERDLL_API void operator=(const Matrix4x4& a);
	RAYTRACERDLL_API float4 GetRow(int r) const;
	RAYTRACERDLL_API float4 GetColumn(int c) const;
	RAYTRACERDLL_API Matrix4x4 Transpose() const;
	RAYTRACERDLL_API static Matrix4x4 Rotate(float angle_rad, float3 axis);
	//RAYTRACERDLL_API static float dot(const float4& a, const float4& b);
	
};

RAYTRACERDLL_API Matrix4x4 operator+(const Matrix4x4& a, const Matrix4x4& b);
RAYTRACERDLL_API Matrix4x4 operator+(const Matrix4x4& a, float c);
RAYTRACERDLL_API Matrix4x4 operator+(float c, const Matrix4x4& a);
RAYTRACERDLL_API Matrix4x4 operator-(const Matrix4x4& a, const Matrix4x4& b);
RAYTRACERDLL_API Matrix4x4 operator-(const Matrix4x4& a, float c);
RAYTRACERDLL_API Matrix4x4 operator-(float c, const Matrix4x4& a);
RAYTRACERDLL_API Matrix4x4 operator*(const Matrix4x4& a, const Matrix4x4& b);
RAYTRACERDLL_API float4 operator*(const Matrix4x4& a, const float4& b);
RAYTRACERDLL_API float4 operator*(const float4& a, const Matrix4x4& b);
RAYTRACERDLL_API Matrix4x4 operator*(const Matrix4x4& a, float c);
RAYTRACERDLL_API Matrix4x4 operator*(float c, const Matrix4x4& a);
RAYTRACERDLL_API Matrix4x4 operator/(const Matrix4x4& a, float c);
RAYTRACERDLL_API std::ostream& operator<<(std::ostream& os, Matrix4x4 const& v);

#endif
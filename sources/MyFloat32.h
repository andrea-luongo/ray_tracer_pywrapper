#pragma once

#include <iostream>
#ifndef MyFloat_H
#define MyFloat_H


class float3 {
public:
	float x;
	float y;
	float z;
	 float3();
	 float3(float x);
	 float3(float x, float y, float z);
	 float operator[](int i);
	 float operator[](int i) const;
	 void operator=(const float3& a);
	 float length() const;
	 static float length(const float3& a);
	 static float3 normalize(const float3& a);
	 static float dot(const float3& a, const float3& b);
	 static float3 cross(const float3& a, const float3& b);
	 static float3 abs(const float3& a);
	 float min();
	 static float3 min(const float3& a, const float3& b);
	 float max();
	 static float3 max(const float3& a, const float3& b);
};

 float3 operator+(const float3& a, const float3& b);
 float3 operator+(const float3& a, float c);
 float3 operator+(float c, const float3& a);
 float3 operator-(const float3& a, const float3& b);
 float3 operator-(const float3& a, float c);
 float3 operator-(float c, const float3& a);
 float3 operator*(const float3& a, const float3& b);
 float3 operator*(const float3& a, float c);
 float3 operator*(float c, const float3& a);
 float3 operator/(const float3& a, const float3& b);
 float3 operator/(const float3& a, float c);
 float3 operator/(float c, const float3& a);
 std::ostream& operator<<(std::ostream& os, float3 const& v);

#endif // MyComplex_H
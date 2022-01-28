#include "MyFloat3.h"
#include "MyFloat4.h"
#include "MyDouble3.h"
#include "MyInt3.h"
#include <algorithm>

int3::int3()
{
	this->x = 0;
	this->y = 0;
	this->z = 0;
};

int3::int3(int32_t x)
{
	this->x = x;
	this->y = x;
	this->z = x;
};

int3::int3(int32_t x, int32_t y, int32_t z)
{
	this->x = x;
	this->y = y;
	this->z = z;
};

//int3::int3( float3 d)
//{
//	this->x = d.x;
//	this->y = d.y;
//	this->z = d.z;
//};

int3::int3(const int3& d)
{
	this->x = d.x;
	this->y = d.y;
	this->z = d.z;
};

//int3::int3(double3 d)
//{
//	this->x = d.x;
//	this->y = d.y;
//	this->z = d.z;
//}

int32_t int3::operator[](int i) {
	if (i == 0)
		return x;
	else if(i == 1)
		return y;
	else
		return z;
}

int32_t int3::operator[](int i) const{
	if (i == 0)
		return x;
	else if(i == 1)
		return y;
	else
		return z;
}

void int3::operator=(const int3& a) {
	x = a.x;
	y = a.y;
	z = a.z;
}

float int3::length() const
{
	return sqrtf(dot(*this, *this));
}

float int3::length(const int3& a)
{
	return sqrtf(dot(a, a));
}

float3 int3::normalize(const int3& a)
{
	return a / a.length();
}

float int3::dot(const int3& a, const int3& b)
{
	return a.x * b.x + a.y * b.y + a.z * b.z;
}

float int3::dot(const int3& b) const
{
	return x * b.x + y * b.y + z * b.z;
}

int3 int3::cross(const int3& a, const int3& b)
{
	int3 result(a.y*b.z - a.z*b.y, a.z*b.x - a.x*b.z, a.x*b.y-a.y*b.x);
	return result;
}

int3 int3::abs(const int3& a)
{
	int3 result(std::abs(a.x), std::abs(a.y), std::abs(a.z));
	return result;
}

int32_t int3::min()
{
	return std::min(x, std::min(y, z));
}

int3 int3::min(const int3& a, const int3& b)
{
	int3 result(std::min(a.x, b.x), std::min(a.y, b.y), std::min(a.z, b.z));
	return result;
}

int32_t int3::max()
{
	return std::max(x, std::max(y, z));
}

int3 int3::max(const int3& a, const int3& b)
{
	int3 result(std::max(a.x, b.x), std::max(a.y, b.y), std::max(a.z, b.z));
	return result;
}

int3 operator+(const int3& a, const int3& b)
{
	int3 result = { a.x + b.x, a.y + b.y, a.z + b.z };
	return result;
}

float3 operator+(const int3& a, float c)
{
	float3 result = { a.x + c, a.y + c, a.z + c};
	return result;
}

float3 operator+(float c, const int3& a)
{
	float3 result = { a.x + c, a.y + c, a.z + c };
	return result;
}

int3 operator+(const int3& a, int32_t c)
{
	int3 result = { a.x + c, a.y + c, a.z + c};
	return result;
}

int3 operator+(int32_t c, const int3& a)
{
	int3 result = { a.x + c, a.y + c, a.z + c };
	return result;
}

int3 operator-(const int3& a, const int3& b)
{
	int3 result = { a.x - b.x, a.y - b.y, a.z - b.z };
	return result;
}

float3 operator-(const int3& a, float c)
{
	float3 result = { a.x - c, a.y - c, a.z - c };
	return result;
}

float3 operator-(float c, const int3& a)
{
	float3 result = { c - a.x, c - a.y, c - a.z };
	return result;
}

int3 operator-(const int3& a, int32_t c)
{
	int3 result = { a.x - c, a.y - c, a.z - c };
	return result;
}

int3 operator-(int32_t c, const int3& a)
{
	int3 result = { c - a.x, c - a.y, c - a.z };
	return result;
}

int3 operator*(const int3& a, const int3& b)
{
	int3 result = { a.x * b.x, a.y * b.y, a.z * b.z };
	return result;
}

float3 operator*(const int3& a, float c)
{
	float3 result = { a.x * c, a.y * c, a.z * c };
	return result;
}

float3 operator*(float c, const int3& a)
{
	float3 result = { a.x * c, a.y * c, a.z * c };
	return result;
}

int3 operator*(const int3& a, int32_t c)
{
	int3 result = { a.x * c, a.y * c, a.z * c };
	return result;
}

int3 operator*(int32_t c, const int3& a)
{
	int3 result = { a.x * c, a.y * c, a.z * c };
	return result;
}

float3 operator/(const int3& a, const int3& b)
{
	float3 result = float3(a) / float3(b);
	return result;
}

float3 operator/(const int3& a, float c)
{
	float3 result = { a.x / c, a.y / c, a.z / c };
	return result;
}

float3 operator/(float c, const int3& a)
{
	float3 result = { c / a.x, c / a.y, c / a.z };
	return result;
}

std::ostream& operator<<(std::ostream& os, int3 const& v)
{
	os << v.x << ' ' << v.y << ' ' << v.z;
	return os;
};

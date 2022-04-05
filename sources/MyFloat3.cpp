#include "MyFloat3.h"
#include "MyFloat4.h"
#include "MyDouble3.h"
#include "MyInt3.h"

float3::float3()
{
	this->x = 0.0f;
	this->y = 0.0f;
	this->z = 0.0f;
};

float3::float3(float x)
{
	this->x = x;
	this->y = x;
	this->z = x;
};

float3::float3(float x, float y, float z)
{
	this->x = x;
	this->y = y;
	this->z = z;
};

float3::float3(const float3& d)
{
	this->x = d.x;
	this->y = d.y;
	this->z = d.z;
};

float3::float3(double3 d)
{
	this->x = d.x;
	this->y = d.y;
	this->z = d.z;
}


float3::float3(int3 d)
{
	this->x = d.x;
	this->y = d.y;
	this->z = d.z;
}

float3::float3(float4 d)
{
	this->x = d.x;
	this->y = d.y;
	this->z = d.z;
}

float float3::operator[](int i) {
	if (i == 0)
		return x;
	else if(i == 1)
		return y;
	else
		return z;
}

float float3::operator[](int i) const{
	if (i == 0)
		return x;
	else if(i == 1)
		return y;
	else
		return z;
}

void float3::operator=(const float3& a) {
	x = a.x;
	y = a.y;
	z = a.z;
}

bool float3::operator==(const float3& a)
{
	return x == a.x && y == a.y && z == a.z;
}

bool float3::operator!=(const float3& a)
{
	return !(*this == a);
}

float float3::length() const
{
	return sqrtf(dot(*this, *this));
}

float float3::length(const float3& a)
{
	return sqrtf(dot(a, a));
}

float3 float3::normalize(const float3& a)
{
	return a / a.length();
}

float3 float3::normalize()
{
	return *this / this->length();
}

float float3::dot(const float3& a, const float3& b)
{
	return a.x * b.x + a.y * b.y + a.z * b.z;
}

float float3::dot(const float3& b) const
{
	return x * b.x + y * b.y + z * b.z;
}

float3 float3::cross(const float3& a, const float3& b)
{
	float3 result(a.y*b.z - a.z*b.y, a.z*b.x - a.x*b.z, a.x*b.y-a.y*b.x);
	return result;
}

float3 float3::abs(const float3& a)
{
	float3 result(std::abs(a.x), std::abs(a.y), std::abs(a.z));
	return result;
}

float float3::min()
{
	return std::fminf(x, std::fminf(y, z));
}

float3 float3::min(const float3& a, const float3& b)
{
	float3 result(std::fminf(a.x, b.x), std::fminf(a.y, b.y), std::fminf(a.z, b.z));
	return result;
}

float float3::max()
{
	return std::fmaxf(x, std::fmaxf(y, z));
}

float3 float3::max(const float3& a, const float3& b)
{
	float3 result(std::fmaxf(a.x, b.x), std::fmaxf(a.y, b.y), std::fmaxf(a.z, b.z));
	return result;
}

float3 operator+(const float3& a, const float3& b)
{
	float3 result = { a.x + b.x, a.y + b.y, a.z + b.z };
	return result;
}

float3 operator+(const float3& a, float c)
{
	float3 result = { a.x + c, a.y + c, a.z + c};
	return result;
}

float3 operator+(float c, const float3& a)
{
	float3 result = { a.x + c, a.y + c, a.z + c };
	return result;
}

float3 operator-(const float3& a, const float3& b)
{
	float3 result = { a.x - b.x, a.y - b.y, a.z - b.z };
	return result;
}

float3 operator-(const float3& a, float c)
{
	float3 result = { a.x - c, a.y - c, a.z - c };
	return result;
}

float3 operator-(float c, const float3& a)
{
	float3 result = { c - a.x, c - a.y, c - a.z };
	return result;
}

float3 operator*(const float3& a, const float3& b)
{
	float3 result = { a.x * b.x, a.y * b.y, a.z * b.z };
	return result;
}

float3 operator*(const float3& a, float c)
{
	float3 result = { a.x * c, a.y * c, a.z * c };
	return result;
}

float3 operator*(float c, const float3& a)
{
	float3 result = { a.x * c, a.y * c, a.z * c };
	return result;
}

float3 operator/(const float3& a, const float3& b)
{
	float3 result = { a.x / b.x, a.y / b.y, a.z / b.z };
	return result;
}

float3 operator/(const float3& a, float c)
{
	float3 result = { a.x / c, a.y / c, a.z / c };
	return result;
}

float3 operator/(float c, const float3& a)
{
	float3 result = { c / a.x, c / a.y, c / a.z };
	return result;
}

std::ostream& operator<<(std::ostream& os, float3 const& v)
{
	os << v.x << ' ' << v.y << ' ' << v.z;
	return os;
};

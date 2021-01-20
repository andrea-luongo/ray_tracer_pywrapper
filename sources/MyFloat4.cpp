#include "MyFloat3.h"
#include "MyFloat4.h"
#include "MyDouble3.h"

float4::float4()
{
	this->x = 0.0f;
	this->y = 0.0f;
	this->z = 0.0f;
	this->w = 0.0f;
};

float4::float4(float x)
{
	this->x = x;
	this->y = x;
	this->z = x;
	this->w = x;
};

float4::float4(float x, float y, float z, float w)
{
	this->x = x;
	this->y = y;
	this->z = z;
	this->w = w;
};

float4::float4(const float4& d)
{
	this->x = d.x;
	this->y = d.y;
	this->z = d.z;
	this->w = d.w;
};
float4::float4(const float3& d)
{
	this->x = d.x;
	this->y = d.y;
	this->z = d.z;
	this->w = 1.0f;
};

float4::float4(const double3& d)
{
	this->x = d.x;
	this->y = d.y;
	this->z = d.z;
	this->w = 1.0f;
}

float float4::operator[](int i) {
	if (i == 0)
		return x;
	else if (i == 1)
		return y;
	else if (i == 2)
		return z;
	else
		return w;
}

float float4::operator[](int i) const{
	if (i == 0)
		return x;
	else if (i == 1)
		return y;
	else if (i == 2)
		return z;
	else
		return w;
}

void float4::operator=(const float4& a) {
	x = a.x;
	y = a.y;
	z = a.z;
	w = a.w;
}

float float4::length() const
{
	return sqrtf(dot(*this, *this));
}

float float4::length(const float4& a)
{
	return sqrtf(dot(a, a));
}

float4 float4::normalize(const float4& a)
{
	return a / a.length();
}

float float4::dot(const float4& a, const float4& b)
{
	return a.x * b.x + a.y * b.y + a.z * b.z + a.w * b.w;
}

float4 float4::abs(const float4& a)
{
	float4 result(std::abs(a.x), std::abs(a.y), std::abs(a.z), std::abs(a.w));
	return result;
}

float float4::min()
{
	return std::fminf(x, std::fminf(y, std::fminf(z, w)));
}

float4 float4::min(const float4& a, const float4& b)
{
	float4 result(std::fminf(a.x, b.x), std::fminf(a.y, b.y), std::fminf(a.z, b.z), std::fminf(a.w, b.w));
	return result;
}

float float4::max()
{
	return std::fmaxf(x, std::fmaxf(y, std::fmaxf(z, w)));
}

float4 float4::max(const float4& a, const float4& b)
{
	float4 result(std::fmaxf(a.x, b.x), std::fmaxf(a.y, b.y), std::fmaxf(a.z, b.z), std::fmaxf(a.w, b.w));
	return result;
}

float4 operator+(const float4& a, const float4& b)
{
	float4 result = { a.x + b.x, a.y + b.y, a.z + b.z, a.w + b.w};
	return result;
}

float4 operator+(const float4& a, float c)
{
	float4 result = { a.x + c, a.y + c, a.z + c, a.w + c};
	return result;
}

float4 operator+(float c, const float4& a)
{
	float4 result = { a.x + c, a.y + c, a.z + c, a.w + c};
	return result;
}

float4 operator-(const float4& a, const float4& b)
{
	float4 result = { a.x - b.x, a.y - b.y, a.z - b.z, a.w - b.w };
	return result;
}

float4 operator-(const float4& a, float c)
{
	float4 result = { a.x - c, a.y - c, a.z - c, a.w - c };
	return result;
}

float4 operator-(float c, const float4& a)
{
	float4 result = { c - a.x, c - a.y, c - a.z, c - a.w};
	return result;
}

float4 operator*(const float4& a, const float4& b)
{
	float4 result = { a.x * b.x, a.y * b.y, a.z * b.z, a.w * b.w };
	return result;
}

float4 operator*(const float4& a, float c)
{
	float4 result = { a.x * c, a.y * c, a.z * c, a.w * c };
	return result;
}

float4 operator*(float c, const float4& a)
{
	float4 result = { a.x * c, a.y * c, a.z * c, a.w * c };
	return result;
}

float4 operator/(const float4& a, const float4& b)
{
	float4 result = { a.x / b.x, a.y / b.y, a.z / b.z, a.w / b.w };
	return result;
}

float4 operator/(const float4& a, float c)
{
	float4 result = { a.x / c, a.y / c, a.z / c, a.w / c };
	return result;
}

float4 operator/(float c, const float4& a)
{
	float4 result = { c / a.x, c / a.y, c / a.z, c / a.w };
	return result;
}

std::ostream& operator<<(std::ostream& os, float4 const& v)
{
	os << v.x << ' ' << v.y << ' ' << v.z << ' ' << v.w;
	return os;
};

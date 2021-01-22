#include "MyDouble3.h"
#include "MyFloat3.h"

double3::double3()
{
	this->x = 0.0;
	this->y = 0.0;
	this->z = 0.0;
};

double3::double3(double x)
{
	this->x = x;
	this->y = x;
	this->z = x;
};

double3::double3(double x, double y, double z)
{
	this->x = x;
	this->y = y;
	this->z = z;
};

double3::double3(const double3& d)
{
	this->x = d.x;
	this->y = d.y;
	this->z = d.z;
};

double3::double3(float3 d)
{
	this->x = d.x;
	this->y = d.y;
	this->z = d.z;
}

double double3::operator[](int i) {
	if (i == 0)
		return x;
	else if (i == 1)
		return y;
	else
		return z;
}

double double3::operator[](int i) const {
	if (i == 0)
		return x;
	else if (i == 1)
		return y;
	else
		return z;
}

void double3::operator=(const double3& a) {
	x = a.x;
	y = a.y;
	z = a.z;
}

double double3::length() const
{
	return sqrt(dot(*this, *this));
}

double double3::length(const double3& a)
{
	return sqrt(dot(a, a));
}

double3 double3::normalize(const double3& a)
{
	return a / a.length();
}

double double3::dot(const double3& a, const double3& b)
{
	return a.x * b.x + a.y * b.y + a.z * b.z;
}

float double3::dot(const double3& b) const
{
	return x * b.x + y * b.y + z * b.z;
}

double3 double3::cross(const double3& a, const double3& b)
{
	double3 result(a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x);
	return result;
}

double3 double3::abs(const double3& a)
{
	double3 result(std::abs(a.x), std::abs(a.y), std::abs(a.z));
	return result;
}

double double3::min()
{
	return std::fmin(x, std::fmin(y, z));
}

double3 double3::min(const double3& a, const double3& b)
{
	double3 result(std::fmin(a.x, b.x), std::fmin(a.y, b.y), std::fmin(a.z, b.z));
	return result;
}

double double3::max()
{
	return std::fmax(x, std::fmax(y, z));
}

double3 double3::max(const double3& a, const double3& b)
{
	double3 result(std::fmax(a.x, b.x), std::fmax(a.y, b.y), std::fmax(a.z, b.z));
	return result;
}

double3 operator+(const double3& a, const double3& b)
{
	double3 result = { a.x + b.x, a.y + b.y, a.z + b.z };
	return result;
}

double3 operator+(const double3& a, double c)
{
	double3 result = { a.x + c, a.y + c, a.z + c };
	return result;
}

double3 operator+(double c, const double3& a)
{
	double3 result = { a.x + c, a.y + c, a.z + c };
	return result;
}

double3 operator-(const double3& a, const double3& b)
{
	double3 result = { a.x - b.x, a.y - b.y, a.z - b.z };
	return result;
}

double3 operator-(const double3& a, double c)
{
	double3 result = { a.x - c, a.y - c, a.z - c };
	return result;
}

double3 operator-(double c, const double3& a)
{
	double3 result = { c - a.x, c - a.y, c - a.z };
	return result;
}

double3 operator*(const double3& a, const double3& b)
{
	double3 result = { a.x * b.x, a.y * b.y, a.z * b.z };
	return result;
}

double3 operator*(const double3& a, double c)
{
	double3 result = { a.x * c, a.y * c, a.z * c };
	return result;
}

double3 operator*(double c, const double3& a)
{
	double3 result = { a.x * c, a.y * c, a.z * c };
	return result;
}

double3 operator/(const double3& a, const double3& b)
{
	double3 result = { a.x / b.x, a.y / b.y, a.z / b.z };
	return result;
}

double3 operator/(const double3& a, double c)
{
	double3 result = { a.x / c, a.y / c, a.z / c };
	return result;
}

double3 operator/(double c, const double3& a)
{
	double3 result = { c / a.x, c / a.y, c / a.z };
	return result;
}

std::ostream& operator<<(std::ostream& os, double3 const& v)
{
	os << v.x << ' ' << v.y << ' ' << v.z;
	return os;
};

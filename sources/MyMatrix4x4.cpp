#include "MyFloat3.h"
#include "MyFloat4.h"
#include "MyDouble3.h"
#include "MyMatrix4x4.h"

Matrix4x4::Matrix4x4()
{
	m_elements[0] = 1;
	m_elements[5] = 1;
	m_elements[10] = 1;
	m_elements[15] = 1;
};

Matrix4x4::Matrix4x4(const std::array<float, 16>& elements)
{
	for (int i = 0; i < 16; i++)
	{
		m_elements[i] = elements[i];
	}
};

Matrix4x4::Matrix4x4(const float4& r_0, const float4& r_1, const float4& r_2, const float4& r_3)
{
	for (int i = 0; i < 4; i++)
	{
		m_elements[i] = r_0[i];
		m_elements[i + 4] = r_1[i];
		m_elements[i + 8] = r_2[i];
		m_elements[i + 12] = r_3[i];
	}
};

Matrix4x4::Matrix4x4(const Matrix4x4& m)
{
	for (int i = 0; i < 16; i++)
	{
		m_elements[i] = m[i];
	}
};

float4 Matrix4x4::GetRow(int r) const
{
	float4 result(m_elements[4* r], m_elements[4 * r + 1], m_elements[4 * r + 2], m_elements[4 * r + 3]);
	return result;
};

float4 Matrix4x4::GetColumn(int c) const
{
	float4 result(m_elements[c], m_elements[c + 4], m_elements[c + 8], m_elements[c + 12]);
	return result;
};

//Matrix4x4 Matrix4x4::Transpose() const
//{
//	//float4 result(m_elements[c], m_elements[c + 4], m_elements[c + 8], m_elements[c + 12]);
//	//return result;
//};

float Matrix4x4::operator[](int i) {
	return m_elements[i];
}

float Matrix4x4::operator[](int i) const {
	return m_elements[i];
}

void Matrix4x4::operator=(const Matrix4x4& m) 
{
	for (int i = 0; i < 16; i++)
	{
		m_elements[i] = m[i];
	}
}

Matrix4x4 operator+(const Matrix4x4& a, const Matrix4x4& b)
{
	std::array<float, 16> result;
	for (int i = 0; i < 16; i++)
	{
		result[i] = a[i] + b[i];
	}
	return Matrix4x4(result);
}

Matrix4x4 operator+(const Matrix4x4& a, float c)
{
	std::array<float, 16> result;
	for (int i = 0; i < 16; i++)
	{
		result[i] = a[i] + c;
	}
	return Matrix4x4(result);
}

Matrix4x4 operator+(float c, const Matrix4x4& a)
{
	return a + c;
}

Matrix4x4 operator-(const Matrix4x4& a, const Matrix4x4& b)
{
	std::array<float, 16> result;
	for (int i = 0; i < 16; i++)
	{
		result[i] = a[i] - b[i];
	}
	return Matrix4x4(result);
}

Matrix4x4 operator-(const Matrix4x4& a, float c)
{
	std::array<float, 16> result;
	for (int i = 0; i < 16; i++)
	{
		result[i] = a[i] - c;
	}
	return Matrix4x4(result);
}

Matrix4x4 operator-(float c, const Matrix4x4& a)
{
	std::array<float, 16> result;
	for (int i = 0; i < 16; i++)
	{
		result[i] = c - a[i];
	}
	return Matrix4x4(result);
}

Matrix4x4 operator*(const Matrix4x4& a, const Matrix4x4& b)
{
	float4 a_row_0 = a.GetRow(0);
	float4 a_row_1 = a.GetRow(1);
	float4 a_row_2 = a.GetRow(2);
	float4 a_row_3 = a.GetRow(3);
	float4 b_col_0 = b.GetColumn(0);
	float4 b_col_1 = b.GetColumn(1);
	float4 b_col_2 = b.GetColumn(2);
	float4 b_col_3 = b.GetColumn(3);
	float4 row_0(a_row_0.dot(b_col_0), a_row_0.dot(b_col_1), a_row_0.dot(b_col_2), a_row_0.dot(b_col_3));
	float4 row_1(a_row_1.dot(b_col_0), a_row_1.dot(b_col_1), a_row_1.dot(b_col_2), a_row_1.dot(b_col_3));
	float4 row_2(a_row_2.dot(b_col_0), a_row_2.dot(b_col_1), a_row_2.dot(b_col_2), a_row_2.dot(b_col_3));
	float4 row_3(a_row_3.dot(b_col_0), a_row_3.dot(b_col_1), a_row_3.dot(b_col_2), a_row_3.dot(b_col_3));
	return Matrix4x4(row_0, row_1, row_2, row_3);
}

Matrix4x4 operator*(const Matrix4x4& a, float c)
{
	std::array<float, 16> result;
	for (int i = 0; i < 16; i++)
	{
		result[i] = a[i] * c;
	}
	return Matrix4x4(result);
}

Matrix4x4 operator*(float c, const Matrix4x4& a)
{
	return a * c;
}

float4 operator*(const Matrix4x4& a, const float4& b)
{
	float4 result(a.GetRow(0).dot(b), a.GetRow(1).dot(b), a.GetRow(2).dot(b), a.GetRow(3).dot(b));
	return result;
}

float4 operator*(const float4& a, const Matrix4x4& b)
{
	float4 result(a.dot(b.GetColumn(0)), a.dot(b.GetColumn(1)), a.dot(b.GetColumn(2)), a.dot(b.GetColumn(3)));
	return result;
}

Matrix4x4 operator/(const Matrix4x4& a, float c)
{
	std::array<float, 16> result;
	for (int i = 0; i < 16; i++)
	{
		result[i] = a[i] / c;
	}
	return Matrix4x4(result);
}

std::ostream& operator<<(std::ostream& os, Matrix4x4 const& v)
{
	os << v[0] << ' ' << v[1] << ' ' << v[2] << ' ' << v[3] << '\n';
	os << v[4] << ' ' << v[5] << ' ' << v[6] << ' ' << v[7] << '\n';
	os << v[8] << ' ' << v[9] << ' ' << v[10] << ' ' << v[11] << '\n';
	os << v[12] << ' ' << v[13] << ' ' << v[14] << ' ' << v[15] << '\n';
	return os;
};

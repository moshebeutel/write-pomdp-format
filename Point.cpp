#include "Point.h"


Point::Point(size_t x, size_t y, double std)
: m_x(x)
, m_y(y)
, m_std(std)
{
}

size_t Point::GetX() const
{
	return m_x;
}

size_t Point::GetY() const
{
	return m_y;
}

size_t Point::GetIdx(size_t gridSize) const
{
	return m_y * gridSize + m_x;
}

double Point::GetStd() const
{
	return m_std;
}


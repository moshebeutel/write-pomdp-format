#pragma once
class Point
{
public:
	Point(size_t x, size_t y, double m_std = 0.0);
	~Point() = default;

	size_t GetX() const;
	size_t GetY() const;
	size_t GetIdx(size_t gridSize) const;
	double GetStd() const;
private:
	size_t m_x;
	size_t m_y;
	double m_std;	//standard deviation for the location of the object
};


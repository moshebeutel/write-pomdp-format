#pragma once

#include "Point.h"

//a base class for objects in grid for pomdp project. 
//can be used as shelter
class ObjInGrid
{
public:
	explicit ObjInGrid(Point& location);
	virtual ~ObjInGrid() = default;
	ObjInGrid(const ObjInGrid&) = default;
	//ObjInGrid(ObjInGrid&&) = default;

	const Point &GetLocation() const;

private:
	Point m_location;
};




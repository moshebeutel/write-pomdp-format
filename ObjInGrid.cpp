#include "ObjInGrid.h"

ObjInGrid::ObjInGrid(Point& location)
: m_location(location)
{
}

const Point & ObjInGrid::GetLocation() const
{
	return m_location;
}

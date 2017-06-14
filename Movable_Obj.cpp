#include "Movable_Obj.h"


Movable_Obj::Movable_Obj(Point& location, Move_Properties& movement)
	: ObjInGrid(location)
	, m_movement(movement)
{
}


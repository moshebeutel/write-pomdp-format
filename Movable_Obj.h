#pragma once
#include "ObjInGrid.h"
#include "Move_Properties.h"
#include "Point.h"

//class of movable objects in grid for pomdp writer. can be use as non-involved objects
class Movable_Obj :
	public ObjInGrid
{
public:
	explicit Movable_Obj(Point& location, Move_Properties& movement);
	virtual ~Movable_Obj() = default;
	Movable_Obj(const Movable_Obj &) = default;

	const Move_Properties& GetMovement() const { return m_movement; }

private:
	Move_Properties m_movement;
};


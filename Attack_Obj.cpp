#include "Attack_Obj.h"


Attack_Obj::Attack_Obj(Point& location, Move_Properties& movement, size_t attackRange, double pHit)
: Movable_Obj(location, movement)
, m_attack(attackRange, pHit)
{
}


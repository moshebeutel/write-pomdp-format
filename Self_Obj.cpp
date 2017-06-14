#include "Self_Obj.h"


Self_Obj::Self_Obj(Point& location, Move_Properties& movement, size_t attackRange, double pHit, size_t rangeObs, double pObservation)
: Attack_Obj(location, movement, attackRange, pHit)
, m_rangeObs(rangeObs)
, m_pObservation(pObservation)
{
}
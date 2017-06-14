#pragma once
#include "Attack_Obj.h"
#include "Move_Properties.h"
#include "Point.h"

class Self_Obj :
	public Attack_Obj
{
public:
	explicit Self_Obj(Point& location, Move_Properties& movement, size_t attackRange, double pHit, size_t rangeObs, double pObservation);
	virtual ~Self_Obj() = default;
	Self_Obj(const Self_Obj&) = default;

	double GetPObs() const { return m_pObservation; }
	size_t GetRange() const { return m_rangeObs; }
private:
	size_t m_rangeObs;
	double m_pObservation;
};
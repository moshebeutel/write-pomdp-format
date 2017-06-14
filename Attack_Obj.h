#pragma once
#include "Movable_Obj.h"
#include "Move_Properties.h"
#include "Point.h"

class Attack_Obj :
	public Movable_Obj
{
public:
	explicit Attack_Obj(Point& location, Move_Properties& movement, size_t attackRange, double pHit);
	virtual ~Attack_Obj() = default;
	Attack_Obj(const Attack_Obj&) = default;

	size_t GetRange() { return m_attack.m_range; }
	double GetPHit() { return m_attack.m_pHit; }

private:
	class Attack
	{
	public:
		Attack(size_t range, double pHit) : m_range(range), m_pHit(pHit) {}
		size_t m_range;
		double m_pHit;
	};
	Attack m_attack;
};


#include "Move_Properties.h"


Move_Properties::Move_Properties(double stay, double towardTarget)
	: m_pEqualShare( (1 - stay - towardTarget) / s_numDirections )
	, m_pStay(stay)
	, m_pTowardTarget(towardTarget)
{
}


#pragma once

//properties of movement for pomdp writer
class Move_Properties
{
public:
	explicit Move_Properties(double stay, double towardTarget = 0.0);
	~Move_Properties() = default;

	double GetEqual() const {return m_pEqualShare;}
	double GetStay() const { return m_pStay; }
	double GetToward() const { return m_pTowardTarget; }

private:
	double m_pEqualShare;
	double m_pStay;
	double m_pTowardTarget;

	static const int s_numDirections = 4;
};


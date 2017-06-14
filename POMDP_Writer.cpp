#include "POMDP_Writer.h"
#include <iostream>
#include <string>
#include <random>
#include <math.h>
#include <memory.h>
#include <sstream>
#include <iomanip>
#include <algorithm>

static const std::string s_WinState = "Win";
static const std::string s_LossState = "Loss";

// value in move states for non-valid move
static const int NVALID_MOVE = -1;
// value in stateVec for dead enemy
static const int DEAD_ENEMY = -2;
// idx of enemy in the stateVec
static const int ENEMY_IDX = 1;

// idx for win state
static size_t s_idxTarget;
// to convey probability between calculations
static double s_pLeftProbability = 1.0;

POMDP_Writer::state_t POMDP_Writer::s_junkState{ NVALID_MOVE };

inline int Abs(int x)
{
	return x * (x >= 0) - x * (x < 0);
}

// translate to string with higher precision
inline std::string to_string_precision(double d, int n = 10)
{
	std::ostringstream out;
	out << std::setprecision(n) << d;
	return out.str();
}

static double Sum(const double *arr, size_t size)
{
	double p = 0;
	for (size_t i = 0; i < size; ++i)
	{
		p += arr[i];
	}

	return p;
}

POMDP_Writer::POMDP_Writer(size_t gridSize, Self_Obj& self, Attack_Obj& enemy, double discount)
: m_gridSize(gridSize)
, m_self(self)
, m_enemy(enemy)
, m_NInvVector()
, m_shelter()
, m_discount(discount)
{
}

void POMDP_Writer::AddObj(Movable_Obj& obj)
{
	m_NInvVector.emplace_back(obj);
}

void POMDP_Writer::AddObj(ObjInGrid& obj)
{
	m_shelter.emplace_back(obj);
}

void POMDP_Writer::SaveInFormat(FILE *fptr, size_t idxTarget)
{
		std::string buffer("");
		s_idxTarget = idxTarget;

		//add comments and init lines(state observations etc.) to file
		CommentsAndInitLines(buffer, fptr);
		
		// add position with and without moving of the robot
		PositionStates(buffer, fptr);

		// add hits calculation
		AttackAction(buffer, fptr);

		// add observations and rewards
		ObservationsAndRewards(buffer, fptr);
}

void POMDP_Writer::CommentsAndInitLines(std::string buffer, FILE *fptr)
{
	// add comments
	buffer += "# pomdp file:\n";
	buffer += "# grid size: " + std::to_string(m_gridSize) + "  target idx: " + std::to_string(s_idxTarget);
	buffer += "\n# self initial location: " + std::to_string(m_self.GetLocation().GetIdx(m_gridSize)) + " std = " + std::to_string(m_self.GetLocation().GetStd());
	buffer += "\n# enemy initial location: " + std::to_string(m_enemy.GetLocation().GetIdx(m_gridSize)) + " std = " + std::to_string(m_enemy.GetLocation().GetStd());
	for (auto v : m_NInvVector)
	{
		buffer += "\n# non- involved initial location: " + std::to_string(v.GetLocation().GetIdx(m_gridSize)) + " std = " + std::to_string(v.GetLocation().GetStd());
	}

	for (auto v : m_shelter)
	{
		buffer += "\n# shelter location: " + std::to_string(v.GetLocation().GetIdx(m_gridSize));
	}

	// add init lines
	buffer += "\n\ndiscount: " + std::to_string(m_discount);
	buffer += "\nvalues: reward\nstates: ";

	// add states names
	std::string type = "s";
	CalcStatesAndObs(2 + m_NInvVector.size(), m_gridSize, type, buffer);
	buffer += s_WinState + " " + s_LossState + "\n";
	buffer += "actions: Stay North South East West Shoot_North Shoot_South Shoot_West Shoot_East\n";

	// add observations names
	buffer += "observations: ";
	type = "o";
	CalcStatesAndObs(2 + m_NInvVector.size(), m_gridSize, type, buffer);
	buffer += "\n\nstart: \n";

	// add start states probability
	CalcStartState(buffer);
	//save to file
	auto err = fputs(buffer.c_str(), fptr);
	if (err < 0) { std::cerr << "Error Writing to file\n"; exit(1); }
}

void POMDP_Writer::PositionStates(std::string buffer, FILE * fptr)
{
	buffer += "\n\nT: * : * : * 0.0\n\n";
	// add move positions when the robot is static
	NoMovePosition(buffer);
	// add move positions when robot is moving
	MovePosition(buffer);
	// save to file
	auto err = fputs(buffer.c_str(), fptr);
	if (err < 0) { std::cerr << "Error Writing to file\n"; exit(1);}
}

void POMDP_Writer::AttackAction(std::string buffer, FILE * fptr)
{
	// calculate states and probability to hit
	CalcHits(buffer);
	// save to file
	auto err = fputs(buffer.c_str(), fptr);
	if (err < 0) { std::cerr << "Error Writing to file\n"; exit(1); }
}

void POMDP_Writer::ObservationsAndRewards(std::string buffer, FILE * fptr)
{
	// calculate observations
	CalcObs(buffer);
	// add rewards
	buffer += "\n\nR: * : * : * : * 0.0\nR: * : "
		+ s_WinState + " : * : * 100\nR: * : "
		+ s_LossState + " : * : * -100\n";
	// save to file 
	auto err = fputs(buffer.c_str(), fptr);
	if (err < 0) { std::cerr << "Error Writing to file\n"; return; }
}

void POMDP_Writer::CalcStatesAndObs(size_t numObjects, size_t gridSize, std::string& type, std::string& buffer)
{
	state_t stateVec(numObjects);

	CalcS_ORec(stateVec, 0, gridSize, type, buffer);

	// add states or observations for dead enemy
	stateVec[1] = DEAD_ENEMY;
	for (size_t i = 0; i < gridSize * gridSize; ++i)
	{
		stateVec[0] = i;
		CalcS_ORec(stateVec, 2, gridSize, type, buffer);
	}
}

void POMDP_Writer::CalcS_ORec(state_t& stateVec, size_t currIdx, size_t gridSize, std::string & type, std::string & buffer)
{
	// stopping condition when finish running on all objects
	if (stateVec.size() == currIdx)
	{
		// insert state to buffer
		buffer += type;
		for (size_t i = 0; i < stateVec.size() - 1; ++i)
		{
			if (stateVec[i] == DEAD_ENEMY)
			{
				buffer += "Dx";
			}
			else
			{
				buffer += std::to_string(stateVec[i]) + "x";
			}
		}
		buffer += std::to_string(stateVec[stateVec.size() - 1]) + " ";
	}
	else
	{
		// run on all possible states (without repetitions)
		for (size_t i = 0; i < gridSize * gridSize; ++i)
		{
			stateVec[currIdx] = i;
			if (NoRepetition(stateVec, currIdx))
			{
				CalcS_ORec(stateVec, currIdx + 1, gridSize, type, buffer);
			}
		}
	}
}


bool POMDP_Writer::NoRepetition(state_t& stateVec, size_t currIdx)
{
	for (size_t i = 0; i < currIdx; ++i)
	{
		if (stateVec[i] == stateVec[currIdx])
		{
			return false;
		}
	}
	return true;
}

void POMDP_Writer::NoRepetitionCheckAndCorrect(state_t& stateVec, int *moveStates, size_t *arrOfIdx)
{
	//if any move state equal to the robot location change location to previous location
	for (size_t i = 1; i < stateVec.size(); ++i)
	{
		if (stateVec[i] == stateVec[0])
		{
			stateVec[i] = moveStates[(i - 1) * 5] ;
		}
	}

	//if one of the stateVec equal to another return the possible state to the previous location
	for (size_t i = 1; i < stateVec.size(); ++i)
	{
		for (size_t j = 1; j < stateVec.size(); ++j)
		{
			if (stateVec[i] == stateVec[j] && i != j)
			{
				if (arrOfIdx[i - 1] % 5 == 0)
				{
					stateVec[j] = moveStates[(j - 1) * 5];
				}
				else
				{
					stateVec[i] = moveStates[(i - 1) * 5];
				}
			}
		}
	}
}

double POMDP_Writer::DividePRepetitions(state_t& stateVec, double * pMat, int currIdx, size_t numStates)
{
	// run on all states until current and add the p from the non-valid idx (i.e. current location cannot be equal to previous objects location)
	double pToDivide = 0;
	for (int i = 0; i < currIdx; ++i)
	{
		pToDivide += pMat[stateVec[i]];
	}

	// add to each slot equal share of the non-valid probability
	pToDivide /= (numStates - currIdx);
	for (size_t i = 0; i < numStates; ++i)
	{
		pMat[i] += pToDivide;
	}

	return pToDivide;
}

void POMDP_Writer::ReturnPRepetition(double *pMat, size_t numStates, double pRep)
{
	// return the p that added earlier from DividePRepetitions()
	for (size_t i = 0; i < numStates; ++i)
	{
		pMat[i] -= pRep;
	}
}

void POMDP_Writer::CalcStartState(std::string& buffer)
{
	size_t statesForObj = m_gridSize * m_gridSize;
	double * pMat(new double[statesForObj * (2 + m_NInvVector.size()) + 1]);

	// calculate individual probability matrix for each object
	CalcSinglePosition(&m_self, m_gridSize, pMat);
	CalcSinglePosition(&m_enemy, m_gridSize, pMat + statesForObj);
	for (size_t i = 0 ; i < m_NInvVector.size() ; ++i)
	{
		CalcSinglePosition(&m_NInvVector[i], m_gridSize, pMat + (i + 2) * statesForObj);
	}

	state_t stateVec(2 + m_NInvVector.size());
	// calculate probability for each state
	CalcStartStateRec(pMat, stateVec, 0, buffer);

	// add the p to start in states where the enemy dead and in lose/win states
	size_t numNonInitStates = statesForObj;
	for (size_t i = 0; i < m_NInvVector.size(); ++i)
	{
		numNonInitStates *= statesForObj;
	}
	for (size_t i = 0; i < numNonInitStates + 2; ++i)
	{
		buffer += "0 ";
	}
	delete[] pMat;
}

void POMDP_Writer::CalcStartStateRec(double * pMat, state_t& stateVec, size_t currIdx, std::string & buffer)
{
	// stopping condition when finish running on all objects
	if (stateVec.size() == currIdx)
	{
		// the probability of the state is the multiplication of each object location probability
		double p = 1;
		for (size_t i = 0; i < stateVec.size(); ++i)
		{
			p *= pMat[stateVec[i] + i * m_gridSize * m_gridSize];
		}
		buffer += to_string_precision(p) + " ";
	}
	else
	{
		// to avoid repetition in state (s0x0) divide the probability of repeated location in all other locations
		double pRep = DividePRepetitions(stateVec, pMat + currIdx * m_gridSize * m_gridSize, currIdx, m_gridSize * m_gridSize);
		for (size_t i = 0; i < m_gridSize * m_gridSize; ++i)
		{
			stateVec[currIdx] = i;
			if (NoRepetition(stateVec, currIdx))
			{
				CalcStartStateRec(pMat, stateVec, currIdx + 1, buffer);
			}
		}
		// return the pMat to previous values
		ReturnPRepetition(pMat + currIdx * m_gridSize * m_gridSize, m_gridSize * m_gridSize, pRep);
	}
}

void POMDP_Writer::CalcSinglePosition(ObjInGrid *obj, size_t gridSize, double *pMat)
{
	// if std = 0 calculate pmat in different way
	if (0 == obj->GetLocation().GetStd())
	{
		CalcSinglePositionNoStd(obj, gridSize, pMat);
		return;
	}

	double * P_x(new double[gridSize]);
	double * P_y(new double[gridSize]);
	double leftx = 1.0;
	double lefty = 1.0;
	double cdf;

	// compute the cdf(x + 0.5) - cdf( (x - 1) + 0.5 ) --> the p(x)
	// do the same thing for y
	for (size_t i = 0 ; i < gridSize; ++i)
	{
		cdf = CumulativeDistFunc(i + 0.5, obj->GetLocation().GetX(), obj->GetLocation().GetStd());
		P_x[i]  = leftx - cdf;
		leftx = cdf;

		cdf = CumulativeDistFunc(i + 0.5, obj->GetLocation().GetY(), obj->GetLocation().GetStd());
		P_y[i] = lefty - cdf;
		lefty = cdf;
	}
	// insert the edges of the distribution to the edges
	P_x[gridSize - 1] += leftx;
	P_y[gridSize - 1] += lefty;

	// calculate pmat in vectoric multiplication of the probability vectors
	for (size_t y = 0; y < gridSize; ++y)
	{
		for (size_t x = 0; x < gridSize; ++x)
		{
			pMat[y * (gridSize) + x] = P_x[x] * P_y[y];
		}
	}

	delete[] P_x;
	delete[] P_y;
}

void POMDP_Writer::CalcSinglePositionNoStd(ObjInGrid * obj, size_t gridSize, double * pMat)
{
	// zero all matrix except pmat[location] = 1
	for (size_t i = 0; i < gridSize * gridSize; ++i)
	{
			pMat[i] = 0;
	}
	pMat[obj->GetLocation().GetY() * (gridSize - 1) + obj->GetLocation().GetX()] = 1;
}

double POMDP_Writer::CumulativeDistFunc(double x, int mean, double std)
{
	// return the cdf of val x from normal distribution with mean and std
	return 0.5 * erfc((x - mean) / (std * sqrt(2)) );
}

void POMDP_Writer::NoMovePosition(std::string & buffer)
{
	state_t stateVec(2 + m_NInvVector.size());
	std::string action = "*";
	CalcPositionRec(stateVec, stateVec, 0, action, buffer);
}

void POMDP_Writer::MovePosition(std::string & buffer)
{
	std::vector<int>toExclude;

	for (size_t i = 0; i < m_gridSize; ++i)
	{
		toExclude.push_back(i);
	}
	toExclude.push_back(-1);

	std::string north = "North";
	MovePositionSingleDirection(buffer, -1 * m_gridSize, toExclude.begin(), north);

	for (size_t i = 0; i < m_gridSize; ++i)
	{
		toExclude[i] = m_gridSize * (m_gridSize - 1) + i;
	}
	std::string south = "South";
	MovePositionSingleDirection(buffer, m_gridSize, toExclude.begin(), south);

	for (size_t i = 0; i < m_gridSize; ++i)
	{
		toExclude[i] = m_gridSize * i;
	}
	std::string west = "West";
	MovePositionSingleDirection(buffer, -1, toExclude.begin(), west);

	for (size_t i = 0; i < m_gridSize; ++i)
	{
		toExclude[i] = m_gridSize * (i + 1) - 1;
	}
	std::string east = "East";
	MovePositionSingleDirection(buffer, 1, toExclude.begin(), east);
}

void POMDP_Writer::MovePositionSingleDirection(std::string & buffer, int advanceFactor, std::vector<int>::iterator toExclude, std::string & action)
{
	state_t stateVec(2 + m_NInvVector.size());
	state_t newStateVec(2 + m_NInvVector.size());
	
	// run on all possible location of the robot, if the move from the location will be possible calculate moves from the position
	for (size_t i = 0; i < m_gridSize * m_gridSize; ++i)
	{
		// insert current location of the robot
		stateVec[0] = i;
		newStateVec[0] = i + advanceFactor;

		// rely that the order of toExclude will be the same as the order of appearence of non-valid moves in i
		if (i != *toExclude)
		{
			CalcPositionRec(stateVec, newStateVec, 1, action, buffer);
		}
		else
		{
			++toExclude;
		}
	}
}


void POMDP_Writer::CalcPositionRec(state_t & originalStateVec, state_t & newStateVec, size_t currObj, std::string & action, std::string & buffer)
{
	if (currObj == newStateVec.size())
	{
		if (InEnemyRange(originalStateVec))
		{
			buffer += "T: " + action + " : " + GetCurrentState(originalStateVec) + " : " + s_LossState + " " + std::to_string(m_enemy.GetPHit()) + "\n";
			s_pLeftProbability = 1 - m_enemy.GetPHit();
		}
		PositionSingleState(newStateVec, GetCurrentState(originalStateVec), action, buffer);
		buffer += "\n";
		s_pLeftProbability = 1;
	}
	else
	{
		for (size_t i = 0; i < m_gridSize * m_gridSize; ++i)
		{
			originalStateVec[currObj] = i;
			newStateVec[currObj] = i;
			if (NoRepetition(originalStateVec, currObj))
			{
				CalcPositionRec(originalStateVec, newStateVec, currObj + 1, action, buffer);
			}
		}
		// if the object is enemy add cases of when the enemy is dead
		if (currObj == 1)
		{
			originalStateVec[currObj] = DEAD_ENEMY;
			newStateVec[currObj] = DEAD_ENEMY;
			CalcPositionRec(originalStateVec, newStateVec, currObj + 1, action, buffer);
		}
	}
}

void POMDP_Writer::PositionSingleState(state_t & stateVec, std::string & currentState, std::string & action, std::string & buffer)
{
	std::string prefix = "T: " + action + " : " + currentState + " : ";
	// if robot position is in the target go to win state
	if (stateVec[0] == s_idxTarget)
	{
		buffer += prefix + s_WinState + " " + std::to_string(s_pLeftProbability) + "\n";
		return;
	}
	prefix += "s";

	// calculate possible move states from current location
	int *moveStates = new int[5 * (1 + m_NInvVector.size())];
	CalcMoveStates(stateVec, moveStates);

	// array of idx pointing to the current move state
	size_t *arrOfIdx = new size_t[1 + m_NInvVector.size()];
	for (size_t i = 0; i < 1 + m_NInvVector.size(); ++i)
	{
		arrOfIdx[i] = i * 5;
	}

	mapProb pMap;
	// calculate the probability of each move state and insert it to pMap
	AddMoveStatesRec(stateVec, moveStates, arrOfIdx, 0, pMap);
	// erase the junk state from the pMap
	pMap.erase(s_junkState);
	// insert the move states to the buffer
	size_t numStates = m_gridSize * m_gridSize;
	std::for_each(pMap.begin(), pMap.end(), [&buffer, &prefix, numStates](pairMap itr)
	{	buffer += prefix;	POMDP_Writer::AddStateToBuffer(buffer, itr, numStates); });

	delete[] moveStates;
	delete[] arrOfIdx;
}

void POMDP_Writer::AddStateToBuffer(std::string& buffer, pairMap itr, size_t numStates)
{
	buffer += std::to_string(itr.first[0]);
	size_t start = 1;

	// if the enemy dead add his state
	if (itr.first[ENEMY_IDX] == DEAD_ENEMY)
	{
		buffer += "xD";
		++start;
	}
	for (size_t i = start; i < itr.first.size(); ++i)
	{
		buffer += "x" + std::to_string(itr.first[i]);
	}
	buffer += " " + std::to_string(itr.second) + "\n";
}

size_t POMDP_Writer::CountEdges(int state, size_t gridSize)
{
	size_t x = state % gridSize;
	size_t y = state / gridSize;

	return (x == 0) + (x == gridSize - 1) + (y == 0) + (y == gridSize - 1);
}

bool POMDP_Writer::InEnemyRange(state_t & stateVec)
{
	if (stateVec[ENEMY_IDX] == DEAD_ENEMY)
	{
		return false;
	}

	int xSelf = stateVec[0] % m_gridSize;
	int ySelf = stateVec[0] / m_gridSize;
	int xEnemy = stateVec[1] % m_gridSize;
	int yEnemy = stateVec[1] / m_gridSize;

	if (xEnemy == xSelf)
	{
		int advanceFactor = m_gridSize - 2 * m_gridSize * (ySelf < yEnemy);
		return InEnemyRangeIMP(stateVec, advanceFactor);
	}
	else if (yEnemy == ySelf)
	{
		int advanceFactor = 1 - 2 * (xSelf < xEnemy);
		return InEnemyRangeIMP(stateVec, advanceFactor);
	}

	return false;
}

bool POMDP_Writer::InEnemyRangeIMP(state_t & stateVec, int advanceFactor)
{
	int shot = stateVec[1] + advanceFactor;
	for (size_t i = 0; i < m_enemy.GetRange(); ++i)
	{
		if (SearchForShelter(shot))
		{
			break;
		}
		else if (stateVec[0] == shot)
		{
			return true;
		}
		shot += advanceFactor;
	}

	return false;
}

std::string POMDP_Writer::GetCurrentState(state_t & stateVec)
{
	std::string currentState = "s";
	size_t i = 0;
	for (; i < stateVec.size() - 1; ++i)
	{
		if (stateVec[i] != DEAD_ENEMY)
		{
			currentState += std::to_string(stateVec[i]) + "x";
		}
		else
		{
			currentState += "Dx";
		}
	}
	currentState += std::to_string(stateVec[i]);

	return std::move(currentState);
}



void POMDP_Writer::CalcHits(std::string & buffer)
{
	int *stateVec = new int[3 + m_NInvVector.size()];
	state_t stateV(2 + m_NInvVector.size());
	stateVec[2 + m_NInvVector.size()] = -1;
	CalcHitsRec(stateV, 0, buffer);
}

void POMDP_Writer::CalcHitsRec(state_t& stateVec, size_t currIdx, std::string & buffer)
{
	if (stateVec.size() == currIdx)
	{
		CalcHitsSingleState(stateVec, buffer);
	}
	else
	{
		for (size_t i = 0; i < m_gridSize * m_gridSize; ++i)
		{
			stateVec[currIdx] = i;
			if (NoRepetition(stateVec, currIdx))
			{
				CalcHitsRec(stateVec, currIdx + 1, buffer);
			}
		}
	}
}

void POMDP_Writer::CalcHitsSingleState(state_t& stateVec, std::string & buffer)
{
	std::string action = "Shoot_North";
	CalcHitsSingleDirection(stateVec, -1 * m_gridSize, action, buffer);

	action = "Shoot_South";
	CalcHitsSingleDirection(stateVec, m_gridSize, action, buffer);

	action = "Shoot_East";
	CalcHitsSingleDirection(stateVec, 1, action, buffer);

	action = "Shoot_West";
	CalcHitsSingleDirection(stateVec, -1, action, buffer);
}

void POMDP_Writer::CalcHitsSingleDirection(state_t & stateVec, int advanceFactor, std::string & action, std::string & buffer)
{
	int target = stateVec[0];
	std::string prefix = "T: " + action + " : " + GetCurrentState(stateVec) + " : ";

	// run on track of the shot to see what it hit
	for (size_t i = 0; i < m_self.GetRange() && InBoundary(target, advanceFactor, m_gridSize); ++i)
	{
		target += advanceFactor;

		// if the shot hit shelter do nothing
		if (SearchForShelter(target))
		{
			return;
		}

		// if the shot hit non-involved the calculate the chance for loss and the rest treat them as moving state
		for (size_t i = 0; i < m_NInvVector.size(); ++i)
		{
			if (stateVec[i + 2] == target)
			{
				CalcHitNInv(stateVec, action, prefix, buffer);
				return;
			}
		}

		// if the shot hits the target calculate the chance that the enemy is dead
		if (stateVec[1] == target)
		{
			CalcHitEnemy(stateVec, action, prefix, buffer);
			return;
		}
	}
}

void POMDP_Writer::CalcHitEnemy(state_t & stateVec, std::string & action, std::string & prefix, std::string & buffer)
{
	// if we are in enemy range calculate first the chance for enemy hit (if both enemy and robot hit we are at loss state)
	if (InEnemyRange(stateVec))
	{
		buffer += "T: " + action + " : " + GetCurrentState(stateVec) + " : " + s_LossState + " " + std::to_string(m_enemy.GetPHit()) + "\n";
		s_pLeftProbability *= 1 - m_enemy.GetPHit();
	}

	// calculate states with a dead enemy and a live robot
	s_pLeftProbability *= m_self.GetPHit();
	int tmp = stateVec[1];
	std::string currentState = GetCurrentState(stateVec);
	stateVec[1] = DEAD_ENEMY;
	PositionSingleState(stateVec, currentState, action, buffer);
	// return states and prob to normal
	stateVec[1] = tmp;
	s_pLeftProbability /= m_self.GetPHit();

	// calculate states with a miss
	s_pLeftProbability *= 1 - m_self.GetPHit();
	PositionSingleState(stateVec, GetCurrentState(stateVec), action, buffer);

	s_pLeftProbability = 1;
	buffer += "\n";
}

void POMDP_Writer::CalcHitNInv(state_t & stateVec, std::string & action, std::string & prefix, std::string & buffer)
{
	// if we are in enemy range calculate first the chance for enemy hit (if both enemy and robot hit we are at loss state)
	double pToLoss = 0.0;
	if (InEnemyRange(stateVec))
	{
		pToLoss = m_enemy.GetPHit();
	}

	// calculate p(robot dead | kill n-inv)
	pToLoss = pToLoss + m_self.GetPHit() - pToLoss * m_self.GetPHit();
	buffer += prefix + s_LossState + " " + std::to_string(pToLoss) + "\n";
	// calculate states with a miss
	s_pLeftProbability = 1 - pToLoss;
	PositionSingleState(stateVec, GetCurrentState(stateVec), action, buffer);

	s_pLeftProbability = 1;
	buffer += "\n";
}


bool POMDP_Writer::SearchForShelter(int location)
{
	for (auto v : m_shelter)
	{
		if (v.GetLocation().GetIdx(m_gridSize) == location)
		{
			return true;
		}
	}
	return false;
}

bool POMDP_Writer::InBoundary(int state, int advanceFactor, int gridSize)
{
	int x = state % gridSize;
	int y = state / gridSize;
	int xdiff = advanceFactor % gridSize;
	int ydiff = advanceFactor / gridSize;

return ((x + xdiff) >= 0) & ((x + xdiff) < gridSize) & ((y + ydiff) >= 0) & ((y + ydiff) < gridSize);
}

void POMDP_Writer::AddMoveStatesRec(state_t & stateVec, int * moveStates, size_t * arrOfIdx, size_t currIdx, mapProb & pMap)
{
	if (currIdx == stateVec.size() - 1)
	{
		pMap[MoveToIdx(stateVec, moveStates, arrOfIdx)] += CalcProb2Move(stateVec, moveStates, arrOfIdx);
	}
	else
	{
		size_t remember = arrOfIdx[currIdx];
		for (size_t i = 0; i < 5; ++i, ++arrOfIdx[currIdx])
		{
			AddMoveStatesRec(stateVec, moveStates, arrOfIdx, currIdx + 1, pMap);
		}

		arrOfIdx[currIdx] = remember;
	}
}


POMDP_Writer::state_t POMDP_Writer::MoveToIdx(state_t stateVec, int * moveStates, size_t * arrOfIdx)
{
	// insert the current move state to the state vec
	for (size_t i = 0; i < stateVec.size() - 1; ++i)
	{
		int currState = moveStates[arrOfIdx[i]];
		if (currState == NVALID_MOVE && moveStates[i * 5] == DEAD_ENEMY)
		{
			return s_junkState;
		}

		stateVec[i + 1] = currState * (-1 != currState) + moveStates[i * 5] * (NVALID_MOVE == currState);
	}

	// correct the state vec in case of repetitions
	NoRepetitionCheckAndCorrect(stateVec, moveStates, arrOfIdx);

	return stateVec;
}

double POMDP_Writer::CalcProb2Move(state_t & stateVec, int * moveStates, size_t * arrOfIdx)
{
	// insert to p the whole (replacement of 1 due to cases that reduce probability)
	double pMoveState = s_pLeftProbability;

	// multiply with the p(enemy = moveState) if the enemy is not dead
	if (stateVec[ENEMY_IDX] != DEAD_ENEMY)
	{
		pMoveState *= m_enemy.GetMovement().GetStay() * (arrOfIdx[0] == 0) +
			m_enemy.GetMovement().GetEqual() * (arrOfIdx[0] != 0);
	}
	
	// multiply with the p(non-involved = moveState)
	for (size_t i = 0; i < m_NInvVector.size(); ++i)
	{
		pMoveState *= m_NInvVector[i].GetMovement().GetStay() * (arrOfIdx[i + 1] % 5 == 0) +
			m_NInvVector[i].GetMovement().GetEqual() * (arrOfIdx[i + 1] % 5 != 0);
	}

	return pMoveState;
}

void POMDP_Writer::CalcMoveStates(state_t & stateVec, int * moveStates)
{
	// if the enemy is dead calculate move state accordingly
	size_t start = 0;
	if (stateVec[ENEMY_IDX] == DEAD_ENEMY)
	{
		moveStates[0] = DEAD_ENEMY;
		start = 1;
		for (size_t i = 1; i < 5; ++i)
		{
			moveStates[i] = NVALID_MOVE;
		}
	}

	// calculate possible move state (for non-valid move state insert -1 in movestates array)
	for (size_t i = start; i < stateVec.size() - 1; ++i)
	{
		int x = stateVec[i + 1] % m_gridSize;
		int y = stateVec[i + 1] / m_gridSize;
		moveStates[5 * i] = x + y * m_gridSize;
		moveStates[5 * i + 1] = ((x + 1) + (y * m_gridSize)) * (x + 1 < m_gridSize) + NVALID_MOVE * (x + 1 == m_gridSize);
		moveStates[5 * i + 2] = ((x - 1) + (y * m_gridSize)) * (x - 1 >= 0) + NVALID_MOVE * (x - 1 < 0);
		moveStates[5 * i + 3] = ((x)+((y + 1) * m_gridSize)) * (y + 1 < m_gridSize) + NVALID_MOVE * (y + 1 == m_gridSize);
		moveStates[5 * i + 4] = ((x)+((y - 1) * m_gridSize)) * (y - 1 >= 0) + NVALID_MOVE * (y - 1 < 0);
	}
}

void POMDP_Writer::CalcObs(std::string& buffer)
{
	state_t stateVec(2 + m_NInvVector.size());
	CalcObsRec(stateVec, 0, buffer);
}
void POMDP_Writer::CalcObsRec(std::vector<int>& stateVec, size_t currIdx, std::string & buffer)
{
	if (currIdx == stateVec.size())
	{
		// arriving here when stateVec is initialize to a state. run on this state calculation of observations
		CalcObsSingleState(stateVec, buffer);
		buffer += "\n";
	}
	else
	{
		// run on all possible locations. if there location is valid call for the calculation of the next object
		for (size_t i = 0; i < m_gridSize*m_gridSize; ++i)
		{
			stateVec[currIdx] = i;
			if (NoRepetition(stateVec, currIdx))
			{
				CalcObsRec(stateVec, currIdx + 1, buffer);
			}
		}

		if (currIdx == ENEMY_IDX)
		{
			stateVec[currIdx] = DEAD_ENEMY;
			CalcObsRec(stateVec, currIdx + 1, buffer);
		}
	}

}

void POMDP_Writer::CalcObsSingleState(state_t& stateVec, std::string& buffer)
{
	std::string prefix = "O: * : " + GetCurrentState(stateVec) + " : o";
	std::vector<bool> inRange(stateVec.size());
	state_t newState(stateVec);
	mapProb pMap;

	// create a vector indicating which one of the different object is in range
	//if (InObsRange(stateVec[0], stateVec[1], m_gridSize, m_self.GetRange()) || )
	for (size_t i = 1; i < stateVec.size(); ++i)
	{
		if (InObsRange(stateVec[0], stateVec[i], m_gridSize, m_self.GetRange()))
		{
			inRange[i] = true;
		}
		else
		{
			inRange[i] = false;
		}
	}

	CalcObsMapRec(newState, stateVec, pMap, inRange, 1.0, 1);
	size_t numStates = m_gridSize * m_gridSize;
	std::for_each(pMap.begin(), pMap.end(), [&buffer, &prefix, numStates](pairMap itr)
	{	buffer += prefix;	POMDP_Writer::AddStateToBuffer(buffer, itr, numStates); });
}

void POMDP_Writer::CalcObsMapRec(state_t& stateVec, state_t& originalState, mapProb& pMap, std::vector<bool>& inRange, double pCurr, size_t currIdx)
{
	// stopping condition: arriving to the end of the state vec
	if (currIdx == stateVec.size())
	{
		// insert p to map
		pMap[stateVec] = pCurr;
	}
	else
	{
		// if the original location is in range & the current location is the original location and there are no repetition the location is observable
		if (inRange[currIdx] & stateVec[currIdx] == originalState[currIdx] & NoRepetition(stateVec, currIdx))
		{
			CalcObsMapRec(stateVec, originalState, pMap, inRange, pCurr * m_self.GetPObs(), currIdx + 1);
			DivergeObs(stateVec, originalState, pMap, inRange, pCurr * (1 - m_self.GetPObs()), currIdx, true);
		}
		else
		{		
			DivergeObs(stateVec, originalState, pMap, inRange, pCurr, currIdx, false);
		}
	}

}

void POMDP_Writer::DivergeObs(state_t& stateVec, state_t& originalState, mapProb& pMap, std::vector<bool>& inRange, double pCurr, size_t currIdx, bool avoidCurrLoc)
{
	// if the enemy is dead do not run on other options(because they are not possible)
	if (stateVec[currIdx] == DEAD_ENEMY)
	{
		CalcObsMapRec(stateVec, originalState, pMap, inRange, pCurr, currIdx + 1);
		return;
	}

	int currLocation = stateVec[currIdx];
	//calculate how many diversion there will be decrease repetitions(- currIdx), add not important repetition(DEAD_ENEMY) and decrease the current location if necessary
	size_t pDivision = m_gridSize * m_gridSize - currIdx + (stateVec[ENEMY_IDX] == DEAD_ENEMY) - (avoidCurrLoc);

	for (size_t i = 0; i < m_gridSize * m_gridSize; ++i)
	{
		stateVec[currIdx] = i;

		// if idx is curr location and is in range or if idx is repeated in previous locations do not call recursive function
		if ( !(avoidCurrLoc && i == currLocation) && NoRepetition(stateVec, currIdx))
		{
			CalcObsMapRec(stateVec, originalState, pMap, inRange, pCurr / pDivision, currIdx + 1);
		}	
	}

	stateVec[currIdx] = currLocation;
}

bool POMDP_Writer::InObsRange(int self, int object, size_t gridSize, size_t range)
{
	int xSelf = self % gridSize;
	int ySelf = self / gridSize;
	int xObj = object % gridSize;
	int yObj = object / gridSize;

	if (Abs(xSelf - xObj) <= range && Abs(ySelf - yObj) <= range)
	{
		return true;
	}
	return false;
}

size_t POMDP_Writer::NextInLine(std::vector<bool>& inRange, size_t currIdx)
{
	bool isInRange = inRange[currIdx];
	auto next = std::find(inRange.begin() + currIdx, inRange.end(), isInRange);
	
	if (next == inRange.end() && isInRange)
	{
		next = std::find(inRange.begin(), inRange.end(), false);
	}

	return next - inRange.begin();
}

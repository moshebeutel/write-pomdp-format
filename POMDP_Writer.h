//	Purpose: create POMDP format given specific state (including 1 robot, 1 enemy, non-involved movable objects and shelters on NxN Grid)
//			win is killing the enemy, loss is killing non-involved and the death of the robot by the enemy.

//	Author: Natan Gold

//	COMMENTS REGARDING THE RULES OF THE GAME:
//	1-	objects can be in the same idx in the grid
//	2-	if the enemy and the non-involved in the same idx shooting toward them will be considered as a loss
//	3-	shooting action is a timeless action and no transition and moving can accure while shooting

// COMMENTS REGARDING IMPLEMENTATION:
//	1-	so far the charging of the enemy toward the target is not implemented

#pragma once

#include <vector>
#include <memory>
#include <map>

#include "Self_Obj.h"
#include "Attack_Obj.h"
#include "Movable_Obj.h"
#include "ObjInGrid.h"

class POMDP_Writer
{
public:
	POMDP_Writer(size_t gridSize, Self_Obj& self, Attack_Obj& enemy, double discount = 0.95);
	~POMDP_Writer() = default;

	void AddObj(Movable_Obj& obj);
	void AddObj(ObjInGrid& obj);

	void SaveInFormat(FILE *fptr, size_t idxTarget);

private:
	size_t m_gridSize;
	
	Self_Obj m_self;
	Attack_Obj m_enemy;
	std::vector<Movable_Obj> m_NInvVector;
	std::vector<ObjInGrid> m_shelter;
	double m_discount;

	using state_t = std::vector<int>;
	using mapProb = std::map<state_t, double>;
	using pairMap = std::pair<state_t, double>;

	// to throw undesirable move to
	static state_t s_junkState;

	void CommentsAndInitLines(std::string buffer, FILE *fptr);
	void PositionStates(std::string buffer, FILE *fptr);
	void AttackAction(std::string buffer, FILE *fptr);
	void ObservationsAndRewards(std::string buffer, FILE *fptr);

	// Calculation of possible states
	static void CalcStatesAndObs(size_t numObjects, size_t gridSize, std::string& type, std::string& buffer);
	// recursion on all possible states or observations
	static void CalcS_ORec(state_t& stateVec, size_t currIdx, size_t gridSize, std::string& type, std::string& buffer);

	// divide the probability to init in a given state to all other states(in case the state is not possible)
	static double DividePRepetitions(state_t& stateVec, double *pMat, int currIdx, size_t numStates);
	// return the division from before to the revregular distribution
	static void ReturnPRepetition(double *pMat, size_t numStates, double pRep);

	// Calculation of initial state:
	void CalcStartState(std::string& buffer);
	void CalcStartStateRec(double * pmat, state_t& stateVec, size_t currIdx, std::string& buffer);

	static void CalcSinglePosition(ObjInGrid *obj, size_t gridSize, double *pMat);
	static void CalcSinglePositionNoStd(ObjInGrid *obj, size_t gridSize, double *pMat);
	static double CumulativeDistFunc(double x, int mean, double std);

	
	// Calculation of move possibility
	void NoMovePosition(std::string& buffer);	// calculation move probabilities when the robot do not move
	void MovePosition(std::string& buffer);		// calculation move probabilities when the robot move
	// calculation of single direction move(i.e. north,east etc.)
	void MovePositionSingleDirection(std::string& buffer, int advanceFactor, std::vector<int>::iterator toExclude, std::string& action);

	// run on all possible states and calculate the end-state
	void CalcPositionRec(state_t& originalStateVec, state_t& newStateVec, size_t currObj, std::string& action, std::string & buffer);

	// calculate the end-state position from a single state(stateVec)
	void PositionSingleState(state_t& stateVec, std::string& currentState, std::string& action, std::string& buffer);

	// calculate possible move states from a start-state
	void CalcMoveStates(state_t & stateVec, int *moveStates);
	// calculate the probability of the possible moveStates
	void AddMoveStatesRec(state_t & stateVec, int *moveStates, size_t *arrOfIdx, size_t currIdx, mapProb& pMap);


	// add state to buffer for the pomdp format
	static void AddStateToBuffer(std::string& buffer, pairMap itr, size_t m_gridSize);
	// count number of edges from a given location
	static size_t CountEdges(int state, size_t gridSize);
	
	// translate a state to the pomdp format
	static std::string GetCurrentState(state_t& stateVec);

	// returns the real end-state from a given moveState
	state_t MoveToIdx(state_t stateVec, int *moveStates, size_t *arrOfIdx);
	// calculate probability to move ffor a given moveState
	double CalcProb2Move(state_t & stateVec, int *moveStates, size_t *arrOfIdx);


	// return true if the robot is in enemy range
	bool InEnemyRange(state_t& stateVec);
	bool InEnemyRangeIMP(state_t& stateVec, int advanceFactor);

	//Calculation Of Hits
	void CalcHits(std::string& buffer);
	void CalcHitsRec(state_t& array, size_t currIdx, std::string& buffer);

	// calculation of single state attacks
	void CalcHitsSingleState(state_t& stateVec, std::string & buffer);

	// calculation of hits for single state single direction attack
	void CalcHitsSingleDirection(state_t& stateVec, int advanceFactor, std::string & prefix, std::string & buffer);

	void CalcHitEnemy(state_t& stateVec, std::string & action, std::string & prefix, std::string & buffer);
	void CalcHitNInv(state_t & stateVec, std::string & action, std::string & prefix, std::string & buffer);

	// returns true if the location is sheltered
	bool SearchForShelter(int location);
	// return true if state + advance factor is inside the grid
	static bool InBoundary(int state, int advanceFactor, int gridSize);

	//Calculation Of Observations
	void CalcObs(std::string& buffer);
	void CalcObsRec(state_t& stateVec, size_t currIdx, std::string& buffer);

	void CalcObsSingleState(state_t& stateVec, std::string& buffer);
	void CalcObsMapRec(state_t& stateVec, state_t& originalState, mapProb& pMap, std::vector<bool>& inRange, double pCurr, size_t currIdx);
	void DivergeObs(state_t& stateVec, state_t& originalState, mapProb& pMap, std::vector<bool>& inRange, double pCurr, size_t currIdx, bool isPrevRange);
	static bool InObsRange(int self, int object, size_t gridSize, size_t range);
	static size_t NextInLine(std::vector<bool>& inRange, size_t currIdx);
	

	// search for repetition in a stateVec or moveState.
	static bool NoRepetition(state_t& stateVec, size_t currIdx);
	static void NoRepetitionCheckAndCorrect(state_t& stateVec, int *moveStates, size_t *arrOfIdx);
};


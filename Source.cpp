#include "POMDP_Writer.h"
#include "Point.h"
#include "Move_Properties.h"
#include "Movable_Obj.h"
#include "Attack_Obj.h"
#include <random>
#include <type_traits>
#include <iostream>
#include <map>

int main()
{
	Point locSelf(1, 1, 0.5);
	Move_Properties mSelf(0.2);
	Self_Obj self(locSelf, mSelf, 2, 0.8, 1, 0.9);

	Point locEnemy(0, 0, 1);
	Move_Properties mEnemy(0.6);
	Attack_Obj enemy(locEnemy, mEnemy, 1, 0.2);

	POMDP_Writer pomdp(3, self, enemy);

	Point x1(1, 2);
	Move_Properties p1(0.8);
	Movable_Obj N1(x1, p1);
	pomdp.AddObj(N1);

	//Point x2(1, 2);
	//Move_Properties p2(0.2);
	//Movable_Obj N2(x2, p2);
	//pomdp.AddObj(N2);

	Point x3(0, 2);
	ObjInGrid s1(x3);
	pomdp.AddObj(s1);

	remove("nxnGrid.POMDP");

	FILE *fptr;
	auto err = fopen_s(&fptr, "nxnGrid.POMDP", "w");
	if (0 != err)
	{
		std::cout << "ERROR OPEN\n";
	}

	pomdp.SaveInFormat(fptr, 8);
	fclose(fptr);


	char c;
	std::cout << "press any key";
	std::cin >> c;
}

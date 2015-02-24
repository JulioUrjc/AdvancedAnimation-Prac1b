#include "Point.h"
#include "Scene.h"
#include "CGSolver.h"

// No se necesita la variable implicit, es igual para los dos metodos implicito y simplectico
void Point::AddToLinearSystem(CGSolver *solver, bool implicit){
	// A=M , b=M*v+h*f
	Matrix3 matrixM = Matrix3(mass, 0, 0, 0, mass, 0, 0, 0, mass);
	Vec3 b = matrixM*velocity + Scene::step*force;

	solver->AddToDiagBlock(matrixM, simIndex);
	solver->AddToVectorBlock(b, simIndex);
}


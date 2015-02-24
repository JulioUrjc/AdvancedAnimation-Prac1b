#include "Spring.h"
#include "Point.h"
#include "Scene.h"
#include "CGSolver.h"

// No se necesita la variable implicit, es igual para los dos metodos simplectico e implicito
void Spring::AddForces(bool implicit){
	
	Vec3 u1 = (1.0f / length) * (a->GetPosition() - b->GetPosition());
	Vec3 f = (stiffness*(rest - length)) * u1;

	if (!this->HalfFixed()){
		a->AddForce(f);
		b->AddForce(-f);
	}
	else{
		(a->IsFixed()) ? b->AddForce(-f) : a->AddForce(f);
	}
}

void Spring::AddToLinearSystem(CGSolver *solver, bool implicit){
	// A= M-h^2*dFdp
	if (implicit){
		Vec3 u1 = (1.0f / length) * (a->GetPosition() - b->GetPosition());
		Matrix3 dFdp = (stiffness*(rest / length - 1)) * Matrix3::IDENTITY - (((stiffness*rest / length) * u1)^u1);
		Matrix3 h2dFdp = -pow(Scene::step,2)*dFdp;

		solver->AddToDiagBlock(h2dFdp, a->GetSimIndex());
		solver->AddToDiagBlock(h2dFdp, b->GetSimIndex());
		solver->AddToSparseBlock(-h2dFdp, simIndex);
	}
}

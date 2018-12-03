#ifndef __CLOTH_H__
#define __CLOTH_H__
#include "fem/World.h"
#include "fem/Mesh/MeshHeader.h"
#include "fem/Constraint/ConstraintHeader.h"
class Cloth 
{
public:
	Cloth();
	void Initialize(FEM::World* world);
	void SetMesh();
	FEM::Mesh* GetMesh() {return mMesh;};
	
private:
	std::vector<FEM::Constraint*>						mConstraints;
	FEM::Mesh*											mMesh;

	double 												mStretchingStiffness;
	double 												mBendingStiffness;
};

#endif
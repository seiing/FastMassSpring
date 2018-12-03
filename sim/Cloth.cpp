#include "Cloth.h"
#include <iostream>
using namespace FEM;

Cloth::
Cloth()
:mMesh(),mStretchingStiffness(1E2),mBendingStiffness(20.0)
{

}
void
Cloth::
Initialize(FEM::World* world)
{
	Eigen::Affine3d T=Eigen::Affine3d::Identity();
	mMesh = new GridMesh(10,10,10.0,10.0,Eigen::Vector3d(-1.0,1.0,0),T);	

	const auto& particles = mMesh->GetParticles();
	const auto& springs = mMesh->GetSprings();

	int idx = 0;
	for(const auto& spr : springs) 
	{
		int i0,i1; 
		Eigen::Vector3d p0,p1;

		i0 = spr[0];
		i1 = spr[1];
		p0 = particles[i0];
		p1 = particles[i1];

		double l0 = (p0-p1).norm();

		mConstraints.push_back(new SpringConstraint(mStretchingStiffness,i0,i1,l0));
		idx +=1;
	}

	world->AddConstraint(new FEM::AttachmentConstraint(500000,1*10-1,particles[1*10-1]));
	// world->AddConstraint(new FEM::AttachmentConstraint(500000,3*10-1,particles[3*10-1]));
	// world->AddConstraint(new FEM::AttachmentConstraint(500000,5*10-1,particles[5*10-1]));
	// world->AddConstraint(new FEM::AttachmentConstraint(500000,7*10-1,particles[7*10-1]));
	// world->AddConstraint(new FEM::AttachmentConstraint(500000,10*10-1,particles[10*10-1]));

	Eigen::VectorXd p(particles.size()*3);
	for(int i =0;i<particles.size();i++)
		p.block<3,1>(i*3,0) = particles[i];

	world->AddBody(p,mConstraints,1.0);

	for(auto& c: mConstraints) {
		world->AddConstraint(c);
	}
}

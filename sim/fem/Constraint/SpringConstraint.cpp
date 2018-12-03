#include "Constraint.h"
#include "SpringConstraint.h"
#include <iostream>
using namespace FEM;
SpringConstraint::
SpringConstraint(const double& stiffness,int i0,int i1,double l0)
	:Constraint(stiffness),mi0(i0),mi1(i1),ml0(l0)
{
	mE = 0.0;
	mg.setZero();
	mH.setZero();
}
void
SpringConstraint::
EvaluatePotentialEnergy(const Eigen::VectorXd& x)
{
	Eigen::Vector3d x_01 = x.block<3,1>(mi1*3,0)-x.block<3,1>(mi0*3,0);
	double l = x_01.norm();
	mE = 0.5*mStiffness*(l-ml0)*(l-ml0);
}
void
SpringConstraint::
EvaluateGradient(const Eigen::VectorXd& x)
{
	Eigen::Vector3d x_01 = x.block<3,1>(mi1*3,0)-x.block<3,1>(mi0*3,0);	
	double l = x_01.norm();
	Eigen::Vector3d g = mStiffness*(ml0-l)*x_01.normalized();
	mg = g;
}
void
SpringConstraint::
EvaluateHessian(const Eigen::VectorXd& x)
{
	Eigen::Vector3d x_01 = x.block<3,1>(mi1*3,0)-x.block<3,1>(mi0*3,0);	
	double l = x_01.norm();
	Eigen::Matrix3d K = mStiffness * (Eigen::Matrix3d::Identity() - ml0/l*(Eigen::Matrix3d::Identity() - (x_01*x_01.transpose())/(l*l)));

	mH = K;
}
void
SpringConstraint:: 
GetPotentialEnergy(double& e)
{
	e +=mE;
}
void
SpringConstraint::	
GetGradient(Eigen::VectorXd& g)
{
	g.block<3,1>(mi0*3,0) += mg.block<3,1>(0*3,0);
	g.block<3,1>(mi1*3,0) -= mg.block<3,1>(0*3,0);
}
void
SpringConstraint:: 
GetHessian(std::vector<Eigen::Triplet<double>>& h_triplets)
{
	for(int i=0;i<3;i++) {
		for(int j=0;j<3;j++) {
			double val = mH(i,j);
			h_triplets.push_back(Eigen::Triplet<double>(mi0*3+i,mi0*3+j,val));
			h_triplets.push_back(Eigen::Triplet<double>(mi1*3+i,mi0*3+j,-val));
			h_triplets.push_back(Eigen::Triplet<double>(mi0*3+i,mi1*3+j,-val));
			h_triplets.push_back(Eigen::Triplet<double>(mi1*3+i,mi1*3+j,val));
		}
	}
}
void
SpringConstraint::
EvaluateJMatrix(int& index, std::vector<Eigen::Triplet<double>>& J_triplets)
{
    J_triplets.push_back(Eigen::Triplet<double>(3*mi0+0,3*index+0,mStiffness));
    J_triplets.push_back(Eigen::Triplet<double>(3*mi0+1,3*index+1,mStiffness));
    J_triplets.push_back(Eigen::Triplet<double>(3*mi0+2,3*index+2,mStiffness));
    
    J_triplets.push_back(Eigen::Triplet<double>(3*mi1+0,3*index+0,-mStiffness));
    J_triplets.push_back(Eigen::Triplet<double>(3*mi1+1,3*index+1,-mStiffness));
    J_triplets.push_back(Eigen::Triplet<double>(3*mi1+2,3*index+2,-mStiffness));

    index+=1;
}
void
SpringConstraint::
EvaluateLMatrix(std::vector<Eigen::Triplet<double>>& L_triplets)
{
	L_triplets.push_back(Eigen::Triplet<double>(3*mi0+0,3*mi0+0,mStiffness));
	L_triplets.push_back(Eigen::Triplet<double>(3*mi0+1,3*mi0+1,mStiffness));
	L_triplets.push_back(Eigen::Triplet<double>(3*mi0+2,3*mi0+2,mStiffness));

	L_triplets.push_back(Eigen::Triplet<double>(3*mi0+0,3*mi1+0,-mStiffness));
	L_triplets.push_back(Eigen::Triplet<double>(3*mi0+1,3*mi1+1,-mStiffness));
	L_triplets.push_back(Eigen::Triplet<double>(3*mi0+2,3*mi1+2,-mStiffness));

	L_triplets.push_back(Eigen::Triplet<double>(3*mi1+0,3*mi0+0,-mStiffness));
	L_triplets.push_back(Eigen::Triplet<double>(3*mi1+1,3*mi0+1,-mStiffness));
	L_triplets.push_back(Eigen::Triplet<double>(3*mi1+2,3*mi0+2,-mStiffness));

	L_triplets.push_back(Eigen::Triplet<double>(3*mi1+0,3*mi1+0,mStiffness));
	L_triplets.push_back(Eigen::Triplet<double>(3*mi1+1,3*mi1+1,mStiffness));
	L_triplets.push_back(Eigen::Triplet<double>(3*mi1+2,3*mi1+2,mStiffness));
}
void
SpringConstraint::
EvaluateDVector(int index,const Eigen::VectorXd& x,Eigen::VectorXd& d)
{
	Eigen::Vector3d x_01 = x.block<3,1>(mi1*3,0)-x.block<3,1>(mi0*3,0);	
	d.block<3,1>(3*index,0) = -ml0*x_01.normalized();
}
int
SpringConstraint::
GetDof()
{
	return 1;
}
ConstraintType
SpringConstraint::
GetType()	   
{
	return ConstraintType::SPRING; 
}


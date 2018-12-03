#include "Constraint.h"
#include "AttachmentConstraint.h"
#include <iostream>

using namespace FEM;
AttachmentConstraint::
AttachmentConstraint(const double& stiffness,int i0,const Eigen::Vector3d& p)
	:Constraint(stiffness),mi0(i0),mp(p)
{
	mE=0.0;
	mg.setZero();
	mH.setZero();
	md.setZero();
}
void
AttachmentConstraint::
EvaluatePotentialEnergy(const Eigen::VectorXd& x)
{
	mE = 0.5*mStiffness*((x.block<3,1>(mi0*3,0)-mp).squaredNorm());
}
void
AttachmentConstraint::
EvaluateGradient(const Eigen::VectorXd& x)
{
	mg = mStiffness*(x.block<3,1>(mi0*3,0)-mp);
}
void
AttachmentConstraint::
EvaluateHessian(const Eigen::VectorXd& x)
{
	Eigen::Vector3d kv;

	kv[0] = mStiffness;
	kv[1] = mStiffness;
	kv[2] = mStiffness;
	mH = kv.asDiagonal();
}
void
AttachmentConstraint::
GetPotentialEnergy(double& e)
{
	e += mE;
}
void
AttachmentConstraint::
GetGradient(Eigen::VectorXd& g)
{
	g.block<3,1>(mi0*3,0) += mg;
}
void
AttachmentConstraint::
GetHessian(std::vector<Eigen::Triplet<double>>& h_triplets)
{
	h_triplets.push_back(Eigen::Triplet<double>(3*mi0+0,3*mi0+0,mH(0,0)));
	h_triplets.push_back(Eigen::Triplet<double>(3*mi0+1,3*mi0+1,mH(1,1)));
	h_triplets.push_back(Eigen::Triplet<double>(3*mi0+2,3*mi0+2,mH(2,2)));
}
int
AttachmentConstraint::
GetDof()
{
	return 1;
}
ConstraintType 
AttachmentConstraint::
GetType()	   
{
	return ConstraintType::ATTACHMENT; 
}
void	
AttachmentConstraint::
EvaluateJMatrix(int& index, std::vector<Eigen::Triplet<double>>& J_triplets)
{
	J_triplets.push_back(Eigen::Triplet<double>(3*mi0+0,3*index+0,mStiffness));
	J_triplets.push_back(Eigen::Triplet<double>(3*mi0+1,3*index+1,mStiffness));
	J_triplets.push_back(Eigen::Triplet<double>(3*mi0+2,3*index+2,mStiffness));
	index++;
}
void	
AttachmentConstraint::
EvaluateLMatrix(std::vector<Eigen::Triplet<double>>& L_triplets)
{
	L_triplets.push_back(Eigen::Triplet<double>(3*mi0+0,3*mi0+0,mStiffness));
	L_triplets.push_back(Eigen::Triplet<double>(3*mi0+1,3*mi0+1,mStiffness));
	L_triplets.push_back(Eigen::Triplet<double>(3*mi0+2,3*mi0+2,mStiffness));
}
void
AttachmentConstraint::
EvaluateDVector(int index,const Eigen::VectorXd& x,Eigen::VectorXd& d)
{
	d.block<3,1>(3*index,0) = mp;
}
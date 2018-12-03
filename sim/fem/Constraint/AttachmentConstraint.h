#ifndef __ATTACHMENT_CONSTRAINT_H__
#define __ATTACHMENT_CONSTRAINT_H__	
#include "Constraint.h"
#include <Eigen/Core>
#include <Eigen/SparseCore>

namespace FEM
{
class Constraint;
enum ConstraintType;

class AttachmentConstraint : public Constraint
{
public:
	AttachmentConstraint(const double& stiffness,int i0,const Eigen::Vector3d& p);

	int GetDof() override;
	ConstraintType GetType() override;

	void	EvaluatePotentialEnergy(const Eigen::VectorXd& x) override;
	void	EvaluateGradient(const Eigen::VectorXd& x) override;
	void	EvaluateHessian(const Eigen::VectorXd& x) override;

	void 	GetPotentialEnergy(double& e) override;
	void	GetGradient(Eigen::VectorXd& g) override;
	void 	GetHessian(std::vector<Eigen::Triplet<double>>& h_triplets) override;

	void	EvaluateJMatrix(int& index, std::vector<Eigen::Triplet<double>>& J_triplets);
	void	EvaluateLMatrix(std::vector<Eigen::Triplet<double>>& L_triplets);
	void 	EvaluateDVector(int index,const Eigen::VectorXd& x,Eigen::VectorXd& d);

	int&			 GetI0() {return mi0;}
	Eigen::Vector3d& GetP()  {return mp;}

protected:
	int mi0;
	Eigen::Vector3d mp;
	
	double 			mE;
	Eigen::Vector3d mg;
	Eigen::Matrix3d mH;
	Eigen::Vector3d md;
};

};
#endif
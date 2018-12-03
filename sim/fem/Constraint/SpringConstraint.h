#ifndef __SPRING_CONSTRAINT_H__
#define __SPRING_CONSTRAINT_H__
#include "Constraint.h"
namespace Eigen {
typedef Matrix<double,6,1> Vector6d;
typedef Matrix<double,6,6> Matrix6d;
};
namespace FEM
{
class SpringConstraint : public Constraint
{
public:
	SpringConstraint(const double& stiffness,int i0,int i1,double l0);

    int GetDof() override;
	ConstraintType GetType() override;

	void  	EvaluatePotentialEnergy(const Eigen::VectorXd& x) override;
	void	EvaluateGradient(const Eigen::VectorXd& x) override;
	void	EvaluateHessian(const Eigen::VectorXd& x) override;

	void 	GetPotentialEnergy(double& e) override;
	void	GetGradient(Eigen::VectorXd& g) override;
	void 	GetHessian(std::vector<Eigen::Triplet<double>>& h_triplets) override;

	void 	EvaluateJMatrix(int& index, std::vector<Eigen::Triplet<double>>& J_triplets);
    void 	EvaluateLMatrix(std::vector<Eigen::Triplet<double>>& L_triplets);
	void 	EvaluateDVector(int index,const Eigen::VectorXd& x,Eigen::VectorXd& d);
	
	int& GetI0() {return mi0;};
	int& GetI1() {return mi1;};

protected:
	int mi0,mi1;
	double ml0;

	double 			mE;
	Eigen::Vector3d mg;
	Eigen::Matrix3d mH;
	Eigen::Vector3d md;
};
}
#endif
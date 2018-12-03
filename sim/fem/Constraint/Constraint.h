#ifndef __CONSTRAINT_H__
#define __CONSTRAINT_H__	
#include <Eigen/Core>
#include <Eigen/Sparse>
#include <Eigen/Geometry>
namespace FEM
{
enum ConstraintType
{
	ATTACHMENT,
	SPRING
};
class Constraint
{
public:
	Constraint(const double& stiffness);

	virtual int GetDof() = 0;
	virtual ConstraintType GetType() = 0;

	virtual void	EvaluatePotentialEnergy(const Eigen::VectorXd& e) = 0;
	virtual void	EvaluateGradient(const Eigen::VectorXd& x) = 0;
	virtual void	EvaluateHessian(const Eigen::VectorXd& x) = 0;

	virtual void 	GetPotentialEnergy(double& e) = 0;
	virtual void	GetGradient(Eigen::VectorXd& g) = 0;
	virtual void 	GetHessian(std::vector<Eigen::Triplet<double>>& h_triplets) = 0;

	virtual void	EvaluateJMatrix(int& index, std::vector<Eigen::Triplet<double>>& J_triplets) =0;
	virtual void	EvaluateLMatrix(std::vector<Eigen::Triplet<double>>& L_triplets) =0;
	virtual void 	EvaluateDVector(int index,const Eigen::VectorXd& x,Eigen::VectorXd& d) = 0;
	
protected:
	double mStiffness;

};
};
#endif
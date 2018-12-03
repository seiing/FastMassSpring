#ifndef __FEM_WORLD_H__
#define __FEM_WORLD_H__
#include <Eigen/Core>
#include <Eigen/SparseCore>
#include <Eigen/SparseCholesky>
#include <vector>
#include <deque>

#include "Mesh/MeshHeader.h"

namespace FEM
{
enum IntegrationMethod
{
	IMPLICIT_METHOD,
	PROJECTIVE_DYNAMICS,
};
enum OptimizationMethod {
	OPTIMIZATION_METHOD_NEWTON,
	OPTIMIZATION_METHOD_LBFGS
};
enum LinearSolveType {
	SOLVER_TYPE_CG,
	SOLVER_TYPE_LDLT,
};
class Constraint;
class AttachmentConstraint;
class World
{
public:
	World(
		IntegrationMethod integration_method=IMPLICIT_METHOD,
		OptimizationMethod optimization_method=OPTIMIZATION_METHOD_NEWTON,
		LinearSolveType linear_solve_type=SOLVER_TYPE_LDLT,
		double time_step = 1.0/100.0,
		int max_iteration = 100,
		double damping_coeff = 0.999
		);
	void 								Initialize();
	void								Reset();

	std::vector<Constraint*>&			GetConstraints();

	const Eigen::VectorXd& 				GetPositions();
	void 								SetPositions(const Eigen::VectorXd& x);
	
	const Eigen::VectorXd& 				GetVelocities();
	void 								SetVelocities(const Eigen::VectorXd& v);

	const int&		 					GetNumVertices();

	void 								AddBody(const Eigen::VectorXd& x0,const std::vector<Constraint*>& c,const double& mass = 1.0);
	void 								AddConstraint(Constraint* c);
	void 								RemoveConstraint(Constraint* c);
	int 								GetClosestNode(const Eigen::Vector3d& x);
	
public:	
	void 								ComputeExternalForces();
	void 								UpdatePositionsAndVelocities(const Eigen::VectorXd& x_n1);

public:
	double 								EvaluateEnergy(const Eigen::VectorXd& x);
	void 								EvaluateGradient(const Eigen::VectorXd& x,Eigen::VectorXd& g);
	void 								EvaluateHessian(const Eigen::VectorXd& x,Eigen::SparseMatrix<double>& H);

	double 								EvaluateConstraintsEnergy(const Eigen::VectorXd& x);
	void 								EvaluateConstraintsGradient(const Eigen::VectorXd& x,Eigen::VectorXd& g);
	void 								EvaluateConstraintsHessian(const Eigen::VectorXd& x,Eigen::SparseMatrix<double>& H);

public:
	void 								TimeStepping(bool isIntegrated = true);

public:
	Eigen::VectorXd						IntegrateImplicitMethod();
	bool								NewtonsMethod(Eigen::VectorXd& x);
	Eigen::VectorXd 					ConjugateGradient(const Eigen::VectorXd& b,const Eigen::VectorXd& x0);
	void								FactorizeLLT(const Eigen::SparseMatrix<double>& A,Eigen::SimplicialLLT<Eigen::SparseMatrix<double>>& lltSolver);
	void								FactorizeLDLT(const Eigen::SparseMatrix<double>& A,Eigen::SimplicialLDLT<Eigen::SparseMatrix<double>>& ldltSolver);

public:
	Eigen::VectorXd						ProjectiveDynamicsMethod();
	void 								PreComputation();
	void								EvaluateJMatrix(Eigen::SparseMatrix<double>& J);
	void								EvaluateLMatrix(Eigen::SparseMatrix<double>& L);
	void								EvaluateDVector(const Eigen::VectorXd& x,Eigen::VectorXd& d);

public:
	double 								GetTimeStep();
	double 								GetTime();
	void								SetTime(double t);

	Eigen::VectorXd						GetExternalForce() {return mExternalForces;};
	void								SetExternalForce(Eigen::VectorXd external_force);

public:
	bool 								mIsInitialized;
	
	IntegrationMethod 					mIntegrationMethod;
	OptimizationMethod 					mOptimizationMethod;
	LinearSolveType 					mLinearSolveType;

	std::vector<Constraint*>			mConstraints;
	int 								mConstraintDofs;

	/****************************************************************/
	
	std::vector<double>			 		mUnitMass;
	Eigen::SparseMatrix<double> 		mMassMatrix;
	Eigen::SparseMatrix<double> 		mInvMassMatrix;
	Eigen::SparseMatrix<double> 		mIdentityMatrix;
	int 								mNumVertices;

	/****************************************************************/
	double 								mTimeStep,mTime;
	int 								mMaxIteration;
	double 								mDampingCoefficinent;
	int 								mFrame;

	/****************************************************************/
	Eigen::VectorXd						mX,mV;
	Eigen::VectorXd						mInitX,mInitV;
	Eigen::VectorXd						mQn;
	Eigen::VectorXd						mExternalForces;

	/****************************************************************/
	FEM::Mesh*							mMesh;

	/****************************************************************/
	Eigen::SparseMatrix<double> 		mJ,mL;
	Eigen::SimplicialLDLT<Eigen::SparseMatrix<double>>	mDynamicSolver,mQuasiStaticSolver;

	/****************************************************************/
	bool								mPrefactorization;
};
};

#endif

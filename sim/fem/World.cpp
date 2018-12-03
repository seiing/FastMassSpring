#include "World.h"
#include "Constraint/ConstraintHeader.h"
#include "Mesh/MeshHeader.h"
#include <iostream>
#include <ctime>
#include <omp.h>
#define EPS 1E-10
using namespace FEM;
World::
World(
	IntegrationMethod integration_method,
		OptimizationMethod optimization_method,
		LinearSolveType linear_solve_type,
		double time_step,
		int max_iteration,
		double damping_coeff)
	:mIntegrationMethod(integration_method),
	mOptimizationMethod(optimization_method),
	mLinearSolveType(linear_solve_type),
	mTimeStep(time_step),
	mFrame(0),
	mMaxIteration(max_iteration),
	mDampingCoefficinent(damping_coeff),
	mNumVertices(0),
	mConstraintDofs(0),
	mIsInitialized(false),
	mPrefactorization(false)
{
}
void
World::
Initialize()
{
	mTime = 0.0;
	mConstraintDofs = 0;

	for(auto c : mConstraints){
		mConstraintDofs += c->GetDof();
	}

	mV.resize(3*mNumVertices);
	mV.setZero();

	mMassMatrix.resize(3*mNumVertices,3*mNumVertices);
	mInvMassMatrix.resize(3*mNumVertices,3*mNumVertices);
	mIdentityMatrix.resize(3*mNumVertices,3*mNumVertices);

	std::vector<Eigen::Triplet<double>> i_triplets;
	std::vector<Eigen::Triplet<double>> m_triplets;
	std::vector<Eigen::Triplet<double>> inv_m_triplets;

	i_triplets.reserve(3*mNumVertices);
	m_triplets.reserve(3*mNumVertices);
	inv_m_triplets.reserve(3*mNumVertices);
	
	for(int i = 0;i<mNumVertices;i++)
	{
		m_triplets.push_back(Eigen::Triplet<double>((i)*3+0,(i)*3+0,mUnitMass[i]));
		m_triplets.push_back(Eigen::Triplet<double>((i)*3+1,(i)*3+1,mUnitMass[i]));
		m_triplets.push_back(Eigen::Triplet<double>((i)*3+2,(i)*3+2,mUnitMass[i]));

		inv_m_triplets.push_back(Eigen::Triplet<double>((i)*3+0,(i)*3+0,1.0/mUnitMass[i]));
		inv_m_triplets.push_back(Eigen::Triplet<double>((i)*3+1,(i)*3+1,1.0/mUnitMass[i]));
		inv_m_triplets.push_back(Eigen::Triplet<double>((i)*3+2,(i)*3+2,1.0/mUnitMass[i]));

		i_triplets.push_back(Eigen::Triplet<double>((i)*3+0,(i)*3+0,1.0));
		i_triplets.push_back(Eigen::Triplet<double>((i)*3+1,(i)*3+1,1.0));
		i_triplets.push_back(Eigen::Triplet<double>((i)*3+2,(i)*3+2,1.0));
	}

	mMassMatrix.setFromTriplets(m_triplets.cbegin(), m_triplets.cend());
	mInvMassMatrix.setFromTriplets(inv_m_triplets.cbegin(), inv_m_triplets.cend());
	mIdentityMatrix.setFromTriplets(i_triplets.cbegin(), i_triplets.cend());

	mExternalForces.resize(3*mNumVertices);
	mExternalForces.setZero();

	mQn.resize(3*mNumVertices);
	mQn.setZero();

	mInitX=mX;
	mInitV=mV;

	if(mIntegrationMethod == PROJECTIVE_DYNAMICS)
		PreComputation();

	mIsInitialized = true;
	std::cout<<"Total degree of freedom : "<<mX.rows()<<std::endl;
	std::cout<<"Total constraints : "<<mConstraints.size()<<std::endl;
}
void
World::
Reset()
{
	mX = mInitX;
	mV = mInitV;
	mTime = 0.0;
}
void
World::
TimeStepping(bool isIntegrated)
{
	if(!mIsInitialized){
		std::cout<<"Engine not initialized."<<std::endl;
		return;
	}

	Eigen::VectorXd x_n1(mNumVertices*3);

	ComputeExternalForces();
	mQn = mX + mTimeStep*mV + (mTimeStep*mTimeStep)*(mInvMassMatrix*mExternalForces);

	switch(mIntegrationMethod) {
		case IMPLICIT_METHOD:
			x_n1=IntegrateImplicitMethod();
			break;
		case PROJECTIVE_DYNAMICS:
			x_n1=ProjectiveDynamicsMethod();
			break;
	}

	UpdatePositionsAndVelocities(x_n1);
	mV *= mDampingCoefficinent;

	if(isIntegrated)
	{	
		mTime += mTimeStep;
		mFrame++;
	} 
}
void
World::
AddBody(const Eigen::VectorXd& x0,const std::vector<Constraint*>& c,const double& mass)
{
	int nv = ((x0.rows())/3);
	mNumVertices += nv;

	auto temp_X(mX);
	mX.resize(mNumVertices*3);

	mX.head(temp_X.rows()) = temp_X;
	mX.tail(x0.rows()) = x0;
	// mConstraints.insert(mConstraints.end(), c.begin(), c.end());
	double unit_mass = mass/((double)nv);
	for(int i=0;i<nv;i++)
		mUnitMass.push_back(unit_mass);

	if(mIsInitialized)
		Initialize();
}
void
World::
AddConstraint(Constraint* c)
{
	mConstraints.push_back(c);
	if((mIntegrationMethod == IntegrationMethod::PROJECTIVE_DYNAMICS)
		&& mIsInitialized) {
		mConstraintDofs = 0;
		for(auto c : mConstraints){
			mConstraintDofs += c->GetDof();
		}

		PreComputation();
	}
}
void
World::
RemoveConstraint(Constraint* c)
{
	bool isRemoved = false;
	for(int i = 0;i<mConstraints.size();i++)
	{
		if(mConstraints[i]==c)
		{
			mConstraints.erase(mConstraints.begin() + i);
			isRemoved = true;
			break;
		}
	}

	if(isRemoved) {
		if((mIntegrationMethod == IntegrationMethod::PROJECTIVE_DYNAMICS) 
		&& mIsInitialized) {
			mConstraintDofs = 0;
			for(auto c : mConstraints){
				mConstraintDofs += c->GetDof();
			}

			PreComputation();
		}
	}
}
int
World::
GetClosestNode(const Eigen::Vector3d& x)
{
	double min_distance = 1E6;
	int ret = -1;
	for(int i=0;i<mNumVertices;i++){
		double distance = (mX.block<3,1>(i*3,0)-x).norm();
		if(min_distance>distance){
			ret = i;
			min_distance=distance;
		}
	}

	if(min_distance>1E-1)
		return -1;
	else
		return ret;
}
double 								
World::
GetTimeStep()
{
	return mTimeStep;
}
double 								
World::
GetTime()
{
	return mTime;
}
void
World::
SetTime(double t)
{
	mTime = t;
}
std::vector<Constraint*>&			
World::
GetConstraints()
{
	return mConstraints;
}
const Eigen::VectorXd& 				
World::
GetPositions()
{
	return mX;
}
void 
World::
SetPositions(const Eigen::VectorXd& x)
{
	mX = x;
}
const Eigen::VectorXd& 				
World::
GetVelocities()
{
	return mV;
}
void
World::
SetVelocities(const Eigen::VectorXd& v)
{
	mV=v;
}
const int&
World::
GetNumVertices()
{
	return mNumVertices;
}
double 
World::
EvaluateEnergy(const Eigen::VectorXd& x)
{
	Eigen::VectorXd x_q = x -mQn;
	double energy = EvaluateConstraintsEnergy(x);

	switch(mIntegrationMethod)
	{
		case IMPLICIT_METHOD:
			energy += 0.5*(1.0/(mTimeStep*mTimeStep))*(x_q.dot(mMassMatrix*x_q));
			break;
	}

	return energy;
}
void
World::
EvaluateGradient(const Eigen::VectorXd& x,Eigen::VectorXd& g)
{
	EvaluateConstraintsGradient(x,g);
	
	switch(mIntegrationMethod)
	{
		case IMPLICIT_METHOD:
			g += (1.0/(mTimeStep*mTimeStep))*mMassMatrix*(x - mQn);	
			break;
	}
}
void
World::
EvaluateHessian(const Eigen::VectorXd& x,Eigen::SparseMatrix<double>& H)
{
	EvaluateConstraintsHessian(x,H);

	switch(mIntegrationMethod)
	{
		case IMPLICIT_METHOD:
			H += (1.0/(mTimeStep*mTimeStep))*mMassMatrix;
			break;
	}
}
double 
World::
EvaluateConstraintsEnergy(const Eigen::VectorXd& x)
{
	double energy=0;
	
// #pragma omp parallel for
	for(int i=0;i<mConstraints.size();i++)
	{
		mConstraints[i]->EvaluatePotentialEnergy(x);
	}

	for(auto& c : mConstraints)
	{
		c->GetPotentialEnergy(energy);
	}

	return energy;
}
void
World::
EvaluateConstraintsGradient(const Eigen::VectorXd& x,Eigen::VectorXd& g)
{
	g.resize(mNumVertices*3);
	g.setZero();
// #pragma omp parallel for 
	for(int i =0;i<mConstraints.size();i++){
		mConstraints[i]->EvaluateGradient(x);
	}

	for(int i =0;i<mConstraints.size();i++){ 
		mConstraints[i]->GetGradient(g);	
	}
}
void
World::
EvaluateConstraintsHessian(const Eigen::VectorXd& x,Eigen::SparseMatrix<double>& H)
{
	H.resize(mNumVertices*3,mNumVertices*3);
	std::vector<Eigen::Triplet<double>> h_triplets;

	for(int i=0;i<mConstraints.size();i++)
	{
		mConstraints[i]->EvaluateHessian(x);
	}

	for(auto& c : mConstraints)
	{
		c->GetHessian(h_triplets);
	}

	H.setFromTriplets(h_triplets.cbegin(),h_triplets.cend());
}






Eigen::VectorXd
World::
IntegrateImplicitMethod()
{
	Eigen::VectorXd x_n1(3*mNumVertices);

	x_n1 = mQn;
	int i=0;

	bool converge = false;
	// while loop until converge or exceeds maximum iterations
	for(; i<mMaxIteration && !converge; i++) {
		converge = NewtonsMethod(x_n1);

		if (converge && i!=0)
		{
			std::cout << "Optimization Converged in iteration #" << i << std::endl;
			std::cout << std::endl;
		}
	}

	return x_n1;
}
bool 
World::
NewtonsMethod(Eigen::VectorXd& x) 
{
	int i=0;
	double step_size =0.01;
	Eigen::VectorXd g(3*mNumVertices);
	g.setZero();

	EvaluateGradient(x,g);
	if(g.squaredNorm() < 1E-4){
		return true;
	}

	Eigen::VectorXd dir;
	switch(mLinearSolveType)
	{		
		case SOLVER_TYPE_CG:
		{
			dir = -ConjugateGradient(g,x);
		}
		case SOLVER_TYPE_LDLT:
		{
			Eigen::SparseMatrix<double> A(3*mNumVertices,3*mNumVertices);
			A.setZero();
			EvaluateHessian(x,A);
			Eigen::SimplicialLDLT<Eigen::SparseMatrix<double>> A_solver;
			FactorizeLDLT(A,A_solver);
			dir = -A_solver.solve(g);
		}
	}
	x = x + step_size*dir;

	if (-dir.dot(g) < 1E-4) {
		return true;
	}
	else{
		return false;
	}
}
Eigen::VectorXd
World::
ConjugateGradient(const Eigen::VectorXd& b,const Eigen::VectorXd& x0)
{
	Eigen::VectorXd r(3*mNumVertices);
	Eigen::VectorXd p(3*mNumVertices);
	Eigen::VectorXd x(3*mNumVertices);
	Eigen::SparseMatrix<double> A(mNumVertices*3,mNumVertices*3);

	EvaluateHessian(x0,A);

	x = x0;
	r = b - A*x;
	p = r;
	double rs_old = r.dot(r);
	int i=0;
	Eigen::VectorXd Ap(3*mNumVertices);

	for(;i<10000;i++)
	{
		Ap = A*p;

		double alpha = rs_old / p.dot(Ap);
		x = x + alpha * p;
		r = r - alpha * Ap;

		double rs_new = r.dot(r);

		if (sqrt(rs_new) < 1E-10) {
			break;
		}
		p = r + (rs_new / rs_old)*p;
		rs_old = rs_new;
	}

	return x;
}
void
World::
FactorizeLLT(const Eigen::SparseMatrix<double>& A, Eigen::SimplicialLLT<Eigen::SparseMatrix<double>>& lltSolver)
{
	Eigen::SparseMatrix<double> A_prime = A;
	lltSolver.analyzePattern(A_prime);
	lltSolver.factorize(A_prime);
	double damping = 1E-6;
	bool success = true;
	while (lltSolver.info() != Eigen::Success)
	{
	    damping *= 10;
	    A_prime = A + damping*mIdentityMatrix;
	    lltSolver.factorize(A_prime);
	    success = false;
	}
	if (!success)
	    std::cout << "factorize failure (damping : " << damping<<" )"<<std::endl;
}
void
World::
FactorizeLDLT(const Eigen::SparseMatrix<double>& A,Eigen::SimplicialLDLT<Eigen::SparseMatrix<double>>& ldltSolver)
{
	Eigen::SparseMatrix<double> A_prime = A;
	ldltSolver.analyzePattern(A_prime);
	ldltSolver.factorize(A_prime);
	double reg = 1E-6;
	bool success = true;
	while (ldltSolver.info() != Eigen::Success)
	{
	    reg *= 10;
	    A_prime = A + reg*mIdentityMatrix;
	    ldltSolver.factorize(A_prime);
	    success = false;
	}
	if (!success)
	    std::cout << "factorize failure (damping : " << reg<<" )"<<std::endl;
}




Eigen::VectorXd
World::
ProjectiveDynamicsMethod()
{
	Eigen::VectorXd x_n1(3*mNumVertices);
	Eigen::VectorXd x_n1_new(3*mNumVertices);
	Eigen::VectorXd b(3*mNumVertices);
	Eigen::VectorXd d(3*mConstraintDofs);
	d.setZero();
	b= (1.0/(mTimeStep*mTimeStep))*mMassMatrix*mQn;

	x_n1 =mQn;

	int i;
	for(i=0; i<mMaxIteration; i++) {
		EvaluateDVector(x_n1,d);
		x_n1_new = mDynamicSolver.solve(b+mJ*d);
		if((x_n1_new - x_n1).norm()/x_n1.size() < EPS) {
			break;
		} 
		x_n1 = x_n1_new;
	}

	// std::cout << "Optimization Converged in iteration #" << i << std::endl;

	return x_n1;
}
void
World::
PreComputation()
{
	EvaluateJMatrix(mJ);
	EvaluateLMatrix(mL);
	Eigen::SparseMatrix<double> H2ML = (1.0/(mTimeStep*mTimeStep))*mMassMatrix+mL;
	FactorizeLDLT(H2ML,mDynamicSolver);
}
void
World::
EvaluateJMatrix(Eigen::SparseMatrix<double>& J) 
{
	J.resize(3*mNumVertices,3*mConstraintDofs);
	std::vector<Eigen::Triplet<double>> J_triplets;
	J_triplets.clear();

	int index = 0;
	for(auto& c : mConstraints) {
		c->EvaluateJMatrix(index,J_triplets);
	}
	J.setFromTriplets(J_triplets.cbegin(), J_triplets.cend());
}
void
World::
EvaluateLMatrix(Eigen::SparseMatrix<double>& L) 
{
	L.resize(3*mNumVertices,3*mNumVertices);

	std::vector<Eigen::Triplet<double>> L_triplets;

	for(auto& c : mConstraints)
		c->EvaluateLMatrix(L_triplets);

	L.setFromTriplets(L_triplets.cbegin(), L_triplets.cend());
}
void
World::
EvaluateDVector(const Eigen::VectorXd& x,Eigen::VectorXd& d) 
{
	d.resize(mConstraintDofs*3);
	d.setZero();
	for(int i=0;i<mConstraints.size();i++)
	{
		mConstraints[i]->EvaluateDVector(i,x,d);
	}
}
void
World::
ComputeExternalForces()
{
	// Add Gravity forces
	for(int i=0;i<mNumVertices;i++)
		mExternalForces[3*i+1] = -9.81;
	
	mExternalForces = mMassMatrix * mExternalForces;
}
void
World::
SetExternalForce(Eigen::VectorXd external_force)
{
	mExternalForces = external_force;
}
void
World::
UpdatePositionsAndVelocities
(const Eigen::VectorXd& x_n1)
{
	mV = (x_n1 - mX)*(1.0/mTimeStep);
	mX = x_n1;
}
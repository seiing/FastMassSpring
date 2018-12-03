#ifndef __FEM_INTERFACE_H__
#define __FEM_INTERFACE_H__
#include "../sim/fem/Constraint/ConstraintHeader.h"
#include <deque>

void DrawConstraint(FEM::Constraint* c,const Eigen::VectorXd& x);
void DrawActivation(const double& x, const double& y,const double& length,const std::vector<Eigen::VectorXd>& act,const int& cur_frame);
void DrawActivation(const double& x, const double& y,const double& length,const Eigen::VectorXd& act);
#endif

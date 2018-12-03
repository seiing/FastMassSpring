#ifndef __GRID_MESH_H__
#define __GRID_MESH_H__
#include <Eigen/Core>
#include <Eigen/Geometry>

namespace FEM
{
class Mesh;
class GridMesh : public Mesh
{
public:
	GridMesh(const int& w_num, const int& h_num,
		const double& w_length, const double& h_length,
		const Eigen::Vector3d& origin,
		const Eigen::Affine3d& T = Eigen::Affine3d::Identity());
};
};
#endif
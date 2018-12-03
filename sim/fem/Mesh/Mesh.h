#ifndef __MESH_H__
#define __MESH_H__
#include <Eigen/Core>
#include <Eigen/Sparse>
#include <Eigen/Geometry>
#include <memory>
namespace FEM
{
class Mesh
{
public:
	Mesh(){};
	virtual const std::vector<Eigen::Vector3d>& GetParticles(){return mParticles;};
	virtual const std::vector<Eigen::Vector2d>& GetSprings(){return mSprings;};
	virtual void Clear() {mParticles.clear(); mSprings.clear();};
	
protected:
	std::vector<Eigen::Vector3d> mParticles;
	std::vector<Eigen::Vector2d> mSprings;
};
};


#endif
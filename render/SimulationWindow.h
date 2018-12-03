#ifndef __SIMULATION_WINDOW_H__
#define __SIMULATION_WINDOW_H__
#include "GLWindow.h"
#include "GL_Functions.h"
#include "FEM_Interface.h"
#include "fem/Constraint/ConstraintHeader.h"
#include "fem/World.h"
#include "Cloth.h"
enum MOUSE_MODE
{
	CAMERA_CONTROL,
	CONSTRAINT_CONTROL
};
class SimulationWindow : public GLWindow
{
public:
	SimulationWindow();
	SimulationWindow(std::string filename);

protected:
	void Display() override;
	void Keyboard(unsigned char key,int x,int y) override;
	void Mouse(int button, int state, int x, int y) override;
	void Motion(int x, int y) override;
	void Reshape(int w, int h) override;
	void Timer(int value) override;
	void Screenshot();

protected:
	FEM::AttachmentConstraint* 	mDragConstraint;

	FEM::World*					mSoftWorld;
	Cloth*						mCloth;

	bool mIsRotate;
	bool mPlay;
	bool mCapture;

	int mCurFrame;
	int mTotalFrame;
	double mElapsedTime;
	double mTimeStep;
};
#endif
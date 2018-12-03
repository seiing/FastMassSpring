#include "SimulationWindow.h"
#ifdef __APPLE__
#include <OpenGL/gl.h>
#include <OpenGL/glu.h>
#include <GLUT/glut.h>
#else
#include <GL/gl.h>
#include <GL/glu.h>
#include <GL/glut.h>
#endif
// #include <boost/filesystem.hpp>
// #include "lodepng.h"

#include <chrono>
#include <ctime>

SimulationWindow::
SimulationWindow()
	:GLWindow(),mIsRotate(false),mPlay(false),mCapture(false)
{
	std::cout << "Simulation Window." << std::endl;
	mTotalFrame = 0;
	mCurFrame = 0;
	mDisplayTimeout = 33;
	mElapsedTime = 0.0;

	mSoftWorld = new FEM::World(
		FEM::IntegrationMethod::PROJECTIVE_DYNAMICS,	//Integration Method
		FEM::OptimizationMethod::OPTIMIZATION_METHOD_NEWTON,
		FEM::LinearSolveType::SOLVER_TYPE_LDLT,
		1.0/100.0,										//time_step
		100, 											//max_iteration	
		0.99											//damping_coeff
		);

	mCloth = new Cloth();
	mCloth->Initialize(mSoftWorld);
	mSoftWorld->Initialize();
}
void
SimulationWindow::
Display()
{	
	glClearColor(0.95, 0.95, 1, 1);
	glClearColor(1, 1, 1, 1);
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	glEnable(GL_DEPTH_TEST);
	// glEnable(GL_LINE_SMOOTH);
 	// glEnable(GL_POLYGON_SMOOTH);
	glDisable(GL_DEPTH_TEST);
	glEnable(GL_LIGHTING);  
	glCullFace(GL_BACK);
	glEnable(GL_CULL_FACE);
	mCamera->Apply();
	
	// glDisable(GL_LIGHTING);  
	// glColor3f(0,0,0);
	// glLineWidth(0.01);
	// glBegin(GL_LINES);
	// for(double x=-10.0;x<=10.0;x+=0.5)
	// {
	// 	glVertex3f(x,0.0,-10.0);
	// 	glVertex3f(x,0.0,10.0);
	// }
	// for(double z=-10.0;z<=10.0;z+=0.5)
	// {
	// 	glVertex3f(-10.0,0.0,z);
	// 	glVertex3f(10.0,0.0,z);
	// }
	// glEnd();
	// glEnable(GL_LIGHTING); 

	glColor3f(0,0,0);
    GUI::DrawStringOnScreen(0.8,0.2,std::to_string(mSoftWorld->GetTime()),
    	true,Eigen::Vector3d(0.0,0.0,0.0));
	
	const auto& particles = mSoftWorld->mX;
	const auto& springs = mCloth->GetMesh()->GetSprings();

	for(int i=0; i<particles.size()/3; i++) 
	{
		GUI::DrawPoint(particles.block<3,1>(3*i,0),3.0,Eigen::Vector3d(0,0,0));
	}

	GUI::DrawPoint(particles.block<3,1>(3*9,0),10.0,Eigen::Vector3d(1,0,0));

	for(const auto& s : springs) 
	{
		auto p0 = particles.block<3,1>(3*s[0],0);
		auto p1 = particles.block<3,1>(3*s[1],0);
		GUI::DrawLine(p0,p1);
	}

	if(mCapture) Screenshot();

	glEnable(GL_DEPTH_TEST);
	glutSwapBuffers();
}
void
SimulationWindow::
Keyboard(unsigned char key,int x,int y)
{
	switch(key)
	{	
		case ' ': {
			mPlay = !mPlay;
			if(mPlay == true)
				std::cout << "Play." << std::endl;
			else 
				std::cout << "Pause." << std::endl;
			break;
		}
		case 'C': {
			mCapture = !mCapture;
			if(mCapture) std::cout << "Capture!" << std::endl;
			break;
		}
		case '`': mIsRotate= !mIsRotate;break;
		case 27: exit(0);break;
		default : break;
	}
	
	glutPostRedisplay();
}
void
SimulationWindow::
Mouse(int button, int state, int x, int y)
{
	if (state == GLUT_DOWN)
	{
		mIsDrag = true;
		mMouseType = button;
		mPrevX = x;
		mPrevY = y;
	}
	else
	{
		mIsDrag = false;
		mMouseType = 0;
	}

	glutPostRedisplay();
}
void
SimulationWindow::
Motion(int x, int y)
{
	if (!mIsDrag)
		return;

	int mod = glutGetModifiers();

	if (mMouseType == GLUT_LEFT_BUTTON)
	{
		if(!mIsRotate) {
			mCamera->Translate(x,y,mPrevX,mPrevY);
		} else {
			mCamera->Rotate(x,y,mPrevX,mPrevY);
		}
	}
	else if (mMouseType == GLUT_RIGHT_BUTTON)
	{
		mCamera->Pan(x,y,mPrevX,mPrevY);
	}
	mPrevX = x;
	mPrevY = y;
	glutPostRedisplay();
}
void
SimulationWindow::
Reshape(int w, int h)
{
	glViewport(0, 0, w, h);
	mCamera->Apply();
	glutPostRedisplay();
}
void
SimulationWindow::
Timer(int value)
{
	if(mPlay) {
		// auto start_time = std::chrono::system_clock::now();
		mSoftWorld->TimeStepping();
		// auto end_time = std::chrono::system_clock::now();
		// std::chrono::duration<double> elapsed_seconds = end_time-start_time;
		// std::cout << "time: " << elapsed_seconds.count() << "s" <<std::endl;
	}
	glutTimerFunc(mDisplayTimeout, TimerEvent,1);
	glutPostRedisplay();
}
void 
SimulationWindow::
Screenshot() {
	// static int count = 0;
	// const char directory[8] = "frames";
	// const char fileBase[8] = "Capture";
	// char fileName[32];

	// boost::filesystem::create_directories(directory);
	// std::snprintf(fileName, sizeof(fileName), "%s%s%s%.4d.png",
	//             directory, "/", fileBase, count++);
	// int tw = glutGet(GLUT_WINDOW_WIDTH);
	// int th = glutGet(GLUT_WINDOW_HEIGHT);

	// glReadPixels(0, 0,  tw, th, GL_RGBA, GL_UNSIGNED_BYTE, &mScreenshotTemp[0]);
	// // reverse temp2 temp1
	// for (int row = 0; row < th; row++) {
	// memcpy(&mScreenshotTemp2[row * tw * 4],
	//        &mScreenshotTemp[(th - row - 1) * tw * 4], tw * 4);
	// }

	// unsigned result = lodepng::encode(fileName, mScreenshotTemp2, tw, th);

	// // if there's an error, display it
	// if (result) {
	// 	std::cout << "lodepng error " << result << ": "
	//           << lodepng_error_text(result) << std::endl;
	// 	return ;
	// } else {
	// 	return ;
	// }
}
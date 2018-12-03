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
#include <iostream>
int main(int argc,char** argv)
{
	omp_set_num_threads(16);
	if( argc < 2 ) {
		SimulationWindow* simwindow = new SimulationWindow();
		glutInit(&argc, argv);
		simwindow->InitWindow(1280,720,"Fast mass spring");
		glutMainLoop();
	}
}

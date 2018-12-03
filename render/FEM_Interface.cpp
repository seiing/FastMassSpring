#include "FEM_Interface.h"
#include "GL_Functions.h"
#include "../sim/fem/Constraint/ConstraintHeader.h"
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

using namespace FEM;
int i=0;
void
DrawConstraint(Constraint* c,const Eigen::VectorXd& x)
{
	glDisable(GL_LIGHTING);
	ConstraintType type = c->GetType();
	if(type == ConstraintType::ATTACHMENT)
	{
		auto* cc = static_cast<AttachmentConstraint*>(c);
		GUI::DrawLine(cc->GetP(),x.block<3,1>(cc->GetI0()*3,0),Eigen::Vector3d(1,0,0));
	}
	
	glEnable(GL_LIGHTING);
}
void
DrawActivation(const double& x, const double& y,
	const double& length,
	const std::vector<Eigen::VectorXd>& act,
	const int& cur_frame)
{
	glDisable(GL_LIGHTING);

	GLint oldMode;
    glGetIntegerv(GL_MATRIX_MODE, &oldMode);
    glMatrixMode(GL_PROJECTION);

    glPushMatrix();
    glLoadIdentity();
    gluOrtho2D(0.0, 1.0, 0.0, 1.0);

	glMatrixMode(GL_MODELVIEW);
    glPushMatrix();
    glLoadIdentity();
    glRasterPos2f(x, y);

	int start_frame;
	if(cur_frame-20 < 0) {
		start_frame = 0;
	} else {
		start_frame = cur_frame-20;
	}

	glBegin(GL_LINE_STRIP);
	glColor3f(0,0,0);
	glVertex2d(x,y);
	glVertex2d(x,y+0.1);
	glVertex2d(x+length,y+0.1);
	glVertex2d(x+length,y);
	glVertex2d(x,y);
	glEnd();

	for(int i=0; i<act[cur_frame].size()-1; i++) {
		float x1,y1;
		float x2,y2;

		x1=x+length/act[cur_frame].size()*i;
		x2=x+length/act[cur_frame].size()*(i+1);

		y1=y+act[cur_frame][i]*0.1;
		y2=y+act[cur_frame][i+1]*0.1;

		glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
		glBegin(GL_POLYGON);
		Eigen::Vector3d color(act[cur_frame][i+1],0.0,1.0-act[cur_frame][i+1]);
	    glColor3f(color[0],color[1],color[2]);
	    glVertex2d(x1,y);
	    glVertex2d(x1,y2);
	    glVertex2d(x2,y2);
	    glVertex2d(x2,y);
	    glVertex2d(x1,y);
	    glEnd();
	}

	// double alpha=0.5;
	// for(int i=cur_frame; i>start_frame; i--) {
	// 	for(int j=0; j<act[i].size()-1; j++) {
	// 		float x1,y1;
	// 		float x2,y2;
	// 		x1=x+length/act[i].size()*j;
	// 		x2=x+length/act[i].size()*(j+1);
	// 		y1=y+act[i][j]*0.1;
	// 		y2=y+act[i][j+1]*0.1;
	// 		glEnable(GL_BLEND);
	// 		glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA); 
	// 		glBegin(GL_LINES);
	// 		Eigen::Vector3d color(act[i][j+1],0.0,1.0-act[i][j+1]);
	// 	    glColor4f(color[0],color[1],color[2],alpha);
	// 	    glVertex2d(x1,y1);
	// 	    glVertex2d(x2,y2);
	// 	    glEnd();
	// 		glDisable(GL_BLEND);
	// 	}
	// 	alpha -= 0.1;
	// }

	glPopMatrix();
	glMatrixMode(GL_PROJECTION);
    glPopMatrix();
    glMatrixMode(oldMode);

    glEnable(GL_LIGHTING);
}
void
DrawActivation(const double& x, const double& y,
	const double& length,
	const Eigen::VectorXd& act)
{
	glDisable(GL_LIGHTING);

	GLint oldMode;
    glGetIntegerv(GL_MATRIX_MODE, &oldMode);
    glMatrixMode(GL_PROJECTION);

    glPushMatrix();
    glLoadIdentity();
    gluOrtho2D(0.0, 1.0, 0.0, 1.0);

	glMatrixMode(GL_MODELVIEW);
    glPushMatrix();
    glLoadIdentity();
    glRasterPos2f(x, y);

	glBegin(GL_LINE_STRIP);
	glColor3f(0,0,0);
	glVertex2d(x,y);
	glVertex2d(x,y+0.1);
	glVertex2d(x+length,y+0.1);
	glVertex2d(x+length,y);
	glVertex2d(x,y);
	glEnd();

	for(int i=0; i<act.size()-1; i++) {
		float x1,y1;
		float x2,y2;

		x1=x+length/act.size()*i;
		x2=x+length/act.size()*(i+1);

		y1=y+act[i]*0.1;
		y2=y+act[i+1]*0.1;

		Eigen::Vector3d color(act[i+1],0.0,1.0-act[i+1]);
		glColor3f(color[0],color[1],color[2]);
		glBegin(GL_POLYGON);
	    glVertex2d(x1,y);
	    glVertex2d(x1,y2);
	    glVertex2d(x2,y2);
	    glVertex2d(x2,y);
	    glVertex2d(x1,y);
	    glEnd();
	}

	// double alpha=0.5;
	// for(int i=cur_frame; i>start_frame; i--) {
	// 	for(int j=0; j<act[i].size()-1; j++) {
	// 		float x1,y1;
	// 		float x2,y2;
	// 		x1=x+length/act[i].size()*j;
	// 		x2=x+length/act[i].size()*(j+1);
	// 		y1=y+act[i][j]*0.1;
	// 		y2=y+act[i][j+1]*0.1;
	// 		glEnable(GL_BLEND);
	// 		glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA); 
	// 		glBegin(GL_LINES);
	// 		Eigen::Vector3d color(act[i][j+1],0.0,1.0-act[i][j+1]);
	// 	    glColor4f(color[0],color[1],color[2],alpha);
	// 	    glVertex2d(x1,y1);
	// 	    glVertex2d(x2,y2);
	// 	    glEnd();
	// 		glDisable(GL_BLEND);
	// 	}
	// 	alpha -= 0.1;
	// }

	glPopMatrix();
	glMatrixMode(GL_PROJECTION);
    glPopMatrix();
    glMatrixMode(oldMode);

    glEnable(GL_LIGHTING);
}
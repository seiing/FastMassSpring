#include "GL_Functions.h"
#ifdef __APPLE__
#include <OpenGL/gl.h>
#include <OpenGL/glu.h>
#include <GLUT/glut.h>
#else
#include <GL/gl.h>
#include <GL/glu.h>
#include <GL/glut.h>
#endif
void
GUI::
DrawTetrahedron(const Eigen::Vector3d& p0,const Eigen::Vector3d& p1,const Eigen::Vector3d& p2,const Eigen::Vector3d& p3,const Eigen::Vector3d& color)
{
    glDisable(GL_LIGHTING);
	DrawTriangle(p0,p1,p2,color);
	DrawTriangle(p0,p1,p3,color);
	DrawTriangle(p0,p2,p3,color);
	DrawTriangle(p1,p2,p3,color);
    glEnable(GL_LIGHTING);
}
void
GUI::
DrawTriangle(const Eigen::Vector3d& p0,const Eigen::Vector3d& p1,const Eigen::Vector3d& p2,const Eigen::Vector3d& color,const bool& draw_line)
{
    glDisable(GL_LIGHTING);
	glColor3f(color[0],color[1],color[2]);
    auto normal = ((p1-p0).cross(p2-p0)).normalized();

    if(draw_line) {
        glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
    } else {
        glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
    }
    
	glBegin(GL_TRIANGLES);
    glNormal3d(normal[0],normal[1],normal[2]);
	glVertex3f(p0[0],p0[1],p0[2]);
	glVertex3f(p1[0],p1[1],p1[2]);
	glVertex3f(p2[0],p2[1],p2[2]);
	glEnd();
    glEnable(GL_LIGHTING);
}
void
GUI::
DrawLine(const Eigen::Vector3d& p0,const Eigen::Vector3d& p1,const Eigen::Vector3d& color)
{
    glDisable(GL_LIGHTING);
	glColor3f(color[0],color[1],color[2]);
	glBegin(GL_LINES);
	glVertex3f(p0[0],p0[1],p0[2]);
	glVertex3f(p1[0],p1[1],p1[2]);
	glEnd();
    glEnable(GL_LIGHTING);
}
void
GUI::
DrawPoint(const Eigen::Vector3d& p0,const double& size,const Eigen::Vector3d& color)
{
    glDisable(GL_LIGHTING);
    glPointSize(size);
	glBegin(GL_POINTS);
    glColor3f(color[0],color[1],color[2]);
	glVertex3f(p0[0],p0[1],p0[2]);
	glEnd();
    glEnable(GL_LIGHTING);
}
void
GUI::
DrawStringOnScreen(float _x, float _y, const std::string& _s,bool _bigFont,const Eigen::Vector3d& color)
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
    glColor3f(color[0],color[1],color[2]);
    glRasterPos2f(_x, _y);
    unsigned int length = _s.length();
    for (unsigned int c = 0; c < length; c++) {
    if (_bigFont)
      glutBitmapCharacter(GLUT_BITMAP_HELVETICA_18, _s.at(c) );
    else
      glutBitmapCharacter(GLUT_BITMAP_HELVETICA_10, _s.at(c) );
    }  
    glPopMatrix();

    glMatrixMode(GL_PROJECTION);
    glPopMatrix();
    glMatrixMode(oldMode);
    glEnable(GL_LIGHTING);
}
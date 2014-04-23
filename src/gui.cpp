#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Header.h>
#include <tf/transform_broadcaster.h>
#include <kdl/frames.hpp>
#include <GL/glut.h>
#include <string>
#define MESSAGE   "Underworld live 17.03.04"

void output(int x, int y, float r, float g, float b, void* font, char *string)
{
  glColor3f( r, g, b );
  glRasterPos2f(x, y);
  int len, i;
  len = (int)strlen(string);
  for (i = 0; i < len; i++) {
    glutBitmapCharacter(font, string[i]);
  }
}
void output2(GLfloat x, GLfloat y, char *text)
{
 char *p;

 glPushMatrix();
 glTranslatef(x, y, 0);
 for (p = text; *p; p++)
   glutStrokeCharacter(GLUT_STROKE_ROMAN, *p);
 glPopMatrix();
}
void glutDisplay(){
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
  glEnable(GL_DEPTH_TEST);

  glMatrixMode(GL_PROJECTION);
  glLoadIdentity();
  glOrtho(-2.0, 2.0, -2.0, 2.0, -2.0, 500.0);

  glMatrixMode(GL_MODELVIEW);
  glLoadIdentity();
  // gluLookAt(2, 2, 2, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0);
  glScalef(.007,.007,.007);
  glRotatef(0, 0, 1, 0);
  glRotatef(0, 0, 0, 1);
  glRotatef(0, 1, 0, 0);
  glTranslatef(-300, 0, 0);
    
  glColor3f(1,1,1);
  glutStrokeCharacter(GLUT_STROKE_ROMAN, 'H');
  glutStrokeCharacter(GLUT_STROKE_ROMAN, 'e');
  glutStrokeCharacter(GLUT_STROKE_ROMAN, 'l');
  glutStrokeCharacter(GLUT_STROKE_ROMAN, 'l');
  glutStrokeCharacter(GLUT_STROKE_ROMAN, 'o');
  
  glutStrokeCharacter(GLUT_STROKE_ROMAN, 'W');
  glutStrokeCharacter(GLUT_STROKE_ROMAN, 'o');
  glutStrokeCharacter(GLUT_STROKE_ROMAN, 'r');
  glutStrokeCharacter(GLUT_STROKE_ROMAN, 'l');
  glutStrokeCharacter(GLUT_STROKE_ROMAN, 'd');
  glutStrokeCharacter(GLUT_STROKE_ROMAN, '!');
        
  glutSwapBuffers();

   //  glClear(GL_COLOR_BUFFER_BIT);
   //  glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
   // glEnable(GL_BLEND);
   // glEnable(GL_LINE_SMOOTH);
   // glLineWidth(10.0);
   // output2(100, 100, "This is antialiased.");


    // glClear(GL_COLOR_BUFFER_BIT);
    // //要显示的字符
    // char *str = "current fps = ";
    // int n = strlen(str);
    // //设置要在屏幕上显示字符的起始位置
    // glRasterPos2i(0,0);
    // //逐个显示字符串中的每个字符
    // for (int i = 0; i < n; i++)
    //     glutBitmapCharacter(GLUT_BITMAP_HELVETICA_18, *(str+i));
  

  // glClear(GL_COLOR_BUFFER_BIT);
  // glBegin(GL_TRIANGLES);
  // glVertex3f(-0.5,-0.5,0.0);
  // glVertex3f(0.5,0.0,0.0);
  // glVertex3f(0.0,0.5,0.0);
  // glEnd();
  // glFlush();
}
int main(int argc, char** argv){
   glutInitDisplayMode(GLUT_DEPTH | GLUT_SINGLE | GLUT_RGBA);
   glutInitWindowPosition( 0, 0 );
   glutInitWindowSize( 1000, 1050 );
   //glClearColor(1.0, 1.0, 1.0, 1.0);
   glutInit( &argc, argv );
   glutCreateWindow("GLUT bitmap font example");
   glutDisplayFunc(glutDisplay);
   glutMainLoop();
   return 0;
}

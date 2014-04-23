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
void glutDisplay(){
  glClear(GL_COLOR_BUFFER_BIT);
    //要显示的字符
    char *str = "current fps = ";
    int n = strlen(str);
    //设置要在屏幕上显示字符的起始位置
    glRasterPos2i(0,0);
    //逐个显示字符串中的每个字符
    for (int i = 0; i < n; i++)
        glutBitmapCharacter(GLUT_BITMAP_HELVETICA_18, *(str+i));
  // glClear(GL_COLOR_BUFFER_BIT);
  // glBegin(GL_TRIANGLES);
  // glVertex3f(-0.5,-0.5,0.0);
  // glVertex3f(0.5,0.0,0.0);
  // glVertex3f(0.0,0.5,0.0);
  // glEnd();
  glFlush();
}
int main(int argc, char** argv){
   glutInitDisplayMode(GLUT_DEPTH | GLUT_SINGLE | GLUT_RGBA);
   glutInitWindowPosition( 0, 0 );
   glutInitWindowSize( 500, 550 );
   //glClearColor(1.0, 1.0, 1.0, 1.0);
   glutInit( &argc, argv );
   glutCreateWindow("GLUT bitmap font example");
   glutDisplayFunc(glutDisplay);
   glutMainLoop();
   return 0;
}

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Header.h>
#include <tf/transform_broadcaster.h>
#include <kdl/frames.hpp>
#include <GL/glut.h>
#include <string>
#define MESSAGE   "Underworld live 17.03.04"

// class gui_message{
// private:
//   ros::Subscriber gui_sub;
// public:
//   std::string gui_msg = "";
//   gui_message(){

//   }
//   void init(){
//     ros::NodeHandle nh;
//     gui_sub = nh.subscribe("gui",1,gui_callback);
//   }
//   void gui_callback(const std_msgs::String::ConstPtr& msg){
//     gui_msg = msg->data;
//   }
//   std::string getMsg(){
//     ros::spinOnce();
//   }
// };
ros::Subscriber gui_sub;
std::string gui_msg = "Hello There";
std::string gui_mode_msg = "Mode:Manual";
std::string gui_gest_msg = "Gest:I stop tracking you";
std::string gui_logo_msg_1 = "Logo:Georgia";
std::string gui_logo_msg_2 = "Logo:Tech";
char h = 0;
void gui_callback(const std_msgs::String::ConstPtr& msg){
  std::string msg_type = msg->data.substr(0,4);
  printf("mode %s\n",msg_type.c_str());
  printf("new message %s\n",msg->data.substr(5).c_str());
  if(strcmp(msg_type.c_str(),"Mode") == 0){
  	gui_mode_msg = msg->data;
  }
  else if(strcmp(msg_type.c_str(),"Gest") == 0){
  	gui_gest_msg = msg->data;
  }
}
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

void strokeString(GLfloat x, GLfloat y, const char *str){
	
	if(strcmp(str,"I Stop Tracking You") == 0){
		glColor3f(1.0f,0.0f,0.0f);
	}else{
		//glColor3f(1.0f,0.0f,0.0f);
	}
	glTranslatef(x,y,0);
  	int i = 0;
  	for(i = 0; i < (int)strlen(str); i++){
  	  glutStrokeCharacter(GLUT_STROKE_ROMAN, str[i]);
  	 // if(i%10 == 0 && i > 0){
  	 //   glTranslatef(-840,-150,0);
  	 // }
  	}
  
}
void glutIdle(){
  ros::spinOnce();
  glutPostRedisplay();
}
void glutDisplay(){
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
  glClearColor(0.1f,0.23f,0.367f,0);
  //glClearColor(1.0f,1.0f,1.0f,0);

  glEnable(GL_DEPTH_TEST);

  glMatrixMode(GL_PROJECTION);
  glLoadIdentity();
  glOrtho(-2.0, 2.0, -2.0, 2.0, -2.0, 500.0);
  glMatrixMode(GL_MODELVIEW);
  glLoadIdentity();
  // gluLookAt(2, 2, 2, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0);
  glScalef(.001,.001,.001);
  // glRotatef(0, 0, 1, 0);
  // glRotatef(0, 0, 0, 1);
  // glRotatef(0, 1, 0, 0);
  // glTranslatef(-450, 200, 0);
  glLineWidth(2.0);
  //glColor3f(0.7,0.7,0.7);
  glColor3f(1.0f,1.0f,1.0f);
  strokeString(-1800.0f,1500.0f,gui_mode_msg.c_str());

  glLoadIdentity();
  glScalef(.002,.002,.002);
  glLineWidth(3.0);
  strokeString(-900.0f,0.0f,gui_gest_msg.substr(5).c_str());

  glLoadIdentity();
  glScalef(.001,.001,.001);
  glColor3f(0.93f,0.695f,0.066f);
  glLineWidth(2.0);
  strokeString(1300.0f,1500.0f,gui_logo_msg_1.substr(5).c_str());

  glLoadIdentity();
  glColor3f(0.93f,0.695f,0.066f);
  glScalef(.001,.001,.001);
  glLineWidth(2.0);
  strokeString(1470.0f,1350.0f,gui_logo_msg_2.substr(5).c_str());

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
    ros::init(argc, argv, "gui");
    ros::NodeHandle nh;
    gui_sub = nh.subscribe("gui",1,gui_callback);
    

   glutInitDisplayMode(GLUT_DEPTH | GLUT_SINGLE | GLUT_RGBA);
   glutInitWindowPosition( 0, 0 );
   glutInitWindowSize( 1280, 500);
   //glClearColor(1.0, 1.0, 1.0, 1.0);
   glutInit( &argc, argv );
   glutCreateWindow("GLUT bitmap font example");
   glutDisplayFunc(glutDisplay);
   glutIdleFunc(glutIdle);
   glutMainLoop();

   // ros::Rate loop_rate(60);
   //  while(ros::ok()){
   //    ros::spinOnce();
   //    loop_rate.sleep();
   //  }
   return 0;
}

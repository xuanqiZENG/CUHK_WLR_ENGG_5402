#include "WLR_controller.h"
#include "main.h"


kin_xy body1_position;
invkin_q joint1_angel;

kin_xy body2_position;
invkin_q joint2_angel;

float pi=3.1415926;
float l=0.14;
float offset_q2=6.239;//6.2832;
float offset_q1=5.32;//5.1508;
float offset2_q1=0.9492;
float q1_1_r;
float q2_1_r;
float q1_2_r;
float q2_2_r;
r_p_y rpy;
float x1;
float y1;
float x2;
float y2;

float x;
float y;
float z;
float w;
float as;


invkin_q invkin(float x1, float y1)
{
	float A1;
	if (x1>0.08)
	{
		x1=0.08;
	}
	else if (x1<-0.08)
	{
		x1=-0.08;
	}
	if (y1>0.278)
	{
		y1=0.278;
	}
	else if (y1<0.05)
	{
		y1=0.05;
	}
	A1=acos((l*l+l*l-x1*x1-y1*y1)/(2*l*l));
	joint1_angel.q2=offset_q2-pi+A1;
	joint1_angel.q1=-(A1/2+atan2(-x1,y1))+offset_q1;
	return joint1_angel;
}

invkin_q invkin_2(float x2, float y2)
{
	float A2;
	if (x2>0.08)
	{
		x2=0.08;
	}
	else if (x2<-0.08)
	{
		x2=-0.08;
	}
	if (y2>0.278)
	{
		y2=0.278;
	}
	else if (y2<0.05)
	{
		y2=0.05;
	}
	A2=acos((l*l+l*l-x2*x2-y2*y2)/(2*l*l));
	joint2_angel.q2=pi-A2;
	joint2_angel.q1=(A2/2+atan2(-x2,y2))+offset2_q1;
	return joint2_angel;
}

kin_xy kinematic(float q1_1, float q2_1)
{
	q1_1_r=-q1_1+offset_q1;
	q2_1_r=offset_q2-q2_1;
	body1_position.x=-(-l*cos(q1_1_r)+l*cos(pi-q2_1_r-q1_1_r));
	body1_position.y=l*sin(q1_1_r)+l*sin(pi-q2_1_r-q1_1_r);
	return body1_position;
}

kin_xy kinematic_2(float q1_2, float q2_2)
{
	q1_2_r=q1_2-offset2_q1;
	q2_2_r=q2_2;
	body2_position.x=(l*cos(q1_2_r)-l*cos(pi-q2_2_r-q1_2_r));
	body2_position.y=l*sin(q1_2_r)+l*sin(pi-q2_2_r-q1_2_r);
	return body2_position;
}

r_p_y q2rpy(float x,float y,float z, float w)
{
	as = fmin(-2. * (y * w - x * z), .99999);
  rpy.yaw =atan2(2 * (y * z + x *w),x*x + y*y - z*z - w*w);
  rpy.pitch = asinf(as);
  rpy.roll =atan2(2 * (z*w + x* y),x*x - y*y - z*z + w*w);
	return rpy;
	
}
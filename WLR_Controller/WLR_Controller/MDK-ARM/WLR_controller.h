
#include "math.h"
#include "main.h"



typedef struct invkin_q
{
	float q1;
	float q2;
	
}invkin_q;

typedef struct kin_xy
{
	float x;
	float y;
	
}kin_xy;

typedef struct r_p_y
{
	float roll;
	float pitch;
	float yaw;
	
}r_p_y;


r_p_y q2rpy(float x,float y,float z, float w);
kin_xy kinematic(float q1_1, float q2_1);
invkin_q invkin(float x1, float y1);
invkin_q invkin_2(float x2, float y2);
kin_xy kinematic_2(float q1_2, float q2_2);
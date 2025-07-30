#include "stm32h7xx.h"
#include "PCA9685.h"
#include "five_rod.h"

float x;
float y;
float lamda=57.297;
float alpha_count_a;
float alpha_count_b;
float beta_count_a;
float beta_count_b;
float left_a=0;
float left_b=0;
float right_a=0;
float right_b=0;
float Count_angle_alpha(float alpha_count){
	float alpha;
    alpha = 2*tan(alpha_count);
	return alpha;
}
float Count_angle_beta(float beta_count){
	float beta;
    beta = 2*tan(beta_count);
	return beta;
}
float Count_pack_alpha(float horizon,float height){
	x=horizon;
	y=height;
	
    float a=2*x*L1;
    float b=2*y*L1;         
    float c=x*x+y*y+L1*L1-L2*L2;
	
	alpha_count_a=Count_angle_alpha((b+sqrt(a*a+b*b-c*c))/(a+c))*lamda;
    alpha_count_b=Count_angle_alpha((b-sqrt(a*a+b*b-c*c))/(a+c))*lamda;
	if(alpha_count_a >= -90.0f && alpha_count_a <=270.0f)return alpha_count_a;
    if(alpha_count_b >= -90.0f && alpha_count_b <=270.0f)return alpha_count_b;
	return 90.0;
}
float Count_pack_beta(float horizon,float height){
	x=horizon;
	y=height;
	
    float d=2*(x-L5)*L4;
    float e=2*y*L4;          
    float f=(x-L5)*(x-L5)+L4*L4+y*y-L3*L3;
	
	beta_count_a=Count_angle_beta((e+sqrt(d*d+e*e-f*f))/(d+f))*lamda;
    beta_count_b=Count_angle_beta((e-sqrt(d*d+e*e-f*f))/(d+f))*lamda;
	
	if(beta_count_a >= -90.0f && beta_count_a <=90.0f)return beta_count_a;
    if(beta_count_b >= -90.0f && beta_count_b <=90.0f)return beta_count_b;
	
	return 90.0;
}
void Servos_left(float horizon,float height){

    left_a=Count_pack_alpha(horizon,height);
    if(left_a<0)SetAngle(0,-left_a-90);
    else SetAngle(0,270+3-left_a);
     
    //left_b=Count_pack_beta(horizon,height);
    //SetAngle(15,90+16-left_b);
   
}

void Servos_right(float horizon,float height){

    right_a=Count_pack_alpha(horizon,height);
    if(right_a<0)SetAngle(0,-right_a);
    else SetAngle(0,right_a);
     
    right_b=Count_pack_beta(horizon,height);
    SetAngle(1,right_b);

}

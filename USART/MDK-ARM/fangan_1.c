#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include "main.h"
#include "memorymap.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"


#define M_PI 3.1415926535f
#define HORIZONTAL_MOTOR_ADDR 5  // 水平电机(底座)
#define VERTICAL_MOTOR_ADDR   4  // 垂直电机
#define MAX_POINTS 100

typedef struct {
    float h_angle;
    float v_angle;
} PathPoint;

PathPoint path_points[MAX_POINTS];
uint16_t path_count = 0;
float current_h_angle = 0.0f;  // 水平角度(度)
float current_v_angle = 0.0f;  // 垂直角度(度)

 void Emm_V5_Pos_Control(uint8_t addr, uint8_t dir, uint16_t vel, uint8_t acc, uint32_t clk, bool raF, bool snF)
{ 
  uint8_t cmd[13] = {0};

  cmd[0]  =  addr;                      
  cmd[1]  =  0xFD;                      
  cmd[2]  =  dir;                       
  cmd[3]  =  (uint8_t)(vel >> 8);       
  cmd[4]  =  (uint8_t)(vel >> 0);        
  cmd[5]  =  acc;                       
  cmd[6]  =  (uint8_t)(clk >> 24);      
  cmd[7]  =  (uint8_t)(clk >> 16);      
  cmd[8]  =  (uint8_t)(clk >> 8);      
  cmd[9]  =  (uint8_t)(clk >> 0);       
  cmd[10] =  raF;                      
  cmd[11] =  snF;                      
  cmd[12] =  0x6B;                     
  
  HAL_UART_Transmit(&huart2,cmd,13,100);
}

void Emm_V5_Synchronous_motion(uint8_t addr)
{
  uint8_t cmd[16] = {0};
  
  cmd[0] =  addr;                       
  cmd[1] =  0xFF;                       
  cmd[2] =  0x66;                       
  cmd[3] =  0x6B;                       
  
  HAL_UART_Transmit(&huart2,cmd,4,1000);
}
void move_to_angle1(float h_angle, float v_angle, uint16_t vel)
{
	float delta_h = h_angle - current_h_angle;
    float delta_v = v_angle - current_v_angle;
	
	uint32_t steps_h = (uint32_t)(fabsf(delta_h) * 142.86f);
    uint32_t steps_v = (uint32_t)(fabsf(delta_v) * 142.86f);
    
	uint8_t dir_h = (delta_h >= 0) ? 1 : 0; // 1=正方向, 0=负方向
    uint8_t dir_v = (delta_v >= 0) ? 1 : 0; // 1=向上, 0=向下
    
	 // 如果两个轴都需要运动，发送同步指令
    if (steps_h > 0 && steps_v > 0) {
        Emm_V5_Pos_Control(HORIZONTAL_MOTOR_ADDR, dir_h, vel, 1, steps_h, 0, 1);
		HAL_Delay(5);
        Emm_V5_Pos_Control(VERTICAL_MOTOR_ADDR, dir_v, vel, 1, steps_v, 0, 1);
	    HAL_Delay(5);
        Emm_V5_Synchronous_motion(0);
    } 
    // 只有水平轴运动
    else if (steps_h > 0) {
        Emm_V5_Pos_Control(HORIZONTAL_MOTOR_ADDR, dir_h, vel, 1, steps_h, 0, false);
    }
    // 只有垂直轴运动
    else if (steps_v > 0) {
        Emm_V5_Pos_Control(VERTICAL_MOTOR_ADDR, dir_v, vel, 1, steps_v, 0, false);
    }
	
	current_h_angle = h_angle;
    current_v_angle = v_angle;
	
	 /*uint32_t max_steps = (steps_h > steps_v) ? steps_h : steps_v;
    if (max_steps > 0) {
        uint32_t time_ms = (max_steps * 1000) / vel + 200; // 增加200ms裕量*/
       HAL_Delay(50);
    
}
void draw_line_in_angle(float h0, float v0, float h1, float v1, uint16_t vel, uint16_t segments)
{
	
        path_points[path_count].h_angle = h0;
        path_points[path_count].v_angle = v0;
        path_count++;
   
	
    // 线性插值
    for (uint16_t i = 1; i <= segments; i++) {
        float ratio = (float)i / (float)segments;
        float h = h0 + ratio * (h1 - h0);
        float v = v0 + ratio * (v1 - v0);
      
            path_points[path_count].h_angle = h;
            path_points[path_count].v_angle = v;
            path_count++;
        
    }
}

void clear_path(void)
{
    path_count = 0;
}

void execute_path(uint16_t vel)
{
    for (uint16_t i = 0; i < path_count; i++) {
        move_to_angle1(path_points[i].h_angle, path_points[i].v_angle, vel);
    }
}

void draw_square_in_angle(float size_h, float size_v, uint16_t vel)
{
    // 计算半尺寸
    float half_h = size_h / 2.0f;
    float half_v = size_v / 2.0f;
    
    // 清空路径
    clear_path();
    
    // 创建正方形路径 (顺时针)
    draw_line_in_angle( 0.0f,  0.0f, size_h, 0.0f, vel, 16);  // 上边
    draw_line_in_angle( size_h,  0.0f, size_h, size_v, vel, 16);  // 左边
    draw_line_in_angle( size_h, size_v,  0.0f, size_v, vel, 16);  // 下边
    draw_line_in_angle( 0.0f, size_v, 0.0f,  0.0f, vel, 16);  // 右边
    
    // 执行路径
    execute_path(vel);
}

void draw_triangle_in_angle(float size_h, float size_v, uint16_t vel)
{
	
	float left_h = -size_h / 2.0f;
	float right_h = size_h / 2.0f;
	float up_v = size_v / 2.0f;
	float down_v = size_v /2.0f;
    
    // 清空路径
    clear_path();
    
    // 创建三角形路径
    draw_line_in_angle(0.0f, 0.0f, left_h, 0.0f, vel, 60);      // 左上边
    draw_line_in_angle(left_h, 0.0f,0.0f, size_v, vel, 60);  // 左底边
    draw_line_in_angle(0.0f, size_v,right_h , 0.0f, vel, 60);    // 右上边
	draw_line_in_angle(right_h, 0.0f, 0.0f , 0.0f, vel, 60);//右底边
    
	
	
    // 执行路径
    execute_path(vel);
}

void draw_circle_in_angle(float radius_h, float radius_v, uint16_t segments, uint16_t vel)
{
    // 限制分段数
    if (segments < 16) segments = 16;
    if (segments > 120) segments = 120;
    
    // 清空路径
    clear_path();
    
    // 创建椭圆路径
    for (uint16_t i = 0; i <= segments; i++) {
        float angle = 2.0f * M_PI * i / segments;
        float h = radius_h * cosf(angle);
        float v = radius_v * sinf(angle);
        
        // 添加点到路径
        if (path_count < MAX_POINTS) {
            path_points[path_count].h_angle = h;
            path_points[path_count].v_angle = v;
            path_count++;
        }
    }
    
    // 执行路径
    execute_path(vel);
}


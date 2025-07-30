#include <math.h>
#include <stdbool.h>
#include <stdint.h>

// 常量定义
#define HORIZONTAL_MOTOR_ADDR 0x04
#define VERTICAL_MOTOR_ADDR   0x05
#define MAX_POINTS 500
#define STEPS_PER_DEGREE 142.86f
#define M_PI 3.1415926535f
// 全局状态
typedef struct {
    float h_angle;
    float v_angle;
} PathPoint;

PathPoint path_points[MAX_POINTS];
uint16_t path_count = 0;
float current_h_angle = 0.0f;  // 当前位置（角度）
float current_v_angle = 0.0f;

// 电机控制函数声明 (需根据实际SDK实现)
void Emm_V5_Pos_Control(uint8_t addr, uint8_t dir, uint16_t vel, uint8_t mode, uint32_t steps, uint8_t opt, uint8_t sync);
void Emm_V5_Synchronous_motion(uint8_t enable);
void HAL_Delay(uint32_t ms);

// 绝对位置移动函数
void move_to_angle_absolute(float h_target, float v_target, uint16_t vel) {
    // 计算目标位置对应的步数
    uint32_t target_h_steps = (uint32_t)(h_target * STEPS_PER_DEGREE);
    uint32_t target_v_steps = (uint32_t)(v_target * STEPS_PER_DEGREE);
    
    // 确定运动方向（驱动器内部可能需要，但绝对位置模式下方向由位置决定）
    uint8_t dir_h = (h_target >= current_h_angle) ? 1 : 0;
    uint8_t dir_v = (v_target >= current_v_angle) ? 1 : 0;
    
    // 双轴运动（使用绝对位置模式，mode=0）
    if (fabsf(h_target - current_h_angle) > 0.01f && 
        fabsf(v_target - current_v_angle) > 0.01f) {
        
        Emm_V5_Pos_Control(HORIZONTAL_MOTOR_ADDR, dir_h, vel, 0, target_h_steps, 0, 1);
        HAL_Delay(5);
        Emm_V5_Pos_Control(VERTICAL_MOTOR_ADDR, dir_v, vel, 0, target_v_steps, 0, 1);
        HAL_Delay(5);
        Emm_V5_Synchronous_motion(1);  // 启用同步运动
    } 
    // 仅水平轴运动
    else if (fabsf(h_target - current_h_angle) > 0.01f) {
        Emm_V5_Pos_Control(HORIZONTAL_MOTOR_ADDR, dir_h, vel, 0, target_h_steps, 0, 0);
    } 
    // 仅垂直轴运动
    else if (fabsf(v_target - current_v_angle) > 0.01f) {
        Emm_V5_Pos_Control(VERTICAL_MOTOR_ADDR, dir_v, vel, 0, target_v_steps, 0, 0);
    }
    
    // 更新当前位置
    current_h_angle = h_target;
    current_v_angle = v_target;
    
    HAL_Delay(50);  // 运动稳定时间
}

// 路径插值（绝对坐标）
void draw_line_absolute(float h0, float v0, float h1, float v1, uint16_t vel, uint16_t segments) {
    path_points[path_count].h_angle = h0;
    path_points[path_count].v_angle = v0;
    path_count++;
    
    for (uint16_t i = 1; i <= segments; i++) {
        float ratio = (float)i / (float)segments;
        float h = h0 + ratio * (h1 - h0);
        float v = v0 + ratio * (v1 - v0);
        
        if (path_count < MAX_POINTS) {
            path_points[path_count].h_angle = h;
            path_points[path_count].v_angle = v;
            path_count++;
        }
    }
}

void clear_path1(void) {
    path_count = 0;
}

void execute_path1(uint16_t vel) {
    for (uint16_t i = 0; i < path_count; i++) {
        move_to_angle_absolute(
            path_points[i].h_angle, 
            path_points[i].v_angle, 
            vel
        );
    }
}

// 图形绘制函数（使用绝对坐标）
void draw_square_absolute(float center_h, float center_v, float size, uint16_t vel) {
    float half_size = size / 2.0f;
    
    clear_path1();
    
    // 定义正方形的四个角（绝对坐标）
    float points[5][2] = {
        {center_h - half_size, center_v - half_size}, // 左下
        {center_h + half_size, center_v - half_size}, // 右下
        {center_h + half_size, center_v + half_size}, // 右上
        {center_h - half_size, center_v + half_size}, // 左上
        {center_h - half_size, center_v - half_size}  // 闭合
    };
    
    // 创建路径
    for (uint8_t i = 0; i < 4; i++) {
        draw_line_absolute(
            points[i][0], points[i][1],
            points[i+1][0], points[i+1][1],
            vel, 20
        );
    }
    
    execute_path1(vel);
}

void draw_triangle_absolute(float center_h, float center_v, float size, uint16_t vel) {
    float height = size * 0.866f;  // 等边三角形高度
    
    clear_path1();
    
    // 定义三角形的三个顶点（绝对坐标）
    float points[4][2] = {
        {center_h, center_v + height/2},       // 上顶点
        {center_h - size/2, center_v - height/2}, // 左下
        {center_h + size/2, center_v - height/2}, // 右下
        {center_h, center_v + height/2}        // 闭合
    };
    
    // 创建路径
    for (uint8_t i = 0; i < 3; i++) {
        draw_line_absolute(
            points[i][0], points[i][1],
            points[i+1][0], points[i+1][1],
            vel, 20
        );
    }
    
    execute_path1(vel);
}

void draw_circle_absolute(float center_h, float center_v, float radius, uint16_t segments, uint16_t vel) {
    if (segments < 16) segments = 16;
    if (segments > 120) segments = 120;
    
    clear_path1();
    
    // 生成圆形路径（绝对坐标）
    for (uint16_t i = 0; i <= segments; i++) {
        float angle = 2.0f * M_PI * i / segments;
        float h = center_h + radius * cosf(angle);
        float v = center_v + radius * sinf(angle);
        
        if (path_count < MAX_POINTS) {
            path_points[path_count].h_angle = h;
            path_points[path_count].v_angle = v;
            path_count++;
        }
    }
    
    // 闭合圆形
    if (path_count < MAX_POINTS) {
        path_points[path_count].h_angle = path_points[0].h_angle;
        path_points[path_count].v_angle = path_points[0].v_angle;
        path_count++;
    }
    
    execute_path1(vel);
}

// 归零函数（执行一次确保绝对坐标基准）
void homing_sequence(uint16_t vel) {
    // 实际实现需根据您的硬件确定
    // 这里假设有归零传感器
    
    // 示例：水平归零
    Emm_V5_Pos_Control(HORIZONTAL_MOTOR_ADDR, 0, vel, 1, 142, 0, 0); // 向左移动直到限位
    // 等待归零完成...
    current_h_angle = 0.0f;
    
    // 示例：垂直归零
    Emm_V5_Pos_Control(VERTICAL_MOTOR_ADDR, 0, vel, 1, 142, 0, 0); // 向下移动直到限位
    // 等待归零完成...
    current_v_angle = 0.0f;
}

// 示例主程序
int main(void) {
    // 初始化
    homing_sequence(1000);  // 低速归零
    
    while (1) {
        // 在工作区中心绘制图形
        draw_square_absolute(45.0f, 45.0f, 30.0f, 1500);
        HAL_Delay(1000);
        
        draw_triangle_absolute(45.0f, 45.0f, 30.0f, 1500);
        HAL_Delay(1000);
        
        draw_circle_absolute(45.0f, 45.0f, 15.0f, 60, 1500);
        HAL_Delay(1000);
    }
}
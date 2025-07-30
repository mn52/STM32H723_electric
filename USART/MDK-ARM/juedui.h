#ifndef MOTOR_PATH_CONTROL_H
#define MOTOR_PATH_CONTROL_H

#include <stdint.h>
#include <stdbool.h>

// ==================== 常量定义 ====================
#define HORIZONTAL_MOTOR_ADDR   0x04        // 水平轴电机地址
#define VERTICAL_MOTOR_ADDR     0x05        // 垂直轴电机地址
#define MAX_POINTS              500         // 路径最大点数
#define STEPS_PER_DEGREE        142.86f     // 每度对应的步数（根据你的电机细分计算）
#define M_PI                    3.1415926535f  // 圆周率

// ==================== 数据结构 ====================
/**
 * @brief 路径点结构体：表示一个二维角度位置
 */
typedef struct {
    float h_angle;  // 水平角度
    float v_angle;  // 垂直角度
} PathPoint;

// ==================== 外部全局变量声明 ====================
/**
 * @brief 外部声明路径点数组和计数器（定义在 .c 文件中）
 */
extern PathPoint path_points[MAX_POINTS];
extern uint16_t path_count;
extern float current_h_angle;  // 当前水平角度
extern float current_v_angle;  // 当前垂直角度

// ==================== 函数声明 ====================

/**
 * @brief 移动到指定的绝对角度位置
 * @param h_target 目标水平角度
 * @param v_target 目标垂直角度
 * @param vel      速度（脉冲/秒）
 */
void move_to_angle_absolute(float h_target, float v_target, uint16_t vel);

/**
 * @brief 绘制一条直线路径（线性插值）
 * @param h0       起点水平角度
 * @param v0       起点垂直角度
 * @param h1       终点水平角度
 * @param v1       终点垂直角度
 * @param vel      运动速度
 * @param segments 插值段数
 */
void draw_line_absolute(float h0, float v0, float h1, float v1, uint16_t vel, uint16_t segments);

/**
 * @brief 清空当前路径
 */
void clear_path(void);

/**
 * @brief 执行已规划的路径
 * @param vel 速度
 */
void execute_path(uint16_t vel);

/**
 * @brief 绘制正方形（绝对坐标）
 * @param center_h 中心水平角度
 * @param center_v 中心垂直角度
 * @param size     正方形边长（角度单位）
 * @param vel      速度
 */
void draw_square_absolute(float center_h, float center_v, float size, uint16_t vel);

/**
 * @brief 绘制等边三角形（绝对坐标）
 * @param center_h 中心水平角度
 * @param center_v 中心垂直角度
 * @param size     三角形边长
 * @param vel      速度
 */
void draw_triangle_absolute(float center_h, float center_v, float size, uint16_t vel);

/**
 * @brief 绘制圆形路径
 * @param center_h 中心水平角度
 * @param center_v 中心垂直角度
 * @param radius   圆半径（角度单位）
 * @param segments 圆周插值段数（建议16~120）
 * @param vel      速度
 */
void draw_circle_absolute(float center_h, float center_v, float radius, uint16_t segments, uint16_t vel);

/**
 * @brief 回零序列（建立绝对坐标基准）
 * @param vel 回零速度
 */
void homing_sequence(uint16_t vel);

// ==================== 底层驱动函数声明（需外部实现）====================
/**
 * @brief 电机位置控制命令（根据你的协议）
 * @param addr  电机地址
 * @param dir   方向
 * @param vel   速度
 * @param mode  模式（0=绝对位置，1=相对等）
 * @param steps 步数
 * @param opt   保留选项
 * @param sync  同步标志（1=加入同步组，0=立即执行）
 */
void Emm_V5_Pos_Control(uint8_t addr, uint8_t dir, uint16_t vel, uint8_t mode, uint32_t steps, uint8_t opt, uint8_t sync);

/**
 * @brief 触发同步运动
 * @param enable 1=执行同步运动，0=无操作
 */
void Emm_V5_Synchronous_motion(uint8_t enable);

/**
 * @brief 延时函数（HAL库）
 * @param ms 延时毫秒数
 */
void HAL_Delay(uint32_t ms);

#endif // MOTOR_PATH_CONTROL_H
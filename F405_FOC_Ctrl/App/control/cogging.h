/** @file cogging.h
 *  @brief 原始机械角度索引的齿槽电流前馈，表按正电机转矩方向定义。
 */
#ifndef COGGING_H
#define COGGING_H
#include <stdint.h>
#include <stdbool.h>

#define COGGING_POINTS 1024U
#define COGGING_SCALE_A 0.001f

/* 轴号已检查。表仅在两轴停止时读写；配置只在该轴暂停时修改。 */
int16_t *Cogging_Table(unsigned int axis);
/* 无有效表时由配置层传0，有效表默认用于速度控制；gain=0关闭补偿。 */
void Cogging_SetGain(unsigned int axis, float gain);
float Cogging_Current(unsigned int axis, float mechanical_angle_rad);
#endif

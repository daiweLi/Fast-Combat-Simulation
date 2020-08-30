#pragma once
// --------------------------------------------------------------------------------------------------------------------------------
/**
*   @file           missile.h
*   @brief          导弹飞行及制导
*   @details
					。
					。
					。
					。
*   @author         LiDaiwei
*   @date           20200201
*   @version        1.0.0.1
*   @par Copyright
*                   LiDaiwei
*   @par History
*                   1.0.0.1: LiDaiwei, 20200201, 首次创建
*

*/

// --------------------------------------------------------------------------------------------------------------------------------
/**
*   @name           头文件。
*   @{
*/

#include "../Tools/coordinate.h"
#include "../FlyTac/aircraft.h"


/** @}  */



// --------------------------------------------------------------------------------------------------------------------------------
/**
*   @brief          由状态值求状态的微分
*   @details        由状态值求状态的微分
*   @param[out]     out_d_state           导弹状态
*   @param[in]      in_state              导弹状态
*   @param[in]      accelerator           油门
*   @param[in]      angular_velocity      角速度
*   @retval         0               正常
*   @retval         1               错误
*/
int __missile_f(
	Eigen::Matrix4d* out_d_state,
	const Eigen::Matrix4d& in_state,
	const double accelerator,
	const Eigen::Vector3d angular_velocity);

// --------------------------------------------------------------------------------------------------------------------------------
/**
*   @brief          四阶龙格库塔法
*   @details        四阶龙格库塔法
*   @param[out]     out_state             导弹状态
*   @param[in]      in_state              导弹状态
*   @param[in]      in_timeslice          时间间隔
*   @param[in]      accelerator           油门
*   @param[in]      angular_velocity      角速度
*   @retval         0               正常
*   @retval         1               错误
*/
int __missile_runge4(
	Eigen::Matrix4d* out_state,
	const Eigen::Matrix4d& in_state,
	const double in_timeslice,
	const double accelerator,
	const Eigen::Vector3d angular_velocity);


// 
// --------------------------------------------------------------------------------------------------------------------------------
/**
*   @brief          计算导弹状态
*   @details        计算导弹状态
*   @param[out]     out_state             导弹状态
*   @param[in]      in_state              导弹状态
*   @param[in]      in_timeslice          时间间隔
*   @param[in]      in_handle             操纵量（俯仰角速度，滚转角速度,油门）
*   @retval         0               正常
*   @retval         1               错误
*/
int missile_Flight(
	Eigen::Matrix4d* out_state,
	const Eigen::Matrix4d& in_state,
	const double in_timeslice,
	const Eigen::Vector4d& in_handle);

// 
// 
// --------------------------------------------------------------------------------------------------------------------------------
/**
*   @brief          导弹过点飞
*   @details        PID控制导弹过点飞
*   @param[out]     out_handle            操纵量（俯仰角速度，滚转角速度,油门）
*   @param[out&in]  errA_last             上一时刻方位角误差
*   @param[out&in]  errP_last             上一时刻俯仰角误差
*   @param[out&in]  errA_sum              累积方位角误差
*   @param[out&in]  errP_sum              累积俯仰角误差
*   @param[in]      in_state              导弹现在状态
*   @param[in]      accelerator           油门
*   @param[in]      in_timeslice          时间间隔
*   @param[in]      target_point          目标点
*   @retval         0               正常
*   @retval         1               错误
*/
int missile_Flight_find_point(
	Eigen::Vector4d* out_handle,
	double* errA_last,
	double* errP_last,
	double* errA_sum,
	double* errP_sum,
	const Eigen::Matrix4d& in_state,
	const double accelerator,
	const double in_timeslice,
	const Eigen::Vector3d target_point);
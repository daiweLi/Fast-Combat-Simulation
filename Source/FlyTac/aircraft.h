// --------------------------------------------------------------------------------------------------------------------------------
/**
*   @file           aircft.h
*   @brief          飞行器动力学
*   @details		导航坐标系原点位于参考点，X轴指向北边，Y轴指向东边，Z轴指向地下。
					机体坐标系原点位于质心点，X轴指向机头前方，Y轴指向右侧，Z轴指向下方。
					所有坐标系均为右手系。
					。
					。
					。
					。
*   @author         LiDaiwei
*   @date           20191212
*   @version        1.0.0.1
*   @par Copyright
*                   LiDaiwei
*   @par History
*                   1.0.0.1: LiDaiwei, 20191212, 首次创建
*

*/

// --------------------------------------------------------------------------------------------------------------------------------
/**
*   @name           头文件。
*   @{
*/

#include "../Tools/coordinate.h"



/** @}  */


/**
*   @name           状态参数定义。
*   @{
*/
/*****
//飞机状态
Matrix4d state=[X,Y,Z,0;//位置(导航坐标系)
			   Vx,Vy,Vz,0;//速度(导航坐标系)
			   q0,q1,q2,q3;//姿态(四元数)
			   0,0,0,0]//留空

//控制杆量
Vector3d handle=[w_roll,//滚转角速度(弧度/秒)
				 w_pitch,//俯仰角速度(弧度/秒)
				 w_yaw,//偏航角速度(弧度/秒)
				 a_body]//油门

	  *****/
/** @}  */


// --------------------------------------------------------------------------------------------------------------------------------
/**
*   @brief          由角速度求四元数的微分
*   @details        由角速度求四元数的微分
*   @param[out]     dq              四元数的微分:4维向量
*   @param[in]      q0              初始四元数:4维向量
*   @param[in]      wx              机体相对导航系的角速度在机体系X轴的投影
*   @param[in]      wy              机体相对导航系的角速度在机体系Y轴的投影
*   @param[in]      wz              机体相对导航系的角速度在机体系Z轴的投影
*   @retval         0               正常
*   @retval         1               错误
*/
int angularvelocity_to_d_quaternion(
	Eigen::Vector4d* dq,
	const Eigen::Vector4d& q0,//不加'&'会报错：具有 __declspec(align('16')) 的形参将不被对齐
	const double wx,
	const double wy,
	const double wz);

// --------------------------------------------------------------------------------------------------------------------------------
/**
*   @name           状态参数定义。
*   @{
*/
/*****
//飞机状态
Matrix4d state=[X,Y,Z,0;//位置
			   Vx,Vy,Vz,0;//速度
			   q0,q1,q2,q3;//姿态(由机体系到导航系的四元数)
			   0,0,0,0]
//控制杆量
Vector3d handle=[w_roll,//滚转角速度(弧度/秒)
				 w_pitch,//俯仰角速度(弧度/秒)
				 w_yaw,//偏航角速度(弧度/秒)
				 a_body]//油门

	  *****/
	  /** @}  */


	  // --------------------------------------------------------------------------------------------------------------------------------
	  /**
	  *   @brief          由状态值求状态的微分
	  *   @details        由状态值求状态的微分
	  *   @param[out]     out_d_state           飞机状态
	  *   @param[in]      in_state              飞机状态
	  *   @param[in]      accelerator           油门
	  *   @param[in]      angular_velocity      角速度
	  *   @retval         0               正常
	  *   @retval         1               错误
	  */
int __f(
	Eigen::Matrix4d* out_d_state,
	const Eigen::Matrix4d& in_state,
	const double accelerator,
	const Eigen::Vector3d angular_velocity);

// --------------------------------------------------------------------------------------------------------------------------------
/**
*   @brief          四阶龙格库塔法
*   @details        四阶龙格库塔法
*   @param[out]     out_state             飞机状态
*   @param[in]      in_state              飞机状态
*   @param[in]      in_timeslice          时间间隔
*   @param[in]      accelerator           油门
*   @param[in]      angular_velocity      角速度
*   @retval         0               正常
*   @retval         1               错误
*/
int runge4(
	Eigen::Matrix4d* out_state,
	const Eigen::Matrix4d& in_state,
	const double in_timeslice,
	const double accelerator,
	const Eigen::Vector3d angular_velocity);


// 
// --------------------------------------------------------------------------------------------------------------------------------
/**
*   @brief          计算飞机状态
*   @details        计算飞机状态
*   @param[out]     out_state             飞机状态
*   @param[in]      in_state              飞机状态
*   @param[in]      in_timeslice          时间间隔
*   @param[in]      in_handle             操纵量（俯仰角速度，滚转角速度,油门）
*   @retval         0               正常
*   @retval         1               错误
*/
int Flight(
	Eigen::Matrix4d* out_state,
	const Eigen::Matrix4d& in_state,
	const double in_timeslice,
	const Eigen::Vector4d& in_handle);



// --------------------------------------------------------------------------------------------------------------------------------
/**
*   @brief          滚转轴PID
*   @details        滚转轴PID
*   @param[out]     dRoll_out             输出值
*   @param[in]      error           当前误差
*   @param[in]      inte_point      累积误差
*   @param[in]      dif_error_      误差微分
*   @retval         0               正常
*   @retval         1               错误
*/
int PID_Roll(
	double* dRoll_out,
	const double error,
	const double inte_error,
	const double dif_error_);

// --------------------------------------------------------------------------------------------------------------------------------
/**
*   @brief          俯仰轴PID
*   @details        俯仰轴PID
*   @param[out]     dPitch_out             输出值
*   @param[in]      error           当前误差
*   @param[in]      inte_point      累积误差
*   @param[in]      dif_error_      误差微分
*   @retval         0               正常
*   @retval         1               错误
*/
int PID_Pitch(
	double* dPitch_out,
	const double error,
	const double inte_error,
	const double dif_error_);

// --------------------------------------------------------------------------------------------------------------------------------
/**
*   @brief          航向轴PID
*   @details        航向轴PID
*   @param[out]     dYaw_out             输出值
*   @param[in]      error           当前误差
*   @param[in]      inte_point      累积误差
*   @param[in]      dif_error_      误差微分
*   @retval         0               正常
*   @retval         1               错误
*/
int PID_Yaw(
	double* dYaw_out,
	const double error,
	const double inte_error,
	const double dif_error_);


// 
// --------------------------------------------------------------------------------------------------------------------------------
/**
*   @brief          过点飞
*   @details        PID控制飞机过点飞
*   @param[out]     out_handle            操纵量（俯仰角速度，滚转角速度,油门）
*   @param[out&in]  errA_last             上一时刻方位角误差
*   @param[out&in]  errP_last             上一时刻俯仰角误差
*   @param[out&in]  errA_sum              累积方位角误差
*   @param[out&in]  errP_sum              累积俯仰角误差
*   @param[in]      in_state              飞机现在状态
*   @param[in]      accelerator           油门
*   @param[in]      in_timeslice          时间间隔
*   @param[in]      target_point          目标点
*   @retval         0               正常
*   @retval         1               错误
*/
int Flight_find_point(
	Eigen::Vector4d* out_handle,
	double* errA_last,
	double* errP_last,
	double* errR_last,
	double* errA_sum,
	double* errP_sum,
	const Eigen::Matrix4d& in_state,
	const double accelerator,
	const double in_timeslice,
	const Eigen::Vector3d target_point);


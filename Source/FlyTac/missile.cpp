// --------------------------------------------------------------------------------------------------------------------------------
/**
*   @file           missile.cpp
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


#include "../FlyTac/missile.h"
#include <algorithm>

#include<iostream>
using namespace std;
using namespace Eigen;

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
	const Eigen::Vector3d angular_velocity)
{
	Vector4d X0 = in_state.row(0),
		V0 = in_state.row(1),
		q0 = in_state.row(2);
	Vector4d dX, dV, dq;
	//求位置的微分
	dX = V0;

	//求速度的微分
	Matrix3d Rnb, Rbn;

	quaternion_to_rotation(&Rbn, q0);
	Rnb = Rbn.transpose();
	double 	ax, ay, az;//空气阻力

	Vector3d v_body,//机体系速度	
		v_n;//导航系速度
	v_n << V0(0), V0(1), V0(2);
	v_body = Rnb * v_n;

	ax = -(v_body(0) < 0 ? -1 : 1) * 8e-10 * fabs(pow(v_body(0), 4));//阻力
	ay = -(v_body(1) < 0 ? -1 : 1) * 1e-8 * fabs(pow(v_body(1), 4));//阻力
	az = -(v_body(2) < 0 ? -1 : 1) * 1e-6 * fabs(pow(v_body(2), 4));//阻力
	//阻力方向和速度方向相反

	Vector3d Acceleration,//导航系加速度
		acceleration;//机体系加速度
	//cout << "vn" << v_n << endl << endl;
	//cout << "R" << Rnb << endl;
	//cout << "vb"<<v_body << endl << endl;
	if (X0(2) > 33000)
		acceleration << 0 + ax, ay,  az;
	else
		acceleration << accelerator + ax, ay,  az;

	//cout <<"a"<< acceleration << endl<<endl;
	Acceleration = Rbn * acceleration;
	Acceleration(2) = Acceleration(2) + simple_gravity;//重力方向向下
	dV << Acceleration(0), Acceleration(1), Acceleration(2), 0;
	//cout << dV << endl;
	//cin.get();
	for (int i = 0; i < 4; i++) {
		if (fabs(V0(i)) > 340 && fabs(dV(i)) > fabs(V0(i)) && dV(i) * V0(i) < 0)
			dV(i) = -V0(i);
	}
	//cout <<"dV" << Acceleration(0)<<"  "<< Acceleration(1) << "  " << Acceleration(2) << endl<<endl;


	//求四元数的微分
	angularvelocity_to_d_quaternion(&dq, q0,
		angular_velocity(0), angular_velocity(1), angular_velocity(2));

	(*out_d_state).row(0) = dX;
	(*out_d_state).row(1) = dV;
	(*out_d_state).row(2) = dq;
	(*out_d_state).row(3) << 0, 0, 0, 0;

	return 0;
}

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
	const Eigen::Vector3d angular_velocity)
{
	double dt = in_timeslice;
	Matrix4d  k1, k2, k3, k4;						// for Runge-Kutta
	Matrix4d xn;

	// K1 = f(tn,xn)
	xn = in_state;
	__missile_f(&k1, xn, accelerator, angular_velocity);

	// K2 = f(t[n]+h/2,xn+h/2*K1)
	xn = in_state + 0.5 * dt * k1;
	__missile_f(&k2, xn, accelerator, angular_velocity);

	// K3 = f(t[n]+h/2,xn+h/2*K2)
	xn = in_state + 0.5 * dt * k2;
	__missile_f(&k3, xn, accelerator, angular_velocity);

	// K4 = f(t[n]+h/2,xn+h*K3)
	xn = in_state + dt * k3;
	__missile_f(&k4, xn, accelerator, angular_velocity);

	// x[n+1]=x[n]+h/6*(K1+2*K2+2*K3+K4)
	*out_state = in_state + dt * (k1 + 2 * k2 + 2 * k3 + k4) / 6.0;

	//四元数归一化
	Vector4d qa, qb;
	qa = (*out_state).row(2);
	quaterntion_normalized(&qb, qa);
	(*out_state).row(2) = qb;

	return 0;
}

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
	const Eigen::Vector4d& in_handle)
{
	Vector3d  angular_velocity;

	double wx, wy, wz, maxw, maxw2;
	maxw = M_PI_2;
	maxw2 = M_PI / 12;
	wx = (fabs(in_handle(0)) < maxw) ? in_handle(0) : (maxw * fabs(in_handle(0)) / in_handle(0));
	wy = (fabs(in_handle(1)) < maxw) ? in_handle(1) : (maxw * fabs(in_handle(1)) / in_handle(1));

	wz = (fabs(in_handle(2)) < maxw2) ? in_handle(2) : (maxw2 * fabs(in_handle(2)) / in_handle(2));


	angular_velocity << wx, wy, wz;//Wnbb:x-滚转y-俯仰z-偏航

	//angular_velocity << in_handle(1), in_handle(0), 0;//Wnbb:x-滚转y-俯仰z-偏航
	//cout << angular_velocity << endl << endl;

	__missile_runge4(out_state, in_state, in_timeslice, in_handle(3), angular_velocity);


	return 0;
}


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
	Vector4d* out_handle,
	double* errA_last,
	double* errP_last,
	double* errA_sum,
	double* errP_sum,
	const Matrix4d& in_state,
	const double accelerator,
	const double in_timeslice,
	const Vector3d target_point)
{
	Vector3d flight_point;
	flight_point << in_state(0, 0), in_state(0, 1), in_state(0, 2);
	Matrix3d Rnb, Rbn;
	quaternion_to_rotation(&Rbn, in_state.row(2));
	Rnb = Rbn.transpose();

	//计算目标点相对飞机的距离
	double distance = (target_point - flight_point).norm();
	//cout << distance << endl;
	//cout << flight_point << endl << endl;
	//cout << target_point << endl << endl;

	//if (distance > 9900 || _isnan(distance)) {

	//	cin.get();
	//	cin.get();
	//}

	double roll = 0, pitch = 0, yaw = 90;
	quaternion_bn_to_euler(&roll, &pitch, &yaw, in_state.row(2));


	if (distance > 50) {
		//分别计算目标点相对飞机的方位角和俯仰角：
		Vector3d vector_flight_to_target;
		vector_flight_to_target = Rnb * (target_point - flight_point);
		//目标方位角
		double Azimuth = atan2(vector_flight_to_target(1), vector_flight_to_target(0));
		//目标俯仰角
		double Pitch = atan2(
			-vector_flight_to_target(2), sqrt(
				pow(vector_flight_to_target(0), 2) + pow(vector_flight_to_target(1), 2)));



		//计算速度矢量相对飞机的的方位角和俯仰角：
		Vector3d vector_velocity;
		vector_velocity << in_state(1, 0), in_state(1, 1), in_state(1, 2);
		vector_velocity = Rnb * vector_velocity;
		//速度方位角
		double Azimuth_V = atan2(vector_velocity(1), vector_velocity(0));
		//速度俯仰角
		double Pitch_V = atan2(
			-vector_velocity(2), sqrt(
				pow(vector_velocity(0), 2) + pow(vector_velocity(1), 2)));


		//cout << Rnb << endl << endl;

		//cout << flight_point << endl << endl;
		//cout << vector_flight_to_target << endl << endl;
		//cout << Azimuth << "  " << Pitch << " |||  " << Azimuth_V << "  " << Pitch_V << endl;

		//cin.get();

		//计算两个角度误差
		double errA = Azimuth - Azimuth_V;
		double errP = Pitch - Pitch_V;




		double d_roll = 0, d_pitch = 0, d_yaw = 0;

		*errA_sum += errA;
		*errP_sum += errP;

		PID_Roll(&d_roll, errA , *errA_sum, errA - (*errA_last));

		PID_Pitch(&d_pitch, errP, *errP_sum, errP - (*errP_last));
		PID_Yaw(&d_yaw, errA, *errA_sum, errA - (*errA_last));
		

		if (roll >= 170) {
			d_roll = -2;
		}
		if (roll <= -170) {
			d_roll = 2;
		}
		


		*errA_last = errA;
		*errP_last = errP;
		(*out_handle)(0) = d_roll;
		(*out_handle)(1) = d_pitch;
		(*out_handle)(2) = d_yaw;
		(*out_handle)(3) = accelerator;

	}
	else {
		(*out_handle)(0) = 0;
		(*out_handle)(1) = 0;
		(*out_handle)(2) = 0;
		(*out_handle)(3) = accelerator;


		*errA_last = 0;
		*errP_last = 0;

	}



	return 0;
}
// --------------------------------------------------------------------------------------------------------------------------------
/**
*   @file           aircft.cpp
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

#include "aircraft.h"
#include <algorithm>

#include<iostream>
using namespace std;
using namespace Eigen;
/** @}  */


// --------------------------------------------------------------------------------------------------------------------------------
/**
*   @brief          由角速度求四元数的微分
*   @details        由角速度求四元数的微分
*   @param[out]     dq              从机体系到导航系的转动四元数微分:4维向量
*   @param[in]      q0              初始从机体系到导航系的转动四元数:4维向量
*   @param[in]      wx              机体相对导航系的角速度在机体系X轴的投影
*   @param[in]      wy              机体相对导航系的角速度在机体系Y轴的投影
*   @param[in]      wz              机体相对导航系的角速度在机体系Z轴的投影
*   @retval         0               正常
*   @retval         1               错误
*/
int angularvelocity_to_d_quaternion(
	Vector4d* dq,
	const Vector4d & q0,//不加'&'会报错：具有 __declspec(align('16')) 的形参将不被对齐
	const double wx,
	const double wy,
	const double wz)
{

	Matrix4d W;
	W << 0, -wx, -wy, -wz,
		wx, 0, wz, -wy,
		wy, -wz, 0, wx,
		wz, wy, -wx, 0;

	*dq = 0.5 * (W * q0);

	return 0;
}

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
	Matrix4d* out_d_state,
	const Matrix4d& in_state,
	const double accelerator,
	const Vector3d angular_velocity)
{

	Vector4d X0 = in_state.row(0),
		V0 = in_state.row(1),
		q0 = in_state.row(2);
	Vector4d dX, dV, dq;
	//求位置的微分
	dX = V0;

	//求速度的微分
	Matrix3d Rnb,Rbn;

	quaternion_to_rotation(&Rbn, q0);
	Rnb = Rbn.transpose();
	double aL,//飞机升力
		ax, ay, az;//空气阻力

	Vector3d v_body,//机体系速度	
		v_n;//导航系速度
	v_n << V0(0), V0(1), V0(2);
	v_body= Rnb * v_n;
	aL = -(v_body(0) < 0 ? -1 : 1) * 5e-5 * v_body(0) * v_body(0);//升力-约220KM/H时起飞
	if(X0(2)>0)
		aL *= exp(	(X0(2)) / 5000);
	ax = -(v_body(0) < 0 ? -1 : 1) * 1e-9 * fabs(pow(v_body(0),4)) ;//阻力
	ay = -(v_body(1) < 0 ? -1 : 1) * 1e-8 * fabs(pow(v_body(1), 4)) ;//阻力
	az = -(v_body(2) < 0 ? -1 : 1) * 1e-6 * fabs(pow(v_body(2), 4)) ;//阻力
	//阻力方向和速度方向相反

	Vector3d Acceleration,//导航系加速度
		acceleration;//机体系加速度
	//cout << "vn" << v_n << endl << endl;
	//cout << "R" << Rnb << endl;
	//cout << "vb"<<v_body << endl << endl;
	if(X0(2)>33000)
		acceleration <<  0+ax, ay, aL + az;
	else
		acceleration << accelerator+ax, ay, aL+az;
	
	//cout <<"a"<< acceleration << endl<<endl;
	Acceleration = Rbn * acceleration;
	Acceleration(2) = Acceleration(2)+simple_gravity;//重力方向向下
	dV << Acceleration(0), Acceleration(1), Acceleration(2), 0;
	//cout << dV << endl;
	//cin.get();
	for (int i = 0; i < 4; i++) {
		if (fabs(V0(i))>340 && fabs(dV(i)) > fabs(V0(i)) && dV(i) * V0(i) < 0)
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
*   @param[out]     out_state             飞机状态
*   @param[in]      in_state              飞机状态
*   @param[in]      in_timeslice          时间间隔
*   @param[in]      accelerator           油门
*   @param[in]      angular_velocity      角速度
*   @retval         0               正常
*   @retval         1               错误
*/
int runge4(
	Matrix4d* out_state,
	const Matrix4d& in_state,
	const double in_timeslice,
	const double accelerator,
	const Vector3d angular_velocity)
{
	double dt = in_timeslice;
	Matrix4d  k1, k2, k3, k4;						// for Runge-Kutta
	Matrix4d xn;

	// K1 = f(tn,xn)
	xn = in_state;
	__f(&k1, xn, accelerator, angular_velocity);

	// K2 = f(t[n]+h/2,xn+h/2*K1)
	xn = in_state + 0.5 * dt * k1;
	__f(&k2, xn, accelerator, angular_velocity);

	// K3 = f(t[n]+h/2,xn+h/2*K2)
	xn = in_state + 0.5 * dt * k2;
	__f(&k3, xn, accelerator, angular_velocity);

	// K4 = f(t[n]+h/2,xn+h*K3)
	xn = in_state + dt * k3;
	__f(&k4, xn, accelerator, angular_velocity);

	// x[n+1]=x[n]+h/6*(K1+2*K2+2*K3+K4)
	*out_state = in_state + dt * (k1 + 2 * k2 + 2 * k3 + k4) / 6.0;
	
	//四元数归一化
	Vector4d qa, qb;
	qa = (*out_state).row(2);
	quaterntion_normalized(&qb, qa);
	(*out_state).row(2) = qb;

	return 0;
}


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
	Matrix4d* out_state,
	const Matrix4d & in_state,
	const double in_timeslice,
	const Vector4d & in_handle)
{
	Vector3d  angular_velocity;

	double wx, wy,wz, maxw,maxw2;
	maxw = M_PI_2;
	maxw2 = M_PI / 12;
	wx = (fabs(in_handle(0)) < maxw) ? in_handle(0) : (maxw * fabs(in_handle(0)) / in_handle(0));
	wy = (fabs(in_handle(1)) < maxw) ? in_handle(1) : (maxw * fabs(in_handle(1)) / in_handle(1));
	
	wz = (fabs(in_handle(2)) < maxw2) ? in_handle(2) : (maxw2 * fabs(in_handle(2)) / in_handle(2));


	angular_velocity << wx, wy, wz;//Wnbb:x-滚转y-俯仰z-偏航
	
	//angular_velocity << in_handle(1), in_handle(0), 0;//Wnbb:x-滚转y-俯仰z-偏航
	//cout << angular_velocity << endl << endl;

	runge4(out_state, in_state, in_timeslice, in_handle(3), angular_velocity);
	   

	return 0;
}



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
	const double dif_error_)
{
	double Kp = 1.,
		Ki = 0.5,
		Kd = 10.;
	*dRoll_out = Kp * error + Ki * inte_error + Kd * dif_error_;

	return 0;
}

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
	const double dif_error_)
{
	double Kp = 1,
		Ki = 0.0,
		Kd = 2;
	*dPitch_out = Kp * error + Ki * inte_error + Kd * dif_error_;

	return 0;
}


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
	const double dif_error_)
{
	double Kp = 1,
		Ki = 0.0,
		Kd = 20;
	*dYaw_out = Kp * error + Ki * inte_error + Kd * dif_error_;

	return 0;
}

// 
// --------------------------------------------------------------------------------------------------------------------------------
/**
*   @brief          过点飞
*   @details        PID控制飞机过点飞
*   @param[out]     out_handle            操纵量（俯仰角速度，滚转角速度,油门）
*   @param[out&in]  errA_last             上一时刻方位角误差
*   @param[out&in]  errP_last             上一时刻俯仰角误差
*   @param[out&in]  errR_last             上一时刻滚转角误差
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
	Vector4d * out_handle,
	double* errA_last,
	double* errP_last,
	double* errR_last,
	double* errA_sum,
	double* errP_sum,
	const Matrix4d & in_state,
	const double accelerator,
	const double in_timeslice,
	const Vector3d target_point)
{
	Vector3d flight_point;
	flight_point << in_state(0, 0), in_state(0, 1), in_state(0, 2);
	Matrix3d Rnb,Rbn;
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

	double roll = 0, pitch = 0, yaw = 90, errRoll = 0;
	quaternion_bn_to_euler(&roll, &pitch, &yaw, in_state.row(2));


	if (distance > 30) {
		//分别计算目标点相对飞机的方位角和俯仰角：
		Vector3d vector_flight_to_target;
		vector_flight_to_target = Rnb * (target_point - flight_point);
		//目标方位角
		double Azimuth = atan2(vector_flight_to_target(1), vector_flight_to_target(0));
		//目标俯仰角
		double Pitch = atan2(
			-vector_flight_to_target(2),	sqrt(
				pow(vector_flight_to_target(0), 2)+pow(vector_flight_to_target(1),2)));



		//计算速度矢量相对飞机的的方位角和俯仰角：
		Vector3d vector_velocity;
		vector_velocity  << in_state(1, 0), in_state(1, 1), in_state(1, 2);
		vector_velocity = Rnb * vector_velocity;
		//速度方位角
		double Azimuth_V = atan2(vector_velocity(1), vector_velocity(0));
		//速度俯仰角
		double Pitch_V = atan2(
			-vector_velocity(2) , sqrt(
				pow(vector_velocity(0), 2) + pow(vector_velocity(1), 2)));


		//cout << Rnb << endl << endl;

		//cout << flight_point << endl << endl;
		//cout << vector_flight_to_target << endl << endl;
		//cout << Azimuth << "  " << Pitch << " |||  " << Azimuth_V << "  " << Pitch_V << endl;

		//cin.get();

		//计算两个角度误差
		double errA = Azimuth - Azimuth_V;
		double errP = Pitch - Pitch_V;


		//保证飞机姿态稳定
		errRoll = ((errA>=0)?1:-1) *  0.4*(roll * M_PI / 180);



		double d_roll = 0, d_pitch = 0, d_yaw = 0;

		if (fabs(errA) > M_PI / 18) {
			//errA += errRoll;
			*errA_sum += errA;
			*errP_sum += errP;
			if (fabs(roll) <= 90) {
				PID_Roll(&d_roll, errA + errRoll, *errA_sum, errA - (*errA_last)+ errRoll - (*errR_last));
			}
			else{
				PID_Roll(&d_roll, errRoll, 0, 0);


			}
			PID_Pitch(&d_pitch, errP, *errP_sum, errP - (*errP_last));
			PID_Yaw(&d_yaw, errA, *errA_sum, errA - (*errA_last));
		}
		else {
			*errA_sum += errA;
			*errP_sum += errP;
			PID_Roll(&d_roll, errRoll, 0, errRoll - (*errR_last));
			PID_Pitch(&d_pitch, errP, *errP_sum, errP - (*errP_last));
			PID_Yaw(&d_yaw, errA, *errA_sum, errA - (*errA_last));

			if (roll >= 170) {
				d_roll = -2;
			}
			if (roll <= -170) {
				d_roll = 2;
			}
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

	*errR_last = errRoll;

	return 0;
}


// --------------------------------------------------------------------------------------------------------------------------------
/**
*   @file           UnitDefine.cpp
*   @brief          部队功能实现。
*   @details        部队功能实现。
*   @author         lidaiwei
*   @date           20200908
*   @version        1.0.0.1
*   @par Copyright
*                   GaoYang
*   @par History
*                   1.0.0.1: lidaiwei, 20200908, 首次创建
*                   
*/

// --------------------------------------------------------------------------------------------------------------------------------
/**
*   @name           头文件。
*   @{
*/
#include "UnitDefine.h"

using namespace Eigen;
using namespace CombatSimulation;
/** @}  */



// --------------------------------------------------------------------------------------------------------------------------------
/**
*   @brief          设置单位坐标
*   @details        设置单位坐标
*   @param[in]      in_coordinate_longitude             坐标，经度，单位：度
*   @param[in]      in_coordinate_latitude              坐标，纬度，单位：度
*   @param[in]      in_coordinate_altitude              坐标，高度，单位：米
*   @param[in]      in_coordinate_roll                  坐标，滚转角，单位：度
*   @param[in]      in_coordinate_pitch                 坐标，俯仰角，单位：度
*   @param[in]      in_coordinate_yaw					坐标，偏航角，单位：度
*   @retval         0                    正常
*/
int Unit_Object_C::SetCoordinate(
	double							in_coordinate_longitude,
	double							in_coordinate_latitude,
	double							in_coordinate_altitude,
	double							in_coordinate_roll,
	double							in_coordinate_pitch,
	double							in_coordinate_yaw)
{
	coordinate_longitude = in_coordinate_longitude;
	coordinate_latitude = in_coordinate_latitude;
	coordinate_altitude = in_coordinate_altitude;
	coordinate_roll = in_coordinate_roll;
	coordinate_pitch = in_coordinate_pitch;
	coordinate_yaw = in_coordinate_yaw;

	return CS_OK;
}


// --------------------------------------------------------------------------------------------------------------------------------
/**
*   @brief          设置单位速度
*   @details        设置单位速度
*   @param[in]      in_velocity_north             速度，北向，单位：米/秒
*   @param[in]      in_velocity_east              速度，东向，单位：米/秒
*   @param[in]      in_velocity_downward          速度，地面方向，单位：米/秒
*   @retval         0                    正常
*/
int Unit_Object_C::SetVelocity(
	double							in_velocity_north,
	double							in_velocity_east,
	double							in_velocity_downward)
{
	velocity_north = in_velocity_north;
	velocity_east = in_velocity_east;
	velocity_downward = in_velocity_downward;

	return CS_OK;
}


// --------------------------------------------------------------------------------------------------------------------------------
/**
*   @brief          初始化一个飞机实体
*   @details        初始化一个飞机实体
*   @param[in]      in_simulation_id        仿真id，唯一标识
*   @param[in]      in_base_name            对象名字，例：F-16
*   @param[in]      in_base_team            所属队伍 1-红方 2-蓝方
*   @param[in]      in_lon                  坐标，经度，单位：度
*   @param[in]      in_lat                  坐标，纬度，单位：度
*   @param[in]      in_alt                  坐标，高度，单位：米
*   @param[in]      in_roll                 坐标，滚转角，单位：度
*   @param[in]      in_pitch                坐标，俯仰角，单位：度
*   @param[in]      in_yaw					坐标，偏航角，单位：度
*   @param[in]      in_velocity_north       速度，北向，单位：米/秒
*   @param[in]      in_velocity_east		速度，东向，单位：米/秒
*   @param[in]      in_velocity_downward	速度，地面方向，单位：米/秒
*   @retval         0                    正常
*/
int Aircraft_Object_C::Init(
	int in_simulation_id,
	std::string in_base_name,
	int    in_base_team,
	double in_lon,
	double in_lat,
	double in_alt,
	double in_roll,
	double in_pitch,
	double in_yaw,
	double in_velocity_north,
	double in_velocity_east,
	double in_velocity_downward) 
{
	Sim_id = in_simulation_id;
	base_live = CS_LIVE;

	strcpy_s(base_name, max_str, in_base_name.c_str());

	strcpy_s(base_type, max_str, "Air+FixedWing");
	base_team = in_base_team;

	coordinate_longitude = in_lon;
	coordinate_latitude =  in_lat;
	coordinate_altitude =  in_alt;
	coordinate_roll =      in_roll;
	coordinate_pitch =     in_pitch;
	coordinate_yaw =       in_yaw;

	radar_state = 1;

	double xn, yn, zn;
	earth_to_navigation(&xn, &yn, &zn, coordinate_longitude, coordinate_latitude, coordinate_altitude,
		p_battle_header->reference_longitude, p_battle_header->reference_latitude, p_battle_header->reference_altitude);
	Vector4d qbn;
	euler_to_quaternion_bn(&qbn, coordinate_roll, coordinate_pitch, coordinate_yaw);

	craft_state << xn, yn, zn, 0.0,
		in_velocity_north, in_velocity_east, in_velocity_downward, 0.,
		qbn(0), qbn(1), qbn(2), qbn(3),
		0, 0, 0, 0;

	return CS_OK;
}


// --------------------------------------------------------------------------------------------------------------------------------
/**
*   @brief          飞机实体单步解算
*   @details        飞机实体单步解算
*   @param[in]      d_time          单步时间间隔 单位：秒
*   @retval         0               正常
*   @retval         -1              飞机已死亡
*/
int Aircraft_Object_C::Run(double d_time)
{
	if (base_live != 1) {
		return CS_NOT_LIVE;
	}

	//飞机状态解算
	Flight(&craft_state, craft_state, d_time, craft_handle);

	//坐标转换

	navigation_to_earth(&coordinate_longitude, &coordinate_latitude, &coordinate_altitude, 
		craft_state(0, 0), craft_state(0, 1),craft_state(0, 2),
		p_battle_header->reference_longitude, p_battle_header->reference_latitude, p_battle_header->reference_altitude);

	quaternion_bn_to_euler(&coordinate_roll, &coordinate_pitch, &coordinate_yaw, craft_state.row(2));

	velocity_north = craft_state(1, 0);
	velocity_east = craft_state(1, 1);
	velocity_downward = craft_state(1, 2);

	return CS_OK;
}



// --------------------------------------------------------------------------------------------------------------------------------
/**
*   @brief          初始化一个导弹实体
*   @details        初始化一个导弹实体
*   @param[in]      in_simulation_id        仿真id，唯一标识
*   @param[in]      in_base_name            对象名字，例：F-16
*   @param[in]      in_base_team            所属队伍 1-红方 2-蓝方
*   @param[in]      in_lon                  坐标，经度，单位：度
*   @param[in]      in_lat                  坐标，纬度，单位：度
*   @param[in]      in_alt                  坐标，高度，单位：米
*   @param[in]      in_roll                 坐标，滚转角，单位：度
*   @param[in]      in_pitch                坐标，俯仰角，单位：度
*   @param[in]      in_yaw					坐标，偏航角，单位：度
*   @param[in]      in_velocity_north       速度，北向，单位：米/秒
*   @param[in]      in_velocity_east		速度，东向，单位：米/秒
*   @param[in]      in_velocity_downward	速度，地面方向，单位：米/秒
*   @retval         0                    正常
*/
int Missile_Object_C::Init(
	int in_simulation_id,
	std::string in_base_name,
	int    in_base_team,
	double in_lon,
	double in_lat,
	double in_alt,
	double in_roll,
	double in_pitch,
	double in_yaw,
	double in_velocity_north,
	double in_velocity_east,
	double in_velocity_downward)
{
	Sim_id = in_simulation_id;
	base_live = CS_LIVE;

	strcpy_s(base_name, max_str, in_base_name.c_str());

	strcpy_s(base_type, max_str, "Air+Missile");
	base_team = in_base_team;

	coordinate_longitude = in_lon;
	coordinate_latitude = in_lat;
	coordinate_altitude = in_alt;
	coordinate_roll = in_roll;
	coordinate_pitch = in_pitch;
	coordinate_yaw = in_yaw;

	radar_state = 1;

	double xn, yn, zn;
	earth_to_navigation(&xn, &yn, &zn, coordinate_longitude, coordinate_latitude, coordinate_altitude,
		p_battle_header->reference_longitude, p_battle_header->reference_latitude, p_battle_header->reference_altitude);
	Vector4d qbn;
	euler_to_quaternion_bn(&qbn, coordinate_roll, coordinate_pitch, coordinate_yaw);

	missile_state << xn, yn, zn, 0.0,
		in_velocity_north, in_velocity_east, in_velocity_downward, 0.,
		qbn(0), qbn(1), qbn(2), qbn(3),
		0, 0, 0, 0;

	return CS_OK;
}

// --------------------------------------------------------------------------------------------------------------------------------
/**
*   @brief          导弹实体单步解算
*   @details        导弹实体单步解算
*   @param[in]      d_time          单步时间间隔 单位：秒
*   @retval         0               正常
*   @retval         -1              导弹已死亡
*/
int Missile_Object_C::Run(
	double d_time)
{
	if (missile_live != 1) {
		return CS_NOT_LIVE;
	}

	Matrix4d target_state = p_target_air->craft_state;

	Vector3d TargetMissile;
	//导弹与目标距离（米）
	distance_target = (target_state.row(0) - missile_state.row(0)).norm();

	//导弹运动目标点坐标
	double K_target = target_state.row(1).norm() / (0.001 * (distance_target));//瞄准目标运动方向的前方
	if (distance_target <= destroy_range * 1.5)
		K_target = 195 * target_state.row(1).norm() / missile_state.row(1).norm();

	TargetMissile << target_state(0, 0) + (target_state(1, 0) / target_state.row(1).norm()) * K_target,
		target_state(0, 1) + (target_state(1, 1) / target_state.row(1).norm()) * K_target,
		target_state(0, 2) + (target_state(1, 2) / target_state.row(1).norm()) * K_target;

	//导弹命中判定
	int t_m_s = HitCheck();
	if (t_m_s < 0 ) {
		missile_live = t_m_s;
		base_live = 0;
		return t_m_s;
	}
	else {
		missile_live = 1;
		base_live = 1;
	}

	Flight_find_point(&missile_handle, &missile_errA, &missile_errP, &missile_errR,
		&missile_errAsum, &missile_errPsum, missile_state, 90, d_time, TargetMissile);


	//导弹飞行计算
	missile_Flight(&missile_state, missile_state, d_time, missile_handle);

	//坐标转换
	navigation_to_earth(&coordinate_longitude, &coordinate_latitude, &coordinate_altitude,
		missile_state(0, 0), missile_state(0, 1), missile_state(0, 2),
		p_battle_header->reference_longitude, p_battle_header->reference_latitude, p_battle_header->reference_altitude);

	quaternion_bn_to_euler(&coordinate_roll, &coordinate_pitch, &coordinate_yaw, missile_state.row(2));

	velocity_north = missile_state(1, 0);
	velocity_east = missile_state(1, 1);
	velocity_downward = missile_state(1, 2);

	return CS_OK;
}

// --------------------------------------------------------------------------------------------------------------------------------
/**
*   @brief          导弹命中判定
*   @details        导弹命中判定
*   @retval         0               运行中，未命中
*   @retval         -1              已命中
*   @retval         -2              超出射程，未命中
*/
int Missile_Object_C::HitCheck()
{
	static Matrix4d missile_position_last = missile_state;
	static double missile_journey = 0;

	if (distance_target <= destroy_range) {

		p_target_air->base_live = 0;

		return CS_NOT_LIVE;
	}

	double d_distance = (missile_state.row(0) - missile_position_last.row(0)).norm();
	missile_journey += d_distance;

	if (missile_journey >= max_journey) {
		return CS_MISS;
	}


	missile_position_last = missile_state;

	return CS_LIVE;
}


// --------------------------------------------------------------------------------------------------------------------------------
/**
*   @brief          设置参考点坐标
*   @details        设置参考点坐标
*   @param[in]      in_reference_longitude             参考点经度，单位：deg
*   @param[in]      in_reference_latitude              参考点纬度，单位：deg
*   @param[in]      in_reference_altitude              参考点高度，单位：米
*   @retval         0                    正常
*/
int Battlefield_C::InitCoordinate(
	double							in_reference_longitude,
	double							in_reference_latitude,
	double							in_reference_altitude)
{
	battle_header.reference_longitude = in_reference_longitude;
	battle_header.reference_latitude = in_reference_latitude;
	battle_header.reference_altitude = in_reference_altitude;

	return CS_OK;
}


// --------------------------------------------------------------------------------------------------------------------------------
/**
*   @brief          飞机发射导弹
*   @details        飞机发射导弹
*   @param[in]      attack_air             发射机
*   @param[in]      target_air             目标机
*   @retval         0                    正常
*/
int Battlefield_C::MissileFire(
	Aircraft_Object_C& attack_air,
	Aircraft_Object_C& target_air)
{
	missile_count++;

	missile_list[missile_count - 1].Init(20000000 + missile_count, "PL-10", attack_air.base_team,
		attack_air.coordinate_longitude, attack_air.coordinate_latitude, attack_air.coordinate_altitude,
		attack_air.coordinate_roll, attack_air.coordinate_pitch, attack_air.coordinate_yaw,
		attack_air.velocity_north, attack_air.velocity_east, attack_air.velocity_downward);

	missile_list[missile_count - 1].father_id = attack_air.Sim_id;
	missile_list[missile_count - 1].p_target_air = &target_air;
	missile_list[missile_count - 1].missile_live = CS_LIVE;

	return CS_OK;
}
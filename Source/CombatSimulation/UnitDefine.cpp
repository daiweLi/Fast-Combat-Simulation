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


BattlefieldHeader_T battle_header;

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

	return 0;
}

int Unit_Object_C::SetVelocity(
	double							in_velocity_north,
	double							in_velocity_east,
	double							in_velocity_downward)
{
	velocity_north = in_velocity_north;
	velocity_east = in_velocity_east;
	velocity_downward = in_velocity_downward;

	return 0;
}

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
	strcpy_s(base_name, max_str, in_base_name.c_str());

	strcpy_s(base_type, max_str, "Aircraft");
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
		battle_header.reference_longitude, battle_header.reference_latitude, battle_header.reference_altitude);
	Vector4d qbn;
	euler_to_quaternion_bn(&qbn, coordinate_roll, coordinate_pitch, coordinate_yaw);

	craft_state << xn, yn, zn, 0.0,
		in_velocity_north, in_velocity_east, in_velocity_downward, 0.,
		qbn(0), qbn(1), qbn(2), qbn(3),
		0, 0, 0, 0;

	return 0;
}

int Aircraft_Object_C::Run(double d_time)
{
	//飞机状态解算
	Flight(&craft_state, craft_state, d_time, craft_handle);

	//坐标转换

	navigation_to_earth(&coordinate_longitude, &coordinate_latitude, &coordinate_altitude, 
		craft_state(0, 0), craft_state(0, 1),craft_state(0, 2),
		battle_header.reference_longitude, battle_header.reference_latitude, battle_header.reference_altitude);

	quaternion_bn_to_euler(&coordinate_roll, &coordinate_pitch, &coordinate_yaw, craft_state.row(2));

	return 0;
}


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
	strcpy_s(base_name, max_str, in_base_name.c_str());

	strcpy_s(base_type, max_str, "Missile");
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
		battle_header.reference_longitude, battle_header.reference_latitude, battle_header.reference_altitude);
	Vector4d qbn;
	euler_to_quaternion_bn(&qbn, coordinate_roll, coordinate_pitch, coordinate_yaw);

	missile_state << xn, yn, zn, 0.0,
		in_velocity_north, in_velocity_east, in_velocity_downward, 0.,
		qbn(0), qbn(1), qbn(2), qbn(3),
		0, 0, 0, 0;

	return 0;
}


int Missile_Object_C::Run(
	double d_time,
	Aircraft_Object_C* target_air_in)
{
	target_air = target_air_in;

	Matrix4d target_state = target_air->craft_state;

	Vector3d TargetMissile;
	//导弹与目标距离（米）
	distance_target = (target_state.row(0) - target_state.row(0)).norm();

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
		battle_header.reference_longitude, battle_header.reference_latitude, battle_header.reference_altitude);

	quaternion_bn_to_euler(&coordinate_roll, &coordinate_pitch, &coordinate_yaw, missile_state.row(2));

	return 0;
}

int Missile_Object_C::HitCheck()
{
	static Matrix4d missile_position_last = missile_state;
	static double missile_journey = 0;

	if (distance_target <= destroy_range) {

		target_air->base_live = 0;

		return -1;
	}

	double d_distance = (missile_state.row(0) - missile_position_last.row(0)).norm();
	missile_journey += d_distance;

	if (missile_journey >= max_journey) {
		return -2;
	}


	missile_position_last = missile_state;

	return 1;
}



int Battlefield_C::MissileFire(
	Aircraft_Object_C& attack_air,
	Aircraft_Object_C& target_air)
{
	missile_count++;

	missile_list[missile_count - 1].Init(missile_count, "PL-10", attack_air.base_team,
		attack_air.coordinate_longitude, attack_air.coordinate_latitude, attack_air.coordinate_altitude,
		attack_air.coordinate_roll, attack_air.coordinate_pitch, attack_air.coordinate_yaw,
		attack_air.velocity_north, attack_air.velocity_east, attack_air.velocity_downward);

	missile_list[missile_count - 1].father_id = attack_air.Sim_id;
	missile_list[missile_count - 1].target_air = &target_air;

	return 0;
}
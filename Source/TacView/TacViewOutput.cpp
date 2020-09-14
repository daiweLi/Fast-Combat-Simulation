// --------------------------------------------------------------------------------------------------------------------------------
/**
*   @file           TacViewStep.cpp
*   @brief          TacView 实体及事件更新。
*   @details        TacView 实体及事件更新，详见 ACMI 格式说明。
*   @author         lidaiwei
*   @date           20200808
*   @version        1.0.0.1
*   @par Copyright
*                   GaoYang
*   @par History
*                   1.0.0.1: lidaiwei, 20200808, 首次创建
*/
#include "TacViewOutput.h"
#include <time.h>

using namespace TacView;
using namespace std;




TacViewOutput::TacViewOutput() 
{
	server = new TacViewServer_T();
	state = new State_T;

	state->object_count = 0;
	state->event_count = 0;
}

TacViewOutput::~TacViewOutput() 
{
	delete server;
	delete state;
}

// --------------------------------------------------------------------------------------------------------------------------------
/**
*   @brief          配置仿真环境并启动服务器
*   @details        战场信息 需要手动填写
*   @retval         0               正常
*   @retval         1               错误
*   @retval         2               错误 已经打开
*   @retval         3               错误 WSAStartup 失败
*   @retval         4               错误 Socket 初始化失败
*   @retval         5               错误 Socket 绑定失败
*   @retval         6               错误 Socket 监听失败
*/
int TacViewOutput::InitServer(){
	//战场信息 
	Header_T* header;

    
	// 头数据-仿真环境设置
	header = new Header_T;
	memset(header, 0, sizeof((*header)));

    SYSTEMTIME stUTC;
    ::GetSystemTime(&stUTC);

    strcpy_s((*header).data_source, max_str, "CASIA SIM Service");
	strcpy_s((*header).data_recorder, max_str, "TacViewHelper");
    sprintf_s((*header).reference_time, max_str, "%d-%02d-%02dT%02d:%02d:%02dZ",stUTC.wYear, stUTC.wMonth, stUTC.wDay,stUTC.wHour, stUTC.wMinute, stUTC.wSecond);
    sprintf_s((*header).recording_time, max_str, "%d-%02d-%02dT%02d:%02d:%02dZ",stUTC.wYear, stUTC.wMonth, stUTC.wDay,stUTC.wHour, stUTC.wMinute, stUTC.wSecond);
	strcpy_s((*header).author, max_str, "Author");
	strcpy_s((*header).title, max_str, "Aircraft Test");
	strcpy_s((*header).category, max_str, "Intercept Mission");
	strcpy_s((*header).briefing, max_str, "Destroy all");
	strcpy_s((*header).debriefing, max_str, "Fire");
	strcpy_s((*header).comments, max_str, "Go Go Go");
	(*header).reference_longitude = 126.0;
	(*header).reference_latitude = 30.0;
	int fp = (*server).Open(42674, (*header));

	delete header;

    return fp;
}

// --------------------------------------------------------------------------------------------------------------------------------
/**
*   @brief          配置初始实体状态参数
*   @details        飞机信息及初始参数需要手动填写
*   @param[in]      object_id                   实体编号
*   @param[in]      simulation_id               仿真唯一ID
*   @param[in]      base_name                   对象名字，例：F-16
*   @param[in]      base_type 					类型，例：Air+FixedWing
*   @param[in]      base_pilot 					飞行员
*   @param[in]      base_color 					颜色，例：Red Orange Green Blue Violet
*   @param[in]      base_length	                长，单位：米
*   @param[in]      base_height	                宽，单位：米
*   @param[in]      base_color	                高，单位：米
*   @param[in]      lon                         坐标 经度 单位 度
*   @param[in]      lat                         坐标 纬度 单位 度
*   @param[in]      alt                         坐标 高度 单位 米
*   @param[in]      roll                        姿态 横滚角 单位 度
*   @param[in]      pitch                       姿态 俯仰角 单位 度
*   @param[in]      yaw                         姿态 偏航角 单位 度
*   @retval         0               正常
*   @retval         1               错误
*/
int TacViewOutput::InitOneObject(
	int object_id,
	int simulation_id,
	std::string base_name,
	std::string base_type,
	std::string base_pilot,
	std::string base_color,
	double base_length,
	double base_width,
	double base_height,
	double lon,
	double lat,
	double alt,
	double roll,
	double pitch,
	double yaw)
{
    // 第一帧-初始条件设置

	(*state).object[object_id].id = simulation_id;

	(*state).object[object_id].base_valid = 1;
	strcpy_s((*state).object[object_id].base_name, max_str, base_name.c_str());
	strcpy_s((*state).object[object_id].base_type, max_str, base_type.c_str());//
	strcpy_s((*state).object[object_id].base_pilot, max_str, base_pilot.c_str());
	//strcpy_s((*state).object[object_id].base_country, max_str, "CN");
	//strcpy_s((*state).object[object_id].base_coalition, max_str, "Allies");
	strcpy_s((*state).object[object_id].base_color, max_str, base_color.c_str());
	strcpy_s((*state).object[object_id].base_label, max_str, "mod 1.0");
	(*state).object[object_id].base_length = base_length;
	(*state).object[object_id].base_width = base_width;
	(*state).object[object_id].base_height = base_height;

	(*state).object[object_id].coordinate_valid = 1;
	(*state).object[object_id].coordinate_type = 2;
	(*state).object[object_id].coordinate_longitude = lon;
	(*state).object[object_id].coordinate_latitude = lat;
	(*state).object[object_id].coordinate_altitude = alt;
	(*state).object[object_id].coordinate_roll = roll;
	(*state).object[object_id].coordinate_pitch = pitch;
	(*state).object[object_id].coordinate_yaw = yaw;

	(*state).object[object_id].live = 1;

	(*state).object_count++;

	//(*server).Send(*state);
	return 0;
}

// --------------------------------------------------------------------------------------------------------------------------------
/**
*   @brief          发送一帧的数据
*   @details        发送一帧的数据
*   @param[in]      frame_time            仿真时间戳
*   @param[in]      object_count          一帧中的实体数
*   @param[in]      event_count           一帧中的事件数
*   @retval         0               正常
*   @retval         1               错误
*/
int TacViewOutput::SendOneFrame(
	double frame_time) 
{
	(*state).time = frame_time;

	(*server).Send(*state);

	//memset(state, 0, sizeof((*state)));
	(*state).event_count = 0;
	(*state).object_count = 0;

	return 0;
}

// --------------------------------------------------------------------------------------------------------------------------------
/**
*   @brief          填写一帧中一架飞机的信息
*   @details        填写一帧中一架飞机的信息
*   @param[in]      object_id                   实体编号
*   @param[in]      flight_id                   仿真唯一ID
*   @param[in]      flight_live                飞机存活  1-存活  2-死亡 
*   @param[in]      lon                         坐标 经度 单位 度
*   @param[in]      lat                         坐标 纬度 单位 度
*   @param[in]      alt                         坐标 高度 单位 米
*   @param[in]      roll                        姿态 横滚角 单位 度
*   @param[in]      pitch                       姿态 俯仰角 单位 度
*   @param[in]      yaw                         姿态 偏航角 单位 度
*   @param[in]      radar_opened                雷达开关机状态
*   @param[in]      flight_locked_target_id     雷达锁定目标的仿真唯一ID（负数表示没有锁定任何目标）
*   @param[in]      flight_radar_azimuth        雷达波束方向 方位角 单位 度
*   @param[in]      flight_radar_elevation      雷达波束方向 俯仰角 单位 度
*   @retval         0               正常
*   @retval         1               错误
*/
int TacViewOutput::OneFrameFlightState(
    int object_id,
    int flight_id,
    int flight_live,
    double lon,
    double lat,
    double alt,
    double roll,
    double pitch,
    double yaw,
	bool radar_opened,
	int flight_locked_target_id,
	double flight_radar_azimuth,
	double flight_radar_elevation)
{
		(*state).object_count++;

		static bool flight_live_last[max_object] = { 0 };//记录飞机上一时刻存活状态

        (*state).object[object_id].id = flight_id;
		if(flight_live == 1){
			(*state).object[object_id].live = 1; 
		}else{
			(*state).object[object_id].live = 0; 
			if (flight_live_last[object_id]) {//被摧毁的瞬间上报事件
				(*state).event_count++;
				strcpy_s((*state).event[(*state).event_count - 1].type, max_str, "Destroyed");
				(*state).event[(*state).event_count - 1].id_list[(*state).event_count - 1] = (*state).object[object_id].id;
				strcpy_s((*state).event[(*state).event_count - 1].text, max_str, "Boom!!");
			}
		}

		flight_live_last[object_id] = flight_live;
        
		(*state).object[object_id].coordinate_valid = 1;
		(*state).object[object_id].coordinate_type = 2;
		(*state).object[object_id].coordinate_longitude = lon;
		(*state).object[object_id].coordinate_latitude  = lat;
		(*state).object[object_id].coordinate_altitude  = alt;
		(*state).object[object_id].coordinate_roll      = roll;
		(*state).object[object_id].coordinate_pitch     = pitch;
		(*state).object[object_id].coordinate_yaw       = yaw;

		if(radar_opened == 1){
			if(flight_locked_target_id >= 0){
				(*state).object[object_id].locked_target_valid = 1;
				(*state).object[object_id].locked_target_mode  = 1;
				(*state).object[object_id].locked_target_id    = flight_locked_target_id;

				(*state).object[object_id].radar_valid = 1;
				(*state).object[object_id].radar_mode  = 0;
				(*state).object[object_id].radar_range = 0;
			}else{
				(*state).object[object_id].radar_valid = 1;
				(*state).object[object_id].radar_mode  = 1;
				(*state).object[object_id].radar_horizontal_beamwidth = RADAR_BEAMWIDTH;
				(*state).object[object_id].radar_vertical_beamwidth   = RADAR_BEAMWIDTH;
				(*state).object[object_id].radar_range = RADAR_RANGE;

				(*state).object[object_id].radar_azimuth = flight_radar_azimuth;
				(*state).object[object_id].radar_elevation = flight_radar_elevation;

				(*state).object[object_id].locked_target_valid = 1;
				(*state).object[object_id].locked_target_mode = 0;
			}


		}
		else{
			(*state).object[object_id].radar_valid = 1;
			(*state).object[object_id].radar_mode  = 0;
			(*state).object[object_id].radar_range = 0;
			(*state).object[object_id].locked_target_valid = 1;
			(*state).object[object_id].locked_target_mode  = 0;
		}

		

	return 0;
}


// --------------------------------------------------------------------------------------------------------------------------------
/**
*   @brief          填写一帧中一个导弹的信息
*   @details        填写一帧中一个导弹的信息
*   @param[in]      object_id                   实体编号
*   @param[in]      missile_id                  仿真唯一ID
*   @param[in]      missile_color               导弹颜色 （“Red”，“Blue”，“Orange”）
*   @param[in]      missile_live               导弹存活  1-存活  2-死亡
*   @param[in]      lon                         坐标 经度 单位 度
*   @param[in]      lat                         坐标 纬度 单位 度
*   @param[in]      alt                         坐标 高度 单位 米
*   @param[in]      roll                        姿态 横滚角 单位 度
*   @param[in]      pitch                       姿态 俯仰角 单位 度
*   @param[in]      yaw                         姿态 偏航角 单位 度
*   @param[in]      missile_target_id           末制导锁定目标的仿真唯一ID（负数表示没有锁定任何目标）
*   @retval         0               正常
*   @retval         1               错误
*/
int TacViewOutput::OneFrameMissileState(
    int object_id,
	int missile_id,
	std::string missile_color,
	int missile_live,
    double lon,
    double lat,
    double alt,
    double roll,
    double pitch,
    double yaw,
	int missile_target_id)
{
	(*state).object_count++;

	(*state).object[object_id].id = missile_id;
	if (std::count(List_missile_id.begin(), List_missile_id.end(), missile_id) == 0) {
		List_missile_id.push_back(missile_id);

		(*state).object[object_id].base_valid = 1;
		strcpy_s((*state).object[object_id].base_name, max_str, "PL-10");
		strcpy_s((*state).object[object_id].base_type, max_str, "Air+Missile");
		strcpy_s((*state).object[object_id].base_pilot, max_str, "");
		//strcpy_s((*state).object[object_id].base_country, max_str, "CN");
		//strcpy_s((*state).object[object_id].base_coalition, max_str, "Allies");
		strcpy_s((*state).object[object_id].base_color, max_str, missile_color.c_str());
		strcpy_s((*state).object[object_id].base_label, max_str, "mod 1.0");
		(*state).object[object_id].base_length = 5;
		(*state).object[object_id].base_width = 2;
		(*state).object[object_id].base_height = 2;
	}
	if(missile_live == 1){
		(*state).object[object_id].live = 1; 

		(*state).object[object_id].coordinate_valid = 1;
		(*state).object[object_id].coordinate_type = 2;
		(*state).object[object_id].coordinate_longitude = lon;
		(*state).object[object_id].coordinate_latitude  = lat;
		(*state).object[object_id].coordinate_altitude  = alt;
		(*state).object[object_id].coordinate_roll      = roll;
		(*state).object[object_id].coordinate_pitch     = pitch;
		(*state).object[object_id].coordinate_yaw       = yaw;

		if(missile_target_id >= 0){
			(*state).object[object_id].locked_target_valid = 1;
			(*state).object[object_id].locked_target_mode  = 1;
			(*state).object[object_id].locked_target_id    = missile_target_id;
		}
		else{
			(*state).object[object_id].locked_target_valid = 1;
			(*state).object[object_id].locked_target_mode  = 0;			
		}			
	}
	else{
		(*state).object[object_id].live = 0; 
	}
        


	return 0;
}
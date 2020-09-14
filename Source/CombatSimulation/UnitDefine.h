// --------------------------------------------------------------------------------------------------------------------------------
/**
*   @file           UnitDefine.h
*   @brief          部队单位定义。
*   @details        部队单位属性及参数定义。
*   @author         lidaiwei
*   @date           20200908
*   @version        1.0.0.1
*   @par Copyright
*                   GaoYang
*   @par History
*                   1.0.0.1: lidaiwei, 20200908, 首次创建
*                   
*/

#ifndef Unit_Define_H
#define Unit_Define_H


// --------------------------------------------------------------------------------------------------------------------------------
/**
*   @name           头文件。
*   @{
*/
#include "../FlyTac/aircraft.h"
#include "../FlyTac/missile.h"
#include "../Tools/JoySticks.h"

/** @}  */


// --------------------------------------------------------------------------------------------------------------------------------
/**
*   @brief          CombatSimulation 命名空间。
*   @details        CombatSimulation 命名空间。
*/
namespace CombatSimulation
{
	// --------------------------------------------------------------------------------------------------------------------------------
	/**
	*   @name           数据长度宏定义。
	*   @{
	*/
#define max_str 1024  //最大字符串长度
#define max_object 16 //最大仿真实体数

#define CS_OK 0
#define CS_LIVE 1
#define CS_NOT_LIVE -1
#define CS_MISS -2
	/** @}  */

	// --------------------------------------------------------------------------------------------------------------------------------
	/**
	*   @brief          战场基本信息。
	*   @details        战场基本信息。
	*/
	struct BattlefieldHeader_T
	{
		char  							reference_time[max_str];		//!< 当前仿真的基准时间(UTC)。这个时间与每个帧偏移量(以秒为单位)结合在一起，得到每个数据样本的最终绝对UTC时间。
		char  							author[max_str];				//!< 创建此仿真的作者或操作员。
		char  							title[max_str];					//!< 任务名称。
		char  							category[max_str];				//!< 任务类别。
		char  							briefing[max_str];				//!< 任务简报。
		double							reference_longitude;			//!< 参考点经度，单位：deg
		double							reference_latitude;				//!< 参考点纬度，单位：deg
		double							reference_altitude;				//!< 参考点高度，单位：米
	};



	// --------------------------------------------------------------------------------------------------------------------------------
	/**
	*   @brief          作战单位基类。
	*   @details        作战单位基类。
	*/
	class Unit_Object_C
	{
	public:
		Unit_Object_C() {
			base_live = 0;
		}
		~Unit_Object_C() {}

		BattlefieldHeader_T*			p_battle_header;					//!< 战场信息

		int  							Sim_id;							//!< 编号，唯一标识
		int  							base_live;						//!< 存活，0 死亡，1 存活

		char  							base_name[max_str];				//!< 基本信息，必填，对象名字，例：F-16
		char  							base_type[max_str];				//!< 基本信息，必填，类型，例：Air+FixedWing
		int   							base_team;		                //!< 基本信息，必填，所属队伍

		double							coordinate_longitude;			//!< 坐标，经度，单位：度
		double							coordinate_latitude;			//!< 坐标，纬度，单位：度
		double							coordinate_altitude;			//!< 坐标，高度，单位：米
		double							coordinate_roll;				//!< 坐标，滚转角，单位：度
		double							coordinate_pitch;				//!< 坐标，俯仰角，单位：度
		double							coordinate_yaw;					//!< 坐标，偏航角，单位：度，原始点：正北，正方向：顺时针

		double							velocity_north;					//!< 速度，北向，单位：米/秒
		double							velocity_east;					//!< 速度，东向，单位：米/秒
		double							velocity_downward;				//!< 速度，地面方向，单位：米/秒

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
		int SetCoordinate(
			double							in_coordinate_longitude,
			double							in_coordinate_latitude,
			double							in_coordinate_altitude,
			double							in_coordinate_roll,
			double							in_coordinate_pitch,
			double							in_coordinate_yaw);

		// --------------------------------------------------------------------------------------------------------------------------------
		/**
		*   @brief          设置单位速度
		*   @details        设置单位速度
		*   @param[in]      in_velocity_north             速度，北向，单位：米/秒
		*   @param[in]      in_velocity_east              速度，东向，单位：米/秒
		*   @param[in]      in_velocity_downward          速度，地面方向，单位：米/秒
		*   @retval         0                    正常
		*/
		int SetVelocity(
			double							in_velocity_north,
			double							in_velocity_east,
			double							in_velocity_downward);
	};

	class Aircraft_Object_C:public Unit_Object_C
	{
	public:
		Aircraft_Object_C() {}
		~Aircraft_Object_C() {}

		int								radar_state;					        //!< 雷达状态 0-关机 1-搜索 2-锁定
		int								locked_id_count;						//!< 锁定目标数量
		int								locked_id_list[max_object];				//!< 锁定目标列表

		int								mounted_missile_count;					//!< 挂载的导弹数量

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
		virtual int Init(
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
			double in_velocity_downward);


		// --------------------------------------------------------------------------------------------------------------------------------
		/**
		*   @brief          飞机实体单步解算
		*   @details        飞机实体单步解算
		*   @param[in]      d_time          单步时间间隔 单位：秒
		*   @retval         0               正常
		*   @retval         -1              飞机已死亡
		*/
		virtual int Run(double d_time);

		//int MissileFire(int target_id);

		//***********FlyTac**************//
		Eigen::Matrix4d					craft_state;							//!< 飞机状态
		Eigen::Vector4d					craft_handle;							//!< 飞机控制参数

		
	};

	class Missile_Object_C :public Unit_Object_C
	{
	public:
		Missile_Object_C() {
			missile_live = 0;
		}
		~Missile_Object_C() {}
		int								father_id;								//!< 发射机id
		int								missile_live;					        //!< 导弹状态 1-运行中  0-未发射/死亡 -1-命中 -2-超出范围未命中
		int								radar_state;					        //!< 末制导雷达状态 0-雷达未捕获目标  1-末制导雷达捕获目标
		int								lead_state;								//!< 引导状态 0-未被引导 1-引导中
		Aircraft_Object_C*				p_target_air;								//!< 目标飞机

		double							destroy_range = 250;					//!< 杀伤半径，单位：米
		double							max_journey = 30000;					//!< 最大射程，单位：米

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
		virtual int Init(
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
			double in_velocity_downward);

		// --------------------------------------------------------------------------------------------------------------------------------
		/**
		*   @brief          导弹实体单步解算
		*   @details        导弹实体单步解算
		*   @param[in]      d_time          单步时间间隔 单位：秒
		*   @retval         0               正常
		*   @retval         -1              导弹已死亡
		*/
		virtual int Run(double d_time);

		// --------------------------------------------------------------------------------------------------------------------------------
		/**
		*   @brief          导弹命中判定
		*   @details        导弹命中判定
		*   @retval         0               运行中，未命中
		*   @retval         -1              已命中
		*   @retval         -2              超出射程，未命中
		*/
		int HitCheck();

		//***********FlyTac**************//
		Eigen::Matrix4d					missile_state;							//!< 导弹状态
		Eigen::Vector4d					missile_handle;							//!< 导弹控制参数
	

	private:
		//导弹与目标距离（米）
		double distance_target;

		//***********FlyTac**************//
		//导弹过点飞控制参数
		double missile_errA = 0, missile_errP = 0, missile_errR = 0,
			missile_errAsum = 0, missile_errPsum = 0;
	};


	// --------------------------------------------------------------------------------------------------------------------------------
/**
*   @brief          仿真战场内容。
*   @details        仿真战场内容。
*/
	class Battlefield_C
	{

	public:
		Battlefield_C() {
			time = 0;
			aircraft_count = 0;
			missile_count = 0;

			for (int i = 0; i < max_object; i++) {
				aircraft_list[i].p_battle_header = &battle_header;
				missile_list[i].p_battle_header = &battle_header;
			}
		}

		~Battlefield_C() {}

		double							time;							//!< 时标
		BattlefieldHeader_T				battle_header;					//!< 战场信息

		int								aircraft_count;					//!< 飞行器 数量
		Aircraft_Object_C				aircraft_list[max_object];		//!< 飞行器 列表
		int								missile_count;					//!< 导弹 数量
		Missile_Object_C				missile_list[max_object];		//!< 导弹 列表

// --------------------------------------------------------------------------------------------------------------------------------
/**
*   @brief          设置参考点坐标
*   @details        设置参考点坐标
*   @param[in]      in_reference_longitude             参考点经度，单位：deg
*   @param[in]      in_reference_latitude              参考点纬度，单位：deg
*   @param[in]      in_reference_altitude              参考点高度，单位：米
*   @retval         0                    正常
*/
		int InitCoordinate(
			double							in_reference_longitude,
			double							in_reference_latitude,
			double							in_reference_altitude);


		// --------------------------------------------------------------------------------------------------------------------------------
		/**
		*   @brief          飞机发射导弹
		*   @details        飞机发射导弹
		*   @param[in]      attack_air             发射机
		*   @param[in]      target_air             目标机
		*   @retval         0                    正常
		*/
		int MissileFire(
			Aircraft_Object_C& attack_air,
			Aircraft_Object_C& target_air);
	};
}



#endif // Unit_Define_H

// --------------------------------------------------------------------------------------------------------------------------------
/**
*   @file           coordinate.h
*   @brief          飞行器用到的各种坐标系变换。
*   @details        现有：地球坐标系，导航坐标系，机体坐标系。
                    地球坐标系采用WGS-84坐标系各项参数。
					地球坐标系采用WGS-84坐标系各项参数。
					地球空间直角坐标系原点为参考椭球的中心，X轴和Y轴位于赤道平面，X轴通过零子午线，Z轴与椭球极轴一致。
					导航坐标系原点位于参考点，X轴指向北边，Y轴指向东边，Z轴指向地下。
					机体坐标系原点位于质心点，X轴指向机头前方，Y轴指向右侧，Z轴指向下方。
					所有坐标系均为右手系。
*   @author         LiDaiwei
*   @date           20191203
*   @version        1.0.0.1
*   @par Copyright
*                   LiDaiwei
*   @par History
*                   1.0.0.1: LiDaiwei, 20191203, 首次创建
*

*/

#ifndef COORDINATE_H_INCLUDED
#define COORDINATE_H_INCLUDED

// --------------------------------------------------------------------------------------------------------------------------------
/**
*   @name           被使用的头文件。
*   @{
*/
#include <math.h>
#include "tool_function.h"
#include "../Tools/eigen337/Eigen/Dense"
/** @}  */

// --------------------------------------------------------------------------------------------------------------------------------
/**
*   @name           基本常数定义。
*   @{
*/
#define earth_radius 6378137.0             	     //!< 地球长半径(米)。
#define eatth_b 6356752.3142 				//!< 地球短半径(米)。
#define earth_ellipticity 0.003352810664             //!< 地球扁率(椭圆度)。
#define earth_e1 sqrt(earth_radius*earth_radius-eatth_b*eatth_b)/earth_radius       //!< 地球椭球第一偏心率。

#define earth_angular_velocity 7292115e-11           //!< 地球平均自转角速度(rad/s)。
#define simple_gravity 	9.8015             	//!< 北京地区的重力，在不需要精确的重力加速度时使用。

/** @}  */
// --------------------------------------------------------------------------------------------------------------------------------
/**
*   @brief          计算重力加速度。
*   @details        计算重力加速度。
*   @param[out]     gravity         精确重力
*   @param[in]      latitude        纬度(角度)
*   @retval         0               正常
*   @retval         1               错误
*/
int accuraty_gravity(
	double* gravity,
	const double latitude);


// --------------------------------------------------------------------------------------------------------------------------------
/**
*   @brief          计算地球主曲率半径。
*   @details        参考椭球子午圈上各点曲率半径RM和卯酉圈（它所在的平面与子午面垂直）上各点的曲率半径RN称为主曲率半径。
*   @param[out]     RM              子午圈曲率半径
*   @param[out]     RN              卯酉圈曲率半径
*   @param[in]      latitude        纬度(角度)
*   @retval         0               正常
*   @retval         1               错误
*/
int earth_curvature_radius(
	double* RM,
	double* RN,
	const double latitude);


// --------------------------------------------------------------------------------------------------------------------------------
/**
*   @brief          地球空间直角坐标系转换到经纬度高度坐标系。
*   @details        地球空间直角坐标系原点为参考椭球的中心，X轴和Y轴位于赤道平面，X轴通过零子午线，Z轴与椭球极轴一致。
*   @param[out]     longitude       经度（角度）
*   @param[out]     latitude        纬度（角度）
*   @param[out]     height          高度
*   @param[in]      x               地球空间直角坐标系X坐标
*   @param[in]      y               地球空间直角坐标系Y坐标
*   @param[in]      z               地球空间直角坐标系Z坐标
*   @retval         0               正常
*   @retval         1               错误
*/
int xyz_to_llh(
	double* longitude,
	double* latitude,
	double* height,
	const double x,
	const double y,
	const double z);

// --------------------------------------------------------------------------------------------------------------------------------
/**
*   @brief          经纬度高度坐标系转换到地球空间直角坐标系。
*   @details        地球空间直角坐标系原点为参考椭球的中心，X轴和Y轴位于赤道平面，X轴通过零子午线，Z轴与椭球极轴一致。
*   @param[out]      x               地球空间直角坐标系X坐标
*   @param[out]      y               地球空间直角坐标系Y坐标
*   @param[out]      z               地球空间直角坐标系Z坐标
*   @param[in]       longitude       经度（角度）
*   @param[in]       latitude        纬度（角度）
*   @param[in]       height          高度
*   @retval          0               正常
*   @retval          1               错误
*/
int llh_to_xyz(
	double* x,
	double* y,
	double* z,
	const double longitude,
	const double latitude,
	const double height);

// --------------------------------------------------------------------------------------------------------------------------------
// --------------------------------------------------------------------------------------------------------------------------------
/**
*   @brief          地球经纬度坐标系转换到导航坐标系的旋转矩阵。
*   @details        导航坐标系原点位于参考点，X轴指向北边，Y轴指向东边，Z轴指向地下。
*   @param[out]     R_en            旋转矩阵:3x3,正交
*   @param[in]      longitude0      导航坐标系参考点经度(角度)
*   @param[in]      latitude0       导航坐标系参考点纬度(角度)
*   @retval         0               正常
*   @retval         1               错误
*/
int rotation_earth_to_navigation(
	Eigen::Matrix3d* R_en,
	const double longitude0,
	const double latitude0);

// --------------------------------------------------------------------------------------------------------------------------------
/**
*   @brief          导航坐标系转换到地球经纬度坐标系的旋转矩阵。
*   @details        导航坐标系原点位于参考点，X轴指向北边，Y轴指向东边，Z轴指向地下。
*   @param[out]     R_ne            旋转矩阵:3x3,正交
*   @param[in]      longitude0      导航坐标系参考点经度(角度)
*   @param[in]      latitude0       导航坐标系参考点纬度(角度)
*   @retval         0               正常
*   @retval         1               错误
*/
int rotation_navigation_to_earth(
	Eigen::Matrix3d* R_ne,
	const double longitude0,
	const double latitude0);

// --------------------------------------------------------------------------------------------------------------------------------
/**
*   @brief          地球经纬度坐标系转换到导航坐标系。
*   @details        导航坐标系原点位于参考点，X轴指向北边，Y轴指向东边，Z轴指向地面。
*   @param[out]     north            导航坐标系内东向坐标
*   @param[out]     east           导航坐标系内北向坐标
*   @param[out]     downward          导航坐标系内天向坐标
*   @param[in]      longitude       机体经度（角度）
*   @param[in]      latitude        机体纬度（角度）
*   @param[in]      height          机体高度
*   @param[in]      longitude0      导航坐标系参考点经度（角度）
*   @param[in]      latitude0       导航坐标系参考点纬度（角度）
*   @param[in]      height0         导航坐标系参考点高度
*   @retval         0               正常
*   @retval         1               错误
*/
int earth_to_navigation(
	double* north,
	double* east,
	double* downward,
	const double longitude,
	const double latitude,
	const double height,
	const double longitude0,
	const double latitude0,
	const double height0);

// --------------------------------------------------------------------------------------------------------------------------------
/**
*   @brief          导航坐标系转换到地球经纬度坐标系。
*   @details        导航坐标系原点位于参考点，X轴指向北边，Y轴指向东边，Z轴指向地面。
*   @param[out]     longitude       机体经度（角度）
*   @param[out]     latitude        机体纬度（角度）
*   @param[out]     height          机体高度
*   @param[in]      north            机体导航坐标系内东向坐标
*   @param[in]      east           机体导航坐标系内北向坐标
*   @param[in]      downward          机体导航坐标系内天向坐标
*   @param[in]      longitude0      导航坐标系参考点经度（角度）
*   @param[in]      latitude0       导航坐标系参考点纬度（角度）
*   @param[in]      height0         导航坐标系参考点高度
*   @retval         0               正常
*   @retval         1               错误
*/
int navigation_to_earth(
	double* longitude,
	double* latitude,
	double* height,
	const double north,
	const double east,
	const double downward,
	const double longitude0,
	const double latitude0,
	const double height0);

// --------------------------------------------------------------------------------------------------------------------------------
/**
*   @brief          导航坐标系转换到机体坐标系的旋转矩阵。
*   @details        导航坐标系原点位于参考点，X轴指向北边，Y轴指向东边，Z轴指向地下。
					机体坐标系原点位于质心点，X轴指向机头，Y轴指向右侧，Z轴指向下方。
*   @param[out]     R_nb            旋转矩阵:3x3,正交
*   @param[in]      roll            机体导航坐标系内横滚角（角度）
*   @param[in]      pitch           机体导航坐标系内俯仰角（角度）
*   @param[in]      yaw             机体导航坐标系内航向角（角度）
*   @retval         0               正常
*   @retval         1               错误
*/
int rotation_navigation_to_body(
	Eigen::Matrix3d* R_nb,
	const double roll,
	const double pitch,
	const double yaw);

// --------------------------------------------------------------------------------------------------------------------------------
/**
*   @brief          机体坐标系转换到导航坐标系的旋转矩阵。
*   @details        导航坐标系原点位于参考点，X轴指向北边，Y轴指向东边，Z轴指向地下。
					机体坐标系原点位于质心点，X轴指向机头，Y轴指向右侧，Z轴指向下方。
*   @param[out]     R_bn            旋转矩阵:3x3,正交
*   @param[in]      roll            机体导航坐标系内横滚角（角度）
*   @param[in]      pitch           机体导航坐标系内俯仰角（角度）
*   @param[in]      yaw             机体导航坐标系内航向角（角度）
*   @retval         0               正常
*   @retval         1               错误
*/
int rotation_body_to_navigation(
	Eigen::Matrix3d* R_bn,
	const double roll,
	const double pitch,
	const double yaw);


// --------------------------------------------------------------------------------------------------------------------------------
/**
*   @brief          由Rnb旋转矩阵得到欧拉角。
*   @details        Rnb是导航坐标系转换到机体坐标系
					导航坐标系原点位于参考点，X轴指向北边，Y轴指向东边，Z轴指向地下。
					机体坐标系原点位于质心点，X轴指向机头，Y轴指向右侧，Z轴指向下方。
*   @param[out]     roll            机体导航坐标系内横滚角（角度）
*   @param[out]     pitch           机体导航坐标系内俯仰角（角度）
*   @param[out]     yaw             机体导航坐标系内航向角（角度）
*   @param[in]      R_nb            旋转矩阵:3x3,正交
*   @retval         0               正常
*   @retval         1               错误
*/
int rotation_nb_to_euler(
	double* roll,
	double* pitch,
	double* yaw,
	const Eigen::Matrix3d R_nb);

// --------------------------------------------------------------------------------------------------------------------------------
/**
*   @brief          由方向余弦旋转矩阵求四元数
*   @details        由方向余弦旋转矩阵求四元数
*   @param[out]     q               四元数:4维向量
*   @param[in]      R               方向余弦旋转矩阵:3x3
*   @retval         0               正常
*   @retval         1               错误
*/
int rotation_to_quaternion(
	Eigen::Vector4d* q,
	const Eigen::Matrix3d R);

// --------------------------------------------------------------------------------------------------------------------------------
/**
*   @brief          由四元数求方向余弦旋转矩阵
*   @details        由四元数求方向余弦旋转矩阵
*   @param[out]     R               方向余弦旋转矩阵:3x3
*   @param[in]      q               四元数:4维向量
*   @retval         0               正常
*   @retval         1               错误
*/
int quaternion_to_rotation(
	Eigen::Matrix3d* R,
	const Eigen::Vector4d & q);

// --------------------------------------------------------------------------------------------------------------------------------
/**
*   @brief          由欧拉角求从载体系到导航系的转动四元数
*   @details        由欧拉角求从载体系到导航系的转动四元数
*   @param[out]     q               从载体系到导航系的转动四元数:4维向量
*   @param[in]      roll            机体导航坐标系内横滚角（角度）
*   @param[in]      pitch           机体导航坐标系内俯仰角（角度）
*   @param[in]      yaw             机体导航坐标系内航向角（角度）
*   @retval         0               正常
*   @retval         1               错误
*/
int euler_to_quaternion_bn(
	Eigen::Vector4d* q,
	const double roll,
	const double pitch,
	const double yaw);


// --------------------------------------------------------------------------------------------------------------------------------
/**
*   @brief          由从载体系到导航系的转动四元数求欧拉角
*   @details        由从载体系到导航系的转动四元数求欧拉角
*   @param[out]     roll            机体导航坐标系内横滚角（角度）
*   @param[out]     pitch           机体导航坐标系内俯仰角（角度）
*   @param[out]     yaw             机体导航坐标系内航向角（角度）
*   @param[in]      q               四元数:4维向量
*   @retval         0               正常
*   @retval         1               错误
*/
int quaternion_bn_to_euler(
	double* roll,
	double* pitch,
	double* yaw,
	const Eigen::Vector4d& q);//不加'&'会报错：具有 __declspec(align('16')) 的形参将不被对齐
// --------------------------------------------------------------------------------------------------------------------------------
/**
*   @brief          四元数归一化
*   @details        四元数归一化
*   @param[out]     q1              归一化后的四元数
*   @param[in]      q0              四元数:4维向量
*   @retval         0               正常
*   @retval         1               错误
*/

int quaterntion_normalized(
	Eigen::Vector4d* q1,
	const  Eigen::Vector4d& q0);

#endif // COORDINATE_H_INCLUDED

// --------------------------------------------------------------------------------------------------------------------------------
/**
*   @file           coordinate.cpp
*   @brief          飞行器用到的各种坐标系变换。
*   @details        现有：地球坐标系，导航坐标系，机体坐标系。
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

// --------------------------------------------------------------------------------------------------------------------------------
/**
*   @name           头文件。
*   @{
*/

#include "coordinate.h"
#include <algorithm>
//#include <Eigen/Dense>

#include<iostream>
using namespace std;
using namespace Eigen;
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
int accuraty_gravity (
	double* gravity,
	const double latitude)
{
	double L=0;
	deg2rad(&L,latitude);
	*gravity=978.03267714*(1+0.00193185138639*sin(L)*sin(L))/
		sqrt(1-0.00669437999013*sin(L)*sin(L));
	return 0;
}


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
int earth_curvature_radius (
	double* RM,
	double* RN,
	const double latitude)
{
	double L=0;
	deg2rad(&L,latitude);
	*RM=earth_radius*(1-2*earth_ellipticity+3*earth_ellipticity*sin(L)*sin(L));
	*RN=earth_radius*(1+earth_ellipticity*sin(L)*sin(L));

	return 0;
}


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
	const double z)
{
	double lon=0,lat=0;
	lon=atan2(y,x);
	//lon = lon < 0 ? (M_PI + lon) : lon;
	rad2deg(longitude,lon);
	//迭代法求纬度和高度
	double HpRN_ip1=0,RN_ip1=0;
	double lat_i=0,lat_ip1=0;
	lat_i=atan(z/(pow((1-earth_ellipticity),2)*sqrt(x*x+y*y)));
	int num = 0;
	do{
		HpRN_ip1=x/(cos(lat_i)*cos(lon));
		RN_ip1=earth_radius/sqrt(pow(cos(lat_i),2)+(1-pow(earth_e1,2))*pow(sin(lat_i),2));
		lat_ip1=atan(HpRN_ip1*z/((HpRN_ip1-RN_ip1*pow(earth_e1,2))*sqrt(x*x+y*y)));
		num++;
	}while (abs( lat_ip1-lat_i)>1e-6 && num<1e3);

	lat=lat_ip1;
	rad2deg(latitude,lat);
	*height=HpRN_ip1-RN_ip1;
	return 0;
}

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
	const double height)
{
	double lon=0,lat=0,RN=0,RM;
	deg2rad(&lon,longitude);
	deg2rad(&lat,latitude);
	earth_curvature_radius(&RM,&RN,latitude);

	*x=(height+RN)*cos(lat)*cos(lon);
	*y=(height+RN)*cos(lat)*sin(lon);
	*z=(RN*pow((1-earth_ellipticity),2)+height)*sin(lat);
	return 0;
}

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
	Matrix3d* R_en,
	const double longitude0,
	const double latitude0)
{
	double lon = 0, L = 0;
	deg2rad(&lon, longitude0); deg2rad(&L, latitude0);
	//先转到天东北系
	Matrix3d R_en2;
	R_en2(0, 0) = cos(L) * cos(lon);
	R_en2(0, 1) = cos(L) * sin(lon);
	R_en2(0, 2) = sin(L);
	R_en2(1, 0) = -sin(lon);
	R_en2(1, 1) = cos(lon);
	R_en2(1, 2) = 0;
	R_en2(2, 0) = -sin(L) * cos(lon);
	R_en2(2, 1) = -sin(L) * sin(lon);
	R_en2(2, 2) = cos(L);
	//再从天东北转到北东地
	Matrix3d R_n2n;
	R_n2n << 0, 0, 1,
		0, 1, 0,
		-1, 0, 0;
	*R_en = R_n2n * R_en2;
	return 0;
}

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
	Matrix3d* R_ne,
	const double longitude0,
	const double latitude0)
{
	Matrix3d R_en;
	rotation_earth_to_navigation(&R_en, longitude0, latitude0);
	*R_ne = R_en.transpose();
	return 0;
}

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
	const double height0)
{

	Matrix3d R_en;
	rotation_earth_to_navigation(&R_en, longitude0, latitude0);
	Vector3d Pe, Pn, t;
	llh_to_xyz(&Pe(0), &Pe(1), &Pe(2), longitude, latitude, height);
	llh_to_xyz(&t(0), &t(1), &t(2), longitude0, latitude0, height0);

	Pn = R_en * (Pe - t);

	*north = Pn(0);
	*east = Pn(1);
	*downward = Pn(2);
	return 0;
}

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
	const double height0)
{
	Matrix3d R_ne;
	rotation_navigation_to_earth(&R_ne, longitude0, latitude0);
	Vector3d Pe, Pn, t;
	Pn << north, east, downward;//在导航系内的坐标
	llh_to_xyz(&t(0), &t(1), &t(2), longitude0, latitude0, height0);
	Pe = R_ne * Pn + t;
	xyz_to_llh(longitude, latitude, height, Pe(0), Pe(1), Pe(2));

	return 0;
}

// --------------------------------------------------------------------------------------------------------------------------------
/**
*   @brief          由欧拉角得到导航坐标系转换到机体坐标系的旋转矩阵。
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
	Matrix3d* R_nb,
	const double roll,
	const double pitch,
	const double yaw)
{
	////////先将东-北-天的导航坐标系转到北-东-地的方向
	//////Matrix3d R_nn2;
	//////R_nn2 << 0, 1, 0,
	//////		1, 0, 0,
	//////		0, 0, -1;

	//从北-东-地的导航坐标系以Z（航向）-Y（俯仰）-X（滚转）的顺序旋转到载体系的旋转矩阵：
	double r, p, y;
	deg2rad(&r, roll); deg2rad(&p, pitch); deg2rad(&y, yaw);
	double c1 = cos(y), s1 = sin(y);
	double c2 = cos(p), s2 = sin(p);
	double c3 = cos(r), s3 = sin(r);
	//可查Z1Y2X3顺规旋转矩阵如下：
	Matrix3d R_n2b;
	R_n2b(0, 0) = c1 * c2;
	R_n2b(0, 1) = c1 * s2 * s3 - c3 * s1;
	R_n2b(0, 2) = s1 * s3 + c1 * c3 * s2;
	R_n2b(1, 0) = c2 * s1;
	R_n2b(1, 1) = c1 * c3 + s1 * s2 * s3;
	R_n2b(1, 2) = c3 * s1 * s2 - c1 * s3;
	R_n2b(2, 0) = -s2;
	R_n2b(2, 1) = c2 * s3;
	R_n2b(2, 2) = c2 * c3;

	*R_nb = R_n2b.transpose();// *R_nn2;

	return 0;
}

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
	Matrix3d* R_bn,
	const double roll,
	const double pitch,
	const double yaw)
{
	Matrix3d R_nb;
	rotation_navigation_to_body(&R_nb, roll, pitch, yaw);
	*R_bn = R_nb.transpose();

	return 0;
}

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
	const Matrix3d R_nb)
{
	double r, p, y;
	//////Matrix3d R_nn2_inv;
	//////R_nn2_inv << 0, 1, 0,
	//////		1, 0, 0,
	//////		0, 0, -1;
	Matrix3d R_n2b = R_nb.transpose();// *R_nn2_inv;
	r = atan2(R_n2b(2,1), R_n2b(2,2));
	p = atan(-R_n2b(2, 0) / sqrt(1 - R_n2b(2, 0) * R_n2b(2, 0)));
	y = atan2(R_n2b(1, 0), R_n2b(0, 0));

	rad2deg(roll, r);
	rad2deg(pitch, p);
	rad2deg(yaw, y);

	return 0;
}



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
	Vector4d* q,
	const Matrix3d R)
{
	Vector4d __q;
	__q(0) = 0.5 * sqrt(1 + R(0, 0) + R(1, 1) + R(2, 2));
	if (fabs(__q(0)) < 1e-4) {
		if (R(0, 0) > R(1, 1) && R(0, 0) > R(2, 2)) {
			double t = sqrt(1 + R(0, 0) - R(1, 1) - R(2, 2));
			__q(0) = (R(2, 1) - R(1, 2)) / t;
			__q(1) = t / 4;
			__q(2) = (R(0, 2) + R(2, 0)) / t;
			__q(3) = (R(0, 1) + R(1, 0)) / t;
		}
		else if (R(1, 1) > R(0, 0) && R(1, 1) > R(2, 2)) {
			double t = sqrt(1 - R(0, 0) + R(1, 1) - R(2, 2));
			__q(0) = (R(0, 2) - R(2, 0)) / t;
			__q(1) = (R(0, 1) + R(1, 0)) / t;
			__q(2) =  t / 4;
			__q(3) = (R(2, 1) + R(1, 2)) / t;
		}
		else {
			double t = sqrt(1 - R(0, 0) - R(1, 1) + R(2, 2));
			__q(0) = (R(1, 0) - R(0, 1)) / t;
			__q(1) = (R(0, 2) + R(2, 0)) / t;
			__q(2) = (R(1, 2) - R(2, 1)) / t;
			__q(3) =  t / 4;
		}
	}
	else {
		__q(1) = (R(2, 1) - R(1, 2)) / (4 * __q(0));
		__q(2) = (R(0, 2) - R(2, 0)) / (4 * __q(0));
		__q(3) = (R(1, 0) - R(0, 1)) / (4 * __q(0));
	}
	quaterntion_normalized(q, __q);
	return 0;
}

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
	Matrix3d* R,
	const Vector4d & q)//不加'&'会报错：具有 __declspec(align('16')) 的形参将不被对齐
{
	(*R)(0, 0) = q(0) * q(0) + q(1) * q(1) - q(2) * q(2) - q(3) * q(3);
	(*R)(0, 1) = 2 * (q(1) * q(2) - q(0) * q(3));
	(*R)(0, 2) = 2 * (q(1) * q(3) + q(0) * q(2));
	(*R)(1, 0) = 2 * (q(1) * q(2) + q(0) * q(3));
	(*R)(1, 1) = q(0) * q(0) - q(1) * q(1) + q(2) * q(2) - q(3) * q(3);
	(*R)(1, 2) = 2 * (q(2) * q(3) - q(0) * q(1));
	(*R)(2, 0) = 2 * (q(1) * q(3) - q(0) * q(2));
	(*R)(2, 1) = 2 * (q(2) * q(3) + q(0) * q(1));
	(*R)(2, 2) = q(0) * q(0) - q(1) * q(1) - q(2) * q(2) + q(3) * q(3);

	//Matrix3d R_nn2_inv;
	//R_nn2_inv << 0, 1, 0,
	//	1, 0, 0,
	//	0, 0, -1;
	//(*R) = (*R) * R_nn2_inv;

	return 0;
}



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
	Vector4d* q,
	const double roll,
	const double pitch,
	const double yaw)
{
	Matrix3d Rnb;
	rotation_navigation_to_body(&Rnb, roll, pitch, yaw);
	//cout << R << endl;
	//cin.get();
	Matrix3d Rbn;
	Rbn = Rnb.transpose();
	rotation_to_quaternion(q, Rbn);
	if (fabs(roll) < 1e-4 && fabs(pitch) < 1e-4 && fabs(yaw) < 1e-4) {
		(*q) << 0, 0, 0, 0;
		(*q)(0) = 1;
	}
		
	return 0;
}


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
	const Vector4d& q)//不加'&'会报错：具有 __declspec(align('16')) 的形参将不被对齐
{
	Matrix3d Rbn;
	quaternion_to_rotation(&Rbn, q);

	rotation_nb_to_euler(roll, pitch, yaw, Rbn.transpose());

	return 0;
}


// --------------------------------------------------------------------------------------------------------------------------------
/**
*   @brief          四元数归一化
*   @details        四元数归一化
*   @param[out]     qout            归一化后的四元数
*   @param[in]      qin             四元数:4维向量
*   @retval         0               正常
*   @retval         1               错误
*/

int quaterntion_normalized(
	Vector4d* qout,
	const Vector4d& qin)
{

	double q0, q1, q2, q3;

	q0 = qin(0) / qin.norm();
	q1 = qin(1) / qin.norm();
	q2 = qin(2) / qin.norm();
	q3 = qin(3) / qin.norm();
	(*qout)<< q0, q1, q2, q3;

	return 0;
}
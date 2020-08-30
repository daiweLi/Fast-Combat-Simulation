#ifndef MISSILE_RADAR_H_INCLUDED
#define MISSILE_RADAR_H_INCLUDED
// --------------------------------------------------------------------------------------------------------------------------------
/**
*   @file           missile_radar.h
*   @brief          雷达及全向雷达告警器功能仿真。
*   @details        雷达及全向雷达告警器功能仿真。
*   @author         LiDaiwei
*   @date           20191113
*   @version        1.0.0.1
*   @par Copyright
*                   LiDaiwei
*   @par History
*                   1.0.0.1: LiDaiwei, 20191113, 首次创建

*/

// --------------------------------------------------------------------------------------------------------------------------------
/**
*   @name           被使用的头文件。
*   @{
*/
#include "radar.h"
/** @}  */


// --------------------------------------------------------------------------------------------------------------------------------
/**
*   @brief          设定导弹跟踪雷达参数。
*   @details        设定导弹跟踪雷达参数。
*   @param[out]      Lambda               雷达工作波长
*   @param[out]      Pgt                  雷达发射功率
*   @param[out]      nG                   天线效率
*   @param[out]      Gmax                 天线最大增益
*   @param[out]      theta_t              主瓣波束宽度 //默认值1.0
*   @param[out]      theta_b              单程半功率点波束宽度 //默认值3.14
*   @param[out]      Lr                   雷达综合损耗Lr
*   @param[out]      Br                   雷达工作的瞬时带宽 //默认值5兆
*   @param[out]      Nf                   雷达接收机噪声系数
*   @param[out]      azimuth_radar_width  雷达扫描方位角范围
*   @param[out]      pitch_radar_width    雷达扫描俯仰角范围
*   @retval          0               正常
*   @retval          1               错误
*/
int Missile_Radar_Initial(
	double* lambda,
	double* Pgt,
	double* nG,
	double* Gmax,
	double* theta_t,
	double* theta_b,
	double* Lr,
//	double* Br,
//	double* Nf,
	double* azimuth_radar_width,
	double* pitch_radar_width);

#endif // MISSILE_RADAR_H_INCLUDED

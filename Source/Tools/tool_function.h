// --------------------------------------------------------------------------------------------------------------------------------
/**
*   @file           tool_function.h
*   @brief          计算功能函数。
*   @details        常用的数学计算。
*   @author         LiDaiwei
*   @date           20191112
*   @version        1.0.0.1
*   @par Copyright
*                   LiDaiwei
*   @par History
*                   1.0.0.1: LiDaiwei, 20191112, 首次创建

*/

#ifndef TOOL_FUNCTION_H_INCLUDED
#define TOOL_FUNCTION_H_INCLUDED

// --------------------------------------------------------------------------------------------------------------------------------
/**
*   @name           被使用的头文件。
*   @{
*/
#define _USE_MATH_DEFINES
#include <math.h>
/** @}  */

// --------------------------------------------------------------------------------------------------------------------------------
/**
*   @brief          角度转弧度。
*   @details        角度转弧度。
*   @param[out]     radOut          弧度输出
*   @param[in]      degIn           角度输入
*   @retval         0               正常
*   @retval         1               错误
*/
int deg2rad(
	double* radOut,
	const double degIn);

// --------------------------------------------------------------------------------------------------------------------------------
/**
*   @brief          弧度转角度。
*   @details        弧度转角度。
*   @param[out]     degOut          弧度输出
*   @param[in]      radIn           角度输入
*   @retval         0               正常
*   @retval         1               错误
*/
int rad2deg(
	double* degOut,
	const double radIn);


// --------------------------------------------------------------------------------------------------------------------------------
/**
*   @brief          生成高斯分布随机数序列。
*   @details        生成高斯分布随机数序列:期望为0.0，方差为1.0。
*   @param[out]     gaussOut        随机数输出
*   @retval         0               正常
*   @retval         1               错误
*/
int gaussrand(
	double* gaussOut
	);


#endif // TOOL_FUNCTION_H_INCLUDED
